#!/usr/bin/env python3
import json
import os
import threading
import time
import urllib.error
import urllib.request

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from ur_task_planner.srv import ExecuteSkill

from scene_geometry import TableWorkspace

from plan_validator import (LOCATION_GRIPPER, LOCATION_TABLE, LOCATION_UNKNOWN,
                            SKILL_PARAMS, PlanValidationError, PlanValidator, SceneState)

GEMINI_URL = 'https://generativelanguage.googleapis.com/v1beta/models/{model}:generateContent'
# Aliases kept up to date by Google, dated model names are retired after a while
DEFAULT_MODEL = 'gemini-flash-latest'
DEFAULT_FALLBACK_MODELS = ['gemini-flash-lite-latest']
# The API answers 500/503 when a model is overloaded, this is usually short
LLM_ATTEMPTS = 2
LLM_RETRY_DELAY_SEC = 2.0
# Errors of one model (retired, out of quota, overloaded): another model may still answer
MODEL_ERRORS = (404, 429, 500, 503)

SKILL_DESCRIPTIONS = {
    'home': 'move the arm back to its home pose',
    'observe': 'refresh cube poses and zone occupancy from the camera',
    'place_on_table': 'release the held object at an LLM-chosen temporary table position [x, y] in gazebo_world metres',
    'pick': 'grasp an object and lift it',
    'place': 'put the object held by the gripper into a zone and release it',
    'move_above': 'move the gripper above an object without touching it',
    'move_to_zone': 'move the gripper above a zone',
    'open_gripper': 'open the gripper',
    'close_gripper': 'close the gripper',
}


class ModelUnavailable(RuntimeError):
    """The model cannot answer right now, the request itself is fine."""


class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner_node',
                         automatically_declare_parameters_from_overrides=True)

        model = self.param('model', os.environ.get('GEMINI_MODEL') or DEFAULT_MODEL)
        fallback_models = self.param('fallback_models', DEFAULT_FALLBACK_MODELS)
        # Tried in this order until one of them answers
        self.models = [model] + [m for m in fallback_models if m and m != model]
        self.llm_timeout = float(self.param('llm_timeout_sec', 60.0))
        self.skill_timeout = float(self.param('skill_timeout_sec', 180.0))

        if not (self.has_parameter('object_names') and self.has_parameter('zone_names')):
            raise RuntimeError(
                'object_names / zone_names are not set, load config/scene.yaml')
        object_names = list(self.get_parameter('object_names').value)
        zone_names = list(self.get_parameter('zone_names').value)
        self.objects = {
            name: self.param(f'objects.{name}.description', name) for name in object_names}
        self.zones = {
            name: self.param(f'zones.{name}.description', name) for name in zone_names}

        self.use_perception = bool(self.param('use_perception', False))
        self.workspace = TableWorkspace(
            bounds=self.param('table_placement_bounds', [-0.25, 0.015, -0.34, 0.34]),
            base=self.get_parameter('robot_base_position').value,
            zones={name: {
                'position': list(self.get_parameter(f'zones.{name}.position').value),
                'size': self.param(f'zones.{name}.size', [0.15, 0.15]),
            } for name in zone_names},
            sizes={name: list(self.get_parameter(f'objects.{name}.size').value) for name in object_names},
            gap=float(self.param('placement_gap', 0.015)),
            keepout=float(self.param('robot_keepout_radius', 0.12)),
            reach=float(self.param('placement_max_reach', 0.56)),
            height=float(self.param('table_surface_z', 0.8)),
        )
        self.validator = PlanValidator(object_names, zone_names, workspace=self.workspace)
        self.system_prompt = self.build_system_prompt()

        # Wait for the authoritative executor snapshot before using any object pose
        self.scene_state = SceneState(
            locations={name: LOCATION_UNKNOWN for name in object_names})
        self.state_lock = threading.Lock()
        self.busy = threading.Lock()

        # Service client to talk to skill executor
        self.skill_client = self.create_client(ExecuteSkill, 'execute_skill')

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_subscription = self.create_subscription(
            String, 'scene_state', self.scene_state_callback, latched)
        self.status_publisher = self.create_publisher(String, 'task_status', 10)

        # Subscribe to user commands
        self.subscription = self.create_subscription(
            String,
            'user_command',
            self.command_callback,
            10)

        self.get_logger().info(
            f"LLM Planner Node initialized (models: {', '.join(self.models)}). "
            "Waiting for commands on /user_command")
        if not os.environ.get('GEMINI_API_KEY'):
            self.get_logger().error(
                "GEMINI_API_KEY environment variable not set. Commands will be rejected.")

    def param(self, name, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def build_system_prompt(self):
        skills = []
        for skill in self.validator.allowed_skills:
            params = SKILL_PARAMS[skill]
            takes = ', '.join(f'"{p}"' for p in params) if params else 'none'
            skills.append(f'- {skill}: {SKILL_DESCRIPTIONS[skill]}. Parameters: {takes}.')
        objects = [f'- {name}: {text}' for name, text in self.objects.items()]
        zones = [f'- {name}: {text}' for name, text in self.zones.items()]

        return '\n'.join([
            'You are the task planner of a UR3e robot arm with a two-finger gripper.',
            'The robot works on a table with a few objects and target zones.',
            'The user gives a command in Vietnamese or English, in any wording.',
            'Convert the command into a plan made only of the skills listed below.',
            '',
            'Skills:',
            *skills,
            '',
            'Objects (use exactly these ids):',
            *objects,
            '',
            'Zones (use exactly these ids):',
            *zones,
            '',
            'Rules:',
            '1. Answer with one JSON object and nothing else: '
            '{"plan": [{"skill": "..."}, ...]}.',
            '2. Every step has "skill" plus exactly the parameters of that skill.',
            '3. To bring an object to a zone: "pick" it, then "place" it. Finish a '
            'plan that moved objects with "home".',
            '4. The gripper holds one object at a time and a zone holds one object '
            'at a time. Take the current state into account.',
            '5. If the target zone contains another cube, FIRST pick that occupant and '
            'place_on_table outside ALL zones. Choose its temporary [x, y] yourself '
            'using current measured positions and the workspace constraints below. '
            'Only AFTER clearing every occupant, pick the requested cube and place it in the target zone. '
            'Do not require an empty spare zone; all zones may be occupied. '
            'If the requested cube is already in the target zone, just home.',
            '6. Never invent skills, objects or zones. If the command cannot be done '
            'with the ids above, or is not a command for the robot, answer '
            '{"plan": [], "reason": "<short explanation in the language of the user>"}.',
            '',
            'Table placement constraints (gazebo_world metres):',
            f'- full cube footprint must fit inside [xmin, xmax, ymin, ymax] = {self.workspace.bounds}',
            f'- table surface Z = {self.workspace.height}; supply only [x, y]',
            f'- robot base XY = {list(self.workspace.base[:2])}; keepout radius = {self.workspace.keepout}',
            f'- maximum radial reach = {self.workspace.reach}; include cube footprint radius in both limits',
            f'- minimum gap from other cubes and every zone = {self.workspace.gap}',
            '- use circumscribed footprint radius sqrt(size_x^2 + size_y^2)/2 for clearance',
            f'- cube sizes: {json.dumps(self.workspace.sizes)}',
            f'- zone geometry: {json.dumps(self.workspace.zones)}',
            '- account for positions changed by earlier plan steps; do not choose an unverified location',
            '',
            'Example for "Đưa khối màu đỏ vào vùng B":',
            '{"plan": [{"skill": "pick", "object": "red_cube"}, '
            '{"skill": "place", "object": "red_cube", "zone": "zone_b"}, '
            '{"skill": "home"}]}',
            'Occupied-zone example, if red_cube occupies zone_b and blue_cube is requested:',
            '{"plan": [{"skill": "pick", "object": "red_cube"}, '
            '{"skill": "place_on_table", "object": "red_cube", "position": ["CHOOSE_X", "CHOOSE_Y"]}, '
            '{"skill": "pick", "object": "blue_cube"}, '
            '{"skill": "place", "object": "blue_cube", "zone": "zone_b"}, '
            '{"skill": "home"}]}',
            'CHOOSE_X and CHOOSE_Y are placeholders: replace with feasible numeric coordinates.',
        ])

    def describe_state(self, state):
        lines = [f'- gripper: holding {state.holding}' if state.holding
                 else '- gripper: empty']
        for name in self.objects:
            location = state.locations.get(name, LOCATION_UNKNOWN)
            if location == LOCATION_GRIPPER:
                lines.append(f'- {name}: in the gripper')
            elif location == LOCATION_TABLE:
                lines.append(f'- {name}: on the table, outside of the zones')
            elif location == LOCATION_UNKNOWN:
                lines.append(f'- {name}: position unknown or not visible, cannot be picked')
            else:
                lines.append(f'- {name}: in {location}')
        lines.append(f'- measured cube centres XYZ: {json.dumps(state.positions)}')
        for name in self.zones:
            occupants = state.occupants(name, self.workspace)
            lines.append(f'- {name}: holds {", ".join(occupants)}' if occupants else f'- {name}: free')
        return '\n'.join(lines)

    def scene_state_callback(self, msg):
        try:
            state = SceneState.from_dict(json.loads(msg.data))
        except ValueError as e:
            self.get_logger().warn(f"Ignoring malformed /scene_state: {e}")
            return
        with self.state_lock:
            if state.revision >= 0 and state.revision < self.scene_state.revision:
                return  # A delayed topic message must not overwrite a newer service snapshot.
            self.scene_state = state

    def command_callback(self, msg):
        command = msg.data.strip()
        if not command:
            return
        self.get_logger().info(f"Received natural language command: '{command}'")

        # The plan runs in its own thread: this callback has to return so that the
        # executor can deliver the service responses of the skill executor.
        if not self.busy.acquire(blocking=False):
            self.get_logger().warn("Still executing the previous command, ignoring this one.")
            self.publish_status(command, 'BUSY', 'previous command is still running')
            return
        threading.Thread(target=self.run_command, args=(command,), daemon=True).start()

    def run_command(self, command):
        try:
            self.process_command(command)
        except Exception as e:  # keep the node alive whatever happens to one command
            self.get_logger().error(f"Unexpected error: {e}")
            self.publish_status(command, 'FAILED', str(e))
        finally:
            self.busy.release()

    def process_command(self, command):
        status, message = self.call_skill('observe')
        if status != 'SUCCESS':
            self.publish_status(command, status, message)
            return
        with self.state_lock:
            state = SceneState(self.scene_state.holding, self.scene_state.locations,
                               self.scene_state.positions, self.scene_state.observed, self.scene_state.revision)
        if self.use_perception and not state.observed:
            self.publish_status(command, 'OBSERVATION_FAILED', 'camera scene has not been confirmed')
            return

        feedback = ''
        for attempt in range(3):
            try:
                plan_json = self.query_llm(command, state, feedback)
            except RuntimeError as error:
                self.get_logger().error(f"LLM API Error: {error}")
                self.publish_status(command, 'LLM_ERROR', str(error))
                return
            self.get_logger().info(f"Generated Plan:\n{plan_json}")
            try:
                plan_data = json.loads(extract_json(plan_json))
                plan = self.validator.validate(plan_data, state)
                break
            except (json.JSONDecodeError, PlanValidationError) as error:
                feedback = (f'Previous plan was rejected: {error}. No step was executed. '
                            f'Return a corrected complete plan. Previous answer: {plan_json}')
                # An explicit refusal has no plan to repair.
                if attempt == 2 or (isinstance(error, PlanValidationError) and
                                    isinstance(plan_data, dict) and plan_data.get('plan') == []):
                    self.publish_status(command, 'REJECTED', str(error))
                    return
                self.get_logger().warn(f"Plan rejected: {error}. Asking LLM to correct it.")

        # Execution
        self.get_logger().info(f"Plan validated ({len(plan)} steps). Executing...")
        results = []
        for index, step in enumerate(plan, start=1):
            status, message = self.call_skill(step['skill'], step['object'], step['zone'], step.get('position', []))
            results.append({**step, 'status': status, 'message': message})
            self.get_logger().info(f"Step {index}/{len(plan)} status: {status} {message}")
            if status != 'SUCCESS':
                self.get_logger().error(
                    f"Task failed with status {status}. Aborting remaining plan.")
                self.publish_status(command, status, message, results)
                return

        if self.use_perception:
            status, message = self.call_skill('observe')
            if status != 'SUCCESS':
                self.publish_status(command, status, message, results)
                return
            # Check final destinations against the image, including displaced cubes.
            destinations = {}
            for step in plan:
                if step['skill'] in ('place', 'place_on_table'):
                    destinations[step['object']] = step
            with self.state_lock:
                final = self.scene_state
                for obj, step in destinations.items():
                    expected = step['zone'] if step['skill'] == 'place' else LOCATION_TABLE
                    actual = final.locations.get(obj, LOCATION_UNKNOWN)
                    if actual != expected or (step['skill'] == 'place_on_table' and
                            any(abs(a-b) > 0.02 for a, b in zip(
                                final.positions.get(obj, [float('inf'), float('inf')]), step['position']))):
                        self.publish_status(command, 'VERIFICATION_FAILED',
                                            f'{obj}: observed destination does not match plan', results)
                        return

        self.get_logger().info("Task completed.")
        self.publish_status(command, 'SUCCESS', '', results)

    def query_llm(self, command, state, feedback=''):
        api_key = os.environ.get('GEMINI_API_KEY')
        if not api_key:
            raise RuntimeError('GEMINI_API_KEY environment variable not set')

        body = {
            'systemInstruction': {'parts': [{'text': self.system_prompt}]},
            'contents': [{'role': 'user', 'parts': [{
                'text': f'Current state:\n{self.describe_state(state)}\n\nCommand: {command}\n\n{feedback}'
            }]}],
            'generationConfig': {'responseMimeType': 'application/json'},
        }
        data = json.dumps(body).encode('utf-8')

        for index, model in enumerate(self.models, start=1):
            try:
                reply = self.request_model(model, data, api_key)
                break
            except ModelUnavailable as e:
                if index == len(self.models):
                    raise
                self.get_logger().warn(f"{e} Trying {self.models[index]} instead.")
        self.get_logger().info(f"Answer from {model}")

        candidates = reply.get('candidates') or []
        if not candidates:
            raise RuntimeError(f"no answer from the model: {reply.get('promptFeedback')}")
        parts = candidates[0].get('content', {}).get('parts', [])
        text = ''.join(p.get('text', '') for p in parts if not p.get('thought'))
        if not text.strip():
            raise RuntimeError(
                f"empty answer from the model (finishReason: "
                f"{candidates[0].get('finishReason')})")
        return text.strip()

    def request_model(self, model, data, api_key):
        request = urllib.request.Request(
            GEMINI_URL.format(model=model),
            data=data,
            headers={'Content-Type': 'application/json', 'x-goog-api-key': api_key},
            method='POST')
        for attempt in range(1, LLM_ATTEMPTS + 1):
            try:
                with urllib.request.urlopen(request, timeout=self.llm_timeout) as response:
                    return json.loads(response.read().decode('utf-8'))
            except urllib.error.HTTPError as e:
                message = f'{model}: HTTP {e.code}: {read_api_error(e)}'
                if e.code in (500, 503) and attempt < LLM_ATTEMPTS:
                    self.get_logger().warn(f"{message} Retrying...")
                    time.sleep(LLM_RETRY_DELAY_SEC)
                elif e.code in MODEL_ERRORS:
                    raise ModelUnavailable(message) from e
                else:
                    raise RuntimeError(message) from e
            except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as e:
                raise RuntimeError(f'{model}: {e}') from e

    def call_skill(self, skill, obj="", zone="", position=None):
        if not self.skill_client.wait_for_service(timeout_sec=5.0):
            return 'FAILED', 'execute_skill service is not available'

        req = ExecuteSkill.Request()
        req.skill = skill
        req.object_name = obj
        req.zone = zone
        req.position = [float(v) for v in (position or [])]

        self.get_logger().info(f"Executing: {skill} {obj} {zone}")

        # rclpy.spin() in the main thread completes the future, this thread only waits
        done = threading.Event()
        future = self.skill_client.call_async(req)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=self.skill_timeout):
            future.cancel()
            return 'FAILED', f'no answer from the skill executor after {self.skill_timeout}s'

        try:
            result = future.result()
        except Exception as e:
            return 'FAILED', f'service call failed: {e}'
        if result is None:
            return 'FAILED', 'service call failed'
        if result.scene_state:
            msg = String()
            msg.data = result.scene_state
            self.scene_state_callback(msg)
        return result.status, result.message

    def publish_status(self, command, status, detail='', steps=None):
        msg = String()
        msg.data = json.dumps(
            {'command': command, 'status': status, 'detail': detail, 'steps': steps or []},
            ensure_ascii=False)
        self.status_publisher.publish(msg)


def extract_json(text):
    """Return the JSON object embedded in the answer of the model."""
    start, end = text.find('{'), text.rfind('}')
    if start == -1 or end < start:
        return text
    return text[start:end + 1]


def read_api_error(error):
    try:
        return json.loads(error.read().decode('utf-8'))['error']['message']
    except (ValueError, KeyError, TypeError):
        return error.reason


def main(args=None):
    rclpy.init(args=args)
    node = LLMPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
