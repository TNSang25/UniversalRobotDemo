"""Exercise command execution without Gemini requests or robot motion."""
import json
from pathlib import Path
import sys
import threading
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from llm_planner_node import LLMPlannerNode
from plan_validator import PlanValidator, SceneState
from scene_geometry import TableWorkspace

PLAN = {'plan': [
    {'skill': 'pick', 'object': 'red_cube'},
    {'skill': 'place_on_table', 'object': 'red_cube', 'position': [-.12, .25]},
    {'skill': 'pick', 'object': 'blue_cube'},
    {'skill': 'place', 'object': 'blue_cube', 'zone': 'zone_b'},
    {'skill': 'home'},
]}


class Harness:
    process_command = LLMPlannerNode.process_command

    def __init__(self, observation_status='SUCCESS', misplaced=False):
        self.use_perception = True
        self.state_lock = threading.Lock()
        self.scene_state = SceneState(locations={'red_cube': 'zone_b', 'blue_cube': 'table'},
                                      positions={'red_cube': [.1, 0, .835], 'blue_cube': [-.1, -.15, .825]},
                                      observed=True)
        workspace = TableWorkspace([-.25, .015, -.34, .34], [-.35, 0, .81],
                                   {'zone_b': {'position': [.1, 0, .81], 'size': [.15, .15]}},
                                   {'red_cube': [.05]*3, 'blue_cube': [.05]*3})
        self.validator = PlanValidator(['red_cube', 'blue_cube'], ['zone_b'], workspace=workspace)
        self.calls, self.queries, self.statuses = [], [], []
        self.observation_status, self.misplaced = observation_status, misplaced

    def get_logger(self):
        return SimpleNamespace(info=lambda _: None, error=lambda _: None, warn=lambda _: None)

    def query_llm(self, command, state, feedback=''):
        self.queries.append(feedback)
        # First answer chooses a position on the target zone; the LLM must repair it.
        answer = json.loads(json.dumps(PLAN))
        if not feedback:
            answer['plan'][1]['position'] = [.1, 0]
        return json.dumps(answer)

    def call_skill(self, skill, obj='', zone='', position=None):
        self.calls.append(skill)
        state = self.scene_state
        if skill == 'observe':
            if self.misplaced and self.calls.count('observe') > 1:
                state.locations['blue_cube'] = 'table'
            return self.observation_status, ''
        if skill == 'pick':
            state.holding = obj
            state.locations[obj] = 'gripper'
        elif skill in ('place', 'place_on_table'):
            state.holding = None
            state.locations[obj] = zone if skill == 'place' else 'table'
            state.positions[obj] = [.1, 0, .835] if skill == 'place' else [*position, .825]
        return 'SUCCESS', ''

    def publish_status(self, command, status, detail='', steps=None):
        self.statuses.append(status)


def test_planner_repairs_temp_position_then_clears_zone_before_requested_cube():
    node = Harness()
    node.process_command('Đưa cube xanh vào B')
    assert node.calls == ['observe', 'pick', 'place_on_table', 'pick', 'place', 'home', 'observe']
    assert len(node.queries) == 2
    assert 'rejected' in node.queries[1]
    assert node.statuses == ['SUCCESS']


def test_no_motion_or_llm_request_when_camera_cannot_confirm_scene():
    node = Harness(observation_status='OBSERVATION_FAILED')
    node.process_command('Đưa cube xanh vào B')
    assert node.calls == ['observe']
    assert node.queries == []
    assert node.statuses == ['OBSERVATION_FAILED']


def test_physical_destination_mismatch_never_reports_success():
    node = Harness(misplaced=True)
    node.process_command('Đưa cube xanh vào B')
    assert node.statuses == ['VERIFICATION_FAILED']


def test_delayed_scene_topic_cannot_overwrite_service_snapshot():
    node = Harness()
    node.scene_state.revision = 8
    msg = SimpleNamespace(data=json.dumps({'revision': 7, 'objects': {'red_cube': 'table'}}))
    LLMPlannerNode.scene_state_callback(node, msg)
    assert node.scene_state.locations['red_cube'] == 'zone_b'
