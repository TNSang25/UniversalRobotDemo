#!/usr/bin/env python3
"""Check the installed assignment_2 assets without starting ROS or calling Gemini."""

from pathlib import Path
import runpy
import sys
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from rclpy.parameter import Parameter
from rclpy.type_support import check_for_type_support
import xacro
import yaml


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def check_installation():
    shares = {
        name: Path(get_package_share_directory(name))
        for name in (
            'ur_description', 'ur_onrobot', 'ur_simulation_gz', 'ur_task_planner',
            'ur_moveit_config', 'ign_ros2_control', 'controller_manager',
            'joint_state_broadcaster', 'joint_trajectory_controller',
            'ros_gz_sim', 'ros_gz_bridge', 'moveit_ros_move_group',
            'moveit_kinematics', 'moveit_planners_ompl', 'moveit_simple_controller_manager',
        )
    }
    for package in ('ur_simulation_gz', 'ur_task_planner'):
        for launch in (shares[package] / 'launch').glob('*.launch.py'):
            runpy.run_path(str(launch))
    require((shares['ur_simulation_gz'] / 'world/ur_table.sdf').is_file(),
            'Gazebo world is missing from the installation')

    controller_file = shares['ur_onrobot'] / 'config/ur_rg2_controllers.yaml'
    mappings = {
        'ur_type': 'ur3e', 'name': 'ur', 'sim_ignition': 'true',
        'safety_limits': 'true', 'safety_pos_margin': '0.15', 'safety_k_position': '20',
        'simulation_controllers': str(controller_file),
    }
    urdf = ET.fromstring(xacro.process_file(
        str(shares['ur_onrobot'] / 'urdf/ur_with_rg2.urdf.xacro'), mappings=mappings
    ).toxml())
    joints = {joint.attrib['name'] for joint in urdf.findall('joint')}
    fingers = {'finger_joint', 'right_finger_joint'}
    require(fingers <= joints, 'URDF is missing the gripper joints')
    for mesh in urdf.findall('.//mesh'):
        uri = urlparse(mesh.attrib['filename'])
        if uri.scheme == 'package':
            path = Path(get_package_share_directory(uri.netloc)) / unquote(uri.path.lstrip('/'))
        elif uri.scheme == 'file':
            path = Path(unquote(uri.path))
        else:
            raise RuntimeError(f'Unsupported mesh URI: {mesh.attrib["filename"]}')
        require(path.is_file(), f'Mesh is missing: {path}')
    for parameters in urdf.findall('.//gazebo/plugin/parameters'):
        require(parameters.text == str(controller_file), 'Gazebo uses a different controller file')

    srdf = ET.parse(shares['ur_task_planner'] / 'config/ur_rg2.srdf').getroot()
    require(srdf.attrib['name'] == urdf.attrib['name'], 'URDF and SRDF robot names differ')
    gripper = srdf.find("group[@name='gripper']")
    require(gripper is not None, 'SRDF has no gripper group')
    require({joint.attrib['name'] for joint in gripper.findall('joint')} == fingers,
            'SRDF and URDF gripper joints differ')
    controllers = yaml.safe_load(controller_file.read_text())
    controller_joints = controllers['gripper_trajectory_controller']['ros__parameters']['joints']
    require(set(controller_joints) == fingers,
            'Trajectory controller and URDF gripper joints differ')

    # Validate values with Humble's parameter types, without creating a ROS node.
    def check_parameters(name, value):
        if isinstance(value, dict):
            for key, child in value.items():
                check_parameters(f'{name}.{key}', child)
        else:
            Parameter(name, value=value)

    for config in ('scene.yaml', 'scene_depth.yaml', 'kinematics.yaml'):
        parameters = yaml.safe_load((shares['ur_task_planner'] / 'config' / config).read_text())
        check_parameters(config, parameters['/**']['ros__parameters'])

    from ur_task_planner.srv import ExecuteSkill
    check_for_type_support(ExecuteSkill)
    from ur_task_planner.msg import ObservedScene
    check_for_type_support(ObservedScene)
    executables = Path(get_package_prefix('ur_task_planner')) / 'lib/ur_task_planner'
    require((executables / 'skill_executor_node').is_file(), 'C++ skill executor is missing')
    require((executables / 'llm_planner_node.py').is_file(), 'LLM planner is missing')
    for asset in ('plan_validator.py', 'scene_geometry.py', 'cube_perception.py', 'scene_observer_node.py'):
        require((executables / asset).is_file(), f'Installed {asset} is missing')
    print('Installation OK: launch imports, URDF/meshes, SRDF, controllers, '
          'ROS parameters, service types.')


def main():
    try:
        check_installation()
    except Exception as error:
        print(f'Installation check failed: {error}', file=sys.stderr)
        print('Source install/setup.bash and run rosdep install --from-paths src --ignore-src '
              '--rosdistro humble -y before rebuilding.', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
