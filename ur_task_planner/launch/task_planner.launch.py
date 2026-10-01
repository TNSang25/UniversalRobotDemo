import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml

GRIPPER_JOINTS = ["finger_joint", "right_finger_joint"]


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type")
    launch_rviz = LaunchConfiguration("launch_rviz")

    description_package = "ur_onrobot"
    description_file = "ur_with_rg2.urdf.xacro"

    # Build the EXACT SAME robot_description that ur_sim_control uses.
    # The key is to include sim_ignition:=true AND simulation_controllers
    # so the resulting URDF is byte-identical to what robot_state_publisher publishes.
    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "ur_rg2_controllers.yaml"]
    )
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "visual_parameters.yaml"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "safety_limits:=true",
            " ",
            "safety_pos_margin:=0.15",
            " ",
            "safety_k_position:=20",
            " ",
            "name:=ur",
            " ",
            "ur_type:=",
            ur_type,
            " ",
            'prefix:=""',
            " ",
            "sim_ignition:=true",
            " ",
            "simulation_controllers:=",
            initial_joint_controllers,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # Use our custom SRDF that matches the robot name "ur_with_rg2"
    srdf_file = os.path.join(
        context.perform_substitution(FindPackageShare("ur_task_planner")),
        "config",
        "ur_rg2.srdf",
    )
    with open(srdf_file, "r") as f:
        robot_description_semantic_content = f.read()
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    # Kinematics
    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare("ur_task_planner"), "config", "kinematics.yaml"]
    )

    # Scene shared by the skill executor and the LLM planner
    scene_params = PathJoinSubstitution(
        [FindPackageShare("ur_task_planner"), "config", "scene.yaml"]
    )

    # Joint limits
    robot_description_planning = {
        "robot_description_planning": load_yaml(
            "ur_task_planner",
            os.path.join("config", "joint_limits.yaml"),
        )
    }

    # OMPL planning
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": (
                "default_planner_request_adapters/AddTimeOptimalParameterization "
                "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints"
            ),
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)
    if "RRTConnectkConfigDefault" in ompl_planning_yaml.get("planner_configs", {}):
        ompl_planning_pipeline_config["move_group"]["gripper"] = {
            "planner_configs": ["RRTConnectkConfigDefault"]
        }

    # Controllers – use sim mode (joint_trajectory_controller as default)
    controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")
    controllers_yaml["scaled_joint_trajectory_controller"]["default"] = False
    controllers_yaml["joint_trajectory_controller"]["default"] = True
    # The gripper is commanded through MoveIt as well
    controllers_yaml["controller_names"].append("gripper_trajectory_controller")
    controllers_yaml["gripper_trajectory_controller"] = {
        "action_ns": "follow_joint_trajectory",
        "type": "FollowJointTrajectory",
        "default": True,
        "joints": GRIPPER_JOINTS,
    }

    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # move_group node – NO robot_state_publisher here, Gazebo already has one
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"publish_robot_description_semantic": True},
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": True},
        ],
    )

    # RViz – also gets robot_description + semantic so it can display the planning scene
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "rviz", "view_robot.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning,
            {"use_sim_time": True},
        ],
    )

    # Skill executor
    skill_executor_node = Node(
        package="ur_task_planner",
        executable="skill_executor_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            scene_params,
            {"startup_timeout_sec": ParameterValue(
                LaunchConfiguration("startup_timeout_sec"), value_type=float
            )},
            {"use_sim_time": True},
        ],
    )

    # LLM planner – the model defaults to $GEMINI_MODEL when llm_model is empty
    llm_planner_params = [scene_params, {"use_sim_time": True}]
    llm_model = context.perform_substitution(LaunchConfiguration("llm_model"))
    if llm_model:
        llm_planner_params.append({"model": llm_model})
    llm_planner_node = Node(
        package="ur_task_planner",
        executable="llm_planner_node.py",
        condition=IfCondition(LaunchConfiguration("launch_llm")),
        output="screen",
        parameters=llm_planner_params,
    )

    return [move_group_node, rviz_node, skill_executor_node, llm_planner_node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("ur_type", default_value="ur3e", choices=["ur3e"]),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("launch_llm", default_value="true"),
        DeclareLaunchArgument("startup_timeout_sec", default_value="120.0"),
        DeclareLaunchArgument(
            "llm_model",
            default_value="",
            description="Gemini model of the planner, $GEMINI_MODEL is used when empty.",
        ),
    ]
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
