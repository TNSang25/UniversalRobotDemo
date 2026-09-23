import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type")
    
    safety_limits = "true"
    safety_pos_margin = "0.15"
    safety_k_position = "20"
    prefix = '""'
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    
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
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
            " ",
            "joint_limit_params:=", joint_limit_params,
            " ",
            "kinematics_params:=", kinematics_params,
            " ",
            "physical_params:=", physical_params,
            " ",
            "visual_params:=", visual_params,
            " ",
            "safety_limits:=", safety_limits,
            " ",
            "safety_pos_margin:=", safety_pos_margin,
            " ",
            "safety_k_position:=", safety_k_position,
            " ",
            "name:=ur",
            " ",
            "ur_type:=", ur_type,
            " ",
            "script_filename:=ros_control.urscript",
            " ",
            "input_recipe_filename:=rtde_input_recipe.txt",
            " ",
            "output_recipe_filename:=rtde_output_recipe.txt",
            " ",
            "prefix:=", prefix,
            " ",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # MoveIt Configuration
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "srdf", moveit_config_file]
            ),
            " ",
            "name:=ur",
            " ",
            "prefix:=", prefix,
            " ",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)}

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )
    
    trajectory_drawer_node = Node(
        package="ur_trajectory_drawer",
        executable="trajectory_drawer_node",
        name="trajectory_drawer_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            {"use_sim_time": True}
        ],
    )

    return [trajectory_drawer_node]

def generate_launch_description():
    ur_type_arg = DeclareLaunchArgument(
        "ur_type", default_value="ur5e", description="Type/series of used UR robot."
    )
    
    # We set default to standard ur_description and ur_moveit_config
    description_package_arg = DeclareLaunchArgument(
        "description_package", default_value="ur_description"
    )
    description_file_arg = DeclareLaunchArgument(
        "description_file", default_value="ur.urdf.xacro"
    )
    moveit_config_package_arg = DeclareLaunchArgument(
        "moveit_config_package", default_value="ur_moveit_config"
    )
    moveit_config_file_arg = DeclareLaunchArgument(
        "moveit_config_file", default_value="ur.srdf.xacro"
    )
    
    return LaunchDescription(
        [
            ur_type_arg,
            description_package_arg,
            description_file_arg,
            moveit_config_package_arg,
            moveit_config_file_arg,
            OpaqueFunction(function=launch_setup)
        ]
    )
