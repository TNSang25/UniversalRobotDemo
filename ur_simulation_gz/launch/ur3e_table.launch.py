from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ur_sim_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_simulation_gz"), "/launch/ur_sim_control.launch.py"]
        ),
        launch_arguments={
            "ur_type": "ur3e",
            "world_file": PathJoinSubstitution([FindPackageShare("ur_simulation_gz"), "world", "ur_table.sdf"]),
            "spawn_x": "-0.35",
            "spawn_y": "0.0",
            "spawn_z": "0.81",
        }.items(),
    )

    return LaunchDescription([
        ur_sim_control_launch
    ])
