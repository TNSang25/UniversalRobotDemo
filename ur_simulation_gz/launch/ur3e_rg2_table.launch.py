from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ur_onrobot_share = get_package_share_directory('ur_onrobot')
    ur_description_share = get_package_share_directory('ur_description')
    resource_paths = [os.path.dirname(ur_onrobot_share), os.path.dirname(ur_description_share)]
    new_path = ':'.join(resource_paths) + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    os.environ['GZ_SIM_RESOURCE_PATH'] = new_path
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = new_path

    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration

    launch_rviz_arg = DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")

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
            "description_package": "ur_onrobot",
            "description_file": "ur_with_rg2.urdf.xacro",
            "runtime_config_package": "ur_onrobot",
            "controllers_file": "ur_rg2_controllers.yaml",
            "launch_rviz": LaunchConfiguration("launch_rviz"),
        }.items(),
    )

    from launch.actions import ExecuteProcess
    
    bash_script = """
for ctrl in gripper_trajectory_controller; do
  while true; do
    out=$(ros2 run controller_manager spawner $ctrl -c /controller_manager --ros-args -p use_sim_time:=true 2>&1)
    if [[ "$?" == "0" || "$out" == *"already active"* || "$out" == *"already loaded"* ]]; then
      echo "Successfully started $ctrl"
      break
    fi
    echo "Retrying $ctrl..."
    sleep 2
  done
done
"""
    spawn_controllers = ExecuteProcess(
        cmd=['bash', '-c', bash_script],
        output='screen'
    )

    return LaunchDescription([
        launch_rviz_arg,
        ur_sim_control_launch,
        spawn_controllers
    ])
