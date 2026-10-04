from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ur_simulation_gz'), 'launch', 'ur_sim_control.launch.py'
            ])
        ),
        launch_arguments={
            'ur_type': 'ur3e',
            'world_file': LaunchConfiguration('world_file'),
            'spawn_x': '-0.35',
            'spawn_y': '0.0',
            'spawn_z': '0.81',
            'description_package': 'ur_onrobot',
            'description_file': 'ur_with_rg2.urdf.xacro',
            'runtime_config_package': 'ur_onrobot',
            'controllers_file': 'ur_rg2_controllers.yaml',
            'additional_controllers': 'gripper_trajectory_controller',
            'controller_spawner_timeout': LaunchConfiguration('controller_spawner_timeout'),
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'gazebo_gui': LaunchConfiguration('gazebo_gui'),
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('world_file', default_value=PathJoinSubstitution([
            FindPackageShare('ur_simulation_gz'), 'world', 'ur_table.sdf'
        ])),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('controller_spawner_timeout', default_value='120'),
        simulation,
    ])
