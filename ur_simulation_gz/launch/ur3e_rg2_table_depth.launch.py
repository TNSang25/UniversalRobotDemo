import math
import random
import tempfile
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


DEFAULT_CUBE_REGION = (-0.25, 0.015, -0.34, 0.34)
CUBE_NAMES = ('red_cube', 'yellow_cube', 'blue_cube', 'green_cube', 'pink_cube')
ROBOT_XY = (-0.35, 0.0)
ROBOT_KEEPOUT_RADIUS = 0.12
CUBE_GAP = 0.015


def randomize_cube_poses(world, bounds, seed=None):
    """Sample full cube footprints inside bounds, away from robot and each other."""
    xmin, xmax, ymin, ymax = bounds
    if not all(math.isfinite(value) for value in bounds) or xmin >= xmax or ymin >= ymax:
        raise ValueError('Cube region must have finite, increasing X/Y bounds.')

    table = world.find("model[@name='table']")
    table_pose = [float(value) for value in table.findtext('pose').split()]
    table_size = [float(value) for value in table.findtext(
        'link/collision/geometry/box/size').split()]
    tray_front = min(
        float(tray.findtext('pose').split()[0])
        - float(tray.findtext('link/collision/geometry/box/size').split()[0]) / 2
        for tray in world.findall('model') if tray.get('name').startswith('tray_')
    )
    if not (
        table_pose[0] - table_size[0] / 2 <= xmin
        and xmax <= min(table_pose[0] + table_size[0] / 2, tray_front - 0.01)
        and table_pose[1] - table_size[1] / 2 <= ymin
        and ymax <= table_pose[1] + table_size[1] / 2
        and xmin >= ROBOT_XY[0]
    ):
        raise ValueError('Cube region must be on the table, between robot and zones '
                         '(at least 1 cm before the trays).')

    rng = random.Random(seed)
    sampled = []
    # Use circumscribed circles so clearance holds for every sampled yaw.
    for name in CUBE_NAMES:
        cube = world.find(f"model[@name='{name}']")
        size = [float(value) for value in cube.findtext(
            'link/collision/geometry/box/size').split()]
        radius = math.hypot(size[0], size[1]) / 2
        if xmax - xmin <= 2 * radius or ymax - ymin <= 2 * radius:
            raise ValueError('Cube region is too small for the cube footprints.')
        for _ in range(10000):
            x = rng.uniform(xmin + radius, xmax - radius)
            y = rng.uniform(ymin + radius, ymax - radius)
            if math.hypot(x - ROBOT_XY[0], y - ROBOT_XY[1]) < ROBOT_KEEPOUT_RADIUS + radius:
                continue
            if any(math.hypot(x - px, y - py) < radius + pr + CUBE_GAP
                   for px, py, pr in sampled):
                continue
            sampled.append((x, y, radius))
            # Place on the table; only yaw changes, so the bottom remains flat.
            z = table_pose[2] + table_size[2] / 2 + size[2] / 2
            yaw = rng.uniform(-math.pi, math.pi)
            cube.find('pose').text = f'{x:.10f} {y:.10f} {z:.10f} 0 0 {yaw:.10f}'
            break
        else:
            raise ValueError('Could not fit five separated cubes in this region; enlarge it.')


def launch_randomized_simulation(context):
    """Generate a private world for this launch; never pass cube poses to skills."""
    package = FindPackageShare('ur_simulation_gz')
    source = PathJoinSubstitution([package, 'world', 'ur_table_depth.sdf']).perform(context)
    tree = ET.parse(source)
    bounds = tuple(float(LaunchConfiguration(name).perform(context)) for name in (
        'cube_x_min', 'cube_x_max', 'cube_y_min', 'cube_y_max'
    ))
    seed_text = LaunchConfiguration('random_seed').perform(context).strip()
    randomize_cube_poses(tree.getroot().find('world'), bounds,
                         seed=int(seed_text) if seed_text else None)
    temporary_world = tempfile.TemporaryDirectory(prefix='ur3e_random_world_')
    world_file = temporary_world.name + '/ur_table_depth.sdf'
    try:
        tree.write(world_file, encoding='utf-8', xml_declaration=True)
    except Exception:
        temporary_world.cleanup()
        raise
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            package, 'launch', 'ur3e_rg2_table.launch.py'
        ])),
        launch_arguments={
            'world_file': world_file,
            'launch_rviz': LaunchConfiguration('launch_rviz'),
            'gazebo_gui': LaunchConfiguration('gazebo_gui'),
            'controller_spawner_timeout': LaunchConfiguration('controller_spawner_timeout'),
        }.items(),
    )
    return [
        RegisterEventHandler(OnShutdown(
            on_shutdown=lambda event, context: temporary_world.cleanup()
        )),
        simulation,
    ]


def generate_launch_description():
    package = FindPackageShare('ur_simulation_gz')
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='depth_camera_bridge',
        parameters=[{
            'use_sim_time': True,
            'config_file': PathJoinSubstitution([
                package, 'config', 'depth_camera_bridge.yaml'
            ]),
        }],
        output='screen',
    )
    # Fortress publishes XYZ with +X forward while labeling the cloud with
    # the optical frame. Correct only its header; keep image headers optical.
    point_cloud_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='depth_camera_points_bridge',
        arguments=[
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
        ],
        parameters=[{'use_sim_time': True, 'override_frame_id': 'depth_camera_link'}],
        output='screen',
    )

    def static_transform(name, parent, child, xyz, rpy):
        return Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=name,
            arguments=[
                '--x', xyz[0], '--y', xyz[1], '--z', xyz[2],
                '--roll', rpy[0], '--pitch', rpy[1], '--yaw', rpy[2],
                '--frame-id', parent, '--child-frame-id', child,
            ],
            parameters=[{'use_sim_time': True}],
        )

    return LaunchDescription([
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('controller_spawner_timeout', default_value='120'),
        DeclareLaunchArgument(
            'random_seed', default_value='',
            description='Empty: new layout every launch; integer: repeatable layout.',
        ),
        *[DeclareLaunchArgument(
              name, default_value=str(value),
              description='Region boundary in gazebo_world (meters), including full cube footprints.',
          )
          for name, value in zip(
              ('cube_x_min', 'cube_x_max', 'cube_y_min', 'cube_y_max'), DEFAULT_CUBE_REGION
          )],
        OpaqueFunction(function=launch_randomized_simulation),
        camera_bridge,
        point_cloud_bridge,
        # The URDF's "world" moves with the robot's Gazebo spawn pose.
        static_transform('gazebo_to_robot_world', 'gazebo_world', 'world',
                         ['-0.35', '0', '0.81'], ['0', '0', '0']),
        # Match depth_camera's model pose in ur_table_depth.sdf.
        static_transform('depth_camera_pose', 'gazebo_world', 'depth_camera_link',
                         ['0', '0', '1.6'], ['0', '1.5707963267948966', '0']),
        # ROS optical convention: +Z forward, +X right, +Y down.
        static_transform('depth_camera_optical_pose', 'depth_camera_link',
                         'depth_camera_optical_frame', ['0', '0', '0'],
                         ['-1.5707963267948966', '0', '-1.5707963267948966']),
    ])
