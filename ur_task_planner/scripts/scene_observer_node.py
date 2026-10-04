#!/usr/bin/env python3
"""Publish measured cube poses; the executor owns the manipulation state."""
import math

from cv_bridge import CvBridge
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformException, TransformListener
from ur_task_planner.msg import ObservedScene

from cube_perception import detect_cubes, rotation_matrix


class SceneObserverNode(Node):
    def __init__(self):
        super().__init__('scene_observer_node', automatically_declare_parameters_from_overrides=True)
        self.bridge = CvBridge()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.sizes = {name: self.get_parameter(f'objects.{name}.size').value
                      for name in self.get_parameter('object_names').value}
        self.height = float(self.get_parameter('table_surface_z').value)
        self.info = None
        self.info_sub = self.create_subscription(
            CameraInfo, '/depth_camera/camera_info', self.set_info, qos_profile_sensor_data)
        self.rgb_sub = message_filters.Subscriber(
            self, Image, '/depth_camera/image', qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(
            self, Image, '/depth_camera/depth_image', qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=5, slop=0.04)
        self.sync.registerCallback(self.observe)
        self.publisher = self.create_publisher(ObservedScene, 'observed_scene', 1)
        self.last_warning = 0

    def set_info(self, msg):
        self.info = msg

    def observe(self, rgb_msg, depth_msg):
        if self.info is None:
            return
        try:
            if (rgb_msg.header.frame_id != depth_msg.header.frame_id
                    or self.info.header.frame_id != rgb_msg.header.frame_id):
                raise ValueError('RGB/depth/CameraInfo must share the registered optical frame')
            tf = self.buffer.lookup_transform(
                'gazebo_world', rgb_msg.header.frame_id, Time.from_msg(rgb_msg.header.stamp))
            q, t = tf.transform.rotation, tf.transform.translation
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'rgb8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough').astype(np.float64)
            if depth_msg.encoding == '16UC1':
                depth *= 0.001
            elif depth_msg.encoding != '32FC1':
                raise ValueError(f'unsupported depth encoding: {depth_msg.encoding}')
            poses = detect_cubes(rgb, depth, self.info.k,
                                 rotation_matrix([q.x, q.y, q.z, q.w]),
                                 np.array([t.x, t.y, t.z]), self.sizes, self.height)
        except (TransformException, ValueError, RuntimeError) as error:
            now = self.get_clock().now().nanoseconds
            if now - self.last_warning > 5_000_000_000:
                self.get_logger().warn(f'Observation unavailable: {error}')
                self.last_warning = now
            return
        msg = ObservedScene()
        msg.header.stamp = rgb_msg.header.stamp
        msg.header.frame_id = 'gazebo_world'
        for name, (centre, yaw) in poses.items():
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = centre
            pose.orientation.z = math.sin(yaw/2)
            pose.orientation.w = math.cos(yaw/2)
            msg.object_names.append(name)
            msg.poses.append(pose)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SceneObserverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
