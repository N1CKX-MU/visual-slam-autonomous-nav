#!/usr/bin/env python3
"""Bridge ORB-SLAM3 camera pose to Nav2-compatible TF transforms.

ORB-SLAM3 publishes poses in camera convention (z-forward, x-right, y-down).
ROS2/Nav2 expects the map->odom transform in ROS convention (x-forward, y-left, z-up).
This node handles the coordinate frame conversion and publishes the TF.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class OrbSlamBridge(Node):
    def __init__(self):
        super().__init__('orb_slam_bridge')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Subscribe to ORB-SLAM3 pose output
        self.create_subscription(
            PoseStamped,
            '/orb_slam3/camera_pose',
            self.pose_callback,
            10,
        )

        # Publish converted odometry for debugging
        self._odom_pub = self.create_publisher(
            Odometry, '/orb_slam3/odom', 10
        )

        self._last_pose_time = None
        self._tracking_lost_count = 0

        self.get_logger().info('ORB-SLAM3 bridge initialized')

    def pose_callback(self, msg: PoseStamped):
        """Convert ORB-SLAM3 camera pose to map->odom TF."""
        # Check for tracking loss (no pose updates)
        current_time = self.get_clock().now()
        if self._last_pose_time is not None:
            dt = (current_time - self._last_pose_time).nanoseconds / 1e9
            if dt > 2.0:
                self._tracking_lost_count += 1
                self.get_logger().warn(
                    f'ORB-SLAM3 tracking gap: {dt:.1f}s '
                    f'(lost count: {self._tracking_lost_count})'
                )
        self._last_pose_time = current_time

        # ORB-SLAM3 camera convention -> ROS convention
        # Camera: x-right, y-down, z-forward
        # ROS:    x-forward, y-left, z-up
        cam_x = msg.pose.position.x
        cam_y = msg.pose.position.y
        cam_z = msg.pose.position.z

        ros_x = cam_z       # camera z -> ROS x (forward)
        ros_y = -cam_x      # camera x -> ROS -y (left)
        ros_z = -cam_y      # camera y -> ROS -z (up)

        # Publish map -> odom transform
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'

        t.transform.translation.x = ros_x
        t.transform.translation.y = ros_y
        t.transform.translation.z = 0.0  # Project to 2D for Nav2

        # Rotation: convert camera quaternion to ROS frame
        # For 2D navigation, we primarily care about yaw
        t.transform.rotation = self._convert_camera_rotation(msg.pose.orientation)

        self.tf_broadcaster.sendTransform(t)

        # Publish as Odometry message for visualization
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = ros_x
        odom.pose.pose.position.y = ros_y
        odom.pose.pose.position.z = ros_z
        odom.pose.pose.orientation = t.transform.rotation
        self._odom_pub.publish(odom)

    def _convert_camera_rotation(self, q):
        """Convert camera-frame quaternion to ROS base_link frame.

        Applies a 90-degree rotation to align camera z-forward
        with ROS x-forward convention.
        """
        # Rotation matrix from camera to ROS frame:
        # R = Rz(-90) * Rx(90) (approximate for common setups)
        # For simplicity, we extract yaw from the camera quaternion
        # and rebuild a planar quaternion for Nav2
        import math

        # Extract yaw from camera frame (rotation around camera y-axis
        # maps to rotation around ROS z-axis)
        siny_cosp = 2.0 * (q.w * q.y - q.z * q.x)
        cosy_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        camera_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Negate because camera y-axis rotation is opposite to ROS z-axis
        ros_yaw = -camera_yaw

        from geometry_msgs.msg import Quaternion
        result = Quaternion()
        result.x = 0.0
        result.y = 0.0
        result.z = math.sin(ros_yaw / 2.0)
        result.w = math.cos(ros_yaw / 2.0)
        return result


def main(args=None):
    rclpy.init(args=args)
    node = OrbSlamBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
