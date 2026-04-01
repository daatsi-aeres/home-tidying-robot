#!/usr/bin/python3
"""
Ground truth TF publisher.

Gets world pose from Gazebo SceneBroadcaster (may be slow ~1-5Hz),
caches it, and republishes at 30Hz using a timer so TF is always
available for laser scan lookups.
"""
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class GroundTruthTF(Node):
    def __init__(self):
        super().__init__('ground_truth_tf')
        self.br = TransformBroadcaster(self)
        self._latest_transform = None
        self._found = False

        # Subscribe to SceneBroadcaster world poses
        self.create_subscription(
            TFMessage,
            '/world/tidy_home/dynamic_pose/info',
            self.pose_cb, 10)

        # Republish cached pose at 30Hz
        self.create_timer(1.0 / 30.0, self.timer_cb)

        # Print boot sequence while Nav2 loads (fills the silent gap)
        self._boot_msgs = [
            (2.0,  "  Loading Nav2 planner server..."),
            (4.0,  "  Loading Nav2 controller server..."),
            (6.0,  "  Loading Nav2 behavior trees..."),
            (8.0,  "  Configuring costmaps..."),
            (10.0, "  Activating navigation stack..."),
            (14.0, "  Waiting for lifecycle transitions..."),
        ]
        self._boot_start = None
        self.create_timer(0.5, self.boot_cb)

        self.get_logger().info('ground_truth_tf: waiting for world pose data...')

    def pose_cb(self, msg):
        for tf in msg.transforms:
            name = tf.child_frame_id
            if name == 'tidybot' or 'tidybot' in name:
                self._latest_transform = tf
                if not self._found:
                    self.get_logger().info(
                        f'ground_truth_tf: locked onto "{name}" '
                        f'pos=[{tf.transform.translation.x:.2f}, '
                        f'{tf.transform.translation.y:.2f}]')
                    self._found = True
                return

    def boot_cb(self):
        if not self._found or not self._boot_msgs:
            return
        if self._boot_start is None:
            self._boot_start = self.get_clock().now()
        elapsed = (self.get_clock().now() - self._boot_start).nanoseconds / 1e9
        while self._boot_msgs and elapsed >= self._boot_msgs[0][0]:
            _, msg = self._boot_msgs.pop(0)
            self.get_logger().info(msg)

    def timer_cb(self):
        if self._latest_transform is None:
            return

        out = TransformStamped()
        # Use current time so TF is always fresh
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_footprint'
        out.transform = self._latest_transform.transform
        self.br.sendTransform(out)


def main():
    rclpy.init()
    node = GroundTruthTF()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
