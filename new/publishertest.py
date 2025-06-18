#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class TargetPosePublisher(Node):
    def __init__(self):
        super().__init__('target_pose_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/target_pose', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_pose)

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'  # Make sure this matches what your controller expects

        msg.pose.position.x = 1.7
        msg.pose.position.y = -1.7
        msg.pose.position.z = 1.6

        msg.pose.orientation.x = -1.38
        msg.pose.orientation.y = -1.51
        msg.pose.orientation.z = 1.91
        msg.pose.orientation.w = 1.0

        self.get_logger().info(f'Publishing: {msg}')
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetPosePublisher()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
