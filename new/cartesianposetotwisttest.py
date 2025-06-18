#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
import tf2_ros
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_pose
from tf_transformations import quaternion_multiply, quaternion_inverse
import numpy as np

class CartesianPoseController(Node):
    def __init__(self):
        super().__init__('cartesian_pose_controller')

        self.target_pose = None
        self.gain = 1.0
        self.rate_hz = 100.0
        self.ee_frame = 'tool0'
        self.base_frame = 'base_link'

        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/target_pose', self.target_pose_callback, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        timer_period = 1.0 / self.rate_hz
        self.timer = self.create_timer(timer_period, self.control_loop)

    def target_pose_callback(self, msg):
        self.target_pose = msg

    def control_loop(self):
        if self.target_pose is None:
            return

        try:
            # Get the transformation from base_frame to ee_frame
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(self.base_frame, self.ee_frame, now, timeout=rclpy.duration.Duration(seconds=0.1))
        except TransformException as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        self.get_logger().info(f"Target pose before transformation: {self.target_pose}")

        try:
            # Transform the target pose to the base frame
            transformed_target_pose = do_transform_pose(self.target_pose, transform)
        except Exception as e:
            self.get_logger().warn(f"Pose transformation failed: {e}")
            return

        # Extract positions
        current_pos = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])
        target_pos = np.array([
            transformed_target_pose.pose.position.x,
            transformed_target_pose.pose.position.y,
            transformed_target_pose.pose.position.z
        ])

        # Linear velocity (P-controller)
        pos_error = target_pos - current_pos
        linear = self.gain * pos_error

        # Extract orientations
        current_ori = np.array([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ])
        target_ori = np.array([
            transformed_target_pose.pose.orientation.x,
            transformed_target_pose.pose.orientation.y,
            transformed_target_pose.pose.orientation.z,
            transformed_target_pose.pose.orientation.w
        ])

        # Angular velocity (quaternion error)
        q_error = quaternion_multiply(target_ori, quaternion_inverse(current_ori))
        angular = self.gain * np.array(q_error[:3])  # Drop w

        # Publish the twist message
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.base_frame
        twist.twist.linear.x = linear[0]
        twist.twist.linear.y = linear[1]
        twist.twist.linear.z = linear[2]
        twist.twist.angular.x = angular[0]
        twist.twist.angular.y = angular[1]
        twist.twist.angular.z = angular[2]

        self.twist_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = CartesianPoseController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()