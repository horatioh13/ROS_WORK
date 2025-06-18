import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import time

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.current_joint_positions = np.zeros(6)  # Replace with actual joint state feedback
        self.target_tcp_position = np.array([0.5, 0.0, 0.3])  # Example target TCP position
        self.target_tcp_orientation = np.array([0.0, 0.0, 0.0, 1.0])  # Example target TCP orientation (quaternion)
        self.timer = self.create_timer(0.01, self.publish_velocity)  # Publish at 100 Hz

    def compute_ik(self, tcp_position, tcp_orientation):
        # Replace this with your IK solver (e.g., trac_ik or kdl_kinematics)
        # For simplicity, this example assumes a direct mapping to joint positions
        joint_positions = np.array([0.0, -1.57, 1.57, 0.0, 0.0, 0.0])  # Example IK result
        return joint_positions

    def compute_joint_velocities(self, current_positions, target_positions, time_step):
        return (target_positions - current_positions) / time_step

    def publish_velocity(self):
        # Compute desired joint positions using IK
        desired_joint_positions = self.compute_ik(self.target_tcp_position, self.target_tcp_orientation)

        # Compute joint velocities
        time_step = 0.01  # 100 Hz
        joint_velocities = self.compute_joint_velocities(self.current_joint_positions, desired_joint_positions, time_step)

        # Publish joint velocities
        msg = Float64MultiArray()
        msg.data = joint_velocities.tolist()
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published joint velocities: {joint_velocities}")

        # Update current joint positions (simulate feedback for this example)
        self.current_joint_positions += joint_velocities * time_step

def main(args=None):
    rclpy.init(args=args)
    velocity_controller = VelocityController()

    try:
        rclpy.spin(velocity_controller)
    except KeyboardInterrupt:
        velocity_controller.get_logger().info("Shutting down...")
    finally:
        velocity_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()