import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.target_velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # Subscribe to the /generated_velocities topic
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/generated_velocities',
            self.update_target_velocities,
            10
        )

        # Create a timer to publish velocity commands at 100 Hz
        self.timer = self.create_timer(0.01, self.publish_velocity)

    def update_target_velocities(self, msg):
        self.target_velocities = msg.data
        self.get_logger().info(f"Received new target velocities: {self.target_velocities}")

    def publish_velocity(self):
        msg = Float64MultiArray()
        msg.data = self.target_velocities
        self.publisher_.publish(msg)

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