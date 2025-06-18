import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math

class VelocityGenerator(Node):
    def __init__(self):
        super().__init__('velocity_generator')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/generated_velocities', 10)
        self.timer = self.create_timer(0.01, self.publish_velocities)  # Publish at 10 Hz

    def publish_velocities(self):
        velocities = [-0.1 ,-0.1 ,0.0, 0.0, 0.0, 0.0]

        msg = Float64MultiArray()
        msg.data = velocities
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published velocities: {velocities}")

def main(args=None):
    rclpy.init(args=args)
    velocity_generator = VelocityGenerator()

    try:
        rclpy.spin(velocity_generator)
    except KeyboardInterrupt:
        velocity_generator.get_logger().info("Shutting down...")
    finally:
        velocity_generator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()