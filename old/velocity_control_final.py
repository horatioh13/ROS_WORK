import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import threading

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.target_velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.running = True

        # Start a thread to listen for user input
        self.input_thread = threading.Thread(target=self.get_user_input)
        self.input_thread.daemon = True
        self.input_thread.start()

        # Create a timer to publish velocity commands at 100 Hz
        self.timer = self.create_timer(0.01, self.publish_velocity)

    def get_user_input(self):
        while self.running:
            try:
                user_input = input("Enter target velocities as a comma-separated list (e.g., 0.1,0.1,0.0,0.0,0.0,0.0): ")
                velocities = [float(v.strip()) for v in user_input.split(',')]
                if len(velocities) == 6:
                    self.target_velocities = velocities
                    self.get_logger().info(f"Target velocities updated to: {self.target_velocities}")
                else:
                    self.get_logger().warn("Please enter exactly 6 values.")
            except ValueError:
                self.get_logger().warn("Invalid input. Please enter valid numbers.")

    def publish_velocity(self):
        msg = Float64MultiArray()
        msg.data = self.target_velocities
        self.publisher_.publish(msg)

    def stop(self):
        self.running = False
        self.input_thread.join()

def main(args=None):
    rclpy.init(args=args)
    velocity_controller = VelocityController()

    try:
        rclpy.spin(velocity_controller)
    except KeyboardInterrupt:
        velocity_controller.get_logger().info("Shutting down...")
    finally:
        velocity_controller.stop()
        velocity_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# 0.02,0.0,0.0,0.0,0.0,0.0
# 0.0,0.0,0.0,0.0,0.0,0.0