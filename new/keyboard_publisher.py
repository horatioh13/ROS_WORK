import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import sys
import termios
import tty


class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(
            PoseStamped,
            '/target_position',
            10
        )
        self.subscription = self.create_subscription(
            PoseStamped,
            '/tcp_pose_broadcaster/pose',
            self.pose_callback,
            10
        )
        self.current_position = None  # Current position will be initialized later
        self.target_position = None  # Target position will be initialized later
        self.get_logger().info("Keyboard publisher initialized. Waiting for initial position...")

    def pose_callback(self, msg):
        # Initialize current and target positions if not already set
        if self.current_position is None or self.target_position is None:
            self.current_position = {'x': msg.pose.position.x, 'y': msg.pose.position.y}
            self.target_position = {'x': msg.pose.position.x, 'y': msg.pose.position.y}
            self.get_logger().info(f"Initialized Current and Target Positions: {self.current_position}")

    def get_key(self):
        # Capture keyboard input
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        # Wait until the current and target positions are initialized
        while self.current_position is None or self.target_position is None:
            self.get_logger().info("Waiting for initial position...")
            rclpy.spin_once(self)

        while rclpy.ok():
            key = self.get_key()

            # Update target position based on key input
            if key == 'w':  # Forward
                self.target_position['x'] += 0.1
            if key == 's':  # Backward
                self.target_position['x'] -= 0.1
            if key == 'a':  # Left
                self.target_position['y'] += 0.1
            if key == 'd':  # Right
                self.target_position['y'] -= 0.1
            if key == 'q':  # Quit
                self.get_logger().info("Exiting keyboard publisher.")
                break

            # Publish the target position
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x = self.target_position['x']
            msg.pose.position.y = self.target_position['y']
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published Target Position: {self.target_position}")

        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()