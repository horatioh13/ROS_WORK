import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
import time  # Import time for rate limiting


class CentralController(Node):
    def __init__(self):
        super().__init__('central_controller')
        self.target_position = None  # Target position will be initialized later
        self.current_position = None  # Current position will be initialized later
        self.kp = 1.0  # Proportional gain for velocity control
        self.last_log_time = time.time()  # Track the last log time
        self.log_interval = 1.0  # Log every 1 second

        # Subscribers
        self.target_subscription = self.create_subscription(
            PoseStamped,
            '/target_position',
            self.target_callback,
            10
        )
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/tcp_pose_broadcaster/pose',
            self.pose_callback,
            10
        )

        # Publisher
        self.publisher_ = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10
        )
        self.get_logger().info("Central controller initialized.")

    def target_callback(self, msg):
        # Update the target position
        self.target_position['x'] = msg.pose.position.x
        self.target_position['y'] = msg.pose.position.y
        self.get_logger().info(f"Received Target Position: {self.target_position}")

    def pose_callback(self, msg):
        # Initialize current and target positions if not already set
        if self.current_position is None or self.target_position is None:
            self.current_position = {'x': msg.pose.position.x, 'y': msg.pose.position.y}
            self.target_position = {'x': msg.pose.position.x, 'y': msg.pose.position.y}
            self.get_logger().info(f"Initialized Current and Target Positions: {self.current_position}")
            return

        # Update the current position
        self.current_position['x'] = msg.pose.position.x
        self.current_position['y'] = msg.pose.position.y

        # Calculate velocity based on the difference between target and current position
        delta_x = self.target_position['x'] - self.current_position['x']
        delta_y = self.target_position['y'] - self.current_position['y']

        # Deadband to ignore small differences
        deadband = 0.001  # 1 mm tolerance
        if abs(delta_x) < deadband and abs(delta_y) < deadband:
            self.get_logger().info("Target position reached. No velocity published.")
            return

        # Adjust proportional gain if needed
        velocity_x = self.kp * delta_x
        velocity_y = self.kp * delta_y

        # Create and publish TwistStamped message
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = velocity_x
        msg.twist.linear.y = velocity_y

        self.publisher_.publish(msg)

        # Log at a reduced rate
        current_time = time.time()
        if current_time - self.last_log_time >= self.log_interval:
            self.get_logger().info(f"Received Current Position: {self.current_position}")
            self.get_logger().info(f"Published Velocity: linear.x={velocity_x}, linear.y={velocity_y}")
            self.last_log_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = CentralController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()