import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from rclpy.clock import Clock


class TwistPublisher(Node):
    def __init__(self):
        super().__init__('twist_publisher')
        self.publisher_ = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10
        )

        self.timer_period = 0.004  # 250 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'  # Change if needed

        # Linear velocity (m/s)
        msg.twist.linear.x = 0.1
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0

        # Angular velocity (rad/s)
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing TwistStamped...')


def main(args=None):
    rclpy.init(args=args)
    node = TwistPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
