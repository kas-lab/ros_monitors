import rclpy
from rclpy.node import Node
from ros_monitors_functions.factory import MonitorFactory
from std_msgs.msg import String
from nav_msgs.msg import Odometry


class MinimalMonitor(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10)
        self.monitor = MonitorFactory()

    def listener_callback(self, msg):
        self.get_logger().info(str(self.monitor["calc_linear_velocity"](self, msg)))


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalMonitor()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()