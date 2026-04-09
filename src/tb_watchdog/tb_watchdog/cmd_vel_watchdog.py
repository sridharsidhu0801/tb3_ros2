import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class CmdVelWatchdog(Node):
    def __init__(self):
        super().__init__('cmd_vel_watchdog')

        self.declare_parameter('input_topic', '/tb1/cmd_vel_raw')
        self.declare_parameter('output_topic', '/tb1/cmd_vel')
        self.declare_parameter('timeout_sec', 0.5)
        self.declare_parameter('publish_rate', 20.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            Twist,
            input_topic,
            self.cmd_callback,
            qos
        )

        self.pub = self.create_publisher(
            Twist,
            output_topic,
            qos
        )

        self.last_cmd = Twist()
        self.last_rx_time = self.get_clock().now()
        self.have_received_cmd = False

        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'Watchdog started. input={input_topic}, output={output_topic}, '
            f'timeout={self.timeout_sec}s, publish_rate={publish_rate}Hz'
        )

    def cmd_callback(self, msg: Twist):
        self.last_cmd = msg
        self.last_rx_time = self.get_clock().now()
        self.have_received_cmd = True

    def timer_callback(self):
        now = self.get_clock().now()
        dt = (now - self.last_rx_time).nanoseconds / 1e9

        out = Twist()

        if self.have_received_cmd and dt <= self.timeout_sec:
            out = self.last_cmd
        else:
            out.linear.x = 0.0
            out.linear.y = 0.0
            out.linear.z = 0.0
            out.angular.x = 0.0
            out.angular.y = 0.0
            out.angular.z = 0.0

        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
