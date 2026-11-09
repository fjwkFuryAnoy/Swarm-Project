import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from math import sqrt


class UavPatrolNode(Node):

    def __init__(self):
        super().__init__('uav_patrol_node')

        default_waypoints = [
            -10.0, -10.0,
            0.0, 0.0,
            -10.0, 8.0,
            8.0, 8.0,
            4.0, -4.0,
            10.0, 10.0,
        ]
        self.declare_parameter('waypoints', default_waypoints)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('tolerance', 0.3)
        self.declare_parameter('kp', 1.0)

        raw_wp = self.get_parameter('waypoints').value
        self.waypoints = [(raw_wp[i], raw_wp[i + 1]) for i in range(0, len(raw_wp), 2)]
        self.max_speed = self.get_parameter('max_speed').value
        self.tolerance = self.get_parameter('tolerance').value
        self.kp = self.get_parameter('kp').value

        self.wp_index = 0
        self.done = False

        self.x = None
        self.y = None

        self.create_subscription(Odometry, '/uav/odom', self.odom_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/uav/cmd_vel', 10)

        done_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.done_pub = self.create_publisher(Bool, '/uav/patrol_done', done_qos)

        self.create_timer(0.1, self.control_loop)

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def control_loop(self):
        if self.x is None:
            return

        cmd = Twist()

        if self.done:
            self.cmd_pub.publish(cmd)
            return

        wx, wy = self.waypoints[self.wp_index]
        dx = wx - self.x
        dy = wy - self.y
        dist = sqrt(dx * dx + dy * dy)

        if dist < self.tolerance:
            self.wp_index += 1
            if self.wp_index >= len(self.waypoints):
                self.done = True
                self.cmd_pub.publish(cmd)
                done_msg = Bool()
                done_msg.data = True
                self.done_pub.publish(done_msg)
                self.get_logger().info('Patrol complete')
                return
            wx, wy = self.waypoints[self.wp_index]
            dx = wx - self.x
            dy = wy - self.y
            dist = sqrt(dx * dx + dy * dy)

        if dist > 0.01:
            speed = min(self.max_speed, self.kp * dist)
            cmd.linear.x = speed * dx / dist
            cmd.linear.y = speed * dy / dist
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = UavPatrolNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
