import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import Twist
import numpy as np


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class UgvControllerNode(Node):

    def __init__(self):
        super().__init__('ugv_controller_node')

        self.declare_parameter('cooperative_mode', True)
        self.cooperative = self.get_parameter('cooperative_mode').value

        self.x = None
        self.y = None
        self.yaw = None
        self.path = None

        self.shared_costmap = None
        self.lidar_costmap = None

        self.current_v = 0.0
        self.reached_goal = False
        self.goal = (12.0, 12.0)

        self.first_motion_time = None
        self.last_pose = None
        self.total_distance = 0.0
        self.mine_time = 0.0
        self.mud_time = 0.0
        self.road_time = 0.0

        costmap_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(Odometry, '/ugv/odom', self.odom_cb, 10)
        self.create_subscription(Path, '/ugv/planned_path', self.path_cb, 10)
        self.create_subscription(OccupancyGrid, '/shared_costmap',
                                 self.shared_costmap_cb, costmap_qos)
        self.create_subscription(OccupancyGrid, '/lidar_costmap',
                                 self.lidar_costmap_cb, costmap_qos)

        self.cmd_pub = self.create_publisher(Twist, '/ugv/cmd_vel', 10)

        self.create_timer(0.05, self.control_loop)

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation)

    def path_cb(self, msg):
        if len(msg.poses) > 0:
            self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        else:
            self.path = None

    def shared_costmap_cb(self, msg):
        self.shared_costmap = np.array(msg.data, dtype=np.int8).reshape((150, 150))

    def lidar_costmap_cb(self, msg):
        self.lidar_costmap = np.array(msg.data, dtype=np.int8).reshape((150, 150))

    def control_loop(self):
        cmd = Twist()

        if self.x is None or self.yaw is None:
            self.cmd_pub.publish(cmd)
            return

        dist_to_goal = math.sqrt((self.x - self.goal[0])**2 + (self.y - self.goal[1])**2)

        if dist_to_goal <= 0.5:
            self.current_v = 0.0
            self.cmd_pub.publish(cmd)
            if not self.reached_goal:
                self.reached_goal = True
                now_sec = self.get_clock().now().nanoseconds / 1e9
                elapsed = (now_sec - self.first_motion_time) if self.first_motion_time else 0.0
                self.get_logger().info(
                    f"MISSION REPORT: elapsed_s={elapsed:.2f} distance_m={self.total_distance:.2f} "
                    f"mine_time_s={self.mine_time:.2f} mud_time_s={self.mud_time:.2f} "
                    f"road_time_s={self.road_time:.2f}")
            return

        if self.reached_goal:
            self.cmd_pub.publish(cmd)
            return

        if self.path is None or len(self.path) == 0:
            self.current_v = max(0.0, self.current_v - 2.0 * 0.05)
            cmd.linear.x = self.current_v
            self.cmd_pub.publish(cmd)
            return

        active_costmap = self.shared_costmap if self.cooperative else self.lidar_costmap
        if active_costmap is None:
            self.cmd_pub.publish(cmd)
            return

        closest_idx = 0
        min_dist = float('inf')
        for i, (px, py) in enumerate(self.path):
            d = math.sqrt((self.x - px)**2 + (self.y - py)**2)
            if d < min_dist:
                min_dist = d
                closest_idx = i

        lookahead_dist = 0.8
        target_idx = closest_idx
        for i in range(closest_idx, len(self.path)):
            d = math.sqrt((self.x - self.path[i][0])**2 + (self.y - self.path[i][1])**2)
            if d >= lookahead_dist:
                target_idx = i
                break
        else:
            target_idx = len(self.path) - 1

        tx, ty = self.path[target_idx]

        dx = tx - self.x
        dy = ty - self.y
        target_yaw = math.atan2(dy, dx)
        alpha = target_yaw - self.yaw
        alpha = (alpha + math.pi) % (2.0 * math.pi) - math.pi

        max_c = 0
        for i in range(closest_idx, len(self.path)):
            d = math.sqrt((self.x - self.path[i][0])**2 + (self.y - self.path[i][1])**2)
            if d <= 1.5:
                gx = int(math.floor((self.path[i][0] + 15.0) / 0.20))
                gy = int(math.floor((self.path[i][1] + 15.0) / 0.20))
                gx = max(0, min(149, gx))
                gy = max(0, min(149, gy))
                cell = int(active_costmap[gy, gx])
                if cell == -1:
                    cell = 30
                cell = min(cell, 90)
                if cell > max_c:
                    max_c = cell
            else:
                break

        v0 = 2.0
        v_terrain = v0 / (1.0 + max_c / 10.0)
        v_terrain = max(0.15, v_terrain)

        v_curv = v_terrain * max(0.25, math.cos(alpha))

        if abs(alpha) > 0.6:
            target_v = 0.0
            target_omega = 0.8 if alpha > 0 else -0.8
        else:
            target_v = v_curv
            target_omega = 2.0 * target_v * math.sin(alpha) / lookahead_dist
            target_omega = max(-1.5, min(1.5, target_omega))

        max_acc = 1.0 * 0.05
        max_dec = 2.0 * 0.05

        if target_v > self.current_v:
            self.current_v = min(target_v, self.current_v + max_acc)
        else:
            self.current_v = max(target_v, self.current_v - max_dec)

        if abs(alpha) > 0.6 and self.current_v > 0.05:
            cmd.linear.x = self.current_v
            cmd.angular.z = target_omega * 0.5
        elif abs(alpha) > 0.6:
            cmd.linear.x = 0.0
            cmd.angular.z = target_omega
        else:
            cmd.linear.x = self.current_v
            cmd.angular.z = target_omega

        self.cmd_pub.publish(cmd)

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.current_v > 0.01:
            if self.first_motion_time is None:
                self.first_motion_time = now_sec

            if self.last_pose is not None:
                step_d = math.sqrt((self.x - self.last_pose[0])**2 + (self.y - self.last_pose[1])**2)
                self.total_distance += step_d

            if self.shared_costmap is not None:
                ugv_gx = int(math.floor((self.x + 15.0) / 0.20))
                ugv_gy = int(math.floor((self.y + 15.0) / 0.20))
                if 0 <= ugv_gx < 150 and 0 <= ugv_gy < 150:
                    under_val = int(self.shared_costmap[ugv_gy, ugv_gx])
                    if under_val >= 95:
                        self.mine_time += 0.05
                    elif under_val == 80:
                        self.mud_time += 0.05
                    elif under_val == 5:
                        self.road_time += 0.05

        self.last_pose = (self.x, self.y)


def main(args=None):
    rclpy.init(args=args)
    node = UgvControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
