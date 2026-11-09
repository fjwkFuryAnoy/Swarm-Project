import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

from tactical_recon.grid_utils import (
    GRID_SIZE,
    CELL_SIZE,
    ORIGIN,
    GOAL_WORLD,
    quaternion_to_yaw,
    world_to_grid,
    in_bounds,
    clamp_grid,
)


class UgvControllerNode(Node):

    def __init__(self):
        super().__init__('ugv_controller_node')

        self.declare_parameter('cooperative_mode', True)
        self.declare_parameter('lookahead_dist', 0.8)
        self.declare_parameter('v0', 2.0)
        self.declare_parameter('max_acc', 1.0)
        self.declare_parameter('max_dec', 2.0)
        self.declare_parameter('rotate_threshold', 0.6)
        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('goal_x', GOAL_WORLD[0])
        self.declare_parameter('goal_y', GOAL_WORLD[1])
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('emergency_stop_dist', 0.35)

        self.cooperative = self.get_parameter('cooperative_mode').value
        self.lookahead_dist = self.get_parameter('lookahead_dist').value
        self.v0 = self.get_parameter('v0').value
        self.max_acc_val = self.get_parameter('max_acc').value
        self.max_dec_val = self.get_parameter('max_dec').value
        self.rotate_threshold = self.get_parameter('rotate_threshold').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.goal = (
            self.get_parameter('goal_x').value,
            self.get_parameter('goal_y').value,
        )
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.emergency_stop_dist = self.get_parameter('emergency_stop_dist').value

        self.x = None
        self.y = None
        self.yaw = None
        self.path = None
        self.last_closest_idx = None
        self.min_forward_dist = float('inf')

        self.shared_costmap = None
        self.lidar_costmap = None

        self.current_v = 0.0
        self.reached_goal = False

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
        self.create_subscription(LaserScan, '/ugv/scan', self.scan_cb, 10)
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

    def scan_cb(self, msg):
        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        if not np.any(valid):
            self.min_forward_dist = float('inf')
            return
        valid_ranges = ranges[valid]
        valid_angles = angles[valid]
        forward_mask = np.abs(valid_angles) < 0.52
        if np.any(forward_mask):
            self.min_forward_dist = float(np.min(valid_ranges[forward_mask]))
        else:
            self.min_forward_dist = float('inf')

    def path_cb(self, msg):
        if len(msg.poses) > 0:
            self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            self.last_closest_idx = 0
        else:
            self.path = None
            self.last_closest_idx = None

    def shared_costmap_cb(self, msg):
        self.shared_costmap = np.array(msg.data, dtype=np.int8).reshape((GRID_SIZE, GRID_SIZE))

    def lidar_costmap_cb(self, msg):
        self.lidar_costmap = np.array(msg.data, dtype=np.int8).reshape((GRID_SIZE, GRID_SIZE))

    def control_loop(self):
        cmd = Twist()

        if self.x is None or self.yaw is None:
            self.cmd_pub.publish(cmd)
            return

        dist_to_goal = math.sqrt((self.x - self.goal[0])**2 + (self.y - self.goal[1])**2)

        if dist_to_goal <= self.goal_tolerance:
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
            self.current_v = max(0.0, self.current_v - self.max_dec_val * 0.05)
            cmd.linear.x = self.current_v
            self.cmd_pub.publish(cmd)
            return

        active_costmap = self.shared_costmap if self.cooperative else self.lidar_costmap
        if active_costmap is None:
            self.cmd_pub.publish(cmd)
            return

        n_pts = len(self.path)
        if self.last_closest_idx is None or self.last_closest_idx >= n_pts:
            search_start = 0
            search_end = n_pts
        else:
            search_start = max(0, self.last_closest_idx - 5)
            search_end = min(n_pts, self.last_closest_idx + 25)

        closest_idx = search_start
        min_dist = float('inf')
        for i in range(search_start, search_end):
            px, py = self.path[i]
            d = math.sqrt((self.x - px)**2 + (self.y - py)**2)
            if d < min_dist:
                min_dist = d
                closest_idx = i
        self.last_closest_idx = closest_idx

        target_idx = closest_idx
        for i in range(closest_idx, n_pts):
            d = math.sqrt((self.x - self.path[i][0])**2 + (self.y - self.path[i][1])**2)
            if d >= self.lookahead_dist:
                target_idx = i
                break
        else:
            target_idx = n_pts - 1

        tx, ty = self.path[target_idx]

        dx = tx - self.x
        dy = ty - self.y
        target_yaw = math.atan2(dy, dx)
        alpha = target_yaw - self.yaw
        alpha = (alpha + math.pi) % (2.0 * math.pi) - math.pi

        max_c = 0
        for i in range(closest_idx, n_pts):
            d = math.sqrt((self.x - self.path[i][0])**2 + (self.y - self.path[i][1])**2)
            if d <= 1.5:
                gx, gy = world_to_grid(self.path[i][0], self.path[i][1])
                gx, gy = clamp_grid(gx, gy)
                cell = int(active_costmap[gy, gx])
                if cell == -1:
                    cell = 30
                cell = min(cell, 90)
                if cell > max_c:
                    max_c = cell
            else:
                break

        v_terrain = self.v0 / (1.0 + max_c / 10.0)
        v_terrain = max(0.15, v_terrain)

        v_curv = v_terrain * max(0.25, math.cos(alpha))

        if self.min_forward_dist < self.emergency_stop_dist:
            target_v = 0.0
            target_omega = 0.0
        elif abs(alpha) > self.rotate_threshold:
            target_v = 0.0
            target_omega = 0.8 if alpha > 0 else -0.8
        else:
            target_v = v_curv
            target_omega = 2.0 * target_v * math.sin(alpha) / self.lookahead_dist
            target_omega = max(-self.max_angular_vel, min(self.max_angular_vel, target_omega))

        max_acc = self.max_acc_val * 0.05
        max_dec = self.max_dec_val * 0.05

        if target_v > self.current_v:
            self.current_v = min(target_v, self.current_v + max_acc)
        else:
            self.current_v = max(target_v, self.current_v - max_dec)

        if abs(alpha) > self.rotate_threshold and self.current_v > 0.05:
            cmd.linear.x = self.current_v
            cmd.angular.z = target_omega * 0.5
        elif abs(alpha) > self.rotate_threshold:
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
                ugv_gx, ugv_gy = world_to_grid(self.x, self.y)
                if in_bounds(ugv_gx, ugv_gy):
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
