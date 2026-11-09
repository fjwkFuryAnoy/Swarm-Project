import heapq
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import numpy as np

from tactical_recon.grid_utils import (
    GRID_SIZE,
    GOAL_CELL,
    GOAL_WORLD,
    world_to_grid,
    grid_to_world,
    clamp_grid,
)

SQRT2 = math.sqrt(2.0)
NEIGHBORS = [
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2),
]


def heuristic(a, b):
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return max(dx, dy) + (SQRT2 - 1.0) * min(dx, dy)


def astar(grid, start, goal, start_immunity_radius=3):
    if start[0] < 0 or start[0] >= GRID_SIZE or start[1] < 0 or start[1] >= GRID_SIZE:
        return None

    open_set = []
    g_score = {}
    came_from = {}

    g_score[start] = 0.0
    f_start = heuristic(start, goal)
    heapq.heappush(open_set, (f_start, start))

    closed = set()

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            node = goal
            while node in came_from:
                path.append(node)
                node = came_from[node]
            path.append(start)
            path.reverse()
            return path

        if current in closed:
            continue
        closed.add(current)

        cx, cy = current

        for dx, dy, base_dist in NEIGHBORS:
            nx, ny = cx + dx, cy + dy

            if nx < 0 or nx >= GRID_SIZE or ny < 0 or ny >= GRID_SIZE:
                continue

            if (nx, ny) in closed:
                continue

            cell_cost = int(grid[ny, nx])
            if cell_cost == -1:
                cell_cost = 30

            is_near_start = (abs(nx - start[0]) <= start_immunity_radius and
                             abs(ny - start[1]) <= start_immunity_radius)

            if cell_cost >= 95 and not is_near_start:
                continue

            if dx != 0 and dy != 0:
                adj1_cost = int(grid[cy, cx + dx])
                adj2_cost = int(grid[cy + dy, cx])
                if adj1_cost == -1:
                    adj1_cost = 30
                if adj2_cost == -1:
                    adj2_cost = 30

                adj1_near = (abs(cx + dx - start[0]) <= start_immunity_radius and
                             abs(cy - start[1]) <= start_immunity_radius)
                adj2_near = (abs(cx - start[0]) <= start_immunity_radius and
                             abs(cy + dy - start[1]) <= start_immunity_radius)

                if (adj1_cost >= 95 and not adj1_near) or (adj2_cost >= 95 and not adj2_near):
                    continue

            effective_cost = min(cell_cost, 90) if is_near_start else cell_cost
            step = base_dist * (1.0 + effective_cost / 10.0)
            tentative_g = g_score[current] + step

            if tentative_g < g_score.get((nx, ny), float('inf')):
                came_from[(nx, ny)] = current
                g_score[(nx, ny)] = tentative_g
                f = tentative_g + heuristic((nx, ny), goal)
                heapq.heappush(open_set, (f, (nx, ny)))

    return None


class GlobalPlannerNode(Node):

    def __init__(self):
        super().__init__('global_planner_node')

        self.declare_parameter('cooperative_mode', True)
        self.declare_parameter('wait_for_patrol', True)

        self.cooperative = self.get_parameter('cooperative_mode').value
        self.wait_for_patrol = self.get_parameter('wait_for_patrol').value

        self.costmap = None
        self.ugv_x = None
        self.ugv_y = None
        self.patrol_done = False
        self.last_warn_time = 0.0

        costmap_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)

        if self.cooperative:
            self.create_subscription(OccupancyGrid, '/shared_costmap',
                                     self.costmap_cb, costmap_qos)
        else:
            self.create_subscription(OccupancyGrid, '/lidar_costmap',
                                     self.costmap_cb, costmap_qos)

        self.create_subscription(Odometry, '/ugv/odom', self.odom_cb, 10)

        done_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, '/uav/patrol_done',
                                 self.patrol_done_cb, done_qos)

        self.path_pub = self.create_publisher(Path, '/ugv/planned_path', 10)

        marker_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.marker_pub = self.create_publisher(Marker, '/ugv/goal_marker', marker_qos)
        self.publish_goal_marker()

        self.create_timer(0.5, self.plan_timer)

    def publish_goal_marker(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'goal'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = GOAL_WORLD[0]
        marker.pose.position.y = GOAL_WORLD[1]
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.8
        marker.scale.y = 0.8
        marker.scale.z = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.color.a = 0.8
        self.marker_pub.publish(marker)

    def costmap_cb(self, msg):
        self.costmap = np.array(msg.data, dtype=np.int8).reshape((GRID_SIZE, GRID_SIZE))

    def odom_cb(self, msg):
        self.ugv_x = msg.pose.pose.position.x
        self.ugv_y = msg.pose.pose.position.y

    def patrol_done_cb(self, msg):
        if msg.data:
            self.patrol_done = True

    def plan_timer(self):
        self.publish_goal_marker()

        if self.wait_for_patrol and not self.patrol_done:
            return

        if self.costmap is None or self.ugv_x is None:
            return

        start_gx, start_gy = world_to_grid(self.ugv_x, self.ugv_y)
        start_gx, start_gy = clamp_grid(start_gx, start_gy)
        start = (start_gx, start_gy)

        path_cells = astar(self.costmap, start, GOAL_CELL)

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        if path_cells is None:
            now_sec = self.get_clock().now().nanoseconds / 1e9
            if now_sec - self.last_warn_time >= 1.0:
                self.get_logger().warn('No path found')
                self.last_warn_time = now_sec
            self.path_pub.publish(path_msg)
            return

        for gx, gy in path_cells:
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = 'map'
            wx, wy = grid_to_world(gx, gy)
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
