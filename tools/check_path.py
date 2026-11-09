import sys
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np


class PathChecker(Node):

    def __init__(self):
        super().__init__('check_path')

        self.path_msg = None
        self.costmap_msg = None

        costmap_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(Path, '/ugv/planned_path', self.path_cb, 10)
        self.create_subscription(OccupancyGrid, '/shared_costmap',
                                 self.costmap_cb, costmap_qos)

    def path_cb(self, msg):
        self.path_msg = msg

    def costmap_cb(self, msg):
        self.costmap_msg = msg


def main():
    rclpy.init()
    node = PathChecker()

    timeout_sec = 15.0
    elapsed = 0.0
    rate = 0.1

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=rate)
        if node.path_msg is not None and node.costmap_msg is not None:
            break
        elapsed += rate
        if elapsed > timeout_sec:
            print("ERROR: Timed out waiting for path and costmap")
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

    path = node.path_msg
    costmap = np.array(node.costmap_msg.data, dtype=np.int8).reshape((150, 150))

    pose_count = len(path.poses)
    if pose_count == 0:
        print("ERROR: Path is empty")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    total_length = 0.0
    for i in range(1, pose_count):
        dx = path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x
        dy = path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y
        total_length += math.sqrt(dx * dx + dy * dy)

    on_hazard = 0
    on_road = 0
    for pose in path.poses:
        wx = pose.pose.position.x
        wy = pose.pose.position.y
        gx = int(math.floor((wx + 15.0) / 0.20))
        gy = int(math.floor((wy + 15.0) / 0.20))
        gx = max(0, min(149, gx))
        gy = max(0, min(149, gy))
        cell = int(costmap[gy, gx])
        if cell >= 95:
            on_hazard += 1
        if cell == 5:
            on_road += 1

    road_pct = 100.0 * on_road / pose_count if pose_count > 0 else 0.0

    print("=== PATH CHECK ===")
    print(f"Pose count: {pose_count}")
    print(f"Path length: {total_length:.2f} m")
    print(f"Poses on cost >= 95: {on_hazard}")
    print(f"Poses on road (cost 5): {on_road} ({road_pct:.1f}%)")

    start_x = path.poses[0].pose.position.x
    start_y = path.poses[0].pose.position.y
    end_x = path.poses[-1].pose.position.x
    end_y = path.poses[-1].pose.position.y
    print(f"Start: ({start_x:.2f}, {start_y:.2f})")
    print(f"End:   ({end_x:.2f}, {end_y:.2f})")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

