import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
import numpy as np


class CostmapChecker(Node):

    def __init__(self):
        super().__init__('check_costmap')

        self.shared_msg = None
        self.lidar_msg = None

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(OccupancyGrid, '/shared_costmap', self.shared_cb, qos)
        self.create_subscription(OccupancyGrid, '/lidar_costmap', self.lidar_cb, qos)

    def shared_cb(self, msg):
        self.shared_msg = msg

    def lidar_cb(self, msg):
        self.lidar_msg = msg


def main():
    rclpy.init()
    node = CostmapChecker()

    timeout_sec = 10.0
    elapsed = 0.0
    rate = 0.1

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=rate)
        if node.shared_msg is not None and node.lidar_msg is not None:
            break
        elapsed += rate
        if elapsed > timeout_sec:
            print("ERROR: Timed out waiting for costmaps")
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

    shared = np.array(node.shared_msg.data, dtype=np.int8)
    lidar = np.array(node.lidar_msg.data, dtype=np.int8)

    np.save('/tmp/shared_costmap.npy', shared)
    np.save('/tmp/lidar_costmap.npy', lidar)

    terrain_only = shared[lidar != 100]

    n_mine = int(np.count_nonzero(terrain_only == 100))
    n_mud = int(np.count_nonzero(terrain_only == 80))
    n_road = int(np.count_nonzero(terrain_only == 5))
    n_dirt = int(np.count_nonzero(terrain_only == 30))
    n_unknown = int(np.count_nonzero(terrain_only == -1))
    n_lidar = int(np.count_nonzero(lidar == 100))

    mine_ok = 650 <= n_mine <= 800
    mud_ok = 360 <= n_mud <= 440
    road_ok = 2150 <= n_road <= 2550

    print("=== COSTMAP CELL COUNTS ===")
    print(f"Total cells: {len(shared)}")
    print(f"LiDAR wall/block cells: {n_lidar}")
    print(f"Mines (100, inflated): {n_mine} (expected 650-800) -> {'PASS' if mine_ok else 'FAIL'}")
    print(f"Mud (80):               {n_mud} (expected 360-440) -> {'PASS' if mud_ok else 'FAIL'}")
    print(f"Road (5):              {n_road} (expected 2150-2550) -> {'PASS' if road_ok else 'FAIL'}")
    print(f"Dirt (30):             {n_dirt}")
    print(f"Unknown (-1):          {n_unknown}")

    all_passed = mine_ok and mud_ok and road_ok
    print(f"Overall result: {'PASS' if all_passed else 'FAIL'}")

    node.destroy_node()
    rclpy.shutdown()

    if not all_passed:
        sys.exit(1)


if __name__ == '__main__':
    main()
