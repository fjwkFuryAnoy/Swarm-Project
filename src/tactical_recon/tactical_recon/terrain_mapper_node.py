import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Image, LaserScan
import numpy as np
import cv2
from cv_bridge import CvBridge

from tactical_recon.grid_utils import (
    GRID_SIZE,
    CELL_SIZE,
    ORIGIN,
    quaternion_to_yaw,
)


def interpolate_pose(queue, t):
    if not queue:
        return None, None, None

    if len(queue) == 1 or t <= queue[0][0]:
        return queue[0][1], queue[0][2], queue[0][3]

    if t >= queue[-1][0]:
        return queue[-1][1], queue[-1][2], queue[-1][3]

    for i in range(len(queue) - 1):
        t1, x1, y1, yaw1 = queue[i]
        t2, x2, y2, yaw2 = queue[i + 1]
        if t1 <= t <= t2:
            if t2 == t1:
                return x1, y1, yaw1
            alpha = (t - t1) / (t2 - t1)
            x = x1 + alpha * (x2 - x1)
            y = y1 + alpha * (y2 - y1)
            dyaw = (yaw2 - yaw1 + math.pi) % (2.0 * math.pi) - math.pi
            yaw = yaw1 + alpha * dyaw
            yaw = (yaw + math.pi) % (2.0 * math.pi) - math.pi
            return x, y, yaw

    return queue[-1][1], queue[-1][2], queue[-1][3]


class TerrainMapperNode(Node):

    def __init__(self):
        super().__init__('terrain_mapper_node')

        self.terrain = np.full((GRID_SIZE, GRID_SIZE), -1, dtype=np.int8)
        self.lidar_raw = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.uint8)

        self.dirt_hits = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.int32)
        self.green_hits = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.int32)
        self.yellow_hits = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.int32)
        self.red_hits = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.int32)

        self.uav_odom_queue = []
        self.ugv_odom_queue = []
        self.image_queue = []

        self.frame_count = 0
        self.bridge = CvBridge()

        u = np.arange(4, 640, 8)
        v = np.arange(4, 480, 8)
        uu, vv = np.meshgrid(u, v)
        self.dx_body = -(vv - 240.0) * (12.0 / 554.256)
        self.dy_body = -(uu - 320.0) * (12.0 / 554.256)

        self.create_subscription(Odometry, '/uav/odom', self.uav_odom_cb, 10)
        self.create_subscription(Odometry, '/ugv/odom', self.ugv_odom_cb, 10)
        self.create_subscription(LaserScan, '/ugv/scan', self.scan_cb, 10)
        self.create_subscription(Image, '/uav/camera/image_raw', self.camera_cb, 10)

        costmap_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)

        self.shared_pub = self.create_publisher(OccupancyGrid, '/shared_costmap', costmap_qos)
        self.lidar_pub = self.create_publisher(OccupancyGrid, '/lidar_costmap', costmap_qos)
        self.debug_pub = self.create_publisher(Image, '/uav/debug_segmentation', 10)

        self.create_timer(0.5, self.publish_costmaps)

    def uav_odom_cb(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.uav_odom_queue.append((stamp, x, y, yaw))
        if len(self.uav_odom_queue) > 200:
            self.uav_odom_queue.pop(0)

        while self.image_queue:
            img_msg = self.image_queue[0]
            img_time = img_msg.header.stamp.sec + img_msg.header.stamp.nanosec * 1e-9
            if self.uav_odom_queue[-1][0] >= img_time:
                self.image_queue.pop(0)
                self.process_image(img_msg, img_time)
            else:
                break

    def ugv_odom_cb(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.ugv_odom_queue.append((stamp, x, y, yaw))
        if len(self.ugv_odom_queue) > 200:
            self.ugv_odom_queue.pop(0)

    def scan_cb(self, msg):
        scan_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        ugv_x, ugv_y, ugv_yaw = interpolate_pose(self.ugv_odom_queue, scan_time)
        if ugv_x is None or ugv_yaw is None:
            return

        ranges = np.array(msg.ranges)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        valid = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
        if not np.any(valid):
            return

        valid_ranges = ranges[valid]
        valid_angles = angles[valid]

        beam_angles = ugv_yaw + valid_angles
        lx = ugv_x + valid_ranges * np.cos(beam_angles)
        ly = ugv_y + valid_ranges * np.sin(beam_angles)

        gx = np.floor((lx - ORIGIN) / CELL_SIZE).astype(int)
        gy = np.floor((ly - ORIGIN) / CELL_SIZE).astype(int)

        in_bounds_mask = (gx >= 0) & (gx < GRID_SIZE) & (gy >= 0) & (gy < GRID_SIZE)
        if np.any(in_bounds_mask):
            self.lidar_raw[gy[in_bounds_mask], gx[in_bounds_mask]] = 1

    def camera_cb(self, msg):
        img_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.uav_odom_queue and self.uav_odom_queue[-1][0] >= img_time:
            self.process_image(msg, img_time)
        else:
            self.image_queue.append(msg)
            if len(self.image_queue) > 20:
                self.image_queue.pop(0)

    def process_image(self, msg, img_time):
        uav_x, uav_y, uav_yaw = interpolate_pose(self.uav_odom_queue, img_time)
        if uav_x is None:
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception:
            return

        hsv = cv2.cvtColor(cv_img, cv2.COLOR_RGB2HSV)
        sampled_hsv = hsv[4::8, 4::8]

        h = sampled_hsv[:, :, 0]
        s = sampled_hsv[:, :, 1]
        v_chan = sampled_hsv[:, :, 2]

        is_red = (((h <= 10) | (h >= 170)) & (s >= 100) & (v_chan >= 80))
        is_yellow = ((h >= 18) & (h <= 34) & (s >= 100) & (v_chan >= 80))
        is_green = ((h >= 35) & (h <= 85) & (s >= 80) & (v_chan >= 50))
        is_dirt = (~is_red) & (~is_yellow) & (~is_green)

        cos_t = np.cos(uav_yaw)
        sin_t = np.sin(uav_yaw)
        xw = uav_x + self.dx_body * cos_t - self.dy_body * sin_t
        yw = uav_y + self.dx_body * sin_t + self.dy_body * cos_t

        gx = np.floor((xw - ORIGIN) / CELL_SIZE).astype(int)
        gy = np.floor((yw - ORIGIN) / CELL_SIZE).astype(int)

        valid = (gx >= 0) & (gx < GRID_SIZE) & (gy >= 0) & (gy < GRID_SIZE)

        val_gx = gx[valid]
        val_gy = gy[valid]
        val_red = is_red[valid]
        val_yellow = is_yellow[valid]
        val_green = is_green[valid]
        val_dirt = is_dirt[valid]

        np.add.at(self.dirt_hits, (val_gy[val_dirt], val_gx[val_dirt]), 1)
        np.add.at(self.green_hits, (val_gy[val_green], val_gx[val_green]), 1)
        np.add.at(self.yellow_hits, (val_gy[val_yellow], val_gx[val_yellow]), 1)
        np.add.at(self.red_hits, (val_gy[val_red], val_gx[val_red]), 1)

        total_hits = self.dirt_hits + self.green_hits + self.yellow_hits + self.red_hits
        observed = total_hits > 0

        self.terrain[observed] = 30
        self.terrain[observed & (self.green_hits > self.dirt_hits) & (self.green_hits > 0)] = 5
        self.terrain[observed & (self.yellow_hits > self.dirt_hits) & (self.yellow_hits > 0)] = 80
        self.terrain[observed & (self.red_hits > self.dirt_hits) & (self.red_hits > 0)] = 100

        self.frame_count += 1
        if self.frame_count % 2 == 0:
            self.publish_debug(cv_img, is_red, is_yellow, is_green, is_dirt, msg.header, uav_x, uav_y, uav_yaw)

    def publish_debug(self, cv_img, is_red, is_yellow, is_green, is_dirt, header, uav_x, uav_y, uav_yaw):
        tint = np.zeros((60, 80, 3), dtype=np.uint8)
        tint[is_red] = [255, 0, 0]
        tint[is_yellow] = [255, 255, 0]
        tint[is_green] = [0, 255, 0]
        tint[is_dirt] = [128, 128, 128]

        tint_full = cv2.resize(tint, (640, 480), interpolation=cv2.INTER_NEAREST)
        debug_img = cv2.addWeighted(cv_img, 0.6, tint_full, 0.4, 0)

        n_road = int(np.count_nonzero(self.terrain == 5))
        n_mud = int(np.count_nonzero(self.terrain == 80))
        n_mine = int(np.count_nonzero(self.terrain == 100))
        n_dirt = int(np.count_nonzero(self.terrain == 30))

        line1 = f"UAV: x={uav_x:.1f} y={uav_y:.1f} yaw={uav_yaw:.2f}"
        line2 = f"Road:{n_road} Mud:{n_mud} Mine:{n_mine} Dirt:{n_dirt}"

        cv2.putText(debug_img, line1, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
        cv2.putText(debug_img, line1, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(debug_img, line2, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
        cv2.putText(debug_img, line2, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='rgb8')
        debug_msg.header = header
        self.debug_pub.publish(debug_msg)

    def publish_costmaps(self):
        now = self.get_clock().now().to_msg()

        kernel = np.ones((5, 5), dtype=np.uint8)
        lidar_inflated = cv2.dilate(self.lidar_raw, kernel)

        lidar_grid = np.full((GRID_SIZE, GRID_SIZE), 30, dtype=np.int8)
        lidar_grid[lidar_inflated > 0] = 100

        lidar_msg = OccupancyGrid()
        lidar_msg.header.stamp = now
        lidar_msg.header.frame_id = 'map'
        lidar_msg.info.resolution = CELL_SIZE
        lidar_msg.info.width = GRID_SIZE
        lidar_msg.info.height = GRID_SIZE
        lidar_msg.info.origin.position.x = ORIGIN
        lidar_msg.info.origin.position.y = ORIGIN
        lidar_msg.info.origin.position.z = 0.0
        lidar_msg.info.origin.orientation.w = 1.0
        lidar_msg.data = lidar_grid.flatten().tolist()
        self.lidar_pub.publish(lidar_msg)

        shared_grid = self.terrain.copy()
        red_mask = (self.terrain == 100).astype(np.uint8)
        red_dilated = cv2.dilate(red_mask, kernel)
        shared_grid[red_dilated > 0] = 100
        shared_grid[lidar_inflated > 0] = 100

        shared_msg = OccupancyGrid()
        shared_msg.header.stamp = now
        shared_msg.header.frame_id = 'map'
        shared_msg.info.resolution = CELL_SIZE
        shared_msg.info.width = GRID_SIZE
        shared_msg.info.height = GRID_SIZE
        shared_msg.info.origin.position.x = ORIGIN
        shared_msg.info.origin.position.y = ORIGIN
        shared_msg.info.origin.position.z = 0.0
        shared_msg.info.origin.orientation.w = 1.0
        shared_msg.data = shared_grid.flatten().tolist()
        self.shared_pub.publish(shared_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TerrainMapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
