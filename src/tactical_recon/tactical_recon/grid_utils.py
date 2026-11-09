import math


GRID_SIZE = 150
CELL_SIZE = 0.20
ORIGIN = -15.0
GOAL_WORLD = (12.0, 12.0)
GOAL_CELL = (135, 135)


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def world_to_grid(x, y):
    gx = int(math.floor((x - ORIGIN) / CELL_SIZE))
    gy = int(math.floor((y - ORIGIN) / CELL_SIZE))
    return gx, gy


def grid_to_world(gx, gy):
    x = ORIGIN + (gx + 0.5) * CELL_SIZE
    y = ORIGIN + (gy + 0.5) * CELL_SIZE
    return x, y


def in_bounds(gx, gy):
    return 0 <= gx < GRID_SIZE and 0 <= gy < GRID_SIZE


def clamp_grid(gx, gy):
    return max(0, min(GRID_SIZE - 1, gx)), max(0, min(GRID_SIZE - 1, gy))
