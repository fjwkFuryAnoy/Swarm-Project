import math
from tactical_recon.grid_utils import (
    GRID_SIZE,
    CELL_SIZE,
    ORIGIN,
    GOAL_WORLD,
    GOAL_CELL,
    quaternion_to_yaw,
    world_to_grid,
    grid_to_world,
    in_bounds,
    clamp_grid,
)


class DummyQuaternion:

    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w


def test_grid_constants():
    assert GRID_SIZE == 150
    assert abs(CELL_SIZE - 0.20) < 1e-6
    assert abs(ORIGIN - (-15.0)) < 1e-6
    assert GOAL_WORLD == (12.0, 12.0)
    assert GOAL_CELL == (135, 135)


def test_world_to_grid_and_back():
    gx, gy = world_to_grid(-15.0, -15.0)
    assert gx == 0 and gy == 0
    gx, gy = world_to_grid(12.0, 12.0)
    assert gx == 135 and gy == 135

    wx, wy = grid_to_world(0, 0)
    assert abs(wx - (-14.9)) < 1e-4
    assert abs(wy - (-14.9)) < 1e-4

    test_cells = [(0, 0), (10, 20), (75, 75), (135, 135), (149, 149)]
    for c_gx, c_gy in test_cells:
        wx, wy = grid_to_world(c_gx, c_gy)
        rgx, rgy = world_to_grid(wx, wy)
        assert rgx == c_gx and rgy == c_gy


def test_in_bounds_and_clamp():
    assert in_bounds(0, 0)
    assert in_bounds(149, 149)
    assert not in_bounds(-1, 0)
    assert not in_bounds(0, 150)
    assert not in_bounds(150, 150)

    assert clamp_grid(-5, 200) == (0, 149)
    assert clamp_grid(50, 60) == (50, 60)


def test_quaternion_to_yaw():
    q_zero = DummyQuaternion(0.0, 0.0, 0.0, 1.0)
    assert abs(quaternion_to_yaw(q_zero)) < 1e-6

    q_half_pi = DummyQuaternion(0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
    assert abs(quaternion_to_yaw(q_half_pi) - math.pi / 2.0) < 1e-5
