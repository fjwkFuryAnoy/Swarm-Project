import numpy as np
from tactical_recon.global_planner_node import astar
from tactical_recon.grid_utils import GRID_SIZE


def test_astar_open_grid():
    grid = np.full((GRID_SIZE, GRID_SIZE), 30, dtype=np.int8)
    start = (10, 10)
    goal = (20, 20)
    path = astar(grid, start, goal, start_immunity_radius=0)
    assert path is not None
    assert path[0] == start
    assert path[-1] == goal
    assert len(path) == 11


def test_astar_avoids_obstacle():
    grid = np.full((GRID_SIZE, GRID_SIZE), 30, dtype=np.int8)
    for y in range(8, 15):
        grid[y, 15] = 100

    start = (10, 10)
    goal = (20, 10)
    path = astar(grid, start, goal, start_immunity_radius=0)
    assert path is not None
    assert path[0] == start
    assert path[-1] == goal
    for x, y in path:
        assert grid[y, x] < 95


def test_astar_corner_cutting_prevention():
    grid = np.full((GRID_SIZE, GRID_SIZE), 30, dtype=np.int8)
    grid[10, 11] = 100

    start = (10, 10)
    goal = (11, 11)
    path = astar(grid, start, goal, start_immunity_radius=0)
    assert path is not None
    assert path[0] == start
    assert path[-1] == goal
    assert (11, 11) not in path[1:2] or path[1] != (11, 11)


def test_astar_blocked_goal():
    grid = np.full((GRID_SIZE, GRID_SIZE), 30, dtype=np.int8)
    for x in range(GRID_SIZE):
        grid[50, x] = 100

    start = (10, 10)
    goal = (10, 80)
    path = astar(grid, start, goal, start_immunity_radius=0)
    assert path is None


def test_astar_out_of_bounds_start():
    grid = np.full((GRID_SIZE, GRID_SIZE), 30, dtype=np.int8)
    assert astar(grid, (-1, 10), (20, 20)) is None
    assert astar(grid, (150, 10), (20, 20)) is None
