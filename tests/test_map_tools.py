# Occupancy maps are saved and reloaded without losing information (issue #3), and map statistics used
# to show incremental growth are right.
import math

import pytest

import map_tools

# 3 wide x 2 tall; ROS order: bottom row first.
WIDTH, HEIGHT = 3, 2
DATA = [0, 100, -1,
        -1, 0, 100]


def test_grid_stats():
    assert map_tools.grid_stats(DATA) == (4, 2, 2)
    assert map_tools.grid_stats([-1] * 10) == (0, 0, 0)


def test_grid_stats_grow_as_cells_are_observed():
    grid = [-1] * 100
    known = []
    for step in range(5):  # a sensor sweep revealing 10 more cells each update
        for i in range(step * 10, step * 10 + 10):
            grid[i] = 100 if i % 7 == 0 else 0
        known.append(map_tools.grid_stats(grid)[0])
    assert known == [10, 20, 30, 40, 50]


def test_pgm_puts_the_bottom_grid_row_at_the_bottom_of_the_image():
    raw = map_tools.to_pgm(WIDTH, HEIGHT, DATA)
    header, pixels = raw[:raw.index(b'255\n') + 4], raw[raw.index(b'255\n') + 4:]
    assert header == b'P5\n3 2\n255\n'
    # image top row = grid row 1 (unknown, free, occupied); bottom row = grid row 0 (free, occupied, unknown)
    assert list(pixels) == [205, 254, 0, 254, 0, 205]


def test_pgm_round_trip():
    assert map_tools.from_pgm(map_tools.to_pgm(WIDTH, HEIGHT, DATA)) == (WIDTH, HEIGHT, DATA)


def test_uncertain_cells_become_unknown():
    _, _, data = map_tools.from_pgm(map_tools.to_pgm(2, 1, [50, 10]))
    assert data == [-1, 0]


def test_pgm_with_comment_header():
    raw = b'P5\n# CREATOR: map_saver\n2 1\n255\n' + bytes([0, 254])
    assert map_tools.from_pgm(raw) == (2, 1, [100, 0])


def test_bad_pgm_is_rejected():
    with pytest.raises(ValueError):
        map_tools.from_pgm(b'P2\n1 1\n255\n0')
    with pytest.raises(ValueError):
        map_tools.from_pgm(b'P5\n4 4\n255\n' + bytes(3))
    with pytest.raises(ValueError):
        map_tools.to_pgm(2, 2, [0])


def test_save_and_reload(tmp_path):
    yaml_path = map_tools.save(str(tmp_path / 'maps' / 'warehouse'), WIDTH, HEIGHT, DATA, 0.05,
                               (-1.5, 2.25, 0.3))
    assert yaml_path.endswith('warehouse.yaml')
    assert (tmp_path / 'maps' / 'warehouse.pgm').exists()
    width, height, data, resolution, origin = map_tools.load(yaml_path)
    assert (width, height, data) == (WIDTH, HEIGHT, DATA)
    assert resolution == 0.05
    assert origin == pytest.approx((-1.5, 2.25, 0.3))


def test_yaw_from_quaternion():
    assert map_tools.yaw_from_quaternion(0, 0, 0, 1) == 0
    assert map_tools.yaw_from_quaternion(0, 0, math.sin(0.4), math.cos(0.4)) == pytest.approx(0.8)
