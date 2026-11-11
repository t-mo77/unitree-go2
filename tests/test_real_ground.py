import numpy as np

from go2_nl_control.perception import ObstacleMonitor, PerceptionConfig, remove_ground
from go2_nl_control.supervisor import BLOCKED, CLEAR
from voice_ltl.perception_config import perception_config


def floor():
    x, y = np.meshgrid(np.linspace(-1.5, 1.5, 41), np.linspace(-1.5, 1.5, 41))
    return np.column_stack([x.ravel(), y.ravel(), -.32 + .02 * x.ravel()])


def test_floor_is_removed_but_close_box_is_still_detected():
    cfg = PerceptionConfig(lidar_x=0, lidar_z=0, remove_ground=True)
    points = floor()
    monitor = ObstacleMonitor(cfg)
    assert monitor.update(points, 0) == CLEAR
    ys, zs = np.meshgrid(np.linspace(-.15, .15, 6), np.linspace(-.1, .2, 6))
    box = np.column_stack([np.full(ys.size, .45), ys.ravel(), zs.ravel()])
    assert monitor.update(np.vstack([points, box]), .1) == BLOCKED
    assert monitor.nearest == .45


def test_small_objects_and_cable_points_are_not_discarded_as_floor():
    cable = np.array([[.32, .28, -.18 + i * .002] for i in range(12)])
    filtered = remove_ground(np.vstack([floor(), cable]))
    for point in cable:
        assert np.any(np.all(np.isclose(filtered, point), axis=1))


def test_wall_and_small_low_platform_are_not_floor():
    y, z = np.meshgrid(np.linspace(-1, 1, 41), np.linspace(-.5, .5, 41))
    wall = np.column_stack([np.full(y.size, .4), y.ravel(), z.ravel()])
    assert len(remove_ground(wall)) == len(wall)
    x, y = np.meshgrid(np.linspace(.2, .5, 20), np.linspace(-.15, .15, 20))
    platform = np.column_stack([x.ravel(), y.ravel(), np.full(x.size, -.25)])
    assert len(remove_ground(platform)) == len(platform)


def test_simulation_keeps_its_config_and_real_uses_separate_mount():
    assert perception_config({}) == PerceptionConfig()
    cfg = perception_config({"GO2_LIDAR_PROFILE": "go2"})
    assert cfg.remove_ground and cfg.lidar_pitch == 2.8782
