import math

import numpy as np

from go2_nl_control.perception import PerceptionConfig
from go2_nl_control.supervisor import BLOCKED, CLEAR, SENSOR_LOST
from voice_ltl.sensing import Sensing


def box_points(cx, cfg):
    """機体座標で x=cx にある板を、LiDAR 座標に戻した点群（perception._to_body の逆変換）。"""
    ys = np.linspace(-0.2, 0.2, 5)
    zs = np.linspace(0.0, 0.2, 4)
    body = np.array([[cx, y, z] for y in ys for z in zs])
    body[:, 0] -= cfg.lidar_x
    body[:, 2] -= cfg.lidar_z
    cp, sp = math.cos(cfg.lidar_pitch), math.sin(cfg.lidar_pitch)
    cy, sy = math.cos(cfg.lidar_yaw), math.sin(cfg.lidar_yaw)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return body @ (rz @ ry)   # (R^-1 p) as row vectors


def test_front_and_back_hysteresis():
    cfg = PerceptionConfig(); s = Sensing(cfg)
    assert s.update(box_points(0.45, cfg), 0.0).front == BLOCKED
    assert s.update(box_points(0.60, cfg), 0.1).front == BLOCKED      # 0.7 m までは戻らない
    assert s.update(box_points(0.75, cfg), 0.2).front == CLEAR
    r = s.update(box_points(-0.45, cfg), 0.3)
    assert r.back == BLOCKED and r.front == CLEAR and abs(r.back_dist - 0.45) < 0.03


def test_stale_is_lost_and_counts_as_blocked():
    s = Sensing(PerceptionConfig()); s.update(np.zeros((0, 3)), 0.0)
    r = s.current(0.6)
    assert r.front == SENSOR_LOST and r.front_blocked and r.back_blocked
