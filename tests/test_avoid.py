import math

import numpy as np
import pytest

from go2_nl_control.perception import PerceptionConfig
from voice_ltl.avoid import AvoidConfig, choose_heading, free_length, obstacle_points


def square(cx, cy, size=0.45, step=0.05):
    """軸に平行な正方形の物の 4 辺の点（機体座標の平面）。"""
    h = size / 2
    edge = np.arange(-h, h + 1e-9, step)
    pts = ([(cx + e, cy - h) for e in edge] + [(cx + e, cy + h) for e in edge]
           + [(cx - h, cy + e) for e in edge] + [(cx + h, cy + e) for e in edge])
    return np.array(pts)


def test_free_length_measures_to_the_front_edge():
    chair = square(2.0, 0.0)
    assert free_length(chair, 0.0) == pytest.approx(2.0 - 0.225 - 0.35)
    assert free_length(chair, math.radians(90)) == math.inf
    assert free_length(np.zeros((0, 2)), 0.0) == math.inf


def test_goes_straight_when_the_way_is_clear_or_the_obstacle_is_beyond_the_goal():
    chair = square(2.0, 0.0)
    assert choose_heading(None, 0.3, 3.0) == (0.3, math.inf, 0)
    heading, _, side = choose_heading(chair, math.radians(60), 3.0)
    assert heading == pytest.approx(math.radians(60)) and side == 0
    # リモコンが椅子の手前なら、椅子は気にしない
    assert choose_heading(chair, 0.0, 0.8)[0] == 0.0


def test_detours_around_an_obstacle_on_the_way_and_keeps_the_side():
    chair = square(1.5, 0.1)     # 少し左寄りの椅子 → 右へよける
    heading, free, side = choose_heading(chair, 0.0, 3.0)
    assert heading < 0 and side == -1 and free >= 0.8
    assert free_length(chair, heading) >= 0.8
    # 左へよけ始めていたら、少し遠回りでも左を続ける
    heading_left, _, side_left = choose_heading(chair, 0.0, 3.0, side=1)
    assert heading_left > 0 and side_left == 1


def test_no_way_through_a_wall():
    wall = np.array([(0.6, y) for y in np.arange(-3, 3, 0.05)]
                    + [(x, 0.5) for x in np.arange(-3, 0.6, 0.05)]
                    + [(x, -0.5) for x in np.arange(-3, 0.6, 0.05)])
    heading, free, _ = choose_heading(wall, 0.0, 3.0, cfg=AvoidConfig(max_detour=math.radians(60)))
    assert heading is None and free < 0.3


def test_obstacle_points_keep_obstacle_heights_and_drop_own_body():
    cfg = PerceptionConfig()   # シミュレーション：LiDAR は機体の 0.28 m 前・0.10 m 上
    lidar = np.array([[1.0, 0.0, 0.0],      # 前の物
                      [1.0, 0.0, -0.5],     # 地面より下
                      [0.0, 0.0, 0.0],      # 自分の体（機体座標 x = 0.28）
                      [np.nan, 0, 0]])
    pts = obstacle_points(lidar, cfg)
    assert pts.shape == (1, 2) and pts[0] == pytest.approx([1.3, 0.0])
