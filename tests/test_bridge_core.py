import math
import re
from pathlib import Path

import numpy as np
import pytest

from go2_nl_control.actions import (BALANCE_STAND, HELLO, MOVE, SIT, SITTING, STAND_UP,
                                    STOP_MOVE, state_from_error_code)
from go2_nl_control.perception import PerceptionConfig
from gazebo.bridge_core import (DOWN_CMD, LIDAR_IN_BODY, STAND_CMD, TROT_CMD, BoxWorld,
                                BridgeCore, drop_self_points, move_input)
from sim.sim_core import FOOTPRINT

XACRO = Path(__file__).resolve().parent.parent / "gazebo" / "go2_sensors.urdf.xacro"


def standing_core() -> BridgeCore:
    b = BridgeCore(0.0)
    b.handle(STAND_UP, "", 0.0)
    for t in (0.0, 0.3, 2.1):
        b.step(t)
    b.handle(BALANCE_STAND, "", 2.1)
    for t in (2.1, 2.7):
        b.step(t)
    assert b.error_code == 1013
    return b


def test_stand_then_walk_then_expire():
    b = BridgeCore(0.0)
    b.handle(STAND_UP, "", 0.0); assert b.step(0.0).command == STAND_CMD
    b.step(2.1); assert b.error_code == 1002
    b.handle(BALANCE_STAND, "", 2.1); assert b.step(2.1).command == TROT_CMD
    b.step(2.7); assert b.error_code == 1013
    b.handle(MOVE, '{"x":0.3,"y":0,"z":0}', 3.0); i = b.step(3.0)
    assert i.command == TROT_CMD and (i.lx, i.ly) != (0, 0)
    assert i.ly == pytest.approx(0.75) and i.lx == 0         # 前後は ly（0.3 m/s ÷ 0.4）
    assert b.error_code == 1015
    i = b.step(4.2); assert (i.lx, i.ly, i.rx) == (0, 0, 0) and b.error_code == 1013


def test_gesture_is_pretend_and_sit_uses_crouch():
    b = standing_core()
    b.handle(HELLO, "", 10.0); assert b.step(10.0) is None and "再現なし" in b.notes[-1]
    b.handle(SIT, "", 13.5); assert b.step(13.5).command == DOWN_CMD
    b.step(15.6); assert b.error_code == 1007 and state_from_error_code(1007) == SITTING


def test_invalid_command_ignored():
    b = BridgeCore(0.0); b.handle(MOVE, '{"x":0.3,"y":0,"z":0}', 0.0)
    assert b.step(0.0) is None and b.error_code == 1001


def test_controller_gets_down_stand_trot_in_order():
    """脱力 → 伏せ → 立つ → 歩く：2 を 2 回、伏せの 1.8 秒が過ぎてから 4。続けて送らない。"""
    b = BridgeCore(0.0)
    sent = []
    t = 0.0
    b.handle(STAND_UP, "", t)
    while t < 6.0:
        if abs(t - 2.5) < 1e-9:
            b.handle(BALANCE_STAND, "", t)
        if abs(t - 3.5) < 1e-9:
            b.handle(MOVE, '{"x":0,"y":0,"z":0.25}', t)
        i = b.step(t)
        if i is not None:
            sent.append((round(t, 2), i))
        t = round(t + 0.05, 2)
    cmds = [(at, i.command) for at, i in sent]
    assert cmds[:3] == [(0.0, STAND_CMD), (0.2, STAND_CMD), (2.5, TROT_CMD)]
    assert sent[3][0] == 3.5 and sent[3][1].command == TROT_CMD
    assert sent[3][1].rx == pytest.approx(-0.5)                # 左回り 0.25 rad/s → rx = −z / 0.5
    assert all(i.ry == 0 for _, i in sent)


def test_slow_gazebo_keeps_app_timing_and_command_order():
    """Gazebo が実時間の半分の速さ（sim = now / 2）。窓口の時間は now、command の間隔は sim で数える。"""
    b = BridgeCore(0.0, 0.0)
    sent = []
    t = 0.0
    b.handle(STAND_UP, "", t, t / 2)
    while t < 8.0:
        if abs(t - 3.0) < 1e-9:                      # アプリは StandUp の 3 秒後（実際の時間）に送る
            assert b.error_code == 1002
            b.handle(BALANCE_STAND, "", t, t / 2)
        i = b.step(t, t / 2)
        if i is not None:
            sent.append((t / 2, i.command))
        t = round(t + 0.05, 2)
    assert b.error_code == 1013
    assert [c for _, c in sent] == [STAND_CMD, STAND_CMD, TROT_CMD]
    assert sent[2][0] >= 1.8 + 0.2 - 1e-9              # 4 は 2 通目の 2 が効いた（伏せの 1.8 秒）後


def test_move_input_follows_controller_table():
    i = move_input(0.3, 0.15, 0.8)
    assert i.command == TROT_CMD
    assert (i.ly, i.lx, i.rx) == pytest.approx((0.75, -0.5, -1.0))   # 旋回は上限 0.5 rad/s で切る
    assert move_input(-1.0, -1.0, 0.0).ly == -1.0 and move_input(-1.0, -1.0, 0.0).lx == 1.0


def test_stop_and_bad_move_parameters():
    b = standing_core()
    b.handle(MOVE, '{"x":NaN,"y":0,"z":0}', 3.0)
    assert b.step(3.0) is None and b.error_code == 1013
    b.handle(MOVE, '{"x":-0.2}', 3.1); assert b.step(3.1).ly == pytest.approx(-0.5)
    b.handle(MOVE, '{"x":-0.2}', 3.2); assert b.step(3.2) is None      # 同じ速度の送り直しは出さない
    b.handle(STOP_MOVE, "", 3.3); i = b.step(3.3)
    assert i.command == TROT_CMD and (i.lx, i.ly, i.rx) == (0, 0, 0) and b.error_code == 1013
    b.handle(STOP_MOVE, "", 3.4); assert b.step(3.4) is None


def test_self_points_dropped_and_mount_matches_perception():
    cfg = PerceptionConfig()
    assert LIDAR_IN_BODY == (cfg.lidar_x, 0.0, cfg.lidar_z) and cfg.lidar_pitch == 0.0
    joint = re.search(r'<joint name="utlidar_joint".*?<origin xyz="([^"]+)" rpy="([^"]+)"',
                      XACRO.read_text(encoding="utf-8"), re.S)
    assert [float(v) for v in joint.group(1).split()] == list(LIDAR_IN_BODY)
    assert [float(v) for v in joint.group(2).split()] == [0.0, 0.0, 0.0]
    pts = np.array([[-0.1, 0.0, -0.3],        # 自分の脚（機体座標 x 0.18、z −0.2）
                    [-0.6, 0.1, -0.2],        # 自分の後ろ足（機体 x −0.32）
                    [0.8, 0.0, -0.2],         # 前の箱（機体 x 1.08）
                    [math.inf, 0.0, 0.0]])    # 当たらなかった本
    assert drop_self_points(pts).tolist() == [[0.8, 0.0, -0.2]]


def test_boxes_follow_heading_and_collision():
    w = BoxWorld()
    name, box = w.place("front", x=1.0, y=2.0, yaw=math.pi / 2)
    assert (box.x, box.y) == pytest.approx((1.0, 3.2)) and box.size == 0.3
    w.add(name, box)
    _, back = w.place("back", x=0.0, y=0.0, yaw=0.0)
    assert back.x == pytest.approx(-1.2)
    assert w.world(1.0, 2.0) == {"boxes": [[1.0, 3.2, 0.3]], "collided": False}
    assert w.world(1.0, 3.2 - 0.15 - FOOTPRINT + 0.01)["collided"] is True
    assert w.clear() == [name] and w.world(1.0, 3.0) == {"boxes": [], "collided": False}
