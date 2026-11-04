import numpy as np
import pytest

from go2_nl_control.actions import (BALANCE_STAND, HELLO, MOVE, RISE_SIT, SIT, STAND_DOWN,
                                    STAND_UP, STANDING, STOP_MOVE, state_from_error_code)
from go2_nl_control.perception import PerceptionConfig
from sim.sim_core import FOOTPRINT, FOOTPRINT_HALF_WIDTH, Box, SimGo2
from voice_ltl.sensing import Sensing


def run(sim, t0, t1, dt=0.1):
    t = t0
    while t < t1:
        sim.step(t)
        t += dt
    return t


def standing_sim():
    return SimGo2(posture=STANDING)


def test_stand_sequence_and_ignore_invalid():
    sim = SimGo2()
    sim.handle(MOVE, '{"x":0.3,"y":0,"z":0}', 0.0)
    assert MOVE not in sim.accepted                       # 伏せたまま歩けない
    sim.handle(STAND_UP, "", 0.0)
    run(sim, 0, 1.6)
    sim.handle(BALANCE_STAND, "", 1.6)
    run(sim, 1.6, 2.2)
    assert sim.error_code == 1013 and state_from_error_code(sim.error_code) == STANDING


def test_move_expires_without_resend():
    sim = standing_sim()
    sim.handle(MOVE, '{"x":0.3,"y":0,"z":0}', 0.0)
    run(sim, 0, 2.0)
    assert 0.25 < sim.x < 0.35                            # 1 秒だけ進んで止まる


def test_box_appears_in_cloud_and_blocks_motion():
    sim = standing_sim()
    sim.place_box("front", 1.2)
    s = Sensing(PerceptionConfig())
    assert s.update(sim.cloud_lidar(), 0).front_dist == pytest.approx(1.05, abs=0.05)
    t = 0.0
    while t < 10:
        sim.handle(MOVE, '{"x":0.3,"y":0,"z":0}', t)
        t = run(sim, t, t + 0.1)
    assert sim.collided and sim.x < 1.2 - 0.15 - FOOTPRINT + 0.05


def test_box_beside_the_body_is_not_hit_but_box_ahead_is():
    side = standing_sim()
    side.boxes.append(Box(0.0, 0.40))                    # 横 0.40 m（体の半幅 0.16 m の外）
    t = 0.0
    while t < 6:
        side.handle(MOVE, '{"x":0.3,"y":0,"z":0}', t)
        t = run(side, t, t + 0.1)
    assert not side.collided and side.x > 1.0
    ahead = standing_sim()
    ahead.place_box("front", 1.2)
    t = 0.0
    while t < 6:
        ahead.handle(MOVE, '{"x":0.3,"y":0,"z":0}', t)
        t = run(ahead, t, t + 0.1)
    assert ahead.collided


def test_footprint_is_a_rectangle():
    assert FOOTPRINT == 0.35 and FOOTPRINT_HALF_WIDTH == 0.16


def test_collided_is_stable_while_move_is_resent_into_box():
    sim = standing_sim()
    sim.place_box("front", 1.2)
    t = 0.0
    seen = []
    while t < 10:
        sim.handle(MOVE, '{"x":0.3,"y":0,"z":0}', t)
        t = run(sim, t, t + 0.1)
        if t > 6: seen.append(sim.collided)
    assert seen and all(seen)                             # 最後の step の位相に依らず True のまま
    sim.handle(MOVE, '{"x":-0.3,"y":0,"z":0}', t)
    t = run(sim, t, t + 0.3)
    assert not sim.collided                               # 離れる動きで解除される
    sim.clear_boxes()
    assert not sim.collided


def test_zero_velocity_move_returns_to_1013():
    sim = standing_sim()
    sim.handle(MOVE, '{"x":0.3,"y":0,"z":0}', 0.0)
    assert sim.error_code == 1015
    sim.handle(MOVE, '{"x":0,"y":0,"z":0}', 0.2)
    assert sim.error_code == 1013                         # StopMove を待たない


def test_gesture_while_walking_stops_and_returns_to_1013():
    sim = standing_sim()
    sim.handle(MOVE, '{"x":0.3,"y":0,"z":0}', 0.0)
    run(sim, 0, 0.3)
    sim.handle(HELLO, "", 0.3)
    assert HELLO in sim.accepted
    assert sim.error_code == 1013 and sim._vel == (0.0, 0.0, 0.0)
    x = sim.x
    run(sim, 0.3, 1.5)
    assert sim.x == x and sim.error_code == 1013          # 3 秒の間 1015 のままにならない


def test_busy_rejects_other_commands():
    sim = standing_sim()
    sim.handle(SIT, "", 0.0)
    sim.handle(MOVE, '{"x":0.3,"y":0,"z":0}', 0.5)
    sim.handle(HELLO, "", 0.5)
    assert sim.accepted == [SIT]
    assert sim.error_code == 1013                         # まだ座り終えていない


def test_stop_move_is_accepted_while_busy():
    sim = standing_sim()
    sim.handle(SIT, "", 0.0)
    sim.handle(STOP_MOVE, "", 0.5)
    assert sim.accepted == [SIT, STOP_MOVE]
    run(sim, 0, 2.2)
    assert sim.error_code == 1007                         # 動作は止まらず最後まで進む


def test_sit_risesit_standdown_transitions():
    sim = standing_sim()
    sim.handle(SIT, "", 0.0)
    run(sim, 0, 2.1)
    assert sim.error_code == 1007
    sim.handle(RISE_SIT, "", 2.1)
    run(sim, 2.1, 3.7)
    assert sim.error_code == 1013
    sim.handle(STAND_DOWN, "", 3.7)
    run(sim, 3.7, 5.3)
    assert sim.error_code == 1004
    assert sim.accepted == [SIT, RISE_SIT, STAND_DOWN]


def test_cloud_of_back_box_is_behind_the_body():
    sim = standing_sim()
    sim.place_box("back", 1.2)
    cloud = sim.cloud_lidar()
    assert cloud.shape[1] == 3 and len(cloud) > 0
    cfg = PerceptionConfig()
    assert np.all(cloud[:, 0] + cfg.lidar_x < 0)          # 機体の後ろ側だけにある
    reading = Sensing(cfg).update(cloud, 0.0)
    assert reading.back_dist == pytest.approx(1.05, abs=0.05)
    assert reading.front_dist is None or reading.front_dist > 5
