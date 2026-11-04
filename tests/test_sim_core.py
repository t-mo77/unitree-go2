import pytest

from go2_nl_control.actions import (BALANCE_STAND, MOVE, STAND_UP, STANDING,
                                    state_from_error_code)
from go2_nl_control.perception import PerceptionConfig
from sim.sim_core import FOOTPRINT, SimGo2
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
    sim = SimGo2(); sim.handle(MOVE, '{"x":0.3,"y":0,"z":0}', 0.0)
    assert MOVE not in sim.accepted                       # 伏せたまま歩けない
    sim.handle(STAND_UP, "", 0.0); run(sim, 0, 1.6)
    sim.handle(BALANCE_STAND, "", 1.6); run(sim, 1.6, 2.2)
    assert sim.error_code == 1013 and state_from_error_code(sim.error_code) == STANDING


def test_move_expires_without_resend():
    sim = standing_sim(); sim.handle(MOVE, '{"x":0.3,"y":0,"z":0}', 0.0); run(sim, 0, 2.0)
    assert 0.25 < sim.x < 0.35                            # 1 秒だけ進んで止まる


def test_box_appears_in_cloud_and_blocks_motion():
    sim = standing_sim(); sim.place_box("front", 1.2)
    s = Sensing(PerceptionConfig()); assert s.update(sim.cloud_lidar(), 0).front_dist == pytest.approx(1.05, abs=0.05)
    t = 0.0
    while t < 10: sim.handle(MOVE, '{"x":0.3,"y":0,"z":0}', t); t = run(sim, t, t + 0.1)
    assert sim.collided and sim.x < 1.2 - 0.15 - FOOTPRINT + 0.05
