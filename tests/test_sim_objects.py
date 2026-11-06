"""Labelled objects and the fake camera in the fake Go2 (sim/sim_core.py, R22).

The objects are not boxes: they block motion and show in the LiDAR, but the page's box buttons
(/sim/world, clear_boxes) never see them. The fake camera has the logical camera's field of view
(120°, 0.1–8 m, at CAMERA_IN_BODY) and remembers what it saw.
"""

import math

import pytest

from go2_nl_control.actions import MOVE, STANDING
from sim import sim_core
from sim.sim_core import FakeCamera, SimGo2, SimObject, in_camera_view
from voice_ltl import objects


def standing_sim():
    return SimGo2(posture=STANDING)


def run_forward(sim, seconds, dt=0.1):
    """Resend MOVE x 0.3 every dt (as the executor does) and step the sim."""
    t = 0.0
    while t < seconds:
        sim.handle(MOVE, '{"x":0.3,"y":0,"z":0}', t)
        sim.step(t)
        t += dt
    return t


def test_objects_are_not_boxes():
    sim = SimGo2()
    assert sim.objects == []
    sim.place_objects()
    assert [o.id for o in sim.objects] == ["box_1", "chair_1"] and sim.boxes == []
    sim.clear_boxes()
    assert len(sim.objects) == 2


def test_objects_block_motion_and_show_in_the_cloud():
    sim = standing_sim()
    sim.objects.append(SimObject("box_1", "box", 0.9, 0.0, 0.5, 0.5))
    run_forward(sim, 3.0)                 # MOVE x 0.3 resent every 0.1 s
    assert sim.collided and sim.x < 0.9 - 0.25 - 0.35 + 0.03
    assert len(sim.cloud_lidar()) > 0


@pytest.mark.parametrize("pose,visible", [((0, 0, 0), True), ((0, 0, math.pi), False),
    ((0, 0, 1.6), False), ((-5.0, 1.5, 0), True), ((-6.0, 1.5, 0), False),   # camera 7.17 m / 8.17 m away
    ((0, 0, 4 * math.pi), True)])                                            # an unwrapped yaw after two laps
def test_camera_sees_120_degrees_and_8_m(pose, visible):
    assert in_camera_view(pose, 2.5, 1.5) is visible


def test_fake_camera_remembers_what_it_saw():
    sim, cam = SimGo2(), FakeCamera()
    sim.place_objects()
    assert [(o.id, o.visible, o.seen) for o in cam.see(sim, 1.0)] == [("box_1", True, 1.0), ("chair_1", True, 1.0)]
    sim.yaw = math.pi
    assert [(o.id, o.visible, o.seen) for o in cam.see(sim, 2.0)] == [("box_1", False, 1.0), ("chair_1", False, 1.0)]
    assert cam.see(sim, 2.0)[1].size == (0.45, 0.45)
    sim2, cam2 = SimGo2(), FakeCamera()
    sim2.place_objects()
    sim2.yaw = math.pi
    assert cam2.see(sim2, 1.0) == []


def test_sim_core_uses_the_shared_wrap():      # one (−π, π] wrap for the whole project (objects.wrap_rad)
    assert sim_core.wrap_rad is objects.wrap_rad and not hasattr(sim_core, "_wrap")
