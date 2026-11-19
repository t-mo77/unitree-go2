"""Labelled objects and the fake camera in the fake Go2 (sim/sim_core.py, R22).

The objects are not boxes: they block motion and show in the LiDAR, but the page's box buttons
(/sim/world, clear_boxes) never see them. The fake camera has the logical camera's field of view
(120°, 0.1–8 m, at CAMERA_IN_BODY) and remembers what it saw.
"""

import json
import math

import numpy as np
import pytest

from go2_nl_control.actions import MOVE, STANDING
from sim import sim_core
from sim.sim_core import BOX_HEIGHT, GROUND_Z, FakeCamera, SimGo2, SimObject, in_camera_view, layout_objects
from voice_ltl import layout, objects


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


def world_points(sim):
    """The fake LiDAR cloud back in world axes (the sim at the origin facing +x, the default LiDAR mount)."""
    k = sim.cfg
    assert (sim.x, sim.y, sim.yaw) == (0.0, 0.0, 0.0) and (k.lidar_pitch, k.lidar_yaw) == (0.0, 0.0)
    c = sim.cloud_lidar()
    return np.column_stack([c[:, 0] + k.lidar_x, c[:, 1], c[:, 2] + k.lidar_z - GROUND_Z])


def test_fake_lidar_uses_object_heights():
    sim = standing_sim()
    sim.place_objects(*layout_objects("default"))
    sim.place_box("front", 1.2)                                  # a button box: (1.2, 0), BOX_HEIGHT
    w = world_points(sim)
    chair, box, button = w[w[:, 1] < -1.0], w[w[:, 1] > 1.0], w[np.abs(w[:, 1]) < 0.5]
    assert chair[:, 2].max() == pytest.approx(0.9) and box[:, 2].max() == pytest.approx(0.5)
    assert button[:, 2].max() == pytest.approx(0.4) == pytest.approx(BOX_HEIGHT)
    assert min(chair[:, 2].min(), box[:, 2].min(), button[:, 2].min()) == pytest.approx(0.0, abs=1e-9)
    assert len(chair) + len(box) + len(button) == len(w)


def test_placed_objects_take_the_vocabulary_height_without_a_layout():
    sim = standing_sim()
    sim.place_objects()                                          # DEFAULT_LAYOUT, no heights given
    assert [o.height for o in sim.objects] == [0.5, 0.9]         # vocabulary.height_of: box, chair
    laid = standing_sim()
    laid.place_objects(*layout_objects("default"))
    assert np.array_equal(sim.cloud_lidar(), laid.cloud_lidar())
    sim.place_objects(objects.DEFAULT_LAYOUT, {"chair_1": 1.2})  # a height given for one id only
    assert [o.height for o in sim.objects] == [0.5, 1.2]
    assert SimObject("box_1", "box", 0.9, 0.0, 0.5, 0.5).height == BOX_HEIGHT


def test_a_wall_reaches_a_height_off_the_point_step():
    sim = standing_sim()
    sim.objects.append(SimObject("sofa_1", "sofa", 2.0, 0.0, 1.0, 1.8, 0.943))
    w = world_points(sim)
    assert w[:, 2].max() == pytest.approx(0.943) and np.isclose(w[:, 2], 0.9).any()


def test_fake_camera_orders_ids_by_number():
    sim, cam = SimGo2(), FakeCamera()
    sim.objects[:] = [SimObject("chair_1", "chair", 2.5, -1.0, 0.45, 0.45), SimObject("box_10", "box", 2.5, 0.0, 0.5, 0.5),
                      SimObject("box_2", "box", 2.5, 1.0, 0.5, 0.5)]
    assert [o.id for o in cam.see(sim, 1.0)] == ["box_2", "box_10", "chair_1"]


def test_layout_objects_reads_a_layout():
    objs, heights = layout_objects("default")
    assert objs == objects.DEFAULT_LAYOUT and heights == {"box_1": 0.5, "chair_1": 0.9}
    with pytest.raises(layout.LayoutError):
        layout_objects("houshold")


def test_fake_go2_message_says_fake_and_keeps_its_rows():
    from sim.fake_go2 import objects_message
    sim, cam = SimGo2(), FakeCamera()
    sim.place_objects(*layout_objects("default"))
    msg = objects_message(cam, sim, 12.5)
    assert list(msg) == ["frame", "stamp", "source", "objects"]
    assert (msg["frame"], msg["stamp"], msg["source"]) == ("odom", 12.5, "fake")
    seen = [o.to_row() for o in FakeCamera().see(sim, 12.5)]
    assert msg["objects"] == seen and len(seen) == 2
    assert all(set(row) == {"id", "label", "x", "y", "size", "seen", "visible"} for row in msg["objects"])
    assert objects.parse(json.loads(json.dumps(msg))) == tuple(
        objects.Obj(o.id, o.label, o.x, o.y, o.size, True, 12.5) for o in objects.DEFAULT_LAYOUT)


def test_fake_go2_places_the_layout_named_by_go2_layout(monkeypatch):
    from sim import fake_go2
    monkeypatch.setenv("GO2_LAYOUT", "houshold")
    with pytest.raises(layout.LayoutError) as e:                 # before any ROS node exists
        fake_go2.FakeGo2()
    assert "houshold" in str(e.value) and "default" in str(e.value)
    asked = []

    class Stop(Exception):
        pass

    def record(name):
        asked.append(name)
        raise Stop

    monkeypatch.setattr(fake_go2, "layout_objects", record)
    monkeypatch.delenv("GO2_LAYOUT")
    with pytest.raises(Stop):
        fake_go2.FakeGo2()
    monkeypatch.setenv("GO2_LAYOUT", "household")
    with pytest.raises(Stop):
        fake_go2.FakeGo2()
    assert asked == ["default", "household"]
