"""O15 (spec §10.5): when the supervisor is empty because no path reaches an object on the grid with a 1-cell
margin (MSG_NO_PATH, R15), the session builds the grid once more with a DETOUR_MARGIN-cell margin and synthesizes
again in the time left of the same budget (builder check M6). It skips the rebuild when that grid is over
MAX_CELLS, and an empty result keeps MSG_NO_PATH."""

import dataclasses
import math

import pytest
from go2_nl_control.actions import LYING, STAND_UP

import voice_ltl.session as session_mod
from sim.sim_core import SimGo2, SimObject
from voice_ltl.automata import SpecError
from voice_ltl.grid import MAX_CELLS, GridError, grid_shape, grid_world
from voice_ltl.objects import Obj, edge_distance, footprint_angle
from voice_ltl.session import DETOUR_MARGIN, MSG_NO_PATH, SYNTH_BUDGET_S

from test_session_objects import drive, llm_for, make_objects, run_to_the_end, walled_box

START = (0.0, 0.0, 0.0)
NEAR_BOX = frozenset({"near_box_1"})


def wall_scene():
    """A 0.5 m box at (3, 0) behind a row of 0.3 m buckets at x = 1.5, y = -1.75 ... 1.75 in 0.5 m steps. The row
    blocks the cells x = 1.0 ... 2.0 for |y| <= 2.0, so the 1-cell-margin grid (|y| <= 1.5) has no way round it
    and the 3-cell one (|y| <= 2.5) has. Buckets, not boxes: the prompt lists 8 objects, the box first."""
    row = [SimObject(f"bucket_{k + 1}", "bucket", 1.5, -1.75 + 0.5 * k, 0.3, 0.3) for k in range(8)]
    return [SimObject("box_1", "box", 3.0, 0.0, 0.5, 0.5)] + row


def rows(scene):
    return tuple(Obj(o.id, o.label, o.x, o.y, (o.sx, o.sy)) for o in scene)


def reachable(gw):
    """Every proposition some reachable state or event of the PLANT carries (as Session._empty_message reads it)."""
    return set(gw.world.initial_label).union(*gw.world.event_label.values())


def area(gw):
    (i0, i1), (j0, j1) = gw.grid.i_range, gw.grid.j_range
    return (i1 - i0 + 1) * (j1 - j0 + 1)


def record_synthesize(monkeypatch, fail_from=None):
    """Wrap session.synthesize: the list gets (timeout_s or None, seconds taken) of every call. With fail_from = k,
    the k-th call and later raise SpecError instead."""
    calls = []
    real = session_mod.synthesize

    def wrapper(*args):                       # positional, as test_session's wrapper takes them
        timeout_s = args[3] if len(args) > 3 else None
        if fail_from is not None and len(calls) + 1 >= fail_from:
            calls.append((timeout_s, 0.0))
            raise SpecError("too big")
        started = session_mod.time.monotonic()
        syn = real(*args)
        calls.append((timeout_s, session_mod.time.monotonic() - started))
        return syn

    monkeypatch.setattr(session_mod, "synthesize", wrapper)
    return calls


def sim_with(scene):
    sim = SimGo2()
    sim.objects.extend(scene)
    return sim


def test_detour_margin_is_three_cells():
    assert DETOUR_MARGIN == 3 and SYNTH_BUDGET_S == 20.0


def test_detour_rebuild_finds_a_path_with_a_wider_margin(tmp_path, monkeypatch):
    objs = rows(wall_scene())
    narrow = grid_world(LYING, False, False, START, objs, NEAR_BOX)
    wide = grid_world(LYING, False, False, START, objs, NEAR_BOX, margin=DETOUR_MARGIN)
    assert grid_shape(START, objs, NEAR_BOX).near["near_box_1"]         # near keys exist ...
    assert "near_box_1" not in reachable(narrow)                         # ... but no path reaches them
    assert "near_box_1" in reachable(wide) and area(wide) <= MAX_CELLS
    calls = record_synthesize(monkeypatch)
    sess, link = make_objects(tmp_path, llm_for("F near_box_1"), sim=sim_with(wall_scene()))
    sess.submit_text("箱まで歩いて")
    path = []
    snap = run_to_the_end(sess, link, path=path)
    assert snap["status"] == "done", snap["message"]
    assert len(calls) == 2
    assert snap["grid"]["j"] == list(wide.grid.j_range)                  # the plan ran on the 3-cell grid
    assert link.sim.accepted[0] == STAND_UP
    box = objs[0]                                                         # spec §10.8's near lines
    assert edge_distance(link.sim.x, link.sim.y, box) <= 1.5
    assert math.degrees(footprint_angle(link.sim.x, link.sim.y, link.sim.yaw, box)) <= 45
    assert min(edge_distance(x, y, o) for x, y in path for o in objs) >= 0.30
    assert max(abs(y) for _, y in path) > 2.0                             # round the end of the row
    assert not getattr(link, "hit", False) and not link.sim.collided


def test_detour_rebuild_is_skipped_over_256_cells(tmp_path, monkeypatch):
    scene = [dataclasses.replace(o, x=o.x + 2.5) for o in walled_box()]   # the walled box at (5, 0)
    objs = rows(scene)
    narrow = grid_world(LYING, False, False, START, objs, NEAR_BOX)
    assert "near_box_1" not in reachable(narrow) and area(narrow) <= MAX_CELLS
    with pytest.raises(GridError):                                        # the 3-cell grid is over MAX_CELLS
        grid_world(LYING, False, False, START, objs, NEAR_BOX, margin=DETOUR_MARGIN)
    calls = record_synthesize(monkeypatch)
    sess, link = make_objects(tmp_path, llm_for("F near_box_1"), sim=sim_with(scene))
    sess.submit_text("箱まで歩いて")
    drive(sess, link, 2.0)
    snap = sess.snapshot()
    assert snap["status"] == "error" and snap["message"] == MSG_NO_PATH
    assert len(calls) == 1 and link.sim.accepted == []


def test_detour_rebuild_runs_once(tmp_path, monkeypatch):
    objs = rows(walled_box())
    narrow = grid_world(LYING, False, False, START, objs, NEAR_BOX)
    wide = grid_world(LYING, False, False, START, objs, NEAR_BOX, margin=DETOUR_MARGIN)
    assert "near_box_1" not in reachable(wide) and area(wide) <= MAX_CELLS   # enclosed at any margin
    calls = record_synthesize(monkeypatch)
    sess, link = make_objects(tmp_path, llm_for("F near_box_1"), sim=sim_with(walled_box()))
    sess.submit_text("箱まで歩いて")
    drive(sess, link, 2.0)
    snap = sess.snapshot()
    assert snap["status"] == "error" and snap["message"] == MSG_NO_PATH
    assert len(calls) == 2 and link.sim.accepted == []
    assert snap["grid"]["j"] == list(narrow.grid.j_range) != list(wide.grid.j_range)   # the 1-cell grid stays


def test_detour_rebuild_gets_the_time_left_of_the_budget(tmp_path, monkeypatch):
    calls = record_synthesize(monkeypatch)
    sess, link = make_objects(tmp_path, llm_for("F near_box_1"), sim=sim_with(walled_box()))
    sess.submit_text("箱まで歩いて")
    drive(sess, link, 2.0)
    (first, took), (second, _) = calls
    assert first == SYNTH_BUDGET_S
    assert 0 < second <= SYNTH_BUDGET_S - took


def test_a_rebuild_that_fails_keeps_the_no_path_message(tmp_path, monkeypatch):
    calls = record_synthesize(monkeypatch, fail_from=2)
    sess, link = make_objects(tmp_path, llm_for("F near_box_1"), sim=sim_with(wall_scene()))
    sess.submit_text("箱まで歩いて")
    drive(sess, link, 2.0)
    snap = sess.snapshot()
    assert snap["status"] == "error" and snap["message"] == MSG_NO_PATH
    assert len(calls) == 2 and link.sim.accepted == []


def test_plans_with_a_path_synthesize_once(tmp_path, monkeypatch):
    calls = record_synthesize(monkeypatch)
    sess, link = make_objects(tmp_path, llm_for("F near_chair_1"))
    sess.submit_text("椅子まで歩いて")
    drive(sess, link, 1.0)
    narrow = grid_world(LYING, False, False, START, sess._scene.objects, frozenset({"near_chair_1"}))
    assert len(calls) == 1 and sess.snapshot()["grid"]["j"] == list(narrow.grid.j_range)
