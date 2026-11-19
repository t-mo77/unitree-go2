"""The household layout end to end in the fake Go2 (open-objects spec §12.4 procedure, §13.1).

Each run starts lying at (0, 0) facing +x, as the fake Go2 does. It goes to the test pose first (the room proposal
after the H3 screening, R-H3a, R-H3b): Bf with F(turned_left & X F turned_left), then F moved_forward; R5 with
F turned_right, then five F moved_forward in one formula; L with F turned_left. It then stands still for a moment,
and the target formula goes through the Session with a FakeLLM. The acceptance lines are spec §10.8's:
- near: the final d_e to the target ≤ 1.5 m, and some footprint point within 45° of the final yaw. The least d_e to
  every footprint over the run is ≥ 0.30 m. The final d_e is reported only (≥ 0.50 expected).
- lap: the winding number about the target's centre is in [0.9, 1.5), and the least d_e to the target is ≥ 0.6 m.
"""

import math
import time

import pytest
from go2_nl_control.actions import STAND_UP, STANDING

from sim.sim_core import SimGo2, layout_objects
from voice_ltl.automata import synthesize
from voice_ltl.grid import grid_world
from voice_ltl.llm import FakeLLM
from voice_ltl.objects import edge_distance, footprint_angle, formula_atoms, name_of, winding_number
from voice_ltl.session import DirectSimLink, Session

FINAL = ("done", "error")
FIVE_STEPS = "F(moved_forward & X F(moved_forward & X F(moved_forward & X F(moved_forward & X F moved_forward))))"
# test pose -> its procedure, (instruction, formula) in order; spec §12.4 step 1 with R-H3a / R-H3b's poses
PROCEDURES = {"Bf": (("左に2回まわって", "F(turned_left & X F turned_left)"), ("前に進んで", "F moved_forward")),
              "R5": (("右にまわって", "F turned_right"), ("前に5歩進んで", FIVE_STEPS)),
              "L": (("左にまわって", "F turned_left"),)}
TARGETS = (("suitcase_1", "Bf"), ("traffic_cone_1", "R5"), ("sofa_1", "Bf"), ("table_1", "L"))
SETTLE_S = 1.0                    # standing still after the turn (spec §12.4 step 2)


def drive(session, link, seconds, dt=0.1, path=None):
    """Advance the simulated time: link.advance(now) then session.tick(now), every dt.
    path, a list, gets the Go2's (x, y) after every tick."""
    end = link.now + seconds
    while link.now < end - 1e-9:
        now = round(link.now + dt, 6)
        link.advance(now)
        session.tick(now)
        if path is not None:
            path.append((link.sim.x, link.sim.y))


def run_to_the_end(session, link, limit=400.0, path=None):
    end = link.now + limit
    while link.now < end - 1e-9:
        drive(session, link, 0.1, path=path)
        snap = session.snapshot()
        if snap["status"] in FINAL:
            return snap
    raise AssertionError(f"not reached in {limit} s: {session.snapshot()['status']}")


def household_run(tmp_path, obj_id, pose, kind):
    """The §12.4 procedure on a fresh fake Go2: go to the pose, settle, then the target formula.
    (link, the whole path, the target formula's path, the target as laid out)."""
    objs, heights = layout_objects("household")
    sim = SimGo2()
    sim.place_objects(objs, heights)
    steps = PROCEDURES[pose]
    formula = f"F {kind}_{obj_id}"
    name = name_of(obj_id.rsplit("_", 1)[0])
    text = f"{name}まで歩いて" if kind == "near" else f"{name}の周りを一周して"
    llm = FakeLLM([{"formula": f, "unsupported": []} for _, f in steps] + [{"formula": formula, "unsupported": []}])
    link = DirectSimLink(sim)
    sess = Session(link, llm, None, tmp_path, background=False)
    path = [(sim.x, sim.y)]
    for step_text, _ in steps:
        sess.submit_text(step_text)
        snap = run_to_the_end(sess, link, path=path)
        assert snap["status"] == "done", snap["message"]
    assert sim.accepted[0] == STAND_UP and sim.posture == STANDING
    drive(sess, link, SETTLE_S, path=path)
    # listed (step 2): seen on the way or from the pose, as the Scene keeps it
    assert any(o["id"] == obj_id for o in sess.snapshot()["objects"]), obj_id
    sess.submit_text(text)
    run = [(sim.x, sim.y)]
    snap = run_to_the_end(sess, link, path=run)
    assert snap["status"] == "done", snap["message"]
    assert len(llm.prompts) == len(steps) + 1 and sess._grid is not None and formula_atoms(formula) <= sess._grid.atoms
    assert not sim.collided
    return link, path + run, run, next(o for o in objs if o.id == obj_id), objs


@pytest.mark.parametrize("obj_id,pose", TARGETS, ids=[t[0] for t in TARGETS])
def test_walk_to_each_household_object(tmp_path, obj_id, pose):
    link, path, _, obj, objs = household_run(tmp_path, obj_id, pose, "near")
    sim = link.sim
    final_edge = edge_distance(sim.x, sim.y, obj)
    angle = math.degrees(footprint_angle(sim.x, sim.y, sim.yaw, obj))
    least = min(edge_distance(x, y, o) for x, y in path for o in objs)
    print(f"{obj_id} near: final d_e {final_edge:.3f} m, footprint angle {angle:.1f} deg, least d_e {least:.3f} m")
    assert final_edge <= 1.5
    assert angle <= 45.0
    assert least >= 0.30


@pytest.mark.parametrize("obj_id,pose", TARGETS, ids=[t[0] for t in TARGETS])
def test_lap_around_each_household_object(tmp_path, obj_id, pose):
    _, _, run, obj, _ = household_run(tmp_path, obj_id, pose, "around")
    winding = winding_number(run, (obj.x, obj.y))
    least = min(edge_distance(x, y, obj) for x, y in run)
    print(f"{obj_id} lap: winding {winding:.3f}, least d_e {least:.3f} m")
    assert 0.9 <= winding < 1.5
    assert least >= 0.6


@pytest.mark.slow
def test_sofa_synthesis_is_fast(tmp_path):
    """The grid and the supervisor of the sofa formulas from test pose Bf (spec §13.1, R-H3a): near ≤ 1 s,
    lap ≤ 3 s."""
    objs, _ = layout_objects("household")
    for formula, limit in (("F near_sofa_1", 1.0), ("F around_sofa_1", 3.0)):
        t0 = time.monotonic()
        gw = grid_world(STANDING, False, False, (-0.5, 0.0, math.pi), objs, formula_atoms(formula))
        syn = synthesize(formula, gw.world, tmp_path / formula.split()[-1])
        seconds = time.monotonic() - t0
        print(f"{formula}: PLANT {len(gw.world.states)}, SUP {syn.sup.size}, {seconds:.2f} s")
        assert not syn.empty and seconds <= limit, (formula, seconds)
