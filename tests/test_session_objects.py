"""The session with objects: the Scene frozen per instruction, the live object list on the page,
object formulas on the grid plant (snap and jump after each step), the diagrams after the plan
starts, and the grid and the plan's cells in the snapshot (Task G of the objects plan)."""

import dataclasses
import math
import threading
import time

import pytest
from go2_nl_control.actions import BALANCE_STAND, HELLO, MOVE, SIT, STAND_DOWN, STAND_UP, STOP_MOVE, UNKNOWN
from go2_nl_control.supervisor import BLOCKED, SENSOR_LOST

import voice_ltl.automata as automata
import voice_ltl.session as session_mod
from sim.sim_core import SimGo2, SimObject
from voice_ltl.diagrams import MAP_HINT_TEXT, TOO_BIG_TEXT
from voice_ltl.grid import GridWorld
from voice_ltl.grid_executor import GRID_EXEC_CONFIG, GRID_SETTLE_S, MIN_SPEED, flows_on
from voice_ltl.llm import FakeLLM
from voice_ltl.objects import (DEFAULT_LAYOUT, MSG_NO_POSE, MSG_POSITION_DRIFT, edge_distance, msg_not_found,
                               winding_number)
from voice_ltl.session import MSG_NO_PATH, DirectSimLink, Session

BOX, CHAIR = DEFAULT_LAYOUT          # box_1 at (2.5, 1.5), chair_1 at (2.5, -1.5)
FINAL = ("done", "error")


def llm_for(*formulas):
    return FakeLLM([{"formula": f, "unsupported": []} for f in formulas])


class LockedLink(DirectSimLink):
    """DirectSimLink for background=True: the worker thread may send StopMove while the test
    thread advances the fake Go2."""

    def __init__(self, sim):
        super().__init__(sim)
        self._mutex = threading.Lock()

    def advance(self, now):
        with self._mutex:
            super().advance(now)

    def send(self, cmd):
        with self._mutex:
            super().send(cmd)


def make_objects(tmp_path, llm, background=False, sim=None, link=None):
    """A session on a fake Go2 with the default objects (box_1 and chair_1) placed."""
    if link is None:
        if sim is None:
            sim = SimGo2()
            sim.place_objects()
        link = LockedLink(sim) if background else DirectSimLink(sim)
    return Session(link, llm, None, tmp_path, background=background), link


def drive(session, link, seconds, dt=0.1, path=None):
    """Advance the simulated time: link.advance(now) then session.tick(now), every dt.
    path, a list, gets the Go2's (x, y) after every tick; link.hit is set once it collides."""
    end = link.now + seconds
    while link.now < end - 1e-9:
        now = round(link.now + dt, 6)
        link.advance(now)
        session.tick(now)
        if path is not None:
            path.append((link.sim.x, link.sim.y))
        if link.sim.collided:
            link.hit = True


def drive_until(session, link, cond, limit=60.0, path=None):
    """drive() one tick at a time until cond(snapshot) holds; the snapshot then."""
    end = link.now + limit
    while link.now < end - 1e-9:
        drive(session, link, 0.1, path=path)
        snap = session.snapshot()
        if cond(snap):
            return snap
    raise AssertionError(f"not reached in {limit} s: {session.snapshot()['status']}")


# ---------------- G2: the Scene and the live objects ----------------
def test_scene_is_frozen_when_the_instruction_arrives(tmp_path):
    sim = SimGo2()
    sim.place_objects()
    del sim.objects[1:]                                       # only box_1
    llm = llm_for("F hello")
    sess, link = make_objects(tmp_path, llm, sim=sim)
    sess.submit_text("挨拶して")
    assert len(llm.prompts) == 1
    prompt = llm.prompts[0]
    assert "- box_1（箱）：左前 2.9 m" in prompt and "chair_1" not in prompt
    sim.objects.append(SimObject("chair_1", "chair", 2.5, -1.5, 0.45, 0.45))
    drive(sess, link, 1.0)
    assert llm.prompts == [prompt]
    assert len(sess._scene.objects) == 1 and sess._scene.pose == (0.0, 0.0, 0.0)
    assert [o["id"] for o in sess.snapshot()["objects"]] == ["box_1", "chair_1"]   # the live list


def test_snapshot_lists_live_objects(tmp_path):
    sess, link = make_objects(tmp_path, llm_for())
    assert sess.snapshot()["objects"] == []                   # no tick yet
    drive(sess, link, 0.1)
    assert sess.snapshot()["objects"] == [
        {"id": "box_1", "label": "box", "name": "箱", "x": 2.5, "y": 1.5, "size": [0.5, 0.5],
         "visible": True, "seen": 0.1, "where": "左前 2.9 m"},
        {"id": "chair_1", "label": "chair", "name": "椅子", "x": 2.5, "y": -1.5, "size": [0.45, 0.45],
         "visible": True, "seen": 0.1, "where": "右前 2.9 m"}]


def test_objects_out_of_view_stay_listed_as_not_visible(tmp_path):
    sess, link = make_objects(tmp_path, llm_for())
    drive(sess, link, 0.1)
    link.sim.yaw = 3.0                                        # facing backwards: nothing in view
    drive(sess, link, 0.1)
    rows = sess.snapshot()["objects"]
    assert [(o["id"], o["visible"], o["seen"]) for o in rows] == [("box_1", False, 0.1), ("chair_1", False, 0.1)]
    assert rows[0]["where"] == "右後ろ 2.9 m"


class NoObjects(DirectSimLink):
    objects = None


def test_links_without_objects_give_an_empty_scene(tmp_path):
    sim = SimGo2()
    sim.place_objects()
    llm = llm_for("F hello")
    sess, link = make_objects(tmp_path, llm, link=NoObjects(sim))
    drive(sess, link, 0.1)
    assert sess.snapshot()["objects"] == []
    sess.submit_text("挨拶して")
    assert "いま見えている物はありません。" in llm.prompts[0] and "box_1" not in llm.prompts[0]
    assert sess._scene.objects == () and sess._scene.pose == (0.0, 0.0, 0.0)


def test_not_found_message_reaches_the_page(tmp_path):
    sess, link = make_objects(tmp_path, FakeLLM([{"formula": None, "unsupported": ["箱"]}]), sim=SimGo2())
    sess.submit_text("箱の周りを一周して")
    drive(sess, link, 0.5)
    snap = sess.snapshot()
    assert snap["message"] == msg_not_found("box") and snap["status"] == "error"
    assert link.sim.accepted == []


def test_audio_instructions_use_the_scene_too(tmp_path):
    sim = SimGo2()
    sim.place_objects()
    llm = llm_for("F hello")
    link = DirectSimLink(sim)
    sess = Session(link, llm, lambda audio: "挨拶して", tmp_path, background=False)
    sess.submit_audio(b"RIFF")
    assert "- box_1（箱）：左前 2.9 m" in llm.prompts[0] and "- chair_1（椅子）：右前 2.9 m" in llm.prompts[0]
    assert len(sess._scene.objects) == 2


# ---------------- G3: object formulas on the grid, snap and jump ----------------
def run_to_the_end(sess, link, limit=300.0, path=None):
    return drive_until(sess, link, lambda s: s["status"] in FINAL, limit=limit, path=path)


def to_chair(sim):
    """(distance to chair_1's centre, |yaw - bearing to it| in degrees) of the Go2 now."""
    bearing = math.atan2(CHAIR.y - sim.y, CHAIR.x - sim.x)
    return (math.hypot(CHAIR.x - sim.x, CHAIR.y - sim.y),
            abs(math.degrees(math.remainder(sim.yaw - bearing, 2 * math.pi))))


def no_supervisor_complaint(snap):
    return not any("監督器が" in line for line in snap["debug_log"])


def test_box_lap_in_the_fake_go2(tmp_path):
    sess, link = make_objects(tmp_path, llm_for("F around_box_1"))
    sess.submit_text("箱の周りを一周して")
    path = [(0.0, 0.0)]
    snap = run_to_the_end(sess, link, path=path)
    assert snap["status"] == "done", snap["message"]
    assert link.sim.accepted[0] == STAND_UP                          # lying: it stands first
    assert 0.9 <= winding_number(path, (BOX.x, BOX.y)) < 1.5         # once, counter-clockwise
    assert min(edge_distance(x, y, BOX) for x, y in path) >= 0.6
    assert not getattr(link, "hit", False) and not link.sim.collided
    assert snap["grid"] is not None and snap["plan_cells"] == []
    assert sess._grid is not None and sess._exec is sess.grid_executor
    assert len(sess.synthesis.world.states) > 12


def test_walk_to_the_chair_in_the_fake_go2(tmp_path):
    sess, link = make_objects(tmp_path, llm_for("F near_chair_1"))
    sess.submit_text("椅子まで歩いて")
    path = []
    snap = run_to_the_end(sess, link, path=path)
    assert snap["status"] == "done", snap["message"]
    assert link.sim.accepted[0] == STAND_UP
    distance, angle = to_chair(link.sim)
    assert distance <= 1.5 and angle <= 45
    assert min(edge_distance(x, y, CHAIR) for x, y in path) >= 0.45
    assert not getattr(link, "hit", False)


def test_object_with_no_near_cell_gives_the_no_path_reason(tmp_path):
    sim = SimGo2()
    sim.objects.append(SimObject("box_1", "box", 2.5, 0.0, 1.0, 1.0))   # r = 0.707: no near band
    sess, link = make_objects(tmp_path, llm_for("F near_box_1"), sim=sim)
    sess.submit_text("箱まで歩いて")
    drive(sess, link, 2.0)
    snap = sess.snapshot()
    assert snap["status"] == "error" and snap["message"] == MSG_NO_PATH
    assert link.sim.accepted == []


def test_stand_walk_to_the_chair_and_say_hello(tmp_path):
    sess, link = make_objects(tmp_path, llm_for("F(near_chair_1 & X F hello)"))
    sess.submit_text("立って椅子まで歩いて挨拶して")
    snap = run_to_the_end(sess, link)
    assert snap["status"] == "done", snap["message"]
    acc = link.sim.accepted
    assert acc[:2] == [STAND_UP, BALANCE_STAND] and HELLO in acc
    assert acc.index(HELLO) > max(i for i, a in enumerate(acc) if a == MOVE)
    distance, angle = to_chair(link.sim)
    assert distance <= 1.5 and angle <= 45


def test_plain_formulas_keep_the_12_state_plant(tmp_path):
    sess, link = make_objects(tmp_path, llm_for("F hello"))
    sess.submit_text("挨拶して")
    snap = run_to_the_end(sess, link)
    assert snap["status"] == "done"
    assert len(sess.synthesis.world.states) == 12
    assert sess._grid is None and sess._exec is sess.executor
    assert snap["grid"] is None and snap["plan_cells"] == []
    assert link.sim.accepted == [STAND_UP, BALANCE_STAND, HELLO]


def turning(snap):
    return snap["current"] in ("turn_left", "turn_right")


def test_drift_to_a_neighbouring_cell_is_followed(tmp_path):
    sess, link = make_objects(tmp_path, llm_for("F near_chair_1"))
    sess.submit_text("椅子まで歩いて")
    drive_until(sess, link, turning)
    link.sim.y += 0.5                                   # one cell to the left, during the turn
    snap = run_to_the_end(sess, link)
    assert snap["status"] == "done", snap["message"]
    assert any(line.startswith("位置合わせ: ") for line in snap["debug_log"])
    distance, angle = to_chair(link.sim)
    assert distance <= 1.5 and angle <= 45


def test_drift_in_heading_stops(tmp_path):
    sess, link = make_objects(tmp_path, llm_for("F(near_chair_1 & X F hello)"))
    sess.submit_text("椅子まで歩いて挨拶して")
    drive_until(sess, link, lambda s: s["current"] == "hello")
    link.sim.yaw += 0.6                                 # 34 degrees off the heading
    snap = run_to_the_end(sess, link)
    assert snap["status"] == "error" and snap["message"] == MSG_POSITION_DRIFT
    acc = link.sim.accepted
    assert STOP_MOVE in acc[acc.index(HELLO) + 1:]
    assert snap["log"][-1] == MSG_POSITION_DRIFT
    assert snap["grid"]["goal"] is None and snap["plan_cells"] == []


class Records(DirectSimLink):
    """Keeps every command sent, with its parameters."""

    def __init__(self, sim):
        super().__init__(sim)
        self.sent = []

    def send(self, cmd):
        self.sent.append(cmd)
        super().send(cmd)


def assert_moves_ramp(sent):
    """Design §6.5 and P7: every Move in `sent` is within one 10 Hz tick of acceleration of the
    command before it, starting from 0 (a StopMove, sent at each goal by design (R11), is 0)."""
    acc_x, _, acc_z = (a * 0.1 for a in GRID_EXEC_CONFIG.acc)
    x0, z0, moves = 0.0, 0.0, 0
    for c in sent:
        if c.api_id == STOP_MOVE:
            x0, z0 = 0.0, 0.0
        elif c.api_id == MOVE:
            x, z = c.parameter["x"], c.parameter["z"]
            assert abs(x - x0) <= acc_x + 1e-3 and abs(z - z0) <= acc_z + 1e-3, (moves, c, x0, z0)
            x0, z0, moves = x, z, moves + 1
    assert moves > 0


def test_drift_out_of_the_grid_stops(tmp_path):
    sim = SimGo2()
    sim.place_objects()
    sess, link = make_objects(tmp_path, llm_for("F near_chair_1"), link=Records(sim))
    sess.submit_text("椅子まで歩いて")
    drive_until(sess, link, turning)
    link.sim.x, link.sim.y = -3.0, -3.0                 # a cell centre outside the grid
    n = len(link.sent)
    snap = run_to_the_end(sess, link)
    assert snap["status"] == "error" and snap["message"] == MSG_POSITION_DRIFT
    assert link.sim.accepted[-1] == STOP_MOVE
    # the turn ends where it is, and no walk goes after the old goal (a turn's Move has x = 0,
    # but for the x of the forward it goes on from (corners), which only falls to 0)
    xs = [c.parameter["x"] for c in link.sent[n:] if c.api_id == MOVE]
    assert all(0 <= b <= a <= MIN_SPEED + 1e-9 for a, b in zip(xs, xs[1:])) and xs[-1] == 0
    assert any(c.api_id == MOVE and c.parameter["z"] != 0 for c in link.sent[n:])
    assert snap["grid"]["goal"] is None and snap["plan_cells"] == []


class LosesBack(DirectSimLink):
    """The back LiDAR reads SENSOR_LOST while now is in `lost` (from, to)."""

    lost = (float("inf"), float("inf"))

    def reading(self, now):
        r = super().reading(now)
        if self.lost[0] <= now < self.lost[1]:
            return dataclasses.replace(r, back=SENSOR_LOST, back_dist=None)
        return r


def walk_to_the_chair_losing_the_back_sensor(tmp_path, seconds):
    sim = SimGo2()
    sim.place_objects()
    sess, link = make_objects(tmp_path, llm_for("F near_chair_1"), link=LosesBack(sim))
    sess.submit_text("椅子まで歩いて")
    drive_until(sess, link, lambda s: s["current"] == "forward")
    link.lost = (link.now, link.now + seconds)        # the forward walk ends inside it
    return sess, link


def test_a_step_ending_while_a_sensor_is_lost_waits_for_it(tmp_path):     # R11
    sess, link = walk_to_the_chair_losing_the_back_sensor(tmp_path, 5.0)
    snap = drive_until(sess, link, lambda s: s["waiting_since"] is not None and s["current"] == "forward")
    assert "センサーが途切れているので、位置合わせを待ちます" in snap["debug_log"]
    assert ("forward", "done") not in [(st["action"], st["status"]) for st in snap["steps"]]
    snap = drive_until(sess, link, lambda s: ("forward", "done") in [(st["action"], st["status"]) for st in s["steps"]])
    assert link.now >= link.lost[1]
    assert run_to_the_end(sess, link)["status"] == "done"


def test_a_sensor_lost_too_long_after_a_step_stops_the_plan(tmp_path):
    sess, link = walk_to_the_chair_losing_the_back_sensor(tmp_path, 60.0)
    snap = run_to_the_end(sess, link)
    assert snap["status"] == "error" and snap["message"] == "15秒待っても進めません"
    assert snap["log"][-1] == "15秒待っても進めません" and link.sim.accepted[-1] == STOP_MOVE


class PoseLost(DirectSimLink):
    """The pose stops coming once `gone` is set."""

    gone = False

    def pose(self):
        return None if self.gone else super().pose()


def test_no_pose_after_a_step_stops_the_plan(tmp_path):
    sim = SimGo2()
    sim.place_objects()
    sess, link = make_objects(tmp_path, llm_for("F(near_chair_1 & X F hello)"), link=PoseLost(sim))
    sess.submit_text("椅子まで歩いて挨拶して")
    drive_until(sess, link, lambda s: s["current"] == "hello")
    link.gone = True
    snap = run_to_the_end(sess, link)
    assert snap["status"] == "error" and snap["message"] == MSG_POSITION_DRIFT
    assert link.sim.accepted[-1] == STOP_MOVE


class PoseOnce(DirectSimLink):
    """The pose is known once (when the instruction arrives), then never again."""

    calls = 0

    def pose(self):
        self.calls += 1
        return super().pose() if self.calls == 1 else None


def test_no_pose_gives_the_message(tmp_path):
    sim = SimGo2()
    sim.place_objects()
    sess, link = make_objects(tmp_path, llm_for("F near_chair_1"), link=PoseOnce(sim))
    sess.submit_text("椅子まで歩いて")
    assert sess._scene.pose == (0.0, 0.0, 0.0)
    drive(sess, link, 1.0)
    snap = sess.snapshot()
    assert snap["status"] == "error" and snap["message"] == MSG_NO_POSE
    assert link.sim.accepted == []


def test_sensor_change_while_standing_up_does_not_abort(tmp_path):
    """R1: standing up, the Go2 sees the box while the supervisor is still lying (the plant
    has no bits there): no 「監督器がセンサーの変化を受け付けませんでした」. Standing, the box
    in front stops the forward walk the plan starts with, so it waits until the box goes."""
    sess, link = make_objects(tmp_path, llm_for("F near_chair_1"))
    link.sim.place_box("front", 0.6)
    sess.submit_text("椅子まで歩いて")
    snap = drive_until(sess, link, lambda s: s["status"] != "executing" and s["status"] != "thinking")
    assert snap["status"] == "waiting" and no_supervisor_complaint(snap)
    debug = snap["debug_log"]
    stood = next(i for i, line in enumerate(debug) if line.startswith("イベント: stand_to_"))
    assert "センサー: front_blocked" in debug[:stood]                # seen while the plan was lying
    assert "センサー: front_blocked" in debug[stood:]                # and again on the standing plan
    link.sim.clear_boxes()
    snap = run_to_the_end(sess, link)
    assert snap["status"] == "done" and no_supervisor_complaint(snap)
    assert not getattr(link, "hit", False)


def test_the_walk_after_the_wait_for_a_box_ramps_from_zero(tmp_path):
    """Design §6.5 and P7 (final review, Important 2): while the session waits for an unlisted
    box (the R1 amendment) nothing ticks the executor; the walk after the box goes still starts
    from 0 at the executor's acceleration."""
    sim = SimGo2()
    sim.place_objects()
    sess, link = make_objects(tmp_path, llm_for("F near_chair_1"), link=Records(sim))
    link.sim.place_box("front", 0.6)
    sess.submit_text("椅子まで歩いて")
    drive_until(sess, link, lambda s: s["status"] == "waiting")
    drive(sess, link, 3.0)
    assert sess.snapshot()["status"] == "waiting"
    n = len(link.sent)
    link.sim.clear_boxes()
    snap = run_to_the_end(sess, link)
    assert snap["status"] == "done" and no_supervisor_complaint(snap)
    assert_moves_ramp(link.sent[n:])


def test_box_on_the_route_during_a_grid_plan(tmp_path):
    sess, link = make_objects(tmp_path, llm_for("F near_chair_1"))
    sess.submit_text("椅子まで歩いて")
    drive_until(sess, link, lambda s: ("forward", "done") in [(st["action"], st["status"]) for st in s["steps"]])
    link.sim.place_box("front", 1.2)
    snap = run_to_the_end(sess, link, limit=120)
    assert not getattr(link, "hit", False) and not link.sim.collided
    assert snap["status"] == "done" or (snap["status"] == "error" and "秒待っても" in snap["message"])
    assert no_supervisor_complaint(snap)


def test_stop_mid_lap_then_a_plain_instruction(tmp_path):
    sess, link = make_objects(tmp_path, llm_for("F around_box_1", "F hello"))
    sess.submit_text("箱の周りを一周して")
    drive_until(sess, link, lambda s: sum(st["status"] == "done" for st in s["steps"]) >= 5)
    sess.stop()
    snap = sess.snapshot()
    assert snap["status"] == "idle" and link.sim.accepted[-1] == STOP_MOVE
    drive(sess, link, 1.0)
    sess.submit_text("挨拶して")
    snap = run_to_the_end(sess, link)
    assert snap["status"] == "done"
    assert len(sess.synthesis.world.states) == 12 and sess._grid is None and sess._exec is sess.executor
    assert link.sim.accepted[-1] == HELLO


def test_second_instruction_from_where_the_lap_ended(tmp_path):
    sess, link = make_objects(tmp_path, llm_for("F around_box_1", "F near_chair_1"))
    sess.submit_text("箱の周りを一周して")
    assert run_to_the_end(sess, link)["status"] == "done"
    where = (link.sim.x, link.sim.y, link.sim.yaw)
    assert math.hypot(where[0], where[1]) > 1.0 and where[2] > math.pi   # yaw is not wrapped: a whole turn
    sess.submit_text("椅子まで歩いて")
    path = []
    snap = run_to_the_end(sess, link, path=path)
    assert snap["status"] == "done", snap["message"]
    assert sess._grid.grid.anchor == where                         # the grid starts where the lap ended
    distance, angle = to_chair(link.sim)
    assert distance <= 1.5 and angle <= 45
    assert min(edge_distance(x, y, CHAIR) for x, y in path) >= 0.45
    assert not getattr(link, "hit", False)


def test_a_second_instruction_after_an_idle_gap_ramps_every_move(tmp_path):
    """Design §6.5 and P7 (final review, Important 2): after 「椅子まで歩いて」 and 5 idle seconds
    (nothing ticks the executor), the lap starts from 0 at the executor's acceleration, and so
    does every step of it (assert_moves_ramp)."""
    sim = SimGo2()
    sim.place_objects()
    sess, link = make_objects(tmp_path, llm_for("F near_chair_1", "F around_box_1"), link=Records(sim))
    sess.submit_text("椅子まで歩いて")
    assert run_to_the_end(sess, link)["status"] == "done"
    drive(sess, link, 5.0)
    n = len(link.sent)
    sess.submit_text("箱の周りを一周して")
    snap = run_to_the_end(sess, link)
    assert snap["status"] == "done", snap["message"]
    assert_moves_ramp(link.sent[n:])


def test_a_lap_from_a_rotated_off_origin_start(tmp_path):     # Review Focus 1 at session level
    sess, link = make_objects(tmp_path, llm_for("F around_box_1"))
    link.sim.x, link.sim.y, link.sim.yaw = 0.3, 0.2, 1.2
    sess.submit_text("箱の周りを一周して")
    path = [(0.3, 0.2)]
    snap = run_to_the_end(sess, link, path=path)
    assert snap["status"] == "done", snap["message"]
    assert 0.9 <= winding_number(path, (BOX.x, BOX.y)) < 1.5
    assert min(edge_distance(x, y, BOX) for x, y in path) >= 0.45
    assert not getattr(link, "hit", False) and not link.sim.collided
    assert snap["grid"]["yaw"] == pytest.approx(1.2) and snap["grid"]["origin"] == pytest.approx([0.3, 0.2])
    assert sess._grid.grid.anchor == (0.3, 0.2, 1.2)                # the pose when the plan started (R2)
    assert no_supervisor_complaint(snap)


def test_the_scene_and_the_anchor_are_taken_at_different_times(tmp_path):   # R2
    sess, link = make_objects(tmp_path, llm_for("F around_box_1"))
    link.sim.x, link.sim.y, link.sim.yaw = 0.3, 0.2, 0.4
    jobs = []
    dispatch = sess._dispatch
    sess._dispatch = lambda job: (jobs.append(job), dispatch(job))
    sess.submit_text("箱の周りを一周して")
    link.sim.yaw = 1.2                                  # the Go2 turns before the first tick
    drive(sess, link, 0.1)
    assert sess._scene.pose == (0.3, 0.2, 0.4) and sess._translation.scene.pose == (0.3, 0.2, 0.4)
    (anchor,) = [payload[2] for kind, _, payload in jobs if kind == "synthesize"]   # _get_ready's pose
    assert anchor == (0.3, 0.2, 1.2) == sess._grid.grid.anchor
    snap = sess.snapshot()
    assert snap["status"] == "executing" and snap["grid"]["yaw"] == pytest.approx(1.2)
    assert snap["grid"]["robot"] == [0, 0, 0]


def test_objects_arriving_after_a_refusal_work_on_the_next_instruction(tmp_path):
    sim = SimGo2()                                       # Gazebo has just started: no objects yet
    llm = FakeLLM([{"formula": None, "unsupported": ["箱"]},
                   {"formula": "F around_box_1", "unsupported": []}])
    sess, link = make_objects(tmp_path, llm, sim=sim)
    sess.submit_text("箱の周りを一周して")
    drive(sess, link, 0.5)
    assert sess.snapshot()["message"] == msg_not_found("box")
    sim.place_objects()
    drive(sess, link, 0.5)
    sess.submit_text("箱の周りを一周して")
    assert "- box_1（箱）：左前 2.9 m" in llm.prompts[1]
    snap = run_to_the_end(sess, link)
    assert snap["status"] == "done", snap["message"]


# ---------------- G4: the plan does not wait for the diagrams (R13, R14) ----------------
DIAGRAM_KEYS = {"FOCUS", "PLANT", "SPEC", "TRIMMED", "SUP"}


def test_plan_starts_before_the_diagrams_are_drawn(tmp_path, monkeypatch):
    real_render = session_mod.render_svg
    gate = threading.Event()

    def held_render(a, title, *rest):
        if title == "SPEC" and threading.current_thread().name == "session-worker":
            gate.wait(30)
        return real_render(a, title, *rest)

    monkeypatch.setattr(session_mod, "render_svg", held_render)
    sess, link = make_objects(tmp_path, llm_for("F near_chair_1"), background=True)
    sess.submit_text("椅子まで歩いて")
    deadline = time.monotonic() + 30
    while STAND_UP not in link.sim.accepted and time.monotonic() < deadline:
        drive(sess, link, 0.1); time.sleep(0.01)
    try:
        assert STAND_UP in link.sim.accepted and not gate.is_set()
        assert sess.diagrams == {} and sess.snapshot()["status"] == "executing"
    finally:
        gate.set()
    deadline = time.monotonic() + 30
    while set(sess.diagrams) != DIAGRAM_KEYS and time.monotonic() < deadline:
        drive(sess, link, 0.1); time.sleep(0.01)
    assert set(sess.diagrams) == DIAGRAM_KEYS


def test_grid_diagrams_are_placeholders(tmp_path):
    sess, link = make_objects(tmp_path, llm_for("F around_box_1"))
    sess.submit_text("箱の周りを一周して")
    drive(sess, link, 0.1)
    assert set(sess.diagrams) == DIAGRAM_KEYS
    for name in ("PLANT", "SUP", "TRIMMED"):
        assert TOO_BIG_TEXT in sess.diagrams[name] and MAP_HINT_TEXT in sess.diagrams[name], name
        assert 'class="too-big"' in sess.diagrams[name]
    assert f"（{sess.synthesis.plant.size} 状態）" in sess.diagrams["PLANT"]
    assert 'class="hl"' in sess.diagrams["FOCUS"] and "<svg" in sess.diagrams["SPEC"]
    focus = sess.diagrams["FOCUS"]
    drive(sess, link, 10.0)                                # past the stand: FOCUS follows, SUP stays
    assert sess.diagrams["FOCUS"] != focus and MAP_HINT_TEXT in sess.diagrams["SUP"]


def test_a_go2_formula_without_trimmed_still_gets_its_diagrams(tmp_path, monkeypatch):
    """R14: PRODUCT and TRIMMED are skipped when big (here: always); the other four are drawn."""
    monkeypatch.setattr(automata, "PRODUCT_DRAW_LIMIT", 0)
    sess, link = make_objects(tmp_path, llm_for("F hello"))
    sess.submit_text("挨拶して")
    drive(sess, link, 0.1)
    assert sess.synthesis.trimmed is None and sess._grid is None
    assert set(sess.diagrams) == DIAGRAM_KEYS
    assert TOO_BIG_TEXT in sess.diagrams["TRIMMED"] and MAP_HINT_TEXT not in sess.diagrams["TRIMMED"]
    for name in ("PLANT", "SPEC", "SUP", "FOCUS"):
        assert TOO_BIG_TEXT not in sess.diagrams[name] and 'class="node' in sess.diagrams[name], name
    assert run_to_the_end(sess, link)["status"] == "done"


# ---------------- G5: the grid and the plan's cells in the snapshot (R27) ----------------
GRID_KEYS = {"cell", "origin", "yaw", "i", "j", "blocked", "robot", "goal"}


def not_done(snap):
    return sum(st["status"] != "done" for st in snap["steps"])


def check_cells(snap):
    """plan_cells: one per step not done; the goal is the last of them."""
    assert len(snap["plan_cells"]) == not_done(snap), (snap["plan_cells"], snap["steps"])
    if snap["plan_cells"]:
        assert snap["grid"]["goal"] == snap["plan_cells"][-1]


def test_snapshot_grid_and_plan_cells(tmp_path):
    sess, link = make_objects(tmp_path, llm_for("F near_chair_1"))
    assert sess.snapshot()["grid"] is None and sess.snapshot()["plan_cells"] == []
    sess.submit_text("椅子まで歩いて")
    drive(sess, link, 0.1)
    check_cells(sess.snapshot())
    snap = drive_until(sess, link, lambda s: s["current"] == "forward")    # after the stand
    grid = snap["grid"]
    assert set(grid) == GRID_KEYS and grid["cell"] == 0.5 and grid["origin"] == [0.0, 0.0] and grid["yaw"] == 0.0
    assert grid["robot"] == [0, 0, 0] and [5, -3] in grid["blocked"]       # the chair's cell
    assert grid["i"] == [-1, 8] and grid["j"] == [-6, 1]                     # box_1 (j = 3) is outside
    check_cells(snap)
    assert snap["plan_cells"][0] == [1, 0, 0] and snap["plan_cells"][-1] == grid["goal"]
    snap = drive_until(sess, link, lambda s: s["steps"][1]["status"] == "done")   # one forward
    check_cells(snap)
    assert snap["grid"]["robot"] == [1, 0, 0]
    goal = snap["grid"]["goal"]
    snap = run_to_the_end(sess, link)
    assert snap["plan_cells"] == [] and snap["grid"]["robot"] == goal and snap["grid"]["goal"] is None
    sess.submit_text("挨拶して")                                      # a new instruction clears them
    assert sess.snapshot()["grid"] is None and sess.snapshot()["plan_cells"] == []


def test_plan_cells_follow_the_steps_while_a_box_blocks_the_way(tmp_path):
    sess, link = make_objects(tmp_path, llm_for("F near_chair_1"))
    sess.submit_text("椅子まで歩いて")
    drive_until(sess, link, lambda s: ("forward", "done") in [(st["action"], st["status"]) for st in s["steps"]])
    link.sim.place_box("front", 1.2)
    seen_off_path = False
    while sess.snapshot()["status"] not in FINAL and link.now < 120:
        drive(sess, link, 0.1)
        snap = sess.snapshot()
        check_cells(snap)
        seen_off_path |= snap["current"] == "forward" and snap["plan"][:1] != ["forward"]
    assert seen_off_path                     # the held forward was off the path for a while


def test_a_stop_mid_plan_clears_the_goal(tmp_path):      # R27: no goal without a path to it
    sess, link = make_objects(tmp_path, llm_for("F near_chair_1"))
    sess.submit_text("椅子まで歩いて")
    snap = drive_until(sess, link, lambda s: ("forward", "done") in [(st["action"], st["status"]) for st in s["steps"]])
    assert snap["grid"]["goal"] is not None and snap["plan_cells"]
    robot, blocked = snap["grid"]["robot"], snap["grid"]["blocked"]
    sess.stop()
    snap = sess.snapshot()
    assert snap["status"] == "idle" and snap["plan_cells"] == [] and snap["grid"]["goal"] is None
    assert snap["grid"]["robot"] == robot and snap["grid"]["blocked"] == blocked     # the view stays


def test_a_move_held_too_long_clears_the_goal(tmp_path):
    sess, link = make_objects(tmp_path, llm_for("F near_chair_1"))
    sess.submit_text("椅子まで歩いて")
    drive_until(sess, link, lambda s: ("forward", "done") in [(st["action"], st["status"]) for st in s["steps"]])
    link.sim.place_box("front", 1.2)                    # the next forward is held until the executor gives up
    snap = run_to_the_end(sess, link, limit=120)
    assert snap["status"] == "error" and "秒待っても" in snap["message"]
    assert snap["plan_cells"] == [] and snap["grid"]["goal"] is None and snap["grid"]["robot"] is not None


def test_a_no_path_error_still_shows_the_grid(tmp_path):
    sim = SimGo2()
    sim.objects.append(SimObject("box_1", "box", 2.5, 0.0, 1.0, 1.0))
    sess, link = make_objects(tmp_path, llm_for("F near_box_1"), sim=sim)
    sess.submit_text("箱まで歩いて")
    drive(sess, link, 1.0)
    snap = sess.snapshot()
    assert snap["message"] == MSG_NO_PATH and snap["plan_cells"] == []
    assert snap["grid"]["robot"] == [0, 0, 0] and snap["grid"]["goal"] is None and snap["grid"]["blocked"]


def test_a_sit_begun_before_an_obstacle_is_seen_completes(tmp_path):
    """The grid has no sit while an obstacle is seen; one seen during the sit (the Go2 is still
    standing for its first 2 s) is dropped with the bits, as when lying (R1)."""
    sess, link = make_objects(tmp_path, llm_for("F(near_chair_1 & X F sitting)"))
    sess.submit_text("椅子まで歩いて座って")
    drive_until(sess, link, lambda s: s["current"] == "sit")
    link.sim.place_box("front", 0.6)
    snap = run_to_the_end(sess, link)
    assert snap["status"] == "done", snap["message"]
    assert "センサー: front_blocked" in snap["debug_log"] and no_supervisor_complaint(snap)
    assert link.sim.posture == "SITTING"


class IgnoresSit(DirectSimLink):
    """A Go2 that does not take Sit (as a busy one would): it keeps standing."""

    def send(self, cmd):
        if cmd.api_id != SIT:
            super().send(cmd)


def test_an_ignored_sit_with_an_obstacle_seen_is_a_posture_error(tmp_path):
    """The Go2 keeps standing and sees a box while the plan sits down: the plant has no sit with
    the bits set, so the bits are dropped as for a sit (R1); the posture check then stops it."""
    sim = SimGo2()
    sim.place_objects()
    sess, link = make_objects(tmp_path, llm_for("F(near_chair_1 & X F sitting)"), link=IgnoresSit(sim))
    sess.submit_text("椅子まで歩いて座って")
    drive_until(sess, link, lambda s: s["current"] == "sit")
    link.sim.place_box("front", 0.6)
    snap = run_to_the_end(sess, link)
    assert snap["status"] == "error" and snap["message"].startswith("姿勢が計画と違います")
    assert no_supervisor_complaint(snap) and link.sim.accepted[-1] == STOP_MOVE
    assert "センサー: sit の途中の物は数えません" in snap["debug_log"]


class IgnoresStandDown(DirectSimLink):
    """A Go2 that does not take StandDown: it keeps standing."""

    def send(self, cmd):
        if cmd.api_id != STAND_DOWN:
            super().send(cmd)


def test_an_ignored_lie_down_with_an_obstacle_seen_is_a_posture_error(tmp_path):
    """As for sit (R1): lie_down leaves standing, so a box seen while it runs is dropped with the
    bits and the supervisor follows the lie_down; the plan's cells stay aligned with its steps
    meanwhile, and the posture check then stops the plan."""
    sim = SimGo2()
    sim.place_objects()
    sess, link = make_objects(tmp_path, llm_for("F(near_chair_1 & X F lying)"), link=IgnoresStandDown(sim))
    sess.submit_text("椅子まで歩いて伏せて")
    drive_until(sess, link, lambda s: s["current"] == "lie_down")
    link.sim.place_box("front", 0.6)
    while sess.snapshot()["status"] not in FINAL and link.now < 120:
        drive(sess, link, 0.1)
        check_cells(sess.snapshot())
    snap = sess.snapshot()
    assert snap["status"] == "error" and snap["message"].startswith("姿勢が計画と違います")
    assert "センサー: front_blocked" in snap["debug_log"] and no_supervisor_complaint(snap)
    assert "センサー: lie_down の途中の物は数えません" in snap["debug_log"]
    assert link.sim.accepted[-1] == STOP_MOVE


def test_only_sit_and_lie_down_leave_standing():          # the actions whose obstacle bits are dropped (R1)
    assert session_mod._LEAVE_STANDING == {"sit", "lie_down"}


class SeesFront(DirectSimLink):
    """The front LiDAR reads BLOCKED while `seeing` is set."""

    seeing = False

    def reading(self, now):
        r = super().reading(now)
        return dataclasses.replace(r, front=BLOCKED, front_dist=0.6) if self.seeing else r


def test_an_action_the_grid_cannot_follow_stops_the_plan(tmp_path, monkeypatch):
    """Only sit and lie_down drop the obstacle bits. Were the grid to have no turn while an obstacle
    is seen, one seen during a turn stops the plan with StopMove, as a step the supervisor cannot
    follow does: the supervisor is not taken back to where the turn began."""
    real = GridWorld._dst
    monkeypatch.setattr(GridWorld, "_dst", lambda self, state, action: (
        None if action in ("turn_left", "turn_right") and state.front else real(self, state, action)))
    sim = SimGo2()
    sim.place_objects()
    sess, link = make_objects(tmp_path, llm_for("F near_chair_1"), link=SeesFront(sim))
    sess.submit_text("椅子まで歩いて")
    turn = drive_until(sess, link, turning)["current"]
    link.seeing = True
    snap = run_to_the_end(sess, link)
    assert snap["status"] == "error" and snap["message"] == f"監督器が許していない結果になりました（{turn}）"
    assert "センサー: front_blocked" in snap["debug_log"]
    assert not any("途中の物は数えません" in line for line in snap["debug_log"])
    assert link.sim.accepted[-1] == STOP_MOVE and snap["grid"]["goal"] is None


def test_a_supervisor_that_cannot_take_the_bits_back_stops_the_plan(tmp_path, monkeypatch):
    """The state the sit began in is always in SUP; if it were not, the plan stops (no crash)."""
    monkeypatch.setattr(automata.Synthesis, "sup_state_for", lambda self, plant_state, spec_state: None)
    sim = SimGo2()
    sim.place_objects()
    sess, link = make_objects(tmp_path, llm_for("F(near_chair_1 & X F sitting)"), link=IgnoresSit(sim))
    sess.submit_text("椅子まで歩いて座って")
    drive_until(sess, link, lambda s: s["current"] == "sit")
    link.sim.place_box("front", 0.6)
    snap = run_to_the_end(sess, link)
    assert snap["status"] == "error" and snap["message"] == "監督器が許していない結果になりました（sit）"
    assert link.sim.accepted[-1] == STOP_MOVE


# ---------------- M1: continuous grid moves ----------------
class Timed(DirectSimLink):
    """Keeps (time, command, the action loaded, how many steps were done) for every command sent,
    and (time, front) for every reading. `sess` is set once the session is made."""

    def __init__(self, sim):
        super().__init__(sim)
        self.sess = None
        self.sent = []
        self.fronts = []

    def send(self, cmd):
        s = self.sess
        self.sent.append((self.now, cmd, s._action, len(s._done)))
        super().send(cmd)

    def reading(self, now):
        r = super().reading(now)
        self.fronts.append((now, r.front))
        return r


def timed_session(tmp_path, *formulas, link_cls=None):
    sim = SimGo2()
    sim.place_objects()
    link = (link_cls or Timed)(sim)
    sess, _ = make_objects(tmp_path, llm_for(*formulas), link=link)
    link.sess = sess
    return sess, link


def stop_points(actions):
    """The moves (forward / turn) of a list of actions that no next move goes on from (M1 and
    corners): a turn not followed by a forward, a forward followed by neither, the last one."""
    moves = [a for a in actions if a in ("forward", "turn_left", "turn_right")]
    return sum(not flows_on(a, b) for a, b in zip(moves, moves[1:] + [None]))


def turns_end_before_walks(link, actions):
    """C2 (the Gazebo tuning round): every forward that goes on from a turn sends x 0 until the
    turn's z is down to 0 (one Move at zero velocity, no StopMove), and x rises only after it.
    The number of such forwards."""
    walks = {}
    for _, c, a, n in link.sent:
        if c.api_id == MOVE and a == "forward":
            walks.setdefault(n, []).append(c.parameter)      # n: the steps done before it
    count = 0
    for n, ms in walks.items():
        if actions[n - 1] in ("turn_left", "turn_right"):
            k = next(i for i, m in enumerate(ms) if m["x"] > 0)
            assert k >= 1 and ms[k - 1] == {"x": 0.0, "y": 0.0, "z": 0.0}, ms[:k + 1]
            assert all(m["x"] == 0 for m in ms[:k])
            count += 1
    return count


def test_box_lap_stops_only_at_its_end(tmp_path):
    """M1 and corners on the fake Go2's lap (design §2): it still passes, and StopMove comes only
    where no move goes on (here the plan's end): no StopMove between consecutive forwards (x
    stays positive across them), after a turn that a forward follows (C2: its z ramps down to 0,
    then x up), or after a forward that a turn follows (the forward slows down, the turn goes on
    from that velocity in the next tick). One 開始 / 完了 per step, as before."""
    sess, link = timed_session(tmp_path, "F around_box_1")
    sess.submit_text("箱の周りを一周して")
    path = [(0.0, 0.0)]
    snap = run_to_the_end(sess, link, path=path)
    assert snap["status"] == "done", snap["message"]
    assert 0.9 <= winding_number(path, (BOX.x, BOX.y)) < 1.5
    assert min(edge_distance(x, y, BOX) for x, y in path) >= 0.6
    assert not getattr(link, "hit", False) and not link.sim.collided
    actions = [st["action"] for st in snap["steps"]]
    moves = [a for a in actions if a != "stand"]
    stops = [(t, a) for t, c, a, _ in link.sent if c.api_id == STOP_MOVE]
    assert len(stops) == stop_points(actions)                          # one per step before M1
    assert len(stops) <= 2 and moves.count("turn_left") + moves.count("turn_right") >= 8
    last_move = max(t for t, c, _, _ in link.sent if c.api_id == MOVE)
    assert stops[-1][0] > last_move                                    # the plan's end
    commands = [(t, c, a, n) for t, c, a, n in link.sent if c.api_id in (MOVE, STOP_MOVE)]
    boundaries = corners = 0
    for (t0, c0, a0, n0), (t1, c1, a1, n1) in zip(commands, commands[1:]):
        if n1 != n0 and flows_on(a0, a1):
            boundaries += 1
            assert c0.api_id == MOVE and c1.api_id == MOVE and t1 - t0 <= 0.1 + 1e-6, (t0, c0, a0, t1, c1, a1)
            if a0 == a1 == "forward":
                assert c0.parameter["x"] > 0 and c1.parameter["x"] > 0
            if a1 != "forward":                                        # a corner: slowed for the turn
                corners += 1
                assert 0 < c0.parameter["x"] <= MIN_SPEED + 1e-9 and c1.parameter == c0.parameter
    assert boundaries == sum(flows_on(a, b) for a, b in zip(moves, moves[1:]))
    assert corners == sum(a == "forward" and b != "forward" for a, b in zip(moves, moves[1:])) >= 4
    assert_moves_ramp([c for _, c, _, _ in link.sent])
    turns = ("turn_left", "turn_right")
    assert turns_end_before_walks(link, actions) == sum(
        a in turns and b == "forward" for a, b in zip(moves, moves[1:])) >= 4
    log = snap["log"]
    assert sum(line.startswith("開始：") for line in log) == len(actions)
    assert sum(line.startswith("完了：") for line in log) == len(actions)


def test_walk_to_the_chair_with_continuous_moves(tmp_path):
    """The chair (design §2) with M1 and corners: the four forwards flow into the turn, the turn
    into the last forward (C2: the turn ends before it walks), and only that one stops; ≤ 1.5 m
    and ≤ 45° from the chair, as before."""
    sess, link = timed_session(tmp_path, "F near_chair_1")
    sess.submit_text("椅子まで歩いて")
    path = []
    snap = run_to_the_end(sess, link, path=path)
    assert snap["status"] == "done", snap["message"]
    distance, angle = to_chair(link.sim)
    assert distance <= 1.5 and angle <= 45
    assert min(edge_distance(x, y, CHAIR) for x, y in path) >= 0.45
    actions = [st["action"] for st in snap["steps"]]
    assert actions == ["stand"] + ["forward"] * 4 + ["turn_right", "forward"]
    assert sum(c.api_id == STOP_MOVE for _, c, _, _ in link.sent) == stop_points(actions) == 1
    turn = [c.parameter for _, c, a, _ in link.sent if a == "turn_right"]
    assert 0 < turn[0]["x"] <= MIN_SPEED + 1e-9 and turn[2]["x"] == 0   # x ramps down as the turn starts
    assert turns_end_before_walks(link, actions) == 1                   # C2: the turn ends, then the walk


def forward_running(k, near=None):
    """The k-th forward of the chair plan's first straight run is running (its goal is (0.5 k, 0)),
    and, with `near`, the Go2 is that close to its goal or closer."""
    def cond(sess, link):
        return (sess._action == "forward" and sess._done.count("forward") == k - 1
                and (near is None or link.sim.x >= 0.5 * k - near))
    return cond


def drive_while(sess, link, cond, limit=60.0):
    end = link.now + limit
    while not cond(sess, link):
        assert link.now < end, sess.snapshot()["status"]
        drive(sess, link, 0.1)


def test_a_box_ahead_mid_run_stops_at_once(tmp_path):
    """M1: a box seen ahead while the Go2 walks through a cell centre at full speed stops it in
    the tick the front reads blocked, and no forward Move goes out while it does: the safety
    behaviour (never moved_forward while obstacle_front) is the same."""
    sess, link = timed_session(tmp_path, "F near_chair_1")
    sess.submit_text("椅子まで歩いて")
    drive_while(sess, link, forward_running(2))                       # a continued forward
    n = len(link.sent)
    assert not [c for _, c, _, _ in link.sent if c.api_id == STOP_MOVE]   # nothing stopped it yet
    link.sim.place_box("front", 0.9)
    snap = run_to_the_end(sess, link, limit=120)
    assert not link.sim.collided and no_supervisor_complaint(snap)
    blocked = sorted({t for t, front in link.fronts if front == BLOCKED})
    assert blocked
    first = blocked[0]
    assert STOP_MOVE in [c.api_id for t, c, _, _ in link.sent[n:] if t == first]   # at once
    assert not [c for t, c, _, _ in link.sent if t in blocked and c.api_id == MOVE and c.parameter["x"] > 0]
    assert (max(c.parameter["x"] for t, c, _, _ in link.sent[n:] if c.api_id == MOVE and t < first)
            == GRID_EXEC_CONFIG.speed[0])                             # at the cruise speed


def test_drift_during_a_continued_forward_stops(tmp_path):
    """M1: a continued forward is snapped as it passes its cell centre, with the thresholds of
    R11: 0.31 m off the nearest centre stops with MSG_POSITION_DRIFT (StopMove sent)."""
    sess, link = timed_session(tmp_path, "F near_chair_1")
    sess.submit_text("椅子まで歩いて")
    drive_while(sess, link, forward_running(2))
    assert not [c for _, c, _, _ in link.sent if c.api_id == STOP_MOVE]
    link.sim.x, link.sim.y = 1.22, 0.22                               # past (1.0, 0) and 0.31 m off it
    snap = run_to_the_end(sess, link)
    assert snap["status"] == "error" and snap["message"] == MSG_POSITION_DRIFT
    assert link.sim.accepted[-1] == STOP_MOVE
    assert [c.api_id for _, c, _, _ in link.sent].count(STOP_MOVE) == 1
    assert any(line.startswith("位置のずれ: forward の後") for line in snap["debug_log"])
    assert snap["grid"]["goal"] is None and snap["plan_cells"] == []


def test_a_jump_to_the_next_row_at_a_pass_continues_the_walk(tmp_path):
    """M1 and R10: a continued forward found one row of cells to the left as it passes is
    snapped there; the supervisor jumps, and the forward the plan has from there, in the same
    heading, continues the walk in that tick towards its own goal; the chair is reached."""
    sess, link = timed_session(tmp_path, "F near_chair_1")
    sess.submit_text("椅子まで歩いて")
    drive_while(sess, link, forward_running(2, near=0.05))
    link.sim.y += 0.3                                                 # into the next row of cells
    n = len(link.sent)
    drive_while(sess, link, lambda s, _: s._done.count("forward") == 2)
    assert "位置合わせ: (2, 0),0 -> (2, 1),0" in sess.snapshot()["debug_log"]
    now = [c for t, c, _, _ in link.sent[n:] if t == link.now]
    assert sess._action == "forward" and sess.grid_executor.current.goal_xy == pytest.approx((1.5, 0.5))
    assert [c.api_id for c in now] == [MOVE] and now[0].parameter["x"] == GRID_EXEC_CONFIG.speed[0]
    snap = run_to_the_end(sess, link)
    assert snap["status"] == "done", snap["message"]
    distance, angle = to_chair(link.sim)
    assert distance <= 1.5 and angle <= 45


def test_a_jump_to_another_heading_at_a_pass_stops_at_once(tmp_path):
    """M1 and R10: a continued forward found facing another heading as it reaches its cell centre
    is snapped to it; the step the plan has from there is no forward in the old heading, so the
    Go2 gets StopMove in that tick and the next step starts GRID_SETTLE_S later."""
    sess, link = timed_session(tmp_path, "F near_chair_1")
    sess.submit_text("椅子まで歩いて")
    drive_while(sess, link, forward_running(2))
    link.sim.x, link.sim.yaw = 1.0, 1.4                               # at (1, 0), 80 degrees left
    n = len(link.sent)
    drive_while(sess, link, lambda s, _: s._done.count("forward") == 2)
    jumped = link.now
    assert "位置合わせ: (2, 0),0 -> (2, 0),1" in sess.snapshot()["debug_log"]
    assert [c.api_id for t, c, _, _ in link.sent[n:] if t == jumped] == [STOP_MOVE]
    assert sess._action != "forward"
    snap = run_to_the_end(sess, link)
    assert snap["status"] == "done", snap["message"]
    assert min(t for t, _, _, _ in link.sent[n:] if t > jumped) >= jumped + GRID_SETTLE_S - 1e-9
    distance, angle = to_chair(link.sim)
    assert distance <= 1.5 and angle <= 45


class Unsettled(Timed):
    """The posture reads UNKNOWN (a real Go2 between two error_codes) while now is in `unknown`."""

    unknown = (float("inf"), float("inf"))

    def posture(self):
        return UNKNOWN if self.unknown[0] <= self.now < self.unknown[1] else super().posture()


def test_a_pass_with_nothing_loaded_next_stops_at_once(tmp_path):
    """M1: a forward flowed on, but the session cannot load the next one in that tick (here the
    posture reads UNKNOWN for a moment): StopMove in that tick; the walk goes on from 0 once
    it is known, and the plan ends."""
    sess, link = timed_session(tmp_path, "F near_chair_1", link_cls=Unsettled)
    sess.submit_text("椅子まで歩いて")
    drive_while(sess, link, forward_running(1, near=0.08))
    link.unknown = (link.now + 0.05, link.now + 0.45)
    drive_while(sess, link, lambda s, _: s._done.count("forward") == 1)
    passed = link.now
    assert link.unknown[0] <= passed < link.unknown[1] and sess._action is None
    assert [c.api_id for t, c, _, _ in link.sent if t == passed] == [STOP_MOVE]
    n = len(link.sent)
    snap = run_to_the_end(sess, link)
    assert snap["status"] == "done", snap["message"]
    assert_moves_ramp([c for _, c, _, _ in link.sent[n:]])
    first_move = min(t for t, c, _, _ in link.sent[n:] if c.api_id == MOVE)
    assert first_move >= max(link.unknown[1], passed + GRID_SETTLE_S) - 1e-9


def test_plan_cells_stay_right_through_continuous_moves(tmp_path):
    """R27 with M1: every tick, plan_cells has one cell per step not done, and while a move runs
    its first cell is where that move goes from the cell the page shows the Go2 in."""
    sess, link = make_objects(tmp_path, llm_for("F around_box_1"))
    sess.submit_text("箱の周りを一周して")
    ahead = {0: (1, 0), 1: (0, 1), 2: (-1, 0), 3: (0, -1)}
    moves = 0
    while sess.snapshot()["status"] not in FINAL and link.now < 300:
        drive(sess, link, 0.1)
        snap = sess.snapshot()
        check_cells(snap)
        i, j, h = snap["grid"]["robot"] if snap["grid"] else (0, 0, 0)
        if snap["current"] == "forward":
            assert snap["plan_cells"][0] == [i + ahead[h][0], j + ahead[h][1], h]
            moves += 1
        elif snap["current"] in ("turn_left", "turn_right"):
            assert snap["plan_cells"][0] == [i, j, (h + (1 if snap["current"] == "turn_left" else -1)) % 4]
    assert snap["status"] == "done" and moves > 0
