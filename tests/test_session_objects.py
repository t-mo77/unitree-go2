"""The session with objects: the Scene frozen per instruction, the live object list on the page,
object formulas on the grid plant (snap and jump after each step), the diagrams after the plan
starts, and the grid and the plan's cells in the snapshot (Task G of the objects plan)."""

import dataclasses
import math
import threading

from go2_nl_control.actions import BALANCE_STAND, HELLO, MOVE, STAND_UP, STOP_MOVE
from go2_nl_control.supervisor import SENSOR_LOST

from sim.sim_core import SimGo2, SimObject
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


def test_drift_out_of_the_grid_stops(tmp_path):
    sess, link = make_objects(tmp_path, llm_for("F near_chair_1"))
    sess.submit_text("椅子まで歩いて")
    drive_until(sess, link, turning)
    link.sim.x, link.sim.y = -3.0, -3.0                 # a cell centre outside the grid
    n = len(link.sim.accepted)
    snap = run_to_the_end(sess, link)
    assert snap["status"] == "error" and snap["message"] == MSG_POSITION_DRIFT
    assert link.sim.accepted[-1] == STOP_MOVE and MOVE not in link.sim.accepted[n:][-1:]


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
