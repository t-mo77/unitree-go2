"""The session with objects: the Scene frozen per instruction, the live object list on the page,
object formulas on the grid plant (snap and jump after each step), the diagrams after the plan
starts, and the grid and the plan's cells in the snapshot (Task G of the objects plan)."""

import threading

from go2_nl_control.actions import STANDING
from go2_nl_control.perception import PerceptionConfig

from sim.sim_core import SimGo2, SimObject
from voice_ltl.llm import FakeLLM
from voice_ltl.objects import msg_not_found
from voice_ltl.session import DirectSimLink, Session


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
    path, a list, gets the Go2's (x, y) after every tick."""
    end = link.now + seconds
    while link.now < end - 1e-9:
        now = round(link.now + dt, 6)
        link.advance(now)
        session.tick(now)
        if path is not None:
            path.append((link.sim.x, link.sim.y))


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
