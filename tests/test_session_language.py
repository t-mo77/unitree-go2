"""The session with the language side of open object kinds (Task 1.4): a refusal forced by the reply field
unresolved_objects or by the kind guard never stands the Go2 up, and the snapshot hides secret-like text in
the attempts' unresolved entries (session._redacted on dicts)."""

import json

from sim.sim_core import SimGo2
from voice_ltl import session as session_mod
from voice_ltl.llm import MSG_REDACTED, FakeLLM
from voice_ltl.objects import msg_not_found
from voice_ltl.session import DirectSimLink, Session

FAKE_TOKEN = "Zk3" * 12                            # made up: a long run of letters and digits


def make(tmp_path, llm):
    """A session on a fake Go2 with the default objects (box_1 and chair_1) placed."""
    sim = SimGo2()
    sim.place_objects()
    link = DirectSimLink(sim)
    return Session(link, llm, None, tmp_path, background=False), link


def drive(session, link, seconds, dt=0.1):
    end = link.now + seconds
    while link.now < end - 1e-9:
        now = round(link.now + dt, 6)
        link.advance(now)
        session.tick(now)


def test_unresolved_object_refusal_does_not_stand_up(tmp_path):
    reply = {"formula": None, "unsupported": ["観葉植物"], "substitutions": [],
             "unresolved_objects": [{"said": "観葉植物", "label": "potted_plant"}]}
    sess, link = make(tmp_path, FakeLLM([reply]))
    drive(sess, link, 0.2)
    sess.submit_text("観葉植物まで行って")
    drive(sess, link, 1.0)
    snap = sess.snapshot()
    assert snap["status"] == "error" and snap["message"] == msg_not_found("potted_plant", "観葉植物")
    assert link.sim.accepted == []
    assert snap["unsupported"] == ["観葉植物"]
    assert snap["attempts"] == [{"formula": None, "unsupported": ["観葉植物"], "errors": [],
                                 "unresolved": [{"said": "観葉植物", "label": "potted_plant"}]}]


def test_an_entry_beside_a_formula_does_not_stand_up(tmp_path):     # O8
    reply = {"formula": "F near_box_1", "unsupported": [], "substitutions": [],
             "unresolved_objects": [{"said": "観葉植物", "label": "potted_plant"}]}
    sess, link = make(tmp_path, FakeLLM([reply]))
    drive(sess, link, 0.2)
    sess.submit_text("観葉植物まで行って")
    drive(sess, link, 1.0)
    snap = sess.snapshot()
    assert snap["status"] == "error" and snap["message"] == msg_not_found("potted_plant", "観葉植物")
    assert link.sim.accepted == [] and snap["unsupported"] == ["観葉植物"]
    assert snap["attempts"][0]["formula"] == "F near_box_1" and snap["attempts"][0]["unsupported"] == ["観葉植物"]


def test_kind_guard_does_not_stand_up(tmp_path):             # O35
    sess, link = make(tmp_path, FakeLLM([{"formula": "F near_box_1", "unsupported": []}]))
    drive(sess, link, 0.2)
    sess.submit_text("ゴミ箱まで行って")
    drive(sess, link, 1.0)
    snap = sess.snapshot()
    assert snap["status"] == "error" and snap["message"] == msg_not_found("trash_can", "ゴミ箱")
    assert link.sim.accepted == [] and snap["formula"] is None
    assert snap["attempts"] == [{"formula": "F near_box_1", "unsupported": [],
                                 "errors": [msg_not_found("trash_can", "ゴミ箱")], "unresolved": []}]


def test_kind_guard_on_a_phrasing_off_the_list_does_not_stand_up(tmp_path):   # pre-final review F1
    for instruction in ("ゴミ箱に行け", "ゴミ箱に移動して"):
        sess, link = make(tmp_path, FakeLLM([{"formula": "F near_box_1", "unsupported": []}]))
        drive(sess, link, 0.2)
        sess.submit_text(instruction)
        drive(sess, link, 1.0)
        snap = sess.snapshot()
        assert snap["status"] == "error" and snap["message"] == msg_not_found("trash_can", "ゴミ箱"), instruction
        assert link.sim.accepted == [] and snap["formula"] is None


def test_kind_guard_with_the_particle_dropped_does_not_stand_up(tmp_path):   # pre-final review round 2, finding 1
    # before: status done, F near_box_1 accepted and the Go2 at (1.99, 0.48) beside box_1
    for instruction in ("ゴミ箱行って", "ゴミ箱向かって"):
        sess, link = make(tmp_path, FakeLLM([{"formula": "F near_box_1", "unsupported": []}]))
        drive(sess, link, 0.2)
        sess.submit_text(instruction)
        drive(sess, link, 1.0)
        snap = sess.snapshot()
        assert snap["status"] == "error" and snap["message"] == msg_not_found("trash_can", "ゴミ箱"), instruction
        assert link.sim.accepted == [] and snap["formula"] is None


def test_snapshot_redacts_unresolved_said(tmp_path):
    reply = {"formula": None, "unsupported": ["箱"], "substitutions": [],
             "unresolved_objects": [{"said": FAKE_TOKEN, "label": "box"}]}
    sess, link = make(tmp_path, FakeLLM([reply]))
    sess.submit_text(f"{FAKE_TOKEN}の箱まで行って")      # said must be words of the instruction
    drive(sess, link, 0.5)
    snap = sess.snapshot()
    entry = snap["attempts"][0]["unresolved"][0]
    assert entry == {"said": MSG_REDACTED, "label": "box"}
    blob = json.dumps({k: snap[k] for k in ("attempts", "unsupported", "message", "log")}, ensure_ascii=False)
    assert FAKE_TOKEN[:9] not in blob


def test_redacted_reaches_strings_inside_dicts_and_lists():
    value = {"said": FAKE_TOKEN, "label": None, "more": [{"x": FAKE_TOKEN}, 3, "箱"]}
    assert session_mod._redacted(value) == {"said": MSG_REDACTED, "label": None,
                                            "more": [{"x": MSG_REDACTED}, 3, "箱"]}
    assert session_mod._redacted([FAKE_TOKEN, "椅子"]) == [MSG_REDACTED, "椅子"]
    assert session_mod._redacted("椅子") == "椅子" and session_mod._redacted(None) is None
