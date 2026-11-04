import dataclasses
import re
import threading
import time

import pytest
from go2_nl_control.actions import (BALANCE_STAND, DANCE1, HELLO, MOVE, SIT, STAND_UP,
                                    STANDING, STOP_MOVE, STRETCH)
from go2_nl_control.supervisor import BLOCKED

import voice_ltl.session as session_mod
from sim.sim_core import SimGo2
from voice_ltl.llm import FakeLLM, LLMError
from voice_ltl.session import DirectSimLink, Session


def llm_for(*formulas):
    return FakeLLM([{"formula": f, "unsupported": []} for f in formulas])


def make(tmp_path, llm, sim=None, transcribe=None, background=False):
    link = DirectSimLink(sim or SimGo2())
    return Session(link, llm, transcribe, tmp_path, background=background), link


def drive(session, link, seconds, dt=0.1):
    """Advance the simulated time: link.advance(now) then session.tick(now), every dt."""
    end = link.now + seconds
    while link.now < end - 1e-9:
        now = round(link.now + dt, 6)
        link.advance(now)
        session.tick(now)


def gestures(link):
    return [a for a in link.sim.accepted if a != MOVE and a != STOP_MOVE]


@pytest.mark.parametrize("formula,expect", [
    ("F hello", [STAND_UP, BALANCE_STAND, HELLO]),
    ("F(hello & F danced)", [STAND_UP, BALANCE_STAND, HELLO, DANCE1]),
    ("(!sitting U hello) & F sitting", [STAND_UP, BALANCE_STAND, HELLO, SIT]),
    ("G !danced & F stretched", [STAND_UP, BALANCE_STAND, STRETCH]),
    ("F(hello & X F(hello & X F hello))", [STAND_UP, BALANCE_STAND, HELLO, HELLO, HELLO]),
])
def test_acceptance_gestures(tmp_path, formula, expect):
    sess, link = make(tmp_path, llm_for(formula))
    sess.submit_text("指示"); drive(sess, link, 60)
    assert [a for a in link.sim.accepted if a != MOVE and a != STOP_MOVE] == expect
    assert sess.snapshot()["status"] == "done"


def test_two_meters_stops_for_box_then_continues(tmp_path):
    sess, link = make(tmp_path, llm_for("F(moved_forward & X F(moved_forward & X F(moved_forward & X F moved_forward)))"))
    sess.submit_text("2メートル進んで"); drive(sess, link, 6)          # 立って歩き始める
    link.sim.place_box("front", 1.2); drive(sess, link, 10)
    assert not link.sim.collided and sess.snapshot()["status"] == "executing"
    assert link.sim.accepted[-1] == STOP_MOVE                           # 途中で止まった
    link.sim.clear_boxes(); drive(sess, link, 20)
    assert sess.snapshot()["status"] == "done" and link.sim.x == pytest.approx(2.0, abs=0.15)


def test_box_never_cleared_aborts_after_15s(tmp_path):
    sess, link = make(tmp_path, llm_for("F moved_forward", "F(moved_forward & X F moved_forward)"))
    # A box already too close. Lying, the readings count as clear (R24), so the Go2 stands;
    # standing, it sees the box: no controllable path, so the session waits and gives up.
    link.sim.place_box("front", 0.6)
    sess.submit_text("前に進んで"); drive(sess, link, 10)
    assert sess.snapshot()["status"] == "waiting"
    assert link.sim.accepted == [STAND_UP, BALANCE_STAND]
    drive(sess, link, 10)
    snap = sess.snapshot()
    assert snap["status"] == "error" and "秒待っても" in snap["message"]
    assert MOVE not in link.sim.accepted
    # A box ahead on the way: the walk pauses in front of it and the executor gives up.
    link.sim.clear_boxes(); link.sim.place_box("front", 1.2)
    sess.submit_text("1メートル進んで"); drive(sess, link, 12)
    assert sess.snapshot()["status"] == "executing" and link.sim.accepted[-1] == STOP_MOVE
    drive(sess, link, 15)
    snap = sess.snapshot()
    assert snap["status"] == "error" and "秒待っても" in snap["message"]
    assert not link.sim.collided and 0.5 < link.sim.x < 0.7


def test_back_box_holds_backward(tmp_path):
    sess, link = make(tmp_path, llm_for("F moved_backward"))
    link.sim.place_box("back", 0.6)
    sess.submit_text("後ろに下がって"); drive(sess, link, 8)
    assert sess.snapshot()["status"] == "waiting"
    assert link.sim.accepted == [STAND_UP, BALANCE_STAND]             # 立ってから箱を見て（R24）
    assert link.sim.x == 0.0                                           # 後退しない
    link.sim.clear_boxes(); link.sim.place_box("back", 0.9)           # 0.75 m 先：下がり始められる
    drive(sess, link, 8)
    snap = sess.snapshot()
    assert snap["status"] == "executing" and link.sim.accepted[-1] == STOP_MOVE
    assert not link.sim.collided and -0.35 < link.sim.x < -0.2        # 0.5 m 手前で止まった
    assert any("後方に障害物" in line for line in snap["log"])
    link.sim.clear_boxes(); drive(sess, link, 10)                    # どければ下がる
    assert sess.snapshot()["status"] == "done"
    assert link.sim.x == pytest.approx(-0.5, abs=0.05)


class GroundSeenWhileLying(DirectSimLink):
    """Lying, the LiDAR sees the ground and the Go2's own legs: the front reads「あり」.
    While `box` is True something is really in front, so standing it reads「あり」too."""

    def __init__(self):
        super().__init__(SimGo2())
        self.box = True

    def reading(self, now):
        r = super().reading(now)
        if self.box or self.sim.posture != STANDING:
            return dataclasses.replace(r, front=BLOCKED, front_dist=0.3)
        return r


def test_readings_while_not_standing_count_as_clear(tmp_path):
    # R24: the start is planned as clear; standing, the box is a front_blocked that holds forward.
    link = GroundSeenWhileLying()
    sess = Session(link, llm_for("F moved_forward"), None, tmp_path, background=False)
    drive(sess, link, 1)
    assert sess.snapshot()["front"] == BLOCKED                         # 伏せたまま「あり」
    sess.submit_text("前に進んで"); drive(sess, link, 10)
    snap = sess.snapshot()
    assert link.sim.accepted == [STAND_UP, BALANCE_STAND]              # 立ち上がった
    assert link.sim.posture == STANDING and snap["status"] == "waiting"  # 物があるので待つ
    assert "センサー: front_blocked" in snap["log"]
    link.box = False; drive(sess, link, 10)                            # 物が消えれば進む
    assert sess.snapshot()["status"] == "done"
    assert link.sim.accepted[:3] == [STAND_UP, BALANCE_STAND, MOVE]
    assert link.sim.x == pytest.approx(0.5, abs=0.05)


def test_stop_word_stops_immediately(tmp_path):
    llm = llm_for("F(moved_forward & X F moved_forward)")
    sess, link = make(tmp_path, llm)
    sess.submit_text("1メートル進んで"); drive(sess, link, 5.5)
    assert link.sim.accepted[-1] == MOVE                               # 歩いている
    calls, sent = len(llm.prompts), len(link.sim.accepted)
    sess.submit_text("止まって")
    assert link.sim.accepted[sent:] == [STOP_MOVE]                     # 直後の送信が StopMove
    x = link.sim.x
    drive(sess, link, 5)
    assert link.sim.accepted[sent:] == [STOP_MOVE]                     # 計画を捨てたので続きは送らない
    assert len(llm.prompts) == calls and link.sim.x == pytest.approx(x, abs=0.01)
    assert sess.snapshot()["status"] == "idle"


def test_new_instruction_replaces_plan(tmp_path):
    sess, link = make(tmp_path, llm_for("F hello", "F stretched"))
    sess.submit_text("挨拶して"); drive(sess, link, 7.5)
    assert sess.snapshot()["current"] == "hello"                       # 挨拶の途中
    sess.submit_text("伸びをして"); drive(sess, link, 15)
    assert link.sim.accepted == [STAND_UP, BALANCE_STAND, HELLO, STOP_MOVE, STRETCH]
    snap = sess.snapshot()
    assert snap["status"] == "done" and snap["formula"] == "F stretched"


def test_unsupported_and_unsatisfiable_do_not_move(tmp_path):
    llm = FakeLLM([{"formula": None, "unsupported": ["キッチン"]},
                   {"formula": "F sitting & G !sitting", "unsupported": []}])
    sess, link = make(tmp_path, llm)
    sess.submit_text("キッチンへ行って"); drive(sess, link, 3)
    snap = sess.snapshot()
    assert snap["status"] == "error" and "キッチン" in snap["message"]
    assert snap["unsupported"] == ["キッチン"] and link.sim.accepted == []
    sess.submit_text("座って、でも絶対に座らないで"); drive(sess, link, 3)
    snap = sess.snapshot()
    assert snap["status"] == "error" and "満たせません" in snap["message"]
    assert link.sim.accepted == []


@pytest.mark.parametrize("word,is_number", [
    ("2階", False), ("3番目の部屋", False), ("1号室", False),
    ("0.7メートル", True), ("0.7 m", True), ("30cm", True), ("45度", True),
    ("10回", True), ("七回", True), ("２メートル", True),
])
def test_number_message_only_for_a_number_with_a_unit(tmp_path, word, is_number):
    # 「この数は指定できません」 is for a distance, angle or count; 「2階」 is a word not in the vocabulary.
    sess, link = make(tmp_path, FakeLLM([{"formula": None, "unsupported": [word]}]))
    sess.submit_text(f"{word}へ"); drive(sess, link, 0.2)
    message = sess.snapshot()["message"]
    assert f"「{word}」" in message
    assert ("この数は指定できません" in message) == is_number
    assert ("単語帳にない" in message) == (not is_number)


def test_empty_transcript_does_not_call_llm(tmp_path):
    llm = llm_for("F hello")
    sess, link = make(tmp_path, llm, transcribe=lambda audio: "")
    sess.submit_text("  ")
    assert llm.prompts == [] and sess.snapshot()["message"] == "聞き取れませんでした"
    sess.submit_audio(b"")                                             # 無音は "" になる
    drive(sess, link, 1)
    snap = sess.snapshot()
    assert llm.prompts == [] and snap["message"] == "聞き取れませんでした"
    assert snap["status"] == "idle" and link.sim.accepted == []


def test_newer_instruction_wins_over_slow_translation(tmp_path):
    llm = FakeLLM([{"formula": "F danced", "unsupported": []},
                   {"formula": "F hello", "unsupported": []}], delay_s=0.5)
    sess, link = make(tmp_path, llm, background=True)
    sess.submit_text("ダンスして")
    deadline = time.monotonic() + 5
    while not llm.prompts and time.monotonic() < deadline:            # 1つ目が翻訳中になるまで
        time.sleep(0.01)
    sess.submit_text("挨拶して")
    deadline = time.monotonic() + 10
    while sess.snapshot()["status"] == "thinking" and time.monotonic() < deadline:
        drive(sess, link, 0.1); time.sleep(0.02)
    drive(sess, link, 30)
    snap = sess.snapshot()
    assert len(llm.prompts) == 2 and snap["formula"] == "F hello" and snap["transcript"] == "挨拶して"
    assert link.sim.accepted == [STAND_UP, BALANCE_STAND, HELLO] and snap["status"] == "done"


def test_llm_failure_reports_and_stays_idle(tmp_path):
    sess, link = make(tmp_path, FakeLLM([LLMError("LLM にログインできていません")]))
    sess.submit_text("挨拶して"); drive(sess, link, 3)
    snap = sess.snapshot()
    assert snap["status"] == "error" and snap["message"] == "LLM にログインできていません"
    assert link.sim.accepted == []


def test_posture_mismatch_aborts(tmp_path):
    sess, link = make(tmp_path, llm_for("F(hello & F danced)"))
    sess.submit_text("挨拶してからダンスして"); drive(sess, link, 8)
    assert link.sim.accepted == [STAND_UP, BALANCE_STAND, HELLO]       # 挨拶は Go2 の中で終わった
    link.sim.error_code = 1004                                         # 強制的に伏せ
    drive(sess, link, 20)
    snap = sess.snapshot()
    assert snap["status"] == "error" and "姿勢が計画と違います" in snap["message"]
    assert DANCE1 not in link.sim.accepted
    assert any("完了: hello" in line for line in snap["log"])


def test_diagrams_are_made_and_sup_follows_the_supervisor(tmp_path):
    session, link = make(tmp_path, llm_for("F hello"))
    assert session.diagrams == {}
    session.submit_text("手を振って")
    v0 = session.diagram_version
    drive(session, link, 0.1)
    assert set(session.diagrams) == {"PLANT", "SPEC", "TRIMMED", "SUP"}
    assert all("<svg" in svg for svg in session.diagrams.values())
    assert session.diagram_version > v0
    sup0, v1 = session.diagrams["SUP"], session.diagram_version
    drive(session, link, 20)
    assert session.snapshot()["status"] == "done"
    assert session.diagrams["SUP"] != sup0 and session.diagram_version > v1
    assert session.diagrams["PLANT"] and "hl" in session.diagrams["SUP"]


def sup_highlight(svg):
    """(the highlighted states, the number of highlighted arrows) of a SUP drawing."""
    states = [int(s) for s in re.findall(r'class="node hl">\s*<title>(\d+)</title>', svg)]
    return states, len(re.findall(r'class="edge(?: dashed)? hl"', svg))


def test_sup_drawing_does_not_block_tick(tmp_path, monkeypatch):
    real_render = session_mod.render_svg
    gate = threading.Event()   # SUP redraws on the drawing thread wait here until the test opens it
    drawn_by_tick = []

    def slow_render(a, title, state=None, events=()):
        if threading.current_thread().name == "session-diagrams":
            gate.wait(30)
        elif threading.current_thread() is threading.main_thread():
            drawn_by_tick.append(title)
        return real_render(a, title, state, events)

    monkeypatch.setattr(session_mod, "render_svg", slow_render)
    session, link = make(tmp_path, llm_for("F hello"), background=True)
    session.submit_text("手を振って")
    deadline = time.monotonic() + 30
    while session.snapshot()["status"] == "thinking" and time.monotonic() < deadline:
        drive(session, link, 0.1); time.sleep(0.02)
    v_start = session.diagram_version
    drive(session, link, 20)                   # the whole plan, while every SUP redraw is held
    assert session.snapshot()["status"] == "done"
    assert link.sim.accepted == [STAND_UP, BALANCE_STAND, HELLO]
    assert drawn_by_tick == [] and session.diagram_version == v_start
    gate.set()                                 # now the held redraws finish, the newest last
    final = ([session.runtime.state], 0)       # the accepting state, no plan arrows left
    deadline = time.monotonic() + 10
    while sup_highlight(session.diagrams["SUP"]) != final and time.monotonic() < deadline:
        time.sleep(0.02)
    assert sup_highlight(session.diagrams["SUP"]) == final
    assert session.diagram_version > v_start
    assert session.runtime.state in session.runtime.sup.marked


def test_no_diagrams_for_a_plan_made_stale_during_synthesis(tmp_path, monkeypatch):
    drawn = []
    monkeypatch.setattr(session_mod, "render_svg",
                        lambda a, title, *rest: drawn.append(title) or "<svg/>")
    sess, link = make(tmp_path, llm_for("F hello"))
    real_synthesize = session_mod.synthesize

    def synthesize_then_stop(*args):
        syn = real_synthesize(*args)
        sess.stop()                            # 「止まって」 while the supervisor was being made
        return syn

    monkeypatch.setattr(session_mod, "synthesize", synthesize_then_stop)
    sess.submit_text("挨拶して"); drive(sess, link, 2)
    assert drawn == [] and sess.diagrams == {}
    assert sess.snapshot()["status"] == "idle" and link.sim.accepted == [STOP_MOVE]


class SlotRaceSession(Session):
    """When the drawing thread reads _sup_slot, `race` runs once there: a new redraw request
    arrives at that very moment, as a tick could make it."""

    race = None

    @property
    def _sup_slot(self):
        value = self.__dict__.get("_slot")
        if value is not None and self.race is not None and threading.current_thread().name == "draw":
            race, self.race = self.race, None
            race()
        return value

    @_sup_slot.setter
    def _sup_slot(self, value):
        self.__dict__["_slot"] = value


def test_a_sup_redraw_requested_while_one_is_taken_is_not_lost(tmp_path, monkeypatch):
    link = DirectSimLink(SimGo2())
    sess = SlotRaceSession(link, llm_for("F hello"), None, tmp_path, background=False)
    sess.submit_text("挨拶して"); drive(sess, link, 0.1)    # the plan runs; its diagrams are drawn
    drawn = []

    def record(a, title, state=None, events=()):
        drawn.append(state)
        return f"<svg>{title} {state}</svg>"

    monkeypatch.setattr(session_mod, "render_svg", record)
    sess.background = True                                  # SUP redraws go through the slot now
    threading.Thread(target=sess._draw_loop, name="draw", daemon=True).start()

    def tick_requests_a_redraw():                           # as tick() would, from its own thread
        t = threading.Thread(target=sess._redraw_sup)
        t.start(); t.join(0.3)

    sess.race = tick_requests_a_redraw
    sess._sup_slot = (sess._syn_id, sess.runtime.sup, -1, [])  # an older request, not yet drawn
    sess._sup_wake.set()
    newest = f"<svg>SUP {sess.runtime.state}</svg>"
    deadline = time.monotonic() + 5
    while sess.diagrams["SUP"] != newest and time.monotonic() < deadline:
        time.sleep(0.01)
    assert drawn == [-1, sess.runtime.state] and sess.diagrams["SUP"] == newest
