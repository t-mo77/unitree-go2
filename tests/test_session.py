import dataclasses
import re
import threading
import time

import pytest
from go2_nl_control.actions import (BALANCE_STAND, DANCE1, HELLO, LYING, MOVE, RISE_SIT, SIT,
                                    STAND_DOWN, STAND_UP, STANDING, STOP_MOVE, STRETCH, UNKNOWN)
from go2_nl_control.supervisor import BLOCKED

import voice_ltl.session as session_mod
from sim.sim_core import SimGo2
from voice_ltl.llm import FakeLLM, LLMError
from voice_ltl.session import DirectSimLink, Session

MSG_POSTURE_UNKNOWN = "Go2 の姿勢がわかりません。リモコンで伏せさせてから、もう一度話してください"


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
    assert any("後方に障害物" in line for line in snap["debug_log"])
    assert "一時停止：後退（後方に障害物があります）" in snap["log"]
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
    assert "センサー: front_blocked" in snap["debug_log"] and "前方に物が出た" in snap["log"]
    link.box = False; drive(sess, link, 10)                            # 物が消えれば進む
    snap = sess.snapshot()
    assert snap["status"] == "done" and "前方の物が消えた" in snap["log"]
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
    snap = sess.snapshot()
    assert snap["status"] == "idle" and snap["stop_word"] == "止まって"
    sess.stop()                                                        # the button: no word
    assert sess.snapshot()["stop_word"] is None


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
    ("10回", True), ("七回", True), ("２メートル", True), ("約45度", True),
    ("障害物の周りを一周する", False), ("この周りを一周する（周回）", False),
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
    # Between two Steps (during a Step the posture is checked too: test_session_safety.py)
    sess, link = make(tmp_path, llm_for("F(hello & F danced)"))
    sess.submit_text("挨拶してからダンスして")
    drive_until(sess, link, lambda s: [st["status"] for st in s["steps"]][:2] == ["done", "done"])
    assert link.sim.accepted == [STAND_UP, BALANCE_STAND, HELLO]       # 挨拶は Go2 の中で終わった
    link.sim.error_code = 1004                                         # 強制的に伏せ
    drive(sess, link, 20)
    snap = sess.snapshot()
    assert snap["status"] == "error" and "姿勢が計画と違います" in snap["message"]
    assert DANCE1 not in link.sim.accepted
    assert [(s["action"], s["status"]) for s in snap["steps"]] == [("stand", "done"), ("hello", "done")]


def test_diagrams_are_made_and_sup_follows_the_supervisor(tmp_path):
    session, link = make(tmp_path, llm_for("F hello"))
    assert session.diagrams == {}
    session.submit_text("手を振って")
    v0 = session.diagram_version
    drive(session, link, 0.1)
    assert set(session.diagrams) == {"FOCUS", "PLANT", "SPEC", "TRIMMED", "SUP"}
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


# ---------------- R28: a posture change or gesture is not cut short ----------------
@pytest.mark.parametrize("first,busy,second,then", [
    ("F hello", HELLO, "F stretched", [STRETCH]),          # hello: 5 s planned
    ("F sitting", SIT, "F hello", [RISE_SIT, HELLO]),      # sit: 5 s planned, then the Go2 sits
])
def test_new_instruction_during_gesture_or_posture_change_waits_for_it(tmp_path, first, busy,
                                                                     second, then):
    sess, link = make(tmp_path, llm_for(first, second))
    sess.submit_text("一つ目の指示"); drive(sess, link, 5.0)
    assert link.sim.accepted == [STAND_UP, BALANCE_STAND, busy]       # sent at 4.2 s
    sess.submit_text("二つ目の指示")
    assert link.sim.accepted[-1] == STOP_MOVE                          # StopMove at once
    drive(sess, link, 4.0)                                             # 9.0 s: its 5 s are not over
    snap = sess.snapshot()
    assert link.sim.accepted == [STAND_UP, BALANCE_STAND, busy, STOP_MOVE]
    assert snap["status"] == "thinking" and snap["message"] == "前の動作が終わるのを待っています"
    drive(sess, link, 15)
    assert link.sim.accepted == [STAND_UP, BALANCE_STAND, busy, STOP_MOVE, *then]
    assert sess.snapshot()["status"] == "done"


def test_stop_during_gesture_then_new_instruction_waits_for_the_gesture(tmp_path):
    sess, link = make(tmp_path, llm_for("F hello", "F stretched"))
    sess.submit_text("挨拶して"); drive(sess, link, 5.0)
    sess.submit_text("止まって")
    assert link.sim.accepted == [STAND_UP, BALANCE_STAND, HELLO, STOP_MOVE]
    drive(sess, link, 1.0)
    sess.submit_text("伸びをして"); drive(sess, link, 3.0)               # 9.0 s: hello's 5 s are not over
    assert link.sim.accepted == [STAND_UP, BALANCE_STAND, HELLO, STOP_MOVE]
    drive(sess, link, 10)
    assert link.sim.accepted == [STAND_UP, BALANCE_STAND, HELLO, STOP_MOVE, STRETCH]
    assert sess.snapshot()["status"] == "done"


class IgnoresSit(DirectSimLink):
    """A Go2 that does not take Sit (as a busy one would): it keeps standing."""

    def send(self, cmd):
        if cmd.api_id != SIT:
            super().send(cmd)


def test_done_only_when_the_go2_is_in_the_planned_posture(tmp_path):
    link = IgnoresSit(SimGo2())
    sess = Session(link, llm_for("F sitting"), None, tmp_path, background=False)
    sess.submit_text("座って"); drive(sess, link, 15)
    snap = sess.snapshot()
    assert snap["status"] == "error" and "姿勢が計画と違います" in snap["message"]
    assert link.sim.posture == STANDING and "終わりました" not in snap["log"]


# ---------------- R27: an unknown posture at the start of a plan ----------------
def test_standing_lock_gets_balance_stand_and_is_planned_as_standing(tmp_path):
    sim = SimGo2()
    sim.error_code = 1002                                              # Standing Lock: UNKNOWN
    sess, link = make(tmp_path, llm_for("F hello"), sim=sim)
    sess.submit_text("挨拶して"); drive(sess, link, 0.1)
    assert link.sim.accepted == [BALANCE_STAND]                        # not StandUp
    drive(sess, link, 10)
    assert link.sim.accepted == [BALANCE_STAND, HELLO]
    assert sess.snapshot()["status"] == "done"


def test_standing_lock_after_a_stop_during_stand_then_lie_down(tmp_path):
    sess, link = make(tmp_path, llm_for("F hello", "F lying"))
    sess.submit_text("挨拶して"); drive(sess, link, 1.0)                # StandUp at 0.1 s
    sess.submit_text("止まって"); drive(sess, link, 3.0)                # StandUp still ends: 1002
    assert link.sim.error_code == 1002
    sess.submit_text("伏せて"); drive(sess, link, 10)
    assert link.sim.accepted == [STAND_UP, STOP_MOVE, BALANCE_STAND, STAND_DOWN]
    assert sess.snapshot()["status"] == "done" and link.sim.posture == LYING


class NoErrorCode(DirectSimLink):
    """Like RosLink: there is no error_code() to read (R27 then only waits)."""

    error_code = None


def test_unknown_posture_is_refused_after_3s(tmp_path):
    sim = SimGo2()
    sim.error_code = 1002
    link = NoErrorCode(sim)
    sess = Session(link, llm_for("F hello"), None, tmp_path, background=False)
    sess.submit_text("挨拶して"); drive(sess, link, 2.8)
    snap = sess.snapshot()
    assert snap["status"] == "thinking" and snap["message"] == "Go2 の姿勢が決まるのを待っています"
    drive(sess, link, 0.5)
    snap = sess.snapshot()
    assert snap["status"] == "error" and snap["message"] == MSG_POSTURE_UNKNOWN
    drive(sess, link, 5)
    assert link.sim.accepted == [] and sess.diagrams == {}


def test_unknown_posture_that_settles_within_3s_is_planned_from_there(tmp_path):
    sim = SimGo2()
    sim.error_code = 2000                                              # not in the table: UNKNOWN
    sess, link = make(tmp_path, llm_for("F hello"), sim=sim)
    sess.submit_text("挨拶して"); drive(sess, link, 2.0)
    assert link.sim.accepted == [] and sess.snapshot()["status"] == "thinking"
    link.sim.error_code = 1004                                         # now lying
    drive(sess, link, 15)
    assert link.sim.accepted == [STAND_UP, BALANCE_STAND, HELLO]
    assert sess.snapshot()["status"] == "done"


def test_a_new_instruction_while_waiting_for_the_posture_replaces_it(tmp_path):
    sim = SimGo2()
    sim.error_code = 2000
    sess, link = make(tmp_path, llm_for("F hello", "F lying"), sim=sim)
    sess.submit_text("挨拶して"); drive(sess, link, 2.0)
    sess.submit_text("伏せて"); drive(sess, link, 2.0)                  # 4 s after the first: its wait is gone
    assert sess.snapshot()["status"] == "thinking"
    link.sim.error_code = 1004
    drive(sess, link, 1.0)
    snap = sess.snapshot()
    assert snap["status"] == "done" and snap["formula"] == "F lying" and link.sim.accepted == []


# ---------------- stop words as voice input writes them ----------------
@pytest.mark.parametrize("word", ["ｽﾄｯﾌﾟ", "止まって止まって止まって止まって", "ＳＴＯＰ！",
                                  "　止まって。", "ストップ、ストップ、ストップ！"])
def test_stop_word_variants_stop_without_the_llm(tmp_path, word):
    llm = llm_for("F(moved_forward & X F moved_forward)")
    sess, link = make(tmp_path, llm)
    sess.submit_text("1メートル進んで"); drive(sess, link, 5.5)
    assert link.sim.accepted[-1] == MOVE
    sent = len(link.sim.accepted)
    sess.submit_text(word)
    assert link.sim.accepted[sent:] == [STOP_MOVE] and len(llm.prompts) == 1
    assert sess.snapshot()["status"] == "idle"


def test_negated_stop_word_is_an_instruction(tmp_path):
    llm = llm_for("F hello")
    sess, link = make(tmp_path, llm)
    sess.submit_text("止まらないで")
    assert len(llm.prompts) == 1 and sess.snapshot()["status"] == "thinking"


# ---------------- the snapshot for the page (S2a) ----------------
def drive_until(session, link, cond, limit=60.0):
    """drive() one tick at a time until cond(snapshot) holds; the snapshot then."""
    end = link.now + limit
    while link.now < end - 1e-9:
        drive(session, link, 0.1)
        snap = session.snapshot()
        if cond(snap):
            return snap
    raise AssertionError(f"not reached in {limit} s: {session.snapshot()['status']}")


def step_states(snap):
    return [(s["action"], s["label"], s["status"]) for s in snap["steps"]]


def test_steps_show_the_whole_plan_done_current_and_pending(tmp_path):
    sess, link = make(tmp_path, llm_for("F(hello & F danced)"))
    assert sess.snapshot()["steps"] == []
    sess.submit_text("立って挨拶してから、ダンスして"); drive(sess, link, 0.1)
    assert step_states(sess.snapshot()) == [
        ("stand", "立つ", "current"), ("hello", "挨拶", "pending"), ("dance", "ダンス", "pending")]
    snap = drive_until(sess, link, lambda s: s["current"] == "hello")
    assert step_states(snap) == [
        ("stand", "立つ", "done"), ("hello", "挨拶", "current"), ("dance", "ダンス", "pending")]
    snap = drive_until(sess, link, lambda s: s["status"] == "done")
    assert [st for _, _, st in step_states(snap)] == ["done", "done", "done"]
    sess.submit_text("伸びをして")                                     # a new instruction: no steps yet
    assert sess.snapshot()["steps"] == []


def test_log_is_for_people_and_debug_log_keeps_the_details(tmp_path):
    sess, link = make(tmp_path, llm_for("F(hello & F danced)"))
    sess.submit_text("立って挨拶してから、ダンスして"); drive(sess, link, 40)
    snap = sess.snapshot()
    assert snap["status"] == "done"
    assert snap["log"] == [
        "指示：「立って挨拶してから、ダンスして」", "式：F(hello & F danced)",
        "計画：立つ → 挨拶 → ダンス",
        "開始：立つ", "完了：立つ（伏せ → 立ち）",
        "開始：挨拶", "完了：挨拶",
        "開始：ダンス", "完了：ダンス",
        "終わりました"]
    debug = snap["debug_log"]                                          # the lines as they were
    assert debug[0] == "指示: 立って挨拶してから、ダンスして" and "式: F(hello & F danced)" in debug
    assert any(line.startswith("監督器: ") for line in debug)
    assert "イベント: " + "stand_to_standing" in debug and "完了: hello" in debug
    assert debug.count("計画をすべて終えました") == 3 and debug[-1] == "終わりました"


def test_stop_word_in_the_log(tmp_path):
    sess, link = make(tmp_path, llm_for("F(moved_forward & X F moved_forward)"))
    sess.submit_text("1メートル進んで"); drive(sess, link, 5.5)
    sess.submit_text("止まって")
    log = sess.snapshot()["log"]
    assert log[-3:] == ["止める言葉：「止まって」", "中止：前進（止める）", "止めました"]


class Whisper:
    """A transcriber like stt.Transcriber: it has model_size, and takes a while."""

    model_size = "small"

    def __call__(self, audio):
        time.sleep(0.25)
        return "挨拶して"


def test_stt_is_the_model_and_seconds_of_the_transcript(tmp_path):
    sess, link = make(tmp_path, llm_for("F hello", "F stretched"), transcribe=Whisper())
    assert sess.snapshot()["stt"] is None
    sess.submit_audio(b"RIFF....")
    snap = sess.snapshot()
    assert snap["transcript"] == "挨拶して" and snap["stt"]["model"] == "small"
    assert 0.2 <= snap["stt"]["seconds"] < 1.0
    sess.submit_text("伸びをして")                                     # typed: no transcription
    assert sess.snapshot()["stt"] is None


class LoadingWhisper(Whisper):
    """Like the first Transcriber call: 0.25 s of it is the one-time model load."""

    last_load_seconds = 0.2


def test_stt_seconds_leave_out_the_model_load(tmp_path):
    sess, _ = make(tmp_path, llm_for("F hello"), transcribe=LoadingWhisper())
    sess.submit_audio(b"RIFF....")
    stt = sess.snapshot()["stt"]
    assert stt["load_seconds"] == 0.2 and stt["seconds"] == 0.1      # 0.25 s total - 0.2 s load
    sess, _ = make(tmp_path, llm_for("F hello"), transcribe=Whisper())
    sess.submit_audio(b"RIFF....")
    assert "load_seconds" not in sess.snapshot()["stt"]


def test_server_now_is_the_clock_of_waiting_since(tmp_path):
    sess, link = make(tmp_path, llm_for("F moved_forward", "F(moved_forward & X F moved_forward)"))
    assert sess.snapshot()["server_now"] is None
    link.sim.place_box("front", 0.6)
    sess.submit_text("前に進んで")
    snap = drive_until(sess, link, lambda s: s["status"] == "waiting")
    assert snap["server_now"] == pytest.approx(link.now)
    assert snap["server_now"] - snap["waiting_since"] < 0.5


def test_link_kind_llm_backend_and_block_timeout(tmp_path):
    from voice_ltl.llm import ClaudeCLI, CodexCLI

    sess, _ = make(tmp_path, llm_for())
    snap = sess.snapshot()
    assert snap["link_kind"] == "fake" and snap["llm_backend"] is None and snap["block_timeout"] == 15
    link = DirectSimLink(SimGo2())
    assert Session(link, ClaudeCLI(), None, tmp_path, background=False,
                   link_kind="gazebo").snapshot()["link_kind"] == "gazebo"
    assert Session(link, ClaudeCLI(), None, tmp_path, background=False).snapshot()["llm_backend"] == "claude"
    assert Session(link, CodexCLI(), None, tmp_path, background=False).snapshot()["llm_backend"] == "codex"


def test_waiting_since_for_a_wait_and_for_a_held_move(tmp_path):
    sess, link = make(tmp_path, llm_for("F moved_forward", "F(moved_forward & X F moved_forward)"))
    link.sim.place_box("front", 0.6)
    sess.submit_text("前に進んで")
    snap = drive_until(sess, link, lambda s: s["status"] == "waiting")
    assert snap["waiting_since"] == pytest.approx(link.now)            # the session waits from now
    since = snap["waiting_since"]
    drive(sess, link, 3)
    assert sess.snapshot()["waiting_since"] == since                   # not moved by later ticks
    link.sim.clear_boxes()
    snap = drive_until(sess, link, lambda s: s["status"] == "done")
    assert snap["waiting_since"] is None
    link.sim.place_box("front", 1.2)                                   # on the way: the move is held
    sess.submit_text("1メートル進んで")
    snap = drive_until(sess, link, lambda s: s["waiting_since"] is not None)
    assert snap["status"] == "executing" and snap["current"] == "forward"
    assert snap["waiting_since"] == pytest.approx(link.now)
    link.sim.clear_boxes()
    snap = drive_until(sess, link, lambda s: s["waiting_since"] is None, limit=2)
    assert snap["current"] == "forward"


def test_focus_diagram_follows_the_supervisor(tmp_path):
    sess, link = make(tmp_path, llm_for("F hello"))
    sess.submit_text("手を振って"); drive(sess, link, 0.1)
    focus = sess.diagrams["FOCUS"]
    assert "<svg" in focus and sup_highlight(focus)[0] == [sess.runtime.state]
    assert "完了" not in focus
    drive_until(sess, link, lambda s: s["status"] == "done")
    focus = sess.diagrams["FOCUS"]
    assert sup_highlight(focus)[0] == [sess.runtime.state] and "完了" in focus


# ---------------- a posture that is UNKNOWN for a moment (real Go2, S2a) ----------------
class UnknownForAMoment(DirectSimLink):
    """posture() reads UNKNOWN until `unknown_until`, as a real Go2 does while its error_code
    changes from one value to the next."""

    def __init__(self):
        super().__init__(SimGo2())
        self.unknown_until = float("-inf")

    def posture(self):
        return UNKNOWN if self.now < self.unknown_until - 1e-9 else super().posture()


@pytest.mark.parametrize("formula,at,then", [
    ("F(hello & F danced)", lambda s: s["steps"][0]["status"] == "done", [HELLO, DANCE1]),  # between steps
    ("F hello", lambda s: all(st["status"] == "done" for st in s["steps"]) and s["steps"], []),  # at the end
])
def test_a_short_unknown_posture_is_waited_for(tmp_path, formula, at, then):
    link = UnknownForAMoment()
    sess = Session(link, llm_for(formula), None, tmp_path, background=False)
    sess.submit_text("指示")
    drive_until(sess, link, at)
    sent = list(link.sim.accepted)
    link.unknown_until = link.now + 0.8
    drive(sess, link, 0.7)
    snap = sess.snapshot()
    assert snap["posture"] == UNKNOWN and snap["status"] == "executing"   # not an error yet
    assert link.sim.accepted == sent                                    # nothing new sent meanwhile
    drive(sess, link, 40)
    snap = sess.snapshot()
    assert snap["status"] == "done", snap["message"]
    assert link.sim.accepted == sent + then


def test_an_unknown_posture_longer_than_1s_aborts(tmp_path):
    link = UnknownForAMoment()
    sess = Session(link, llm_for("F(hello & F danced)"), None, tmp_path, background=False)
    sess.submit_text("挨拶してからダンスして")
    drive_until(sess, link, lambda s: s["steps"][0]["status"] == "done")
    sent = list(link.sim.accepted)
    link.unknown_until = link.now + 5
    drive(sess, link, 0.9)
    assert sess.snapshot()["status"] == "executing"
    drive(sess, link, 0.3)
    snap = sess.snapshot()
    assert snap["status"] == "error" and snap["message"] == "姿勢が計画と違います（計画: 立ち / 実際: 不明）"
    assert link.sim.accepted == sent + [STOP_MOVE] and HELLO not in link.sim.accepted
