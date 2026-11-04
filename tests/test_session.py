import time

import pytest
from go2_nl_control.actions import (BALANCE_STAND, DANCE1, HELLO, MOVE, SIT, STAND_UP,
                                    STOP_MOVE, STRETCH)

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
    # A box already too close: no controllable path, so the session waits (lying) and gives up.
    link.sim.place_box("front", 0.6)
    sess.submit_text("前に進んで"); drive(sess, link, 10)
    assert sess.snapshot()["status"] == "waiting" and link.sim.accepted == []
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
    assert link.sim.accepted == [] and link.sim.x == 0.0              # 後退しない（立ちもしない）
    link.sim.clear_boxes(); link.sim.place_box("back", 0.9)           # 0.75 m 先：下がり始められる
    drive(sess, link, 8)
    snap = sess.snapshot()
    assert snap["status"] == "executing" and link.sim.accepted[-1] == STOP_MOVE
    assert not link.sim.collided and -0.35 < link.sim.x < -0.2        # 0.5 m 手前で止まった
    assert any("後方に障害物" in line for line in snap["log"])
    link.sim.clear_boxes(); drive(sess, link, 10)                    # どければ下がる
    assert sess.snapshot()["status"] == "done"
    assert link.sim.x == pytest.approx(-0.5, abs=0.05)


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


def test_sup_drawing_does_not_block_tick(tmp_path):
    session, link = make(tmp_path, llm_for("F hello"), background=True)
    session.submit_text("手を振って")
    t0 = time.monotonic()
    while session.snapshot()["status"] == "thinking" and time.monotonic() - t0 < 30:
        time.sleep(0.05)
        link.advance(round(link.now + 0.1, 6))
        session.tick(link.now)
    slowest = 0.0
    for _ in range(150):
        link.advance(round(link.now + 0.1, 6))
        t = time.monotonic()
        session.tick(link.now)
        slowest = max(slowest, time.monotonic() - t)
        if session.snapshot()["status"] == "done":
            break
    assert session.snapshot()["status"] == "done"
    assert slowest < 0.1
    deadline = time.monotonic() + 10
    while "hl" not in session.diagrams["SUP"] and time.monotonic() < deadline:
        time.sleep(0.05)
