"""Session safety fixes from the final review (Task FXA).

- SAFETY-1 / UI-3 (R36): a stop word is never stale; a recording bumps the generation only once
  its text is known to be an instruction, so an empty or failed recording cancels nothing.
- SAFETY-2: during a Step the Go2 must stay in a posture its action allows.
- REAL-2: another program sending commands to the Go2 refuses each instruction.
- TD-1: an action with no Japanese name shows its own name.
- R37: secret-like text is hidden in the snapshot.
- TD-5: a sensor dropout through the Session holds a move at once and aborts after 15 s.
"""

import json
import threading
import time

import pytest
from go2_nl_control.actions import (BALANCE_STAND, DANCE1, HELLO, MOVE, STAND_DOWN, STAND_UP,
                                    STANDING, STOP_MOVE, UNKNOWN)
from go2_nl_control.supervisor import SENSOR_LOST

import voice_ltl.session as session_mod
from sim.sim_core import SimGo2
from voice_ltl.llm import MSG_REDACTED, FakeLLM
from voice_ltl.sensing import Reading
from voice_ltl.session import DirectSimLink, Session

ONE_M = "F(moved_forward & X F moved_forward)"
TWO_M = "F(moved_forward & X F(moved_forward & X F(moved_forward & X F moved_forward)))"
MSG_POSTURE_CHANGED = "姿勢が変わったので止めました"
MSG_OTHER_SENDERS = "ほかのプログラムが Go2 に指令を送っています"


def llm_for(*formulas):
    return FakeLLM([{"formula": f, "unsupported": []} for f in formulas])


class LockedLink(DirectSimLink):
    """DirectSimLink for background=True: the worker thread may send StopMove while the test
    thread advances the fake Go2."""

    def __init__(self, sim=None):
        super().__init__(sim or SimGo2())
        self._mutex = threading.Lock()

    def advance(self, now):
        with self._mutex:
            super().advance(now)

    def send(self, cmd):
        with self._mutex:
            super().send(cmd)


def make(tmp_path, llm, link=None, transcribe=None, background=False):
    link = link or (LockedLink() if background else DirectSimLink(SimGo2()))
    return Session(link, llm, transcribe, tmp_path, background=background), link


def drive(session, link, seconds, dt=0.1):
    """Advance the simulated time: link.advance(now) then session.tick(now), every dt."""
    end = link.now + seconds
    while link.now < end - 1e-9:
        now = round(link.now + dt, 6)
        link.advance(now)
        session.tick(now)


def drive_until(session, link, cond, limit=60.0):
    """drive() one tick at a time until cond(snapshot) holds; the snapshot then."""
    end = link.now + limit
    while link.now < end - 1e-9:
        drive(session, link, 0.1)
        snap = session.snapshot()
        if cond(snap):
            return snap
    raise AssertionError(f"not reached in {limit} s: {session.snapshot()['status']}")


def live_until(session, link, cond, limit=30.0):
    """For background=True: drive() while the worker runs, until cond(snapshot) holds."""
    deadline = time.monotonic() + limit
    while time.monotonic() < deadline:
        drive(session, link, 0.1)
        time.sleep(0.01)
        snap = session.snapshot()
        if cond(snap):
            return snap
    raise AssertionError(f"not reached in {limit} s: {session.snapshot()['status']}")


class LLMByInstruction:
    """An LLM that answers by the instruction it is given: table[instruction] is the formula."""

    def __init__(self, table):
        self.table = table
        self.instructions = []

    def __call__(self, prompt, schema):
        instruction = prompt.split("## Query\n指示: ", 1)[1].split("\n", 1)[0]
        self.instructions.append(instruction)
        return {"formula": self.table[instruction], "unsupported": []}


def gestures(link):
    return [a for a in link.sim.accepted if a not in (MOVE, STOP_MOVE)]


class ScriptedWhisper:
    """A transcriber like stt.Transcriber. script: recording -> (seconds it takes, its text or
    the exception it raises, or a threading.Event to wait for first)."""

    model_size = "small"

    def __init__(self, script):
        self.script = script

    def __call__(self, audio):
        wait, text = self.script[audio]
        if isinstance(wait, threading.Event):
            assert wait.wait(30)
        else:
            time.sleep(wait)
        if isinstance(text, Exception):
            raise text
        return text


# ---------------- SAFETY-1 / UI-3 (R36) ----------------
@pytest.mark.parametrize("second,message", [
    ((0.0, ""), "聞き取れませんでした"),
    ((0.0, RuntimeError("PyAV が読めません")), "音声を文字にできませんでした"),
], ids=["empty", "failed"])
def test_a_stop_recording_then_an_empty_or_failed_one_still_stops(tmp_path, second, message):
    whisper = ScriptedWhisper({b"stop": (0.5, "止まって"), b"tap": second})
    sess, link = make(tmp_path, llm_for(TWO_M), transcribe=whisper, background=True)
    sess.submit_text("2メートル進んで")
    live_until(sess, link, lambda s: link.sim.accepted[-1:] == [MOVE])     # walking
    sess.submit_audio(b"stop")
    time.sleep(0.05)
    sess.submit_audio(b"tap")                                                # a short tap after it
    snap = live_until(sess, link, lambda s: s["status"] in ("idle", "done", "error")
                      and s["message"] == message)                     # the tap is done too
    assert snap["status"] == "idle" and snap["log"][-2:] == ["止めました", message]
    assert snap["stop_word"] == "止まって" and snap["transcript"] == "止まって"
    stopped = len(link.sim.accepted)
    assert STOP_MOVE in link.sim.accepted[-1:]
    x = link.sim.x
    live_until(sess, link, lambda s: link.now > 30, limit=60)
    assert link.sim.accepted[stopped:] == [] and link.sim.x == pytest.approx(x, abs=0.01)


def test_a_stop_word_is_never_stale(tmp_path):
    # 「止まって」 was said first and is still being transcribed when an instruction is typed:
    # the stop still stops (the safe side), and the typed instruction does not run.
    gate = threading.Event()
    llm = llm_for(ONE_M)
    sess, link = make(tmp_path, llm, transcribe=ScriptedWhisper({b"stop": (gate, "止まって")}),
                      background=True)
    sess.submit_audio(b"stop")
    sess.submit_text("1メートル進んで")
    gate.set()
    snap = live_until(sess, link, lambda s: s["status"] == "idle" and s["message"] == "止めました")
    drive(sess, link, 10)
    assert MOVE not in link.sim.accepted and STAND_UP not in link.sim.accepted
    assert sess.snapshot()["status"] == "idle" and snap["stop_word"] == "止まって"


@pytest.mark.parametrize("tap", [(0.0, ""), (0.0, RuntimeError("PyAV が読めません"))],
                         ids=["empty", "failed"])
def test_an_empty_or_failed_recording_keeps_the_instruction_being_translated(tmp_path, tap):
    llm = FakeLLM([{"formula": "F hello", "unsupported": []}], delay_s=0.3)
    sess, link = make(tmp_path, llm, transcribe=ScriptedWhisper({b"tap": tap}), background=True)
    sess.submit_text("立って挨拶して")
    deadline = time.monotonic() + 5
    while not llm.prompts and time.monotonic() < deadline:             # being translated
        time.sleep(0.01)
    sess.submit_audio(b"tap")
    snap = live_until(sess, link, lambda s: s["status"] in ("idle", "done", "error"))
    assert snap["status"] == "done" and snap["formula"] == "F hello"
    assert snap["transcript"] == "立って挨拶して" and snap["stt"] is None
    assert gestures(link) == [STAND_UP, BALANCE_STAND, HELLO]


def test_an_empty_recording_while_executing_keeps_the_transcript_and_the_plan(tmp_path):
    sess, link = make(tmp_path, llm_for(ONE_M), transcribe=ScriptedWhisper({b"tap": (0.0, "")}))
    sess.submit_text("1メートル進んで"); drive(sess, link, 5.5)
    assert link.sim.accepted[-1] == MOVE
    sess.submit_audio(b"tap")
    snap = sess.snapshot()
    assert snap["status"] == "executing" and snap["current"] == "forward"
    assert snap["transcript"] == "1メートル進んで" and snap["stt"] is None
    assert snap["message"] == "" and snap["log"][-1] == "聞き取れませんでした"
    drive(sess, link, 15)
    snap = sess.snapshot()
    assert snap["status"] == "done" and link.sim.x == pytest.approx(1.0, abs=0.05)


def test_a_failing_transcriber_reports_and_does_not_call_the_llm(tmp_path):
    llm = llm_for("F hello")
    sess, link = make(tmp_path, llm, transcribe=ScriptedWhisper({b"x": (0.0, RuntimeError("壊れた"))}))
    sess.submit_audio(b"x"); drive(sess, link, 1)
    snap = sess.snapshot()
    assert snap["status"] == "idle" and snap["message"] == "音声を文字にできませんでした"
    assert llm.prompts == [] and link.sim.accepted == []


def test_status_is_transcribing_until_the_text_is_known(tmp_path):
    gate = threading.Event()
    sess, link = make(tmp_path, llm_for(), transcribe=ScriptedWhisper({b"tap": (gate, "")}),
                      background=True)
    sess.submit_audio(b"tap")
    assert sess.snapshot()["status"] == "transcribing"
    gate.set()
    snap = live_until(sess, link, lambda s: s["status"] != "transcribing")
    assert snap["status"] == "idle" and snap["message"] == "聞き取れませんでした"
    assert snap["transcript"] == "" and snap["stt"] is None


def test_a_typed_instruction_after_a_recording_wins(tmp_path):
    gate = threading.Event()
    llm = llm_for("F hello")
    sess, link = make(tmp_path, llm, transcribe=ScriptedWhisper({b"a": (gate, "伸びをして")}),
                      background=True)
    sess.submit_audio(b"a")
    sess.submit_text("挨拶して")                                       # typed after the recording
    assert sess.snapshot()["status"] == "thinking"
    gate.set()
    snap = live_until(sess, link, lambda s: s["status"] in ("idle", "done", "error"))
    assert snap["status"] == "done" and snap["transcript"] == "挨拶して" and len(llm.prompts) == 1
    assert gestures(link) == [STAND_UP, BALANCE_STAND, HELLO]


def test_a_later_recording_wins_over_an_earlier_one(tmp_path):
    llm = LLMByInstruction({"ダンスして": "F danced", "挨拶して": "F hello"})
    whisper = ScriptedWhisper({b"a": (0.2, "ダンスして"), b"b": (0.0, "挨拶して")})
    sess, link = make(tmp_path, llm, transcribe=whisper, background=True)
    sess.submit_audio(b"a")
    sess.submit_audio(b"b")
    snap = live_until(sess, link, lambda s: s["status"] in ("idle", "done", "error"))
    assert snap["status"] == "done" and snap["formula"] == "F hello"
    assert snap["transcript"] == "挨拶して" and llm.instructions[-1] == "挨拶して"
    assert gestures(link) == [STAND_UP, BALANCE_STAND, HELLO]


# ---------------- SAFETY-2: the posture during a Step ----------------
def test_a_remote_lie_down_mid_walk_aborts_and_the_go2_does_not_walk_on(tmp_path):
    sess, link = make(tmp_path, llm_for(TWO_M))
    sess.submit_text("2メートル進んで")
    drive_until(sess, link, lambda s: link.sim.x >= 0.3)
    assert link.sim.accepted[-1] == MOVE
    link.sim.handle(STAND_DOWN, "", link.now)                          # the remote: lie down
    drive(sess, link, 2)
    snap = sess.snapshot()
    assert snap["status"] == "error" and snap["message"] == MSG_POSTURE_CHANGED
    assert link.sim.accepted[-1] == STOP_MOVE
    assert "中止：前進（姿勢が変わったので止めました）" in snap["log"]
    x = link.sim.x
    link.sim.handle(STAND_UP, "", link.now); drive(sess, link, 2)      # the remote: stand again
    link.sim.handle(BALANCE_STAND, "", link.now); drive(sess, link, 5)
    assert link.sim.posture == STANDING
    assert link.sim.accepted[-1] == BALANCE_STAND                      # nothing sent after it
    assert link.sim.x == pytest.approx(x, abs=0.01)                    # it does not walk on


def test_a_posture_change_during_a_gesture_aborts(tmp_path):
    sess, link = make(tmp_path, llm_for("F(hello & F danced)"))
    sess.submit_text("挨拶してからダンスして")
    drive_until(sess, link, lambda s: s["current"] == "hello")
    drive(sess, link, 1.0)
    link.sim.error_code = 1004                                          # lying, mid-gesture
    drive(sess, link, 0.2)
    snap = sess.snapshot()
    assert snap["status"] == "error" and snap["message"] == MSG_POSTURE_CHANGED
    assert link.sim.accepted[-1] == STOP_MOVE and DANCE1 not in link.sim.accepted


class UnknownForAWhile(DirectSimLink):
    """posture() reads UNKNOWN until `unknown_until`, as a real Go2 does between two error_codes."""

    def __init__(self):
        super().__init__(SimGo2())
        self.unknown_until = float("-inf")

    def posture(self):
        return UNKNOWN if self.now < self.unknown_until - 1e-9 else super().posture()


def test_a_short_unknown_posture_during_a_move_is_waited_for(tmp_path):
    link = UnknownForAWhile()
    sess = Session(link, llm_for("F moved_forward"), None, tmp_path, background=False)
    sess.submit_text("前に進んで")
    drive_until(sess, link, lambda s: s["current"] == "forward" and link.sim.x > 0.05)
    link.unknown_until = link.now + 0.8
    drive(sess, link, 0.7)
    snap = sess.snapshot()
    assert snap["posture"] == UNKNOWN and snap["status"] == "executing"
    drive(sess, link, 10)
    snap = sess.snapshot()
    assert snap["status"] == "done", snap["message"]
    assert link.sim.x == pytest.approx(0.5, abs=0.05)


def test_an_unknown_posture_during_a_move_longer_than_1s_aborts(tmp_path):
    link = UnknownForAWhile()
    sess = Session(link, llm_for(ONE_M), None, tmp_path, background=False)
    sess.submit_text("1メートル進んで")
    drive_until(sess, link, lambda s: s["current"] == "forward" and link.sim.x > 0.05)
    link.unknown_until = link.now + 5
    drive(sess, link, 0.9)
    assert sess.snapshot()["status"] == "executing"
    drive(sess, link, 0.3)
    snap = sess.snapshot()
    assert snap["status"] == "error" and snap["message"] == MSG_POSTURE_CHANGED
    assert link.sim.accepted[-1] == STOP_MOVE
    x = link.sim.x
    drive(sess, link, 5)
    assert link.sim.x == pytest.approx(x, abs=0.01)


# ---------------- REAL-2: another program sending commands ----------------
class WithOtherSenders(DirectSimLink):
    def __init__(self):
        super().__init__(SimGo2())
        self.senders = 0

    def other_senders(self):
        return self.senders


def test_direct_sim_link_has_no_other_senders():
    assert DirectSimLink(SimGo2()).other_senders() == 0


def test_another_sender_refuses_an_instruction(tmp_path):
    link = WithOtherSenders()
    link.senders = 1
    llm = llm_for("F hello")
    sess = Session(link, llm, ScriptedWhisper({b"a": (0.0, "挨拶して")}), tmp_path, background=False)
    for submit in (lambda: sess.submit_text("挨拶して"), lambda: sess.submit_audio(b"a")):
        submit(); drive(sess, link, 5)
        snap = sess.snapshot()
        assert snap["status"] == "error" and snap["message"].startswith(MSG_OTHER_SENDERS)
        assert llm.prompts == [] and link.sim.accepted == []
    link.senders = 0                                                   # that program ended
    sess.submit_text("挨拶して"); drive(sess, link, 20)
    assert sess.snapshot()["status"] == "done" and gestures(link) == [STAND_UP, BALANCE_STAND, HELLO]


def test_with_another_sender_a_stop_word_still_stops(tmp_path):
    link = WithOtherSenders()
    sess = Session(link, llm_for(ONE_M), None, tmp_path, background=False)
    sess.submit_text("1メートル進んで"); drive(sess, link, 5.5)
    assert link.sim.accepted[-1] == MOVE
    link.senders = 1
    sent = len(link.sim.accepted)
    sess.submit_text("止まって")
    assert link.sim.accepted[sent:] == [STOP_MOVE]
    assert sess.snapshot()["status"] == "idle"


def test_with_another_sender_a_new_instruction_ends_the_running_plan(tmp_path):
    link = WithOtherSenders()
    llm = llm_for(ONE_M, "F hello")
    sess = Session(link, llm, None, tmp_path, background=False)
    sess.submit_text("1メートル進んで"); drive(sess, link, 5.5)
    link.senders = 1
    sent = len(link.sim.accepted)
    sess.submit_text("挨拶して")
    assert link.sim.accepted[sent:] == [STOP_MOVE]                     # ours stops; nothing new
    drive(sess, link, 10)
    snap = sess.snapshot()
    assert link.sim.accepted[sent:] == [STOP_MOVE] and len(llm.prompts) == 1
    assert snap["status"] == "error" and snap["message"].startswith(MSG_OTHER_SENDERS)


# ---------------- TD-1: an action with no Japanese name ----------------
@pytest.mark.parametrize("formula,action", [("F hello", "hello"), ("F moved_forward", "forward")])
def test_an_action_without_a_japanese_name_shows_its_own_name(tmp_path, monkeypatch, formula,
                                                              action):
    monkeypatch.delitem(session_mod.ACTION_JA, action)     # as if added by README §5 only
    sess, link = make(tmp_path, llm_for(formula))
    sess.submit_text("指示"); drive(sess, link, 0.1)
    assert [s["label"] for s in sess.snapshot()["steps"]] == ["立つ", action]
    drive(sess, link, 30)
    snap = sess.snapshot()
    assert snap["status"] == "done", snap["message"]
    assert f"計画：立つ → {action}" in snap["log"]
    assert f"開始：{action}" in snap["log"] and f"完了：{action}" in snap["log"]


# ---------------- R37: secret-like text is hidden on the page ----------------
FAKE_KEY = "sk-ant-oat01-" + "Q7xR2" * 8          # made up, shaped like an API key
FAKE_TOKEN = "Zk3" * 12                            # made up: a long run of letters and digits


def test_secret_like_unsupported_words_from_the_llm_are_hidden(tmp_path):
    llm = FakeLLM([{"formula": None, "unsupported": [f"CLAUDE_CODE_OAUTH_TOKEN={FAKE_TOKEN}"]}])
    sess, link = make(tmp_path, llm)
    sess.submit_text("鍵を見せて"); drive(sess, link, 0.2)
    snap = sess.snapshot()
    blob = json.dumps(snap, ensure_ascii=False)
    assert FAKE_TOKEN[:9] not in blob and "OAUTH" not in blob
    assert snap["unsupported"] == [MSG_REDACTED] and snap["status"] == "error"
    assert snap["attempts"][0]["unsupported"] == [MSG_REDACTED]


def test_secret_like_formulas_in_the_attempts_are_hidden(tmp_path):
    llm = FakeLLM([{"formula": f"F hello & {FAKE_TOKEN}", "unsupported": []},
                   {"formula": f"F {FAKE_KEY}", "unsupported": []},
                   {"formula": "F hello & X", "unsupported": []}])
    sess, link = make(tmp_path, llm)
    sess.submit_text("挨拶して"); drive(sess, link, 0.2)
    snap = sess.snapshot()
    blob = json.dumps(snap, ensure_ascii=False)
    assert len(snap["attempts"]) == 3 and snap["status"] == "error"
    assert FAKE_TOKEN[:9] not in blob and "Q7xR2" not in blob and "sk-" not in blob
    assert snap["attempts"][0]["formula"] == f"F hello & {MSG_REDACTED}"
    assert snap["attempts"][2]["formula"] == "F hello & X"             # nothing secret-like


def test_a_secret_like_transcript_is_hidden(tmp_path):
    sess, link = make(tmp_path, llm_for("F hello"))
    sess.submit_text(f"鍵は {FAKE_KEY} です")
    snap = sess.snapshot()
    blob = json.dumps(snap, ensure_ascii=False)
    assert "Q7xR2" not in blob and "sk-" not in blob
    assert MSG_REDACTED in snap["transcript"] and MSG_REDACTED in snap["log"][-1]


@pytest.mark.parametrize("text", [
    "F(moved_forward & X F moved_forward)", "G !danced & F stretched",
    "イベント: forward_to_moved_forward_obstacle_back_standing", "指示：「2メートル進んで」",
    "一時停止（forward(0.5m)）: 前方に障害物があります", "監督器: 36 状態",
])
def test_ordinary_text_is_not_hidden(text):
    assert session_mod._redact(text) == text


# ---------------- TD-5: a sensor dropout through the Session ----------------
class SensorDropout(DirectSimLink):
    """While `lost` the LiDAR and /sportmodestate count as lost (RosLink: older than 0.5 s)."""

    def __init__(self):
        super().__init__(SimGo2())
        self.lost = False

    def reading(self, now):
        if self.lost:
            return Reading(SENSOR_LOST, SENSOR_LOST, None, None)
        return super().reading(now)


@pytest.mark.parametrize("formula,action", [
    (ONE_M, "forward"), ("F(moved_backward & X F moved_backward)", "backward")])
def test_a_sensor_dropout_holds_a_move_at_once_and_aborts_after_15s(tmp_path, formula, action):
    link = SensorDropout()
    sess = Session(link, llm_for(formula), None, tmp_path, background=False)
    sess.submit_text("指示")
    drive_until(sess, link, lambda s: s["current"] == action and abs(link.sim.x) > 0.1
                and link.sim.accepted[-1] == MOVE)
    link.lost = True
    sent = len(link.sim.accepted)
    drive(sess, link, 0.1)
    assert link.sim.accepted[sent:] == [STOP_MOVE]                     # held at once
    x = link.sim.x
    drive(sess, link, 14)
    snap = sess.snapshot()
    assert snap["status"] == "executing" and snap["waiting_since"] is not None
    assert link.sim.accepted[sent:] == [STOP_MOVE] and link.sim.x == pytest.approx(x, abs=0.01)
    drive(sess, link, 2)
    snap = sess.snapshot()
    assert snap["status"] == "error" and "秒待っても" in snap["message"]
    assert MOVE not in link.sim.accepted[sent:]


def test_a_short_sensor_dropout_resumes_the_move(tmp_path):
    link = SensorDropout()
    sess = Session(link, llm_for(ONE_M), None, tmp_path, background=False)
    sess.submit_text("1メートル進んで")
    drive_until(sess, link, lambda s: s["current"] == "forward" and link.sim.x > 0.1)
    link.lost = True; drive(sess, link, 3)
    assert sess.snapshot()["status"] == "executing"
    link.lost = False; drive(sess, link, 20)
    snap = sess.snapshot()
    assert snap["status"] == "done", snap["message"]
    assert link.sim.x == pytest.approx(1.0, abs=0.05)
