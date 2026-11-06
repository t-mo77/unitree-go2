"""run_formula の、ROS 2 なしで確かめられる所：ログの差分、引数、終了コード、時間切れ、Ctrl+C、待つ秒数。"""

import sys
import types

import pytest

from voice_ltl import run_formula
from voice_ltl.objects import DEFAULT_LAYOUT
from voice_ltl.run_formula import OBJECT_WAIT_S, _follow, _new_lines, _parse, main
from voice_ltl.server import WAIT_READY_GAZEBO_S, WAIT_READY_S, wait_ready_seconds
from voice_ltl.session import LOG_LINES, MSG_STOPPED

FORMULA = "F(moved_forward & X F lying)"


# ---------------- 待つ秒数（GO2_WAIT_READY_S、Ruling R38） ----------------
@pytest.mark.parametrize("env,seconds", [
    ({}, 5.0),                                                  # 実機（SIM が無い）
    ({"SIM": "1"}, 5.0),                                        # 偽の Go2
    ({"SIM": "1", "SIM_KIND": "fake"}, 5.0),
    ({"SIM": "1", "SIM_KIND": "gazebo"}, 90.0),                 # Gazebo は立ち上がりに 20〜40 秒
    ({"SIM": "1", "SIM_KIND": " Gazebo "}, 90.0),
    ({"SIM": "0", "SIM_KIND": "gazebo"}, 5.0),                  # 実機なら SIM_KIND は関係ない
    ({"SIM": "1", "SIM_KIND": "gazebo", "GO2_WAIT_READY_S": ""}, 90.0),   # 空は既定（compose の既定）
    ({"SIM": "1", "SIM_KIND": "gazebo", "GO2_WAIT_READY_S": " "}, 90.0),
    ({"GO2_WAIT_READY_S": "12.5"}, 12.5),
    ({"SIM": "1", "SIM_KIND": "gazebo", "GO2_WAIT_READY_S": "30"}, 30.0),
])
def test_wait_ready_seconds(env, seconds):
    assert (WAIT_READY_S, WAIT_READY_GAZEBO_S) == (5.0, 90.0)
    assert wait_ready_seconds(env) == seconds


@pytest.mark.parametrize("raw", ["0", "-1", "abc", "nan", "inf", "5s"])
def test_wait_ready_seconds_rejects_bad_values(raw):
    with pytest.raises(ValueError, match="GO2_WAIT_READY_S"):
        wait_ready_seconds({"GO2_WAIT_READY_S": raw})


# ---------------- _new_lines ----------------
def test_new_lines_appended():
    assert _new_lines([], ["a", "b"]) == ["a", "b"]
    assert _new_lines(["a", "b"], ["a", "b", "c"]) == ["c"]
    assert _new_lines(["a", "b"], ["a", "b"]) == []
    assert _new_lines(["a", "a"], ["a", "a", "a"]) == ["a"]     # 同じ行が続いても、重なりは最長で取る


def test_new_lines_after_old_lines_drop_off_the_front():
    """Session のログは最後の LOG_LINES 行だけ。いっぱいになると古い行が前から消える。"""
    full = [f"行 {k}" for k in range(LOG_LINES)]
    later = full[2:] + ["行 new 1", "行 new 2"]
    assert len(later) == LOG_LINES
    assert _new_lines(full, later) == ["行 new 1", "行 new 2"]
    assert _new_lines(["a", "b", "c"], ["c", "d"]) == ["d"]
    assert _new_lines(["a", "b"], ["x", "y"]) == ["x", "y"]     # 重なりが無ければ全部新しい


# ---------------- 引数と終了コード ----------------
def test_parse_formula_and_timeout():
    args = _parse([FORMULA])
    assert (args.formula, args.timeout) == (FORMULA, 120.0)
    assert _parse([FORMULA, "--timeout", "40"]).timeout == 40.0


@pytest.mark.parametrize("argv,code", [
    (["--help"], 0),
    ([], 1),                                    # 式が無い（argparse の 2 にしない）
    ([FORMULA, "extra"], 1),
    ([FORMULA, "--timeout", "0"], 1),
    ([FORMULA, "--timeout", "-3"], 1),
    ([FORMULA, "--timeout", "nan"], 1),
    ([FORMULA, "--timeout", "abc"], 1),
])
def test_usage_exit_codes(argv, code, capsys):
    assert main(argv) == code


# ---------------- ROS の代わり ----------------
class Clock:
    """time の代わり。sleep で時計が進む。"""

    def __init__(self):
        self.t = 1000.0

    def monotonic(self):
        return self.t

    def time(self):
        return self.t

    def sleep(self, seconds):
        self.t += max(seconds, 0.0)


@pytest.fixture
def clock(monkeypatch):
    c = Clock()
    monkeypatch.setattr(run_formula, "time", c)
    return c


class ScriptedSession:
    """tick 1 回ごとに、script の次の (status, message, log) になる。最後のものに留まる。"""

    def __init__(self, script):
        self.script, self.i, self.ticks, self.stopped = list(script), 0, 0, False

    def snapshot(self):
        status, message, log = self.script[self.i]
        if self.stopped:   # Session.stop と同じ：idle・「止めました」
            status, message, log = "idle", MSG_STOPPED, log + [MSG_STOPPED]
        return {"status": status, "message": message, "log": list(log)}

    def tick(self, now):
        self.ticks += 1
        self.i = min(self.i + 1, len(self.script) - 1)

    def stop(self):
        self.stopped = True


class SpinOnly:
    spins = 0

    def spin_once(self):
        self.spins += 1


def test_follow_prints_each_line_once_and_exits_0_on_done(clock, capsys):
    s = ScriptedSession([("thinking", "", []),
                         ("executing", "", ["計画：立つ"]),
                         ("executing", "", ["計画：立つ", "開始：立つ"]),
                         ("done", "終わりました", ["計画：立つ", "開始：立つ", "完了：立つ"])])
    assert _follow(SpinOnly(), s, timeout=10.0) == 0
    assert capsys.readouterr().out.splitlines() == [
        "計画：立つ", "開始：立つ", "完了：立つ", "結果：done（終わりました）"]
    assert s.ticks == 3


def test_follow_exits_1_on_error(clock, capsys):
    s = ScriptedSession([("executing", "", []), ("error", "15秒待っても進めません", ["中止"])])
    assert _follow(SpinOnly(), s, timeout=10.0) == 1
    assert capsys.readouterr().out.splitlines()[-1] == "結果：error（15秒待っても進めません）"


def test_timeout_stops_and_reports_the_status_after_stopping(clock, capsys):
    """時間切れ：止めて、止めた後の status を出す（止める前の古い status は出さない）。"""
    s = ScriptedSession([("executing", "", ["開始：前進"])])
    link = SpinOnly()
    assert _follow(link, s, timeout=1.0) == 1
    assert s.stopped and 10 <= s.ticks <= 11 and link.spins == s.ticks   # 10 Hz で 1 秒
    out = capsys.readouterr().out.splitlines()
    assert out[-2] == MSG_STOPPED
    assert out[-1] == f"結果：時間切れ（1 秒たっても終わりませんでした。止めた後の status：idle（{MSG_STOPPED}））"
    assert "executing" not in out[-1]


@pytest.fixture
def fake_ros(monkeypatch, tmp_path):
    """voice_ltl.ros_link と Session を差し替える。behaviour で RosLink の振る舞いを決める。"""
    made, sessions = [], []
    behaviour = {"init": None, "wait": None, "others": 0}

    class FakeRosLink:
        def __init__(self, perception, node_name="go2_voice_ltl"):
            if behaviour["init"] is not None:
                raise behaviour["init"]
            self.node_name, self.waited, self.closed_with = node_name, None, None
            made.append(self)

        def wait_ready(self, timeout_s=5.0):
            self.waited = timeout_s
            if behaviour["wait"] is not None:
                raise behaviour["wait"]

        def other_senders(self):
            return behaviour["others"]

        def objects(self):
            return behaviour.get("objects", [])

        def spin_once(self):
            self.spins = getattr(self, "spins", 0) + 1

        def close(self, stop_times=3):
            self.closed_with = stop_times

    class FakeSession(ScriptedSession):
        def __init__(self, link, llm, transcriber, workdir, background=True):
            super().__init__([("done", "終わりました", [])])
            self.llm, self.workdir, self.background, self.texts = llm, workdir, background, []
            sessions.append(self)

        def submit_text(self, text):
            self.texts.append(text)

    ros_link = types.ModuleType("voice_ltl.ros_link")
    ros_link.RosLink = FakeRosLink
    monkeypatch.setitem(sys.modules, "voice_ltl.ros_link", ros_link)
    monkeypatch.setattr("voice_ltl.session.Session", FakeSession)
    monkeypatch.setattr(run_formula, "WORKDIR", tmp_path)
    for name in ("SIM", "SIM_KIND", "GO2_WAIT_READY_S"):
        monkeypatch.delenv(name, raising=False)
    return types.SimpleNamespace(made=made, sessions=sessions, behaviour=behaviour)


def test_rejected_formula_exits_1_without_ros(fake_ros, capsys):
    assert main(["F flying"]) == 1
    assert "式が文法か単語帳に合いません" in capsys.readouterr().out
    assert fake_ros.made == []


def test_done_exits_0_and_sends_stopmove_x3(fake_ros, clock, capsys):
    assert main([FORMULA]) == 0
    (link,), (session,) = fake_ros.made, fake_ros.sessions
    assert link.node_name == "go2_run_formula" and link.closed_with == 3
    assert session.background is False and session.texts == [run_formula.INSTRUCTION]
    assert session.llm.replies == [{"formula": FORMULA, "unsupported": []}]
    assert capsys.readouterr().out.splitlines()[-1] == "結果：done（終わりました）"


@pytest.mark.parametrize("env,seconds", [
    ({}, 5.0),
    ({"SIM": "1", "SIM_KIND": "gazebo"}, 90.0),
    ({"SIM": "1", "SIM_KIND": "gazebo", "GO2_WAIT_READY_S": "40"}, 40.0),
])
def test_waits_go2_wait_ready_s_then_not_found_exits_1(fake_ros, monkeypatch, capsys, env, seconds):
    for name, value in env.items():
        monkeypatch.setenv(name, value)
    fake_ros.behaviour["wait"] = RuntimeError("Go2 が見つかりません")
    assert main([FORMULA]) == 1
    (link,) = fake_ros.made
    assert link.waited == seconds and link.closed_with == 3
    assert "Go2 が見つかりません" in capsys.readouterr().out


def test_another_sender_exits_1_without_stopmove(fake_ros):
    fake_ros.behaviour.update(wait=RuntimeError("別のプログラムがあります"), others=1)
    assert main([FORMULA]) == 1
    assert [link.closed_with for link in fake_ros.made] == [0]


def test_bad_go2_wait_ready_s_exits_1_before_ros(fake_ros, monkeypatch, capsys):
    monkeypatch.setenv("GO2_WAIT_READY_S", "abc")
    assert main([FORMULA]) == 1
    assert "GO2_WAIT_READY_S" in capsys.readouterr().out
    assert fake_ros.made == []


def test_second_start_is_refused(fake_ros, capsys):
    fake_ros.behaviour["init"] = RuntimeError("もう起動しています")
    assert main([FORMULA]) == 1
    assert "もう起動しています" in capsys.readouterr().out


@pytest.mark.parametrize("where", ["init", "wait"])
def test_ctrl_c_ends_with_interrupted(fake_ros, capsys, where):
    """Ctrl+C（RosLink が SIGTERM も KeyboardInterrupt にする）は、RosLink を作っている間でも「中断しました」。"""
    fake_ros.behaviour[where] = KeyboardInterrupt()
    assert main([FORMULA]) == 1
    assert capsys.readouterr().out.splitlines()[-1] == "結果：中断しました"
    assert [link.closed_with for link in fake_ros.made] == ([] if where == "init" else [3])


# ---------------- 物の命題（R24） ----------------
BOX_ROW = DEFAULT_LAYOUT[0].to_row()


def test_object_formula_passes_the_precheck_and_runs(fake_ros, clock, capsys):
    fake_ros.behaviour["objects"] = [BOX_ROW]
    assert main(["F around_box_1"]) == 0
    (link,), (session,) = fake_ros.made, fake_ros.sessions
    assert session.texts == [run_formula.INSTRUCTION] and link.closed_with == 3
    assert session.llm.replies == [{"formula": "F around_box_1", "unsupported": []}]


class StepClock:
    """time for run_formula: monotonic() goes on 1 s at every call; sleep() does nothing."""

    def __init__(self):
        self.t = 1000.0

    def monotonic(self):
        self.t += 1.0
        return self.t

    def time(self):
        return self.t

    def sleep(self, seconds):
        pass


def test_missing_object_exits_1(fake_ros, monkeypatch, capsys):
    assert OBJECT_WAIT_S == 3.0
    monkeypatch.setattr(run_formula, "time", StepClock())
    fake_ros.behaviour["objects"] = []
    assert main(["F(near_chair_1 & X F around_box_1)"]) == 1
    out = capsys.readouterr().out
    assert "「box_1」が見つかりません" in out and "chair_1" not in out     # the first one, sorted
    (link,) = fake_ros.made
    assert link.closed_with == 3 and fake_ros.sessions == []
    assert 1 <= link.spins <= 4                                             # spun while waiting, 3 s at most


def test_an_object_arriving_while_waiting_is_taken(fake_ros, clock, capsys):
    rows = []

    def arrives():                                    # /go2/objects comes after 1 s
        if clock.t >= 1001.0:
            rows[:] = [BOX_ROW]
        return rows

    fake_ros.behaviour["objects"] = rows
    original = run_formula.time.sleep

    def sleep(seconds):
        original(seconds)
        arrives()

    clock.sleep = sleep
    assert main(["F around_box_1"]) == 0
    assert fake_ros.sessions[0].texts == [run_formula.INSTRUCTION]


def test_an_unknown_object_proposition_is_still_checked(fake_ros, capsys):
    assert main(["F near_table_1"]) == 1                 # not an object proposition: not in the vocabulary
    assert "式が文法か単語帳に合いません" in capsys.readouterr().out
    assert fake_ros.made == []
