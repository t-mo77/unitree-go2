"""ROS 2 もロボットも無しで動くテスト:  python3 -m pytest -q tests"""

import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from go2_nl_control.actions import (ACTIONS, BALANCE_STAND, LYING, MOVE, SITTING, STAND_UP,
                                    STANDING, STOP_MOVE, UNKNOWN, state_from_error_code)
from go2_nl_control.executor import ExecConfig, Executor
from go2_nl_control.perception import ObstacleMonitor, PerceptionConfig
from go2_nl_control.supervisor import BLOCKED, CLEAR, SENSOR_LOST, Step, check_plan
from go2_nl_control.translator import Translator, extract_json, is_stop_command
import offline_sim


def plan(*steps):
    return {"steps": [{"action": a, "value": v} for a, v in steps]}


# ---------- 監督器（計画の検証） ----------
def test_valid_plan():
    steps, why = check_plan(plan(("stand", 0), ("forward", 1.0), ("sit", 0)), LYING, ACTIONS)
    assert why == "" and [s.action for s in steps] == ["stand", "forward", "sit"]


def test_rejects_walking_while_lying():
    steps, why = check_plan(plan(("forward", 1.0),), LYING, ACTIONS)
    assert steps is None and "LYING" in why


def test_rejects_walking_while_sitting_but_accepts_rise_first():
    assert check_plan(plan(("hello", 0),), SITTING, ACTIONS)[0] is None
    assert check_plan(plan(("rise_sit", 0), ("hello", 0)), SITTING, ACTIONS)[0] is not None


def test_rejects_unknown_action_and_out_of_range():
    assert "語彙" in check_plan(plan(("front_flip", 0),), STANDING, ACTIONS)[1]
    assert "範囲" in check_plan(plan(("forward", 5.0),), STANDING, ACTIONS)[1]
    assert "範囲" in check_plan(plan(("turn_left", 720),), STANDING, ACTIONS)[1]
    assert "数値" in check_plan({"steps": [{"action": "forward"}]}, STANDING, ACTIONS)[1]
    assert check_plan({"steps": []}, STANDING, ACTIONS)[0] is None
    assert check_plan(plan(*[("wait", 1)] * 11), STANDING, ACTIONS)[0] is None


def test_error_code_mapping():
    assert state_from_error_code(1013) == STANDING
    assert state_from_error_code(1007) == SITTING
    assert state_from_error_code(1001) == LYING
    assert state_from_error_code(1002) == UNKNOWN   # StandUp 直後（歩けない）
    assert state_from_error_code(None) == UNKNOWN


# ---------- LLM 翻訳 ----------
def test_reprompt_includes_reason():
    _, _, llm = offline_sim.run(verbose=False)
    assert len(llm.calls) == 2
    assert "却下" in llm.calls[1][-1]["content"] and "LYING" in llm.calls[1][-1]["content"]


def test_gives_up_after_three_tries():
    calls = []

    def bad(messages, schema):
        calls.append(1)
        return "了解しました！"
    steps, log = Translator(bad).translate("踊って", STANDING, ACTIONS)
    assert steps is None and len(calls) == 3


def test_llm_error_does_not_retry():
    def broken(messages, schema):
        raise TimeoutError("timeout")
    steps, log = Translator(broken).translate("踊って", STANDING, ACTIONS)
    assert steps is None and len(log) == 1


def test_extract_json_strips_wrappers():
    assert extract_json('<think>…</think>```json\n{"steps": []}\n```') == {"steps": []}
    assert extract_json('計画です: {"steps": []}') == {"steps": []}


def test_stop_words():
    assert is_stop_command("止まって")
    assert is_stop_command("ストップ！")
    assert not is_stop_command("前に進んで、壁の手前で止まって挨拶して")  # 長い指示は LLM へ


# ---------- 環境認識 ----------
def front(x, n=10, y=0.0):
    return np.array([[x - 0.28, y, 0.0]] * n)  # lidar_x=0.28 の分を引いて LiDAR 座標にする


def test_obstacle_hysteresis_and_filters():
    m = ObstacleMonitor(PerceptionConfig(lidar_x=0.28, lidar_z=0.0))
    assert m.update(front(1.0), 0.0) == CLEAR
    assert m.update(front(0.45), 0.1) == BLOCKED
    assert m.update(front(0.60), 0.2) == BLOCKED      # 0.5〜0.7 m は止まったまま（ガタつき防止）
    assert m.update(front(0.75), 0.3) == CLEAR
    assert m.update(front(0.30, y=0.8), 0.4) == CLEAR  # 横の物は無視
    assert m.update(front(0.30, n=3), 0.5) == CLEAR    # 点が少ない（ノイズ）は無視
    ground = np.array([[0.3, 0.0, -0.5]] * 20)
    assert m.update(ground, 0.6) == CLEAR              # 地面は無視
    assert m.current(0.6) == CLEAR
    assert m.current(1.2) == SENSOR_LOST               # 0.5 s 届かなければ途切れ扱い


# ---------- 実行器 ----------
def test_stand_sends_standup_then_balancestand():
    ex = Executor(ACTIONS)
    ex.posture = LYING
    ex.load([Step("stand")])
    sent = []
    t = 0.0
    while ex.busy:
        sent += [(round(t, 1), c.api_id) for c in ex.tick(t, CLEAR, (0, 0, 0))]
        t += 0.1
    assert [a for _, a in sent] == [STAND_UP, BALANCE_STAND]
    assert 2.9 <= sent[1][0] <= 3.1          # StandUp から 3 秒後に BalanceStand
    assert ex.posture == STANDING


def test_only_forward_is_blocked():
    ex = Executor(ACTIONS)
    ex.load([Step("backward", 0.5)])
    assert any(c.api_id == MOVE for c in ex.tick(0.0, BLOCKED, (0, 0, 0)))  # 後退は障害物があってもよい
    ex.load([Step("forward", 0.5)])
    out = ex.tick(0.1, BLOCKED, (0, 0, 0))
    assert [c.api_id for c in out] == [STOP_MOVE]


def test_sensor_lost_holds_all_moves_and_times_out():
    ex = Executor(ACTIONS, ExecConfig(block_timeout=2.0))
    ex.load([Step("turn_left", 90)])
    out = []
    t = 0.0
    while ex.busy and t < 5:
        out += ex.tick(t, SENSOR_LOST, (0, 0, 0))
        t += 0.1
    assert not ex.busy                       # 2 秒で計画を中止
    assert all(c.api_id == STOP_MOVE for c in out)
    assert any("中止" in e for e in ex.events)


def test_cancel_mid_posture_change_makes_posture_unknown():
    ex = Executor(ACTIONS)
    ex.posture = STANDING
    ex.load([Step("sit")])
    ex.tick(0.0, CLEAR, (0, 0, 0))           # Sit を送った直後に
    ex.cancel("止める指示")
    assert ex.posture == UNKNOWN


def test_full_scenario_stops_and_resumes():
    go2, ex, _ = offline_sim.run(verbose=False)
    assert abs(go2.x - 1.0) < 0.1 and abs(go2.y) < 0.05
    assert ex.posture == STANDING and not ex.busy
    # 一時停止のあいだ（StopMove の後、再開まで）は Move を送っていない
    log = go2.log
    k = next(i for i, (t, a) in enumerate(log) if a == STOP_MOVE)
    t_stop = log[k][0]
    gap = [t for t, a in log[k + 1:] if a == MOVE]
    assert gap and gap[0] - t_stop >= 2.0   # 障害物は 3 秒いたので、少なくとも 2 秒は止まっている


if __name__ == "__main__":
    import pytest
    sys.exit(pytest.main(["-q", __file__]))
