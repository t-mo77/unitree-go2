"""階段の歩き方（公式の ClassicWalk、api 2049）：動作の一覧・式の単語帳・実行器・偽の Go2・Gazebo の橋渡し。"""

import json

import pytest

from go2_nl_control.actions import (ACTIONS, CLASSIC_WALK, LYING, MOVE, SITTING, STANDING, STOP_MOVE,
                                    state_from_error_code)
from go2_nl_control.executor import Command, Executor
from go2_nl_control.supervisor import CLEAR, Step
from gazebo.bridge_core import BridgeCore, api_name
from sim.sim_core import SimGo2, gait_code, parse_gait
from voice_ltl import prompt
from voice_ltl.ltlf import parse
from voice_ltl.session import ACTION_JA, BUSY_S, EXEC_ACTIONS
from voice_ltl.world import (ACTION_NAMES, ACTION_PROP, PROP_DESCRIPTION_JA, PROPOSITIONS, PlantState,
                             go2_world)

from test_bridge_core import standing_core
from test_session import drive, llm_for, make

ON, OFF = '{"data": true}', '{"data": false}'


# ---- 動作の一覧 ----

def test_classic_walk_is_the_official_api_id():
    assert CLASSIC_WALK == 2049   # unitree_sdk2 の sport_api（ROBOT_SPORT_API_ID_CLASSICWALK）


@pytest.mark.parametrize("action,data", [("stairs_gait", True), ("normal_gait", False)])
def test_gait_actions_send_classic_walk_with_its_flag(action, data):
    spec = ACTIONS[action]
    assert spec.transitions == {STANDING: STANDING}      # 立っているときだけ。姿勢は変えない
    assert [api for api, _ in spec.sequence] == [CLASSIC_WALK]
    assert spec.parameter == {"data": data}
    assert spec.param is None and not spec.forward


def test_executor_sends_the_parameter():
    ex = Executor(ACTIONS)
    ex.posture = STANDING
    ex.load([Step("stairs_gait", 0.0), Step("normal_gait", 0.0)])
    assert ex.tick(0.0, CLEAR, (0.0, 0.0, 0.0)) == [Command(CLASSIC_WALK, {"data": True})]
    wait = ACTIONS["stairs_gait"].sequence[0][1]
    sent, t = [], 0.0
    while t < wait + 0.15:            # 実行器は 1 周期を 0.5 秒までに切るので、10 Hz で回す
        t = round(t + 0.1, 6)
        sent += ex.tick(t, CLEAR, (0.0, 0.0, 0.0))
    assert sent == [Command(CLASSIC_WALK, {"data": False})]
    assert str(Command(CLASSIC_WALK, {"data": True})) == '2049 {"data": true}'


def test_other_actions_send_no_parameter():
    assert all(spec.parameter is None for name, spec in ACTIONS.items()
               if name not in ("stairs_gait", "normal_gait"))


def test_busy_time_of_classic_walk():
    assert BUSY_S[CLASSIC_WALK] == ACTIONS["stairs_gait"].sequence[0][1] == ACTIONS["normal_gait"].sequence[0][1]


# ---- 式の単語帳 ----

def test_vocabulary_has_both_gaits():
    for action, prop in (("stairs_gait", "stairs_gait"), ("normal_gait", "normal_gait")):
        assert action in ACTION_NAMES and ACTION_PROP[action] == prop
        assert prop in PROPOSITIONS and PROP_DESCRIPTION_JA[prop]
        assert action in ACTION_JA and action in EXEC_ACTIONS


def test_plant_offers_gait_switches_only_while_standing():
    world = go2_world(PlantState(LYING, False, False))
    by_posture = {}
    for src, ev, _, kind in world.transitions:
        if kind == "c":
            by_posture.setdefault(src.posture, set()).add(ev.split("_to_", 1)[0])
    assert {"stairs_gait", "normal_gait"} <= by_posture[STANDING]
    assert not {"stairs_gait", "normal_gait"} & (by_posture[LYING] | by_posture.get(SITTING, set()))


# ---- 音声の式（プロンプト） ----

def _examples():
    return {e[0]: e for e in prompt._EXAMPLES}


def test_climb_n_meters_is_gait_then_forward_then_back():
    ex = _examples()["階段を2メートル登って"]
    assert ex[1] == prompt._chain("stairs_gait", *["moved_forward"] * 4, "normal_gait")
    parse(ex[1])


def test_climb_without_distance_only_switches_the_gait():
    formula, unsupported, subs = _examples()["階段を登って"][1:]
    assert formula == "F stairs_gait" and unsupported == []
    assert subs and subs[0]["said"] == "階段を登って" and "距離" in subs[0]["as"]


def test_mode_phrases():
    ex = _examples()
    assert ex["階段モードにして"][1] == "F stairs_gait"
    assert ex["普通の歩き方に戻して"][1] == "F normal_gait"


def test_prompt_no_longer_refuses_stairs_and_explains_them():
    text = prompt.build_prompt("階段を登って")
    assert "階段を上る" not in text
    assert "### 階段" in text and "ClassicWalk" not in text   # LLM には API の名前でなく、することを書く
    assert "- stairs_gait: " in text and "- normal_gait: " in text


# ---- 偽の Go2 ----

def _standing_sim():
    return SimGo2(posture=STANDING)


@pytest.mark.parametrize("parameter,gait", [(ON, "classic"), (OFF, "agile"), ('{"data": 1}', None),
                                            ("{}", None), ("", None), ("[true]", None), ("null", None)])
def test_parse_gait(parameter, gait):
    assert parse_gait(parameter) == gait


@pytest.mark.parametrize("code,gait,reported", [
    (1013, None, 1013), (1015, None, 1015), (1013, "classic", 2010), (1015, "classic", 2010),
    (1013, "agile", 100), (1015, "agile", 100), (1004, "classic", 1004), (1002, "classic", 1002),
    (1007, "agile", 1007), (1001, "classic", 1001)])
def test_gait_code(code, gait, reported):
    assert gait_code(code, gait) == reported


def test_sim_classic_walk_reports_2010_and_still_walks():
    sim = _standing_sim()
    sim.handle(CLASSIC_WALK, ON, 0.0)
    assert sim.gait == "classic" and CLASSIC_WALK in sim.accepted
    sim.step(1.5)
    assert sim.reported_code == 2010 and state_from_error_code(sim.reported_code) == STANDING
    sim.handle(MOVE, '{"x": 0.3, "y": 0, "z": 0}', 1.5)
    for t in (1.6, 1.8, 2.0, 2.2, 2.4):
        sim.step(t)
    assert sim.x > 0.2 and sim.reported_code == 2010
    sim.handle(STOP_MOVE, "", 2.5)
    sim.handle(CLASSIC_WALK, OFF, 2.5)
    sim.step(4.0)
    assert sim.gait == "agile" and sim.reported_code == 100


def test_sim_ignores_classic_walk_unless_standing_or_with_a_bad_parameter():
    lying = SimGo2(posture=LYING)
    lying.handle(CLASSIC_WALK, ON, 0.0)
    assert lying.gait is None and CLASSIC_WALK not in lying.accepted
    sim = _standing_sim()
    sim.handle(CLASSIC_WALK, '{"data": "yes"}', 0.0)
    assert sim.gait is None and CLASSIC_WALK not in sim.accepted


# ---- Gazebo の橋渡し ----

def test_bridge_accepts_classic_walk_as_pretend():
    b = standing_core()
    assert api_name(CLASSIC_WALK) == "ClassicWalk 2049"
    b.handle(CLASSIC_WALK, ON, 10.0)
    assert b.step(10.0) is None                        # controller には何も送らない（歩き方は同じ）
    assert b.gait == "classic" and "Gazebo では歩き方は変わりません" in b.notes[-1]
    b.step(12.0)
    assert b.error_code == 1013 and b.reported_code == 2010
    b.handle(CLASSIC_WALK, OFF, 12.0)
    b.step(14.0)
    assert b.gait == "agile" and b.reported_code == 100


def test_bridge_refuses_classic_walk_while_lying_or_with_a_bad_parameter():
    b = BridgeCore(0.0)
    b.handle(CLASSIC_WALK, ON, 0.0)
    assert b.gait is None and "受け付けません" in b.notes[-1]
    s = standing_core()
    s.handle(CLASSIC_WALK, "{}", 10.0)
    assert s.gait is None and "受け付けません" in s.notes[-1]


# ---- 通しで（偽の Go2 と、音声のかわりの式） ----

def test_climb_two_meters_end_to_end(tmp_path):
    formula = prompt._chain("stairs_gait", *["moved_forward"] * 4, "normal_gait")
    sess, link = make(tmp_path, llm_for(formula))
    seen = [(link.sim.gait, link.sim.reported_code, link.sim.x)]
    sess.submit_text("階段を2メートル登って")
    for _ in range(120):
        drive(sess, link, 0.5)
        seen.append((link.sim.gait, link.sim.reported_code, link.sim.x))
    assert sess.snapshot()["status"] == "done"
    assert [a for a in link.sim.accepted if a not in (MOVE, STOP_MOVE)][-2:] == [CLASSIC_WALK, CLASSIC_WALK]
    assert any(code == 2010 for _, code, _ in seen) and link.sim.gait == "agile"
    assert 1.8 < link.sim.x < 2.3
    # 前へ進んだのは、階段の歩き方に切り替えた後だけ（0.5 秒ごとの、進み始めの時点の歩き方）
    assert all(g0 == "classic" for (g0, _, x0), (_, _, x1) in zip(seen, seen[1:]) if x1 - x0 > 0.01)


def test_gait_steps_are_named_on_the_page():
    assert ACTION_JA["stairs_gait"] != "stairs_gait" and ACTION_JA["normal_gait"] != "normal_gait"
    assert json.dumps(ACTION_JA, ensure_ascii=False)


# ---- キーボードで試す（scripts/keyboard_velocity.py。実機の Linux の PC で、アプリなしで） ----

def test_keyboard_sends_the_same_classic_walk():
    from scripts import keyboard_velocity as kv
    assert kv.CLASSIC_WALK == CLASSIC_WALK
    assert kv.GAIT_KEYS == {"3": True, "4": False}
    assert not set(kv.GAIT_KEYS) & (set(kv.VELOCITY_KEYS) | {"1", "2", " ", "z"})
    assert " 3 " in kv.HELP and " 4 " in kv.HELP and "ClassicWalk" in kv.HELP


def test_keyboard_names_the_official_states():
    from scripts import keyboard_velocity as kv
    assert "ClassicWalk" in kv.state_line(2010) and "Agile" in kv.state_line(100)
    assert kv.state_line(9999) == "[STATE] error_code 9999"
    # 名前を付けた番号は、1002（StandUp 直後の歩けない立ち方）を除いて、actions.py の公式の表にある
    from go2_nl_control.actions import UNKNOWN
    assert all(state_from_error_code(c) != UNKNOWN for c in kv.STATE_NAMES if c != 1002)
