"""scripts/object_phrasing_trials.py (Task J): the judge, the groups and the counting. FakeLLM only, no network."""

import json

import pytest

from scripts import object_phrasing_trials as trials
from scripts.object_phrasing_trials import (GROUPS, ONE_BOX, SCENE, TWO_BOXES, judge, main, plan_actions,
                                            run_groups)
from voice_ltl.grammar import check
from voice_ltl.llm import FakeLLM, LLMError
from voice_ltl.objects import Obj, Scene, msg_ambiguous, msg_not_found, msg_too_far, where
from voice_ltl.translator import Translation
from voice_ltl.world import PROPOSITIONS

def ok(formula, scene=SCENE):
    return Translation(True, formula, [], scene=scene)


def refused(message, scene=SCENE, unsupported=("箱",)):
    return Translation(False, None, list(unsupported), message=message, scene=scene)


def test_judge_accepts_an_equivalent_formula_with_the_same_plan(tmp_path):
    case = ("箱の周りを一周して", SCENE, "F around_box_1", None)
    assert judge(case, ok("F(around_box_1)"), tmp_path) == (True, "")
    # 「立って」 written out is the same plan: the Go2 stands first anyway
    assert judge(case, ok("F(standing & X F around_box_1)"), tmp_path)[0]


def test_judge_rejects_the_wrong_object_or_direction(tmp_path):
    passed, reason = judge(("箱の周りを一周して", SCENE, "F around_box_1", None), ok("F around_cw_box_1"), tmp_path)
    assert not passed and "around_cw_box_1" in reason
    passed, reason = judge(("椅子まで歩いて", SCENE, "F near_chair_1", None), ok("F near_box_1"), tmp_path)
    assert not passed and "near_box_1" in reason


def test_judge_rejects_the_same_object_with_a_different_plan(tmp_path):
    # 「2周」 must be capped at 1: a second around after the first one is one more step
    passed, reason = judge(("箱を2周して", SCENE, "F around_box_1", None),
                           ok("F(around_box_1 & X F around_box_1)"), tmp_path)
    assert not passed and "plan" in reason


def test_judge_rejects_a_refusal_or_a_formula_that_cannot_be_planned(tmp_path):
    case = ("椅子まで歩いて", SCENE, "F near_chair_1", None)
    passed, reason = judge(case, refused("「椅子」は単語帳にないので、まだできません。"), tmp_path)
    assert not passed and "単語帳" in reason
    passed, reason = judge(("挨拶して", SCENE, "F hello", None), ok("F(hello & !hello)"), tmp_path)
    assert not passed and reason


def test_refusal_needs_the_right_message(tmp_path):
    case = ("箱まで行って", TWO_BOXES, None, "決められません")
    assert judge(case, refused(msg_ambiguous("box", 2), TWO_BOXES), tmp_path) == (True, "")
    passed, reason = judge(case, refused(msg_not_found("box"), TWO_BOXES), tmp_path)
    assert not passed and "見つかりません" in reason
    passed, reason = judge(case, ok("F near_box_1", TWO_BOXES), tmp_path)
    assert not passed and "F near_box_1" in reason
    far = Scene((Obj("box_1", "box", 7.0, 0.0),), (0.0, 0.0, 0.0))
    assert judge(("箱まで行って", far, None, "遠すぎます"), refused(msg_too_far("box"), far, ()), tmp_path)[0]


def test_plan_actions_uses_the_grid_only_for_object_formulas(tmp_path):
    assert plan_actions("F hello", SCENE, tmp_path) == ["stand", "hello"]
    walk = plan_actions("F near_chair_1", SCENE, tmp_path)
    assert walk[0] == "stand" and "forward" in walk and set(walk) <= {"stand", "forward", "turn_left", "turn_right"}
    lap = plan_actions("F around_box_1", SCENE, tmp_path)
    assert lap[0] == "stand" and lap != plan_actions("F around_cw_box_1", SCENE, tmp_path)
    assert plan_actions("F lying", SCENE, tmp_path) == []                  # already true while lying: no step
    with pytest.raises(trials.TrialError):
        plan_actions("F(hello & !hello)", SCENE, tmp_path)


def test_groups_follow_the_plan():
    assert {g: len(cases) for g, cases in GROUPS.items()} == {
        "lap": 10, "near": 10, "objects": 7, "refusals": 7, "regression": 12}
    assert all(c[1:] == (SCENE, "F around_box_1", None) for c in GROUPS["lap"])
    assert all(c[1:] == (SCENE, "F near_chair_1", None) for c in GROUPS["near"])
    assert [c[0] for c in GROUPS["lap"]] == [
        "箱の周りを一周して", "箱の周りを回って", "箱を一周して", "箱の周りをぐるっと回って", "箱のまわりを1周して",
        "箱の周りを左回りに一周して", "ボックスの周りを回って", "箱の周囲を一周して", "箱の周りを歩いて一周して",
        "立って箱の周りを回って"]
    assert [c[0] for c in GROUPS["near"]] == [
        "椅子まで歩いて", "椅子のところまで行って", "椅子の前まで歩いて", "椅子に近づいて", "いすまで行って",
        "チェアのところへ行って", "椅子の方へ歩いて行って", "椅子のそばまで行って", "椅子まで進んで", "立って椅子まで歩いて"]
    obstacle = [c for c in GROUPS["objects"] if c[0] == "障害物の周りを右回りで回って"]
    assert [(c[1], c[2], c[3]) for c in obstacle] == [(ONE_BOX, "F around_cw_box_1", None),
                                                      (SCENE, None, "単語帳にない")]
    assert [c[2] for c in GROUPS["objects"] if c[1] is TWO_BOXES] == ["F near_box_2", "F around_box_1"]
    assert [where(o, TWO_BOXES.pose) for o in TWO_BOXES.objects] == ["左前 2.9 m", "右前 2.9 m"]
    assert trials.REFUSAL_WORDS == ("見つかりません", "決められません", "遠すぎます", "単語帳にない")
    for group, cases in GROUPS.items():
        for instruction, scene, reference, word in cases:
            assert (reference is None) != (word is None), (group, instruction)
            assert word in (None, *trials.REFUSAL_WORDS)


def test_every_reference_formula_passes_the_check_and_has_a_plan(tmp_path):
    for group, cases in GROUPS.items():
        for instruction, scene, reference, _ in cases:
            if reference is not None:
                assert check(reference, PROPOSITIONS + scene.propositions()) == [], instruction
                assert plan_actions(reference, scene, tmp_path)[0] == "stand", instruction


def test_run_groups_counts_per_group(tmp_path):
    groups = {"near": [("椅子まで歩いて", SCENE, "F near_chair_1", None),
                       ("いすまで行って", SCENE, "F near_chair_1", None)],
              "refusals": [("机まで行って", SCENE, None, "単語帳にない")]}
    llm = FakeLLM([{"formula": "F near_chair_1", "unsupported": []},
                   {"formula": "F near_box_1", "unsupported": []},
                   {"formula": None, "unsupported": ["机"]}])
    seen = []
    counts = run_groups(llm, groups, tmp_path, on_result=seen.append)
    assert counts == {"near": (1, 2), "refusals": (1, 1)}
    assert [(r.group, r.ok) for r in seen] == [("near", True), ("near", False), ("refusals", True)]
    assert "## Objects" in llm.prompts[0] and "- chair_1（椅子）：右前 2.9 m" in llm.prompts[0]


def test_an_llm_error_fails_that_case_only(tmp_path):
    groups = {"regression": [("挨拶して", SCENE, "F hello", None), ("お座りして", SCENE, "F sitting", None)]}
    llm = FakeLLM([LLMError("LLM の返事が 60 秒で返ってきませんでした"), {"formula": "F sitting", "unsupported": []}])
    seen = []
    assert run_groups(llm, groups, tmp_path, on_result=seen.append) == {"regression": (1, 2)}
    assert "60 秒" in seen[0].reason and seen[0].translation is None


class LLMByInstruction:
    """Answers by the instruction in the prompt: table[instruction] is the reply (default: a refusal)."""

    def __init__(self, table):
        self.table = table

    def __call__(self, prompt, schema):
        instruction = json.loads(prompt.split("## Query\n指示: ", 1)[1].split("\n", 1)[0])
        return self.table.get(instruction, {"formula": None, "unsupported": ["x"], "substitutions": []})


def _right_answers(wrong_laps=0):
    table = {}
    for k, (instruction, _, reference, _) in enumerate(GROUPS["lap"]):
        table[instruction] = {"formula": "F near_box_1" if k < wrong_laps else reference, "unsupported": []}
    for instruction, _, reference, _ in GROUPS["near"]:
        table[instruction] = {"formula": reference, "unsupported": []}
    return table


@pytest.mark.parametrize("wrong_laps, code", [(0, 0), (1, 0), (2, 1)])
def test_main_exits_0_only_when_lap_and_near_reach_9_of_10(monkeypatch, capsys, tmp_path, wrong_laps, code):
    monkeypatch.setattr(trials, "make_llm", lambda env: LLMByInstruction(_right_answers(wrong_laps)))
    assert main(["--groups", "lap,near", "--workers", "2", "--workdir", str(tmp_path)]) == code
    lines = capsys.readouterr().out.strip().splitlines()
    cases = [json.loads(line) for line in lines if line.startswith("{\"group\"")]
    assert len(cases) == 20 and sum(not c["ok"] for c in cases) == wrong_laps
    assert all({"instruction", "scene", "formula", "attempts", "message", "reason"} <= set(c) for c in cases)
    summary = json.loads(lines[-1])
    assert summary["summary"] == {"lap": [10 - wrong_laps, 10], "near": [10, 10]} and summary["pass"] == (code == 0)
