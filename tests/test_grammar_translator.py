import pytest

from voice_ltl.grammar import check
from voice_ltl.llm import FakeLLM
from voice_ltl.prompt import build_prompt
from voice_ltl.translator import translate
from voice_ltl.world import PROPOSITIONS


def test_check_accepts_and_rejects():
    P = PROPOSITIONS
    assert check("F(hello & X F hello) & G !danced", P) == []
    assert check("(!sitting U hello) & F(last & sitting)", P) == []
    assert any("括弧" in e for e in check("F (hello", P))
    assert any("at_kitchen" in e for e in check("F at_kitchen", P))
    assert check("", P) and check("F hello " + "& F hello " * 40, P)


def test_check_never_repairs():
    f = "F (hello"
    check(f, PROPOSITIONS)
    assert f == "F (hello"


def test_translate_first_try():
    llm = FakeLLM([{"formula": "F hello", "unsupported": []}])
    t = translate("立って挨拶して", llm)
    assert t.ok and t.formula == "F hello" and len(t.attempts) == 1


def test_translate_reprompts_with_reason_then_gives_up():
    llm = FakeLLM([{"formula": "F at_kitchen", "unsupported": []}] * 3)
    t = translate("台所へ", llm)
    assert not t.ok and len(t.attempts) == 3
    assert "at_kitchen" in llm.prompts[1] and "却下" in llm.prompts[1]


def test_unsupported_stops_without_retry():
    llm = FakeLLM([{"formula": None, "unsupported": ["キッチン"]}])
    t = translate("キッチンへ行って", llm)
    assert not t.ok and t.unsupported == ["キッチン"] and len(llm.prompts) == 1
    assert "キッチン" in t.message and "単語帳" in t.message


def test_prompt_has_four_parts():
    p = build_prompt("挨拶して")
    for part in ("Grammar", "Vocabulary", "Examples", "挨拶して", "unsupported"):
        assert part in p


@pytest.mark.parametrize("bad", [None, 3, ["F hello"], {"a": 1}])
def test_check_non_string_is_rejected_not_raised(bad):
    errors = check(bad, PROPOSITIONS)
    assert errors and all(isinstance(e, str) for e in errors)
    assert any("文字列" in e for e in errors)


@pytest.mark.parametrize("formula", ["hello - danced", "F hello > danced", "F -hello", "F hello -"])
def test_check_rejects_stray_minus_and_gt_with_reason(formula):
    errors = check(formula, PROPOSITIONS)
    assert any("->" in e for e in errors)
    assert not any("読めません" in e for e in errors)  # 文法の検査で落ちる（flloat まで行かない）


def test_check_still_accepts_implication():
    assert check("G(hello -> F danced)", PROPOSITIONS) == []


def test_translator_uses_llm_type_from_llm_module():
    from voice_ltl import llm as llm_module
    from voice_ltl import translator
    assert translator.LLM is llm_module.LLM
