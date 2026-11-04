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


def test_unsupported_number_says_it_cannot_be_given():
    # design §7.4.0: a number off the 0.5 m / 90 度 / 1回 units -> 「この数は指定できません」
    llm = FakeLLM([{"formula": None, "unsupported": ["0.7メートル"]}])
    t = translate("0.7メートル進んで", llm)
    assert not t.ok and "0.7メートル" in t.message and "指定できません" in t.message
    assert "単語帳" not in t.message
    llm = FakeLLM([{"formula": None, "unsupported": ["キッチン", "七回"]}])
    t = translate("キッチンで七回挨拶して", llm)
    assert "「キッチン」は単語帳にない" in t.message and "「七回」" in t.message
    assert "指定できません" in t.message


def test_prompt_has_four_parts():
    p = build_prompt("挨拶して")
    for part in ("Grammar", "Vocabulary", "Examples", "挨拶して", "unsupported"):
        assert part in p
