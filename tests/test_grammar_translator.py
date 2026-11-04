from voice_ltl.grammar import check
from voice_ltl.prompt import build_prompt
from voice_ltl.translator import translate
from voice_ltl.world import PROPOSITIONS


class FakeLLM:
    """Temporary stand-in (Task 6 swaps it for voice_ltl.llm.FakeLLM)."""

    def __init__(self, replies):
        self.replies = list(replies)
        self.prompts = []

    def __call__(self, prompt, schema):
        self.prompts.append(prompt)
        return self.replies[len(self.prompts) - 1]


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
