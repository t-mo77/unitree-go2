"""Instruction -> LTLf formula, with up to max_tries re-asks on rejected formulas."""

import re
from dataclasses import dataclass, field

from voice_ltl.grammar import check
from voice_ltl.llm import LLM
from voice_ltl.prompt import SCHEMA, build_prompt
from voice_ltl.world import MAX_REPEAT, PROPOSITIONS, STEP_ANGLE, STEP_DISTANCE

# An unsupported part that is a number with a unit (0.7メートル, 45度, 七回): design §7.4.0.
# Digits without one (2階, 3番目の部屋) are a word that is not in the vocabulary.
_NUMBER = re.compile(r"(?:\d+(?:[.．]\d+)?|[〇一二三四五六七八九十百千]+)\s*"
                     r"(?:メートル|センチ|ミリ|cm|mm|m|ｍ|度|°|回|歩|周)(?![A-Za-z])")


@dataclass
class Attempt:
    formula: str | None
    unsupported: list[str]
    errors: list[str]


@dataclass
class Translation:
    ok: bool
    formula: str | None
    unsupported: list[str]
    attempts: list[Attempt] = field(default_factory=list)
    message: str = ""


def _unsupported_message(unsupported: list[str]) -> str:
    numbers = [u for u in unsupported if _NUMBER.search(u)]
    words = [u for u in unsupported if u not in numbers]
    parts = []
    if words or not numbers:
        what = "、".join(words) if words else "この指示"
        parts.append(f"「{what}」は単語帳にないので、まだできません。")
    if numbers:
        parts.append(f"「{'、'.join(numbers)}」：この数は指定できません"
                     f"（{STEP_DISTANCE:g} m・{STEP_ANGLE:g} 度・1 回を単位に、{MAX_REPEAT} 回まで）。")
    return "".join(parts)


def translate(instruction: str, llm: LLM, max_tries: int = 3) -> Translation:
    """LLMError is not caught here; the caller handles it."""
    attempts: list[Attempt] = []
    rejected: str | None = None
    error: str | None = None
    for _ in range(max_tries):
        reply = llm(build_prompt(instruction, rejected, error), SCHEMA)
        formula = reply.get("formula")
        raw = reply.get("unsupported")
        if raw is None or raw == "":
            raw = []
        elif isinstance(raw, str):
            raw = [raw]
        if not isinstance(raw, list):
            errors = [f"unsupported は文字列の配列で渡してください（{type(raw).__name__} が渡されました）。"]
            attempts.append(Attempt(formula if isinstance(formula, str) else None, [], errors))
            rejected, error = str(formula), " ".join(errors)
            continue
        unsupported = [str(u) for u in raw]
        if unsupported or formula is None:
            attempts.append(Attempt(formula, unsupported, []))
            return Translation(False, None, unsupported, attempts, _unsupported_message(unsupported))
        errors = check(formula, PROPOSITIONS)
        attempts.append(Attempt(formula, [], errors))
        if not errors:
            return Translation(True, formula, [], attempts, "")
        rejected, error = formula, " ".join(errors)
    return Translation(
        False, None, [], attempts,
        f"指示を式に直せませんでした（{max_tries}回試しました）。言い方を変えてもう一度お願いします。",
    )
