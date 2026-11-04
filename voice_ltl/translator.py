"""Instruction -> LTLf formula, with up to max_tries re-asks on rejected formulas."""

import re
from dataclasses import dataclass, field

from voice_ltl.grammar import check
from voice_ltl.llm import LLM
from voice_ltl.prompt import SCHEMA, build_prompt
from voice_ltl.world import MAX_REPEAT, PROPOSITIONS, STEP_ANGLE, STEP_DISTANCE

# An unsupported part that is a number with a unit (0メートル, -2度): design §7.4.0. The prompt
# rounds off-unit numbers and caps big ones, so this is for what is still left (0, minus) or for an
# LLM that does not follow it. Digits without a unit (2階, 3番目の部屋) are a word not in the vocabulary.
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
    # what was read differently from the words, for the page: [{"said": ..., "as": ...}]
    substitutions: list[dict[str, str]] = field(default_factory=list)


MAX_SUBSTITUTIONS = 10


def _substitutions(raw: object) -> list[dict[str, str]]:
    """The reply's substitutions as {said, as} strings. The CLI's JSON Schema already requires
    them (a reply without the key is unreadable, design §7.4.1); this guards replies that come
    another way (FakeLLM, run_formula). The page clips them after hiding secrets (session.py)."""
    if not isinstance(raw, list):
        return []
    out = []
    for item in raw[:MAX_SUBSTITUTIONS]:
        if isinstance(item, dict) and isinstance(item.get("said"), str) and isinstance(item.get("as"), str):
            said, as_ = item["said"].strip(), item["as"].strip()
            if said and as_:
                out.append({"said": said, "as": as_})
    return out


def _is_number(item: str) -> bool:
    # Only an item that IS a number with a unit (0.7メートル, 約45度) gets the number message.
    # 「障害物の周りを一周する」 contains 一周 but is a behavior the vocabulary lacks.
    return re.fullmatch(r"\s*(?:約|およそ)?\s*(?:" + _NUMBER.pattern + r")\s*(?:だけ)?\s*", item) is not None


def _unsupported_message(unsupported: list[str]) -> str:
    numbers = [u for u in unsupported if _is_number(u)]
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
    earlier_subs: list[dict[str, str]] = []   # from a rejected answer: kept if the fix drops them
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
        subs = _substitutions(reply.get("substitutions"))
        if not errors:
            return Translation(True, formula, [], attempts, "", subs or earlier_subs)
        earlier_subs = subs or earlier_subs
        rejected, error = formula, " ".join(errors)
    return Translation(
        False, None, [], attempts,
        f"指示を式に直せませんでした（{max_tries}回試しました）。言い方を変えてもう一度お願いします。",
    )
