"""Instruction -> LTLf formula, with up to max_tries re-asks on rejected formulas."""

from dataclasses import dataclass, field

from voice_ltl.grammar import check
from voice_ltl.llm import LLM
from voice_ltl.prompt import SCHEMA, build_prompt
from voice_ltl.world import PROPOSITIONS


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


def translate(instruction: str, llm: LLM, max_tries: int = 3) -> Translation:
    """LLMError is not caught here; the caller handles it."""
    attempts: list[Attempt] = []
    rejected: str | None = None
    error: str | None = None
    for _ in range(max_tries):
        reply = llm(build_prompt(instruction, rejected, error), SCHEMA)
        formula = reply.get("formula")
        unsupported = [str(u) for u in (reply.get("unsupported") or [])]
        if unsupported or formula is None:
            attempts.append(Attempt(formula, unsupported, []))
            what = "、".join(unsupported) if unsupported else "この指示"
            return Translation(
                False, None, unsupported, attempts,
                f"「{what}」は単語帳にないので、まだできません。",
            )
        errors = check(formula, PROPOSITIONS)
        attempts.append(Attempt(formula, [], errors))
        if not errors:
            return Translation(True, formula, [], attempts, "")
        rejected, error = formula, " ".join(errors)
    return Translation(
        False, None, [], attempts,
        f"指示を式に直せませんでした（{max_tries}回試しました）。言い方を変えてもう一度お願いします。",
    )
