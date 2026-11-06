"""Instruction -> LTLf formula, with up to max_tries re-asks on rejected formulas."""

import math
import re
import unicodedata
from dataclasses import dataclass, field

from voice_ltl.grammar import check
from voice_ltl.llm import LLM
from voice_ltl.objects import (MAX_OBJECT_DISTANCE, Scene, label_in, msg_ambiguous, msg_not_found, msg_too_far,
                               split_prop)
from voice_ltl.prompt import SCHEMA, build_prompt
from voice_ltl.world import MAX_REPEAT, PROPOSITIONS, STEP_ANGLE, STEP_DISTANCE

# An unsupported part that is a number with a unit (0メートル, -2度): design §7.4.0. The prompt
# rounds off-unit numbers and caps big ones, so this is for what is still left (0, minus) or for an
# LLM that does not follow it. Digits without a unit (2階, 3番目の部屋) are a word not in the vocabulary.
_NUMBER = re.compile(r"(?:マイナス|[-−－])?\s*(?:\d+(?:[.．]\d+)?|[〇一二三四五六七八九十百千]+|ゼロ|零)\s*"
                     r"(?:メートル|センチ|ミリ|cm|mm|m|ｍ|度|°|回|歩|周|秒間|秒|分間|分|時間)(?![A-Za-z])")


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
    # the objects and pose frozen for this instruction (design §6.3); None for a link without objects
    scene: Scene | None = None


MAX_SUBSTITUTIONS = 10


def _plain(text: str) -> str:
    return re.sub(r"\s+", "", unicodedata.normalize("NFKC", text))


_TRAILING_NOTE = re.compile(r"\s*[（(][^（()）]*[）)]\s*$")


def _substitutions(raw: object, instruction: str) -> list[dict[str, str]]:
    """The reply's substitutions as {said, as} strings. The CLI's JSON Schema already requires
    them (a reply without the key is unreadable, design §7.4.1); this guards replies that come
    another way (FakeLLM, run_formula). `said` must be words of the instruction: a note the LLM
    added in brackets is cut off, and a said copied from the prompt's examples is dropped.
    The page clips them after hiding secrets (session.py)."""
    if not isinstance(raw, list):
        return []
    words = _plain(instruction)
    out = []
    for item in raw[:MAX_SUBSTITUTIONS]:
        if isinstance(item, dict) and isinstance(item.get("said"), str) and isinstance(item.get("as"), str):
            said, as_ = _TRAILING_NOTE.sub("", item["said"]).strip(), item["as"].strip()
            if said and as_ and _plain(said) in words:
                out.append({"said": said, "as": as_})
    return out


def _is_number(item: str) -> bool:
    # Only an item that IS a number with a unit (0.7メートル, 約45度) gets the number message.
    # 「障害物の周りを一周する」 contains 一周 but is a behavior the vocabulary lacks.
    return re.fullmatch(r"\s*(?:約|およそ)?\s*(?:" + _NUMBER.pattern + r")\s*(?:だけ)?\s*", item) is not None


# Handling an object (design §3: not this step). An item with one of these is never an object's name,
# so 「箱を持つ」 keeps the 単語帳 message however many boxes there are (R16).
OBJECT_ACTION_WORDS = ("持", "押", "運", "拾", "動か", "どけ", "どか")


def _object_messages(unsupported: list[str], scene: Scene | None) -> tuple[list[str], list[str]]:
    """R16: an item naming a label (箱, 椅子, ...) that has 0 objects in the scene is 見つかりません and
    with 2 or more it is 決められません, once per label. Returns (those messages, the other items);
    an item whose label has exactly one object keeps today's message."""
    listed = scene.listed() if scene is not None else ()
    messages, rest, told = [], [], set()
    for item in unsupported:
        label = None if any(w in item for w in OBJECT_ACTION_WORDS) else label_in(item)
        n = sum(o.label == label for o in listed)
        if label is None or n == 1:
            rest.append(item)
        elif label not in told:
            told.add(label)
            messages.append(msg_not_found(label) if n == 0 else msg_ambiguous(label, n))
    return messages, rest


def _unsupported_message(unsupported: list[str], scene: Scene | None = None) -> str:
    parts, unsupported = _object_messages(unsupported, scene)
    numbers = [u for u in unsupported if _is_number(u)]
    words = [u for u in unsupported if u not in numbers]
    if words or not (numbers or parts):
        what = "、".join(words) if words else "この指示"
        parts.append(f"「{what}」は単語帳にないので、まだできません。")
    if numbers:
        parts.append(f"「{'、'.join(numbers)}」：この数は指定できません"
                     f"（0・マイナスの数と、秒・分などの時間は使えません。"
                     f"{STEP_DISTANCE:g} m・{STEP_ANGLE:g} 度・1 回を単位に、{MAX_REPEAT} 回まで）。")
    return "".join(parts)


def _too_far(formula: str, scene: Scene | None) -> str | None:
    """P5: the label of the first object in the formula whose centre is more than MAX_OBJECT_DISTANCE
    from the frozen pose, so the Go2 does not stand up first. The grid checks again from its anchor."""
    if scene is None or scene.pose is None:
        return None
    for name in re.findall(r"\w+", formula):
        prop = split_prop(name)
        obj = scene.find(prop[1]) if prop else None
        if obj is not None and math.hypot(obj.x - scene.pose[0], obj.y - scene.pose[1]) > MAX_OBJECT_DISTANCE:
            return obj.label
    return None


def translate(instruction: str, llm: LLM, max_tries: int = 3, scene: Scene | None = None) -> Translation:
    """LLMError is not caught here; the caller handles it. `scene` is the object list frozen for this
    instruction: its propositions join the vocabulary, and it is returned on every Translation."""
    propositions = PROPOSITIONS + (scene.propositions() if scene is not None else ())
    attempts: list[Attempt] = []
    rejected: str | None = None
    error: str | None = None
    earlier_subs: list[dict[str, str]] = []   # from a rejected answer: kept if the fix drops them
    for _ in range(max_tries):
        reply = llm(build_prompt(instruction, rejected, error, scene=scene), SCHEMA)
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
            return Translation(False, None, unsupported, attempts, _unsupported_message(unsupported, scene),
                               scene=scene)
        errors = check(formula, propositions)
        attempts.append(Attempt(formula, [], errors))
        subs = _substitutions(reply.get("substitutions"), instruction)
        if not errors:
            far = _too_far(formula, scene)
            if far is not None:
                return Translation(False, None, [], attempts, msg_too_far(far), scene=scene)
            return Translation(True, formula, [], attempts, "", subs or earlier_subs, scene=scene)
        earlier_subs = subs or earlier_subs
        rejected, error = formula, " ".join(errors)
    if attempts and any("長すぎ" in e for e in attempts[-1].errors):
        return Translation(False, None, [], attempts,
                           "指示が長すぎて式にできませんでした。いくつかに分けて言ってください。", scene=scene)
    return Translation(
        False, None, [], attempts,
        f"指示を式に直せませんでした（{max_tries}回試しました）。言い方を変えてもう一度お願いします。",
        scene=scene,
    )
