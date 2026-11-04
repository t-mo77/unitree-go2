"""Prompt for the instruction -> LTLf translation (Goal / Grammar / Vocabulary / Examples / Query)."""

import json

from voice_ltl.world import MAX_REPEAT, PROP_DESCRIPTION_JA, PROPOSITIONS, STEP_ANGLE, STEP_DISTANCE

SCHEMA = {
    "type": "object",
    "properties": {
        "formula": {"type": ["string", "null"]},
        "unsupported": {"type": "array", "items": {"type": "string"}},
    },
    "required": ["formula", "unsupported"],
    "additionalProperties": False,
}

_EXAMPLES = (
    ("立って挨拶して", "F hello", []),
    ("挨拶してからダンスして", "F(hello & F danced)", []),
    ("挨拶するまで座らないで", "!sitting U hello", []),
    ("1メートル進んで", "F(moved_forward & X F moved_forward)", []),
    ("キッチンへ行って", None, ["キッチン"]),
    ("45度回って", None, ["45度"]),
)


def _example_line(instruction: str, formula: str | None, unsupported: list[str]) -> str:
    answer = json.dumps({"formula": formula, "unsupported": unsupported}, ensure_ascii=False)
    return f"指示: {instruction}\n答え: {answer}"


def build_prompt(instruction: str, rejected: str | None = None, error: str | None = None) -> str:
    vocab = "\n".join(f"- {p}: {PROP_DESCRIPTION_JA[p]}" for p in PROPOSITIONS)
    examples = "\n\n".join(_example_line(*e) for e in _EXAMPLES)
    text = f"""## Goal
ロボット犬への日本語の指示を、LTLf（有限列の線形時相論理）の式に翻訳してください。
答えは JSON で、formula（式の文字列。翻訳できないときは null）と unsupported（表せなかった部分の一覧）を返します。

## Grammar
使ってよい名前: 下の Vocabulary の命題、F、G、X、U、last
使ってよい記号: ! & | -> ( )
- F φ: いつか φ になる
- G φ: ずっと φ である
- X φ: 次の動作で φ になる（X は「次の動作」の意味）
- φ U ψ: ψ になるまで φ が続く
- last: 最後の時点
- ! は否定、& は かつ、| は または、-> は ならば

## Vocabulary
{vocab}

単語帳で表せない部分は、近い言葉に置き換えず unsupported に入れて、formula を null にしてください。
数は {STEP_DISTANCE} m・{STEP_ANGLE:g} 度・1回を単位とした繰り返しで表します。繰り返しは {MAX_REPEAT} 回までです。割り切れない数や、それを超える数は unsupported に入れてください。

## Examples
{examples}

## Query
指示: {instruction}
答え:"""
    if rejected is not None:
        text += (
            f"\n\n前の答えは {rejected} で、{error} という理由で却下されました。"
            "直した式だけを返してください。"
        )
    return text
