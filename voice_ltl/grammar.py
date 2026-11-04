"""Check an LTLf formula against the vocabulary. Never repairs the formula."""

import re
from typing import Sequence

from flloat.parser.ltlf import LTLfParser

MAX_FORMULA_LEN = 300
KEYWORDS = ("F", "G", "X", "U", "last")
_ALLOWED_SYMBOLS = set("!&|->() \t")
_NAME = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")


def _parens_balanced(formula: str) -> bool:
    depth = 0
    for ch in formula:
        if ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
            if depth < 0:
                return False
    return depth == 0


def check(formula: object, propositions: Sequence[str]) -> list[str]:
    """Return Japanese rejection reasons; an empty list means the formula passes."""
    if not isinstance(formula, str):
        return [f"式は文字列で渡してください（{type(formula).__name__} が渡されました）。"]
    if not formula.strip():
        return ["式が空です。"]
    errors: list[str] = []
    if len(formula) > MAX_FORMULA_LEN:
        errors.append(f"式が長すぎます（{len(formula)}文字。{MAX_FORMULA_LEN}文字以下にしてください）。")
    if not _parens_balanced(formula):
        errors.append("括弧の対応が取れていません。")
    allowed = set(propositions) | set(KEYWORDS)
    unknown = []
    for name in _NAME.findall(formula):
        if name not in allowed and name not in unknown:
            unknown.append(name)
    for name in unknown:
        errors.append(f"単語帳にない名前 {name} が使われています。")
    rest = _NAME.sub(" ", formula)
    bad = sorted({c for c in rest if c not in _ALLOWED_SYMBOLS})
    if bad:
        errors.append("使えない記号があります：" + " ".join(bad))
    stray = sorted(set(rest.replace("->", "  ")) & {"-", ">"})
    if stray:
        errors.append("- と > は -> （ならば）の形でだけ使えます：" + " ".join(stray))
    if not errors:
        try:
            LTLfParser()(formula)
        except Exception as e:  # flloat/lark raise various error types
            errors.append(f"式として読めません（{type(e).__name__}）。")
    return errors
