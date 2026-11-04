"""LTLf on finite traces by formula progression (lecture notes 5.2).

flloat only reads the formula. `parse` turns flloat's tree into a Node in negation
normal form; `progress` gives what is still to be satisfied after one label; and
`build_dfa` explores, breadth first, only the labels it is given (the plant's), so it
never enumerates all 2^atoms letters the way flloat's DFA translation does.

A Node is an immutable tuple:
    ("true",) ("false",) ("last",)
    ("ap", name, positive)           an atom, or its negation when positive is False
    ("and", frozenset) ("or", frozenset)   always an `|` of `&`s, see conj / disj
    ("X", a) ("WX", a)               strong / weak next
    ("F", a) ("G", a) ("U", a, b) ("R", a, b)
    ("strong", a) ("weak", a)        a must hold from the next position on; strong also
                                     needs that position to exist, weak is met if the trace ends
WX and R only come from pushing a negation inward; the grammar (check()) has neither.

`holds` is the reference semantics, computed straight from the definition on flloat's
own tree. It shares nothing with `parse`'s normal form or `progress`, and is the oracle
the tests compare against.
"""

import threading
import time
from collections import deque
from contextlib import contextmanager
from functools import lru_cache
from typing import Iterable, Sequence

from flloat import ltlf as fl
from flloat.parser.ltlf import LTLfParser

Node = tuple

TRUE: Node = ("true",)
FALSE: Node = ("false",)
LAST: Node = ("last",)

# A node with one of these tags holds on the empty rest of a trace (see accepts_empty).
_HOLDS_ON_EMPTY = frozenset({"true", "WX", "weak", "G", "R", "last"})

_parser_lock = threading.Lock()
_parser: LTLfParser | None = None


TOO_BIG = "式が大きすぎます"
DEFAULT_MAX_CLAUSES = 5000


class _Limits(threading.local):
    """Per-thread budget for the DNF conversion (set by build_dfa, see _limited)."""
    deadline: float | None = None
    max_clauses: int = DEFAULT_MAX_CLAUSES


_limits = _Limits()


@contextmanager
def _limited(deadline: float | None, max_clauses: int):
    saved = (_limits.deadline, _limits.max_clauses)
    _limits.deadline, _limits.max_clauses = deadline, max_clauses
    try:
        yield
    finally:
        _limits.deadline, _limits.max_clauses = saved


def _check_budget(count: int = 0) -> None:
    """ValueError when `count` clauses exceed the limit or the deadline has passed."""
    if count > _limits.max_clauses or (
            _limits.deadline is not None and time.monotonic() > _limits.deadline):
        raise ValueError(TOO_BIG)


@lru_cache(maxsize=256)
def _flloat_tree(formula: str):
    global _parser
    with _parser_lock:
        if _parser is None:
            _parser = LTLfParser()
        try:
            return _parser(formula)
        except Exception as exc:  # lark and flloat raise several error types for bad input
            raise ValueError(f"式が読めません: {formula}") from exc


# `&` and `|` are kept in one normal form, an `|` of `&`s (DNF), so that formulas equal up
# to associativity, commutativity, repetition, true/false and absorption are the same
# tuple. Flattening alone is not enough for a finite DFA: progressing `(G a) U (F b)`
# nests `(F b | (G a & U)) & G a | F b` one level deeper at every step.
Clause = frozenset  # frozenset[Node]: an `&` of nodes that are neither `&` nor `|`


def _clauses(node: Node) -> set[Clause]:
    if node == TRUE:
        return {frozenset()}
    if node == FALSE:
        return set()
    if node[0] == "or":
        return {_clause(part) for part in node[1]}
    return {_clause(node)}


def _clause(node: Node) -> Clause:
    return node[1] if node[0] == "and" else frozenset({node})


def _absorb(clauses: set[Clause]) -> set[Clause]:
    """Drop every clause that contains another one: a | (a & b) = a."""
    return {c for c in clauses if not any(d < c for d in clauses)}


def _from_clauses(clauses: set[Clause]) -> Node:
    if not clauses:
        return FALSE
    if frozenset() in clauses:
        return TRUE
    nodes = {next(iter(c)) if len(c) == 1 else ("and", c) for c in clauses}
    return nodes.pop() if len(nodes) == 1 else ("or", frozenset(nodes))


def conj(parts: Iterable[Node]) -> Node:
    clauses: set[Clause] = {frozenset()}
    for part in parts:
        theirs = _clauses(part)
        _check_budget(len(clauses) * len(theirs))  # before the product is built
        clauses = _absorb({c | d for c in clauses for d in theirs})
        if not clauses:
            return FALSE
    return _from_clauses(clauses)


def disj(parts: Iterable[Node]) -> Node:
    clauses: set[Clause] = set()
    for part in parts:
        clauses |= _clauses(part)
        _check_budget(len(clauses))
    return _from_clauses(_absorb(clauses))


def strong(a: Node) -> Node:
    # Strong(a) is a when a already fails on the empty rest, e.g. Strong(F a) is F a.
    return a if not accepts_empty(a) else ("strong", a)


def weak(a: Node) -> Node:
    # Weak(a) is a when a already holds on the empty rest, e.g. Weak(G a) is G a.
    return a if accepts_empty(a) else ("weak", a)


def _second(formulas, cls):
    """flloat keeps `a U b U c` as one node with three parts; split off the first."""
    return formulas[1] if len(formulas) == 2 else cls(formulas[1:])


def _nnf(f, negate: bool) -> Node:
    """flloat's tree -> Node, with the negation (when `negate`) pushed down to the atoms."""
    if isinstance(f, fl.LTLfTrue):  # LTLfTrue / LTLfFalse are subclasses of LTLfAtomic
        return FALSE if negate else TRUE
    if isinstance(f, fl.LTLfFalse):
        return TRUE if negate else FALSE
    if isinstance(f, fl.LTLfAtomic):
        return ("ap", f.s, not negate)
    if isinstance(f, fl.LTLfNot):
        return _nnf(f.f, not negate)
    if isinstance(f, fl.LTLfAnd):
        parts = [_nnf(g, negate) for g in f.formulas]
        return disj(parts) if negate else conj(parts)
    if isinstance(f, fl.LTLfOr):
        parts = [_nnf(g, negate) for g in f.formulas]
        return conj(parts) if negate else disj(parts)
    if isinstance(f, fl.LTLfImplies):
        # flloat reads a -> b -> c as (a -> b) -> c
        left = f.formulas[0] if len(f.formulas) == 2 else fl.LTLfImplies(f.formulas[:-1])
        right = f.formulas[-1]
        if negate:
            return conj([_nnf(left, False), _nnf(right, True)])
        return disj([_nnf(left, True), _nnf(right, False)])
    if isinstance(f, fl.LTLfNext):
        return ("WX", _nnf(f.f, True)) if negate else ("X", _nnf(f.f, False))
    if isinstance(f, fl.LTLfEventually):
        return ("G", _nnf(f.f, True)) if negate else ("F", _nnf(f.f, False))
    if isinstance(f, fl.LTLfAlways):
        return ("F", _nnf(f.f, True)) if negate else ("G", _nnf(f.f, False))
    if isinstance(f, fl.LTLfUntil):
        # flloat reads a U b U c as a U (b U c)
        a, b = f.formulas[0], _second(f.formulas, fl.LTLfUntil)
        return ("R", _nnf(a, True), _nnf(b, True)) if negate else ("U", _nnf(a, False), _nnf(b, False))
    if isinstance(f, fl.LTLfLast):
        return ("X", TRUE) if negate else LAST
    raise ValueError(f"使えない演算子です: {f}")


@lru_cache(maxsize=256)
def parse(formula: str) -> Node:
    """Read `formula` with flloat and return it as a Node. ValueError if it cannot be read."""
    return _nnf(_flloat_tree(formula), False)


def accepts_empty(node: Node) -> bool:
    """True when `node` is satisfied if the trace ends here (nothing more is read)."""
    tag = node[0]
    if tag == "and":
        return all(accepts_empty(p) for p in node[1])
    if tag == "or":
        return any(accepts_empty(p) for p in node[1])
    return tag in _HOLDS_ON_EMPTY


def progress(node: Node, label: frozenset[str]) -> Node:
    """What is still to be satisfied after reading `label` at the current position."""
    tag = node[0]
    if tag in ("true", "false"):
        return node
    if tag == "ap":
        return TRUE if (node[1] in label) == node[2] else FALSE
    if tag == "and":
        return conj(progress(p, label) for p in node[1])
    if tag == "or":
        return disj(progress(p, label) for p in node[1])
    if tag == "X":
        return strong(node[1])
    if tag == "WX":
        return weak(node[1])
    if tag in ("strong", "weak"):
        return progress(node[1], label)
    if tag == "F":
        return disj([progress(node[1], label), strong(node)])
    if tag == "G":
        return conj([progress(node[1], label), weak(node)])
    if tag == "U":
        a, b = node[1], node[2]
        return disj([progress(b, label), conj([progress(a, label), strong(node)])])
    if tag == "R":
        a, b = node[1], node[2]
        return conj([progress(b, label), disj([progress(a, label), weak(node)])])
    if tag == "last":
        return weak(FALSE)
    raise ValueError(f"unknown node {node!r}")


def _atoms(node: Node) -> frozenset[str]:
    if node[0] == "ap":
        return frozenset({node[1]})
    if node[0] in ("and", "or"):
        return frozenset().union(*(_atoms(p) for p in node[1]))
    return frozenset().union(*(_atoms(p) for p in node[1:] if isinstance(p, tuple)))


def build_dfa(
    formula: str,
    initial_label: frozenset[str],
    labels: Iterable[frozenset[str]],
    max_states: int = 2000,
    deadline: float | None = None,
    max_clauses: int = DEFAULT_MAX_CLAUSES,
) -> tuple[list[Node], dict[tuple[int, frozenset[str]], int], set[int]]:
    """The DFA of `formula` over `labels`, by progression from the initial label.

    State 0 is progress(parse(formula), initial_label). Returns (states, transitions,
    accepting): transitions maps (state, label) -> state for every given label.
    ValueError("式が大きすぎます") when there would be more than max_states states, a DNF
    would have more than max_clauses clauses, or time.monotonic() passes `deadline`.
    """
    with _limited(deadline, max_clauses):
        return _build_dfa(formula, initial_label, labels, max_states)


def _build_dfa(formula, initial_label, labels, max_states):
    root = parse(formula)
    atoms = _atoms(root)
    labels = list(dict.fromkeys(frozenset(lab) for lab in labels))
    start = progress(root, frozenset(initial_label))
    states = [start]
    index = {start: 0}
    trans: dict[tuple[int, frozenset[str]], int] = {}
    queue = deque([0])
    while queue:
        _check_budget()
        q = queue.popleft()
        by_atoms: dict[frozenset[str], int] = {}  # only the formula's atoms change the result
        for lab in labels:
            key = lab & atoms
            if key not in by_atoms:
                nxt = progress(states[q], key)
                if nxt not in index:
                    if len(states) >= max_states:
                        raise ValueError(TOO_BIG)
                    index[nxt] = len(states)
                    states.append(nxt)
                    queue.append(index[nxt])
                by_atoms[key] = index[nxt]
            trans[(q, lab)] = by_atoms[key]
    accepting = {i for i, s in enumerate(states) if accepts_empty(s)}
    return states, trans, accepting


def holds(formula: str, trace: Sequence[frozenset[str]]) -> bool:
    """LTLf truth of `formula` on the non-empty finite `trace`, from the definition (slow)."""
    if not trace:
        raise ValueError("列が空です")
    return _holds(_flloat_tree(formula), list(trace), 0)


def _holds(f, trace: list[frozenset[str]], i: int) -> bool:
    n = len(trace)
    if isinstance(f, fl.LTLfTrue):
        return True
    if isinstance(f, fl.LTLfFalse):
        return False
    if isinstance(f, fl.LTLfAtomic):
        return f.s in trace[i]
    if isinstance(f, fl.LTLfNot):
        return not _holds(f.f, trace, i)
    if isinstance(f, fl.LTLfAnd):
        return all(_holds(g, trace, i) for g in f.formulas)
    if isinstance(f, fl.LTLfOr):
        return any(_holds(g, trace, i) for g in f.formulas)
    if isinstance(f, fl.LTLfImplies):
        # flloat reads a -> b -> c as (a -> b) -> c
        value = _holds(f.formulas[0], trace, i)
        for g in f.formulas[1:]:
            value = (not value) or _holds(g, trace, i)
        return value
    if isinstance(f, fl.LTLfNext):  # strong: the next position must exist
        return i + 1 < n and _holds(f.f, trace, i + 1)
    if isinstance(f, fl.LTLfEventually):  # a witness inside the trace
        return any(_holds(f.f, trace, j) for j in range(i, n))
    if isinstance(f, fl.LTLfAlways):
        return all(_holds(f.f, trace, j) for j in range(i, n))
    if isinstance(f, fl.LTLfUntil):
        # flloat reads a U b U c as a U (b U c)
        a, b = f.formulas[0], _second(f.formulas, fl.LTLfUntil)
        return any(_holds(b, trace, j) and all(_holds(a, trace, k) for k in range(i, j))
                   for j in range(i, n))
    if isinstance(f, fl.LTLfLast):
        return i == n - 1
    raise ValueError(f"使えない演算子です: {f}")
