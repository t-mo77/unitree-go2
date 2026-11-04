import itertools
import random
import re
import time

import pytest
from flloat.parser.ltlf import LTLfParser
from go2_nl_control.actions import LYING

from voice_ltl.automata import synthesize
from voice_ltl.ltlf import build_dfa, holds, parse
from voice_ltl.world import PlantState, go2_world

UNARY = ("!", "F", "G", "X")
BINARY = ("&", "|", "->", "U")


def all_labels(props) -> list[frozenset[str]]:
    return [frozenset(c) for n in range(len(props) + 1) for c in itertools.combinations(props, n)]


def random_formula(rng: random.Random, props, depth: int) -> str:
    """A random formula over props with ! F G X & | -> U, last, true and false."""
    if depth == 0 or rng.random() < 0.15:
        return rng.choice(list(props) * 4 + ["last", "true", "false"])
    if rng.random() < 0.5:
        return f"{rng.choice(UNARY)}({random_formula(rng, props, depth - 1)})"
    left, right = random_formula(rng, props, depth - 1), random_formula(rng, props, depth - 1)
    return f"({left}) {rng.choice(BINARY)} ({right})"


def run(trans, trace) -> int:
    """The DFA state after the trace (its first label is already in state 0)."""
    q = 0
    for lab in trace[1:]:
        q = trans[(q, lab)]
    return q


def test_lecture_three_states():
    labels = [frozenset(), frozenset({"at_goal"}), frozenset({"collide"})]
    states, trans, acc = build_dfa("F at_goal & G !collide", frozenset(), labels)
    assert len(states) == 3 and len(acc) == 1
    assert states[0] == parse("F at_goal & G !collide")
    assert set(states) == {parse("F at_goal & G !collide"), parse("G !collide"), parse("false")}
    assert [states[i] for i in acc] == [parse("G !collide")]
    assert len(trans) == 3 * 3


@pytest.mark.parametrize("seed", range(5))
def test_progression_matches_reference_semantics(seed):
    rng = random.Random(seed)
    props = ["a", "b", "c"]
    labels = all_labels(props)
    for _ in range(200):
        f = random_formula(rng, props, depth=3)
        trace = [frozenset(p for p in props if rng.random() < 0.4) for _ in range(rng.randint(1, 6))]
        states, trans, acc = build_dfa(f, trace[0], labels)
        assert (run(trans, trace) in acc) == holds(f, trace), (f, trace)


SMALL_FORMULAS = [
    "F a", "G !a", "a U b", "F(a & X F b)", "(!a U b) & F a", "F(last & a)", "X a", "!X a",
    "a -> b -> a", "a U b U a", "G(a -> X b)", "!(a U b)", "!last & a", "true", "F false",
]


@pytest.mark.parametrize("formula", SMALL_FORMULAS)
def test_progression_agrees_with_flloat_on_small_formulas(formula):
    """flloat's own DFA, build_dfa and holds give the same answer on every trace of length 1 to 4."""
    props = [p for p in ("a", "b") if re.search(rf"\b{p}\b", formula)]
    dfa = LTLfParser()(formula).to_automaton()
    labels = all_labels(props)
    for n in range(1, 5):
        for trace in itertools.product(labels, repeat=n):
            states, trans, acc = build_dfa(formula, trace[0], labels)
            expected = dfa.accepts([{p: p in lab for p in props} for lab in trace])
            assert (run(trans, trace) in acc) == expected, (formula, trace)
            assert holds(formula, list(trace)) == expected, (formula, trace)


def test_holds_follows_the_ltlf_definition():
    a, none = frozenset({"a"}), frozenset()
    assert not holds("X a", [a])            # strong next needs a next position
    assert holds("!X a", [a])
    assert not holds("F a", [none, none])   # F needs a witness inside the trace
    assert holds("G a", [a])
    assert not holds("last", [a, a]) and holds("X last", [a, a]) and holds("last", [a])
    assert holds("a U a", [a]) and not holds("!a U a", [none])


def test_too_many_states_raises():
    labels = all_labels(["a", "b", "c"])
    states, _, _ = build_dfa("F a & F b & F c", frozenset(), labels, max_states=8)
    assert len(states) == 8  # one state per set of goals still open
    with pytest.raises(ValueError, match="式が大きすぎます"):
        build_dfa("F a & F b & F c", frozenset(), labels, max_states=7)


def test_states_do_not_grow_under_until():
    # Without absorption, progressing this nests one level deeper at every step.
    states, _, acc = build_dfa("(G a) U (F b)", frozenset({"a"}), all_labels(["a", "b"]))
    assert set(states) == {parse("F b | ((G a) & ((G a) U (F b)))"), parse("F b"), parse("true")}
    assert [states[i] for i in acc] == [parse("true")]


@pytest.mark.parametrize("bad", ["F (a", "a R b", "WX a", ""])
def test_parse_rejects_what_it_cannot_read(bad):
    with pytest.raises(ValueError):
        parse(bad)


def test_negation_normal_form():
    assert parse("!F a") == parse("G !a")
    assert parse("!G a") == parse("F !a")
    assert parse("b & a & a") == parse("a & b")
    assert parse("a -> b") == parse("!a | b")
    assert parse("!last") == parse("X true")
    assert parse("a & true") == parse("a") and parse("a & false") == parse("false")


def test_six_repeats_fast(tmp_path):
    f = "F(hello & X F(hello & X F(hello & X F(hello & X F(hello & X F hello)))))"
    t0 = time.monotonic()
    s = synthesize(f, go2_world(PlantState(LYING, False, False)), tmp_path)
    assert time.monotonic() - t0 < 5.0
    assert not s.empty


def test_deadline_and_clause_limit_raise():
    labels = all_labels(["a", "b"])
    with pytest.raises(ValueError, match="式が大きすぎます"):
        build_dfa("F a", frozenset(), labels, deadline=time.monotonic() - 1.0)
    big = " & ".join(f"(a{i} | b{i})" for i in range(14))  # 2**14 clauses in DNF
    with pytest.raises(ValueError, match="式が大きすぎます"):
        build_dfa(big, frozenset(), labels)
    with pytest.raises(ValueError, match="式が大きすぎます"):
        build_dfa("(a | b) & (c | d)", frozenset(), labels, max_clauses=3)
    states, _, _ = build_dfa("(a | b) & (c | d)", frozenset(), labels, max_clauses=4)
    assert states
