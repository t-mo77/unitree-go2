import time
from concurrent.futures import ThreadPoolExecutor

import pytest
from go2_nl_control.actions import LYING

from voice_ltl import automata
from voice_ltl.automata import SpecError, World, synthesize
from voice_ltl.ltlf import holds
from voice_ltl.world import MAX_REPEAT, SAFETY_FORMULA, PlantState, event_action, go2_world

# The lecture's 3 x 3 room (docs/reference/nl2fl_grid.ipynb cells 3, 5, 19, 21)
WIDTH, HEIGHT = 3, 3
GOAL, OBSTACLE, START = (2, 1), (1, 1), (0, 1)
MOVES = {"up": (0, 1), "down": (0, -1), "left": (-1, 0), "right": (1, 0)}


def grid_label(cell) -> frozenset[str]:
    if cell == GOAL:
        return frozenset({"at_goal"})
    if cell == OBSTACLE:
        return frozenset({"collide"})
    return frozenset()


def grid_world() -> World:
    """The lecture room as a World: START is state 0, the goal is absorbing and the only marked cell."""
    cells = [START] + [(x, y) for x in range(WIDTH) for y in range(HEIGHT) if (x, y) != START]
    transitions = []
    event_label = {}
    for x, y in cells:
        if (x, y) == GOAL:
            continue
        for action, (dx, dy) in MOVES.items():
            nxt = (x + dx, y + dy)
            if 0 <= nxt[0] < WIDTH and 0 <= nxt[1] < HEIGHT:
                label = grid_label(nxt)
                event = f"{action}_to_" + ("_".join(sorted(label)) or "free")
                event_label[event] = label
                transitions.append(((x, y), event, nxt, "c"))
    return World(
        states=cells,
        initial=START,
        transitions=transitions,
        marked=[GOAL],
        event_label=event_label,
        initial_label=grid_label(START),
        propositions=("at_goal", "collide"),
    )


def test_lecture_grid_numbers(tmp_path):
    s = synthesize("F at_goal & G !collide", grid_world(), tmp_path)
    assert (s.plant.size, len(s.plant.trans)) == (9, 21)
    assert (s.spec.size, len(s.spec.trans)) == (3, 30)
    assert (s.product.size, len(s.product.trans)) == (17, 38)
    assert (s.trimmed.size, len(s.trimmed.trans)) == (8, 14)
    assert (s.sup.size, len(s.sup.trans)) == (8, 14)


def test_go2_unsatisfiable_is_empty(tmp_path):
    w = go2_world(PlantState(LYING, False, False))
    assert synthesize("F sitting & G !sitting", w, tmp_path).empty


def test_go2_uncontrollable_kept(tmp_path):
    w = go2_world(PlantState(LYING, False, False))
    s = synthesize("F hello", w, tmp_path)
    assert {"front_blocked", "front_clear"} <= {e for _, e, _ in s.sup.trans}
    assert not s.empty


def test_forbidden_is_the_safety_formula():
    """An event is forbidden exactly when its label breaks SAFETY_FORMULA."""
    w = go2_world(PlantState(LYING, False, False))
    assert w.forbidden
    for event, label in w.event_label.items():
        assert (event in w.forbidden) == (not holds(SAFETY_FORMULA, [label])), event


@pytest.mark.parametrize("formula", ["F moved_forward", "F moved_backward"])
def test_safety_events_never_in_sup(tmp_path, formula):
    w = go2_world(PlantState(LYING, False, False))
    s = synthesize(formula, w, tmp_path)
    assert not s.empty
    sup_events = {e for _, e, _ in s.sup.trans}
    assert not sup_events & w.forbidden
    assert not {e for _, e, _ in s.spec.trans} & w.forbidden
    moves = {e for e in w.event_label if e not in w.forbidden and event_action(e) in ("forward", "backward")}
    assert moves & sup_events  # the safe moves are still allowed


def repeated(prop: str, n: int) -> str:
    """n times `prop`, in order: repeated("hello", 3) == "F(hello & X F(hello & X F hello))"."""
    f = f"F {prop}"
    for _ in range(n - 1):
        f = f"F({prop} & X {f})"
    return f


def test_max_repeats_within_budget(tmp_path):
    assert MAX_REPEAT == 6
    t0 = time.monotonic()
    s = synthesize(repeated("hello", MAX_REPEAT), go2_world(PlantState(LYING, False, False)), tmp_path)
    assert time.monotonic() - t0 < 5.0
    assert not s.empty


def test_too_many_spec_states_raises(tmp_path, monkeypatch):
    monkeypatch.setattr(automata, "MAX_SPEC_STATES", 3)
    with pytest.raises(SpecError, match="式が大きすぎます"):
        synthesize(repeated("hello", 3), go2_world(PlantState(LYING, False, False)), tmp_path)


def test_out_of_time_raises(tmp_path):
    with pytest.raises(SpecError, match="式が大きすぎます"):
        synthesize("F hello", go2_world(PlantState(LYING, False, False)), tmp_path, timeout_s=-1.0)  # already past on the first check


def test_pitct_failure_is_spec_error(tmp_path, monkeypatch):
    def broken(*args):
        raise RuntimeError("boom")

    monkeypatch.setattr(automata, "trim", broken)
    with pytest.raises(SpecError, match="boom"):
        synthesize("F hello", go2_world(PlantState(LYING, False, False)), tmp_path)


def test_bad_formula_raises(tmp_path):
    with pytest.raises(SpecError):
        synthesize("F (hello", go2_world(PlantState(LYING, False, False)), tmp_path)


def test_parallel_calls_are_serialised(tmp_path):
    """Each call gets its own (temporary) folder and pitct state, even from several threads."""
    w = go2_world(PlantState(LYING, False, False))
    formulas = ["F hello", "F danced", "F sitting & G !sitting", "F(hello & X F hello)"] * 2
    expected = [synthesize(f, w, tmp_path / "seq").sup for f in formulas]
    with ThreadPoolExecutor(max_workers=4) as pool:
        got = list(pool.map(lambda f: synthesize(f, w, tmp_path / "par").sup, formulas))
    assert got == expected
    assert list((tmp_path / "par").glob("synth-*")) == []


def test_huge_formula_is_spec_error_within_time(tmp_path):
    # The DNF of this conjunction of 20 disjunctions has 2**20 clauses.
    f = " & ".join(f"(p{i} | q{i})" for i in range(20))
    t0 = time.monotonic()
    with pytest.raises(SpecError, match="式が大きすぎます"):
        synthesize(f, go2_world(PlantState(LYING, False, False)), tmp_path, timeout_s=5.0)
    assert time.monotonic() - t0 < 6.0


@pytest.mark.parametrize("formula", ["F hello", "F (hello & G !hello)"])
def test_synth_folders_are_removed(tmp_path, formula):
    synthesize(formula, go2_world(PlantState(LYING, False, False)), tmp_path)
    assert list(tmp_path.glob("synth-*")) == []


def test_synth_folder_removed_on_failure(tmp_path, monkeypatch):
    monkeypatch.setattr(automata, "trim", lambda *a: (_ for _ in ()).throw(RuntimeError("boom")))
    with pytest.raises(SpecError):
        synthesize("F hello", go2_world(PlantState(LYING, False, False)), tmp_path)
    assert list(tmp_path.glob("synth-*")) == []
