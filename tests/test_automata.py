import pytest
from go2_nl_control.actions import LYING

from voice_ltl.automata import SpecError, World, synthesize
from voice_ltl.world import MAX_REPEAT, SAFETY_FORMULA, PlantState, go2_world

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
    assert synthesize(f"(F sitting & G !sitting) & {SAFETY_FORMULA}", w, tmp_path).empty


def test_go2_uncontrollable_kept(tmp_path):
    w = go2_world(PlantState(LYING, False, False))
    s = synthesize(f"(F hello) & {SAFETY_FORMULA}", w, tmp_path)
    assert {"front_blocked", "front_clear"} <= {e for _, e, _ in s.sup.trans}
    assert not s.empty
    unsafe = {e for e, label in w.event_label.items() if {"obstacle_front", "moved_forward"} <= label}
    assert unsafe and not unsafe & {e for _, e, _ in s.sup.trans}


def repeated(prop: str, n: int) -> str:
    """n times `prop`, in order: repeated("hello", 3) == "F(hello & X F(hello & X F hello))"."""
    f = f"F {prop}"
    for _ in range(n - 1):
        f = f"F({prop} & X {f})"
    return f


def test_max_repeats_within_budget(tmp_path):
    # The plan asked for 6 repeats; flloat needs > 20 s from 4 on, so MAX_REPEAT is the largest that fits.
    f = repeated("hello", MAX_REPEAT)
    s = synthesize(f"({f}) & {SAFETY_FORMULA}", go2_world(PlantState(LYING, False, False)), tmp_path)
    assert not s.empty


def test_too_large_formula_times_out(tmp_path):
    f = repeated("hello", MAX_REPEAT + 1)
    with pytest.raises(SpecError, match="式が大きすぎます"):
        synthesize(f"({f}) & {SAFETY_FORMULA}", go2_world(PlantState(LYING, False, False)), tmp_path,
                   timeout_s=1.0)


def test_bad_formula_raises(tmp_path):
    with pytest.raises(SpecError):
        synthesize("F (hello", go2_world(PlantState(LYING, False, False)), tmp_path)
