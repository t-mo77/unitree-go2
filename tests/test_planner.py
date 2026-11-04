import time

import pytest
from test_automata import grid_world

from voice_ltl.automata import synthesize
from voice_ltl.planner import PlanError, Runtime
from voice_ltl.world import LYING, STANDING, PlantState, event_action, go2_world


def grid_action(event: str) -> str:
    return event.split("_to_")[0]


def go2_sup(tmp_path, formula, init):
    return synthesize(formula, go2_world(init), tmp_path).sup


def test_lecture_plan_and_reroute(tmp_path):
    rt = Runtime(synthesize("F at_goal & G !collide", grid_world(), tmp_path).sup)
    assert [a for a in map(grid_action, rt.plan())] == ["up", "right", "right", "down"]
    rt.apply(rt.next_event())  # up -> (0,2)
    blocked = frozenset({(rt.state, "right_to_free")})  # the next cell (1,2) holds the bag
    assert len(rt.plan(avoid=blocked)) == 5


@pytest.mark.parametrize("formula,actions", [
    ("F hello", ["stand", "hello"]),
    ("F(hello & F danced)", ["stand", "hello", "dance"]),
    ("(!sitting U hello) & F sitting", ["stand", "hello", "sit"]),
    ("G !danced & F stretched", ["stand", "stretch"]),
    ("F(hello & X F(hello & X F hello))", ["stand", "hello", "hello", "hello"]),
    ("F(moved_forward & X F moved_forward)", ["stand", "forward", "forward"]),
])
def test_go2_plans(tmp_path, formula, actions):
    rt = Runtime(go2_sup(tmp_path, formula, PlantState(LYING, False, False)))
    assert [event_action(e) for e in rt.plan()] == actions


def test_three_step_instruction_fast(tmp_path):
    t0 = time.monotonic()
    sup = go2_sup(tmp_path, "F(hello & F(danced & F sitting))", PlantState(LYING, False, False))
    assert time.monotonic() - t0 < 5.0
    assert [event_action(e) for e in Runtime(sup).plan()] == ["stand", "hello", "dance", "sit"]


def test_waits_when_front_blocked_then_resumes(tmp_path):
    rt = Runtime(go2_sup(tmp_path, "F moved_forward", PlantState(LYING, True, False)))
    assert rt.next_event() is None
    rt.apply("front_clear")
    assert [event_action(e) for e in rt.plan()] == ["stand", "forward"]


def test_forward_never_allowed_while_blocked(tmp_path):
    rt = Runtime(go2_sup(tmp_path, "F moved_forward", PlantState(STANDING, True, False)))
    assert not any(event_action(e) == "forward" for e in rt.allowed())


def test_apply_unknown_event_raises(tmp_path):
    rt = Runtime(go2_sup(tmp_path, "F hello", PlantState(LYING, False, False)))
    with pytest.raises(PlanError):
        rt.apply("hello_to_hello_standing")  # cannot greet while lying
