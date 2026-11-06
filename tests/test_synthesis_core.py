"""Synthesis core (Task A): supcon on PLANT and SPEC2, size guards, SUP state pairs,
Runtime.jump/walk and the diagram cap (design §6.1, §6.6; Rulings R9, R10, R14)."""

import time
from collections import deque
from pathlib import Path

import graphviz
import pitct
import pytest
from go2_nl_control.actions import LYING

from voice_ltl import automata
from voice_ltl.automata import Automaton, SpecError, World, synthesize
from voice_ltl.diagrams import MAX_DRAW_STATES, render_svg, too_big_svg
from voice_ltl.planner import PlanError, Runtime
from voice_ltl.world import PlantState, go2_world

# The lecture's 3 x 3 room: the one fixture in tests/test_automata.py, imported (not copied) so the
# pinned numbers here and there always test the same world. The alias keeps it apart from grid.py's world.
from test_automata import GOAL, OBSTACLE, START, grid_world as lecture_world


def canonical(size: int, trans, marked) -> tuple[int, list, list]:
    """A deterministic automaton renumbered by a BFS from 0 (events in sorted order), so two
    isomorphic automata compare equal however pitct numbered them."""
    if size == 0:
        return (0, [], [])
    out: dict[int, dict[str, int]] = {}
    for s, e, d in trans:
        out.setdefault(s, {})[e] = d
    num = {0: 0}
    queue = deque([0])
    while queue:
        s = queue.popleft()
        for e in sorted(out.get(s, {})):
            d = out[s][e]
            if d not in num:
                num[d] = len(num)
                queue.append(d)
    return (size, sorted((num[s], e, num[d]) for s, e, d in trans), sorted(num[m] for m in marked))


def old_supcon(formula: str, world: World, tmp: Path) -> tuple[int, list, list]:
    """Today's pipeline, run directly with pitct: SPEC2 = SPEC ∧ SAFE, PRODUCT = PLANT ∧ SPEC2,
    TRIMMED = trim(PRODUCT), SUP = supcon(PRODUCT, TRIMMED)."""
    spec = automata.spec_from_formula(formula, world)
    plant = automata.plant_from_world(world)
    tmp.mkdir(parents=True, exist_ok=True)
    with automata._PITCT_LOCK:
        pitct.init(Path(tmp).resolve(), overwrite=True)
        automata._create(plant)
        automata._create(spec)
        automata._create(automata.safe_from_world(world))
        pitct.meet("SPEC2", "SPEC", "SAFE")
        pitct.meet("PRODUCT", "PLANT", "SPEC2")
        pitct.trim("TRIMMED", "PRODUCT")
        pitct.supcon("SUP", "PRODUCT", "TRIMMED")
        trans = [(s, e, d) for s, e, d, *_ in pitct.trans("SUP")]
        return canonical(pitct.statenum("SUP"), trans, pitct.marker("SUP"))


def test_lecture_world_is_the_test_automata_fixture():
    """One source of truth: a copy here could drift from test_automata.grid_world."""
    import test_automata

    assert lecture_world is test_automata.grid_world
    w = lecture_world()
    assert (w.initial, w.marked, len(w.states)) == (START, [GOAL], 9) and OBSTACLE in w.states


def test_sup_pairs_map_every_sup_state(tmp_path):
    s = synthesize("F at_goal & G !collide", lecture_world(), tmp_path)
    assert len(s.sup_pairs) == s.sup.size and s.world is not None
    assert s.plant_state(0) == START and s.spec_state(0) == 0
    nxt = {(src, ev): dst for src, ev, dst, _ in s.world.transitions}
    for a, ev, b in s.sup.trans:
        assert s.plant_state(b) == nxt[(s.plant_state(a), ev)]
        assert s.spec_state(b) == s.spec_after(s.spec_state(a), ev)
    assert all(s.sup_state_for(s.plant_state(k), s.spec_state(k)) == k for k in range(s.sup.size))
    assert s.sup_state_for(OBSTACLE, 0) is None and s.spec_after(0, "no_such_event") is None
    assert s.sup_state_for((9, 9), 0) is None              # a state not in the World: None, never KeyError


def test_sup_pairs_on_the_go2_world_use_spec2(tmp_path):
    """With forbidden events, spec is SPEC2; the pairs follow its numbering, sensor events included."""
    w = go2_world(PlantState(LYING, False, False))
    s = synthesize("F(hello & X F hello)", w, tmp_path)
    assert s.spec.name == "SPEC2" and len(s.sup_pairs) == s.sup.size
    nxt = {(src, ev): dst for src, ev, dst, _ in w.transitions}
    for a, ev, b in s.sup.trans:
        assert s.plant_state(b) == nxt[(s.plant_state(a), ev)]
        assert s.spec_state(b) == s.spec_after(s.spec_state(a), ev)
    assert len(set(s.sup_pairs)) == s.sup.size


def test_empty_sup_has_no_pairs(tmp_path):
    s = synthesize("F sitting & G !sitting", go2_world(PlantState(LYING, False, False)), tmp_path)
    assert s.empty and s.sup_pairs == [] and s.world is not None
    assert s.sup_state_for(s.world.initial, 0) is None


# a bug path only, in the usual form and without internal state numbers
CANNOT_BUILD = r"^監督器を作れませんでした（[^0-9（）]+）$"


def test_pairs_refuse_a_sup_outside_plant_x_spec():
    plant = Automaton("PLANT", 3, [(0, "a", 1), (0, "b", 2)], {1, 2}, set())
    spec = Automaton("SPEC", 1, [(0, "a", 0), (0, "b", 0)], {0}, set())
    good = Automaton("SUP", 3, [(0, "a", 1), (0, "b", 2)], {1, 2}, set())
    assert automata._sup_pairs(good, plant, spec) == [(0, 0), (1, 0), (2, 0)]
    for bad in ([(0, "a", 1), (0, "b", 1)],   # one SUP state, two plant states
                [(0, "a", 1), (1, "c", 2)],   # an event the plant does not have there
                [(0, "a", 1)]):               # state 2 cannot be reached
        with pytest.raises(SpecError, match=CANNOT_BUILD):
            automata._sup_pairs(Automaton("SUP", 3, bad, {1}, set()), plant, spec)


def test_pairs_refuse_two_sup_states_on_one_pair():   # R10: sup_state_for can never return the wrong state
    plant = Automaton("PLANT", 2, [(0, "a", 1), (0, "b", 1)], {1}, set())
    spec = Automaton("SPEC", 1, [(0, "a", 0), (0, "b", 0)], {0}, set())
    sup = Automaton("SUP", 3, [(0, "a", 1), (0, "b", 2)], {1, 2}, set())   # SUP 1 and 2 are both (1, 0)
    with pytest.raises(SpecError, match=CANNOT_BUILD):
        automata._sup_pairs(sup, plant, spec)
    one = Automaton("SUP", 2, [(0, "a", 1), (0, "b", 1)], {1}, set())       # the same moves on one state: fine
    assert automata._sup_pairs(one, plant, spec) == [(0, 0), (1, 0)]


@pytest.mark.parametrize("formula", ["F hello", "F(hello & X F danced) & G !sitting",
                                     "F(hello & X F(hello & X F hello))", "F sitting & G !sitting"])
def test_sup_equals_the_old_product_supcon(tmp_path, formula):
    w = go2_world(PlantState(LYING, False, False))
    new = synthesize(formula, w, tmp_path / "new").sup
    assert canonical(new.size, new.trans, new.marked) == old_supcon(formula, w, tmp_path / "old")


def test_product_and_trimmed_only_when_small(tmp_path, monkeypatch):
    w = go2_world(PlantState(LYING, False, False))
    s = synthesize("F hello", w, tmp_path)
    assert s.product is not None and s.trimmed is not None
    monkeypatch.setattr(automata, "PRODUCT_DRAW_LIMIT", 10)
    s = synthesize("F hello", w, tmp_path)
    assert s.product is None and s.trimmed is None and not s.empty


def test_big_estimate_is_refused_before_pitct(tmp_path, monkeypatch):
    called = []
    monkeypatch.setattr(automata, "supcon", lambda *a: called.append(a))
    monkeypatch.setattr(automata, "MAX_PRODUCT_ESTIMATE", 10)
    with pytest.raises(SpecError, match="式が大きすぎます"):
        synthesize("F hello", go2_world(PlantState(LYING, False, False)), tmp_path)
    assert called == []


def test_size_limits():                                # plan Global Constraints and Ruling R9
    assert automata.MAX_SUP_STATES == 50_000 and automata.MAX_PRODUCT_ESTIMATE == 400_000
    assert automata.PRODUCT_DRAW_LIMIT == 5_000


def test_sup_over_the_cap_is_refused_before_it_is_read(tmp_path, monkeypatch):
    read, real = [], automata._read
    monkeypatch.setattr(automata, "_read", lambda name, u: read.append(name) or real(name, u))
    monkeypatch.setattr(automata, "MAX_SUP_STATES", 2)
    with pytest.raises(SpecError, match="式が大きすぎます"):
        synthesize("F hello", go2_world(PlantState(LYING, False, False)), tmp_path)
    assert "SUP" not in read


def test_time_is_checked_after_supcon(tmp_path, monkeypatch):
    w = go2_world(PlantState(LYING, False, False))
    synthesize("F hello", w, tmp_path / "warm")  # the first parse of a formula is slow; not timed here
    real, ran = automata.supcon, []
    monkeypatch.setattr(automata, "supcon", lambda *a: (ran.append(a), time.sleep(0.5), real(*a))[2])
    with pytest.raises(SpecError, match="式が大きすぎます"):
        synthesize("F hello", w, tmp_path, timeout_s=0.3)
    assert ran                    # refused by the check after supcon, not by an earlier one


def test_fast_read_matches_pitct(tmp_path):
    with automata._PITCT_LOCK:
        pitct.init(Path(tmp_path).resolve(), overwrite=True)
        plant = automata.plant_from_world(lecture_world())
        automata._create(plant)
        got = automata._read("PLANT", plant.uncontrollable)
        assert sorted(got.trans) == sorted((s, e, d) for s, e, d, *_ in pitct.trans("PLANT"))
        assert got.marked == set(pitct.marker("PLANT")) and got.size == pitct.statenum("PLANT") == 9


def test_jump_and_walk():
    rt = Runtime(Automaton("SUP", 3, [(0, "a", 1), (1, "b", 2)], {2}, set()))
    assert rt.walk(["a", "b"]) == [1, 2] and rt.state == 0
    with pytest.raises(PlanError):
        rt.walk(["b"])
    rt.jump(2)
    assert rt.state == 2 and rt.done
    for bad in (-1, 3):
        with pytest.raises(ValueError):
            rt.jump(bad)


def test_big_automaton_is_a_placeholder_and_dot_is_not_called(monkeypatch):
    monkeypatch.setattr(graphviz.Digraph, "pipe", lambda *a, **k: pytest.fail("dot was called"))
    a = Automaton("SUP", 201, [(i, "hello_to_hello", i + 1) for i in range(200)], {200}, set())
    svg = render_svg(a, "SUP")
    assert "大きすぎるので図は省きました（201 状態）。" in svg and "上から見た図" not in svg
    assert 'class="too-big"' in svg and 'class="graph"' not in svg
    assert "（9000 状態）。上から見た図を見てください" in too_big_svg("PLANT", 9000, map_hint=True)
    assert "大きすぎるので図は省きました。" in too_big_svg("TRIMMED") and MAX_DRAW_STATES == 200


def test_placeholder_texts_and_the_cap_edge():
    assert "大きすぎるので図は省きました（9000 状態）。<" in too_big_svg("P", 9000)
    assert "大きすぎるので図は省きました。上から見た図を見てください" in too_big_svg("T", None, True)
    assert "<title>A &lt;b&gt;</title>" in too_big_svg("A <b>")
    at_cap = Automaton("SUP", 200, [(i, "hello_to_hello", i + 1) for i in range(199)], {199}, set())
    assert 'class="too-big"' not in render_svg(at_cap, "SUP")   # 200 states are still drawn
