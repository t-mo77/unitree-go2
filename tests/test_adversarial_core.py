"""Adversarial checks of the core guarantees (Task C1).

Many seeded random formulas over the Go2 vocabulary, on all 12 initial plant states
(posture x front x back), check that the supervisor is safe, controllable and
nonblocking, that Runtime.plan() reaches a marked state by a shortest controllable
path whose labels satisfy the formula, that build_dfa means what ltlf.holds means, and
that unsatisfiable formulas give an empty supervisor.

Every oracle here comes from a definition (walking the plant, breadth first search,
ltlf.holds, the supcon fixpoint), never from the code it checks.
"""

import random
from collections import deque
from dataclasses import dataclass

import pytest
from go2_nl_control.actions import LYING, SITTING, STANDING

from voice_ltl.automata import Automaton, SpecError, Synthesis, World, synthesize
from voice_ltl.ltlf import build_dfa, holds
from voice_ltl.planner import PlanError, Runtime
from voice_ltl.world import ACTION_PROP, PROPOSITIONS, PlantState, event_action, go2_label, go2_world

SEED = 20261104
INITIAL_STATES = tuple(
    PlantState(posture, front, back)
    for posture in (LYING, STANDING, SITTING) for front in (False, True) for back in (False, True)
)

UNARY = ("!", "F", "G", "X")
BINARY = ("&", "|", "->", "U")
LEAVES = PROPOSITIONS + ("last",)


def random_formula(rng: random.Random, depth: int, full: bool = True) -> str:
    """A formula over the Go2 propositions and last. With full, its depth is exactly `depth`."""
    if depth == 0 or (not full and rng.random() < 0.25):
        return rng.choice(LEAVES)
    if rng.random() < 0.5:
        return f"{rng.choice(UNARY)}({random_formula(rng, depth - 1, full)})"
    left = random_formula(rng, depth - 1, full)
    right = random_formula(rng, depth - 1, False)
    if rng.random() < 0.5:
        left, right = right, left
    return f"({left}) {rng.choice(BINARY)} ({right})"


def random_formulas(seed: int, n: int) -> list[str]:
    rng = random.Random(seed)
    return [random_formula(rng, rng.choice((3, 4))) for _ in range(n)]


ACTION_PROPS = tuple(p for p in ACTION_PROP.values() if p)


def goal_formulas(seed: int, n: int) -> list[str]:
    """F goal & a random depth-3 formula: fewer cases that are done before they start."""
    rng = random.Random(seed)
    return [f"F({rng.choice(ACTION_PROPS)}) & ({random_formula(rng, 3)})" for _ in range(n)]


# Hand-made formulas aimed at the edges: sensors inside the formula, moves the safety
# rules forbid in some states, last, X chains, and goals only the environment can bring.
EDGE_FORMULAS = [
    "F moved_forward",
    "F moved_backward",
    "F(moved_forward & X F moved_backward)",
    "F(moved_forward & X F(moved_forward & X F(moved_forward & X F moved_forward)))",
    "G(obstacle_front -> F hello)",
    "F(moved_forward & obstacle_back)",
    "F(moved_backward & obstacle_front)",
    "F obstacle_front",
    "G !obstacle_front & F moved_forward",
    "(!moved_forward U obstacle_front) & F moved_forward",
    "G(moved_forward -> X moved_backward) & F moved_forward",
    "G(hello -> X danced) & F hello",
    "G F hello",
    "F G sitting",
    "last",
    "!last & G !hello",
    "X X X hello",
    "F(lying & X standing & X X sitting)",
    "(!sitting U hello) & F sitting",
    "F(hello & X F(hello & X F(hello & X F(hello & X F(hello & X F hello)))))",
    "F(turned_left & X F(turned_left & X F turned_left)) & G !moved_forward",
    "(F hello) U (F danced)",
    "G(obstacle_back -> !X moved_backward) & F(moved_backward & X F moved_forward)",
    # a longer route whose first event sorts first (or last) next to a shorter one
    "F(moved_backward & X F(moved_backward & X F moved_backward)) | F(hello & X F hello)",
    "F(turned_right & X F(turned_right & X F turned_right)) | F(content & X F content)",
    "F(sitting & X F(standing & X F(sitting & X F hello))) | F(danced & X F(danced & X F danced))",
]

# Unsatisfiable on every initial state: the supervisor must be empty.
UNSATISFIABLE = [
    "F sitting & G !sitting",
    "F hello & G !hello",
    "X X X X X X X X X X hello & G !standing",
    "F hello & G !standing",
    "last & F hello",
    "G lying & F standing",
    "F(turned_left & turned_right)",
    "F(lying & X sitting)",
    "F(sitting & X hello)",
    "F(moved_forward & obstacle_front)",
    "F(moved_backward & obstacle_back)",
    "F hello & G(hello -> X false)",
    "false",
]

SYNTH_FORMULAS = EDGE_FORMULAS + random_formulas(SEED, 36) + goal_formulas(SEED + 1, 12)


@dataclass
class Case:
    formula: str
    initial: PlantState
    world: World
    syn: Synthesis | None  # None when synthesis refused the formula
    error: str = ""

    @property
    def sup(self) -> Automaton:
        return self.syn.sup


@pytest.fixture(scope="module")
def cases(tmp_path_factory) -> list[Case]:
    workdir = tmp_path_factory.mktemp("adversarial")
    out = []
    for formula in SYNTH_FORMULAS:
        for initial in INITIAL_STATES:
            world = go2_world(initial)
            try:
                out.append(Case(formula, initial, world, synthesize(formula, world, workdir)))
            except SpecError as exc:
                out.append(Case(formula, initial, world, None, str(exc)))
    return out


def nonempty(cases: list[Case]) -> list[Case]:
    return [c for c in cases if c.syn is not None and not c.syn.empty]


def report(failures: list[tuple]) -> str:
    """The shortest formulas first, so the first line is the smallest repro."""
    failures = sorted(failures, key=lambda f: (len(f[0]), str(f)))
    return "\n".join(map(str, failures[:5])) + f"\n({len(failures)} failures in all)"


# ---- graph helpers (plain breadth first search, nothing from voice_ltl.planner) ----

def edges(a: Automaton) -> dict[int, dict[str, int]]:
    """state -> event -> next state. Fails if the automaton is not deterministic."""
    out: dict[int, dict[str, int]] = {}
    for src, ev, dst in a.trans:
        assert 0 <= src < a.size and 0 <= dst < a.size, f"{a.name}: state out of range {(src, ev, dst)}"
        assert out.setdefault(src, {}).setdefault(ev, dst) == dst, f"{a.name}: two targets for {ev} at {src}"
    return out


def plant_edges(world: World) -> dict[PlantState, dict[str, tuple[PlantState, str]]]:
    out: dict = {}
    for src, ev, dst, kind in world.transitions:
        out.setdefault(src, {})[ev] = (dst, kind)
    return out


def event_kinds(world: World) -> dict[str, str]:
    return {ev: kind for _, ev, _, kind in world.transitions}


def reachable(a: Automaton) -> dict[int, list[str]]:
    """Every state reachable from 0, with one event path from 0 to it."""
    if a.size == 0:
        return {}
    out = edges(a)
    path = {0: []}
    queue = deque([0])
    while queue:
        s = queue.popleft()
        for ev, d in sorted(out.get(s, {}).items()):
            if d not in path:
                path[d] = path[s] + [ev]
                queue.append(d)
    return path


def coreachable(a: Automaton) -> set[int]:
    """States from which some marked state can be reached (any events)."""
    into: dict[int, set[int]] = {}
    for src, _, dst in a.trans:
        into.setdefault(dst, set()).add(src)
    seen = set(a.marked)
    queue = deque(seen)
    while queue:
        for s in into.get(queue.popleft(), ()):
            if s not in seen:
                seen.add(s)
                queue.append(s)
    return seen


def pairs(sup: Automaton, world: World) -> set[tuple[int, PlantState]]:
    """(SUP state, plant state) pairs reached by running the same events on both from the start."""
    if sup.size == 0:
        return set()
    sout, pout = edges(sup), plant_edges(world)
    start = (0, world.initial)
    seen = {start}
    queue = deque([start])
    while queue:
        s, p = queue.popleft()
        for ev, d in sout.get(s, {}).items():
            if ev in pout[p]:
                nxt = (d, pout[p][ev][0])
                if nxt not in seen:
                    seen.add(nxt)
                    queue.append(nxt)
    return seen


def shortest_controllable(sup: Automaton, kinds: dict[str, str], start: int) -> int | None:
    """Length of a shortest controllable-only path from start to a marked state, or None."""
    out = edges(sup)
    dist = {start: 0}
    queue = deque([start])
    while queue:
        s = queue.popleft()
        if s in sup.marked:
            return dist[s]
        for ev, d in out.get(s, {}).items():
            if kinds[ev] == "c" and d not in dist:
                dist[d] = dist[s] + 1
                queue.append(d)
    return None


def trace_of(world: World, events: list[str]) -> list[frozenset[str]]:
    """The LTLf trace of an event list: the initial label, then one label per controllable event."""
    kinds = event_kinds(world)
    return [world.initial_label] + [world.event_label[ev] for ev in events if kinds[ev] == "c"]


def supcon_by_definition(formula: str, world: World):
    """The supremal controllable nonblocking supervisor, from its definition (not pitct).

    States are (plant state, DFA state) pairs; a controllable event moves the DFA by its
    label, an uncontrollable one leaves it; forbidden events are removed (SAFE). Bad states
    (cannot reach a marked state, or an uncontrollable event leads to a bad state) are
    removed until nothing changes. Returns (start, edges, marked) on the good states
    reachable from the start, or None when the start is bad.
    """
    labels = sorted(set(world.event_label.values()), key=sorted)
    _, step, accepting = build_dfa(formula, world.initial_label, labels)
    pout = plant_edges(world)
    start = (world.initial, 0)
    out: dict = {}
    queue = deque([start])
    while queue:
        x = queue.popleft()
        if x in out:
            continue
        p, q = x
        out[x] = {}
        for ev, (dst, kind) in pout[p].items():
            if ev not in world.forbidden:
                out[x][ev] = (dst, step[(q, world.event_label[ev])] if kind == "c" else q)
                queue.append(out[x][ev])
    marked = {x for x in out if x[0] in world.marked and x[1] in accepting}
    kinds = event_kinds(world)
    good = set(out)
    while True:
        into: dict = {}
        for x in good:
            for y in out[x].values():
                if y in good:
                    into.setdefault(y, set()).add(x)
        co = marked & good
        queue = deque(co)
        while queue:
            for x in into.get(queue.popleft(), ()):
                if x not in co:
                    co.add(x)
                    queue.append(x)
        keep = {x for x in co if all(y in co for ev, y in out[x].items() if kinds[ev] == "u")}
        if keep == good:
            break
        good = keep
    if start not in good:
        return None
    sub = {x: {ev: y for ev, y in out[x].items() if y in good} for x in good}
    return start, sub, marked & good


def first_difference(sup: Automaton, ref) -> str | None:
    """Walk SUP and the reference supervisor side by side; the first place they differ."""
    if sup.size == 0 or ref is None:
        return None if (sup.size == 0) == (ref is None) else f"SUP size {sup.size}, reference empty: {ref is None}"
    start, rout, rmarked = ref
    sout = edges(sup)
    seen = {(0, start)}
    queue = deque([((0, start), [])])
    while queue:
        (s, x), path = queue.popleft()
        if (s in sup.marked) != (x in rmarked):
            return f"after {path}: SUP marked {s in sup.marked}, reference marked {x in rmarked}"
        sup_ev, ref_ev = set(sout.get(s, {})), set(rout[x])
        if sup_ev != ref_ev:
            return f"after {path}: SUP extra {sorted(sup_ev - ref_ev)}, SUP missing {sorted(ref_ev - sup_ev)}"
        for ev in sorted(sup_ev):
            nxt = (sout[s][ev], rout[x][ev])
            if nxt not in seen:
                seen.add(nxt)
                queue.append((nxt, path + [ev]))
    return None


# ---- the cases themselves are meaningful ----

def test_cases_are_not_vacuous(cases):
    """Synthesis only refuses for size, and enough cases have a supervisor and a real plan."""
    assert len(cases) == len(SYNTH_FORMULAS) * 12
    assert [c.error for c in cases if c.syn is None and "式が大きすぎます" not in c.error] == []
    full = nonempty(cases)
    assert len(full) >= len(cases) // 3
    plans = [Runtime(c.sup).plan() for c in full]
    assert sum(1 for p in plans if p and len(p) >= 2) >= len(full) // 4
    assert any(p is None for p in plans)  # some supervisors must wait for the environment
    assert any(p == [] for p in plans)    # and some are done at once


# ---- 1. safety ----

def test_sup_never_allows_a_forbidden_or_unsafe_move(cases):
    failures = []
    for c in nonempty(cases):
        hit = {e for _, e, _ in c.sup.trans} & c.world.forbidden
        if hit:
            failures.append((c.formula, c.initial, f"forbidden events in SUP: {sorted(hit)}"))
        out = edges(c.sup)
        for s, p in pairs(c.sup, c.world):
            for ev in out.get(s, {}):
                action = event_action(ev)
                if (action == "forward" and p.front) or (action == "backward" and p.back):
                    failures.append((c.formula, c.initial, f"{ev} allowed in plant state {p}"))
    assert not failures, report(failures)


# ---- 2. controllability (and SUP only uses what the plant can do) ----

def test_sup_never_disables_a_sensor_event(cases):
    failures = []
    for c in nonempty(cases):
        out, pout = edges(c.sup), plant_edges(c.world)
        for s, p in pairs(c.sup, c.world):
            sup_ev = set(out.get(s, {}))
            sensor = {ev for ev, (_, kind) in pout[p].items() if kind == "u"}
            assert len(sensor) == 2
            if sensor - sup_ev:
                failures.append((c.formula, c.initial, f"SUP {s} / plant {p} drops {sorted(sensor - sup_ev)}"))
            if sup_ev - set(pout[p]):
                failures.append((c.formula, c.initial, f"SUP {s} / plant {p} allows {sorted(sup_ev - set(pout[p]))}"))
    assert not failures, report(failures)


# ---- 3. nonblocking ----

def test_every_reachable_sup_state_can_finish(cases):
    failures = []
    for c in nonempty(cases):
        stuck = set(reachable(c.sup)) - coreachable(c.sup)
        if stuck:
            failures.append((c.formula, c.initial, f"no way to a marked state from {sorted(stuck)[:5]}"))
    assert not failures, report(failures)


# ---- 4. plans reach a marked state and satisfy the formula ----

def test_plan_reaches_marked_and_satisfies_the_formula(cases):
    """From every reachable SUP state (reached by one event path), plan() runs to a marked
    state, uses only controllable events, and the whole trace satisfies the formula."""
    failures = []
    for c in nonempty(cases):
        kinds = event_kinds(c.world)
        for state, prefix in reachable(c.sup).items():
            rt = Runtime(c.sup)
            rt.state = state
            plan = rt.plan()
            if plan is None:
                continue
            if any(kinds[ev] != "c" for ev in plan):
                failures.append((c.formula, c.initial, f"uncontrollable event in plan {plan}"))
                continue
            run = Runtime(c.sup)
            try:
                for ev in prefix + plan:
                    run.apply(ev)
            except PlanError as exc:  # the plan leaves the supervisor
                failures.append((c.formula, c.initial, f"{prefix} + {plan}: {exc}"))
                continue
            if not run.done:
                failures.append((c.formula, c.initial, f"{prefix} + {plan} ends unmarked"))
            elif not holds(c.formula, trace_of(c.world, prefix + plan)):
                failures.append((c.formula, c.initial, f"{prefix} + {plan} breaks the formula"))
    assert not failures, report(failures)


def test_plan_from_the_start_satisfies_the_formula(cases):
    """The case the brief names: plan() from SUP's initial state, labels into ltlf.holds."""
    failures = []
    for c in nonempty(cases):
        plan = Runtime(c.sup).plan()
        if plan is not None and not holds(c.formula, trace_of(c.world, plan)):
            failures.append((c.formula, c.initial, plan))
    assert not failures, report(failures)


# ---- 5. plans are shortest ----

def test_plan_length_is_the_shortest_controllable_path(cases):
    failures = []
    for c in nonempty(cases):
        kinds = event_kinds(c.world)
        for state in reachable(c.sup):
            rt = Runtime(c.sup)
            rt.state = state
            plan = rt.plan()
            got = None if plan is None else len(plan)
            want = shortest_controllable(c.sup, kinds, state)
            if got != want:
                failures.append((c.formula, c.initial, f"state {state}: plan {plan}, shortest {want}"))
    assert not failures, report(failures)


# ---- supcon is the maximally permissive supervisor (design 7.5) ----

def test_sup_equals_the_supcon_definition(cases):
    """SUP allows exactly what the textbook fixpoint allows, state by state, and is empty
    exactly when the fixpoint is. This checks SPEC x SAFE x PLANT, trim and supcon in pitct."""
    failures = []
    for c in cases:
        if c.syn is None:
            continue
        diff = first_difference(c.sup, supcon_by_definition(c.formula, c.world))
        if diff:
            failures.append((c.formula, c.initial, diff))
    assert not failures, report(failures)


# ---- 6. build_dfa means what holds means, on the real Go2 labels ----

def plant_walk_trace(rng: random.Random, n: int) -> list[frozenset[str]]:
    """A trace the plant can really produce: n labels, sensor events mixed in at random."""
    initial = rng.choice(INITIAL_STATES)
    world = go2_world(initial)
    pout = plant_edges(world)
    trace = [world.initial_label]
    p = initial
    while len(trace) < n:
        ev, (dst, kind) = rng.choice(sorted(pout[p].items()))
        if kind == "c":
            trace.append(world.event_label[ev])
        p = dst
    return trace


def test_build_dfa_agrees_with_holds_on_go2_labels():
    world = go2_world(INITIAL_STATES[0])
    event_labels = sorted(set(world.event_label.values()), key=sorted)
    first_labels = sorted(set(event_labels) | {go2_label(s, None) for s in INITIAL_STATES}, key=sorted)
    rng = random.Random(SEED + 6)
    formulas = EDGE_FORMULAS + UNSATISFIABLE + random_formulas(SEED + 6, 160)
    failures, checked = [], 0
    for formula in formulas:
        dfas = {}
        for k in range(12):
            n = rng.randint(1, 8)
            if k % 2:
                trace = plant_walk_trace(rng, n)
            else:
                trace = [rng.choice(first_labels)] + [rng.choice(event_labels) for _ in range(n - 1)]
            if trace[0] not in dfas:
                try:
                    dfas[trace[0]] = build_dfa(formula, trace[0], event_labels)
                except ValueError as exc:  # too many states: refused, not wrong
                    assert "式が大きすぎます" in str(exc)
                    dfas[trace[0]] = None
            if dfas[trace[0]] is None:
                continue
            _, step, accepting = dfas[trace[0]]
            q = 0
            for lab in trace[1:]:
                q = step[(q, lab)]
            checked += 1
            if (q in accepting) != holds(formula, trace):
                failures.append((formula, [sorted(lab) for lab in trace], f"dfa {q in accepting}"))
    assert checked >= len(formulas) * 10
    assert not failures, report(failures)


# ---- 7. unsatisfiable formulas give an empty supervisor ----

@pytest.mark.parametrize("formula", UNSATISFIABLE)
def test_unsatisfiable_formula_gives_an_empty_supervisor(tmp_path, formula):
    for initial in INITIAL_STATES:
        world = go2_world(initial)
        assert supcon_by_definition(formula, world) is None, (formula, initial)  # the oracle agrees
        syn = synthesize(formula, world, tmp_path)
        assert syn.empty and syn.sup.size == 0 and syn.sup.trans == [], (formula, initial)
        rt = Runtime(syn.sup)  # whoever builds the Runtime sees there is nothing to do
        assert (rt.done, rt.allowed(), rt.plan(), rt.next_event()) == (False, [], None, None), (formula, initial)
