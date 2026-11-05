"""Shared types for the plant (World) and the automata built from LTLf formulas,
and the synthesis: formula -> SPEC (meet SAFE) -> supcon with the plant (design §6.1).
PRODUCT and TRIMMED (as in nl2fl_grid.ipynb) are still built when they are small, for the page."""

import shutil
import tempfile
import threading
import time
from collections import defaultdict, deque
from dataclasses import dataclass, field
from functools import cached_property
from pathlib import Path
from typing import Any

import pitct
from pitct import create, meet, statenum, supcon, trim
from pitct.name_converter import NameConverter

from voice_ltl.ltlf import TOO_BIG, build_dfa

_PITCT_LOCK = threading.Lock()


@dataclass
class World:
    """Plant model: states, labelled transitions, and how events are labelled.

    states[0] is always `initial`. transitions are (src, event, dst, kind) with
    kind "c" (controllable) or "u" (uncontrollable). event_label holds the label
    of each controllable event (the propositions true after it happens).
    forbidden holds controllable events the supervisor must never allow (the safety rules).
    """

    states: list
    initial: Any
    transitions: list  # list[tuple[src, event: str, dst, kind: "c" | "u"]]
    marked: list
    event_label: dict  # dict[str, frozenset[str]]
    initial_label: frozenset
    propositions: tuple
    forbidden: frozenset[str] = frozenset()  # controllable events that must never happen


@dataclass
class Automaton:
    """A TCT automaton: states 0..size-1, state 0 is initial."""

    name: str
    size: int
    trans: list[tuple[int, str, int]] = field(default_factory=list)
    marked: set[int] = field(default_factory=set)
    uncontrollable: set[str] = field(default_factory=set)


@dataclass
class Synthesis:
    """The automata of one synthesis. spec is SPEC2 (SPEC meet SAFE) when the world has
    forbidden events. product and trimmed are None when they were too big to build.

    sup_pairs[k] is (plant index, spec index) of SUP state k: SUP is a sub-automaton of
    PLANT x spec, so each SUP state is one such pair and no two share one (Ruling R10).
    """

    formula: str
    plant: Automaton
    spec: Automaton
    product: Automaton | None
    trimmed: Automaton | None
    sup: Automaton
    world: World | None = None
    sup_pairs: list[tuple[int, int]] = field(default_factory=list)

    @property
    def empty(self) -> bool:
        """True when no supervisor exists, i.e. the formula cannot be satisfied."""
        return self.sup.size == 0

    def plant_state(self, sup_state: int) -> Any:
        """The World state of SUP state `sup_state`."""
        return self.world.states[self.sup_pairs[sup_state][0]]

    def spec_state(self, sup_state: int) -> int:
        """The spec (SPEC2 or SPEC) state of SUP state `sup_state`."""
        return self.sup_pairs[sup_state][1]

    def spec_after(self, spec_state: int, event: str) -> int | None:
        """The spec state after `event`, or None when the spec has no such move."""
        return self._spec_next.get((spec_state, event))

    def sup_state_for(self, plant_state: Any, spec_state: int) -> int | None:
        """The SUP state of (World state, spec state), or None when SUP has no such state."""
        p = self._plant_index.get(plant_state)
        return None if p is None else self._sup_index.get((p, spec_state))

    # The lookup tables are built once, on first use (dicts, never a scan per call).
    @cached_property
    def _plant_index(self) -> dict[Any, int]:
        return {s: i for i, s in enumerate(self.world.states)}

    @cached_property
    def _spec_next(self) -> dict[tuple[int, str], int]:
        return {(q, e): d for q, e, d in self.spec.trans}

    @cached_property
    def _sup_index(self) -> dict[tuple[int, int], int]:
        return {pair: k for k, pair in enumerate(self.sup_pairs)}


class SpecError(Exception):
    """The formula cannot be read, is too large, or synthesis fails or takes too long."""


MAX_SPEC_STATES = 2000  # a SPEC larger than this is refused as too large
MAX_SUP_STATES = 50_000          # a SUP above this is refused before it is read
MAX_PRODUCT_ESTIMATE = 400_000   # len(world.states) * spec.size above this is refused before pitct
PRODUCT_DRAW_LIMIT = 5_000       # PRODUCT and TRIMMED are built only up to this estimate


def spec_from_formula(
    formula: str, world: World, deadline: float | None = None
) -> Automaton:
    """The DFA of `formula` (by formula progression) as a TCT automaton over the events of `world`.

    State 0 is the formula after reading the initial label. A controllable event moves
    the DFA by its label; an uncontrollable event is a self-loop. world.forbidden is not
    applied here (synthesize does that with SAFE).
    """
    events = sorted({(ev, kind) for _, ev, _, kind in world.transitions})
    labels = [world.event_label[ev] for ev, kind in events if kind == "c"]
    try:
        states, step, accepting = build_dfa(formula, world.initial_label, labels, MAX_SPEC_STATES, deadline)
    except ValueError as exc:  # cannot be read, or too many states
        raise SpecError(str(exc)) from exc
    spec_trans = [
        (q, ev, step[(q, world.event_label[ev])] if kind == "c" else q)
        for q in range(len(states)) for ev, kind in events
    ]
    return Automaton(
        name="SPEC",
        size=len(states),
        trans=spec_trans,
        marked=set(accepting),
        uncontrollable={ev for ev, kind in events if kind == "u"},
    )


def safe_from_world(world: World) -> Automaton:
    """SAFE: one marked state with a self-loop on every event except world.forbidden."""
    events = {(ev, kind) for _, ev, _, kind in world.transitions}
    if any(ev in world.forbidden for ev, kind in events if kind == "u"):
        raise ValueError("forbidden events must be controllable")
    allowed = sorted(ev for ev, _ in events if ev not in world.forbidden)
    return Automaton(
        name="SAFE",
        size=1,
        trans=[(0, ev, 0) for ev in allowed],
        marked={0},
        uncontrollable={ev for ev, kind in events if kind == "u"},
    )


def plant_from_world(world: World) -> Automaton:
    sid = {s: i for i, s in enumerate(world.states)}
    return Automaton(
        name="PLANT",
        size=len(world.states),
        trans=[(sid[src], ev, sid[dst]) for src, ev, dst, _ in world.transitions],
        marked={sid[s] for s in world.marked},
        uncontrollable={ev for _, ev, _, kind in world.transitions if kind == "u"},
    )


def _create(a: Automaton) -> None:
    kinds = [(s, e, d, "u" if e in a.uncontrollable else "c") for s, e, d in a.trans]
    create(a.name, a.size, kinds, sorted(a.marked))


def _read(name: str, uncontrollable: set[str]) -> Automaton:
    """Read a pitct automaton, fast (same result as statenum, trans and marker).

    pitct.trans and pitct.marker each load the file (des_info) and decode every event by
    a scan of all event names; statenum also checks the whole automaton in TCT (1.3 s at
    40k states). So load it once, take pitct's numbers and decode them with one dict."""
    des = pitct.des_info(name)  # every state, with its marking and its moves
    events = dict(NameConverter.event_encode_dict)  # pitct's number -> our event name
    return Automaton(
        name=name,
        size=sum(1 for _ in des),
        trans=[(s, events[e], d) for s, e, d, *_ in des.trans(convert=False)],
        marked=set(des.marked(convert=False)),
        uncontrollable=set(uncontrollable),
    )


def synthesize(formula: str, world: World, workdir: Path, timeout_s: float = 20.0) -> Synthesis:
    """formula -> SPEC (meet SAFE), then supcon with the plant; product and trim only when small.

    Every failure raises SpecError: the formula cannot be read, its SPEC has more than
    MAX_SPEC_STATES states, PLANT x SPEC is estimated above MAX_PRODUCT_ESTIMATE, SUP has
    more than MAX_SUP_STATES states, more than timeout_s seconds have passed when a step
    ends (pitct steps cannot be interrupted), or pitct fails. pitct keeps its state per
    process, so calls run one at a time, each in a new folder under workdir.
    """
    with _PITCT_LOCK:
        try:
            return _synthesize(formula, world, Path(workdir), timeout_s)
        except SpecError:
            raise
        except Exception as exc:  # pitct, the file system and bad worlds raise many types
            raise SpecError(f"監督器を作れませんでした（{type(exc).__name__}: {exc}）") from exc


def _synthesize(formula: str, world: World, workdir: Path, timeout_s: float) -> Synthesis:
    deadline = time.monotonic() + timeout_s

    def check_time() -> None:
        if time.monotonic() > deadline:
            raise SpecError(TOO_BIG)

    spec = spec_from_formula(formula, world, deadline)
    check_time()
    plant = plant_from_world(world)
    workdir.mkdir(parents=True, exist_ok=True)
    # init() sets SAVE_FOLDER to a Path (a str fails) and forgets earlier event names.
    folder = Path(tempfile.mkdtemp(prefix="synth-", dir=workdir)).resolve()
    try:
        return _run_pitct(formula, world, folder, plant, spec, check_time)
    finally:
        shutil.rmtree(folder, ignore_errors=True)  # the results are already read into memory


def _run_pitct(formula, world, folder, plant, spec, check_time) -> Synthesis:
    # supcon(PRODUCT, TRIMMED) needs memory in |PRODUCT| x |TRIMMED| (7 GB at 21k states);
    # supcon(PLANT, SPEC2) gives the same SUP in little memory (design §6.1, Ruling R9).
    if len(world.states) * spec.size > MAX_PRODUCT_ESTIMATE:
        raise SpecError(TOO_BIG)
    pitct.init(folder, overwrite=True)
    _create(plant)
    _create(spec)
    if world.forbidden:
        # meet, not sync: sync lets an event that only one side has run freely, and SAFE
        # has no forbidden event at all. The same holds for SPEC2 with the plant below.
        _create(safe_from_world(world))
        meet("SPEC2", "SPEC", "SAFE")
        spec = _read("SPEC2", spec.uncontrollable)
        check_time()
    product = trimmed = None
    if len(world.states) * spec.size <= PRODUCT_DRAW_LIMIT:
        # Only for the TRIMMED tab and the lecture's numbers. SPEC has every plant event,
        # so this meet is the lecture's sync; it differs only by dropping the forbidden
        # events, which SPEC2 no longer has.
        meet("PRODUCT", "PLANT", spec.name)
        check_time()
        trim("TRIMMED", "PRODUCT")
        check_time()
        product, trimmed = (_read(n, plant.uncontrollable) for n in ("PRODUCT", "TRIMMED"))
        check_time()
    supcon("SUP", "PLANT", spec.name)
    check_time()
    if statenum("SUP") > MAX_SUP_STATES:
        raise SpecError(TOO_BIG)
    sup = _read("SUP", plant.uncontrollable)
    check_time()
    sup_pairs = _sup_pairs(sup, plant, spec)
    check_time()
    return Synthesis(formula, plant, spec, product, trimmed, sup, world, sup_pairs)


def _sup_pairs(sup: Automaton, plant: Automaton, spec: Automaton) -> list[tuple[int, int]]:
    """(plant state, spec state) of each SUP state, by running SUP's events on both from 0."""
    if sup.size == 0:
        return []
    plant_next = {(s, e): d for s, e, d in plant.trans}
    spec_next = {(s, e): d for s, e, d in spec.trans}
    out: dict[int, list[tuple[str, int]]] = defaultdict(list)
    for s, e, d in sup.trans:
        out[s].append((e, d))
    pairs: list[tuple[int, int] | None] = [None] * sup.size
    pairs[0] = (0, 0)
    queue = deque([0])
    while queue:
        k = queue.popleft()
        p, q = pairs[k]
        for e, d in out[k]:
            pair = (plant_next.get((p, e)), spec_next.get((q, e)))
            if pairs[d] is None and None not in pair:
                pairs[d] = pair
                queue.append(d)
            elif pairs[d] != pair:  # SUP is not a sub-automaton of PLANT x spec: a bug
                raise SpecError(f"監督器の状態が PLANT と要求の組になりません（{d}）")
    if None in pairs:  # supcon's result is trim, so every state is reachable from 0
        raise SpecError("監督器に初めの状態から行けない状態があります")
    return pairs
