"""Shared types for the plant (World) and the automata built from LTLf formulas,
and the synthesis: formula -> SPEC -> product -> trim -> supcon (as in nl2fl_grid.ipynb)."""

import tempfile
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import pitct
from pitct import create, marker, meet, statenum, supcon, trans, trim

from voice_ltl.ltlf import build_dfa

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
    formula: str
    plant: Automaton
    spec: Automaton
    product: Automaton
    trimmed: Automaton
    sup: Automaton

    @property
    def empty(self) -> bool:
        """True when no supervisor exists, i.e. the formula cannot be satisfied."""
        return self.sup.size == 0


class SpecError(Exception):
    """The formula cannot be read, is too large, or synthesis fails or takes too long."""


MAX_SPEC_STATES = 2000  # a SPEC larger than this is refused as too large


def spec_from_formula(formula: str, world: World) -> Automaton:
    """The DFA of `formula` (by formula progression) as a TCT automaton over the events of `world`.

    State 0 is the formula after reading the initial label. A controllable event moves
    the DFA by its label; an uncontrollable event is a self-loop. world.forbidden is not
    applied here (synthesize does that with SAFE).
    """
    events = sorted({(ev, kind) for _, ev, _, kind in world.transitions})
    labels = [world.event_label[ev] for ev, kind in events if kind == "c"]
    try:
        states, step, accepting = build_dfa(formula, world.initial_label, labels, MAX_SPEC_STATES)
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
    return Automaton(
        name=name,
        size=statenum(name),
        trans=[(s, e, d) for s, e, d, *_ in trans(name)],
        marked=set(marker(name)),
        uncontrollable=set(uncontrollable),
    )


def synthesize(formula: str, world: World, workdir: Path, timeout_s: float = 20.0) -> Synthesis:
    """formula -> SPEC (meet SAFE), then with the plant: product, trim, and supcon.

    Every failure raises SpecError: the formula cannot be read, its SPEC has more than
    MAX_SPEC_STATES states, more than timeout_s seconds have passed when a step ends
    (pitct steps cannot be interrupted), or pitct fails. pitct keeps its state per
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
    started = time.monotonic()

    def check_time() -> None:
        if time.monotonic() - started > timeout_s:
            raise SpecError("式が大きすぎます")

    spec = spec_from_formula(formula, world)
    check_time()
    plant = plant_from_world(world)
    workdir.mkdir(parents=True, exist_ok=True)
    # init() sets SAVE_FOLDER to a Path (a str fails) and forgets earlier event names.
    pitct.init(Path(tempfile.mkdtemp(prefix="synth-", dir=workdir)).resolve(), overwrite=True)
    _create(plant)
    _create(spec)
    if world.forbidden:
        # meet, not sync: sync lets an event that only one side has run freely, and SAFE
        # has no forbidden event at all. The same holds for SPEC2 with the plant below.
        _create(safe_from_world(world))
        meet("SPEC2", "SPEC", "SAFE")
        spec = _read("SPEC2", spec.uncontrollable)
    # SPEC has every plant event, so this meet is the lecture's sync; it differs only by
    # dropping the forbidden events, which SPEC2 no longer has.
    meet("PRODUCT", "PLANT", spec.name)
    check_time()
    trim("TRIMMED", "PRODUCT")
    check_time()
    supcon("SUP", "PRODUCT", "TRIMMED")
    product, trimmed, sup = (_read(n, plant.uncontrollable) for n in ("PRODUCT", "TRIMMED", "SUP"))
    return Synthesis(formula, plant, spec, product, trimmed, sup)
