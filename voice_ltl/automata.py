"""Shared types for the plant (World) and the automata built from LTLf formulas,
and the synthesis: formula -> SPEC -> sync -> trim -> supcon (as in nl2fl_grid.ipynb)."""

import multiprocessing
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import pitct
from flloat.parser.ltlf import LTLfParser
from pitct import create, marker, statenum, supcon, sync, trans, trim


@dataclass
class World:
    """Plant model: states, labelled transitions, and how events are labelled.

    states[0] is always `initial`. transitions are (src, event, dst, kind) with
    kind "c" (controllable) or "u" (uncontrollable). event_label holds the label
    of each controllable event (the propositions true after it happens).
    """

    states: list
    initial: Any
    transitions: list  # list[tuple[src, event: str, dst, kind: "c" | "u"]]
    marked: list
    event_label: dict  # dict[str, frozenset[str]]
    initial_label: frozenset
    propositions: tuple


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
    """The formula cannot be read, or turning it into an automaton takes too long."""


def _guard_true(guard, labels) -> bool:
    syms = getattr(guard, "free_symbols", set())
    return bool(guard.subs({s: (str(s) in labels) for s in syms}))


def spec_from_formula(formula: str, world: World) -> Automaton:
    """The flloat DFA of `formula` as a TCT automaton over the events of `world`.

    State 0 is the DFA state after reading the initial label. A controllable
    event moves the DFA by its label; an uncontrollable event is a self-loop.
    """
    try:
        parsed = LTLfParser()(formula)
    except Exception as exc:  # lark and flloat raise several error types for bad input
        raise SpecError(f"式が読めません: {formula}") from exc
    dfa = parsed.to_automaton()

    outgoing: dict[Any, list] = {}
    for src, guard, dst in dfa.get_transitions():
        outgoing.setdefault(src, []).append((guard, dst))
    atoms = {str(s) for edges in outgoing.values() for g, _ in edges
             for s in getattr(g, "free_symbols", set())}
    steps: dict[tuple, Any] = {}

    def dfa_step(q, labels):
        key = (q, frozenset(labels) & atoms)
        if key not in steps:
            dst = next((d for g, d in outgoing.get(q, []) if _guard_true(g, key[1])), None)
            if dst is None:
                raise RuntimeError(f"no DFA transition from {q} on {sorted(labels)}")
            steps[key] = dst
        return steps[key]

    events = sorted({(ev, kind) for _, ev, _, kind in world.transitions})
    q_start = dfa_step(dfa.initial_state, world.initial_label)
    order = [q_start] + [q for q in sorted(dfa.states) if q != q_start]
    qid = {q: i for i, q in enumerate(order)}
    spec_trans = [
        (qid[q], ev, qid[dfa_step(q, world.event_label[ev]) if kind == "c" else q])
        for q in order for ev, kind in events
    ]
    return Automaton(
        name="SPEC",
        size=len(order),
        trans=spec_trans,
        marked={qid[q] for q in order if q in dfa.accepting_states},
        uncontrollable={ev for ev, kind in events if kind == "u"},
    )


def _spec_worker(conn, formula: str, world: World) -> None:
    try:
        conn.send(("ok", spec_from_formula(formula, world)))
    except SpecError as exc:
        conn.send(("spec_error", str(exc)))
    except Exception as exc:
        conn.send(("error", repr(exc)))
    finally:
        conn.close()


def _spec_with_timeout(formula: str, world: World, timeout_s: float) -> Automaton:
    """spec_from_formula in a child process, killed after timeout_s seconds."""
    ctx = multiprocessing.get_context("fork")
    recv, send = ctx.Pipe(duplex=False)
    proc = ctx.Process(target=_spec_worker, args=(send, formula, world), daemon=True)
    proc.start()
    send.close()
    try:
        if not recv.poll(timeout_s):
            raise SpecError("式が大きすぎます")
        try:
            status, value = recv.recv()
        except EOFError:
            raise RuntimeError("SPEC process exited without a result") from None
    finally:
        if proc.is_alive():
            proc.terminate()
        proc.join()
        recv.close()
    if status == "spec_error":
        raise SpecError(value)
    if status == "error":
        raise RuntimeError(f"SPEC process failed: {value}")
    return value


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
    """formula -> SPEC, then sync with the plant, trim, and supcon (TCT files in workdir).

    Raises SpecError when the formula cannot be read or flloat needs more than timeout_s.
    """
    spec = _spec_with_timeout(formula, world, timeout_s)
    plant = plant_from_world(world)
    # init() sets SAVE_FOLDER to a Path (a str fails) and forgets earlier event names.
    pitct.init(Path(workdir).resolve(), overwrite=True)
    _create(plant)
    _create(spec)
    sync("PRODUCT", "PLANT", "SPEC")
    trim("TRIMMED", "PRODUCT")
    supcon("SUP", "PRODUCT", "TRIMMED")
    product, trimmed, sup = (_read(n, plant.uncontrollable) for n in ("PRODUCT", "TRIMMED", "SUP"))
    return Synthesis(formula, plant, spec, product, trimmed, sup)
