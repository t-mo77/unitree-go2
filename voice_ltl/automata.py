"""Shared types for the plant (World) and the automata built from LTLf formulas."""

from dataclasses import dataclass, field
from typing import Any


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
    name: str
    size: int
    trans: dict = field(default_factory=dict)
    marked: set = field(default_factory=set)
    uncontrollable: set = field(default_factory=set)
