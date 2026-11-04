"""State-transition diagrams as SVG (graphviz `dot`), for the Web page (design §7.8).

Arrows between the same two states are merged into one whose label lists the action
names (event_action), at most MAX_LABELS of them, then 「ほか N 件」. Uncontrollable events
are dashed, marked states are double circles. highlight_state is filled (class="hl" too); highlight_events
are thick coloured arrows carrying class="hl".
"""

import re
import xml.sax.saxutils as xml
from collections import defaultdict
from typing import Sequence

import graphviz

from voice_ltl.automata import Automaton
from voice_ltl.world import event_action

MAX_LABELS = 4
HL_COLOR = "#e8590c"
HL_FILL = "#ffd8a8"
_HL_GROUP = re.compile(r'(<g id="(?:edge|node)\d+" class="(?:edge|node)(?: dashed)? hl">.*?</g>)', re.S)
EMPTY_TEXT = "（空：この指示は満たせません）"


def _empty_svg(title: str) -> str:
    return (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<svg xmlns="http://www.w3.org/2000/svg" width="360" height="80" viewBox="0 0 360 80">'
        f'<title>{xml.escape(title)}</title>'
        '<text x="180" y="45" text-anchor="middle" font-family="sans-serif" font-size="16">'
        f'{xml.escape(EMPTY_TEXT)}</text></svg>')


def _name(event: str) -> str:
    return event_action(event) or event  # sensor events keep their own name


def _label(events: list[str]) -> str:
    names = list(dict.fromkeys(_name(e) for e in sorted(events)))
    shown = names[:MAX_LABELS]
    if len(names) > MAX_LABELS:
        shown.append(f"ほか {len(names) - MAX_LABELS} 件")
    return "\\n".join(shown)


def render_svg(a: Automaton, title: str, highlight_state: int | None = None,
               highlight_events: Sequence[tuple[int, str]] = ()) -> str:
    if a.size == 0:
        return _empty_svg(title)
    hl = set(highlight_events)
    g = graphviz.Digraph(name="G", format="svg")
    g.attr(rankdir="LR", label=title, labelloc="t", fontname="sans-serif")
    g.attr("node", fontname="sans-serif", shape="circle", fontsize="11")
    g.attr("edge", fontname="sans-serif", fontsize="10")
    g.node("__start", label="", shape="none", width="0", height="0")
    for s in range(a.size):
        attrs = {"shape": "doublecircle" if s in a.marked else "circle"}
        if s == highlight_state:
            attrs.update(style="filled", fillcolor=HL_FILL, color=HL_COLOR, **{"class": "hl"})
        g.node(str(s), **attrs)
    g.edge("__start", "0")
    groups: dict[tuple[int, int], list[str]] = defaultdict(list)
    for src, ev, dst in a.trans:
        groups[(src, dst)].append(ev)
    for (src, dst), events in sorted(groups.items()):
        attrs = {"label": _label(events)}
        # dashed only when every merged event is uncontrollable
        if all(e in a.uncontrollable for e in events):
            attrs.update(style="dashed", class_="dashed")
        if any((src, e) in hl for e in events):
            attrs.update(color=HL_COLOR, fontcolor=HL_COLOR, penwidth="3.0")
            attrs["class_"] = (attrs.get("class_", "") + " hl").strip()
        g.edge(str(src), str(dst), **{k.rstrip("_"): v for k, v in attrs.items()})
    svg = g.pipe(encoding="utf-8")
    # graphviz writes class="edge hl"; the page looks for class="hl", so wrap those groups.
    return _HL_GROUP.sub(lambda m: f'<g class="hl">{m.group(1)}</g>', svg)
