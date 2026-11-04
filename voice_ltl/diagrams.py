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
HL_COLOR = "#0A66D6"
HL_FILL = "#D6E6FB"
GREY = "#86868B"
FONT = "-apple-system, 'Hiragino Sans', sans-serif"
DONE_TEXT = "完了"
WAITING_TEXT = "待っています"
_HL_GROUP = re.compile(r'(<g id="(?:edge|node)\d+" class="(?:edge|node)(?: dashed)? hl">.*?</g>)', re.S)
EMPTY_TEXT = "（空：この指示は満たせません）"


def _empty_svg(title: str) -> str:
    return (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<svg xmlns="http://www.w3.org/2000/svg" width="360" height="80" viewBox="0 0 360 80">'
        f'<title>{xml.escape(title)}</title>'
        '<text x="180" y="45" text-anchor="middle" font-family="sans-serif" font-size="16">'
        f'{xml.escape(EMPTY_TEXT)}</text></svg>')


def _pipe_svg(g: graphviz.Digraph) -> str:
    """Render to SVG. graphviz writes class="edge hl"; the page looks for class="hl", so wrap
    those groups."""
    return _HL_GROUP.sub(lambda m: f'<g class="hl">{m.group(1)}</g>', g.pipe(encoding="utf-8"))


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
    return _pipe_svg(g)


def render_focus_svg(a: Automaton, title: str, current: int, path: Sequence[tuple[int, str]],
                     max_side_edges: int = 6) -> str:
    """Small readable view: the current state, the states on the remaining plan path (thick
    blue arrows, class="hl"), and the other arrows leaving the current state (thin grey;
    sensor events dotted). Targets not drawn collapse into one 「…」 node. 「完了」 is shown
    only when nothing remains of the path and the current state is marked; an empty path in an
    unmarked state means waiting (title gets 「待っています」 and the outgoing arrows are drawn)."""
    if a.size == 0:
        return _empty_svg(title)
    nxt = {(src, ev): dst for src, ev, dst in a.trans}
    path_edges: dict[tuple[int, int], list[str]] = defaultdict(list)
    drawn = [current]
    for src, ev in path:
        dst = nxt.get((src, ev))
        if dst is None:
            continue
        path_edges[(src, dst)].append(ev)
        for st in (src, dst):
            if st not in drawn:
                drawn.append(st)
    done = not path_edges and current in a.marked
    if not path_edges and not done:
        title = f"{title}（{WAITING_TEXT}）"
    g = graphviz.Digraph(name="G", format="svg")
    g.attr(rankdir="LR", label=title, labelloc="t", fontname=FONT, fontcolor=GREY)
    g.attr("node", fontname=FONT, shape="circle", fontsize="12", color=GREY, fontcolor="#1D1D1F")
    g.attr("edge", fontname=FONT, fontsize="11", color=GREY, fontcolor=GREY)
    for st in drawn:
        attrs = {"shape": "doublecircle" if st in a.marked else "circle"}
        if st == current:
            attrs.update(style="filled", fillcolor=HL_COLOR, color=HL_COLOR, fontcolor="white",
                         **{"class": "hl"})
        g.node(str(st), **attrs)
    if done:
        g.node("__done", label=DONE_TEXT, shape="plaintext", fontcolor=HL_COLOR)
        g.edge(str(current), "__done", style="invis")
    else:
        for (src, dst), events in path_edges.items():
            g.edge(str(src), str(dst), label=_label(events), color=HL_COLOR, fontcolor=HL_COLOR,
                   penwidth="3.0", **{"class": "hl"})
        path_here = {ev for src, ev in path if src == current}
        side = []  # (dst, events, dotted); the path's own arrows are already drawn thick
        for (_, dst), events in sorted(_group_from(a, current).items(), key=lambda kv: kv[0][1]):
            for dotted in (False, True):
                evs = [e for e in events if (e in a.uncontrollable) == dotted and (dotted or e not in path_here)]
                if evs:
                    side.append((dst, evs, dotted))
        shown, rest = side[:max_side_edges], side[max_side_edges:]
        need_dots = bool(rest) or any(d not in drawn for d, _, _ in shown)
        if need_dots:
            g.node("__more", label="…", shape="circle", width="0.3", fontsize="12")
        for dst, evs, dotted in shown:
            tgt = str(dst) if dst in drawn else "__more"
            g.edge(str(current), tgt, label=_label(evs), penwidth="1.0",
                   **({"style": "dotted"} if dotted else {}))
        if rest:
            n = sum(len(set(_name(e) for e in evs)) for _, evs, _ in rest)
            g.edge(str(current), "__more", label=f"ほか {n} 件", penwidth="1.0")
    return _pipe_svg(g)


def _group_from(a: Automaton, src: int) -> dict[tuple[int, int], list[str]]:
    groups: dict[tuple[int, int], list[str]] = defaultdict(list)
    for s, ev, dst in a.trans:
        if s == src:
            groups[(s, dst)].append(ev)
    return groups
