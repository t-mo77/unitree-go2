from go2_nl_control.actions import LYING

from voice_ltl.automata import Automaton, synthesize
from voice_ltl.diagrams import render_svg
from voice_ltl.planner import Runtime
from voice_ltl.world import PlantState, go2_world


def test_svg_merges_edges_and_highlights(tmp_path):
    s = synthesize("F hello", go2_world(PlantState(LYING, False, False)), tmp_path)
    rt = Runtime(s.sup)
    svg = render_svg(s.sup, "SUP", highlight_state=0, highlight_events=[(0, rt.plan()[0])])
    assert svg.startswith("<?xml") or svg.lstrip().startswith("<svg")
    assert "stand" in svg and 'class="hl"' in svg and "dashed" in svg


def test_empty_automaton_svg():
    assert "満たせません" in render_svg(Automaton("SUP", 0, [], set(), set()), "SUP")


def test_parallel_edges_merge_and_overflow_is_counted():
    events = [f"{a}_to_x" for a in ("stand", "sit", "hello", "dance", "stretch", "turn_left")]
    a = Automaton("A", 2, [(0, e, 1) for e in events] + [(1, "front_blocked", 0)], {1}, {"front_blocked"})
    svg = render_svg(a, "A")
    assert svg.count("<g id=\"edge") == 3  # start arrow, 0->1 (merged), 1->0
    assert "ほか 2 件" in svg
    assert "doublecircle" in svg or svg.count("<ellipse") >= 3  # marked state: double ring


def _focus_fixture():
    a = Automaton("A", 5, [(0, "stand_to_x", 1), (1, "sit_to_x", 2), (1, "hello_to_x", 3),
                           (1, "dance_to_x", 4), (1, "front_blocked", 0), (0, "dance_to_x", 4)],
                  {2}, {"front_blocked"})
    return a, [(0, "stand_to_x"), (1, "sit_to_x")]


def test_focus_svg_is_small_and_marks_the_path():
    from voice_ltl.diagrams import HL_COLOR, render_focus_svg
    a, path = _focus_fixture()
    svg = render_focus_svg(a, "SUP", 0, path)
    nodes = svg.count('<g id="node')
    assert nodes <= len(path) + 2
    assert 'class="hl"' in svg and HL_COLOR == "#0A66D6" and "#0a66d6" in svg.lower()
    assert "stroke-dasharray" in render_focus_svg(a, "SUP", 1, [(1, "sit_to_x")])  # sensor edge dotted
    assert "ほか" in render_focus_svg(a, "SUP", 1, [(1, "sit_to_x")], max_side_edges=1)


def test_focus_svg_done_and_empty():
    from voice_ltl.diagrams import render_focus_svg
    a, _ = _focus_fixture()
    svg = render_focus_svg(a, "SUP", 2, [])
    assert "完了" in svg and svg.count('<g id="node') <= 2
    assert "満たせません" in render_focus_svg(Automaton("S", 0, [], set(), set()), "S", 0, [])
