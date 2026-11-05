"""Synthesis memory and time on a large plant (design §4, §6.1, §8; Ruling R28)."""

import json
import os
import subprocess
import sys
import time
from pathlib import Path

import pytest
from go2_nl_control.actions import LYING

from voice_ltl.automata import SpecError, World, synthesize
from voice_ltl.prompt import _repeat
from voice_ltl.world import (
    ACTION_NAMES, PROPOSITIONS, UNSAFE_LABELS, PlantState, go2_label, go2_sensor_events, go2_step, go2_world,
)

ROOT = Path(__file__).resolve().parent.parent
HEADINGS = ((1, 0), (0, 1), (-1, 0), (0, -1))  # heading h points along HEADINGS[h]; turn_left adds 1


def synthetic_grid_world(w: int, h: int) -> World:
    """A w x h grid plant: state (cell, heading, posture, front, back), sensor bits in every
    posture. forward/backward move one cell along the heading and only inside the grid, turns
    change the heading, posture changes and gestures follow ACTIONS. Labels are the go2 labels
    (posture, action and obstacle bits), so the events are go2_world's. 8 x 8 gives 3072 states."""
    initial = ((0, 0), 0, LYING, False, False)
    states = [initial]
    index = {initial: 0}
    transitions = []
    event_label: dict[str, frozenset[str]] = {}
    i = 0
    while i < len(states):
        src = states[i]
        i += 1
        (x, y), head, posture, front, back = src
        plant = PlantState(posture, front, back)
        outgoing = []
        for action in ACTION_NAMES:
            step = go2_step(plant, action)
            if step is None:
                continue
            ev, dst = step
            cell, new_head = (x, y), head
            dx, dy = HEADINGS[head]
            if action == "forward":
                cell = (x + dx, y + dy)
            elif action == "backward":
                cell = (x - dx, y - dy)
            elif action == "turn_left":
                new_head = (head + 1) % 4
            elif action == "turn_right":
                new_head = (head - 1) % 4
            if not (0 <= cell[0] < w and 0 <= cell[1] < h):
                continue
            event_label[ev] = go2_label(dst, action)
            outgoing.append((src, ev, (cell, new_head, dst.posture, dst.front, dst.back), "c"))
        for ev, dst in go2_sensor_events(plant):
            outgoing.append((src, ev, ((x, y), head, dst.posture, dst.front, dst.back), "u"))
        for _, _, dst, _ in outgoing:
            if dst not in index:
                index[dst] = len(states)
                states.append(dst)
        transitions += outgoing
    return World(
        states=states,
        initial=initial,
        transitions=transitions,
        marked=list(states),
        event_label=event_label,
        initial_label=go2_label(PlantState(LYING, False, False), None),
        propositions=PROPOSITIONS,
        forbidden=frozenset(ev for ev, label in event_label.items()
                            if any(unsafe <= label for unsafe in UNSAFE_LABELS)),
    )


# The child caps its address space so that a regression fails this test instead of
# OOM-killing the Docker VM (the old supcon(PRODUCT, TRIMMED) needs over 10 GB here).
CHILD = """
import json, resource, tempfile, time
from pathlib import Path
resource.setrlimit(resource.RLIMIT_AS, (3 * 2**30, 3 * 2**30))
from test_synthesis_memory import synthetic_grid_world
from voice_ltl.automata import SpecError, synthesize
from voice_ltl.ltlf import TOO_BIG
from voice_ltl.prompt import _repeat
world = synthetic_grid_world(8, 8)
with tempfile.TemporaryDirectory() as tmp:
    t0 = time.monotonic()
    try:
        synthesize(_repeat("hello", 12), world, Path(tmp))
        result = "sup"
    except SpecError as exc:
        result = "too_big" if str(exc) == TOO_BIG else f"error: {exc}"
    seconds = time.monotonic() - t0
print(json.dumps({"seconds": seconds, "maxrss_mb": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss // 1024,
                  "result": result}))
"""


def test_synthetic_grid_size():
    world = synthetic_grid_world(8, 8)
    assert len(world.states) == 3072 and world.forbidden
    assert {ev for _, ev, _, _ in world.transitions} == {ev for _, ev, _, _ in go2_world(
        PlantState(LYING, False, False)).transitions}


def test_grid_hello12_finishes_fast_in_little_memory():
    proc = subprocess.run(
        [sys.executable, "-c", CHILD], cwd=ROOT,
        env={**os.environ, "PYTHONPATH": f"{ROOT}:{ROOT}/nl_control:{ROOT}/tests"},
        capture_output=True, text=True, timeout=120)
    assert proc.returncode == 0, proc.stderr[-2000:]
    got = json.loads(proc.stdout.strip().splitlines()[-1])
    assert got["result"] in ("sup", "too_big"), got
    assert got["seconds"] < 5.0, got
    assert got["maxrss_mb"] < 1024, got


@pytest.mark.slow
def test_latent_go2_formula_is_refused_in_time(tmp_path):
    formula = f"{_repeat('hello', 12)} & {_repeat('danced', 12)} & {_repeat('heart', 10)}"
    t0 = time.monotonic()
    with pytest.raises(SpecError, match="^式が大きすぎます$"):
        synthesize(formula, go2_world(PlantState(LYING, False, False)), tmp_path)
    assert time.monotonic() - t0 < 25.0
