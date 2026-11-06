"""GridExecutor: closed-loop moves to a cell centre and a grid heading, a settle after StopMove,
and the zip's holds and timeouts (R11, R12, P7)."""

import json
import math

from go2_nl_control.actions import MOVE, STANDING, STOP_MOVE
from go2_nl_control.supervisor import CLEAR, SENSOR_LOST, Step

from sim.sim_core import SimGo2
from voice_ltl.grid_executor import (GRID_DONE_DIST, GRID_DONE_YAW, GRID_EXEC_CONFIG, GRID_SETTLE_S,
                                     GridExecutor, GridStep)
from voice_ltl.session import EXEC_ACTIONS


def standing_sim():
    return SimGo2(posture=STANDING)


def _tick(sim, ex, t, env, cmds):
    sim.step(t)
    for c in ex.tick(t, env, (sim.x, sim.y, sim.yaw)):
        sim.handle(c.api_id, "" if c.parameter is None else json.dumps(c.parameter), t)
        cmds.append((t, c))


def run(sim, ex, steps, env=CLEAR, dt=0.1, limit=30.0):
    """Load steps and tick ex against sim every dt until it is done; the (time, command)s."""
    ex.load(steps)
    cmds, t = [], 0.0
    while ex.busy and t < limit:
        t = round(t + dt, 6)
        _tick(sim, ex, t, env, cmds)
    assert not ex.busy, "not done in time"
    return cmds


def run_until_done(sim, ex, step, env_at=lambda t: CLEAR, dt=0.1, limit=30.0):
    """run() for one step, env_at(t) the env of each tick; also the time ex.busy became False."""
    ex.load([step])
    cmds, t = [], 0.0
    while ex.busy and t < limit:
        t = round(t + dt, 6)
        _tick(sim, ex, t, env_at(t), cmds)
    assert not ex.busy, "not done in time"
    return cmds, t


def test_constants():
    assert GRID_SETTLE_S == 0.5 and GRID_DONE_DIST == 0.03 and GRID_DONE_YAW == math.radians(3.0)
    assert GRID_EXEC_CONFIG.speed == (0.3, 0.3, 0.5) and GRID_EXEC_CONFIG.acc == (0.6, 0.6, 1.0)
    assert GRID_EXEC_CONFIG.block_timeout == 15.0 and GRID_EXEC_CONFIG.move_timeout_margin == 2.5
    assert GridExecutor(EXEC_ACTIONS).cfg == GRID_EXEC_CONFIG


def test_forward_reaches_the_cell_centre_and_holds_heading():
    sim = standing_sim(); sim.y, sim.yaw = 0.1, 0.05
    cmds = run(sim, GridExecutor(EXEC_ACTIONS), [GridStep("forward", 0.5, (0.5, 0.0), 0.0)])
    assert abs(sim.x - 0.5) <= 0.03 + 0.03 and abs(sim.y) < 0.1 and abs(sim.yaw) < 0.1   # no drift added
    moves = [c for _, c in cmds if c.api_id == MOVE]
    assert all(abs(m.parameter["z"]) <= 0.3 and abs(m.parameter["x"]) <= 0.3 for m in moves)
    assert all(abs(b.parameter["x"] - a.parameter["x"]) <= 0.06 + 1e-6 for a, b in zip(moves, moves[1:]))
    assert all(m.parameter["y"] == 0 for m in moves)
    assert min(m.parameter["z"] for m in moves) < 0                   # it steered back towards the line


def test_forward_corrects_a_sideways_offset_more_than_open_loop():
    sim = standing_sim(); sim.y = 0.2
    run(sim, GridExecutor(EXEC_ACTIONS), [GridStep("forward", 0.5, (0.5, 0.0), 0.0),
                                          GridStep("forward", 0.5, (1.0, 0.0), 0.0),
                                          GridStep("forward", 0.5, (1.5, 0.0), 0.0)])
    assert abs(sim.y) < 0.2 and abs(sim.x - 1.5) <= 0.06


def test_turn_reaches_the_heading_slowly():
    sim = standing_sim()
    cmds = run(sim, GridExecutor(EXEC_ACTIONS), [GridStep("turn_left", 90.0, (0.0, 0.0), math.pi / 2)])
    assert abs(math.remainder(sim.yaw - math.pi / 2, 2 * math.pi)) <= math.radians(3) + 0.06
    zs = [c.parameter["z"] for _, c in cmds if c.api_id == MOVE]
    assert max(map(abs, zs)) <= 0.5 and all(abs(b - a) <= 0.1 + 1e-6 for a, b in zip(zs, zs[1:]))
    assert all(c.parameter["x"] == 0 for _, c in cmds if c.api_id == MOVE)


def test_turn_right_across_the_wrap():
    sim = standing_sim(); sim.yaw = -math.pi + 0.2 + 4 * math.pi   # an unwrapped yaw, as SimGo2 keeps it
    run(sim, GridExecutor(EXEC_ACTIONS), [GridStep("turn_right", 90.0, (0.0, 0.0), math.pi / 2 + 0.2)])
    assert abs(math.remainder(sim.yaw - (math.pi / 2 + 0.2), 2 * math.pi)) <= math.radians(3) + 0.06
    assert sim.yaw < -math.pi + 0.2 + 4 * math.pi                  # the short way: clockwise


def test_done_only_after_the_settle_time():
    sim, ex = standing_sim(), GridExecutor(EXEC_ACTIONS)
    cmds, done_at = run_until_done(sim, ex, GridStep("forward", 0.5, (0.5, 0.0), 0.0))
    stop_at = max(t for t, c in cmds if c.api_id == STOP_MOVE)
    assert done_at - stop_at >= GRID_SETTLE_S - 1e-9
    assert [c.api_id for t, c in cmds if t > stop_at] == []           # nothing sent while settling
    assert sum(c.api_id == STOP_MOVE for _, c in cmds) == 1            # one StopMove


def test_sensor_lost_during_settle_delays_done():
    sim, ex = standing_sim(), GridExecutor(EXEC_ACTIONS)
    step = GridStep("forward", 0.5, (0.5, 0.0), 0.0)
    cmds, clear_done = run_until_done(sim, ex, step)
    stop_at = max(t for t, c in cmds if c.api_id == STOP_MOVE)
    lost_from = round(stop_at + 0.2, 6)                                 # inside the 0.5 s settle
    sim2, ex2 = standing_sim(), GridExecutor(EXEC_ACTIONS)
    _, done_at = run_until_done(sim2, ex2, step,
                                env_at=lambda t: SENSOR_LOST if lost_from <= t < lost_from + 1.0 else CLEAR)
    assert done_at >= lost_from + 1.0 - 1e-9 and done_at > clear_done
    assert any(e.startswith("一時停止") for e in ex2.events)


def test_backward_goal_behind():
    sim = standing_sim()
    run(sim, GridExecutor(EXEC_ACTIONS), [GridStep("backward", 0.5, (-0.5, 0.0), 0.0)])
    assert abs(sim.x + 0.5) <= 0.06 and abs(sim.y) < 0.05 and abs(sim.yaw) < 0.05


def test_backward_corrects_a_sideways_offset():
    sim = standing_sim(); sim.y = 0.1
    run(sim, GridExecutor(EXEC_ACTIONS), [GridStep("backward", 0.5, (-0.5, 0.0), 0.0),
                                          GridStep("backward", 0.5, (-1.0, 0.0), 0.0)])
    assert abs(sim.y) < 0.1 and abs(sim.x + 1.0) <= 0.06


def test_forward_is_held_by_a_blocked_front_and_times_out_as_the_executor():
    sim, ex = standing_sim(), GridExecutor(EXEC_ACTIONS)
    ex.load([GridStep("forward", 0.5, (0.5, 0.0), 0.0)])
    cmds = []
    for k in range(1, 170):
        _tick(sim, ex, k / 10, "BLOCKED", cmds)
    assert not ex.steps and any("15 秒待っても" in e for e in ex.events)   # block_timeout 15 s
    assert sim.x == 0.0 and all(c.api_id == STOP_MOVE for _, c in cmds)


def test_a_move_that_never_arrives_is_cancelled():
    class Stuck(SimGo2):           # takes the commands but never moves
        def step(self, now):
            pass

    sim, ex = Stuck(posture=STANDING), GridExecutor(EXEC_ACTIONS)
    ex.load([GridStep("forward", 0.5, (0.5, 0.0), 0.0)])
    cmds = []
    for k in range(1, 200):
        _tick(sim, ex, k / 10, CLEAR, cmds)
    ideal = 0.5 / 0.3
    assert not ex.steps and any("終わりません" in e for e in ex.events)
    cancel_at = max(t for t, c in cmds)
    assert ideal * 2.5 + 5.0 <= cancel_at <= ideal * 2.5 + 5.0 + 0.3


def test_gestures_run_as_in_the_executor():
    sim, ex = SimGo2(), GridExecutor(EXEC_ACTIONS)
    run(sim, ex, [Step("stand"), Step("hello")])
    assert sim.accepted[:3] == [1004, 1002, 1016]


def test_grid_step_text_is_the_step_text():
    assert GridStep("forward", 0.5, (1, 2), 0.3).text(EXEC_ACTIONS) == Step("forward", 0.5).text(EXEC_ACTIONS)
    assert GridStep("turn_left", 90.0, (1, 2), 0.3).text(EXEC_ACTIONS) == "turn_left(90度)"
