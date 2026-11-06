"""GridExecutor: closed-loop moves to a cell centre and a grid heading, a settle after StopMove,
the zip's holds and timeouts (R11, R12, P7), the steps that flow on without a stop (M1, and
the corners: a forward into the turn after it), and the Gazebo tuning C2 (a turn ends before
the walk after it starts; forwards at 0.22 m/s, turns at 0.4 rad/s)."""

import json
import math

import pytest
from go2_nl_control.actions import HELLO, MOVE, STANDING, STOP_MOVE
from go2_nl_control.supervisor import BLOCKED, CLEAR, SENSOR_LOST, Step

from sim.sim_core import SimGo2
from voice_ltl.grid_executor import (GRID_DONE_DIST, GRID_DONE_YAW, GRID_EXEC_CONFIG, GRID_SETTLE_S,
                                     K_ALONG, K_CROSS, K_YAW, MIN_SPEED, MIN_TURN, MOVE_TURN,
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
    assert GRID_SETTLE_S == 0.2 and GRID_DONE_DIST == 0.03 and GRID_DONE_YAW == math.radians(3.0)
    assert GRID_EXEC_CONFIG.speed == (0.22, 0.3, 0.4) and GRID_EXEC_CONFIG.acc == (0.6, 0.6, 1.0)   # C2
    assert GRID_EXEC_CONFIG.block_timeout == 15.0 and GRID_EXEC_CONFIG.move_timeout_margin == 2.5
    assert GridExecutor(EXEC_ACTIONS).cfg == GRID_EXEC_CONFIG


def test_gains_are_p7():
    """P7, as the plan gives them. Task I fix round 1: the tuned gains tried in Gazebo completed no lap
    while P7 did, so they were reverted (docs/reference/gazebo_objects_e2e.md)."""
    assert (K_YAW, K_CROSS) == (1.5, 1.0)                    # yaw and cross-track gains
    assert (K_ALONG, MIN_SPEED) == (1.2, 0.08)               # clip(1.2 · along, 0.08, speed[0])
    assert MOVE_TURN == 0.3                                  # z within ±0.3 while moving
    assert MIN_TURN == 0.15                                  # turn |z| within [0.15, speed[2]]
    # C2 (the Gazebo tuning round, 2026-11-06) lowered only the top speeds: 0.3 -> 0.22 m/s, 0.5 -> 0.4 rad/s
    assert (GRID_EXEC_CONFIG.speed[0], GRID_EXEC_CONFIG.speed[2]) == (0.22, 0.4)


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
    assert max(map(abs, zs)) == 0.4 and all(abs(b - a) <= 0.1 + 1e-6 for a, b in zip(zs, zs[1:]))   # C2
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
    lost_from = round(stop_at + 0.1, 6)                                 # inside the 0.2 s settle
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


@pytest.mark.parametrize("step", [GridStep("forward", 0.5, (1.0, 0.0), 0.0),
                                  GridStep("turn_right", 90.0, (0.5, 0.0), -math.pi / 2)],
                         ids=["forward", "turn_right"])
def test_a_step_loaded_after_an_idle_gap_ramps_from_zero(step):
    """Design §6.5 and P7 (final review, Important 2): the session ticks the executor only while
    a step is loaded, so the next step (a second instruction, the move after the R1-amendment
    wait or after a posture settle) can come long after the last tick. load() starts the ramp
    afresh: the first Move is not the full speed that a capped 0.5 s dt would give."""
    sim, ex = standing_sim(), GridExecutor(EXEC_ACTIONS)
    cmds, t = run_until_done(sim, ex, GridStep("forward", 0.5, (0.5, 0.0), 0.0))
    ex.load([step])                          # 10 s with no tick: the Go2 stood still
    cmds = []
    for k in range(30):
        _tick(sim, ex, round(t + 10.0 + k * 0.1, 6), CLEAR, cmds)
    moves = [c.parameter for _, c in cmds if c.api_id == MOVE]
    acc_x, _, acc_z = (a * 0.1 for a in GRID_EXEC_CONFIG.acc)   # one 10 Hz tick of acceleration
    assert abs(moves[0]["x"]) <= acc_x + 1e-9 and abs(moves[0]["z"]) <= acc_z + 1e-9
    assert all(abs(b["x"] - a["x"]) <= acc_x + 1e-3 and abs(b["z"] - a["z"]) <= acc_z + 1e-3
               for a, b in zip(moves, moves[1:]))
    assert max(abs(m["x"]) + abs(m["z"]) for m in moves) > acc_z    # and it does get going


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


# ---------------- M1: steps that flow on without a stop ----------------
RUN = [GridStep("forward", 0.5, (0.5, 0.0), 0.0), GridStep("forward", 0.5, (1.0, 0.0), 0.0),
       GridStep("forward", 0.5, (1.5, 0.0), 0.0)]
ACC_X, _, ACC_Z = (a * 0.1 for a in GRID_EXEC_CONFIG.acc)   # one 10 Hz tick of acceleration


def assert_ramps(cmds):
    """Every Move within one tick of acceleration of the command before it (StopMove is 0)."""
    x0 = z0 = 0.0
    for _, c in cmds:
        if c.api_id == STOP_MOVE:
            x0 = z0 = 0.0
        elif c.api_id == MOVE:
            x, z = c.parameter["x"], c.parameter["z"]
            assert abs(x - x0) <= ACC_X + 1e-3 and abs(z - z0) <= ACC_Z + 1e-3, (c, x0, z0)
            x0, z0 = x, z


def assert_no_gap(cmds, until):
    """A command in every tick from the first one to `until`: no tick left at the last velocity."""
    ticks = sorted({t for t, _ in cmds if t <= until + 1e-9})
    assert all(b - a <= 0.1 + 1e-6 for a, b in zip(ticks, ticks[1:])), ticks


def flow_one(sim, ex, step, into="forward"):
    """As the session drives a step it knows the next action (`into`) goes on from: flow set,
    one step loaded."""
    ex.flow = into
    cmds, t = run_until_done(sim, ex, step)
    ex.flow = None
    return cmds, t


def test_consecutive_forwards_flow_without_a_stop():
    """M1: a forward followed by a forward in its heading is done as it passes the cell centre,
    with no StopMove and at full speed: x stays positive from the first Move to the last
    forward's goal, which ends as before (StopMove, then the settle)."""
    sim, ex = standing_sim(), GridExecutor(EXEC_ACTIONS)
    cmds = run(sim, ex, RUN)
    stops = [t for t, c in cmds if c.api_id == STOP_MOVE]
    assert len(stops) == 1                                            # the end of the run only
    moves = [c.parameter for t, c in cmds if c.api_id == MOVE]
    first = next(i for i, m in enumerate(moves) if m["x"] > 0)
    assert all(m["x"] > 0 for m in moves[first:])
    assert max(m["x"] for m in moves) == GRID_EXEC_CONFIG.speed[0] == 0.22   # C2's cruise
    assert_ramps(cmds)
    assert_no_gap(cmds, stops[0])
    assert abs(sim.x - 1.5) <= 0.06 and abs(sim.y) < 0.05
    assert [c for t, c in cmds if t > stops[0]] == []                 # the settle after the last one
    assert [e for e in ex.events if e.startswith("完了: ")] == ["完了: forward(0.5m)"] * 3


def test_a_forward_loaded_after_one_that_flowed_on_continues_the_walk():
    """M1 as the session drives it: one step at a time, `flow` set from the plan. The first forward
    ends without StopMove as it passes (0.5, 0); the next, loaded in the same tick, goes on from
    the velocity the Go2 has (no ramp from 0) towards its own goal."""
    sim, ex = standing_sim(), GridExecutor(EXEC_ACTIONS)
    cmds, t = flow_one(sim, ex, RUN[0])
    assert not [c for _, c in cmds if c.api_id == STOP_MOVE] and ex.rolling
    assert 0.5 - GRID_DONE_DIST <= sim.x <= 0.5 + 0.03 + 1e-6        # done as it passed the centre
    last = [c.parameter for _, c in cmds if c.api_id == MOVE][-1]
    assert last["x"] == GRID_EXEC_CONFIG.speed[0]
    ex.load([RUN[1]])
    assert not ex.rolling
    more = []
    _tick(sim, ex, t, CLEAR, more)                                    # the same tick
    assert [c.api_id for _, c in more] == [MOVE] and more[0][1].parameter["x"] == last["x"]
    while ex.busy:
        t = round(t + 0.1, 6)
        _tick(sim, ex, t, CLEAR, more)
    assert [c.api_id for _, c in more].count(STOP_MOVE) == 1         # the run ends here
    assert all(c.parameter["x"] > 0 for _, c in more if c.api_id == MOVE)
    assert_ramps(cmds + more)
    assert abs(sim.x - 1.0) <= 0.06


@pytest.mark.parametrize("nxt", [GridStep("turn_left", 90.0, (0.5, 0.0), math.pi / 2),
                                 GridStep("forward", 0.5, (0.5, 0.5), math.pi / 2),
                                 GridStep("backward", 0.5, (0.0, 0.0), 0.0),
                                 Step("hello")],
                         ids=["turn", "forward_other_heading", "backward", "hello"])
def test_a_step_after_a_pass_that_does_not_continue_it_stops_first(nxt):
    """M1: a step loaded after a pass that is not a forward in the same heading (a jump or a
    sensor change changed the plan) gets StopMove in its first tick, then GRID_SETTLE_S of
    nothing, then starts from 0."""
    sim, ex = standing_sim(), GridExecutor(EXEC_ACTIONS)
    _, t = flow_one(sim, ex, RUN[0])
    ex.load([nxt])
    cmds = []
    _tick(sim, ex, t, CLEAR, cmds)                                    # the same tick
    assert [c.api_id for _, c in cmds] == [STOP_MOVE]
    why = f"M1: 続けないので止めます（{nxt.text(EXEC_ACTIONS)} は forward(0.5m) の続きではありません）"
    assert [e for e in ex.events if e.startswith("M1: ")] == [why]    # for the debug log (the review of M1)
    for k in range(1, 40):
        _tick(sim, ex, round(t + k * 0.1, 6), CLEAR, cmds)
    later = [(tt, c) for tt, c in cmds if tt > t]
    assert later and min(tt for tt, _ in later) >= t + GRID_SETTLE_S - 1e-9
    moves = [c.parameter for _, c in later if c.api_id == MOVE]
    if moves:
        assert abs(moves[0]["x"]) <= ACC_X + 1e-9 and abs(moves[0]["z"]) <= ACC_Z + 1e-9
    else:
        assert HELLO in [c.api_id for _, c in later]


def test_halt_stops_a_step_that_flowed_on_at_once():
    """M1: when nothing continues the walk (the supervisor waits or is done), halt() gives
    StopMove at once; the next step starts GRID_SETTLE_S later, from 0. Once is enough. (The
    session, which knows why, logs it; the executor adds no event.)"""
    sim, ex = standing_sim(), GridExecutor(EXEC_ACTIONS)
    _, t = flow_one(sim, ex, RUN[0])
    out = ex.halt(t)
    assert [c.api_id for c in out] == [STOP_MOVE] and not ex.rolling and ex.halt(t) == []
    assert not [e for e in ex.events if e.startswith("M1: ")]
    ex.load([GridStep("turn_left", 90.0, (0.5, 0.0), math.pi / 2)])
    cmds = []
    for k in range(1, 40):
        _tick(sim, ex, round(t + k * 0.1, 6), CLEAR, cmds)
    assert min(tt for tt, _ in cmds) >= t + GRID_SETTLE_S - 1e-9
    assert all(c.api_id == MOVE for _, c in cmds[:3]) and abs(cmds[0][1].parameter["z"]) <= ACC_Z + 1e-9


ZERO = {"x": 0.0, "y": 0.0, "z": 0.0}


def assert_turn_ends_before_the_walk(moves):
    """C1 (after the tuning round in Gazebo, 2026-11-06): where a turn flows into a forward, the
    turn's z ramps down to 0 first, one tick of acceleration at a time with x at 0, and only in
    the tick after the one at zero velocity does x ramp up from 0. Never x and the turn's z
    together. `moves` are the Move parameters of the turn (from any tick of it) and the walk."""
    k = next(i for i, m in enumerate(moves) if m["x"] > 0)            # the walk starts
    assert moves[k - 1] == ZERO and 0 < moves[k]["x"] <= ACC_X + 1e-9
    assert all(m["x"] == 0 for m in moves[:k])
    zs = [abs(m["z"]) for m in moves[:k]]
    peak = max(range(k), key=lambda i: (zs[i], i))                     # the turn's last fastest tick
    assert all(b <= a for a, b in zip(zs[peak:], zs[peak + 1:]))     # from there it only slows down


def test_a_turn_flows_into_a_forward():
    """M1 and C1: a turn whose next step is a forward ends at its yaw goal with no StopMove and no
    settle; then its z ramps down to 0 (no StopMove: the controller keeps stepping) and only
    then the forward's x ramps up from 0. One Move at zero velocity between them, no more."""
    sim, ex = standing_sim(), GridExecutor(EXEC_ACTIONS)
    steps = [GridStep("turn_left", 90.0, (0.0, 0.0), math.pi / 2),
             GridStep("forward", 0.5, (0.0, 0.5), math.pi / 2)]
    cmds = run(sim, ex, steps)
    stops = [t for t, c in cmds if c.api_id == STOP_MOVE]
    assert len(stops) == 1                                            # the end of the forward only
    moves = [c.parameter for t, c in cmds if c.api_id == MOVE]
    assert moves[0] == ZERO                                           # the first tick of a load has dt 0
    assert len([m for m in moves[1:] if m == ZERO]) == 1              # then once: the turn has ended
    assert_turn_ends_before_the_walk(moves[1:])
    assert_ramps(cmds)
    assert_no_gap(cmds, stops[0])
    assert abs(sim.y - 0.5) <= 0.06 and abs(sim.x) < 0.06


def test_a_turn_loaded_one_at_a_time_flows_into_the_forward():
    """The same, as the session drives it: the forward loaded in the tick the turn ends starts
    with the turn's z and x 0; z falls by one tick of acceleration at a time to 0 (x stays 0),
    and x is up in the tick after that."""
    sim, ex = standing_sim(), GridExecutor(EXEC_ACTIONS)
    cmds, t = flow_one(sim, ex, GridStep("turn_left", 90.0, (0.0, 0.0), math.pi / 2))
    assert not [c for _, c in cmds if c.api_id == STOP_MOVE] and ex.rolling
    z = [c.parameter for _, c in cmds if c.api_id == MOVE][-1]["z"]
    assert z >= MIN_TURN - 1e-9
    ex.load([GridStep("forward", 0.5, (0.0, 0.5), math.pi / 2)])
    more = []
    for k in range(6):
        _tick(sim, ex, round(t + k * 0.1, 6), CLEAR, more)
    assert [c.api_id for _, c in more] == [MOVE] * 6
    moves = [c.parameter for _, c in more]
    assert moves[0] == {"x": 0.0, "y": 0.0, "z": z}
    assert moves[1] == {"x": 0.0, "y": 0.0, "z": round(z - ACC_Z, 3)}
    assert moves[2] == ZERO                                           # z 0.15 is down in two ticks
    assert 0 < moves[3]["x"] <= ACC_X + 1e-9 and moves[4]["x"] > moves[3]["x"]
    assert_turn_ends_before_the_walk(moves)
    assert_ramps(cmds + more)


def test_a_blocked_front_mid_run_stops_at_once():
    """The holds are the Executor's in a run that flows too: a blocked front sends StopMove in
    the tick it is seen, no forward Move while it lasts, and the walk ramps from 0 after it."""
    sim, ex = standing_sim(), GridExecutor(EXEC_ACTIONS)
    ex.load(RUN)
    cmds, t, x_held = [], 0.0, None
    while ex.busy and t < 30:
        t = round(t + 0.1, 6)
        env = BLOCKED if 3.2 <= t < 4.2 else CLEAR                  # (C2: 0.22 m/s)
        n = len(cmds)
        _tick(sim, ex, t, env, cmds)
        if env == BLOCKED:
            assert not [c for _, c in cmds[n:] if c.api_id == MOVE and c.parameter["x"] > 0]
            x_held = sim.x if x_held is None else x_held
            assert sim.x == x_held
    assert 0.5 < x_held < 1.0                                         # in the second forward, at speed
    assert [c.api_id for tt, c in cmds if tt == 3.2] == [STOP_MOVE]
    after = [c.parameter for tt, c in cmds if tt >= 4.2 and c.api_id == MOVE]
    assert after[0]["x"] <= ACC_X + 1e-9
    assert [c.api_id for _, c in cmds].count(STOP_MOVE) == 2          # the hold and the end
    assert abs(sim.x - 1.5) <= 0.06 and any(e.startswith("一時停止") for e in ex.events)


def test_flow_withdrawn_mid_step_ends_the_step_as_before():
    """M1: when the plan no longer has a forward after it (a sensor change), the step slows down
    and ends with StopMove and the settle at its goal."""
    sim, ex = standing_sim(), GridExecutor(EXEC_ACTIONS)
    ex.flow = "forward"
    ex.load([RUN[0]])
    cmds = []
    for k in range(1, 8):
        _tick(sim, ex, round(k * 0.1, 6), CLEAR, cmds)
    ex.flow = None
    t = 0.7
    while ex.busy:
        t = round(t + 0.1, 6)
        _tick(sim, ex, t, CLEAR, cmds)
    stops = [tt for tt, c in cmds if c.api_id == STOP_MOVE]
    assert len(stops) == 1 and t - stops[0] >= GRID_SETTLE_S - 1e-9 and not ex.rolling
    assert abs(sim.x - 0.5) <= 0.06


def test_cancel_forgets_a_step_that_flowed_on():
    sim, ex = standing_sim(), GridExecutor(EXEC_ACTIONS)
    _, t = flow_one(sim, ex, RUN[0])
    ex.cancel("止める")
    assert not ex.rolling and ex.halt(t) == []


# ---------------- corners: a forward flows into the turn after it ----------------
CORNER = [GridStep("forward", 0.5, (0.5, 0.0), 0.0), GridStep("turn_left", 90.0, (0.5, 0.0), math.pi / 2),
          GridStep("forward", 0.5, (0.5, 0.5), math.pi / 2)]


def test_a_forward_flows_into_the_turn_after_it():
    """Corners: a forward whose next step is a turn in the cell it ends in slows down for its goal
    as before, then is done there with no StopMove and no settle; the turn goes on from that
    velocity (x down to 0, z up), and flows into the forward after it (M1; C1: z down to 0, then
    x up). One StopMove, at the end; after the first Move, zero velocity only once, where the
    turn has ended."""
    sim, ex = standing_sim(), GridExecutor(EXEC_ACTIONS)
    cmds = run(sim, ex, CORNER)
    stops = [t for t, c in cmds if c.api_id == STOP_MOVE]
    assert len(stops) == 1 and stops[0] == max(t for t, _ in cmds)  # the end of the last forward only
    moves = [c.parameter for t, c in cmds if c.api_id == MOVE]
    assert moves[0] == ZERO                                           # the first tick of a load has dt 0
    zeros = [i for i, m in enumerate(moves) if i and m == ZERO]
    assert len(zeros) == 1 and moves[zeros[0] - 1]["x"] == 0          # only where the turn has ended (C1)
    assert_turn_ends_before_the_walk(moves[zeros[0] - 2:])
    assert_ramps(cmds)
    assert_no_gap(cmds, stops[0])
    turn_from = next(i for i, m in enumerate(moves) if abs(m["z"]) >= 0.3)   # the turn is under way
    assert 0 < max(m["x"] for m in moves[turn_from - 3:turn_from]) <= MIN_SPEED + 1e-9   # slowed for it
    assert all(m["x"] == 0 for m in moves[turn_from:] if abs(m["z"]) >= 0.3)  # on the spot
    assert abs(sim.x - 0.5) <= 0.06 and abs(sim.y - 0.5) <= 0.06
    assert abs(math.remainder(sim.yaw - math.pi / 2, 2 * math.pi)) < 0.1
    assert [e for e in ex.events if e.startswith("完了: ")] == [
        "完了: forward(0.5m)", "完了: turn_left(90度)", "完了: forward(0.5m)"]


def test_a_turn_loaded_after_a_forward_that_slowed_for_it_goes_on():
    """Corners, as the session drives it: flow is the turn; the forward ends without StopMove at
    its goal, slowly; the turn loaded in the same tick sends that velocity, then x falls to 0
    in a tick of acceleration at a time while z rises to the turn's."""
    sim, ex = standing_sim(), GridExecutor(EXEC_ACTIONS)
    cmds, t = flow_one(sim, ex, CORNER[0], into="turn_left")
    assert not [c for _, c in cmds if c.api_id == STOP_MOVE] and ex.rolling
    assert 0.5 - GRID_DONE_DIST - 0.03 <= sim.x <= 0.5 + 0.03
    last = [c.parameter for _, c in cmds if c.api_id == MOVE][-1]
    assert 0 < last["x"] <= MIN_SPEED + 1e-9                          # slowed down as for a stop
    ex.load([CORNER[1]])
    assert not ex.rolling
    more = []
    _tick(sim, ex, t, CLEAR, more)                                    # the same tick
    for k in range(1, 6):
        _tick(sim, ex, round(t + k * 0.1, 6), CLEAR, more)
    assert [c.api_id for _, c in more] == [MOVE] * 6
    assert more[0][1].parameter == last
    xs = [c.parameter["x"] for _, c in more]
    zs = [c.parameter["z"] for _, c in more]
    assert all(b <= a for a, b in zip(xs, xs[1:])) and xs[2] == 0     # down to 0 in two ticks
    assert zs[-1] >= 0.4 and all(b >= a for a, b in zip(zs, zs[1:]))
    assert_ramps(cmds + more)


@pytest.mark.parametrize("nxt", [GridStep("turn_left", 90.0, (1.0, 0.0), math.pi / 2),
                                 GridStep("forward", 0.5, (0.5, 0.5), math.pi / 2)],
                         ids=["turn_in_another_cell", "forward_other_heading"])
def test_a_step_after_a_corner_pass_that_does_not_continue_it_stops_first(nxt):
    """Corners: the forward slowed for a turn in its cell, but the step loaded is another (a jump
    changed the plan): StopMove in its first tick, then GRID_SETTLE_S, then from 0."""
    sim, ex = standing_sim(), GridExecutor(EXEC_ACTIONS)
    _, t = flow_one(sim, ex, CORNER[0], into="turn_left")
    ex.load([nxt])
    cmds = []
    _tick(sim, ex, t, CLEAR, cmds)
    assert [c.api_id for _, c in cmds] == [STOP_MOVE]
    assert [e for e in ex.events if e.startswith("M1: ")] == [
        f"M1: 続けないので止めます（{nxt.text(EXEC_ACTIONS)} は forward(0.5m) の続きではありません）"]
    for k in range(1, 10):
        _tick(sim, ex, round(t + k * 0.1, 6), CLEAR, cmds)
    later = [(tt, c) for tt, c in cmds if tt > t]
    assert min(tt for tt, _ in later) >= t + GRID_SETTLE_S - 1e-9
    first = later[0][1].parameter
    assert abs(first["x"]) <= ACC_X + 1e-9 and abs(first["z"]) <= ACC_Z + 1e-9


def test_a_forward_after_a_corner_pass_continues_the_walk():
    """The plan changed at the pass from the turn to a forward in the same heading: it goes on
    from the slow velocity (no stop; the ramp up to 0.3 is the usual one)."""
    sim, ex = standing_sim(), GridExecutor(EXEC_ACTIONS)
    cmds, t = flow_one(sim, ex, CORNER[0], into="turn_left")
    last = [c.parameter for _, c in cmds if c.api_id == MOVE][-1]
    ex.load([RUN[1]])
    more = []
    _tick(sim, ex, t, CLEAR, more)
    assert [c.api_id for _, c in more] == [MOVE] and more[0][1].parameter == last
    while ex.busy:
        t = round(t + 0.1, 6)
        _tick(sim, ex, t, CLEAR, more)
    assert [c.api_id for _, c in more].count(STOP_MOVE) == 1
    assert_ramps(cmds + more)
    assert abs(sim.x - 1.0) <= 0.06


def test_a_turn_does_not_flow_into_a_turn():
    """Only a forward flows into a turn: two turns in a row stop between them as before."""
    sim, ex = standing_sim(), GridExecutor(EXEC_ACTIONS)
    cmds = run(sim, ex, [GridStep("turn_left", 90.0, (0.0, 0.0), math.pi / 2),
                         GridStep("turn_left", 90.0, (0.0, 0.0), math.pi)])
    assert [c.api_id for _, c in cmds].count(STOP_MOVE) == 2
