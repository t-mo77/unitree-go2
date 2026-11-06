"""The executor for plans on the grid (design §6.5, Rulings R11, R12, P7, M1).

GridExecutor is the zip's Executor with another move: a GridStep carries the odom pose it must
reach (the centre of the next cell, or the yaw of the next heading), and each tick steers
towards it from the Go2's pose (closed loop), so errors do not add up from step to step.

- forward / backward: drive along the goal's heading; the speed falls with the distance left,
  and z holds that heading and pulls the Go2 back onto the line through the goal.
- turn_left / turn_right: turn on the spot towards the goal yaw, slower near it.
- When the goal is reached: StopMove once, then nothing for GRID_SETTLE_S (the Go2 coasts),
  and only then is the step done, so the session snaps a Go2 that has stopped (R11).
- M1 (amends R11): a step whose next step is a forward in the heading it ends in (a forward in
  a straight run, or a turn) flows on. It does not slow down for its goal, and is done as it
  gets there (a forward as it passes the cell centre) with no StopMove and no settle; the Go2
  keeps the velocity it has. The next step is either in this executor's list or, as the
  session loads one step at a time, told by `flow` (the plan's next action). A forward in that
  heading loaded next goes on from that velocity (no ramp from 0); any other step gets
  StopMove in its first tick and starts GRID_SETTLE_S later. halt() stops at once when nothing
  is loaded next. So a straight run stops once, at its end, and a turn goes straight into the
  forward after it.
- Corners (after the M1 Gazebo measurement, 2026-11-06): a forward whose next step is a turn in
  the cell it ends in flows into that turn too. It slows down for its goal as before (about
  0.08 m/s at the end), and is done there with no StopMove and no settle; the turn goes on from
  that velocity: x ramps down to 0 while z ramps up to the turn's. In Gazebo every fall at a
  turn came 0.6-0.9 s into a turn started after a straight run's StopMove; a turn that flows
  from the walk never stops the controller's stepping. So a plan of forwards and single turns
  stops once, at its end. A turn after a forward that ran through its goal at full speed (the
  plan changed at the pass) still gets StopMove first.
- load() forgets the last tick's time, so each step's ramp starts from 0 however long the
  executor went unticked before it (not for a step that goes on from one that flowed on).
- Everything else is the Executor's: gestures and posture changes, the holds (a blocked front,
  a lost sensor, no pose; they apply in the settle too, and stop a flowing walk at once), and
  the timeouts. A GridStep keeps Step's action and value (step_value), so its text() and the
  param names "distance" and "angle" are the ones the session and the holds use (R12).
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from go2_nl_control.actions import MOVE, ActionSpec
from go2_nl_control.executor import STOP, Command, ExecConfig, Executor, Pose, _ramp
from go2_nl_control.supervisor import Step

# [s] after StopMove before the step is done; also before a step after a walk that flowed on
# and was not continued (M1). M1: 0.2 s (two 10 Hz ticks) replaces R11's 0.5 s. The wait is only
# for the snap, and the snap needs little: a move ends 0.03 m from its goal at about 0.08 m/s
# and coasts 0.05-0.10 m (design §6.5), well inside the 0.25 m to the next cell and the
# 0.3 m / 30° tolerance; the turn after it ignores the position and the forward after that
# steers to its cell centre. Two ticks let the StopMove reach the controller (the Gazebo bridge
# runs at 20 Hz) and fresh poses come (odometry at 25 Hz). Waiting longer at zero velocity does
# not calm the sway after a stop: docs/reference/gazebo_e2e.md §4 (a 1.0 s wait made it worse;
# the zip's Executor, with no wait, never passed roll 0.4 turning after a forward) and
# gazebo_objects_e2e.md (falls 0.7 s and 1.7 s after a StopMove).
GRID_SETTLE_S = 0.2
GRID_DONE_DIST = 0.03                  # [m] a move is done this close to its goal (along its heading)
GRID_DONE_YAW = math.radians(3.0)      # [rad] a turn is done this close to its goal yaw
GRID_EXEC_CONFIG = ExecConfig(speed=(0.3, 0.3, 0.5), acc=(0.6, 0.6, 1.0), block_timeout=15.0,
                              move_timeout_margin=2.5)

# P7. Task I tried other gains in Gazebo (gentler steering while walking, a faster end of a move,
# a 0.25 rad/s turn floor, 0.3 rad/s turns): none completed a lap, so P7 stays
# (docs/reference/gazebo_objects_e2e.md). The gazebo bridge ramps a held first Move in instead.
K_YAW = 1.5          # [1/s] yaw error -> z
K_CROSS = 1.0        # [rad/(m·s)] offset across the line through the goal -> z
K_ALONG = 1.2        # [1/s] distance left -> forward speed
MIN_SPEED = 0.08     # [m/s] the slowest forward / backward speed while a move is not done
MOVE_TURN = 0.3      # [rad/s] the largest |z| while walking
MIN_TURN = 0.15      # [rad/s] the slowest turn while a turn is not done

# M1: the steps that flow into a forward in the heading they end in
FLOWS_INTO_FORWARD = ("forward", "turn_left", "turn_right")
TURNS = ("turn_left", "turn_right")    # corners: a forward flows into these in the cell it ends in


def flows_on(action: str, nxt: Optional[str]) -> bool:
    """M1 and corners: `action` goes on into `nxt` without a stop (when the geometry fits: a
    forward in the heading it ends in, a turn in the cell a forward ends in)."""
    return ((nxt == "forward" and action in FLOWS_INTO_FORWARD)
            or (nxt in TURNS and action == "forward"))


@dataclass
class GridStep(Step):
    """A Step with its goal in odom: the cell centre after a move, the heading's yaw after a
    turn (a move keeps the heading; a turn keeps the cell)."""

    goal_xy: Tuple[float, float] = (0.0, 0.0)
    goal_yaw: float = 0.0


def _clip(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def _continues(prev: Step, nxt: Optional[Step]) -> bool:
    """M1: `nxt` goes on from `prev` without a stop: a forward in the heading `prev` ends in, or
    (corners) a turn in the cell the forward `prev` ends in."""
    if not (isinstance(prev, GridStep) and isinstance(nxt, GridStep) and flows_on(prev.action, nxt.action)):
        return False
    if nxt.action == "forward":
        return abs(math.remainder(nxt.goal_yaw - prev.goal_yaw, 2 * math.pi)) < 1e-6
    return math.dist(prev.goal_xy, nxt.goal_xy) < 1e-6


class GridExecutor(Executor):
    def __init__(self, cat: Dict[str, ActionSpec], cfg: ExecConfig = GRID_EXEC_CONFIG):
        super().__init__(cat, cfg)
        # M1. Set by the session each tick: the action that follows the step loaded last without
        # a stop (the plan's next action from where it lands, sensors as they are), or None.
        self.flow: Optional[str] = None
        self.rolling = False      # the last step flowed on: the Go2 still moves, nothing loaded yet
        # that step, its velocity, and the action it flowed into
        self._rolled: Optional[Tuple[Step, List[float], str]] = None
        self._halt_due = False    # the step loaded does not continue it: StopMove first
        self._quiet_until: Optional[float] = None   # the step loaded starts no earlier

    def load(self, steps: List[Step]):
        """The session ticks the executor only while a step is loaded, so the last tick can be
        long ago (a second instruction, the R1-amendment wait, a posture settle). The first tick
        after load() then has dt 0, and the ramp starts from 0 (design §6.5, P7): a stale dt,
        capped at 0.5 s, would ramp to the full speed in one tick. A forward that goes on from a
        step that flowed on (M1) keeps that step's velocity and clock instead."""
        super().load(steps)
        if not self._take_roll():
            self._last_now = None

    def cancel(self, reason: str) -> List[Command]:
        self.rolling, self._rolled, self._halt_due, self._quiet_until = False, None, False, None
        return super().cancel(reason)

    def halt(self, now: float) -> List[Command]:
        """M1: a step flowed on and no step continues it (the supervisor waits or is done, the
        posture is not settled, the snap must wait): StopMove now, and the next step loaded
        starts GRID_SETTLE_S later. Nothing if no step flowed on."""
        if not self.rolling:
            return []
        self.rolling, self._rolled = False, None
        self._quiet_until = now + GRID_SETTLE_S
        return [STOP]

    def _reset_step(self):
        super()._reset_step()
        self.settle_until: Optional[float] = None   # set when the goal is reached (StopMove sent)

    def _take_roll(self) -> bool:
        """The current step comes right after one that flowed on (M1): a forward in its heading,
        or a turn in its cell after a forward that slowed down for it (corners), goes on with
        its velocity (True); any other step owes a StopMove first."""
        if not self.rolling:
            return False
        prev, vel, into = self._rolled
        self.rolling, self._rolled = False, None
        cur = self.current
        if _continues(prev, cur) and (cur.action == "forward" or into in TURNS):
            self.vel = list(vel)
            return True
        self._halt_due = True
        return False

    def _flows_into(self, step: Step) -> Optional[str]:
        """M1: the action `step` flows into (the next step of the list, or, for the last one,
        `flow`), or None when it stops at its goal."""
        if self.idx + 1 < len(self.steps):
            nxt = self.steps[self.idx + 1]
            return nxt.action if _continues(step, nxt) else None
        if isinstance(step, GridStep) and flows_on(step.action, self.flow):
            return self.flow
        return None

    def _tick(self, now: float, dt: float, env: str, pose: Optional[Pose]) -> List[Command]:
        if self._halt_due and self.busy:
            self._halt_due = False
            self.vel = [0.0, 0.0, 0.0]
            self._quiet_until = now + GRID_SETTLE_S
            return [STOP]
        if self._quiet_until is not None and self.busy:
            if now < self._quiet_until - 1e-9:
                return []
            self._quiet_until = None
        return super()._tick(now, dt, env, pose)

    def _tick_move(self, step: Step, spec: ActionSpec, dt: float, pose: Pose,
                   now: float, env: str) -> List[Command]:
        if not isinstance(step, GridStep):   # a plain Step moves as the Executor moves it
            return super()._tick_move(step, spec, dt, pose, now, env)
        self.started = True
        if self.settle_until is not None:
            if now < self.settle_until - 1e-9:
                return []
            self._advance()
            return self._tick(now, 0.0, env, pose)
        x, y, yaw = pose
        yaw_err = math.remainder(step.goal_yaw - yaw, 2 * math.pi)   # SimGo2's yaw is not wrapped
        into = self._flows_into(step)
        if spec.param == "angle":
            if abs(yaw_err) <= GRID_DONE_YAW:
                return self._pass(step, into, now, env, pose) if into else self._settle(now)
            target = (0.0, 0.0, math.copysign(_clip(K_YAW * abs(yaw_err), MIN_TURN, self.cfg.speed[2]), yaw_err))
            ideal = math.radians(step.value) / self.cfg.speed[2]
        else:
            sign = 1.0 if spec.velocity[0] >= 0 else -1.0    # forward +1, backward -1
            c, s = math.cos(step.goal_yaw), math.sin(step.goal_yaw)
            dx, dy = step.goal_xy[0] - x, step.goal_xy[1] - y
            along = sign * (dx * c + dy * s)                 # left to go, in the direction of travel
            cross = -dx * s + dy * c                         # the goal's offset to the left of the Go2
            if along <= GRID_DONE_DIST:                      # there, or past it
                return self._pass(step, into, now, env, pose) if into else self._settle(now)
            # a forward that flows into a forward does not slow down for a goal it walks through;
            # one that flows into a turn slows down as one that stops (the turn is on the spot)
            speed = (self.cfg.speed[0] if into == "forward"
                     else _clip(K_ALONG * along, MIN_SPEED, self.cfg.speed[0]))
            target = (sign * speed, 0.0, _clip(K_YAW * yaw_err + sign * K_CROSS * cross, -MOVE_TURN, MOVE_TURN))
            ideal = step.value / max(self.cfg.speed[0], self.cfg.speed[1])

        self.active += dt
        if self.active > ideal * self.cfg.move_timeout_margin + 5.0:
            return self.cancel(f"{step.text(self.cat)} が {self.active:.0f} 秒たっても終わりません")
        self.vel = [_ramp(v, t, a * dt) for v, t, a in zip(self.vel, target, self.cfg.acc)]
        return [Command(MOVE, {"x": round(self.vel[0], 3), "y": round(self.vel[1], 3),
                               "z": round(self.vel[2], 3)})]

    def _settle(self, now: float) -> List[Command]:
        self.settle_until = now + GRID_SETTLE_S
        self.vel = [0.0, 0.0, 0.0]
        return [STOP]

    def _pass(self, step: Step, into: str, now: float, env: str, pose: Pose) -> List[Command]:
        """M1: the goal of a step that flows into `into` is reached: done now, no StopMove, the
        velocity kept for the next step (the next of the list at once; else the one the
        session loads)."""
        vel = list(self.vel)
        self._advance()
        self.rolling, self._rolled = True, (step, vel, into)
        if self.busy:
            self._take_roll()
            return self._tick(now, 0.0, env, pose)
        return []
