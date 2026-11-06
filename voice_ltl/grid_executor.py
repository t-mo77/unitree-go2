"""The executor for plans on the grid (design §6.5, Rulings R11, R12, P7).

GridExecutor is the zip's Executor with another move: a GridStep carries the odom pose it must
reach (the centre of the next cell, or the yaw of the next heading), and each tick steers
towards it from the Go2's pose (closed loop), so errors do not add up from step to step.

- forward / backward: drive along the goal's heading; the speed falls with the distance left,
  and z holds that heading and pulls the Go2 back onto the line through the goal. |z| is capped
  in proportion to the forward speed (MOVE_TURN at full speed), so the Go2 does not turn while it
  starts or stops walking: in Gazebo, turning at a slow walk rocked it over (Task I).
- turn_left / turn_right: turn on the spot towards the goal yaw, slower near it.
- When the goal is reached: StopMove once, then nothing for GRID_SETTLE_S (the Go2 coasts),
  and only then is the step done, so the session snaps a Go2 that has stopped (R11).
- Everything else is the Executor's: gestures and posture changes, the holds (a blocked front,
  a lost sensor, no pose; they apply in the settle too), and the timeouts. A GridStep keeps
  Step's action and value (step_value), so its text() and the param names "distance" and
  "angle" are the ones the session and the holds use (R12).
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from go2_nl_control.actions import MOVE, ActionSpec
from go2_nl_control.executor import STOP, Command, ExecConfig, Executor, Pose, _ramp
from go2_nl_control.supervisor import Step

GRID_SETTLE_S = 0.5                    # [s] after StopMove before the step is done
GRID_DONE_DIST = 0.03                  # [m] a move is done this close to its goal (along its heading)
GRID_DONE_YAW = math.radians(3.0)      # [rad] a turn is done this close to its goal yaw
# P7. Task I tried turns at 0.3 rad/s and 0.4 m/s² in Gazebo: no better (turns: median roll 0.32
# and 2 falls in 27, against 0.24 and none in 12 at 0.5), so these stay.
GRID_EXEC_CONFIG = ExecConfig(speed=(0.3, 0.3, 0.5), acc=(0.6, 0.6, 1.0), block_timeout=15.0,
                              move_timeout_margin=2.5)

K_YAW = 1.5          # [1/s] yaw error -> z, turning on the spot
# Walking (Task I, Gazebo): the Go2 sways sideways after each StopMove (roll up to 0.6 rad, yaw
# ±0.15 rad, ±0.1 m, period about 1.5 s). With 1.5 and 1.0 here, z followed that sway (±0.25
# rad/s) and pumped it: roll 0.55-1.06 on forward after forward, and two falls. Gentle gains fix
# the heading over a few steps and leave the sway alone.
K_WALK_YAW = 0.6     # [1/s] yaw error -> z, walking
K_CROSS = 0.4        # [rad/(m·s)] offset across the line through the goal -> z, walking
# Forward speed clip(K_ALONG · distance left, MIN_SPEED, 0.3). In Gazebo the Go2 leaned over while
# walking slowly at the end of a move (0.08-0.14 m/s for about 1 s: roll 0.5-1.07, and a fall);
# 2.0 and 0.15 keep that end short (about 0.5 s) and the walk above the slow, unsteady gait (Task I).
K_ALONG = 2.0        # [1/s] distance left -> forward speed
MIN_SPEED = 0.15     # [m/s] the slowest forward / backward speed while a move is not done
MOVE_TURN = 0.15     # [rad/s] the largest |z| while walking at full speed (less when slower)
# [rad/s] the slowest turn while a turn is not done. Gazebo's unitree_guide controller stops
# stepping when |z| < 0.2 with no forward speed (StateTrotting::checkStepOrNot), so a slower end of
# a turn twists the body on planted feet (with a longer slow end, roll up to 0.9 in 15 turns). Task I.
MIN_TURN = 0.25


@dataclass
class GridStep(Step):
    """A Step with its goal in odom: the cell centre after a move, the heading's yaw after a
    turn (a move keeps the heading; a turn keeps the cell)."""

    goal_xy: Tuple[float, float] = (0.0, 0.0)
    goal_yaw: float = 0.0


def _clip(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class GridExecutor(Executor):
    def __init__(self, cat: Dict[str, ActionSpec], cfg: ExecConfig = GRID_EXEC_CONFIG):
        super().__init__(cat, cfg)

    def _reset_step(self):
        super()._reset_step()
        self.settle_until: Optional[float] = None   # set when the goal is reached (StopMove sent)

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
        if spec.param == "angle":
            if abs(yaw_err) <= GRID_DONE_YAW:
                return self._settle(now)
            target = (0.0, 0.0, math.copysign(_clip(K_YAW * abs(yaw_err), MIN_TURN, self.cfg.speed[2]), yaw_err))
            ideal = math.radians(step.value) / self.cfg.speed[2]
        else:
            sign = 1.0 if spec.velocity[0] >= 0 else -1.0    # forward +1, backward -1
            c, s = math.cos(step.goal_yaw), math.sin(step.goal_yaw)
            dx, dy = step.goal_xy[0] - x, step.goal_xy[1] - y
            along = sign * (dx * c + dy * s)                 # left to go, in the direction of travel
            cross = -dx * s + dy * c                         # the goal's offset to the left of the Go2
            if along <= GRID_DONE_DIST:
                return self._settle(now)
            vx = sign * _clip(K_ALONG * along, MIN_SPEED, self.cfg.speed[0])
            # the turn this tick may use: in proportion to the forward speed this tick will have
            turn_cap = MOVE_TURN * min(1.0, abs(_ramp(self.vel[0], vx, self.cfg.acc[0] * dt)) / self.cfg.speed[0])
            target = (vx, 0.0, _clip(K_WALK_YAW * yaw_err + sign * K_CROSS * cross, -turn_cap, turn_cap))
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
