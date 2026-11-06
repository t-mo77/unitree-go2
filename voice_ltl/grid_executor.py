"""The executor for plans on the grid (design §6.5, Rulings R11, R12, P7).

GridExecutor is the zip's Executor with another move: a GridStep carries the odom pose it must
reach (the centre of the next cell, or the yaw of the next heading), and each tick steers
towards it from the Go2's pose (closed loop), so errors do not add up from step to step.

- forward / backward: drive along the goal's heading; the speed falls with the distance left,
  and z holds that heading and pulls the Go2 back onto the line through the goal.
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
            target = (sign * _clip(K_ALONG * along, MIN_SPEED, self.cfg.speed[0]), 0.0,
                      _clip(K_YAW * yaw_err + sign * K_CROSS * cross, -MOVE_TURN, MOVE_TURN))
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
