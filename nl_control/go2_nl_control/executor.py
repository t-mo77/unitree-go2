"""実行器：計画を1つずつ実行する。ROS に依存しない純粋なロジック。

メインループから 10 Hz で tick() を呼ぶと、その時点で Go2 に送るべき指令の列を返す。
- 同時に実行する動作は常に1つだけ。
- 移動は「決まった時間」ではなく、/sportmodestate の位置と向きを見て決まった距離・角度で止める
  （demo_script.py の move_until と同じ考え方）。
- 前方に障害物があれば一時停止（保留）し、なくなれば続きから動く。
  保留が block_timeout を超えたら計画を中止する。
- Move は「最後の指令が1秒続く・フィルタされない」仕様なので、毎周期送り直し、加速を制限する。
"""

import json
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from .actions import MOVE, STOP_MOVE, UNKNOWN, ActionSpec
from .supervisor import Step, runtime_permits

Pose = Tuple[float, float, float]  # (x [m], y [m], yaw [rad])


@dataclass
class Command:
    api_id: int
    parameter: Optional[dict] = None

    def __str__(self):
        return f"{self.api_id}{'' if self.parameter is None else ' ' + json.dumps(self.parameter)}"


STOP = Command(STOP_MOVE)


@dataclass
class ExecConfig:
    speed: Tuple[float, float, float] = (0.3, 0.3, 0.8)  # [前後 m/s, 左右 m/s, 旋回 rad/s]
    acc: Tuple[float, float, float] = (0.6, 0.6, 2.0)    # 加速度の上限 [m/s^2, m/s^2, rad/s^2]
    block_timeout: float = 15.0   # 保留がこれ以上続いたら計画を中止 [s]
    move_timeout_margin: float = 2.5  # 移動が 理論時間×これ+5秒 を超えたら中止


def _ramp(cur: float, target: float, step: float) -> float:
    return min(target, cur + step) if target > cur else max(target, cur - step)


class Executor:
    def __init__(self, cat: Dict[str, ActionSpec], cfg: Optional[ExecConfig] = None):
        self.cat = cat
        self.cfg = cfg or ExecConfig()
        self.steps: List[Step] = []
        self.idx = 0
        self.events: List[str] = []   # 画面に出す報告（メインループが取り出して表示する）
        self.posture = UNKNOWN        # 今の姿勢（完了した動作から求める）
        self._last_now: Optional[float] = None
        self._reset_step()

    # ---------------- 状態 ----------------
    @property
    def busy(self) -> bool:
        return self.idx < len(self.steps)

    @property
    def current(self) -> Optional[Step]:
        return self.steps[self.idx] if self.busy else None

    def _reset_step(self):
        self.started = False
        self.seq_i = 0           # sequence の何番目を送ったか
        self.wait_left = 0.0     # 次へ進むまでの残り秒数
        self.paused_since: Optional[float] = None
        self.vel = [0.0, 0.0, 0.0]
        self.start_xy: Optional[Tuple[float, float]] = None
        self.last_yaw = 0.0
        self.turned = 0.0        # 回った角度の累計 [rad]
        self.active = 0.0        # 移動で実際に動いていた時間 [s]

    # ---------------- 操作 ----------------
    def load(self, steps: List[Step]):
        self.steps, self.idx = list(steps), 0
        self._reset_step()
        self.events.append("計画: " + " → ".join(s.text(self.cat) for s in steps))

    def cancel(self, reason: str) -> List[Command]:
        if self.busy:
            self.events.append(f"中止: {reason}")
            spec = self.cat[self.steps[self.idx].action]
            if self.started and spec.sequence:
                self.posture = UNKNOWN  # 姿勢変更の途中で止めたので、今の姿勢はわからない
        self.steps, self.idx = [], 0
        self._reset_step()
        return [STOP]

    def _advance(self):
        spec = self.cat[self.steps[self.idx].action]
        self.posture = spec.transitions.get(self.posture, self.posture)
        self.events.append(f"完了: {self.steps[self.idx].text(self.cat)}")
        self.idx += 1
        self._reset_step()
        if not self.busy:
            self.events.append("計画をすべて終えました")

    # ---------------- 1 周期 ----------------
    def tick(self, now: float, env: str, pose: Optional[Pose]) -> List[Command]:
        dt = 0.0 if self._last_now is None else min(0.5, max(0.0, now - self._last_now))
        self._last_now = now
        return self._tick(now, dt, env, pose)

    def _tick(self, now: float, dt: float, env: str, pose: Optional[Pose]) -> List[Command]:
        step = self.current
        if step is None:
            return []
        spec = self.cat[step.action]
        moving = spec.param in ("distance", "angle")

        ok, why = runtime_permits(spec, env)
        if moving and pose is None:
            ok, why = False, "/sportmodestate（位置）が届いていません"
        # 始まった姿勢変更・ジェスチャーは途中で止めない。移動と、まだ始まっていない動作だけ保留する
        if not ok and (moving or not self.started):
            out: List[Command] = []
            if self.paused_since is None:
                self.paused_since = now
                self.events.append(f"一時停止（{step.text(self.cat)}）: {why}")
                out.append(STOP)
                self.vel = [0.0, 0.0, 0.0]
            elif now - self.paused_since > self.cfg.block_timeout:
                return self.cancel(f"{self.cfg.block_timeout:g} 秒待っても {why}")
            return out
        if self.paused_since is not None:
            self.events.append(f"再開（{step.text(self.cat)}）")
            self.paused_since = None

        if moving:
            return self._tick_move(step, spec, dt, pose, now, env)

        if spec.param == "seconds":  # wait
            if not self.started:
                self.started, self.wait_left = True, step.value
            self.wait_left -= dt
            if self.wait_left <= 0:
                self._advance()
                return self._tick(now, 0.0, env, pose)
            return []

        # 姿勢変更・ジェスチャー：sequence の指令を順に送り、それぞれ決まった秒数待つ
        self.started = True
        out = []
        self.wait_left -= dt
        while self.wait_left <= 0:
            if self.seq_i >= len(spec.sequence):
                self._advance()
                return out + self._tick(now, 0.0, env, pose)
            api, wait = spec.sequence[self.seq_i]
            out.append(Command(api))
            self.seq_i += 1
            self.wait_left = wait
        return out

    def _tick_move(self, step: Step, spec: ActionSpec, dt: float, pose: Pose,
                   now: float, env: str) -> List[Command]:
        x, y, yaw = pose
        if not self.started:
            self.started = True
            self.start_xy, self.last_yaw, self.turned = (x, y), yaw, 0.0
        # yaw は ±180° で飛ぶので、前回からの差を ±180° に直して足していく
        self.turned += (yaw - self.last_yaw + math.pi) % (2 * math.pi) - math.pi
        self.last_yaw = yaw

        if spec.param == "distance":
            done = math.hypot(x - self.start_xy[0], y - self.start_xy[1]) >= step.value
            ideal = step.value / max(self.cfg.speed[0], self.cfg.speed[1])
        else:
            done = abs(math.degrees(self.turned)) >= step.value
            ideal = math.radians(step.value) / self.cfg.speed[2]
        if done:
            self._advance()
            return [STOP] + self._tick(now, 0.0, env, pose)

        self.active += dt
        if self.active > ideal * self.cfg.move_timeout_margin + 5.0:
            return self.cancel(f"{step.text(self.cat)} が {self.active:.0f} 秒たっても終わりません")

        target = [d * s for d, s in zip(spec.velocity, self.cfg.speed)]
        self.vel = [_ramp(v, t, a * dt) for v, t, a in zip(self.vel, target, self.cfg.acc)]
        return [Command(MOVE, {"x": round(self.vel[0], 3), "y": round(self.vel[1], 3),
                               "z": round(self.vel[2], 3)})]
