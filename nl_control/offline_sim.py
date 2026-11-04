"""ロボットも ROS 2 も無い PC で、計画 → 実行 → 障害物で停止 → 再開 の流れを確かめる。

    python3 offline_sim.py

LLM の代わりに決まった JSON を返す偽の LLM を使い、Go2 の代わりに
「Move の速度で位置が変わるだけ」の簡単な模型を動かす。
1回目の LLM 出力はわざと間違え（伏せたまま前進）、監督器に却下されて聞き直す様子も見せる。
"""

import json
import math

import numpy as np

from go2_nl_control.actions import ACTIONS, LYING, MOVE, STOP_MOVE
from go2_nl_control.executor import ExecConfig, Executor
from go2_nl_control.perception import ObstacleMonitor, PerceptionConfig
from go2_nl_control.translator import Translator


class FakeLLM:
    """1回目は状態遷移が間違った計画、2回目に正しい計画を返す。"""

    def __init__(self):
        self.calls = []

    def __call__(self, messages, schema):
        self.calls.append(messages)
        if len(self.calls) == 1:
            return json.dumps({"steps": [{"action": "forward", "value": 1.0}]})
        return "```json\n" + json.dumps({"steps": [
            {"action": "stand", "value": 0},
            {"action": "forward", "value": 1.0},
            {"action": "turn_left", "value": 90},
            {"action": "hello", "value": 0},
        ]}) + "\n```"


class FakeGo2:
    """Move の速度どおりに位置と向きが変わるだけの模型（1 秒 Move が来なければ止まる）。"""

    def __init__(self):
        self.x = self.y = self.yaw = 0.0
        self.v = (0.0, 0.0, 0.0)
        self.v_time = -9.0
        self.log = []

    def receive(self, cmd, t):
        self.log.append((round(t, 1), cmd.api_id))
        if cmd.api_id == MOVE:
            p = cmd.parameter
            self.v, self.v_time = (p["x"], p["y"], p["z"]), t
        elif cmd.api_id == STOP_MOVE:
            self.v = (0.0, 0.0, 0.0)

    def step(self, t, dt):
        vx, vy, vz = self.v if t - self.v_time <= 1.0 else (0, 0, 0)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        self.x += (vx * c - vy * s) * dt
        self.y += (vx * s + vy * c) * dt
        self.yaw = (self.yaw + vz * dt + math.pi) % (2 * math.pi) - math.pi


def lidar_points(robot, obstacle_xy):
    """障害物（世界座標の点）を、ロボットから見た点群にする。無ければ遠くの壁だけ。"""
    pts = [[5.0, 0.0, 0.0]]
    if obstacle_xy is not None:
        dx, dy = obstacle_xy[0] - robot.x, obstacle_xy[1] - robot.y
        c, s = math.cos(-robot.yaw), math.sin(-robot.yaw)
        bx, by = dx * c - dy * s, dx * s + dy * c
        for k in range(10):  # 縦に並んだ 10 点（人の脚くらい）
            pts.append([bx - 0.28, by, -0.05 + 0.03 * k])
    return np.array(pts)


def run(text="立って1メートル進んで、左を向いて挨拶して", verbose=True):
    llm = FakeLLM()
    steps, log = Translator(llm).translate(text, LYING, ACTIONS)
    if verbose:
        print(f"指示: {text}")
        for line in log:
            print("  LLM", line)
    assert steps is not None

    ex = Executor(ACTIONS, ExecConfig())
    ex.posture = LYING
    ex.load(steps)
    mon = ObstacleMonitor(PerceptionConfig(lidar_x=0.28, lidar_z=0.0))
    go2 = FakeGo2()

    dt, t = 0.1, 0.0
    obstacle = None
    last_env = None
    timeline = []
    while ex.busy and t < 120:
        # 前進の途中（x≈0.3 m）で、0.6 m 先に人が立ち、3 秒後に立ち去る
        if obstacle is None and go2.x > 0.3 and "placed" not in timeline:
            obstacle, placed_at = (go2.x + 0.6, 0.0), t
            timeline.append("placed")
        if obstacle is not None and t - placed_at > 3.0:
            obstacle = None
        mon.update(lidar_points(go2, obstacle), t)
        env = mon.current(t)
        if env != last_env and verbose:
            print(f"  t={t:5.1f}s  前方: {env}" + (f" ({mon.nearest:.2f} m)" if mon.nearest else ""))
        last_env = env
        for c in ex.tick(t, env, (go2.x, go2.y, go2.yaw)):
            go2.receive(c, t)
        for e in ex.events:
            if verbose:
                print(f"  t={t:5.1f}s  {e}")
        ex.events.clear()
        go2.step(t, dt)
        t += dt
    if verbose:
        print(f"結果: 位置 ({go2.x:.2f}, {go2.y:.2f}) m, 向き {math.degrees(go2.yaw):.0f} 度, 姿勢 {ex.posture}")
    return go2, ex, llm


if __name__ == "__main__":
    run()
