"""監督器：計画の検証（実行前）と、実行中の許可判定（毎周期）。

Cay 先生の講義の考え方に沿っている。
- 可制御イベント：計画の各動作（立つ・進む・おすわり…）。監督器が禁止できる。
- 不可制御イベント：障害物が現れる・消える、センサーが途切れる。監督器には止められない。
  そこで「障害物がある間は前へ進む動作を禁止する」という形で、可制御イベントの側を絞る。

LLM の出力は、ここを通らない限り実行されない。
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from .actions import LIMITS, MAX_STEPS, ActionSpec

# 環境の状態（perception が決める）
CLEAR = "CLEAR"              # 前方に障害物なし
BLOCKED = "BLOCKED"          # 前方に障害物あり
SENSOR_LOST = "SENSOR_LOST"  # LiDAR か /sportmodestate が途切れている


@dataclass
class Step:
    action: str
    value: float = 0.0   # distance [m] / angle [度] / seconds [s]（param の無い動作では 0）

    def text(self, cat: Dict[str, ActionSpec]) -> str:
        p = cat[self.action].param
        unit = {"distance": "m", "angle": "度", "seconds": "秒"}.get(p, "")
        return f"{self.action}({self.value:g}{unit})" if p else self.action


def check_plan(raw: object, start_state: str,
               cat: Dict[str, ActionSpec]) -> Tuple[Optional[List[Step]], str]:
    """LLM が返した計画（JSON を読み込んだもの）を検証する。

    合格なら (steps, "")、不合格なら (None, 理由)。理由は LLM への聞き直しに使う。
    範囲外の値を丸めて直したりはしない（LLM の誤りを隠さないため）。
    """
    if not isinstance(raw, dict) or not isinstance(raw.get("steps"), list):
        return None, 'トップレベルは {"steps": [...]} の形にしてください'
    items = raw["steps"]
    if not items:
        return None, "steps が空です"
    if len(items) > MAX_STEPS:
        return None, f"steps は {MAX_STEPS} 個以下にしてください（{len(items)} 個ありました）"

    steps: List[Step] = []
    state = start_state
    for i, it in enumerate(items, 1):
        if not isinstance(it, dict):
            return None, f"{i}番目の要素がオブジェクトではありません"
        name = it.get("action")
        spec = cat.get(name)
        if spec is None:
            return None, f"{i}番目の action '{name}' は語彙にありません。使えるのは {sorted(cat)} です"

        step = Step(name)
        if spec.param:
            v = it.get("value")
            if not isinstance(v, (int, float)) or isinstance(v, bool):
                return None, f"{i}番目の {name} には数値の value（{spec.param}）が必要です"
            lo, hi = LIMITS[spec.param]
            if not lo <= float(v) <= hi:
                return None, f"{i}番目の {name} の value={v} は {spec.param} の範囲 [{lo}, {hi}] の外です"
            step.value = float(v)

        # 状態遷移の検証：この順番では実行できない計画を落とす
        if state not in spec.transitions:
            return None, (f"{i}番目の {name} は状態 {state} では実行できません"
                          f"（実行できる状態: {sorted(spec.transitions)}）")
        state = spec.transitions[state]
        steps.append(step)
    return steps, ""


def runtime_permits(spec: ActionSpec, env: str) -> Tuple[bool, str]:
    """今その動作をしてよいか。False なら実行器は保留（一時停止）する。"""
    moving = spec.param in ("distance", "angle")
    if env == SENSOR_LOST and moving:
        return False, "センサー（LiDAR か /sportmodestate）が途切れています"
    if env == BLOCKED and spec.forward:
        return False, "前方に障害物があります"
    return True, ""


def state_after(steps: List[Step], start: str, cat: Dict[str, ActionSpec]) -> str:
    s = start
    for st in steps:
        s = cat[st.action].transitions.get(s, s)
    return s
