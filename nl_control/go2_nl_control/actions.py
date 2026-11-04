"""動作カタログ：LLM が使ってよい語彙（Vocabulary）と、Go2 の api_id の対応。

api_id は公式 unitree_sdk2_python の go2/sport/sport_api.py と同じ番号。
ここに無い動作は、LLM が出しても check_plan() で却下される。
宙返り・逆立ち・二足歩行・ジャンプなど危ない動作は、わざと入れていない。
"""

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

# ---- /api/sport/request の api_id（公式 sport_api.py より） ----
BALANCE_STAND = 1002  # 歩ける立ち方にする（StandUp のあとに送る）
STOP_MOVE = 1003      # 移動を止める
STAND_UP = 1004       # 立つ（関節を固めた立ち方。このままでは歩けない）
STAND_DOWN = 1005     # 伏せる
MOVE = 1008           # 速度で動く。parameter = {"x": 前後 [m/s], "y": 左右 [m/s], "z": 旋回 [rad/s]}
SIT = 1009            # 座る（急に動くので後ろを空ける）
RISE_SIT = 1010       # 座った状態から立ち上がる
HELLO = 1016          # 挨拶（前足を振る）
STRETCH = 1017        # 伸び
CONTENT = 1020        # 喜ぶ
DANCE1 = 1022         # ダンス
SCRAPE = 1029         # おじぎ
HEART = 1036          # 前足でハートを描く

# ---- 姿勢の状態（監督器の状態） ----
LYING = "LYING"        # 伏せ・脱力
STANDING = "STANDING"  # 歩ける立ち方
SITTING = "SITTING"    # おすわり
UNKNOWN = "UNKNOWN"    # 状態がわからない


@dataclass(frozen=True)
class ActionSpec:
    name: str
    description: str                       # LLM に見せる説明
    transitions: Dict[str, str]            # 出発状態 → 到着状態（ここに無い状態からは実行できない）
    sequence: Tuple[Tuple[int, float], ...] = ()  # 送る api_id と、送った後に待つ秒数の列
    param: Optional[str] = None            # "distance" [m] / "angle" [度] / "seconds" [s]
    velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # 移動系：Move の向き（+1/-1）
    forward: bool = False                  # 体が前へ出る動作（前方に障害物があると保留）


# 歩ける姿勢でだけできるジェスチャー
_G = {STANDING: STANDING}

ACTIONS: Dict[str, ActionSpec] = {
    "stand": ActionSpec(
        "stand", "立ち上がって歩ける姿勢になる（伏せ・不明な状態から）",
        {LYING: STANDING, UNKNOWN: STANDING},
        sequence=((STAND_UP, 3.0), (BALANCE_STAND, 1.0))),
    "lie_down": ActionSpec(
        "lie_down", "伏せる", {STANDING: LYING}, sequence=((STAND_DOWN, 3.0),)),
    "sit": ActionSpec(
        "sit", "おすわりする", {STANDING: SITTING}, sequence=((SIT, 5.0),)),
    "rise_sit": ActionSpec(
        "rise_sit", "おすわりから立ち上がる", {SITTING: STANDING}, sequence=((RISE_SIT, 3.0),)),

    "forward": ActionSpec("forward", "distance [m] だけ前へ進む", _G,
                          param="distance", velocity=(1, 0, 0), forward=True),
    "backward": ActionSpec("backward", "distance [m] だけ後ろへ下がる", _G,
                           param="distance", velocity=(-1, 0, 0)),
    "left": ActionSpec("left", "向きを変えずに distance [m] だけ左へ横歩きする", _G,
                       param="distance", velocity=(0, 1, 0)),
    "right": ActionSpec("right", "向きを変えずに distance [m] だけ右へ横歩きする", _G,
                        param="distance", velocity=(0, -1, 0)),
    "turn_left": ActionSpec("turn_left", "その場で angle [度] だけ左回りに向きを変える", _G,
                            param="angle", velocity=(0, 0, 1)),
    "turn_right": ActionSpec("turn_right", "その場で angle [度] だけ右回りに向きを変える", _G,
                             param="angle", velocity=(0, 0, -1)),

    "wait": ActionSpec("wait", "何もせず seconds [s] 待つ",
                       {STANDING: STANDING, SITTING: SITTING, LYING: LYING}, param="seconds"),

    "hello": ActionSpec("hello", "前足を振って挨拶する", _G, sequence=((HELLO, 5.0),)),
    "stretch": ActionSpec("stretch", "伸びをする", _G, sequence=((STRETCH, 5.0),)),
    "content": ActionSpec("content", "喜びを表す", _G, sequence=((CONTENT, 4.0),)),
    "heart": ActionSpec("heart", "前足でハートを描く", _G, sequence=((HEART, 5.0),)),
    "scrape": ActionSpec("scrape", "おじぎで挨拶する", _G, sequence=((SCRAPE, 4.0),)),
    "dance": ActionSpec("dance", "ダンスをする", _G, sequence=((DANCE1, 12.0),)),
}

# パラメータの範囲（範囲外は丸めずに却下する）
LIMITS = {
    "distance": (0.05, 3.0),   # [m]
    "angle": (5.0, 360.0),     # [度]
    "seconds": (0.5, 30.0),    # [s]
}
MAX_STEPS = 10

# /sportmodestate の error_code（公式 Motion Services Interface V2.0 の状態表）→ 姿勢
_ERROR_CODE_TO_STATE = {
    1001: LYING,      # Damping
    1004: LYING,      # Crouch
    2006: LYING,      # Crouch
    1007: SITTING,    # Sit
    100: STANDING,    # Agile
    1013: STANDING,   # Balance Standing
    1015: STANDING,   # Regular Walking
    1016: STANDING,   # Regular Running
    1017: STANDING,   # Regular Endurance
    2010: STANDING,   # Classic
    # 1002（Standing Lock）は StandUp 直後の「歩けない立ち方」なので UNKNOWN 扱い
    #  → stand（StandUp → BalanceStand）からやり直させる
}


def state_from_error_code(code: Optional[int]) -> str:
    if code is None:
        return UNKNOWN
    return _ERROR_CODE_TO_STATE.get(int(code), UNKNOWN)
