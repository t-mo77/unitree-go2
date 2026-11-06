"""物（箱・椅子）の共通部品：物の行、凍結した Scene、命題名、文言、/go2/objects の JSON。

座標はすべて odom（x 前、y 左、REP-103）。標準ライブラリだけを使う
（Jazzy の Gazebo ブリッジもこのモジュールを読み込むが、そこには pitct も flloat もない）。
受け取った行の検査（R21）は parse_rows / parse の一か所だけで行う。
"""

from __future__ import annotations

import math
import re
from collections.abc import Iterable, Sequence
from dataclasses import dataclass

Pose = tuple[float, float, float]                       # (x [m], y [m], yaw [rad]), odom

OBJECTS_TOPIC = "/go2/objects"
FRAME = "odom"
OBJECTS_HZ = 2.0
OBJECTS_STALE_S = 2.0

LABEL_JA = {"box": "箱", "chair": "椅子"}
LABEL_WORDS = {"box": ("箱", "ボックス", "段ボール"), "chair": ("椅子", "いす", "イス", "チェア")}
ID_RE = re.compile(r"(box|chair)_[1-9][0-9]?")                                  # always fullmatch
PROP_RE = re.compile(r"(near|around_cw|around)_((?:box|chair)_[1-9][0-9]?)")    # always fullmatch
KEYWORDS = ("F", "G", "X", "U", "last")                                          # == grammar.KEYWORDS
_NAME = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")

DEFAULT_SIZE = (0.5, 0.5)
MAX_SIZE = 3.0
MAX_PROMPT_OBJECTS = 8
MAX_OBJECT_DISTANCE = 6.0

CAMERA_IN_BODY = (0.32715, -0.00003, 0.04297)
CAMERA_HFOV = 2.094
CAMERA_NEAR = 0.1
CAMERA_FAR = 8.0
CAMERA_HZ = 5.0

MSG_TOO_SPREAD = "物どうしが離れすぎていて、一度には計画できません"
MSG_NO_POSE = "Go2 の位置がわからないので、物を使う指示はできません"
MSG_POSITION_DRIFT = "位置がずれたので止めました。"
NO_PATH_REASON = "（物のまわりに通れる道がありません）"

_NOT_VISIBLE = "（いまは見えていません）"
_DIRECTION_WORDS = ("前", "左前", "左", "左後ろ", "後ろ", "右後ろ", "右", "右前")   # k = 0..7, 45° each


@dataclass(frozen=True)
class Obj:
    id: str
    label: str
    x: float
    y: float
    size: tuple[float, float] = DEFAULT_SIZE    # (extent along odom x, extent along odom y)
    visible: bool = True
    seen: float | None = None                   # the sender's clock; never compared with the app's

    @property
    def name(self) -> str:
        return LABEL_JA[self.label]

    @property
    def radius(self) -> float:
        return math.hypot(*self.size) / 2

    def to_row(self) -> dict:
        return {"id": self.id, "label": self.label, "x": _mm(self.x), "y": _mm(self.y),
                "size": [_mm(v) for v in self.size],
                "seen": None if self.seen is None else _mm(self.seen), "visible": self.visible}


DEFAULT_LAYOUT: tuple[Obj, ...] = (Obj("box_1", "box", 2.5, 1.5, (0.5, 0.5)),
                                   Obj("chair_1", "chair", 2.5, -1.5, (0.45, 0.45)))


@dataclass(frozen=True)
class Scene:
    """指示を受けた時点で凍結した物の一覧と Go2 の位置。"""

    objects: tuple[Obj, ...] = ()
    pose: Pose | None = None

    def listed(self) -> tuple[Obj, ...]:
        """プロンプト・語彙・検査に載せる物：位置から近い順に 8 個（位置がなければ id 順に 8 個）。
        id はラベルの中で番号の順（box_2 は box_10 より前）。"""
        if self.pose is None:
            ordered = sorted(self.objects, key=lambda o: _id_key(o.id))
        else:
            px, py = self.pose[0], self.pose[1]
            ordered = sorted(self.objects, key=lambda o: (math.hypot(o.x - px, o.y - py), _id_key(o.id)))
        return tuple(ordered[:MAX_PROMPT_OBJECTS])

    def find(self, obj_id: str) -> Obj | None:
        return next((o for o in self.objects if o.id == obj_id), None)

    def propositions(self) -> tuple[str, ...]:
        return tuple(p for o in self.listed() for p in object_props(o.id))

    def descriptions(self) -> dict[str, str]:
        out = {}
        for o in self.listed():
            who = f"{o.id}（{o.name}）"
            out[near_prop(o.id)] = f"{who}の手前のマスにいて、{o.name}の方を向いている"
            out[around_prop(o.id)] = f"{who}の周りを左回りに 1 周し終えた"
            out[around_cw_prop(o.id)] = f"{who}の周りを右回りに 1 周し終えた"
        return out


def near_prop(obj_id: str) -> str:
    return f"near_{obj_id}"


def around_prop(obj_id: str) -> str:
    return f"around_{obj_id}"


def around_cw_prop(obj_id: str) -> str:
    return f"around_cw_{obj_id}"


def object_props(obj_id: str) -> tuple[str, str, str]:
    return near_prop(obj_id), around_prop(obj_id), around_cw_prop(obj_id)


def split_prop(name: str) -> tuple[str, str] | None:
    """例：around_cw_box_1 -> ("around_cw", "box_1")。物の命題でなければ None。"""
    m = PROP_RE.fullmatch(name)
    return (m.group(1), m.group(2)) if m else None


def formula_atoms(formula: str) -> frozenset[str]:
    return frozenset(_NAME.findall(formula)) - set(KEYWORDS)


def formula_object_props(formula: str) -> frozenset[str]:
    return frozenset(a for a in formula_atoms(formula) if PROP_RE.fullmatch(a))


def label_in(text: str) -> str | None:
    """LABEL_WORDS の語を 1 語として含む最初のラベル（LABEL_WORDS の順）。
    アイス・アドバイス・かわいすぎる・ボックスステップ・箱根のように、長い語の一部なら当てない（R16）。"""
    return next((label for label, pattern in _LABEL_RES.items() if pattern.search(text)), None)


def where(obj: Obj, pose: Pose | None) -> str | None:
    """Go2 から見た向き（45° ごとの 8 方位）と中心どうしの距離。例：「左前 2.9 m」。
    位置がないか、位置・物の x, y・距離が有限でなければ None（例外は出さない）。"""
    if pose is None or not all(math.isfinite(v) for v in (obj.x, obj.y, pose[0], pose[1], pose[2])):
        return None
    dx, dy = obj.x - pose[0], obj.y - pose[1]
    distance = math.hypot(dx, dy)
    if not math.isfinite(distance):                         # finite inputs can still overflow
        return None
    yaw = math.remainder(pose[2], 2 * math.pi)              # degrees() of a huge finite yaw (1e308) is inf
    bearing = _wrap_deg(math.degrees(math.atan2(dy, dx) - yaw))
    k = math.floor((bearing + 22.5) / 45) % 8
    return f"{_DIRECTION_WORDS[k]} {distance:.1f} m"


def describe(obj: Obj, pose: Pose | None) -> str:
    text = f"{obj.id}（{obj.name}）"
    w = where(obj, pose)
    if w is not None:
        text += f"：{w}"
    if not obj.visible:
        text += _NOT_VISIBLE
    return text


def parse(payload: object) -> tuple[Obj, ...]:
    """/go2/objects のメッセージ全体（JSON を読んだ dict）。odom でなければ ()。"""
    if not isinstance(payload, dict) or payload.get("frame") != FRAME:
        return ()
    return parse_rows(payload.get("objects"))


def parse_rows(rows: object) -> tuple[Obj, ...]:
    """正しい行だけを Obj にする（R21）。同じ id が続いたら最初の正しい行を残す。"""
    if not isinstance(rows, (list, tuple)):
        return ()
    out: list[Obj] = []
    ids: set[str] = set()
    for row in rows:
        obj = _parse_row(row)
        if obj is not None and obj.id not in ids:
            ids.add(obj.id)
            out.append(obj)
    return tuple(out)


def format_message(objs: Iterable[Obj], stamp: float) -> dict:
    return {"frame": FRAME, "stamp": float(stamp), "objects": [o.to_row() for o in objs]}


def snapshot_rows(objs: Iterable[Obj], pose: Pose | None) -> list[dict]:
    """Session.snapshot() の "objects"：to_row() のすべての鍵に name と where を足す。"""
    return [{**o.to_row(), "name": o.name, "where": where(o, pose)} for o in objs]


def wrap_rad(a: float) -> float:
    """(−π, π] に折り返す。grid と sim_core もこれを使う。"""
    a %= 2 * math.pi
    return a - 2 * math.pi if a > math.pi else a


_wrap_rad = wrap_rad            # the name before it was public


def winding_number(path: Sequence[tuple[float, float]], centre: tuple[float, float]) -> float:
    """centre から見た角度の変化（各区間 (−π, π] に折り返す）の和を 2π で割る。左回りが正。"""
    cx, cy = centre
    angles = [math.atan2(y - cy, x - cx) for x, y in path]
    return sum(wrap_rad(b - a) for a, b in zip(angles, angles[1:])) / (2 * math.pi)


def edge_distance(x: float, y: float, obj: Obj) -> float:
    """(x, y) から物の軸平行な底面（size）の縁までの距離。中なら 0。"""
    dx = max(abs(x - obj.x) - obj.size[0] / 2, 0.0)
    dy = max(abs(y - obj.y) - obj.size[1] / 2, 0.0)
    return math.hypot(dx, dy)


def msg_not_found(label: str) -> str:
    return f"「{LABEL_JA[label]}」が見つかりません。Go2 の前に置いてから、もう一度言ってください。"


def msg_too_far(label: str) -> str:
    return f"「{LABEL_JA[label]}」が遠すぎます（{MAX_OBJECT_DISTANCE:g} m まで）。"


def msg_ambiguous(label: str, n: int) -> str:
    name = LABEL_JA[label]
    return f"「{name}」が {n} つあるので、どれか決められません。「左の{name}」のように言ってください。"


# ---- 内部 ----

_KATAKANA = "ァ-ヺー"                     # the long-vowel mark too, but not the middle dot ・
_HIRAGANA = "ぁ-ゖ"
_KANJI = "々一-鿿"
_PARTICLES_BEFORE = "のとやもをにへがはで"
_PARTICLES_AFTER = "まで|から|[のとやもをにへがはで]"


def _script(ch: str) -> str | None:
    return next((name for name, chars in (("kata", _KATAKANA), ("hira", _HIRAGANA), ("kanji", _KANJI))
                 if re.fullmatch(f"[{chars}]", ch)), None)


# 語の境目は word_start / word_end の一か所だけで決める（助詞の一覧もここだけ）。

def word_start(word: str) -> str:
    """word の前に置く正規表現：word がそこで語として始まるときだけ当たる（長さ 0）。
    カタカナで始まる語はカタカナの後ろでは始まらない。ひらがなで始まる語は、ひらがなでない字の後ろか
    助詞の後ろで始まる（このいす は可、かわいすぎる は不可）。漢字で始まる語はどこでも始まる。
    物の名前（label_in）と、ひらがなの「持つ・押す…」（translator の OBJECT_ACTION_KANA）が使う。"""
    return {"kata": f"(?<![{_KATAKANA}])",
            "hira": f"(?:(?<![{_HIRAGANA}])|(?<=[{_PARTICLES_BEFORE}]))"}.get(_script(word[0]), "")


def word_end(word: str, after: str = _PARTICLES_AFTER) -> str:
    """word の後ろに置く正規表現：word がそこで語として終わるときだけ当たる。カタカナで終わる語は
    カタカナと続かない（チェアー は可）。ひらがなで終わる語は、ひらがなでない字か after（正規表現、
    既定は助詞）と続く（いすまで は可、いすわる は不可）。漢字で終わる語は漢字と続かない（箱根 は不可）。"""
    return {"kata": f"ー?(?![{_KATAKANA}])",
            "hira": f"(?:(?={after})|(?![{_HIRAGANA}]))",
            "kanji": f"(?![{_KANJI}])"}.get(_script(word[-1]), "")


def _word_pattern(word: str) -> str:
    """word が長い語の一部でないときだけ当たる正規表現。"""
    return word_start(word) + re.escape(word) + word_end(word)


_LABEL_RES = {label: re.compile("|".join(_word_pattern(w) for w in words)) for label, words in LABEL_WORDS.items()}


def _mm(v: float) -> float:
    return round(float(v), 3)


def _wrap_deg(d: float) -> float:
    """(−180, 180] に折り返す。"""
    d %= 360.0
    return d - 360.0 if d > 180.0 else d


def _id_key(obj_id: str) -> tuple[str, int]:
    """並べるときの id：(ラベル, 番号)。ID_RE に合わない id は (id, 0)。"""
    m = ID_RE.fullmatch(obj_id)
    return (m.group(1), int(obj_id[len(m.group(1)) + 1:])) if m else (obj_id, 0)


def _number(v: object) -> bool:
    """A finite int or float (not bool) that float() can hold; never raises (R21)."""
    if not isinstance(v, (int, float)) or isinstance(v, bool):
        return False
    try:
        return math.isfinite(v)
    except OverflowError:   # an int too large for a float, e.g. a 400-digit JSON number
        return False


def _parse_row(row: object) -> Obj | None:
    if not isinstance(row, dict):
        return None
    obj_id, label = row.get("id"), row.get("label")
    m = ID_RE.fullmatch(obj_id) if isinstance(obj_id, str) else None
    if m is None or label != m.group(1):
        return None
    x, y = row.get("x"), row.get("y")
    if not (_number(x) and _number(y)):
        return None
    size = row.get("size")
    if size is None:
        size = DEFAULT_SIZE
    elif not (isinstance(size, (list, tuple)) and len(size) == 2
              and all(_number(v) and 0 < v <= MAX_SIZE for v in size)):
        return None
    visible, seen = row.get("visible", True), row.get("seen")
    if not isinstance(visible, bool) or not (seen is None or _number(seen)):
        return None
    return Obj(obj_id, label, float(x), float(y), (float(size[0]), float(size[1])), visible,
               None if seen is None else float(seen))
