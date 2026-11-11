"""物の共通部品：物の行、凍結した Scene、命題名、文言、/go2/objects の JSON、足跡の形。

物の種類（label・日本語の名前・言い方・大きさ）は単語帳 voice_ltl/vocabulary.json から取る
（open-objects spec §7〜§9）。id は <label>_<n>（n は 1〜99）、命題は near_ / around_ / around_cw_ + id。
座標はすべて odom（x 前、y 左、REP-103）。標準ライブラリだけを使う
（Jazzy の Gazebo ブリッジと検出器もこのモジュールを読み込むが、そこには pitct も flloat もない）。
受け取った行の検査（R21）は parse_rows / parse の一か所だけで行う。
"""

from __future__ import annotations

import math
import re
from collections.abc import Iterable, Sequence
from dataclasses import dataclass

# The vocabulary (standard library only, like this module), read and checked at import: fail closed.
# Relative, so every absolute import here stays a standard-library one (test_objects checks those).
from . import vocabulary

Pose = tuple[float, float, float]                       # (x [m], y [m], yaw [rad]), odom

OBJECTS_TOPIC = "/go2/objects"
FRAME = "odom"
OBJECTS_HZ = 2.0
OBJECTS_STALE_S = 2.0

LABELS: frozenset[str] = vocabulary.LABELS
_ALT = "|".join(re.escape(label) for label in sorted(LABELS, key=lambda s: (-len(s), s)))   # longest first
ID_RE = re.compile(rf"({_ALT})_([1-9][0-9]?)")                                   # always fullmatch
PROP_RE = re.compile(rf"(near|around_cw|around)_((?:{_ALT})_[1-9][0-9]?)")       # always fullmatch
SOURCES = ("logical", "detector", "fake")      # the message's optional "source" (spec §11.8)
KEYWORDS = ("F", "G", "X", "U", "last")                                          # == grammar.KEYWORDS
_NAME = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")

DEFAULT_SIZE = (0.5, 0.5)                       # Obj's field default; a row without a size gets default_size()
MEASURE_MIN_RADIUS = 0.6                        # the bridge's LiDAR radius for sizing (spec §10.9, O12)
MEASURE_MARGIN = 0.3
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
    confidence: float | None = None             # the detector's, in [0, 1]; never shown (O23)
    measured: bool | None = None                # False until the size was measured (bridge, detector); O43

    @property
    def name(self) -> str:
        return name_of(self.label)

    @property
    def radius(self) -> float:
        return math.hypot(*self.size) / 2

    def to_row(self) -> dict:
        """The wire row. "confidence" and "measured" only when set, so the fake Go2's rows stay as they were."""
        row = {"id": self.id, "label": self.label, "x": _mm(self.x), "y": _mm(self.y),
               "size": [_mm(v) for v in self.size],
               "seen": None if self.seen is None else _mm(self.seen), "visible": self.visible}
        if self.confidence is not None:
            row["confidence"] = _mm(self.confidence)
        if self.measured is not None:
            row["measured"] = self.measured
        return row


DEFAULT_LAYOUT: tuple[Obj, ...] = (Obj("box_1", "box", 2.5, 1.5, (0.5, 0.5)),
                                   Obj("chair_1", "chair", 2.5, -1.5, (0.45, 0.45)))


@dataclass(frozen=True)
class Scene:
    """指示を受けた時点で凍結した物の一覧と Go2 の位置。"""

    objects: tuple[Obj, ...] = ()
    pose: Pose | None = None
    focus: frozenset[str] = frozenset()         # labels the instruction names (labels_in); listed first (O6)

    def listed(self) -> tuple[Obj, ...]:
        """プロンプト・語彙・検査に載せる物：位置から近い順（位置がなければ id 順）に 8 個まで。
        id はラベルの中で番号の順（box_2 は box_10 より前）。focus の種類の物をその順で先に 8 個まで取り、
        残りをその順で 8 個まで埋め、その順で返す（spec §9.1）。focus が空なら今までと同じ一覧。"""
        if self.pose is None:
            ordered = sorted(self.objects, key=lambda o: _id_key(o.id))
        else:
            px, py = self.pose[0], self.pose[1]
            ordered = sorted(self.objects, key=lambda o: (math.hypot(o.x - px, o.y - py), _id_key(o.id)))
        first = [i for i, o in enumerate(ordered) if o.label in self.focus][:MAX_PROMPT_OBJECTS]
        rest = [i for i, o in enumerate(ordered) if o.label not in self.focus][:MAX_PROMPT_OBJECTS - len(first)]
        return tuple(ordered[i] for i in sorted(first + rest))

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


Kind = vocabulary.Kind


def name_of(label: str) -> str:
    """単語帳の ja。単語帳にない label なら、_ を空白にしたもの（例外は出さない）。"""
    return vocabulary.name_of(label)


def default_size(label: str) -> tuple[float, float]:
    """測るまでの足跡：単語帳の長い辺の正方形 (L, L)（向きを知らないので）。単語帳にない label なら DEFAULT_SIZE。"""
    size = vocabulary.size_of(label)
    return DEFAULT_SIZE if size is None else (size[0], size[0])


def measure_radius(label: str) -> float:
    """橋渡しが LiDAR で大きさを測る半径：max(0.6, 単語帳の対角の半分 + 0.3) m。単語帳にない label なら 0.6。"""
    size = vocabulary.size_of(label)
    if size is None:
        return MEASURE_MIN_RADIUS
    return max(MEASURE_MIN_RADIUS, math.hypot(*size) / 2 + MEASURE_MARGIN)


def kinds_in(text: str) -> tuple[Kind, ...]:
    """指示の中の単語帳の言葉を文の順に（spec §9.5。同じ所では長い言葉が勝ち、not_words は当てない）。"""
    return vocabulary.kinds_in(text)


def label_in(text: str) -> str | None:
    """最後に当たった単語帳の言葉の種類（日本語の名詞句は後ろが中心：「椅子の上の箱」は box。O1）。
    アイス・かわいすぎる・箱根・車椅子のように、長い語の一部や not_words なら当てない（R16）。"""
    kinds = kinds_in(text)
    return kinds[-1].label if kinds else None


def labels_in(text: str) -> frozenset[str]:
    """当たったすべての種類（Scene.focus に使う。spec §9.1）。"""
    return frozenset(k.label for k in kinds_in(text))


def direction(obj: Obj, pose: Pose | None) -> str | None:
    """Go2 から見た物の中心の向き：45° ごとの 8 方位の言葉（前・左前・左…）。where() もこれを使う。
    位置がないか、位置・物の x, y・距離が有限でなければ None（例外は出さない）。"""
    if pose is None or not all(math.isfinite(v) for v in (obj.x, obj.y, pose[0], pose[1], pose[2])):
        return None
    dx, dy = obj.x - pose[0], obj.y - pose[1]
    if not math.isfinite(math.hypot(dx, dy)):               # finite inputs can still overflow
        return None
    yaw = math.remainder(pose[2], 2 * math.pi)              # degrees() of a huge finite yaw (1e308) is inf
    bearing = _wrap_deg(math.degrees(math.atan2(dy, dx) - yaw))
    return _DIRECTION_WORDS[math.floor((bearing + 22.5) / 45) % 8]


def where(obj: Obj, pose: Pose | None) -> str | None:
    """Go2 から見た向き（direction）と中心どうしの距離。例：「左前 2.9 m」。
    位置がないか、位置・物の x, y・距離が有限でなければ None（例外は出さない）。"""
    word = direction(obj, pose)
    if word is None:
        return None
    return f"{word} {math.hypot(obj.x - pose[0], obj.y - pose[1]):.1f} m"


def describe(obj: Obj, pose: Pose | None) -> str:
    text = f"{obj.id}（{obj.name}）"
    w = where(obj, pose)
    if w is not None:
        text += f"：{w}"
    if not obj.visible:
        text += _NOT_VISIBLE
    return text


def parse_source(payload: object) -> str | None:
    """メッセージの "source"（SOURCES のどれか）。ないか、ほかの値なら None（出どころは不明）。"""
    if not isinstance(payload, dict):
        return None
    source = payload.get("source")
    return source if isinstance(source, str) and source in SOURCES else None


def parse(payload: object) -> tuple[Obj, ...]:
    """/go2/objects のメッセージ全体（JSON を読んだ dict）。odom でないか、"source" があって
    SOURCES のどれでもなければ ()（メッセージをまるごと捨てる。R21）。"""
    if not isinstance(payload, dict) or payload.get("frame") != FRAME:
        return ()
    if "source" in payload and parse_source(payload) is None:
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


def format_message(objs: Iterable[Obj], stamp: float, source: str | None = None) -> dict:
    """/go2/objects に出す dict。source（SOURCES のどれか）を渡したときだけ "source" を書く。"""
    if source is not None and source not in SOURCES:
        raise ValueError(f"source は {' / '.join(SOURCES)} のどれかです: {source!r}")
    msg: dict = {"frame": FRAME, "stamp": float(stamp)}
    if source is not None:
        msg["source"] = source
    msg["objects"] = [o.to_row() for o in objs]
    return msg


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
    """(x, y) から物の軸平行な底面（size、足跡）の縁までの距離 d_e。中なら 0。"""
    dx = max(abs(x - obj.x) - obj.size[0] / 2, 0.0)
    dy = max(abs(y - obj.y) - obj.size[1] / 2, 0.0)
    return math.hypot(dx, dy)


# 足跡の形（spec §10.2、design-near §3.1）：a = sx/2、b = sy/2、m = min(a, b)、e = |a − b|。
# 背骨は中心を通って長い辺の向きに伸びる半分の長さ e の線分（正方形なら中心の 1 点）。
# grid（手前の帯 B）と objects_check（縁から測る合格の線）が同じ定義を使う。

def spine_point(x: float, y: float, obj: Obj) -> tuple[float, float]:
    """(x, y) にいちばん近い背骨の点 s(p)。"""
    a, b = obj.size[0] / 2, obj.size[1] / 2
    e = abs(a - b)
    if a >= b:
        return obj.x + min(max(x - obj.x, -e), e), obj.y
    return obj.x, obj.y + min(max(y - obj.y, -e), e)


def core_distance(x: float, y: float, obj: Obj) -> float:
    """芯（足跡の中のいちばん大きい、角の丸い四角）までの距離 d_k = |p − s(p)| − m。芯の中なら負。
    d_e ≤ d_k ≤ d_e + 0.414·m。長い辺の横では d_k = d_e、正方形では d_k = d_c − a。"""
    sx, sy = spine_point(x, y, obj)
    return math.hypot(x - sx, y - sy) - min(obj.size) / 2


def spine_angle(x: float, y: float, yaw: float, obj: Obj) -> float:
    """向き yaw と、(x, y) からいちばん近い背骨の点への方向との差 [rad]（0〜π）。"""
    sx, sy = spine_point(x, y, obj)
    return abs(wrap_rad(math.atan2(sy - y, sx - x) - yaw))


def footprint_angle(x: float, y: float, yaw: float, obj: Obj) -> float:
    """向き yaw と、足跡のいちばん近い点の方向との差 [rad]：yaw が足跡の角の広がりの中を向いていれば 0
    （足跡の中でも 0）。spec §10.8・O29：手前の合格の線は、足跡のどこかの点が 45° 以内。"""
    if edge_distance(x, y, obj) == 0.0:
        return 0.0
    c = wrap_rad(math.atan2(obj.y - y, obj.x - x) - yaw)          # the centre is inside the angular hull
    rel = [wrap_rad(wrap_rad(math.atan2(cy - y, cx - x) - yaw) - c) for cx, cy in _corners(obj)]
    lo, hi = c + min(rel), c + max(rel)                            # hull width < pi outside the rectangle
    return 0.0 if lo <= 0.0 <= hi else min(abs(wrap_rad(lo)), abs(wrap_rad(hi)))


# ---- 文言（spec §9.9・§10.6）。{name} は利用者の単語帳の言葉、なければ name_of(label)（O3） ----

def msg_not_found(label: str, word: str | None = None) -> str:
    return (f"「{_said(label, word)}」が見つかりません。Go2 の前に置くか、Go2 をそちらに向けてから、"
            "もう一度言ってください。")


def msg_too_far(label: str, word: str | None = None) -> str:
    return f"「{_said(label, word)}」が遠すぎます（{MAX_OBJECT_DISTANCE:g} m まで）。"


def msg_ambiguous(label: str, n: int, directions: Sequence[str | None] = (), word: str | None = None) -> str:
    """n は Scene の本当の数。directions は載せた候補の向き（direction()、載せた順）。
    2〜3 個がどれもあってすべて違えば「〈向き〉の〈名前〉」を並べ、ちょうど 2 個で同じなら「近いほう・遠いほう」、
    ほか（位置がないときも）は今までの「左の〈名前〉」。"""
    name = _said(label, word)
    dirs = tuple(directions)
    given = all(isinstance(d, str) and d for d in dirs)
    if given and 2 <= len(dirs) <= 3 and len(set(dirs)) == len(dirs):
        hint = "".join(f"「{d}の{name}」" for d in dirs)
    elif given and len(dirs) == 2 and dirs[0] == dirs[1]:
        hint = f"「近いほうの{name}」「遠いほうの{name}」"
    else:
        hint = f"「左の{name}」"
    return f"「{name}」が {_count(n)}あるので、どれか決められません。{hint}のように言ってください。"


def msg_not_listed(label: str, word: str | None = None) -> str:
    return (f"「{_said(label, word)}」はほかの物より遠いので、選べません"
            f"（近い順に {MAX_PROMPT_OBJECTS} 個まで）。")


def msg_lap_too_big(label: str) -> str:
    return f"「{name_of(label)}」は大きすぎて、周りを回る計画が作れません。"


def msg_too_big_here(label: str) -> str:
    return f"「{name_of(label)}」は大きいので、ここからは計画できません。近づいてから、もう一度言ってください。"


# ---- 内部 ----

# 語の境目（word_start / word_end と助詞の一覧）は vocabulary.py に移した。ここと translator はそれを使う。
_KATAKANA, _HIRAGANA, _KANJI = vocabulary._KATAKANA, vocabulary._HIRAGANA, vocabulary._KANJI
_PARTICLES_BEFORE, _PARTICLES_AFTER = vocabulary._PARTICLES_BEFORE, vocabulary._PARTICLES_AFTER
_script, word_start, word_end = vocabulary._script, vocabulary.word_start, vocabulary.word_end
_word_pattern = vocabulary._word_pattern


def _mm(v: float) -> float:
    return round(float(v), 3)


def _said(label: str, word: str | None) -> str:
    return word if word else name_of(label)


def _count(n: int) -> str:
    """「2 つ」（9 まで）・「12 個」（10 から）。"""
    return f"{n} つ" if n <= 9 else f"{n} 個"


def _corners(obj: Obj) -> tuple[tuple[float, float], ...]:
    a, b = obj.size[0] / 2, obj.size[1] / 2
    return tuple((obj.x + sx * a, obj.y + sy * b) for sx in (-1, 1) for sy in (-1, 1))


def _wrap_deg(d: float) -> float:
    """(−180, 180] に折り返す。"""
    d %= 360.0
    return d - 360.0 if d > 180.0 else d


def _id_key(obj_id: str) -> tuple[str, int]:
    """並べるときの id：(ラベル, 番号)。ID_RE に合わない id は (id, 0)。"""
    m = ID_RE.fullmatch(obj_id)
    return (m.group(1), int(m.group(2))) if m else (obj_id, 0)


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
        size = default_size(label)
    elif not (isinstance(size, (list, tuple)) and len(size) == 2
              and all(_number(v) and 0 < v <= MAX_SIZE for v in size)):
        return None
    visible, seen = row.get("visible", True), row.get("seen")
    if not isinstance(visible, bool) or not (seen is None or _number(seen)):
        return None
    confidence, measured = row.get("confidence"), row.get("measured")      # optional (spec §11.8)
    if not (confidence is None or (_number(confidence) and 0.0 <= confidence <= 1.0)):
        return None
    if not (measured is None or isinstance(measured, bool)):
        return None
    return Obj(obj_id, label, float(x), float(y), (float(size[0]), float(size[1])), visible,
               None if seen is None else float(seen), None if confidence is None else float(confidence), measured)
