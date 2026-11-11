"""検出の見え方を追跡にし、確かめ、id を付け、忘れる（open-objects spec §11.7、design-detector §8）。

1 枚の止まった絵ごとに `update` を呼ぶ。見え方（`Observation`）は geometry.Sighting から 1 対 1 で作る（Task 2.7）。
- 見え方：単語帳の label で、確からしさが `sighting` 以上、中心がカメラから 8 m（`DET_RANGE`）以内。
- 結び付け（絵ごと、中心の距離が近い順に 1 対 1）：同じ label で 0.5 + 0.5·max(r) m 以内（予備の見え方か、
  LiDAR の支えのない追跡は +0.5 m）。結び付かなかった見え方は、結び付かなかった別の label の追跡と
  足跡の IoU が 0.4 以上なら、その追跡への名前の票。ほかは新しい仮の追跡。
- 確かめ：仮の追跡の直近 N = 3 回の「見える絵」（in_view が真で、中心がカメラから 6 m 以内）のうち、
  シミュレーションで 0.5 秒以上離れた当たりが M = 2 回、1 回以上が LiDAR に支えられ、勝った label の当たりが
  1 回以上その label の `confirm` 以上。支えのある当たりがなければ 3 回中 3 回、どれも `confirm_unsupported`
  以上。窓に当たりがなくなるか、30 秒（`TENTATIVE_TTL_S`、wall）で確かにならなければ、id なしで捨てる。
- id：確かになったとき `<label>_<n>`、n は label ごとに 1 から。走っている間は使い回さない。99 を超えた label の
  新しい追跡は出さず、note を 1 つ残す（C6）。
- 名前の付け直し：直近 5 回の当たりで、別の label の合計が今の label の 2 倍以上で、その label の `confirm` 以上の
  当たりが 1 回あれば、今の id を終わりにし、同じ点で新しい label の新しい id にする。
- まとめ：絵ごとに、確かな追跡 2 つの足跡が IoU 0.5 以上なら古い id にまとめ、新しい id を終わりにする。
  同じ label なら、片方の中心がもう片方の足跡の中にあるときもまとめる。違う label の少しの重なり
  （机の下に押し込んだ椅子。IoU 約 0.17 で、椅子の中心は机の足跡の中）は 2 つのまま。確かになるときや
  名前を付け直すときに、この決まりですぐまとまる確かな追跡があれば、新しい id を出さずにその追跡にまとめる
  （同じ絵の中の重なった見え方、台の上のランプと desk・ソファとその一部・box と bench が、止まるたびに id を
  使い切らないように。C6）。
- 忘れ方：LiDAR に支えられた確かな追跡は、空きの判定（`mark_vacant`）が 0.5 秒以上離れて 2 回続いたときだけ
  消す。支えのない確かな追跡は、見える絵で 3 回続けて当たらなければ消す。ほかは `visible: false` で覚えておく。
- 見える絵：`in_view` は呼ぶ側（core）が、カメラから 6 m 以内・絵の中・LiDAR に隠されていない、のすべてを
  確かめる。追跡はカメラの位置を見え方の `cam_xy` からしか知らないので、見え方のない絵では `in_view` だけで決まる。
- 形：LiDAR に支えられた直近 6 回の当たりの点（0.05 m で間引き、2000 点まで）を `fuse` に渡す（向きは
  いちばん新しい見え方）。予備の見え方は、LiDAR に支えられた追跡を動かさない。

名前は見え方の label だけから取る（ほかのトピックやファイルは読まない）。出すときに単語帳の label か確かめ直し、
仮の追跡は出さない。時刻：frame_t・scan_t はセンサーの時刻（シミュレーション）、wall_t・wall_now は `seen` の
時計（core が絵の届いたときの time.time() を渡す）。標準ライブラリと numpy とこのリポジトリだけを使う。
"""

from __future__ import annotations

import logging
import math
import statistics
from collections import deque
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field

import numpy as np

from voice_ltl import objects, vocabulary
from detector.protocol import Thresholds, check_thresholds

M = 2; N = 3; MIN_HIT_GAP_S = 0.5; TENTATIVE_TTL_S = 30.0; ASSOC_BASE = 0.5; ASSOC_GAIN = 0.5; FALLBACK_EXTRA = 0.5
XLABEL_IOU = 0.4; MERGE_IOU = 0.5; RELABEL_HITS = 5; RELABEL_RATIO = 2.0; CONF_WINDOW = 10; TRACK_POINT_FRAMES = 6
VACANT_FRAMES = 2; MISS_UNSUPPORTED = 3; MAX_ID = 99; VISIBLE_HOLD_MIN_S = 5.0
INVIEW_RANGE = objects.MAX_OBJECT_DISTANCE      # 6 m: a frame counts toward N only within this
DET_RANGE = objects.CAMERA_FAR                  # 8 m: a sighting's centre at most this far from the camera
THIN_CELL = 0.05                                # the fused points are thinned to one per 5 cm cell
MAX_TRACK_POINTS = 2000
SIZE_MIN = 0.1
_EPS = 1e-9

_LOG = logging.getLogger("detector.tracker")


@dataclass(frozen=True)
class Observation:
    """One detection with usable geometry (geometry.Sighting with a footprint, Task 2.1)."""

    label: str
    confidence: float
    box: tuple[float, float, float, float]      # the image box (pixels)
    x: float                                    # footprint centre, odom
    y: float
    size: tuple[float, float]                   # footprint [sx, sy] along odom x and y
    filled: bool                                # the depth came from the vocabulary prior (C2)
    points: np.ndarray | None = field(compare=False)   # N x 2 odom XY of the picked cluster; None unless supported
    supported: bool                             # LiDAR-supported
    fallback: bool                              # the ground-ray fallback
    cam_xy: tuple[float, float]                 # camera position in odom at the image stamp


@dataclass(frozen=True)
class _Hit:
    frame_t: float
    label: str
    confidence: float
    supported: bool


class _Track:
    def __init__(self, serial: int, label: str, wall_t: float) -> None:
        self.serial = serial                    # creation order
        self.label = label                      # tentative: argmax of the summed votes; confirmed: fixed
        self.obj_id: str | None = None          # set at confirmation
        self.order = -1                         # confirmation order (older id = smaller)
        self.born_wall = wall_t
        self.x = self.y = 0.0
        self.size: tuple[float, float] = objects.DEFAULT_SIZE
        self.filled = False
        self.measured = False                   # the geometry comes from LiDAR-supported hits
        self.cam_xy = (0.0, 0.0)                # the newest view
        self.point_frames: deque[np.ndarray] = deque(maxlen=TRACK_POINT_FRAMES)
        self.hits: deque[_Hit] = deque(maxlen=RELABEL_HITS)
        self.votes: dict[str, float] = {}       # label -> summed confidence, every hit (the tentative label)
        self.confidences: dict[str, deque[float]] = {}
        self.window: deque[_Hit | None] = deque(maxlen=N)   # the last N in-view frames
        self.frame_hit: _Hit | None = None      # this frame's hit
        self.last_hit_wall: float | None = None
        self.misses = 0                         # consecutive in-view frames without a hit
        self.vacant_count = 0
        self.vacant_last: float | None = None

    @property
    def confirmed(self) -> bool:
        return self.obj_id is not None

    @property
    def radius(self) -> float:
        return math.hypot(*self.size) / 2


class Tracker:
    def __init__(self, thresholds: Thresholds, priors: Mapping[str, tuple[float, float]],
                 fuse: Callable[[np.ndarray, tuple[float, float], tuple[float, float]], object | None]) -> None:
        """priors: label -> vocabulary size [long, short]; fuse(points_xy, cam_xy, prior) returns an object with
        x, y, size and filled (geometry.footprint), or None."""
        check_thresholds(thresholds, vocabulary.LABELS)
        self._th = thresholds
        self._priors = dict(priors)
        self._fuse_fn = fuse
        self.reset()

    # --- state ------------------------------------------------------------------------------------------------

    def reset(self) -> None:
        """Forget every track and restart every label's numbering at 1 (notes cleared too)."""
        self._tracks: list[_Track] = []
        self._serial = 0
        self._order = 0
        self._next: dict[str, int] = {}         # label -> the next n
        self._retired = 0
        self._capped: set[str] = set()
        self._notes: list[str] = []

    # --- one still image --------------------------------------------------------------------------------------

    def update(self, observations: Sequence[Observation], frame_t: float, wall_t: float,
               in_view: Callable[[float, float], bool]) -> None:
        """One still image. in_view(x, y): the image's camera sees odom point (x, y). in_view must itself apply
        all of: within INVIEW_RANGE (6 m) of the image's camera, inside the image (frustum), and not occluded by
        LiDAR returns (for a track without LiDAR support also not cut short by returns in front). The tracker
        takes the camera only from the observations' cam_xy, so it can add the 6 m check only to frames with
        observations; in a frame without any, in_view alone decides whether the frame counts toward N and
        whether an unsupported confirmed track misses."""
        frame_t, wall_t = float(frame_t), float(wall_t)
        cam = next((_xy(o.cam_xy) for o in observations if isinstance(o, Observation) and _xy(o.cam_xy)), None)
        usable = [o for o in observations if self._usable(o)]
        for t in self._tracks:
            t.frame_hit = None
        self._tracks = [t for t in self._tracks if t.confirmed or wall_t - t.born_wall <= TENTATIVE_TTL_S]

        pairs, rest = self._associate(usable)
        for track, o in pairs:
            self._hit(track, o, frame_t, wall_t)
        for o in rest:
            self._serial += 1
            track = _Track(self._serial, o.label, wall_t)
            self._tracks.append(track)
            self._hit(track, o, frame_t, wall_t)

        for track in list(self._tracks):                     # in_view only where it counts
            if not track.confirmed:
                if self._in_view(track, cam, in_view):
                    track.window.append(track.frame_hit)
                    if len(track.window) == N and all(h is None for h in track.window):
                        self._tracks.remove(track)           # its hits left the window: no id
                        continue
                if self._confirmable(track):
                    self._confirm(track)
            elif track.frame_hit is not None:
                self._relabel(track)                 # confirmed before this frame (new tracks are tentative)
            elif not track.measured and self._in_view(track, cam, in_view):
                track.misses += 1
                if track.misses >= MISS_UNSUPPORTED:
                    self._retire(track)
        self._merge()

    def _usable(self, o: object) -> bool:
        """A sighting (spec §11.7): a vocabulary label, confidence >= sighting, finite geometry, within 8 m."""
        if not isinstance(o, Observation) or o.label not in vocabulary.LABELS:
            return False
        if not (_finite(o.confidence) and self._th.sighting <= o.confidence <= 1.0):
            return False
        if not (_finite(o.x) and _finite(o.y) and _size(o.size) is not None):
            return False
        cam = _xy(o.cam_xy)
        return cam is not None and math.hypot(o.x - cam[0], o.y - cam[1]) <= DET_RANGE

    def _associate(self, obs: list[Observation]) -> tuple[list[tuple[_Track, Observation]], list[Observation]]:
        """Same label within the gate, nearest first, one to one; then label votes by IoU; the rest are new."""
        tracks = self._tracks
        cands = []
        for ti, t in enumerate(tracks):
            for oi, o in enumerate(obs):
                if o.label != t.label:
                    continue
                d = math.hypot(o.x - t.x, o.y - t.y)
                gate = ASSOC_BASE + ASSOC_GAIN * max(t.radius, math.hypot(*o.size) / 2)
                if not o.supported or not t.measured:
                    gate += FALLBACK_EXTRA
                if d <= gate + _EPS:
                    cands.append((d, t.serial, oi, ti))
        cands.sort()
        used_t: set[int] = set()
        used_o: set[int] = set()
        pairs = []
        for _d, _s, oi, ti in cands:
            if ti not in used_t and oi not in used_o:
                used_t.add(ti)
                used_o.add(oi)
                pairs.append((tracks[ti], obs[oi]))
        votes = []
        for oi, o in enumerate(obs):
            if oi in used_o:
                continue
            for ti, t in enumerate(tracks):
                if ti in used_t or t.label == o.label:
                    continue
                iou = _iou((o.x, o.y, o.size), (t.x, t.y, t.size))
                if iou >= XLABEL_IOU - _EPS:
                    votes.append((-iou, math.hypot(o.x - t.x, o.y - t.y), t.serial, oi, ti))
        votes.sort()
        for _i, _d, _s, oi, ti in votes:
            if ti not in used_t and oi not in used_o:
                used_t.add(ti)
                used_o.add(oi)
                pairs.append((tracks[ti], obs[oi]))
        return pairs, [o for oi, o in enumerate(obs) if oi not in used_o]

    def _hit(self, track: _Track, o: Observation, frame_t: float, wall_t: float) -> None:
        conf = float(o.confidence)
        points = _points(o.points) if o.supported else None
        hit = _Hit(frame_t, o.label, conf, bool(o.supported))
        track.frame_hit = hit
        track.hits.append(hit)
        track.votes[o.label] = track.votes.get(o.label, 0.0) + conf
        track.confidences.setdefault(o.label, deque(maxlen=CONF_WINDOW)).append(conf)
        track.last_hit_wall = wall_t
        track.misses = 0
        track.vacant_count, track.vacant_last = 0, None          # seen again: the place is not empty
        if not track.confirmed:
            track.label = max(track.votes, key=track.votes.__getitem__)   # ties: the label seen first
        cam = _xy(o.cam_xy)
        if o.supported and points is not None and len(points):
            track.point_frames.append(_thin(points))
            track.cam_xy = cam
            track.measured = True
            fused = self._fuse_fn(_thin(np.concatenate(list(track.point_frames))), cam, self._prior(track.label))
            if fused is not None and _finite(fused.x) and _finite(fused.y) and _size(fused.size) is not None:
                track.x, track.y, track.size, track.filled = (float(fused.x), float(fused.y), _size(fused.size),
                                                              bool(fused.filled))
                return
        elif track.measured:
            return                                  # a fallback view never moves a LiDAR-supported track
        if o.supported:
            track.measured = True
        track.x, track.y, track.size, track.filled = float(o.x), float(o.y), _size(o.size), bool(o.filled)
        track.cam_xy = cam

    def _prior(self, label: str) -> tuple[float, float]:
        prior = self._priors.get(label) or vocabulary.size_of(label) or objects.DEFAULT_SIZE
        return (float(prior[0]), float(prior[1]))

    @staticmethod
    def _in_view(track: _Track, cam: tuple[float, float] | None, in_view: Callable[[float, float], bool]) -> bool:
        if cam is not None and math.hypot(track.x - cam[0], track.y - cam[1]) > INVIEW_RANGE + _EPS:
            return False
        return bool(in_view(track.x, track.y))

    def _confirmable(self, track: _Track) -> bool:
        hits = [h for h in track.window if h is not None]
        if not hits:
            return False
        label = track.label
        if max((h.confidence for h in hits if h.label == label), default=0.0) < self._th.confirm_for(label):
            return False
        spaced = _spaced(hits)
        if any(h.supported for h in hits):
            return spaced >= M
        return (len(hits) == N and spaced >= N
                and all(h.confidence >= self._th.confirm_unsupported for h in hits))

    def _issue(self, label: str) -> str | None:
        """The next id of label, or None (with one note per label) once its ids ran out (C6)."""
        n = self._next.get(label, 1)
        if n > MAX_ID:
            if label not in self._capped:
                self._capped.add(label)
                note = f"{label}: ids 1-{MAX_ID} used, new {label} tracks ignored"
                self._notes.append(note)
                _LOG.warning(note)
            return None
        self._next[label] = n + 1
        return f"{label}_{n}"

    def _absorber(self, track: _Track, label: str) -> _Track | None:
        """The oldest confirmed track that track, published as label, would merge into at once (_overlap)."""
        found = [t for t in self._tracks if t.confirmed and t is not track and _overlap(t, track, label)]
        return min(found, key=lambda t: (t.order, t.serial), default=None)

    def _confirm(self, track: _Track) -> None:
        old = self._absorber(track, track.label)
        if old is not None:                         # a duplicate view of a confirmed track: it joins it and
            _fold(old, track)                       # takes no id, so duplicates never use up a label's ids
            self._tracks.remove(track)
            return
        obj_id = self._issue(track.label)
        if obj_id is None:
            self._tracks.remove(track)
            return
        track.obj_id = obj_id
        track.order = self._order
        self._order += 1

    def _relabel(self, track: _Track) -> None:
        """Another label's sum over the last 5 hits >= 2 x the published label's, with one hit at its confirm."""
        sums: dict[str, float] = {}
        for h in track.hits:
            sums[h.label] = sums.get(h.label, 0.0) + h.confidence
        own = sums.get(track.label, 0.0)
        best = None
        for label, total in sums.items():
            if label == track.label or total < RELABEL_RATIO * own - _EPS:
                continue
            if not any(h.label == label and h.confidence >= self._th.confirm_for(label) for h in track.hits):
                continue
            if best is None or total > sums[best]:
                best = label
        if best is None:
            return
        old = self._absorber(track, best)
        if old is not None:                         # the new label would merge into old at once: join it, the
            _fold(old, track)                       # current id ends and no new id is used
            self._retire(track)
            return
        obj_id = self._issue(best)
        if obj_id is None:
            return                                  # the new label's ids ran out: keep the old id
        self._retired += 1
        track.label, track.obj_id = best, obj_id
        track.order = self._order
        self._order += 1

    def _retire(self, track: _Track) -> None:
        self._tracks.remove(track)
        self._retired += 1

    def _merge(self) -> None:
        """Confirmed pairs with IoU >= 0.5 (same label: or a centre inside the other) merge into the older id."""
        confirmed = sorted((t for t in self._tracks if t.confirmed), key=lambda t: (t.order, t.serial))
        gone: set[int] = set()
        for i, old in enumerate(confirmed):
            if id(old) in gone:
                continue
            for young in confirmed[i + 1:]:
                if id(young) in gone or not _overlap(old, young):
                    continue
                _fold(old, young)
                gone.add(id(young))
                self._retire(young)

    # --- forgetting -------------------------------------------------------------------------------------------

    def vacancy_targets(self) -> list[tuple[str, float, float, tuple[float, float], str]]:
        """(key, x, y, size, label) of every LiDAR-supported confirmed track: the places the core tests on
        each still scan (geometry.vacancy_counts with the label's height)."""
        tracks = sorted((t for t in self._tracks if t.confirmed and t.measured),
                        key=lambda t: objects._id_key(t.obj_id))
        return [(t.obj_id, t.x, t.y, t.size, t.label) for t in tracks]

    def mark_vacant(self, key: str, vacant: bool, scan_t: float) -> None:
        """One still scan's vacancy result for a key of vacancy_targets(). Two vacant scans >= 0.5 s (sim)
        apart in a row retire the track; a scan that is not vacant starts the count again. Unknown keys: ignored."""
        track = next((t for t in self._tracks if t.confirmed and t.measured and t.obj_id == key), None)
        if track is None:
            return
        if not vacant:
            track.vacant_count, track.vacant_last = 0, None
            return
        scan_t = float(scan_t)
        if track.vacant_last is None or scan_t - track.vacant_last >= MIN_HIT_GAP_S - _EPS:
            track.vacant_count += 1
            track.vacant_last = scan_t
        if track.vacant_count >= VACANT_FRAMES:
            self._retire(track)

    # --- output -----------------------------------------------------------------------------------------------

    def rows(self, wall_now: float, idle_period_s: float) -> list[objects.Obj]:
        """The confirmed tracks as rows, in _id_key order (vocabulary labels only, re-checked here)."""
        idle = float(idle_period_s) if _finite(idle_period_s) else 0.0
        hold = max(VISIBLE_HOLD_MIN_S, 2.0 * idle)
        out = []
        for t in self._tracks:
            if not t.confirmed or t.label not in vocabulary.LABELS or not (_finite(t.x) and _finite(t.y)):
                continue
            size = tuple(round(min(max(v, SIZE_MIN), objects.MAX_SIZE), 3) for v in t.size)
            visible = t.last_hit_wall is not None and float(wall_now) - t.last_hit_wall <= hold + _EPS
            seen = None if t.last_hit_wall is None else round(t.last_hit_wall, 3)
            out.append(objects.Obj(t.obj_id, t.label, round(t.x, 3), round(t.y, 3), size, visible, seen,
                                   self._confidence(t), t.measured))
        return sorted(out, key=lambda o: objects._id_key(o.id))

    @staticmethod
    def _confidence(t: _Track) -> float:
        """The median of the last <= 10 hit confidences of the published label, 3 decimals."""
        values = list(t.confidences.get(t.label, ())) or [h.confidence for h in t.hits] or [0.0]
        return round(min(max(statistics.median(values), 0.0), 1.0), 3)

    def stats(self) -> dict:
        confirmed = sorted((t for t in self._tracks if t.confirmed), key=lambda t: objects._id_key(t.obj_id))
        return {
            "tracks_tentative": sum(1 for t in self._tracks if not t.confirmed),
            "tracks_confirmed": len(confirmed),
            "ids_issued": {label: n - 1 for label, n in sorted(self._next.items())},
            "ids_retired": self._retired,
            "filled": [t.obj_id for t in confirmed if t.filled],
            "notes": list(self._notes),
        }


# --- helpers --------------------------------------------------------------------------------------------------

def _finite(v: object) -> bool:
    if isinstance(v, (bool, np.bool_)):
        return False
    try:
        return math.isfinite(float(v))
    except (TypeError, ValueError, OverflowError):
        return False


def _xy(v: object) -> tuple[float, float] | None:
    try:
        x, y = v
    except (TypeError, ValueError):
        return None
    return (float(x), float(y)) if _finite(x) and _finite(y) else None


def _size(v: object) -> tuple[float, float] | None:
    s = _xy(v)
    return s if s is not None and s[0] > 0 and s[1] > 0 else None


def _points(points: object) -> np.ndarray | None:
    """N x 2 finite float64 odom XY, or None."""
    if points is None:
        return None
    a = np.asarray(points, dtype=np.float64)
    if a.ndim != 2 or a.shape[1] < 2:
        return None
    a = a[:, :2]
    return a[np.isfinite(a).all(axis=1)]


def _thin(points: np.ndarray) -> np.ndarray:
    """One point per THIN_CELL cell (the first), then at most MAX_TRACK_POINTS evenly spaced."""
    if len(points) == 0:
        return points
    keys = np.floor(points / THIN_CELL).astype(np.int64)
    _unique, first = np.unique(keys, axis=0, return_index=True)
    out = points[np.sort(first)]
    if len(out) > MAX_TRACK_POINTS:
        out = out[np.linspace(0, len(out) - 1, MAX_TRACK_POINTS).round().astype(np.int64)]
    return out


def _spaced(hits: list[_Hit]) -> int:
    """How many hits count: each at least MIN_HIT_GAP_S (sim) after the last one counted."""
    count, last = 0, None
    for h in sorted(hits, key=lambda h: h.frame_t):
        if last is None or h.frame_t - last >= MIN_HIT_GAP_S - _EPS:
            count, last = count + 1, h.frame_t
    return count


def _rect(x: float, y: float, size: tuple[float, float]) -> tuple[float, float, float, float]:
    return (x - size[0] / 2, y - size[1] / 2, x + size[0] / 2, y + size[1] / 2)


def _iou(a: tuple[float, float, tuple[float, float]], b: tuple[float, float, tuple[float, float]]) -> float:
    ax0, ay0, ax1, ay1 = _rect(*a)
    bx0, by0, bx1, by1 = _rect(*b)
    inter = max(0.0, min(ax1, bx1) - max(ax0, bx0)) * max(0.0, min(ay1, by1) - max(ay0, by0))
    union = (ax1 - ax0) * (ay1 - ay0) + (bx1 - bx0) * (by1 - by0) - inter
    return inter / union if union > 0 else 0.0


def _inside(x: float, y: float, t: _Track) -> bool:
    return abs(x - t.x) <= t.size[0] / 2 + _EPS and abs(y - t.y) <= t.size[1] / 2 + _EPS


def _overlap(a: _Track, b: _Track, b_label: str | None = None) -> bool:
    """The merge rule: footprint IoU >= MERGE_IOU, or, for the same label, one centre inside the other's footprint.
    b_label: b's label to test with (a relabel), default b.label."""
    if _iou((a.x, a.y, a.size), (b.x, b.y, b.size)) >= MERGE_IOU - _EPS:
        return True
    return a.label == (b.label if b_label is None else b_label) and (_inside(a.x, a.y, b) or _inside(b.x, b.y, a))


def _fold(old: _Track, young: _Track) -> None:
    """young merges into old: old keeps its id and its geometry, unless only young has LiDAR support."""
    if young.last_hit_wall is not None and (old.last_hit_wall is None or young.last_hit_wall > old.last_hit_wall):
        old.last_hit_wall = young.last_hit_wall
    if not old.measured and young.measured:
        old.x, old.y, old.size, old.filled = young.x, young.y, young.size, young.filled
        old.measured, old.cam_xy = True, young.cam_xy
        old.point_frames = young.point_frames
