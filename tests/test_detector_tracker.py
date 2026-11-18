"""detector/tracker.py：見え方を追跡にし、確かめ、id を付け、忘れる（spec §11.7、design-detector §8・§16 "Tracker"）。

fuse は geometry.footprint の代わりの小さな関数（点の軸に沿った四角）。時刻はシミュレーション（frame_t）と
壁の時計（wall_t）を 1 秒ずつ進める。カメラは odom の原点にあり、in_view は既定でいつも真。
"""

import ast
import json
import math
from pathlib import Path
from typing import NamedTuple

import numpy as np
import pytest

from voice_ltl import objects, vocabulary
from detector import tracker as tracker_module
from detector.protocol import YOLOE_THRESHOLDS, Thresholds
from detector.tracker import (
    ASSOC_BASE, ASSOC_GAIN, CONF_WINDOW, FALLBACK_EXTRA, M, MAX_ID, MERGE_IOU, MIN_HIT_GAP_S, MISS_UNSUPPORTED, N,
    RELABEL_HITS, RELABEL_RATIO, TENTATIVE_TTL_S, TRACK_POINT_FRAMES, VACANT_FRAMES, VISIBLE_HOLD_MIN_S, XLABEL_IOU,
    Observation, Tracker,
)

ROOT = Path(__file__).resolve().parent.parent
PRIORS = {e.label: e.size for e in vocabulary.VOCABULARY.entries}
BOX = (100.0, 120.0, 220.0, 260.0)          # an image box; the tracker never looks inside it


class FP(NamedTuple):
    x: float
    y: float
    size: tuple
    filled: bool


def aabb_fuse(points, cam_xy, prior, views=None):
    """The stub fuse: the axis-aligned box of the points (at least 0.1 m a side); the single views are unused."""
    if points is None or len(points) == 0:
        return None
    lo, hi = points.min(axis=0), points.max(axis=0)
    return FP(float(lo[0] + hi[0]) / 2, float(lo[1] + hi[1]) / 2,
              (max(float(hi[0] - lo[0]), 0.1), max(float(hi[1] - lo[1]), 0.1)), False)


def square_points(x, y, size, step=0.1):
    """Points on a grid over the rectangle centred at (x, y), edges included."""
    sx, sy = size
    xs = np.linspace(x - sx / 2, x + sx / 2, max(2, int(round(sx / step)) + 1))
    ys = np.linspace(y - sy / 2, y + sy / 2, max(2, int(round(sy / step)) + 1))
    gx, gy = np.meshgrid(xs, ys)
    return np.column_stack([gx.ravel(), gy.ravel()])


def obs(label="chair", conf=0.6, x=2.0, y=0.0, size=None, supported=True, points="grid", cam=(0.0, 0.0),
        filled=False):
    """One Observation; a supported one carries the grid points of its footprint, a fallback one none."""
    size = tuple(PRIORS[label]) if size is None else size
    if isinstance(points, str) and points == "grid":
        points = square_points(x, y, size) if supported else None
    return Observation(label, conf, BOX, x, y, size, filled, points, supported, not supported, cam)


def always(x, y):
    return True


def never(x, y):
    return False


class Run:
    """Drives one Tracker frame by frame: sim and wall time advance together (1 s per frame by default)."""

    def __init__(self, thresholds=YOLOE_THRESHOLDS, fuse=aabb_fuse):
        self.tracker = Tracker(thresholds, PRIORS, fuse)
        self.t = 100.0          # sim time of the last frame
        self.wall = 5000.0      # wall time of the last frame
        self.idle = 3.0

    def frame(self, *observations, in_view=always, dt=1.0):
        self.t += dt
        self.wall += dt
        self.tracker.update(list(observations), self.t, self.wall, in_view)

    def rows(self, at=None):
        return self.tracker.rows(self.wall if at is None else at, self.idle)

    def ids(self):
        return [r.id for r in self.rows()]

    def stats(self):
        return self.tracker.stats()

    def key(self, obj_id):
        """The vacancy key of the confirmed track published as obj_id."""
        x, y = next((r.x, r.y) for r in self.rows() if r.id == obj_id)
        return next(k for k, kx, ky, _size, _label in self.tracker.vacancy_targets()
                    if math.hypot(kx - x, ky - y) < 1e-3)

    def retire(self, obj_id):
        key = self.key(obj_id)
        self.tracker.mark_vacant(key, True, self.t)
        self.tracker.mark_vacant(key, True, self.t + MIN_HIT_GAP_S)
        self.t += MIN_HIT_GAP_S


def confirmed(run, *observations):
    """Two hits 1 s apart of the same observations: enough for supported ones."""
    run.frame(*observations)
    run.frame(*observations)


# --- constants (plan Task 2.2; spec §11.7, §16) -------------------------------------------------------------

def test_constants_are_the_spec_values():
    assert (M, N, MIN_HIT_GAP_S, TENTATIVE_TTL_S) == (2, 3, 0.5, 30.0)
    assert (ASSOC_BASE, ASSOC_GAIN, FALLBACK_EXTRA) == (0.5, 0.5, 0.5)
    assert (XLABEL_IOU, MERGE_IOU, RELABEL_HITS, RELABEL_RATIO) == (0.4, 0.5, 5, 2.0)
    assert (CONF_WINDOW, TRACK_POINT_FRAMES, VACANT_FRAMES, MISS_UNSUPPORTED) == (10, 6, 2, 3)
    assert (MAX_ID, VISIBLE_HOLD_MIN_S) == (99, 5.0)
    assert tracker_module.INVIEW_RANGE == objects.MAX_OBJECT_DISTANCE == 6.0
    assert tracker_module.DET_RANGE == objects.CAMERA_FAR == 8.0


def test_bad_thresholds_are_refused():
    with pytest.raises(ValueError):
        Tracker(Thresholds(0.4, 0.35, 0.5), PRIORS, aabb_fuse)
    with pytest.raises(ValueError):
        Tracker(Thresholds(0.25, 0.35, 0.5, by_label={"unicorn": 0.5}), PRIORS, aabb_fuse)


# --- sightings -------------------------------------------------------------------------------------------------

def test_a_sighting_below_0_25_is_ignored():
    run = Run()
    for _ in range(4):
        run.frame(obs(conf=0.24))
    assert run.ids() == []
    assert run.stats()["tracks_tentative"] == 0


def test_0_30_refreshes_visible_but_creates_no_id():
    run = Run()
    confirmed(run, obs("chair", 0.6))
    for _ in range(7):
        run.frame()                                      # walking: no hits for 7 s (the hold is 6 s)
    assert [(r.id, r.visible) for r in run.rows()] == [("chair_1", False)]
    run.frame(obs("chair", 0.30))
    assert [(r.id, r.visible) for r in run.rows()] == [("chair_1", True)]
    for _ in range(3):                                   # a new place, only 0.30: tentative, never an id
        run.frame(obs("box", 0.30, x=2.0, y=2.0))
    assert run.ids() == ["chair_1"]
    assert run.stats()["tracks_tentative"] == 1


def test_a_new_id_needs_one_hit_at_0_35():
    run = Run()
    run.frame(obs(conf=0.30))
    run.frame(obs(conf=0.34))
    assert run.ids() == []
    run.frame(obs(conf=0.35))
    assert run.ids() == ["chair_1"]


def test_a_by_label_override_raises_confirm_for_that_label_only():
    run = Run(Thresholds(0.25, 0.35, 0.5, by_label={"chair": 0.6}))
    for _ in range(3):
        run.frame(obs("chair", 0.5), obs("box", 0.5, y=2.0))
    assert run.ids() == ["box_1"]
    run.frame(obs("chair", 0.6), obs("box", 0.5, y=2.0))
    assert run.ids() == ["box_1", "chair_1"]


def test_labels_outside_the_vocabulary_never_become_tracks(monkeypatch):
    run = Run()
    for _ in range(3):
        run.frame(obs("chair"), Observation("person", 0.9, BOX, 2.0, 2.0, (0.5, 0.5), False,
                                            square_points(2.0, 2.0, (0.5, 0.5)), True, False, (0.0, 0.0)))
    assert run.ids() == ["chair_1"]
    assert run.stats()["tracks_tentative"] == 0
    monkeypatch.setattr(vocabulary, "LABELS", vocabulary.LABELS - {"chair"})
    assert run.ids() == []                               # re-checked on output (spec §11.7)


def test_sightings_beyond_8_m_are_ignored():
    run = Run()
    for _ in range(3):
        run.frame(obs(x=8.2))
    assert run.stats()["tracks_tentative"] == 0


# --- confirmation ----------------------------------------------------------------------------------------------

def test_one_hit_makes_nothing():
    run = Run()
    run.frame(obs())
    assert run.ids() == []
    assert run.stats()["tracks_tentative"] == 1 and run.stats()["tracks_confirmed"] == 0


def test_two_hits_in_three_in_view_frames_confirm_chair_1():
    run = Run()
    run.frame(obs())
    run.frame()                                          # an in-view miss
    assert run.ids() == []
    run.frame(obs())
    assert run.ids() == ["chair_1"]
    assert run.stats()["tracks_tentative"] == 0 and run.stats()["tracks_confirmed"] == 1


def test_two_identical_frames_half_a_second_apart_count_twice():
    run = Run()
    o = obs()
    run.frame(o, dt=0.5)
    run.frame(o, dt=0.5)
    assert run.ids() == ["chair_1"]


def test_two_hits_within_half_a_second_count_once():
    run = Run()
    run.frame(obs(), dt=1.0)
    run.frame(obs(), dt=0.3)
    assert run.ids() == []
    run.frame(obs(), dt=0.3)                             # 0.6 s after the first hit
    assert run.ids() == ["chair_1"]


def test_frames_out_of_view_do_not_count_toward_n():
    run = Run()
    run.frame(obs())
    run.frame(in_view=never)
    run.frame(in_view=never)
    run.frame(in_view=never)
    run.frame(obs())                                     # the window is [hit, hit]: out-of-view frames skipped
    assert run.ids() == ["chair_1"]


def test_in_view_misses_push_hits_out_of_the_window():
    run = Run()
    run.frame(obs())
    run.frame()
    run.frame()
    run.frame(obs())                                     # the window is [miss, miss, hit]
    assert run.ids() == []


def test_a_tentative_track_whose_window_empties_is_dropped():    # pre-final review F15 (spec §11.7)
    run = Run()
    run.frame(obs())
    assert run.stats()["tracks_tentative"] == 1
    run.frame()
    run.frame()
    run.frame()                                          # three in-view frames without a hit: the window is empty
    assert run.stats()["tracks_tentative"] == 0 and run.ids() == []


def test_a_box_seen_from_two_sides_gets_its_observed_depth():   # pre-final review F3 (spec §11.6)
    """The tracker passes each hit's points with its camera as footprint's views, so a box seen from the front
    and then from the side is published with the depth those views saw (filled False), as geometry's
    test_a_depth_seen_to_its_far_end_is_kept_below_the_prior assumes of the tracker."""
    from detector import geometry as g
    from test_detector_geometry import BASE as STAND, Q0, Q_SIDE, SIDE_BASE, cam_xy, covers, scan, solid

    box = solid(2.175, 0.0, 0.35, 0.35, height=0.45)          # smaller than its prior (0.5, 0.5)
    prior = (0.5, 0.5)
    tracker = Tracker(YOLOE_THRESHOLDS, {**PRIORS, "box": prior}, g.footprint)
    t, wall = 100.0, 5000.0
    for base, q, seed in ((STAND, Q0, 1), ((box[0], SIDE_BASE[1], SIDE_BASE[2]), Q_SIDE, 2)):
        points = g.cut(scan([box], base=base, q=q, seed=seed))[:, :2]
        cam = cam_xy(base, q)
        fp = g.footprint(points, cam, prior)
        assert fp.filled                                       # one face alone shows no depth
        for _ in range(3):
            t, wall = t + 1.0, wall + 1.0
            tracker.update([Observation("box", 0.6, BOX, fp.x, fp.y, fp.size, fp.filled, points, True, False, cam)],
                           t, wall, always)
    rows = tracker.rows(wall, 3.0)
    assert [r.id for r in rows] == ["box_1"] and tracker.stats()["filled"] == []
    row = rows[0]
    assert covers(row, box) and max(row.size) <= 0.35 + 0.16 < 0.5 + 2 * g.EDGE_PAD   # observed, not the prior
    assert (row.x, row.y) == pytest.approx((box[0], box[1]), abs=0.05)



# O12 at the tracker level from two stops (spec §11.6; pre-final review round 2, findings 7 and 8; R-PF3; R-PF3b):
# every cover case the tracker publishes short of the object, by how far the object reaches outside the published
# rectangle (m, its worst corner, from the row's rounded values), for each sweep: (degrees around the object, the
# faces: plain, 2 mm of range noise, or leaning back 0.5°). Before R-PF3 the union's far end was judged from the
# newest camera, whose rays also reach the returns only an older stop saw: 9, 20, 29 and 34 misses at +45, -45, +90
# and -90 (plain), up to 0.177 m. A far end counts as seen only when one single view's own returns reach it at
# <= 60°, otherwise the prior fills it; a far end that several views saw takes the tightest of their bounds (round-3
# review, finding 1). No miss left is short at its far end (one at -45 with noise, 0.0017 m).
# R-PF3b: a lateral side of a union from two stops counts as seen only when one single view's own returns reach it
# the same way; otherwise it reaches as far as the views' own pads past their last returns and the tightest of their
# own footprints there. That closed every lateral miss of an unseen side above 0.05 m: 0.0935 m at -45 and 0.0654 m
# at +90 (plain), and 0.350 m (-45, noise), 0.271 m (±45, lean) and 0.451 m (-45, lean), which existed on 97da92a.
# The rule needs views from more than one stop (geometry.ONE_STOP; test_a_one_stop_union_keeps_its_own_lateral_sides).
# UNION_LATERAL (R-PF3b (a); named UNION_THINNING before it): the misses left within §11.6's 0.05 m thinning step,
# each pinned with its size. Few of them are the thinning. All but two are short on a lateral side, and all but four
# of those on a side that one view saw at <= 60° incidence, which keeps the union's own margin from the newest camera
# (EDGE_PAD where two stops' face ends lie at one lateral coordinate, as if a face lay along that side, or one point
# spacing at the newest camera's incidence). The four are on an unseen side, no larger than the second stop's own
# footprint alone misses there: (2.0, 2.0, 0.8, yaw 30, 1.0 m, 30) at -45 (plain, noise and lean; alone 0.053,
# 0.056 and 0.029 m) and (2.0, 2.0, 0.8, 15, 1.0, 30) at +45 with the lean (alone 0.0011 m). The two others: the
# near side of (2.0, 2.0, 0.8, 0, 1.0, 0) at -90 and the far side of (0.9, 0.45, 1.2, 90, 3.0, 30) at -45 with
# noise. Among them F7's rest, (0.4, 0.3, 0.7, 15, 2.0, 0) at -45 (accepted by R-PF3).
UNION_LATERAL = {
    (45, "plain"): {(1.2, 0.4, 0.45, 75, 1.0, 0): 0.0015},
    (-45, "plain"): {(0.3, 0.3, 0.45, 15, 2.0, 0): 0.0004, (0.4, 0.3, 0.7, 15, 2.0, 0): 0.0025,
                     (1.8, 0.9, 0.85, 90, 1.0, 30): 0.0099, (2.0, 2.0, 0.8, 30, 1.0, 30): 0.0316},
    (90, "plain"): {(0.3, 0.3, 0.45, 75, 3.0, 30): 0.0013, (0.3, 0.3, 0.45, 75, 4.0, 30): 0.0298,
                    (0.4, 0.3, 0.7, 75, 5.0, 30): 0.0243, (0.5, 0.5, 0.5, 45, 5.0, 0): 0.0066,
                    (0.5, 0.5, 0.5, 75, 4.0, 30): 0.0105, (0.5, 0.5, 0.5, 75, 5.0, 30): 0.0440,
                    (0.9, 0.45, 1.2, 45, 4.0, 0): 0.0081, (0.9, 0.45, 1.2, 45, 5.0, 0): 0.0188,
                    (0.9, 0.45, 1.2, 75, 4.0, 30): 0.0455, (1.0, 1.0, 0.8, 75, 3.0, 30): 0.0274,
                    (1.0, 1.0, 0.8, 75, 4.0, 30): 0.0114, (1.2, 0.4, 0.45, 75, 4.0, 30): 0.0167,
                    (1.2, 0.4, 0.45, 75, 5.0, 30): 0.0142, (1.8, 0.9, 0.85, 30, 3.0, 0): 0.0083,
                    (1.8, 0.9, 0.85, 45, 5.0, 0): 0.0142, (1.8, 0.9, 0.85, 60, 3.0, 30): 0.0081,
                    (1.8, 0.9, 0.85, 75, 3.0, 30): 0.0051, (1.8, 0.9, 0.85, 75, 5.0, 30): 0.0496,
                    (2.0, 1.0, 0.5, 75, 5.0, 30): 0.0037, (2.0, 2.0, 0.8, 45, 4.0, 0): 0.0154},
    (-90, "plain"): {(0.3, 0.3, 0.45, 15, 2.0, 30): 0.0139, (0.4, 0.3, 0.7, 0, 5.0, 30): 0.0276,
                     (0.4, 0.3, 0.7, 75, 5.0, 30): 0.0296, (0.5, 0.5, 0.5, 0, 4.0, 30): 0.0024,
                     (0.5, 0.5, 0.5, 45, 5.0, 0): 0.0066, (0.5, 0.5, 0.5, 75, 4.0, 30): 0.0110,
                     (0.5, 0.5, 0.5, 75, 5.0, 30): 0.0345, (0.5, 0.5, 0.5, 90, 4.0, 30): 0.0024,
                     (0.9, 0.45, 1.2, 90, 3.0, 0): 0.0275, (0.9, 0.45, 1.2, 90, 5.0, 30): 0.0172,
                     (1.0, 1.0, 0.8, 75, 3.0, 30): 0.0273, (1.0, 1.0, 0.8, 75, 4.0, 30): 0.0113,
                     (1.2, 0.4, 0.45, 0, 4.0, 30): 0.0072, (1.2, 0.4, 0.45, 75, 2.0, 0): 0.0020,
                     (1.2, 0.4, 0.45, 75, 2.0, 30): 0.0009, (1.2, 0.4, 0.45, 75, 3.0, 30): 0.0065,
                     (1.2, 0.4, 0.45, 90, 1.0, 30): 0.0038, (1.8, 0.9, 0.85, 0, 5.0, 30): 0.0180,
                     (1.8, 0.9, 0.85, 45, 5.0, 0): 0.0031, (1.8, 0.9, 0.85, 75, 3.0, 30): 0.0264,
                     (1.8, 0.9, 0.85, 75, 4.0, 30): 0.0289, (1.8, 0.9, 0.85, 75, 5.0, 30): 0.0384,
                     (2.0, 1.0, 0.5, 45, 4.0, 0): 0.0063, (2.0, 1.0, 0.5, 75, 4.0, 30): 0.0424,
                     (2.0, 1.0, 0.5, 75, 5.0, 30): 0.0285, (2.0, 2.0, 0.8, 0, 1.0, 0): 0.0175,
                     (2.0, 2.0, 0.8, 45, 4.0, 0): 0.0154},
    (45, "noise"): {(0.9, 0.45, 1.2, 90, 3.0, 30): 0.0047, (1.2, 0.4, 0.45, 75, 1.0, 0): 0.0020},
    (-45, "noise"): {(0.4, 0.3, 0.7, 15, 2.0, 0): 0.0000, (0.9, 0.45, 1.2, 90, 3.0, 30): 0.0017,
                     (1.8, 0.9, 0.85, 90, 1.0, 30): 0.0104, (2.0, 2.0, 0.8, 30, 1.0, 30): 0.0271},
    (90, "noise"): {(0.3, 0.3, 0.45, 75, 3.0, 30): 0.0033, (0.3, 0.3, 0.45, 75, 4.0, 30): 0.0288,
                    (0.4, 0.3, 0.7, 60, 5.0, 30): 0.0003, (0.4, 0.3, 0.7, 75, 5.0, 30): 0.0293,
                    (0.5, 0.5, 0.5, 45, 5.0, 0): 0.0061, (0.5, 0.5, 0.5, 75, 4.0, 30): 0.0095,
                    (0.5, 0.5, 0.5, 75, 5.0, 30): 0.0415, (0.9, 0.45, 1.2, 45, 4.0, 0): 0.0073,
                    (0.9, 0.45, 1.2, 45, 5.0, 0): 0.0176, (0.9, 0.45, 1.2, 75, 4.0, 30): 0.0460,
                    (1.0, 1.0, 0.8, 75, 3.0, 30): 0.0274, (1.0, 1.0, 0.8, 75, 4.0, 30): 0.0104,
                    (1.2, 0.4, 0.45, 75, 4.0, 30): 0.0157, (1.2, 0.4, 0.45, 75, 5.0, 30): 0.0167,
                    (1.8, 0.9, 0.85, 30, 3.0, 0): 0.0073, (1.8, 0.9, 0.85, 45, 5.0, 0): 0.0131,
                    (1.8, 0.9, 0.85, 60, 3.0, 30): 0.0081, (1.8, 0.9, 0.85, 75, 3.0, 30): 0.0036,
                    (1.8, 0.9, 0.85, 75, 5.0, 30): 0.0476, (2.0, 1.0, 0.5, 75, 5.0, 30): 0.0037,
                    (2.0, 2.0, 0.8, 45, 4.0, 0): 0.0119},
    (-90, "noise"): {(0.3, 0.3, 0.45, 15, 2.0, 30): 0.0119, (0.3, 0.3, 0.45, 75, 4.0, 30): 0.0165,
                     (0.4, 0.3, 0.7, 0, 5.0, 30): 0.0251, (0.4, 0.3, 0.7, 75, 5.0, 30): 0.0311,
                     (0.5, 0.5, 0.5, 0, 4.0, 30): 0.0034, (0.5, 0.5, 0.5, 45, 5.0, 0): 0.0061,
                     (0.5, 0.5, 0.5, 75, 4.0, 30): 0.0095, (0.5, 0.5, 0.5, 75, 5.0, 30): 0.0350,
                     (0.5, 0.5, 0.5, 90, 4.0, 30): 0.0034, (0.9, 0.45, 1.2, 90, 3.0, 0): 0.0250,
                     (0.9, 0.45, 1.2, 90, 5.0, 30): 0.0177, (1.0, 1.0, 0.8, 75, 3.0, 30): 0.0268,
                     (1.0, 1.0, 0.8, 75, 4.0, 30): 0.0093, (1.2, 0.4, 0.45, 75, 2.0, 0): 0.0010,
                     (1.2, 0.4, 0.45, 75, 3.0, 30): 0.0030, (1.2, 0.4, 0.45, 90, 1.0, 30): 0.0008,
                     (1.8, 0.9, 0.85, 0, 5.0, 30): 0.0165, (1.8, 0.9, 0.85, 45, 5.0, 0): 0.0021,
                     (1.8, 0.9, 0.85, 75, 3.0, 30): 0.0284, (1.8, 0.9, 0.85, 75, 4.0, 30): 0.0274,
                     (1.8, 0.9, 0.85, 75, 5.0, 30): 0.0369, (2.0, 1.0, 0.5, 45, 4.0, 0): 0.0033,
                     (2.0, 1.0, 0.5, 75, 4.0, 30): 0.0429, (2.0, 1.0, 0.5, 75, 5.0, 30): 0.0270,
                     (2.0, 2.0, 0.8, 0, 1.0, 0): 0.0160, (2.0, 2.0, 0.8, 45, 4.0, 0): 0.0137},
    (45, "lean"): {(0.5, 0.5, 0.5, 15, 1.0, 30): 0.0039, (0.9, 0.45, 1.2, 90, 3.0, 30): 0.0042,
                   (0.9, 0.45, 1.2, 90, 4.0, 30): 0.0002, (2.0, 2.0, 0.8, 0, 4.0, 30): 0.0186,
                   (2.0, 2.0, 0.8, 15, 1.0, 30): 0.0009, (2.0, 2.0, 0.8, 90, 4.0, 30): 0.0186},
    (-45, "lean"): {(1.8, 0.9, 0.85, 90, 1.0, 30): 0.0084, (2.0, 2.0, 0.8, 30, 1.0, 30): 0.0246},
    (90, "lean"): {(0.3, 0.3, 0.45, 75, 3.0, 30): 0.0008, (0.4, 0.3, 0.7, 75, 5.0, 30): 0.0233,
                   (0.5, 0.5, 0.5, 45, 5.0, 0): 0.0066, (0.5, 0.5, 0.5, 75, 4.0, 30): 0.0090,
                   (0.5, 0.5, 0.5, 75, 5.0, 30): 0.0415, (0.9, 0.45, 1.2, 45, 4.0, 0): 0.0031,
                   (0.9, 0.45, 1.2, 45, 5.0, 0): 0.0186, (0.9, 0.45, 1.2, 75, 4.0, 30): 0.0430,
                   (1.0, 1.0, 0.8, 75, 3.0, 30): 0.0264, (1.0, 1.0, 0.8, 75, 4.0, 30): 0.0089,
                   (1.2, 0.4, 0.45, 75, 4.0, 30): 0.0152, (1.2, 0.4, 0.45, 75, 5.0, 30): 0.0117,
                   (1.8, 0.9, 0.85, 30, 3.0, 0): 0.0058, (1.8, 0.9, 0.85, 45, 5.0, 0): 0.0141,
                   (1.8, 0.9, 0.85, 60, 3.0, 30): 0.0056, (1.8, 0.9, 0.85, 75, 3.0, 30): 0.0001,
                   (1.8, 0.9, 0.85, 75, 5.0, 30): 0.0476, (2.0, 1.0, 0.5, 15, 4.0, 0): 0.0022,
                   (2.0, 1.0, 0.5, 75, 5.0, 30): 0.0017, (2.0, 2.0, 0.8, 45, 4.0, 0): 0.0142},
    (-90, "lean"): {(0.3, 0.3, 0.45, 15, 2.0, 30): 0.0144, (0.4, 0.3, 0.7, 0, 5.0, 30): 0.0241,
                    (0.4, 0.3, 0.7, 75, 5.0, 30): 0.0281, (0.5, 0.5, 0.5, 0, 4.0, 30): 0.0004,
                    (0.5, 0.5, 0.5, 45, 5.0, 0): 0.0066, (0.5, 0.5, 0.5, 75, 4.0, 30): 0.0085,
                    (0.5, 0.5, 0.5, 75, 5.0, 30): 0.0330, (0.5, 0.5, 0.5, 90, 4.0, 30): 0.0004,
                    (0.9, 0.45, 1.2, 75, 5.0, 0): 0.0051, (0.9, 0.45, 1.2, 90, 3.0, 0): 0.0270,
                    (0.9, 0.45, 1.2, 90, 5.0, 30): 0.0157, (1.0, 1.0, 0.8, 75, 3.0, 30): 0.0273,
                    (1.0, 1.0, 0.8, 75, 4.0, 30): 0.0088, (1.2, 0.4, 0.45, 0, 4.0, 30): 0.0057,
                    (1.2, 0.4, 0.45, 75, 2.0, 0): 0.0010, (1.2, 0.4, 0.45, 75, 2.0, 30): 0.0009,
                    (1.2, 0.4, 0.45, 75, 3.0, 30): 0.0035, (1.2, 0.4, 0.45, 90, 1.0, 30): 0.0038,
                    (1.8, 0.9, 0.85, 0, 5.0, 30): 0.0135, (1.8, 0.9, 0.85, 75, 3.0, 30): 0.0234,
                    (1.8, 0.9, 0.85, 75, 4.0, 30): 0.0239, (1.8, 0.9, 0.85, 75, 5.0, 30): 0.0324,
                    (2.0, 1.0, 0.5, 15, 4.0, 30): 0.0090, (2.0, 1.0, 0.5, 45, 4.0, 0): 0.0047,
                    (2.0, 1.0, 0.5, 75, 4.0, 30): 0.0404, (2.0, 1.0, 0.5, 75, 5.0, 30): 0.0265,
                    (2.0, 2.0, 0.8, 45, 4.0, 0): 0.0142},
}
# Larger than the thinning step: R-PF3b (a) accepts it as a documented O12 limit of two-stop unions, with these
# numbers (the same on 97da92a; not hidden by wider pads, R-PF3). A lateral side that the first stop saw only as a
# face ending there at <= 60° (its last return 0.121 m short of the end, one point spacing 0.206 m there at 5.4 m),
# where both stops' face ends lie at one lateral coordinate, which the union takes as a face along that side, so
# the union's own margin is EDGE_PAD, while each view alone bounds that side 0.19-0.21 m out. The tightest bound of
# the views that saw it would cover it, but the same evidence (a face end seen at <= 60° from about 5 m) cannot
# tell it from (2.0, 1.0, 0.5, 75, 4.0, 30), whose side was observed to within 1 mm: that bound widens sides
# observed to their end by up to 0.125 m and loses rows at the 6 m window (tracker3-report.md, R-PF3b).
LATERAL_FROM_NEWEST = {
    (90, "plain"): {(2.0, 2.0, 0.8, 75, 4.0, 30): 0.0554},
    (90, "noise"): {(2.0, 2.0, 0.8, 75, 4.0, 30): 0.0539},
    (90, "lean"): {(2.0, 2.0, 0.8, 75, 4.0, 30): 0.0514},
}
# The +45 case where F3's thinned views missed by 6 mm, which the old call (no views) covered (round 2, finding 7)
VIEWS_MISSED = ((0.9, 0.45, 1.2, 0, 1.0, 30), 45, "plain")
# The cases published, of 630 (the rest: the object's centre beyond 6 m of a camera), as on 97da92a. A far pad
# taken as the largest of the views' margins lost one at ±90: (2.0, 1.0, 0.5, yaw 45, 5.0 m, 0), whose centre it
# moved past 6 m of the second camera (round-3 review, finding 1)
PUBLISHED = {(45, "plain"): 610, (-45, "plain"): 609, (90, "plain"): 607, (-90, "plain"): 607,
             (45, "noise"): 610, (-45, "noise"): 609, (90, "noise"): 608, (-90, "noise"): 607,
             (45, "lean"): 610, (-45, "lean"): 610, (90, "lean"): 607, (-90, "lean"): 607}
TWO_STOP_SWEEPS = [(deg, faces) for faces in ("plain", "noise", "lean") for deg in (45, -45, 90, -90)]


def _outside(row, s):
    """How far the solid s reaches outside the row's rectangle (m, its worst corner; 0 when covered)."""
    from test_detector_geometry import corners_xy

    c = corners_xy(s)
    return max(0.0, float((np.abs(c[:, 0] - row.x) - row.size[0] / 2).max()),
               float((np.abs(c[:, 1] - row.y) - row.size[1] / 2).max()))


@pytest.mark.parametrize("deg, faces", TWO_STOP_SWEEPS)
def test_two_stops_miss_only_the_pinned_residuals(deg, faces):   # R-PF3, R-PF3b (spec §11.6, O12)
    """O12 at the tracker level: the 630 cover cases, 3 hits from BASE and then 3 from a stop turned ±45° or ±90°
    around the object, fused by geometry.footprint with each hit as a raw view; plain faces, 2 mm of range noise
    (seeded per hit) or every side face leaning back 0.5°. Every published case covers the object except
    UNION_LATERAL (each <= THIN_CELL, by its pinned size) and the pinned LATERAL_FROM_NEWEST; no far end that only
    an older stop grazed is taken as seen any more, and no lateral side that no single view saw. The case of
    VIEWS_MISSED covers, and as many cases are published as on 97da92a (PUBLISHED)."""
    from detector import geometry as g
    from test_detector_geometry import BASE as STAND, Q0, _cover_cases, _cover_solid, around, cam_xy, scan

    lean, noise = (0.5 if faces == "lean" else 0.0), (0.002 if faces == "noise" else 0.0)
    missed, published = {}, 0
    for case in _cover_cases():
        prior = case[:2]
        s = _cover_solid(*case)
        tracker = Tracker(YOLOE_THRESHOLDS, {**PRIORS, "box": prior}, g.footprint)
        t, wall, seed = 100.0, 5000.0, 0
        for base, q in ((STAND, Q0), around(s, deg)):
            cam = cam_xy(base, q)
            for _ in range(3):
                seed += 1
                points = g.cut(scan([s], base=base, q=q, seed=seed, lean_deg=lean, noise=noise))[:, :2]
                if len(points) < g.MIN_CLUSTER_POINTS:
                    continue
                fp = g.footprint(points, cam, prior)
                t, wall = t + 1.0, wall + 1.0
                tracker.update([Observation("box", 0.6, BOX, fp.x, fp.y, fp.size, fp.filled, points, True, False,
                                            cam)], t, wall, always)
        rows = tracker.rows(wall, 3.0)
        if (case, deg, faces) == VIEWS_MISSED:
            assert rows
        if not rows:                         # the object's centre beyond 6 m of a camera: no frame counted
            continue
        published += 1
        size = _outside(rows[0], s)
        if size > 1e-9:
            missed[case] = round(size, 4)
    thin, wide = UNION_LATERAL.get((deg, faces), {}), LATERAL_FROM_NEWEST.get((deg, faces), {})
    assert published == PUBLISHED[deg, faces]
    assert sorted(missed) == sorted({**thin, **wide})
    assert missed == pytest.approx({**thin, **wide}, abs=5e-4)
    assert max(thin.values(), default=0.0) <= tracker_module.THIN_CELL
    assert min(wide.values(), default=1.0) > tracker_module.THIN_CELL


# One stop, the tracker's usual case (every hit taken from one standstill): R-G4's cabinet scenes (geometry's
# test_partly_hidden_furniture_claims_no_small_seen_footprint) through the tracker, 6 hits from BASE, the base
# swaying on a circle of `sway` m around BASE from hit to hit (0: still). R-PF3b's cause, another stop's grazed face
# ends, does not exist there, but its lateral rule applied all the same: the thinned union and a raw single view can
# take different face axes as their depth, and the view's footprint alone, filled with the prior along its own depth
# axis, then set a lateral side of the union. ('sofa', 40, 2.5, 1.2, 1.0) grew from 2.063 x 1.945 to 2.677 x
# 2.677 m, its lateral margin from -0.005 to +0.950 m (the review of 67f9d75, finding 1; 9 rows of that sweep grew by
# more than 0.05 m, up to 0.73 m; 0.70 m with noise, 0.94 m with the lean). Pinned: the rows of 3cc9d29, which cover
# each object; with the rule the first three were 2.677 x 2.677, 2.677 x 2.677 and 1.831 x 1.831 m (67f9d75), and
# with the rule bounded by the prior's other side (the review of 8891c46, finding 3) they are 2.098 x 1.987,
# 2.098 x 1.987 and 1.831 x 1.831 m. The first three are 97da92a's rows too. The fourth is not: from one stop too the
# far end is judged from the views (R-PF3), and 97da92a gave it 1.494 x 1.493 m.
ONE_STOP_ROWS = {   # (label, yaw, distance, cabinet width, cabinet y, faces, sway): (x, y, sx, sy)
    ("sofa", 40, 2.5, 1.2, 1.0, "plain", 0.0): (2.482, 0.021, 2.063, 1.945),
    ("sofa", 40, 3.0, 1.2, 0.9, "plain", 0.002): (2.992, 0.010, 2.082, 1.968),
    ("table", 50, 3.0, 1.2, 0.7, "lean", 0.0): (2.980, 0.017, 1.446, 1.509),
    ("table", 45, 3.0, 0.6, -0.7, "lean", 0.0): (2.989, 0.019, 1.498, 1.497),
    # a standing Go2's odometry wanders a few cm (the review of 8891c46, finding 2): a 3 cm sway puts the cameras
    # up to 6 cm from the newest, still one stop (the rows of 97da92a). With ONE_STOP below that the rule applies:
    # the table's row becomes 1.489 x 1.585 m. The bed's became 2.959 x 2.959 m on 1a09cba (neither face axis is
    # observed longer than S + EDGE_PAD, and the views of the 2 x 1 m bed, seen only partly, filled L along the
    # union's lateral axis); since R-PF3c such a union takes only the views' own pads, and its row is the same
    ("bed", 40, 2.5, 0.9, 0.6, "plain", 0.03): (2.502, -0.003, 2.321, 2.198),
    ("table", 55, 3.0, 0.9, 0.3, "plain", 0.03): (3.006, -0.004, 1.434, 1.547),
}


def test_a_one_stop_union_keeps_its_own_lateral_sides():   # R-PF3b (the review of 67f9d75, finding 1)
    """A union whose views' cameras all lie within geometry.ONE_STOP of the newest camera comes from one stop: its
    lateral sides keep the union's own margins from that camera, as every view's rays are that camera's (the far end
    is still judged from the views, R-PF3). A sway of 2 mm or 3 cm (a standing Go2's odometry wanders a few cm) is
    still one stop."""
    from detector import geometry as g
    from test_detector_geometry import BASE as STAND, cam_xy, covers, scan, solid

    rows_of = {}
    for scene in ONE_STOP_ROWS:
        label, yaw, dist, width, cabinet_y, faces, sway = scene
        L, S = vocabulary.size_of(label)
        obj = solid(dist, 0.0, L, S, yaw_deg=yaw, height=vocabulary.height_of(label))
        cabinet = solid(1.6, cabinet_y, 0.45, width, height=vocabulary.height_of("cabinet"))
        tracker = Tracker(YOLOE_THRESHOLDS, {**PRIORS, "box": (L, S)}, g.footprint)
        t, wall, cams = 100.0, 5000.0, []
        for seed in range(1, 7):
            turn = math.radians(60.0 * seed)
            base = (STAND[0] + sway * math.cos(turn), STAND[1] + sway * math.sin(turn), STAND[2])
            cam = cam_xy(base)
            pts, k = scan([obj, cabinet], base=base, which=True, lean_deg=0.5 if faces == "lean" else 0.0, seed=seed)
            p = pts[(pts[:, 2] >= g.GROUND_Z) & (pts[:, 2] <= g.HEIGHT_CAP) & (k == 0)][:, :2]
            assert len(p) >= g.MIN_CLUSTER_POINTS
            fp = g.footprint(p, cam, (L, S))
            t, wall = t + 1.0, wall + 1.0
            tracker.update([Observation("box", 0.6, BOX, fp.x, fp.y, fp.size, fp.filled, p, True, False, cam)],
                           t, wall, always)
            cams.append(cam)
        # a sway of 2 mm or 3 cm: the cameras lie up to 4 mm or 6 cm apart, so one stop is not one point
        assert max(math.dist(a, b) for a in cams for b in cams) == pytest.approx(2 * sway, abs=1e-9)
        rows = tracker.rows(wall, 3.0)
        assert len(rows) == 1, (scene, rows)
        rows_of[scene] = (rows[0].x, rows[0].y, *rows[0].size)
        x, y, sx, sy = ONE_STOP_ROWS[scene]
        assert covers(FP(x, y, (sx, sy), False), obj), scene      # the pinned row covers the object
    wrong = {scene: row for scene, row in rows_of.items() if row != pytest.approx(ONE_STOP_ROWS[scene], abs=1.5e-3)}
    assert wrong == {}


def _cabinet_scene(label, yaw, dist, width, cabinet_y):
    """R-G4's cabinet scene: the object (its vocabulary size, turned yaw°, its centre dist m ahead) and a cabinet
    (0.45 m deep, width m wide, at (1.6, cabinet_y)) between it and the Go2; and the object's prior."""
    from test_detector_geometry import solid

    L, S = vocabulary.size_of(label)
    return ([solid(dist, 0.0, L, S, yaw_deg=yaw, height=vocabulary.height_of(label)),
             solid(1.6, cabinet_y, 0.45, width, height=vocabulary.height_of("cabinet"))], (L, S))


def _stops(s, move):
    """The two stops of a scene around the solid s: ("around", deg) BASE, then a stop turned deg around the object
    (geometry's around); ("step", m) BASE, then BASE moved m toward the object's centre, facing it; ("ahead", m)
    BASE, then BASE moved m straight ahead (B to Bf); ("turn", deg) BASE turned -deg/2, then +deg/2 in place."""
    from test_detector_geometry import BASE as STAND, Q0, around, yaw_q

    kind, x = move
    if kind == "around":
        return [(STAND, Q0), around(s, x)]
    if kind == "turn":
        return [(STAND, yaw_q(-x / 2.0)), (STAND, yaw_q(x / 2.0))]
    if kind == "ahead":
        return [(STAND, Q0), ((STAND[0] + x, STAND[1], STAND[2]), Q0)]
    u = (np.array(s[:2]) - np.array(STAND[:2])) / math.hypot(s[0] - STAND[0], s[1] - STAND[1])
    return [(STAND, Q0), ((STAND[0] + x * u[0], STAND[1] + x * u[1], STAND[2]),
                          yaw_q(math.degrees(math.atan2(u[1], u[0]))))]


def _two_stop_row(solids, prior, stops, lean=0.0):
    """The row the tracker publishes for solids[0] (_two_stop_rows; it publishes one), and how far the cameras lie
    from the newest."""
    rows, apart = _two_stop_rows(solids, prior, stops, lean)
    assert len(rows) == 1, rows
    return rows[0], apart


def _two_stop_rows(solids, prior, stops, lean=0.0, noise=0.0):
    """Every row the tracker publishes for solids[0] (its own returns only), in id order, as (x, y, sx, sy): 3 hits
    from each stop, each hit a raw view fused by geometry.footprint, as in the two-stop sweeps (range noise seeded
    per hit); and how far the cameras lie from the newest."""
    from detector import geometry as g
    from test_detector_geometry import cam_xy, scan

    tracker = Tracker(YOLOE_THRESHOLDS, {**PRIORS, "box": prior}, g.footprint)
    t, wall, seed, cams = 100.0, 5000.0, 0, []
    for base, q in stops:
        cam = cam_xy(base, q)
        for _ in range(3):
            seed += 1
            pts, k = scan(solids, base=base, q=q, which=True, lean_deg=lean, noise=noise, seed=seed)
            p = pts[(pts[:, 2] >= g.GROUND_Z) & (pts[:, 2] <= g.HEIGHT_CAP) & (k == 0)][:, :2]
            assert len(p) >= g.MIN_CLUSTER_POINTS
            fp = g.footprint(p, cam, prior)
            t, wall = t + 1.0, wall + 1.0
            tracker.update([Observation("box", 0.6, BOX, fp.x, fp.y, fp.size, fp.filled, p, True, False, cam)],
                           t, wall, always)
            cams.append(cam)
    rows = [(r.x, r.y, *r.size) for r in tracker.rows(wall, 3.0)]
    return rows, max(math.dist(c, cams[-1]) for c in cams)


# Two stops a short move apart are two stops (geometry.ONE_STOP; the review of 8891c46, finding 2): a 0.5 m step
# toward the object (B to Bf, R-H3a), a 0.15 m step, and a 96° turn in place (R-H3e; the camera, 0.327 m ahead of
# the base, moves 0.486 m). Behind a cabinet, or with leaning faces, the thinned union and the single views take
# different face axes as their depth there too, and R-PF3b's lateral rule completes the side that no view saw: the
# first row (a 2 m square, leaning faces) misses by 0.6085 m without it (ONE_STOP above the move, or 97da92a).
# The other unions observe neither face axis longer than S + EDGE_PAD (0.522 and 0.863 m of a 1.8 x 0.9 m sofa in
# the fifth), so they cannot tell their long side. 1a09cba let an unseen side take L - w_obs + EDGE_PAD there, from
# the views' footprints filled with L along the union's lateral axis: L x L, the box of both orientations. That
# covered the second to fourth, which miss by 0.9418, 0.4913 and 0.6512 m on 97da92a and 3cc9d29, but it grew sides
# whose returns end within EDGE_PAD of the object by up to 1.10 m and lost rows (the review of 1a09cba, findings 1
# and 2): the fifth was 2.677 x 2.677 m (1.971 x 2.084 m on 3cc9d29), and in the sixth and seventh it bought
# nothing: 2.638 x 2.638 and 2.937 x 2.938 m, missing as on 3cc9d29 (0.2113 and 0.6947 m), each without its second
# row. Narrowed (R-PF3c): such a union takes only the views' own pads there, unless its prior is square. The rows
# are 3cc9d29's or within 0.03 m of them, and the second to fourth miss again, by no more than on 3cc9d29.
SHORT_MOVE_ROWS = {   # (scene, move, lean): (every row in id order as (x, y, sx, sy), the best row's miss in m)
    ((2.0, 2.0, 0.8, 60, 1.0, 30), ("step", 0.5), 0.5): ([(2.329, 1.184, 2.867, 2.868)], 0.0),
    (("sofa", 35, 2.5, 0.9, 0.6), ("turn", 96), 0.0): ([(2.000, -0.360, 1.130, 1.215)], 0.9303),
    (("sofa", 35, 2.5, 0.9, 0.7), ("step", 0.5), 0.0): ([(2.358, -0.649, 1.845, 2.085)], 0.4913),
    (("sofa", 40, 2.5, 0.6, -0.1), ("step", 0.15), 0.0): ([(3.147, -0.045, 1.958, 2.073)], 0.6467),
    (("sofa", 50, 2.5, 0.9, 0.5), ("step", 0.5), 0.0): ([(2.498, 0.002, 1.984, 2.095)], 0.0),
    (("sofa", 35, 2.5, 0.6, 0.0), ("step", 0.5), 0.0): ([(2.585, -0.130, 1.942, 1.607),
                                                         (2.252, -1.236, 1.228, 1.687)], 0.2113),
    (("bed", 55, 2.5, 1.2, -0.3), ("step", 0.5), 0.0): ([(3.280, 0.252, 2.137, 1.837),
                                                         (3.140, 1.652, 1.503, 1.737)], 0.6947),
}


def _rows_match(rows, pinned):
    return len(rows) == len(pinned) and all(r == pytest.approx(p, abs=1.5e-3) for r, p in zip(rows, pinned))


def test_two_stops_a_short_move_apart_are_two_stops():   # R-PF3b (the review of 8891c46, findings 1 and 2), R-PF3c
    """A union whose cameras lie a step or a turn in place apart (0.15-0.56 m) is not one stop: the lateral rule
    applies (SHORT_MOVE_ROWS: every row, and how far the object reaches outside the best one). With ONE_STOP at or
    above the move the rows are 97da92a's, and the first misses by 0.6085 m."""
    from detector import geometry as g
    from test_detector_geometry import _cover_solid

    for (scene, move, lean), (pinned, miss) in SHORT_MOVE_ROWS.items():
        if isinstance(scene[0], str):
            solids, prior = _cabinet_scene(*scene)
        else:
            solids, prior = [_cover_solid(*scene)], scene[:2]
        rows, apart = _two_stop_rows(solids, prior, _stops(solids[0], move), lean)
        assert apart > g.ONE_STOP and apart == pytest.approx(move[1] if move[0] == "step" else 0.486, abs=0.07)
        assert _rows_match(rows, pinned), (scene, move, rows)
        best = min(_outside(FP(*p[:2], p[2:], False), solids[0]) for p in pinned)
        assert best == pytest.approx(miss, abs=5e-4), (scene, move, best)


# A union from two stops that observes neither face axis longer than S + EDGE_PAD cannot tell its long side (the
# review of 1a09cba, finding 1). After a second stop 45° around R-G4's cabinet, 1a09cba let its unseen side take
# L - w_obs + EDGE_PAD: the 1.2 x 0.75 m table (0.446 and 0.648 m observed, leaning faces) grew to 1.831 x 1.831 m and
# took the second stop's later returns, so its second row, the only one covering the table's +y side (by 0.043 m),
# was never made, and the table's +y side was missed by 0.507 m; the 2 x 1 m bed's side, whose returns reach within
# 2 mm of its edge, grew by 0.72 m (2.959 x 2.959 m). Now such a union takes only the views' own pads (R-PF3c), and
# the rows are those of 97da92a and 3cc9d29.
AMBIGUOUS_ROWS = {   # (cabinet scene, move, lean): every row in id order, as (x, y, sx, sy)
    (("table", 50, 3.0, 1.2, 0.7), ("around", -45), 0.5): [(2.503, -0.566, 1.460, 1.520),
                                                          (3.414, 0.150, 1.253, 1.188)],
    (("bed", 40, 2.5, 0.9, 0.6), ("around", -45), 0.0): [(1.426, -0.937, 2.351, 2.234),
                                                        (3.338, 0.056, 1.812, 2.029)],
    # the views' pads stop at S - w_obs + EDGE_PAD here (one point spacing past their last return would give
    # 1.637 x 1.629 m). 1a09cba's L made this one row of 2.383 x 2.151 m, which covered the bed; it misses by
    # 0.6804 m again, as on 97da92a and 3cc9d29
    (("bed", 35, 3.0, 1.2, 0.9), ("around", -45), 0.0): [(2.625, -0.296, 1.601, 1.603),
                                                        (3.925, 0.041, 1.605, 2.031)],
}


def test_a_union_that_cannot_tell_its_long_side_takes_only_the_views_pads():   # R-PF3c (the review of 1a09cba)
    """An unseen lateral side of a union that observed neither face axis longer than S + EDGE_PAD (a non-square
    prior) reaches only as far as the views' own pads, capped at S - w_obs + EDGE_PAD: their footprints, filled with
    the prior along their own depth axes, say nothing about which side is long. The rows are pinned
    (AMBIGUOUS_ROWS), and each side of the table is covered by one of its rows."""
    from test_detector_geometry import corners_xy

    for (scene, move, lean), pinned in AMBIGUOUS_ROWS.items():
        solids, prior = _cabinet_scene(*scene)
        rows, _ = _two_stop_rows(solids, prior, _stops(solids[0], move), lean)
        assert _rows_match(rows, pinned), (scene, move, rows)
        if scene[0] == "table":
            c = corners_xy(solids[0])
            reach = [[c[:, 0].min() - (x - sx / 2), x + sx / 2 - c[:, 0].max(),
                      c[:, 1].min() - (y - sy / 2), y + sy / 2 - c[:, 1].max()] for x, y, sx, sy in rows]
            assert all(max(r[i] for r in reach) >= 0.0 for i in range(4)), reach


# What stays against 3cc9d29 in R-G4's cabinet scenes after R-PF3c (UNION_LATERAL's counterpart there, R-PF3b (a)),
# all from R-PF3b's lateral rule before R-PF3c and the same on 1a09cba, every one pinned with every row (R-PF3d).
# Rows lost: a row grows over a second row's hits, which then join it. In the sweeps 18 scenes publish one row fewer
# than on 3cc9d29 (every one also on 1a09cba; behind the cabinet a step toward the object and a step straight ahead
# give the same scans, so 11 are distinct). In none is a side of the object left that a row of 3cc9d29 covered: in 8
# the best row's miss closes or shrinks (the sofa around -45: two partial rows that missed by 0.4308-0.5268 m at
# best become one that covers; the sofa ('sofa', 40, 2.5, 1.2, -0.4) after a step: 0.6607 -> 0.6467 m, with noise
# 0.6612 -> 0.6447 m; the bed around -45, leaning: 0.5884 m -> 0), and in the other 10 the lost row was a stray
# piece that missed the object by more than 1.3 m, while the best row's miss changes by at most 0.5 mm (0.0902 ->
# 0.0897 m, 0.0547 -> 0.0552 m, the bed 0 -> 0). A seen side grown: an unseen side gets the views' pad past the true
# edge, where 3cc9d29 had 0.016 m (the bed around +90, leaning: +0.054 m on a side whose returns reach within 3.5 mm
# of the edge, the only side in the vocabulary sweeps grown by more than 0.05 m whose returns reach within 5 mm; and
# one of the 28 rows of the bed's +90 sweep with noise that grow by 0.050-0.051 m on a side their returns stop
# 49.5 mm short of: 3cc9d29's row is (3.474, -0.027, 2.216, 2.216)). The second value is the best row's miss on
# 3cc9d29 (m).
UNION_LATERAL_ROWS = {   # (cabinet scene, move, faces): (every row in id order as (x, y, sx, sy), 3cc9d29's best miss)
    (("sofa", 35, 3.0, 1.2, 0.9), ("around", -45), "plain"): ([(3.008, -0.011, 2.145, 1.931)], 0.5268),
    (("sofa", 35, 3.0, 1.2, 0.9), ("around", -45), "noise"): ([(3.007, -0.014, 2.147, 1.934)], 0.5263),
    (("sofa", 35, 3.0, 1.2, 0.9), ("around", -45), "lean"): ([(3.009, -0.012, 2.147, 1.933)], 0.4308),
    **{(scene, (kind, 0.5), "plain"): rows for kind in ("step", "ahead") for scene, rows in (
        (("sofa", 40, 2.5, 0.6, 0.0), ([(2.550, -0.059, 1.928, 1.785)], 0.0902)),
        (("sofa", 40, 2.5, 1.2, -0.4), ([(3.147, -0.045, 1.958, 2.073)], 0.6607)),
        (("sofa", 40, 2.5, 1.2, 0.3), ([(2.550, -0.059, 1.928, 1.785)], 0.0902)),
        (("sofa", 40, 3.0, 0.6, 0.1), ([(3.046, -0.055, 1.980, 1.846)], 0.0547)),
        (("sofa", 40, 3.0, 1.2, 0.4), ([(3.046, -0.055, 1.980, 1.846)], 0.0547)),
        (("bed", 45, 3.0, 0.9, 0.2), ([(3.030, -0.030, 2.245, 2.245)], 0.0)))},
    **{(("sofa", 40, 2.5, 1.2, -0.4), (kind, 0.5), "noise"): ([(3.147, -0.043, 1.962, 2.077)], 0.6612)
       for kind in ("step", "ahead")},
    (("bed", 35, 3.0, 1.2, 0.9), ("around", -45), "lean"): ([(3.017, -0.024, 2.384, 2.153)], 0.5884),
    (("bed", 50, 3.0, 0.6, 0.1), ("around", 90), "lean"): ([(3.001, 0.000, 2.191, 2.315)], 0.0),
    (("bed", 45, 3.5, 0.6, -1.0), ("around", 90), "noise"): ([(3.499, -0.001, 2.267, 2.267)], 0.0),
}


def test_the_lateral_rules_residuals_behind_a_cabinet_are_pinned():   # R-PF3c, R-PF3d (R-PF3b (a))
    """UNION_LATERAL_ROWS: the rows that still differ from 3cc9d29's by a lost row or a seen side grown by more than
    0.05 m, each pinned with every row; the best row misses by no more than 3cc9d29's best (row rounding aside)."""
    assert len(UNION_LATERAL_ROWS) == 20
    for (scene, move, faces), (pinned, base_miss) in UNION_LATERAL_ROWS.items():
        solids, prior = _cabinet_scene(*scene)
        rows, _ = _two_stop_rows(solids, prior, _stops(solids[0], move), 0.5 if faces == "lean" else 0.0,
                                 0.002 if faces == "noise" else 0.0)
        assert _rows_match(rows, pinned), (scene, move, faces, rows)
        best = min(_outside(FP(*p[:2], p[2:], False), solids[0]) for p in pinned)
        assert best <= base_miss + 1e-3, (scene, move, faces, best)


# An unseen lateral side reaches no farther than the prior allows (R-PF3b; the review of 8891c46, finding 3). Behind
# a cabinet every single view of a 2 x 1 m bed saw only part of its long face, so each filled L along its own depth
# axis (R-G3), which runs along the union's lateral axis. The union saw the short face to its end, grazed beyond
# 60° (so that side was unseen), and the tightest view set it: the union became an L x L square (2.959 x 2.959 and
# 2.987 x 2.983 m), which an L x S object rules out once the union's observed depth (1.38 and 1.98 m) is longer
# than S. Pinned: the first is the row of 97da92a and 3cc9d29; the second is theirs (2.334 x 2.204 m) but for the
# unseen side's pad, one point spacing past the views' last return. The bound holds the views' own pads too: in the
# third the union saw 1.787 m of a 1.8 m sofa's long face along its lateral axis, so 0.013 m can be left there,
# while each view's pad there (one point spacing at 3.5 m and beyond 60°) is 0.139 m, which grew the row from
# 2.042 x 2.042 m (3cc9d29) to 2.104 x 2.104 m (8891c46; 28 such rows of that sweep grew by 0.05-0.06 m).
LATERAL_BOUNDED_ROWS = {   # (cabinet scene, move): (x, y, sx, sy)
    (("bed", 50, 2.5, 1.2, -0.5), ("around", -90)): (2.510, -0.008, 2.212, 2.332),
    (("bed", 40, 2.5, 0.6, -0.2), ("turn", 96)): (2.514, 0.012, 2.344, 2.217),
    (("sofa", 45, 3.5, 1.2, -1.0), ("around", 90)): (3.500, -0.000, 2.051, 2.051),
}


def test_an_unseen_lateral_side_reaches_no_farther_than_the_prior_allows():   # R-PF3b (the review of 8891c46, finding 3)
    """A union whose observed depth is longer than S + EDGE_PAD has its long side along the depth axis, so an
    unseen lateral side reaches at most S - w_obs + EDGE_PAD past the union's last return there (S - w_obs is what
    the object can have left; L - w_obs only where the observed width exceeds S + EDGE_PAD, R-PF3c): the views'
    prior fill along that axis, and their own pads, stop there. Each pinned row covers (LATERAL_BOUNDED_ROWS)."""
    from test_detector_geometry import covers

    for (scene, move), pinned in LATERAL_BOUNDED_ROWS.items():
        solids, prior = _cabinet_scene(*scene)
        row, _ = _two_stop_row(solids, prior, _stops(solids[0], move))
        assert row == pytest.approx(pinned, abs=1.5e-3), (scene, move, row)
        assert covers(FP(*pinned[:2], pinned[2:], False), solids[0]), scene


# A prior larger than the object (cover125: the cover cases with the prior 1.25 x the true size), two stops: what stays
# against 3cc9d29 and 97da92a, a documented limit (R-PF3d), pinned with every row, the best row's miss now and on
# 3cc9d29 (m; 97da92a's is the same). MAX_SIZE_CLIP_ROWS: every such miss larger than on 3cc9d29, 39 in the 23
# two-stop cover125 sweeps, all of the 2 m square, whose prior is 2.5 x 2.5 m (square, so the views' reach counts,
# R-PF3c). Its filled rectangle's axis-aligned box is already about MAX_SIZE; R-PF3b's lateral rule grows an unseen
# side by up to 0.62 m, and geometry's box, clipped to MAX_SIZE about its new centre, loses the opposite side, which
# the union saw: up to 0.268 m short (0.102 m on 3cc9d29). No vocabulary kind is that large (the bed's L x L box at
# 45° is 2.97 m), and no vocabulary sweep shows it. R-PF3d's narrowing (an unseen side grows only as far as keeps the
# box within MAX_SIZE) gives 3cc9d29's rows here, but it makes 22 other cover125 misses larger than now (by 2-10.5
# mm, one new: the 2 m square (75°, 2.0 m, 30°) after a 20° turn, 0.0034 m), so it is not taken.
MAX_SIZE_CLIP_ROWS = {   # (cover case, move, faces): (every row in id order, the best row's miss, 3cc9d29's)
    ((2.0, 2.0, 0.8, 30, 1.0, 0), ("ahead", 0.5), "lean"): ([(2.868, 0.116, 3.000, 3.000)], 0.088, 0.082),
    ((2.0, 2.0, 0.8, 30, 1.0, 0), ("ahead", 0.5), "noise"): ([(2.862, 0.119, 3.000, 3.000)], 0.082, 0.077),
    ((2.0, 2.0, 0.8, 30, 1.0, 0), ("ahead", 0.5), "plain"): ([(2.865, 0.121, 3.000, 3.000)], 0.085, 0.081),
    ((2.0, 2.0, 0.8, 30, 1.0, 0), ("step", 0.15), "plain"): ([(2.865, 0.121, 3.000, 3.000)], 0.085, 0.083),
    ((2.0, 2.0, 0.8, 30, 1.0, 0), ("step", 0.3), "plain"): ([(2.865, 0.121, 3.000, 3.000)], 0.085, 0.082),
    ((2.0, 2.0, 0.8, 30, 1.0, 0), ("step", 0.5), "lean"): ([(2.868, 0.116, 3.000, 3.000)], 0.088, 0.082),
    ((2.0, 2.0, 0.8, 30, 1.0, 0), ("step", 0.5), "noise"): ([(2.862, 0.119, 3.000, 3.000)], 0.082, 0.077),
    ((2.0, 2.0, 0.8, 30, 1.0, 0), ("step", 0.5), "plain"): ([(2.865, 0.121, 3.000, 3.000)], 0.085, 0.081),
    ((2.0, 2.0, 0.8, 30, 1.0, 0), ("turn", 20), "plain"): ([(2.868, 0.116, 3.000, 3.000)], 0.088, 0.086),
    ((2.0, 2.0, 0.8, 30, 1.0, 30), ("around", 45), "lean"): ([(2.236, 1.113, 3.000, 3.000)], 0.09, 0.0181),
    ((2.0, 2.0, 0.8, 30, 1.0, 30), ("around", 45), "noise"): ([(2.231, 1.110, 3.000, 3.000)], 0.085, 0.0406),
    ((2.0, 2.0, 0.8, 30, 1.0, 30), ("around", 45), "plain"): ([(2.236, 1.111, 3.000, 3.000)], 0.09, 0.0396),
    ((2.0, 2.0, 0.8, 30, 2.0, 30), ("around", -45), "lean"): ([(3.114, 1.593, 3.000, 3.000)], 0.1019, 0.0),
    ((2.0, 2.0, 0.8, 30, 2.0, 30), ("around", -45), "noise"): ([(3.109, 1.592, 3.000, 3.000)], 0.0969, 0.0),
    ((2.0, 2.0, 0.8, 30, 2.0, 30), ("around", -45), "plain"): ([(3.112, 1.594, 3.000, 3.000)], 0.0999, 0.0),
    ((2.0, 2.0, 0.8, 30, 3.0, 30), ("around", -45), "lean"): ([(3.983, 2.088, 3.000, 3.000)], 0.1049, 0.0),
    ((2.0, 2.0, 0.8, 30, 3.0, 30), ("around", -45), "noise"): ([(3.978, 2.087, 3.000, 3.000)], 0.0999, 0.0),
    ((2.0, 2.0, 0.8, 30, 3.0, 30), ("around", -45), "plain"): ([(3.982, 2.088, 3.000, 3.000)], 0.1039, 0.0),
    ((2.0, 2.0, 0.8, 45, 1.0, 0), ("ahead", 0.5), "lean"): ([(3.048, 0.000, 3.000, 3.000)], 0.268, 0.1002),
    ((2.0, 2.0, 0.8, 45, 1.0, 0), ("ahead", 0.5), "plain"): ([(3.048, 0.000, 3.000, 3.000)], 0.268, 0.1022),
    ((2.0, 2.0, 0.8, 45, 1.0, 0), ("step", 0.15), "plain"): ([(3.048, 0.000, 3.000, 3.000)], 0.268, 0.0982),
    ((2.0, 2.0, 0.8, 45, 1.0, 0), ("step", 0.3), "plain"): ([(3.048, 0.000, 3.000, 3.000)], 0.268, 0.0992),
    ((2.0, 2.0, 0.8, 45, 1.0, 0), ("step", 0.5), "lean"): ([(3.048, 0.000, 3.000, 3.000)], 0.268, 0.1002),
    ((2.0, 2.0, 0.8, 45, 1.0, 0), ("step", 0.5), "plain"): ([(3.048, 0.000, 3.000, 3.000)], 0.268, 0.1022),
    ((2.0, 2.0, 0.8, 45, 2.0, 0), ("ahead", 0.5), "lean"): ([(3.878, 0.171, 3.000, 3.000)], 0.098, 0.0952),
    ((2.0, 2.0, 0.8, 45, 2.0, 0), ("step", 0.15), "plain"): ([(3.875, 0.173, 3.000, 3.000)], 0.095, 0.0912),
    ((2.0, 2.0, 0.8, 45, 2.0, 0), ("step", 0.5), "lean"): ([(3.878, 0.171, 3.000, 3.000)], 0.098, 0.0952),
    ((2.0, 2.0, 0.8, 45, 3.0, 0), ("ahead", 0.5), "noise"): ([(5.044, -0.001, 3.000, 3.000)], 0.264, 0.0992),
    ((2.0, 2.0, 0.8, 45, 3.0, 0), ("step", 0.5), "noise"): ([(5.044, -0.001, 3.000, 3.000)], 0.264, 0.0992),
    ((2.0, 2.0, 0.8, 45, 3.0, 0), ("turn", 20), "plain"): ([(4.904, -0.143, 3.000, 3.000)], 0.124, 0.115),
    ((2.0, 2.0, 0.8, 60, 1.0, 0), ("ahead", 0.5), "lean"): ([(2.868, -0.116, 3.000, 3.000)], 0.088, 0.082),
    ((2.0, 2.0, 0.8, 60, 1.0, 0), ("ahead", 0.5), "noise"): ([(2.862, -0.120, 3.000, 3.000)], 0.082, 0.079),
    ((2.0, 2.0, 0.8, 60, 1.0, 0), ("ahead", 0.5), "plain"): ([(2.865, -0.121, 3.000, 3.000)], 0.085, 0.081),
    ((2.0, 2.0, 0.8, 60, 1.0, 0), ("step", 0.15), "plain"): ([(2.865, -0.121, 3.000, 3.000)], 0.085, 0.083),
    ((2.0, 2.0, 0.8, 60, 1.0, 0), ("step", 0.3), "plain"): ([(2.865, -0.121, 3.000, 3.000)], 0.085, 0.082),
    ((2.0, 2.0, 0.8, 60, 1.0, 0), ("step", 0.5), "lean"): ([(2.868, -0.116, 3.000, 3.000)], 0.088, 0.082),
    ((2.0, 2.0, 0.8, 60, 1.0, 0), ("step", 0.5), "noise"): ([(2.862, -0.120, 3.000, 3.000)], 0.082, 0.079),
    ((2.0, 2.0, 0.8, 60, 1.0, 0), ("step", 0.5), "plain"): ([(2.865, -0.121, 3.000, 3.000)], 0.085, 0.081),
    ((2.0, 2.0, 0.8, 60, 1.0, 30), ("step", 0.15), "plain"): ([(2.671, 1.275, 3.000, 3.000)], 0.208, 0.058),
}
# PRIOR_FILL_GROWN_ROWS: the sides grown by more than 0.05 m over 3cc9d29 where the returns reach within 5 mm of the
# object's edge, 7 in the cover125 sweeps (the third value: the growth, m). The rule judges them unseen (grazed beyond
# 60°) and fills them to the prior's other side, S - w_obs + EDGE_PAD past the union's last return, which here is
# 1.25 x the object's: up to 0.25 S past its edge. Each row covers.
PRIOR_FILL_GROWN_ROWS = {   # (cover case, move, faces): (every row in id order, the best row's miss, the growth)
    ((0.9, 0.45, 1.2, 90, 1.0, 0), ("around", -45), "lean"): ([(1.562, 0.000, 0.662, 1.000)], 0.0, 0.118),
    ((1.2, 0.4, 0.45, 75, 1.0, 30), ("around", -90), "lean"): ([(1.685, 0.770, 0.916, 1.410)], 0.0, 0.104),
    ((1.2, 0.4, 0.45, 90, 1.0, 30), ("ahead", 0.5), "lean"): ([(1.606, 0.736, 0.600, 1.299)], 0.0, 0.101),
    ((1.2, 0.4, 0.45, 90, 1.0, 30), ("ahead", 0.5), "noise"): ([(1.598, 0.735, 0.603, 1.301)], 0.0, 0.099),
    ((1.2, 0.4, 0.45, 90, 1.0, 30), ("ahead", 0.5), "plain"): ([(1.606, 0.735, 0.600, 1.297)], 0.0, 0.104),
    ((1.8, 0.9, 0.85, 45, 2.0, 0), ("step", 0.15), "plain"): ([(3.473, 0.080, 2.528, 2.528)], 0.0, 0.123),
    ((1.8, 0.9, 0.85, 75, 1.0, 30), ("around", -90), "lean"): ([(2.082, 0.948, 1.675, 2.151)], 0.0, 0.228),
}


def test_a_prior_larger_than_the_object_keeps_its_pinned_rows():   # R-PF3d (cover125, a documented limit)
    """The prior 1.25 x the true size, two stops: MAX_SIZE_CLIP_ROWS (each row clipped to MAX_SIZE, each miss larger
    than 3cc9d29's) and PRIOR_FILL_GROWN_ROWS (each covers), every row and the best row's miss pinned."""
    from detector import geometry as g
    from test_detector_geometry import _cover_solid

    assert (len(MAX_SIZE_CLIP_ROWS), len(PRIOR_FILL_GROWN_ROWS)) == (39, 7)
    for table in (MAX_SIZE_CLIP_ROWS, PRIOR_FILL_GROWN_ROWS):
        for (case, move, faces), (pinned, miss, base) in table.items():
            s = _cover_solid(*case)
            rows, _ = _two_stop_rows([s], (1.25 * case[0], 1.25 * case[1]), _stops(s, move),
                                     0.5 if faces == "lean" else 0.0, 0.002 if faces == "noise" else 0.0)
            assert _rows_match(rows, pinned), (case, move, faces, rows)
            best = min(_outside(FP(*p[:2], p[2:], False), s) for p in pinned)
            assert best == pytest.approx(miss, abs=5e-4), (case, move, faces, best)
            if table is MAX_SIZE_CLIP_ROWS:
                assert miss > base and all(max(p[2:]) == g.MAX_SIZE for p in pinned), (case, move, faces)


def test_frames_beyond_6_m_do_not_count_toward_n():
    run = Run()
    for _ in range(4):
        run.frame(obs(x=6.3))                            # a sighting (within 8 m), never in view (beyond 6 m)
    assert run.ids() == []
    assert run.stats()["tracks_tentative"] == 1
    run = Run()
    for _ in range(4):
        run.frame(obs(x=2.0, cam=(-4.5, 0.0)))           # the camera 6.5 m away
    assert run.ids() == []
    run = Run()
    run.frame(obs(x=5.8))
    run.frame(obs(x=5.8))
    assert run.ids() == ["chair_1"]


def test_hits_in_frames_the_core_calls_out_of_view_do_not_count():
    run = Run()
    for _ in range(4):
        run.frame(obs(), in_view=never)                  # e.g. occluded by LiDAR returns
    assert run.ids() == []


def test_a_tentative_track_older_than_30_s_is_dropped_without_an_id():
    run = Run()
    run.frame(obs())
    run.frame(in_view=never, dt=29.0)
    run.frame(obs(), dt=0.5)                             # 29.5 s after the first hit: still in time
    assert run.ids() == ["chair_1"]

    run = Run()
    run.frame(obs())
    run.frame(in_view=never, dt=30.5)
    assert run.stats()["tracks_tentative"] == 0          # dropped
    run.frame(obs())                                     # starts a new tentative track
    assert run.ids() == []
    assert run.stats()["tracks_tentative"] == 1
    assert run.stats()["ids_issued"] == {}


def test_without_a_supported_hit_three_of_three_at_0_5():
    run = Run()
    run.frame(obs(conf=0.6, supported=False))
    run.frame(obs(conf=0.6, supported=False))
    assert run.ids() == []
    run.frame(obs(conf=0.6, supported=False))
    assert run.ids() == ["chair_1"]
    assert run.rows()[0].measured is False

    run = Run()                                          # one hit below 0.5
    for conf in (0.6, 0.45, 0.6):
        run.frame(obs(conf=conf, supported=False))
    assert run.ids() == []

    run = Run()                                          # a miss in the window
    run.frame(obs(supported=False))
    run.frame()
    run.frame(obs(supported=False))
    run.frame(obs(supported=False))
    assert run.ids() == []
    run.frame(obs(supported=False))
    assert run.ids() == ["chair_1"]


def test_a_supported_hit_lets_two_hits_confirm():
    run = Run()
    run.frame(obs(conf=0.4, supported=False))
    run.frame(obs(conf=0.4, supported=True))
    assert run.ids() == ["chair_1"]
    assert run.rows()[0].measured is True


# --- association -----------------------------------------------------------------------------------------------

def test_same_label_within_the_gate_is_one_track():
    run = Run()
    run.frame(obs(x=2.0))
    run.frame(obs(x=2.3))
    assert run.ids() == ["chair_1"]
    assert run.stats()["tracks_tentative"] == 0


def test_two_boxes_one_metre_apart_and_counters_per_label():
    run = Run()
    confirmed(run, obs("box", y=0.0), obs("box", y=1.0), obs("chair", y=-2.0))
    assert run.ids() == ["box_1", "box_2", "chair_1"]
    assert run.stats()["ids_issued"] == {"box": 2, "chair": 1}
    assert [r.y for r in run.rows()] == pytest.approx([0.0, 1.0, -2.0], abs=0.06)


def test_a_fallback_observation_widens_the_gate():
    run = Run()
    confirmed(run, obs("chair", x=2.0))
    run.frame(obs("chair", x=2.0 + 0.85, supported=False))   # beyond 0.5 + 0.5 r (0.66), within + 0.5
    assert run.stats()["tracks_tentative"] == 0
    run.frame(obs("chair", x=2.0 + 0.85, supported=True))    # a supported one there is another object
    assert run.stats()["tracks_tentative"] == 1


def test_a_supported_hit_moves_a_fallback_track():
    run = Run()
    for _ in range(3):
        run.frame(obs("chair", x=2.0, supported=False))
    assert run.ids() == ["chair_1"]
    run.frame(obs("chair", x=2.85, supported=True))          # the fallback track's gate is widened too
    assert run.stats()["tracks_tentative"] == 0
    (row,) = run.rows()
    assert (row.id, row.measured) == ("chair_1", True)
    assert row.x == pytest.approx(2.85, abs=0.06)


def test_a_fallback_observation_does_not_move_a_supported_track():
    run = Run()
    confirmed(run, obs("chair", x=2.0))
    run.frame(obs("chair", x=2.6, supported=False))
    (row,) = run.rows()
    assert row.x == pytest.approx(2.0, abs=0.06) and row.visible


# --- ids -------------------------------------------------------------------------------------------------------

def test_ids_are_never_reused_and_stop_at_99():
    run = Run()
    confirmed(run, obs("box"))
    assert run.ids() == ["box_1"]
    run.retire("box_1")
    assert run.ids() == []
    confirmed(run, obs("box"))
    assert run.ids() == ["box_2"]                        # box_1 is gone for good
    for n in range(3, MAX_ID + 1):
        run.retire(f"box_{n - 1}")
        confirmed(run, obs("box"))
        assert run.ids() == [f"box_{n}"]
    run.retire("box_99")
    confirmed(run, obs("box"))
    run.frame(obs("box"))
    assert run.ids() == []                               # no box_100
    assert run.stats()["notes"] == ["box: ids 1-99 used, new box tracks ignored"]
    confirmed(run, obs("box"))
    assert run.stats()["notes"] == ["box: ids 1-99 used, new box tracks ignored"]   # one note (C6)
    assert run.stats()["ids_issued"] == {"box": 99}
    assert run.stats()["ids_retired"] == 99
    confirmed(run, obs("chair", y=2.0))                  # other labels go on
    assert run.ids() == ["chair_1"]


def test_rows_are_in_id_key_order():
    run = Run()
    boxes = [obs("box", x=2.0, y=-4.5 + k) for k in range(10)]
    confirmed(run, obs("chair", x=1.0, y=0.0), *boxes)
    assert run.ids() == [f"box_{n}" for n in range(1, 11)] + ["chair_1"]


# --- votes, merges and relabels --------------------------------------------------------------------------------

def test_a_bench_vote_on_a_confirmed_box_makes_no_new_track():
    run = Run()
    confirmed(run, obs("box", 0.6))
    for _ in range(2):
        run.frame(obs("bench", 0.5, x=2.05, size=(0.5, 0.5)))
    assert run.ids() == ["box_1"]
    assert run.stats()["tracks_tentative"] == 0
    assert run.rows()[0].visible


def test_overlapping_confirmed_tracks_keep_the_older_id():
    # Two fallback-only tracks (their gates let fallback views move them) confirm apart, then bench_1 walks
    # onto box_1: the merge keeps the older id and retires the younger one.
    run = Run()
    for _ in range(3):
        run.frame(obs("box", 0.6, x=2.0, supported=False))
    for _ in range(3):
        run.frame(obs("box", 0.6, x=2.0, supported=False), obs("bench", 0.6, x=3.2, size=(0.5, 0.5),
                                                               supported=False))
    assert run.ids() == ["bench_1", "box_1"]
    run.frame(obs("box", 0.6, x=2.0, supported=False), obs("bench", 0.6, x=2.6, size=(0.5, 0.5), supported=False))
    assert run.ids() == ["bench_1", "box_1"]             # no overlap yet
    run.frame(obs("box", 0.6, x=2.0, supported=False), obs("bench", 0.6, x=2.05, size=(0.5, 0.5),
                                                           supported=False))
    assert run.ids() == ["box_1"]
    assert run.stats()["ids_issued"] == {"bench": 1, "box": 1}
    assert run.stats()["ids_retired"] == 1               # bench_1 merged into box_1


def test_a_duplicate_of_a_confirmed_track_gets_no_id():
    # box and bench in one image on the same place: the box track takes the box view, so the bench view starts a
    # tentative track; it would merge into box_1 at once, so it joins box_1 at confirmation and burns no id.
    run = Run()
    confirmed(run, obs("box", 0.6))
    both = (obs("box", 0.6), obs("bench", 0.6, x=2.05, size=(0.5, 0.5)))
    confirmed(run, *both)
    assert run.ids() == ["box_1"]
    assert run.stats()["ids_issued"] == {"box": 1}
    assert run.stats()["ids_retired"] == 0
    assert run.stats()["notes"] == []


def test_two_new_overlapping_tracks_confirm_as_one_id():
    run = Run()
    confirmed(run, obs("lamp", 0.6, size=(0.4, 0.4)), obs("desk", 0.64, size=(0.4, 0.4)))
    assert run.ids() == ["lamp_1"]                       # the first one keeps its id; the other gets none
    assert run.stats()["ids_issued"] == {"lamp": 1}
    assert run.stats()["ids_retired"] == 0


DUPLICATES = {
    # spec §11.7: a lamp on its stand also called desk (same points), a sofa and a part of it, a box also bench.
    "lamp_desk": ([obs("lamp", 0.6, size=(0.4, 0.4)), obs("desk", 0.64, size=(0.4, 0.4))], "lamp_1",
                  obs("desk", 0.6, x=1.0, y=-3.0), "desk_1"),
    "sofa_part": ([obs("sofa", x=3.0, y=0.0, size=(1.8, 0.9)), obs("sofa", x=3.6, y=0.2, size=(0.5, 0.5))],
                  "sofa_1", obs("sofa", x=1.0, y=-3.0), "sofa_2"),
    "box_bench": ([obs("box", 0.6), obs("bench", 0.6, x=2.05, size=(0.5, 0.5))], "box_1",
                  obs("bench", 0.6, x=1.0, y=-3.0), "bench_1"),
}


@pytest.mark.parametrize("case", sorted(DUPLICATES))
def test_a_duplicate_in_every_frame_uses_no_ids(case):
    # Review fix round 1: 200 still frames 3 s apart (10 minutes at the idle period) of an object and its
    # duplicate view issue one id; the duplicate's label keeps all its ids for real objects (D2, C6).
    pair, kept, real, real_id = DUPLICATES[case]
    run = Run()
    for _ in range(200):
        run.frame(*pair, dt=3.0)
    assert run.ids() == [kept]
    label = kept.rsplit("_", 1)[0]
    assert run.stats()["ids_issued"] == {label: 1}
    assert run.stats()["ids_retired"] == 0
    assert run.stats()["notes"] == []
    for _ in range(2):
        run.frame(*pair, real, dt=3.0)                   # a real object of the duplicate's label elsewhere
    assert real_id in run.ids()
    assert run.stats()["ids_retired"] == 0


def test_a_relabel_onto_an_overlapping_track_joins_it():
    # A chair under a table, then called table: the relabelled track would merge into table_1 at once, so it joins
    # table_1 (chair_1 ends) instead of taking table_2.
    run = Run()
    table = obs("table", x=2.0, y=0.0, size=(1.2, 0.75))
    chair = obs("chair", x=2.0, y=0.24, size=(0.45, 0.45))
    confirmed(run, table, chair)
    assert run.ids() == ["chair_1", "table_1"]
    as_table = obs("table", 0.6, x=2.0, y=0.24, size=(0.45, 0.45))
    for _ in range(4):
        run.frame(table, as_table)                       # table_1 takes the table view; the other votes on chair_1
    assert run.ids() == ["table_1"]
    assert run.stats()["ids_issued"] == {"chair": 1, "table": 1}
    assert run.stats()["ids_retired"] == 1               # chair_1
    for _ in range(10):
        run.frame(table, as_table)
    assert run.ids() == ["table_1"]
    assert run.stats()["ids_issued"] == {"chair": 1, "table": 1}
    assert run.stats()["ids_retired"] == 1
    assert run.rows()[0].size == pytest.approx((1.2, 0.75), abs=0.06)


def test_a_chair_under_a_table_stays_two_objects():
    # spec §11.7: a chair pushed under a table, IoU about 0.17, stays two objects although
    # the chair's centre lies inside the table's footprint (different labels merge by IoU only).
    run = Run()
    table = obs("table", x=2.0, y=0.0, size=(1.2, 0.75))
    chair = obs("chair", x=2.0, y=0.375 - 0.135, size=(0.45, 0.45))
    confirmed(run, table, chair)
    run.frame(table, chair)
    assert run.ids() == ["chair_1", "table_1"]
    assert run.stats()["ids_retired"] == 0


def test_same_label_tracks_merge_when_a_centre_is_inside():
    # A whole sofa and a part of it in one image (NMS at IoU 0.7 keeps both boxes): IoU 0.15, the part's
    # centre inside the sofa's footprint. Both are ready in the same frame; the first one takes the id and the
    # part joins it without one.
    run = Run()
    whole = obs("sofa", x=3.0, y=0.0, size=(1.8, 0.9))
    part = obs("sofa", x=3.6, y=0.2, size=(0.5, 0.5))
    confirmed(run, whole, part)
    assert run.ids() == ["sofa_1"]
    assert run.stats()["ids_issued"] == {"sofa": 1}
    assert run.stats()["ids_retired"] == 0
    assert run.rows()[0].size == pytest.approx((1.8, 0.9), abs=0.06)


def test_different_labels_with_a_centre_inside_stay_two():
    run = Run()
    confirmed(run, obs("sofa", x=3.0, y=0.0, size=(1.8, 0.9)), obs("box", x=3.6, y=0.2, size=(0.5, 0.5)))
    assert run.ids() == ["box_1", "sofa_1"]


def test_relabel_at_twice_the_sum_over_five_hits_retires_the_old_id():
    run = Run()
    confirmed(run, obs("box", 0.4))
    run.frame(obs("bench", 0.6, x=2.0, size=(0.5, 0.5)))
    run.frame(obs("bench", 0.6, x=2.0, size=(0.5, 0.5)))
    assert run.ids() == ["box_1"]                        # bench 1.2 < 2 x box 0.8
    run.frame(obs("bench", 0.6, x=2.0, size=(0.5, 0.5)))
    assert run.ids() == ["bench_1"]                      # bench 1.8 >= 1.6, one bench hit >= 0.35
    assert run.stats()["ids_retired"] == 1
    assert run.rows()[0].x == pytest.approx(2.0, abs=0.06)


def test_relabel_needs_one_hit_at_the_new_labels_confirm():
    run = Run()
    confirmed(run, obs("box", 0.26))
    run.frame(obs("box", 0.35))
    assert run.ids() == ["box_1"]
    for _ in range(4):
        run.frame(obs("bench", 0.34, x=2.0, size=(0.5, 0.5)))   # 4 x 0.34 >= 2 x 0.35, but none >= 0.35
    assert run.ids() == ["box_1"]


def test_a_tentative_track_takes_the_label_of_its_votes():
    run = Run()
    run.frame(obs("box", 0.3, size=(0.5, 0.5)))
    run.frame(obs("bench", 0.6, size=(0.5, 0.5)))
    assert run.ids() == ["bench_1"]


# --- forgetting ------------------------------------------------------------------------------------------------

def test_vacancy_targets_list_supported_confirmed_tracks():
    run = Run()
    run.frame(obs("box", x=2.0, y=2.0))                  # tentative: not a target
    confirmed(run, obs("chair"), obs("lamp", y=-2.0, supported=False), obs("box", x=2.0, y=2.0))
    run.frame(obs("lamp", y=-2.0, supported=False))
    assert set(run.ids()) == {"box_1", "chair_1", "lamp_1"}
    targets = run.tracker.vacancy_targets()
    assert sorted(label for *_rest, label in targets) == ["box", "chair"]
    for key, x, y, size, label in targets:
        assert isinstance(key, str) and len(size) == 2


def test_vacant_twice_half_a_second_apart_retires():
    run = Run()
    confirmed(run, obs("box"))
    key = run.key("box_1")
    run.tracker.mark_vacant(key, True, 200.0)
    run.tracker.mark_vacant(key, True, 200.3)            # within 0.5 s: counts once
    assert run.ids() == ["box_1"]
    run.tracker.mark_vacant(key, True, 200.6)
    assert run.ids() == []
    assert run.stats()["ids_retired"] == 1
    run.tracker.mark_vacant(key, True, 201.2)            # an unknown key is ignored


def test_a_not_vacant_scan_resets_the_count():
    run = Run()
    confirmed(run, obs("box"))
    key = run.key("box_1")
    run.tracker.mark_vacant(key, True, 300.0)
    run.tracker.mark_vacant(key, False, 300.5)
    run.tracker.mark_vacant(key, True, 301.0)
    assert run.ids() == ["box_1"]
    run.tracker.mark_vacant(key, True, 301.5)
    assert run.ids() == []


def test_a_hit_resets_the_vacancy_count():
    run = Run()
    confirmed(run, obs("box"))
    key = run.key("box_1")
    run.tracker.mark_vacant(key, True, run.t)
    run.frame(obs("box"))
    run.tracker.mark_vacant(key, True, run.t)
    assert run.ids() == ["box_1"]


def test_misses_with_lidar_still_occupied_keep_the_track():
    run = Run()
    confirmed(run, obs("box"))
    last_hit = run.wall
    key = run.key("box_1")
    for _ in range(10):
        run.frame()                                      # in view, no hit
        run.tracker.mark_vacant(key, False, run.t)
    assert [(r.id, r.visible) for r in run.rows(at=last_hit + 4.9)] == [("box_1", True)]
    assert [(r.id, r.visible) for r in run.rows(at=last_hit + 6.1)] == [("box_1", False)]
    assert run.ids() == ["box_1"]


def test_out_of_view_tracks_are_kept():
    run = Run()
    confirmed(run, obs("box"))
    for _ in range(3):
        run.frame(obs("chair", y=2.0, supported=False))
    assert run.ids() == ["box_1", "chair_1"]
    for _ in range(20):
        run.frame(in_view=never)
    assert run.ids() == ["box_1", "chair_1"]


def test_an_unsupported_track_retires_after_three_in_view_misses_not_when_occluded():
    run = Run()
    for _ in range(3):
        run.frame(obs("chair", supported=False))
    assert run.ids() == ["chair_1"]
    run.frame()
    run.frame()
    run.frame(obs("chair", supported=False))             # a hit resets the count
    run.frame()
    run.frame()
    for _ in range(5):
        run.frame(in_view=never)                         # occluded: does not count, does not reset
    assert run.ids() == ["chair_1"]
    run.frame()
    assert run.ids() == []
    assert run.stats()["ids_retired"] == 1


def test_a_supported_track_is_never_retired_by_misses():
    run = Run()
    confirmed(run, obs("box"))
    for _ in range(MISS_UNSUPPORTED * 5):
        run.frame()
    assert run.ids() == ["box_1"]


# --- geometry, visibility, stats -------------------------------------------------------------------------------

def test_fused_size_recovers_after_one_bad_frame():
    run = Run()
    confirmed(run, obs("box"))
    assert run.rows()[0].size == pytest.approx((0.5, 0.5), abs=0.06)
    bad = np.vstack([square_points(2.0, 0.0, (0.5, 0.5)), square_points(2.0, 0.9, (0.3, 0.3))])
    run.frame(obs("box", points=bad))
    assert run.rows()[0].size[1] > 1.0                   # the bad frame's points are in the fused set
    for _ in range(TRACK_POINT_FRAMES - 1):
        run.frame(obs("box"))
        assert run.rows()[0].size[1] > 1.0
    run.frame(obs("box"))                                # the bad frame leaves the window after 6 hits
    assert run.rows()[0].size == pytest.approx((0.5, 0.5), abs=0.06)
    assert run.rows()[0].y == pytest.approx(0.0, abs=0.06)


def test_fuse_gets_the_newest_view_and_the_prior():
    calls = []

    def recording_fuse(points, cam_xy, prior, views=None):
        calls.append((len(points), cam_xy, prior, views))
        return aabb_fuse(points, cam_xy, prior)

    run = Run(fuse=recording_fuse)
    run.frame(obs("sofa", x=3.0, size=(1.8, 0.9), cam=(0.0, 0.0)))
    run.frame(obs("sofa", x=3.0, size=(1.8, 0.9), cam=(0.5, 0.2)))
    assert calls[-1][1] == (0.5, 0.2)
    assert calls[-1][2] == PRIORS["sofa"]
    # the single views, oldest first, each with its own camera (pre-final review F3)
    views = calls[-1][3]
    assert [cam for _, cam in views] == [(0.0, 0.0), (0.5, 0.2)]
    assert sum(len(p) for p, _ in views) >= calls[-1][0]


def test_fused_points_are_thinned_and_capped():
    seen = []

    def counting_fuse(points, cam_xy, prior, views=None):
        seen.append(len(points))
        return aabb_fuse(points, cam_xy, prior)

    run = Run(fuse=counting_fuse)
    dense = np.vstack([square_points(3.0, 0.0, (1.8, 0.9), step=0.01)] * 2)   # 32 942 points, 703 cells of 5 cm
    for _ in range(TRACK_POINT_FRAMES + 2):
        run.frame(obs("sofa", x=3.0, size=(1.8, 0.9), points=dense))
    assert 0 < max(seen) <= 37 * 19
    seen.clear()
    wide = square_points(3.0, 0.0, (2.9, 2.9), step=0.01)                     # 3 481 cells
    run.frame(obs("sofa", x=3.0, size=(1.8, 0.9), points=wide))
    assert seen == [tracker_module.MAX_TRACK_POINTS] == [2000]
    assert tracker_module.THIN_CELL == 0.05


def test_visible_hold_follows_the_idle_period():
    run = Run()
    confirmed(run, obs("box"))
    hit = run.wall
    assert run.tracker.rows(hit + 5.0, 1.0)[0].visible               # max(5, 2) = 5
    assert not run.tracker.rows(hit + 5.1, 1.0)[0].visible
    assert run.tracker.rows(hit + 6.0, 3.0)[0].visible               # max(5, 6) = 6
    assert not run.tracker.rows(hit + 6.1, 3.0)[0].visible
    assert run.tracker.rows(hit + 12.0, 6.0)[0].visible              # slow: the idle period doubled
    assert not run.tracker.rows(hit + 12.1, 6.0)[0].visible


def test_seen_is_the_wall_time_of_the_last_hit():
    run = Run()
    confirmed(run, obs("box"))
    hit = run.wall
    run.frame()
    assert run.rows()[0].seen == pytest.approx(hit)


def test_confidence_is_the_median_of_the_last_10_hits_of_the_label():
    run = Run()
    for conf in (0.9, 0.9, 0.9, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.5):
        run.frame(obs("box", conf))
    assert run.rows()[0].confidence == pytest.approx(0.4)
    run.frame(obs("box", 0.61237))
    run.frame(obs("box", 0.61237))
    run.frame(obs("box", 0.61237))
    run.frame(obs("box", 0.61237))
    run.frame(obs("box", 0.61237))
    assert run.rows()[0].confidence == pytest.approx(0.556, abs=1e-9)   # median(0.4 x 4, 0.5, 0.61237 x 5)


def test_filled_ids_are_reported():
    def filling_fuse(points, cam_xy, prior, views=None):
        fp = aabb_fuse(points, cam_xy, prior)
        return fp._replace(filled=fp.size[0] < prior[0] - 0.2)   # the sofa is seen face-on: depth filled

    run = Run(fuse=filling_fuse)
    confirmed(run, obs("box"), obs("sofa", x=3.0, y=2.0, size=(0.3, 1.8)))
    assert run.ids() == ["box_1", "sofa_1"]
    assert run.stats()["filled"] == ["sofa_1"]


def test_stats_keys():
    run = Run()
    run.frame(obs("box", y=2.0))
    confirmed(run, obs("chair"), obs("box", y=2.0))
    stats = run.stats()
    assert set(stats) == {"tracks_tentative", "tracks_confirmed", "ids_issued", "ids_retired", "filled", "notes"}
    assert (stats["tracks_tentative"], stats["tracks_confirmed"]) == (0, 2)
    json.dumps(stats)                                    # it goes into /go2/detector_status


def test_reset_restarts_numbering():
    run = Run()
    confirmed(run, obs("box", y=0.0), obs("box", y=1.0))
    run.frame(obs("chair", y=-2.0))
    run.retire("box_1")
    run.tracker.reset()
    assert run.ids() == []
    assert run.stats() == {"tracks_tentative": 0, "tracks_confirmed": 0, "ids_issued": {}, "ids_retired": 0,
                           "filled": [], "notes": []}
    confirmed(run, obs("box"))
    assert run.ids() == ["box_1"]


def test_reset_clears_the_notes():
    run = Run()
    for n in range(1, MAX_ID + 1):
        confirmed(run, obs("box"))
        run.retire(f"box_{n}")
    confirmed(run, obs("box"))
    assert run.stats()["notes"]
    run.tracker.reset()
    assert run.stats()["notes"] == []


def test_rows_round_trip_through_objects_parse():
    run = Run()
    confirmed(run, obs("sofa", 0.71, x=-4.4, y=0.0, size=(1.09, 1.85), cam=(0.0, 0.0)), obs("box", 0.5, y=1.5))
    for _ in range(3):
        run.frame(obs("lamp", 0.8, y=-1.5, supported=False))
    rows = run.rows()
    msg = objects.format_message(rows, run.wall, source="detector")
    payload = json.loads(json.dumps(msg))
    assert objects.parse_source(payload) == "detector"
    back = objects.parse(payload)
    assert [o.id for o in back] == ["box_1", "lamp_1", "sofa_1"]
    assert all(0.0 <= o.confidence <= 1.0 for o in back)
    assert [o.measured for o in back] == [True, False, True]
    assert back == tuple(rows)
    for o in back:
        assert all(0 < v <= objects.MAX_SIZE for v in o.size)


# --- what the tracker never does (spec §11.7) ------------------------------------------------------------------

def test_the_tracker_reads_no_names_from_gazebo():
    """No topic name and no ROS, Gazebo or layout import: the names come from the detections only."""
    tree = ast.parse((ROOT / "detector" / "tracker.py").read_text(encoding="utf-8"))
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(a.name.split(".")[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom):
            assert node.level == 0
            imported.add(node.module)
        elif isinstance(node, ast.Constant) and isinstance(node.value, str):
            assert not node.value.startswith("/"), node.value
    assert imported <= {"__future__", "collections", "collections.abc", "dataclasses", "logging", "math",
                        "statistics", "typing", "numpy", "voice_ltl", "detector.protocol"}


def test_a_tentative_track_is_never_published():
    run = Run()
    run.frame(obs("box"), obs("chair", y=2.0))
    assert run.rows() == []
    assert run.stats()["tracks_tentative"] == 2
