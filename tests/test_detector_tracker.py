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


def aabb_fuse(points, cam_xy, prior):
    """The stub fuse: the axis-aligned box of the points (at least 0.1 m a side)."""
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
    run = Run()
    confirmed(run, obs("box", 0.6))
    both = (obs("box", 0.6), obs("bench", 0.6, x=2.05, size=(0.5, 0.5)))
    confirmed(run, *both)                                # the box track is taken: bench is a new track
    assert run.ids() == ["box_1"]
    assert run.stats()["ids_issued"] == {"bench": 1, "box": 1}
    assert run.stats()["ids_retired"] == 1               # bench_1 merged into box_1


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
    # centre inside the sofa's footprint. Both confirm in the same frame; the first one keeps its id.
    run = Run()
    whole = obs("sofa", x=3.0, y=0.0, size=(1.8, 0.9))
    part = obs("sofa", x=3.6, y=0.2, size=(0.5, 0.5))
    confirmed(run, whole, part)
    assert run.ids() == ["sofa_1"]
    assert run.stats()["ids_issued"] == {"sofa": 2}
    assert run.stats()["ids_retired"] == 1
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

    def recording_fuse(points, cam_xy, prior):
        calls.append((len(points), cam_xy, prior))
        return aabb_fuse(points, cam_xy, prior)

    run = Run(fuse=recording_fuse)
    run.frame(obs("sofa", x=3.0, size=(1.8, 0.9), cam=(0.0, 0.0)))
    run.frame(obs("sofa", x=3.0, size=(1.8, 0.9), cam=(0.5, 0.2)))
    assert calls[-1][1] == (0.5, 0.2)
    assert calls[-1][2] == PRIORS["sofa"]


def test_fused_points_are_thinned_and_capped():
    seen = []

    def counting_fuse(points, cam_xy, prior):
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
    def filling_fuse(points, cam_xy, prior):
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
