"""detector/oracle.py: the truth projected into the image (Task 2.7; spec §11.10, §11.11; design-detector §11.1).

The oracle box is the truth footprint x the vocabulary height (or the given heights) projected with the camera
model at the image-stamp pose. Through DetectorCore it isolates the geometry and tracking error: on a synthetic
still scene (the ray caster of tests/test_detector_geometry.py) the published rows sit within the target line
(0.15 m) and cover the truth on the observed extent (C2: gazebo/tools/objects_check.py covers()).
"""

import math

import numpy as np
import pytest

from detector.camera import CAMERA_H, CAMERA_W, CameraModel, world_to_camera
from detector.core import View
from detector.oracle import OracleDetector
from detector.protocol import YOLOE_THRESHOLDS, Detector
from gazebo.tools import objects_check
from test_detector_core import IMAGE, Rig
from test_detector_geometry import BASE, Q0, solid, yaw_q
from voice_ltl import objects, vocabulary
from voice_ltl.objects import Obj

CAM = CameraModel.from_hfov()


def expected_box(obj, height, base=BASE, q=Q0, cam=CAM):
    """The camera model's box of a footprint x height (all corners ahead of the camera), clipped to the image."""
    sx, sy = obj.size
    corners = np.array([[obj.x + i * sx / 2, obj.y + j * sy / 2, z] for i in (-1, 1) for j in (-1, 1)
                        for z in (0.0, height)])
    uv, _inside = cam.project(world_to_camera(corners, base, q))
    assert np.isfinite(uv).all()
    lo = np.clip(uv.min(axis=0), 0, [cam.width, cam.height])
    hi = np.clip(uv.max(axis=0), 0, [cam.width, cam.height])
    return (*lo, *hi)


def oracle(rows, base=BASE, q=Q0, heights=None):
    det = OracleDetector()
    det.set_vocabulary(*_prompts())
    det.set_truth(rows, heights)
    det.prepare(View(base, q, CAM, 12.0))
    return det


def _prompts():
    return {e.label: tuple(e.detect) for e in vocabulary.VOCABULARY.entries}, tuple(vocabulary.VOCABULARY.detect_ignore)


def test_name_thresholds_and_protocol():
    det = OracleDetector()
    assert isinstance(det, Detector) and det.name == "oracle" and det.thresholds == YOLOE_THRESHOLDS
    assert det.detect(IMAGE) == []                         # no view prepared yet: nothing to project
    det.close()


@pytest.mark.parametrize("base, q", [pytest.param(BASE, Q0, id="ahead"),
                                     pytest.param((0.4, -0.3, 0.35), yaw_q(20.0), id="turned")])
def test_the_truth_box_is_the_camera_models_box(base, q):
    sofa = Obj("sofa_1", "sofa", 3.5, 0.8, (0.9, 1.8))
    chair = Obj("chair_1", "chair", 2.4, -0.9, (0.45, 0.45))
    det = oracle([sofa, chair], base, q)
    image = IMAGE.copy()
    dets = det.detect(image)
    assert [d.label for d in dets] == ["chair", "sofa"]            # nearer first
    for d, obj in zip(dets, (chair, sofa)):
        assert d.confidence == 1.0 and d.mask is None
        assert d.box == pytest.approx(expected_box(obj, vocabulary.height_of(obj.label), base, q), abs=1.0)
    assert np.array_equal(image, IMAGE)


def test_heights_by_id_replace_the_vocabulary_height():
    lamp = Obj("lamp_1", "lamp", 2.0, 0.3, (0.4, 0.4))
    plain = oracle([lamp]).detect(IMAGE)[0].box
    tall = oracle([lamp], heights={"lamp_1": 2.2}).detect(IMAGE)[0].box
    assert plain == pytest.approx(expected_box(lamp, vocabulary.height_of("lamp")), abs=1.0)
    assert tall == pytest.approx(expected_box(lamp, 2.2), abs=1.0)
    assert tall[1] < plain[1] and tall[3] == pytest.approx(plain[3])   # only the top moves
    other = oracle([lamp], heights={"sofa_1": 2.2}).detect(IMAGE)[0].box   # no height for lamp_1: the vocabulary's
    assert other == pytest.approx(plain)


def test_out_of_view_truth_gives_no_box_and_partly_seen_truth_is_clipped():
    behind = Obj("box_1", "box", -2.0, 0.0, (0.5, 0.5))
    aside = Obj("chair_1", "chair", 0.0, 3.0, (0.45, 0.45))       # 90° to the left: outside the 120° view
    edge = Obj("table_1", "table", 1.0, 1.4, (1.2, 0.75))         # its left part outside the image
    across = Obj("bench_1", "bench", 0.9, 0.0, (1.0, 1.0))        # the near plane cuts it
    dets = oracle([behind, aside, edge, across]).detect(IMAGE)
    assert sorted(d.label for d in dets) == ["bench", "table"]
    for d in dets:
        x0, y0, x1, y1 = d.box
        assert 0 <= x0 < x1 <= CAMERA_W and 0 <= y0 < y1 <= CAMERA_H
    table = next(d for d in dets if d.label == "table")
    assert table.box[0] == 0.0


def test_only_vocabulary_labels_given_to_set_vocabulary():
    rows = [Obj("sim_box_1", "box", 2.0, 0.0, (0.3, 0.3)), Obj("chair_1", "chair", 2.4, -0.9, (0.45, 0.45))]
    det = OracleDetector()
    det.set_vocabulary({"box": ("box",)}, ("person",))
    det.set_truth(rows)
    det.prepare(View(BASE, Q0, CAM, 1.0))
    assert [d.label for d in det.detect(IMAGE)] == ["box"]
    det.close()
    assert det.detect(IMAGE) == []


def test_overlapping_truth_boxes_keep_the_nearer():
    near = Obj("box_1", "box", 2.0, 0.0, (0.5, 0.5))
    hidden = Obj("box_2", "box", 2.1, 0.0, (0.5, 0.5))           # right behind it: the same image box nearly
    dets = oracle([hidden, near]).detect(IMAGE)
    assert len(dets) == 1 and dets[0].box == pytest.approx(expected_box(near, 0.5), abs=1.0)


# --- through DetectorCore --------------------------------------------------------------------------------------

SOFA = solid(3.4, 1.0, 0.9, 1.8, height=0.85)
LAMP = solid(2.8, -1.1, 0.4, 0.4, height=1.5)
BACK = solid(5.55, 0.0, 0.1, 9.0, height=2.0)
TRUTH = (Obj("sofa_1", "sofa", 3.4, 1.0, (0.9, 1.8)), Obj("lamp_1", "lamp", 2.8, -1.1, (0.4, 0.4)))


def test_through_the_core_the_oracle_publishes_the_truth_on_its_observed_extent():
    def make(rig):
        det = OracleDetector()
        det.set_vocabulary(*_prompts())
        det.set_truth(TRUTH)                                       # no heights: the vocabulary's (review M8)
        return det

    rig = Rig(detector=make, scene=(SOFA, LAMP, BACK))
    rig.feed(10.0, 12.0)
    rows = {r.id: r for r in objects.parse(rig.message())}
    assert sorted(rows) == ["lamp_1", "sofa_1"]
    h = rig.health()
    assert h["tracks_tentative"] == 0 and h["shapeless"] == 0
    cam_xy = rig.base[:2]
    for truth in TRUTH:
        det = rows[truth.id]
        assert det.measured is True and det.confidence == 1.0
        assert math.hypot(det.x - truth.x, det.y - truth.y) <= 0.15          # the target line (spec §13.3)
        ok, short = objects_check.covers(det, truth, det.id in h["filled"], cam_xy)
        assert ok, (det, truth, short)
    assert "sofa_1" in h["filled"]                                 # its depth is hidden: the vocabulary's
