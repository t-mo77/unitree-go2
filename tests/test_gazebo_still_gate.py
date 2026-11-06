"""The sizing gate of the Gazebo bridge (Task I): LiDAR sizes of box_1 and chair_1 come only from
clouds taken while the Go2's body was still.

Measured in Gazebo (Task I, 2026-11-06): the Go2 spawns at 0.5 m and drops onto the floor. A cloud
of that drop (pitch rate 0.57 rad/s, yaw rate under 0.1) passed the old yaw-only gate, ground
points came out above GROUND_Z, and box_1 / chair_1 were sized 1.07 x 0.67 / 1.16 x 0.48 before
the first instruction (set_size keeps the max, so for good). Standing up, trotting in place and
walking did the same (roll rates up to 1.6 rad/s with yaw rates under 0.1).
"""

import math

import pytest

from gazebo.bridge_core import (SIZE_STILL_RATE, SIZE_STILL_S, SIZE_STILL_SPEED, ObjectMemory,
                                StillGate)

STILL = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))


def test_gate_constants():
    assert (SIZE_STILL_RATE, SIZE_STILL_SPEED, SIZE_STILL_S) == (0.1, 0.05, 0.3)


def test_no_odometry_yet_no_sizing():
    assert not StillGate().allows(5.0)


def test_a_cloud_taken_after_the_body_has_been_still_long_enough_is_used():
    g = StillGate()
    for k in range(11):                       # odometry at 50 Hz from t = 1.0 to 1.2, all still
        g.observe(1.0 + 0.02 * k, *STILL)
    assert g.allows(1.0 + SIZE_STILL_S)       # taken SIZE_STILL_S after the stillness began
    assert g.allows(2.0)
    assert not g.allows(1.0 + SIZE_STILL_S - 0.01)   # taken too soon after the body came to rest


@pytest.mark.parametrize("linear, angular", [
    ((0.0, 0.0, 0.0), (0.0, 0.57, 0.05)),     # pitching (the drop after spawn; standing up)
    ((0.0, 0.0, 0.0), (1.57, 0.0, 0.0)),      # rolling (trotting in place)
    ((0.0, 0.0, 0.0), (0.0, 0.0, -0.2)),      # turning on the spot
    ((0.3, 0.0, 0.0), (0.0, 0.0, 0.0)),       # walking straight
    ((0.0, 0.0, -0.4), (0.0, 0.0, 0.0)),      # dropping onto the floor
])
def test_any_motion_closes_the_gate_until_still_again(linear, angular):
    g = StillGate()
    g.observe(1.0, *STILL)
    g.observe(1.5, *STILL)
    assert g.allows(1.4)
    g.observe(1.52, linear, angular)
    assert not g.allows(1.4) and not g.allows(1.52)   # whatever the cloud's stamp
    g.observe(1.6, *STILL)
    assert not g.allows(1.7)                          # still again, but not long enough
    assert g.allows(1.6 + SIZE_STILL_S)


def test_the_limits_themselves_count_as_still():
    g = StillGate()
    g.observe(0.0, (SIZE_STILL_SPEED, 0.0, 0.0), (SIZE_STILL_RATE, -SIZE_STILL_RATE, SIZE_STILL_RATE))
    assert g.allows(SIZE_STILL_S)
    g.observe(0.4, (SIZE_STILL_SPEED + 0.01, 0.0, 0.0), STILL[1])
    assert not g.allows(1.0)


def test_non_finite_odometry_closes_the_gate():
    g = StillGate()
    g.observe(0.0, *STILL)
    g.observe(0.1, (math.nan, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert not g.allows(1.0)


def test_the_drop_after_spawn_leaves_the_layout_sizes_alone():
    """The Gazebo case: clouds while the Go2 drops after spawn are not measured; the first
    measurement comes once it lies still."""
    g, m = StillGate(), ObjectMemory()
    m.update([("box_1", "box", 2.5, 1.5)], 0.0)
    measured = []

    def cloud(stamp, size):              # what _on_cloud does with one cloud
        if g.allows(stamp):
            m.set_size("box_1", size)
            measured.append(stamp)

    g.observe(0.40, (0.0, 0.0, -0.9), (0.0, -0.57, 0.06))   # dropping, pitching
    cloud(0.40, (1.07, 0.67))                                  # ground points inflate it
    g.observe(0.90, *STILL)                                    # lying still
    cloud(0.94, (1.16, 0.48))                                  # taken while it was still settling
    for k in range(1, 30):
        g.observe(0.90 + 0.02 * k, *STILL)
    cloud(1.30, (0.51, 0.51))
    assert measured == [1.30] and m.objects()[0].size == (0.51, 0.51)
