"""voice_ltl.sensor_check: the real-robot LiDAR check (docs/real_robot.md). ROS is never started here."""

import math

import numpy as np
import pytest

from go2_nl_control.perception import PerceptionConfig
from go2_nl_control.supervisor import BLOCKED, CLEAR, SENSOR_LOST

from voice_ltl.sensing import Reading, Sensing
from voice_ltl.sensor_check import PERIOD_S, config_from_args, format_line, header, run


def test_line_has_both_judgements_and_nearest_distances():
    assert format_line(Reading(BLOCKED, CLEAR, 0.4123, None)) == "前方 あり 0.41 m ｜ 後方 なし —"
    assert format_line(Reading(SENSOR_LOST, SENSOR_LOST, None, 0.8)) == \
        "前方 途切れ — ｜ 後方 途切れ 0.80 m"


def _box_in_lidar(cfg: PerceptionConfig, x_body: float, n: int = 20) -> np.ndarray:
    """n points of a box face x_body [m] ahead of (behind, if < 0) the body centre, 0.1 m up,
    in the LiDAR frame of cfg's mount (the inverse of ObstacleMonitor._to_body)."""
    body = np.column_stack([np.full(n, x_body), np.linspace(-0.15, 0.15, n), np.full(n, 0.1)])
    body -= np.array([cfg.lidar_x, 0.0, cfg.lidar_z])
    cp, sp = math.cos(cfg.lidar_pitch), math.sin(cfg.lidar_pitch)
    cy, sy = math.cos(cfg.lidar_yaw), math.sin(cfg.lidar_yaw)
    rot = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return body @ rot


# the zip's placeholder mount, and the Go2 model's L1 (under the head, upside down; docs/real_robot.md)
MOUNTS = [PerceptionConfig(lidar_x=0.28, lidar_z=0.10, lidar_pitch=0.0),
          PerceptionConfig(lidar_x=0.28945, lidar_z=-0.046825, lidar_pitch=2.8782)]


@pytest.mark.parametrize("cfg", MOUNTS, ids=["placeholder", "go2_description"])
@pytest.mark.parametrize("x_body, line", [
    # docs/real_robot.md: a box 0.4 m ahead -> 前方 あり; 0.8 m -> なし (the distance is still shown)
    (0.4, "前方 あり 0.40 m ｜ 後方 なし —"),
    (0.8, "前方 なし 0.80 m ｜ 後方 なし —"),
    # the same behind
    (-0.4, "前方 なし — ｜ 後方 あり 0.40 m"),
    (-0.8, "前方 なし — ｜ 後方 なし 0.80 m"),
])
def test_readme_box_distances(cfg, x_body, line):
    assert format_line(Sensing(cfg).update(_box_in_lidar(cfg, x_body), now=0.0)) == line


def test_config_defaults_and_mount_options():
    assert config_from_args([]) == PerceptionConfig()
    cfg = config_from_args(["--lidar-x", "0.29", "--lidar-z", "-0.05", "--lidar-pitch", "2.88"])
    assert (cfg.lidar_x, cfg.lidar_z, cfg.lidar_pitch) == (0.29, -0.05, 2.88)
    assert cfg.stop_distance == PerceptionConfig().stop_distance


def test_header_names_the_mount_and_the_distances():
    cfg = config_from_args(["--lidar-x", "0.28945", "--lidar-z", "-0.046825", "--lidar-pitch", "2.8782"])
    text = "\n".join(header(cfg))
    assert "Go2 には何も送りません" in text and "Ctrl+C" in text
    assert "lidar_x 0.289 m" in text and "lidar_z -0.047 m" in text and "lidar_pitch 2.878 rad" in text
    assert "0.50 m" in text and "0.70 m" in text


class FakeLink:
    def __init__(self):
        self.spins, self.sent, self.closed, self.read_at = 0, [], [], []

    def spin_once(self):
        self.spins += 1

    def reading(self, now):
        self.read_at.append(now)
        return Reading(CLEAR, BLOCKED, None, 0.42)

    def send(self, cmd):          # never called by sensor_check
        self.sent.append(cmd)

    def close(self, stop_times=3):
        self.closed.append(stop_times)


def test_run_prints_every_half_second_sends_nothing_and_closes_without_stopmove():
    link, lines, ticks = FakeLink(), [], [0]

    def clock():
        return round(ticks[0] * 0.05, 6)

    def sleep(dt):
        assert dt == 0.05
        ticks[0] += 1
        if clock() > 2.0:
            raise KeyboardInterrupt   # Ctrl+C

    run(link, out=lines.append, clock=clock, sleep=sleep)
    assert lines == ["前方 なし — ｜ 後方 あり 0.42 m"] * 5      # t = 0, 0.5, 1.0, 1.5, 2.0
    assert link.read_at == [0.0, 0.5, 1.0, 1.5, 2.0] and PERIOD_S == 0.5
    assert link.spins == 41                                   # the receive queue is drained every 0.05 s
    assert link.sent == [] and link.closed == [0]             # no StopMove either


def test_run_closes_without_stopmove_when_receiving_fails():
    link = FakeLink()

    def boom():
        raise RuntimeError("callback failed")
    link.spin_once = boom
    with pytest.raises(RuntimeError):
        run(link, out=lambda line: None, clock=lambda: 0.0, sleep=lambda dt: None)
    assert link.sent == [] and link.closed == [0]


def test_late_loop_does_not_print_a_burst():
    link, lines, t = FakeLink(), [], [0.0]

    def sleep(dt):
        t[0] += 1.7 if t[0] == 0.0 else 0.05   # one slow turn (1.7 s), then normal
        if t[0] > 2.4:
            raise KeyboardInterrupt
    run(link, out=lines.append, clock=lambda: round(t[0], 6), sleep=sleep)
    assert link.read_at == [0.0, 1.7, 2.2]
