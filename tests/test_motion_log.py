"""gazebo/tools/motion_log.py: splitting a recording into motions and the per-kind tilt summary (Task I)."""

import json

from go2_nl_control.actions import BALANCE_STAND, MOVE, STOP_MOVE

from gazebo.tools.motion_log import FALL_TILT, main, motions, read_rows, summarize_motions


def move(w, x=0.0, z=0.0):
    return {"r": [MOVE, json.dumps({"x": x, "y": 0, "z": z})], "w": w}


def stop(w):
    return {"r": [STOP_MOVE, ""], "w": w}


def odom(w, roll=0.0, pitch=0.0):
    return {"o": [w, 0.0, 0.0, 0.0, 0.33, roll, pitch], "w": w}


def lap_rows():
    """Stand, a turn (sways 0.6 in the settle after it), a walk, a turn that falls."""
    return [
        {"r": [BALANCE_STAND, ""], "w": 0.0}, odom(0.5, 0.05),
        move(1.0, z=0.1), odom(1.2, 0.1), move(1.1, z=0.2), stop(2.0), odom(2.3, -0.6),     # turn, then sway
        move(3.0, x=0.06), odom(3.1, 0.0, 0.2), move(3.1, x=0.12, z=-0.05), stop(4.0), odom(4.2, 0.3),
        move(5.0, z=-0.1), odom(5.2, 1.4), odom(5.3, 3.1),                                  # falls, no StopMove
    ]


def test_motions_split_at_the_first_move_after_a_stop():
    found = motions(lap_rows())
    assert [m["kind"] for m in found] == ["turn", "walk", "turn"]
    assert [m["start"] for m in found] == [1.0, 3.0, 5.0]
    assert [m["stop"] for m in found] == [2.0, 4.0, None]
    assert [m["tilt"] for m in found] == [0.6, 0.3, 3.1]       # the settle after StopMove counts
    assert [m["fell"] for m in found] == [False, False, True]
    assert found[1]["x_peak"] == 0.12 and found[1]["z_first"] == 0.0 and found[0]["z_peak"] == 0.2
    assert FALL_TILT == 1.0


def test_resent_moves_and_a_second_stop_stay_in_one_motion():
    rows = [move(1.0, z=0.1), move(1.1, z=0.1), stop(1.5), stop(1.6), odom(1.7, 0.2), move(2.0, x=0.1)]
    found = motions(rows)
    assert len(found) == 2 and found[0]["stop"] == 1.5 and found[0]["tilt"] == 0.2
    assert found[1]["tilt"] is None and not found[1]["fell"]   # no odometry after it


def test_summary_per_kind():
    found = motions(lap_rows())
    s = summarize_motions(found)
    assert s["turn"] == {"n": 2, "median": 1.85, "p90": 3.1, "sway": 1, "falls": 1}
    assert s["walk"] == {"n": 1, "median": 0.3, "p90": 0.3, "sway": 0, "falls": 0}
    tilts = [{"kind": "walk", "tilt": t} for t in (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.2)]
    assert summarize_motions(tilts)["walk"] == {"n": 10, "median": 0.55, "p90": 0.9, "sway": 5, "falls": 1}
    assert summarize_motions([])["turn"] == {"n": 0, "median": None, "p90": None, "sway": 0, "falls": 0}


def test_table_reads_a_recording(tmp_path, capsys):
    f = tmp_path / "lap.jsonl"
    rows = lap_rows()
    f.write_text("noise from the container\n" + "\n".join(json.dumps(r) for r in reversed(rows)) + "\n")
    assert [r["w"] for r in read_rows(str(f))] == sorted(r["w"] for r in rows)
    assert main(["table", str(f)]) == 0
    lines = capsys.readouterr().out.strip().splitlines()
    assert len(lines) == 4 and "FELL" in lines[2] and "sway" in lines[0]
    last = json.loads(lines[-1])
    assert last["motions"] == 3 and last["turn"]["falls"] == 1
