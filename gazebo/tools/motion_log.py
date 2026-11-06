#!/usr/bin/env python3
"""Gazebo の Go2 の、動作ごとの傾きを記録して数える（Task I。objects_check.py の run と並べて使う）。

app コンテナ（ROS 2 Humble）で動かす。
- record [--seconds 900]：/model/robot/odometry（RECORD_EVERY 通に 1 通。Gazebo の 50 Hz なら 25 Hz）・
  /api/sport/request・/go2/objects の大きさの変化を、1 行 1 つの JSON（JSONL）で標準出力に出す。
  --seconds か Ctrl+C で終わる。
- table FILE：record の JSONL を動作に分け（motions）、動作ごとの最大傾きの表と、種類ごとのまとめ
  （summarize_motions）を出す。最後の行はまとめの JSON。

1 つの動作は、StopMove の後の最初の Move から、次の動作の最初の Move の前まで（StopMove の後の
待ちの揺れも入る）。最後の動作は記録の終わりまで。Move の x が一度でも 0 でなければ walk、
そうでなければ turn。傾きは max(|roll|, |pitch|)、FALL_TILT 以上は転んだ動作（橋渡しの Ruling R38 と同じ線）。

JSONL の行（w は実際の時刻）：
    {"o": [sim 時刻, x, y, yaw, z, roll, pitch], "w": …}
    {"r": [api_id, parameter], "w": …}
    {"s": {"box_1": [sx, sy], …}, "w": …}

使い方（Gazebo を立ち上げてから。このリポジトリで）:
    docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 --name go2-objects-rec app \\
        python3 gazebo/tools/motion_log.py record --seconds 900 > lap.jsonl &
    docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 -e SIM_KIND=gazebo app \\
        python3 gazebo/tools/objects_check.py run --expect lap "F around_box_1"
    docker stop go2-objects-rec
    python3 gazebo/tools/motion_log.py table lap.jsonl
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]          # このリポジトリ（コンテナでは /work）
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from go2_nl_control.actions import MOVE, STOP_MOVE  # noqa: E402
from voice_ltl.objects import OBJECTS_TOPIC  # noqa: E402

ODOM_TOPIC = "/model/robot/odometry"
REQUEST_TOPIC = "/api/sport/request"
RECORD_EVERY = 2          # odometry を何通に 1 通残すか
FALL_TILT = 1.0           # これ以上の傾きは転んだ動作 [rad]（gazebo/bridge_core.py の FALL_TILT と同じ）
SWAY_TILT = 0.5           # まとめの「0.5〜1.0」の下の線 [rad]


def motions(rows: list[dict]) -> list[dict]:
    """record の行（時刻 w の順）を動作に分ける。各動作：
    {"start": w, "stop": StopMove の w か None, "kind": "walk"|"turn", "z_first": 最初の Move の z,
     "z_peak": |z| の最大, "x_peak": |x| の最大, "tilt": 窓の中の最大傾き, "fell": tilt >= FALL_TILT}"""
    found: list[dict] = []
    cur = None
    for row in rows:
        if "r" not in row:
            continue
        api_id, parameter = row["r"]
        if api_id == MOVE:
            p = json.loads(parameter) if parameter else {}
            x, z = float(p.get("x", 0.0)), float(p.get("z", 0.0))
            if cur is None or cur["stop"] is not None:
                cur = {"start": row["w"], "stop": None, "kind": "turn", "z_first": z, "z_peak": 0.0, "x_peak": 0.0}
                found.append(cur)
            if abs(x) > 1e-9:
                cur["kind"] = "walk"
            cur["z_peak"], cur["x_peak"] = max(cur["z_peak"], abs(z)), max(cur["x_peak"], abs(x))
        elif api_id == STOP_MOVE and cur is not None and cur["stop"] is None:
            cur["stop"] = row["w"]
    odom = [(row["w"], max(abs(row["o"][5]), abs(row["o"][6]))) for row in rows if "o" in row]
    for k, m in enumerate(found):
        end = found[k + 1]["start"] if k + 1 < len(found) else math.inf
        tilts = [tilt for w, tilt in odom if m["start"] <= w < end]
        m["tilt"] = max(tilts, default=None)
        m["fell"] = m["tilt"] is not None and m["tilt"] >= FALL_TILT
    return found


def summarize_motions(found: list[dict]) -> dict:
    """種類ごと（walk・turn）：数・最大傾きの中央値・上位 10 %（大きい方から 1 割の境、nearest-rank）・
    SWAY_TILT 以上 FALL_TILT 未満の数・転んだ数。"""
    out = {}
    for kind in ("walk", "turn"):
        tilts = sorted(m["tilt"] for m in found if m["kind"] == kind and m["tilt"] is not None)
        if not tilts:
            out[kind] = {"n": 0, "median": None, "p90": None, "sway": 0, "falls": 0}
            continue
        n = len(tilts)
        median = tilts[n // 2] if n % 2 else (tilts[n // 2 - 1] + tilts[n // 2]) / 2
        p90 = tilts[max(0, math.ceil(0.9 * n) - 1)]
        out[kind] = {"n": n, "median": round(median, 3), "p90": round(p90, 3),
                     "sway": sum(SWAY_TILT <= t < FALL_TILT for t in tilts),
                     "falls": sum(t >= FALL_TILT for t in tilts)}
    return out


def read_rows(path: str) -> list[dict]:
    rows = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            if line.startswith("{"):
                rows.append(json.loads(line))
    return sorted(rows, key=lambda r: r["w"])


def _table(args: argparse.Namespace) -> int:
    rows = read_rows(args.file)
    found = motions(rows)
    t0 = next((r["w"] for r in rows if "o" in r), rows[0]["w"] if rows else 0.0)
    for k, m in enumerate(found, 1):
        tilt = "-" if m["tilt"] is None else f"{m['tilt']:.2f}"
        stop = "-" if m["stop"] is None else f"{m['stop'] - m['start']:.1f}s"
        mark = "  FELL" if m["fell"] else ("  sway" if m["tilt"] is not None and m["tilt"] >= SWAY_TILT else "")
        print(f"{k:3d} {m['start'] - t0:7.1f} {m['kind']:4s} until StopMove {stop:>6s} "
              f"x_peak {m['x_peak']:.2f} z_first {m['z_first']:+.2f} z_peak {m['z_peak']:.2f} tilt {tilt}{mark}")
    print(json.dumps({"motions": len(found), **summarize_motions(found)}, ensure_ascii=False))
    return 0


def _record(args: argparse.Namespace) -> int:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from unitree_api.msg import Request

    rclpy.init()
    node = rclpy.create_node("motion_log")
    state = {"k": 0, "sizes": None}

    def out(row: dict) -> None:
        row["w"] = round(time.time(), 3)
        print(json.dumps(row), flush=True)

    def on_odom(m) -> None:
        state["k"] += 1
        if state["k"] % RECORD_EVERY:
            return
        p, q = m.pose.pose.position, m.pose.pose.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        out({"o": [round(v, 3) for v in (stamp, p.x, p.y, yaw, p.z, roll, pitch)]})

    def on_request(m) -> None:
        out({"r": [int(m.header.identity.api_id), m.parameter]})

    def on_objects(m) -> None:
        try:
            sizes = {o["id"]: o["size"] for o in json.loads(m.data).get("objects", [])}
        except (ValueError, AttributeError, KeyError, TypeError):
            return
        if sizes != state["sizes"]:
            state["sizes"] = sizes
            out({"s": sizes})

    node.create_subscription(Odometry, ODOM_TOPIC, on_odom, qos_profile_sensor_data)
    node.create_subscription(Request, REQUEST_TOPIC, on_request, qos_profile_sensor_data)
    node.create_subscription(String, OBJECTS_TOPIC, on_objects, qos_profile_sensor_data)
    end = time.monotonic() + args.seconds
    try:
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    rec = sub.add_parser("record", help="odometry・指令・物の大きさを JSONL で出す")
    rec.add_argument("--seconds", type=float, default=900.0)
    tab = sub.add_parser("table", help="record の JSONL を動作ごとの表とまとめにする")
    tab.add_argument("file")
    args = ap.parse_args(argv)
    return _record(args) if args.cmd == "record" else _table(args)


if __name__ == "__main__":
    sys.exit(main())
