#!/usr/bin/env python3
"""Gazebo で「箱の周りを一周」「椅子まで歩く」の合格の条件（設計書 §2）を、LLM なしで測る（R24）。

app コンテナ（ROS 2 Humble）で動かす。式は run_formula にそのまま渡す（翻訳を通らない）。
- run：/model/robot/odometry（nav_msgs/Odometry、Gazebo の本当の位置。odom == Gazebo の world）を
  記録しながら、子プロセスで `python3 -m voice_ltl.run_formula <式> --timeout <秒>` を走らせる。
  子が終わったら TAIL_S 秒だけ記録を続け（止めた後の惰性も最後の位置に入れる）、summarize() の結果を
  JSON 1 行で出す。合格なら終了コード 0、不合格は 1。子の出力は標準エラーにそのまま流す。
- rtf：/clock のシミュレーションの秒数を実際の秒数で割る。{"rtf": …} を JSON 1 行で。
- objects：/go2/objects を 1 通、JSON 1 行で出す。どの行も objects.parse を通れば 0、
  通らない行があるか物がなければ 1、届かなければ 2。

合格の線（THRESHOLDS）：
- lap：box_1 の中心の周りの回転数（winding_number）が 0.9 以上 1.5 未満（裁定 C1）、
  箱の縁から胴の中心まで 0.6 m 以上。
- near：最後の位置で、椅子の中心まで 1.5 m 以内、椅子の方向との角度差 45° 以内、
  椅子の縁まで 0.45 m 以上。
- どちらも：傾き max(|roll|, |pitch|) が 1.0 rad 未満、最初の動作が「立つ」、run_formula の終了コードが 0。
回転数と縁までの距離は、初めて胴の高さが STAND_Z 以上になった（立った）ときからの記録だけで測る。
傾きは記録の全部で、最後の位置は最後の記録で測る。

使い方（Gazebo を立ち上げてから。このリポジトリで）:
    docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 -e SIM_KIND=gazebo app \\
        python3 gazebo/tools/objects_check.py run --expect lap "F around_box_1"
    docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 app python3 gazebo/tools/objects_check.py rtf --seconds 20
    docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 app python3 gazebo/tools/objects_check.py objects
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]          # このリポジトリ（コンテナでは /work）
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from voice_ltl.objects import (DEFAULT_LAYOUT, OBJECTS_TOPIC, Obj, edge_distance,  # noqa: E402
                               parse, winding_number)

Sample = tuple[float, float, float, float, float, float]   # (t, x, y, yaw, z, tilt=max(|roll|,|pitch|))

THRESHOLDS = {"lap": {"winding": (0.9, 1.5), "min_edge_box_1": 0.6},
              "near": {"final_dist_chair_1": 1.5, "final_angle_chair_1_deg": 45.0, "min_edge_chair_1": 0.45},
              "all": {"max_tilt": 1.0, "first_action": "立つ", "exit_code": 0}}

STAND_Z = 0.25               # 胴の高さがこれ以上なら立っている [m]
START_MARK = "開始："          # Session のログの「開始：立つ」
ODOM_TOPIC = "/model/robot/odometry"
CLOCK_TOPIC = "/clock"
TAIL_S = 1.0                 # 子が終わった後も記録する秒数（実際の時間）
CHILD_MARGIN_S = 150.0       # run_formula の --timeout に足す余裕（Go2 を待つ 90 秒と物を待つ 3 秒を含む）
STOP_WAIT_S = 10.0           # 子に SIGINT を送ってから待つ秒数。過ぎたら kill
MSG_WAIT_S = 10.0            # /go2/objects・/clock の最初の 1 通を待つ秒数


def summarize(samples: list[Sample], expect: str, exit_code: int, log: list[str],
              layout=DEFAULT_LAYOUT) -> dict:
    """記録（samples）・run_formula の終了コードと出力（log）から、§2 の値と合否を出す。"""
    if expect not in ("lap", "near"):
        raise ValueError(f"expect は 'lap' か 'near'（{expect!r}）")
    box, chair = _find(layout, "box_1"), _find(layout, "chair_1")
    standing = _from_standing(samples)
    path = [(s[1], s[2]) for s in standing]
    last = samples[-1] if samples else None

    winding = winding_number(path, (box.x, box.y)) if box is not None and path else None
    edge_box = _min_edge(path, box)
    edge_chair = _min_edge(path, chair)
    if chair is not None and last is not None:
        _, x, y, yaw, _, _ = last
        dist = math.hypot(chair.x - x, chair.y - y)
        angle = abs(math.degrees(_wrap(math.atan2(chair.y - y, chair.x - x) - yaw)))
    else:
        dist = angle = None
    max_tilt = max(abs(s[5]) for s in samples) if samples else None
    first = _first_action(log)

    lap, near, every = THRESHOLDS["lap"], THRESHOLDS["near"], THRESHOLDS["all"]
    failures: list[str] = []

    def at_least(name: str, value: float | None, limit: float) -> None:
        if value is None:
            failures.append(f"{name}: not measured")
        elif not value >= limit:
            failures.append(f"{name}: {value:.3f} < {limit:g}")

    def at_most(name: str, value: float | None, limit: float) -> None:
        if value is None:
            failures.append(f"{name}: not measured")
        elif not value <= limit:
            failures.append(f"{name}: {value:.3f} > {limit:g}")

    if expect == "lap":
        lo, hi = lap["winding"]
        if winding is None:
            failures.append("winding: not measured")
        elif not lo <= winding < hi:
            failures.append(f"winding: {winding:.3f} not in [{lo:g}, {hi:g})")
        at_least("min_edge_box_1", edge_box, lap["min_edge_box_1"])
    else:
        at_most("final_dist_chair_1", dist, near["final_dist_chair_1"])
        at_most("final_angle_chair_1_deg", angle, near["final_angle_chair_1_deg"])
        at_least("min_edge_chair_1", edge_chair, near["min_edge_chair_1"])
    if max_tilt is None:
        failures.append("max_tilt: not measured")
    elif not max_tilt < every["max_tilt"]:
        failures.append(f"max_tilt: {max_tilt:.3f} >= {every['max_tilt']:g}")
    if first != every["first_action"]:
        failures.append(f"first_action: {first!r} != {every['first_action']!r}")
    if exit_code != every["exit_code"]:
        failures.append(f"exit_code: {exit_code} != {every['exit_code']}")

    return {"expect": expect, "exit_code": exit_code, "first_action": first,
            "winding_box_1": _r(winding), "min_edge_box_1": _r(edge_box),
            "min_edge_chair_1": _r(edge_chair), "final_dist_chair_1": _r(dist),
            "final_angle_chair_1_deg": _r(angle, 1), "max_tilt": _r(max_tilt),
            "passed": not failures, "failures": failures}


def odometry_sample(msg) -> Sample:
    """nav_msgs/Odometry 1 通 → Sample。t はシミュレーションの時刻（header.stamp）。"""
    stamp, p, q = msg.header.stamp, msg.pose.pose.position, msg.pose.pose.orientation
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return (stamp.sec + stamp.nanosec * 1e-9, p.x, p.y, yaw, p.z, max(abs(roll), abs(pitch)))


def valid_message(payload: object) -> bool:
    """/go2/objects の 1 通で、物が 1 つ以上あり、どの行も objects.parse を通る（落ちる行がない）。"""
    rows = payload.get("objects") if isinstance(payload, dict) else None
    return isinstance(rows, list) and 0 < len(parse(payload)) == len(rows)


# ---- 内部 ----

def _find(layout, obj_id: str) -> Obj | None:
    return next((o for o in layout if o.id == obj_id), None)


def _from_standing(samples: list[Sample]) -> list[Sample]:
    """初めて立った（z ≥ STAND_Z）記録から後ろ。立たなければ []。"""
    k = next((i for i, s in enumerate(samples) if s[4] >= STAND_Z), len(samples))
    return samples[k:]


def _min_edge(path: list[tuple[float, float]], obj: Obj | None) -> float | None:
    if obj is None or not path:
        return None
    return min(edge_distance(x, y, obj) for x, y in path)


def _first_action(log: list[str]) -> str | None:
    """「開始：」がある最初の行の、その後ろ。"""
    for line in log:
        _, mark, rest = line.partition(START_MARK)
        if mark:
            return rest.strip()
    return None


def _wrap(a: float) -> float:
    """(−π, π] に折り返す。"""
    a %= 2 * math.pi
    return a - 2 * math.pi if a > math.pi else a


def _r(v: float | None, nd: int = 3) -> float | None:
    return None if v is None else round(v, nd)


def _say(text: str) -> None:
    print(f"[objects_check] {text}", file=sys.stderr, flush=True)


def _pump(stream, lines: list[str]) -> None:
    """子の標準出力を 1 行ずつ lines に入れ、標準エラーにも流す。"""
    for raw in stream:
        line = raw.rstrip("\n")
        lines.append(line)
        print(line, file=sys.stderr, flush=True)


def _stop_child(child: subprocess.Popen) -> None:
    """run_formula は SIGINT で StopMove を送って終わる。STOP_WAIT_S で終わらなければ kill。"""
    if child.poll() is not None:
        return
    child.send_signal(signal.SIGINT)
    try:
        child.wait(STOP_WAIT_S)
    except subprocess.TimeoutExpired:
        child.kill()
        child.wait()


def _start_ros(name: str):
    import rclpy
    from rclpy.signals import SignalHandlerOptions

    # Ctrl+C は自分で扱う（子に SIGINT を送って StopMove させてから終わる）
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    return rclpy, rclpy.create_node(name)


def _spin_until(rclpy, node, done, timeout: float) -> bool:
    end = time.monotonic() + timeout
    while not done():
        if time.monotonic() >= end:
            return False
        rclpy.spin_once(node, timeout_sec=0.05)
    return True


def _run(args: argparse.Namespace) -> int:
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data

    rclpy, node = _start_ros("go2_objects_check")
    samples: list[Sample] = []
    node.create_subscription(Odometry, ODOM_TOPIC, lambda m: samples.append(odometry_sample(m)),
                             qos_profile_sensor_data)
    lines: list[str] = []
    # 別のセッションにするのは、Ctrl+C が子に二重に届かないようにするため（子へはこちらから 1 回だけ送る）
    child = subprocess.Popen([sys.executable, "-m", "voice_ltl.run_formula", args.formula,
                              "--timeout", f"{args.timeout:g}"],
                             cwd=ROOT, stdout=subprocess.PIPE, text=True, bufsize=1, start_new_session=True)
    reader = threading.Thread(target=_pump, args=(child.stdout, lines), daemon=True)
    reader.start()
    try:
        if not _spin_until(rclpy, node, lambda: child.poll() is not None, args.timeout + CHILD_MARGIN_S):
            _say(f"run_formula が {args.timeout + CHILD_MARGIN_S:g} 秒で終わらないので止めます")
            _stop_child(child)
        _spin_until(rclpy, node, lambda: False, TAIL_S)
    except KeyboardInterrupt:
        _say("中断しました。run_formula を止めます")
        _stop_child(child)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    reader.join(5.0)
    result = summarize(list(samples), args.expect, child.wait(), list(lines))
    print(json.dumps({"formula": args.formula, **result, "samples": len(samples)}, ensure_ascii=False),
          flush=True)
    return 0 if result["passed"] else 1


def _rtf(args: argparse.Namespace) -> int:
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock

    rclpy, node = _start_ros("go2_objects_check_rtf")
    ticks: list[tuple[float, float]] = []    # (実際の時刻 monotonic, シミュレーションの時刻)
    node.create_subscription(
        Clock, CLOCK_TOPIC,
        lambda m: ticks.append((time.monotonic(), m.clock.sec + m.clock.nanosec * 1e-9)),
        qos_profile_sensor_data)
    try:
        if not _spin_until(rclpy, node, lambda: bool(ticks), MSG_WAIT_S):
            print(json.dumps({"rtf": None, "error": f"no {CLOCK_TOPIC} in {MSG_WAIT_S:g} s"}), flush=True)
            return 2
        first = ticks[-1]
        _spin_until(rclpy, node, lambda: time.monotonic() >= first[0] + args.seconds, args.seconds + 5.0)
        last = ticks[-1]
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    rtf = (last[1] - first[1]) / max(last[0] - first[0], 1e-6)
    print(json.dumps({"rtf": round(rtf, 2)}), flush=True)
    return 0


def _objects(args: argparse.Namespace) -> int:
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String

    rclpy, node = _start_ros("go2_objects_check_objects")
    got: list[str] = []
    node.create_subscription(String, OBJECTS_TOPIC, lambda m: got.append(m.data), qos_profile_sensor_data)
    try:
        arrived = _spin_until(rclpy, node, lambda: bool(got), MSG_WAIT_S)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    if not arrived:
        print(json.dumps({"error": f"no {OBJECTS_TOPIC} in {MSG_WAIT_S:g} s"}), flush=True)
        return 2
    try:
        payload = json.loads(got[0])
    except ValueError:
        print(json.dumps({"error": "not JSON", "data": got[0]}, ensure_ascii=False), flush=True)
        return 1
    print(json.dumps(payload, ensure_ascii=False), flush=True)
    return 0 if valid_message(payload) else 1


def _parse(argv: list[str] | None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(prog="objects_check.py",
                                 description="Gazebo の Go2 で、物の周りを一周・物まで歩くの合格の条件を測る")
    sub = ap.add_subparsers(dest="command", required=True)
    run = sub.add_parser("run", help="run_formula を走らせて odometry を記録し、合否を JSON 1 行で出す")
    run.add_argument("--expect", choices=("lap", "near"), required=True,
                     help="lap：box_1 の周りを一周、near：chair_1 の手前まで")
    run.add_argument("--timeout", type=float, default=300.0,
                     help="run_formula に渡す --timeout [s]（既定 300）")
    run.add_argument("formula", help='LTLf の式（例："F around_box_1"）')
    rtf = sub.add_parser("rtf", help="/clock の秒数 ÷ 実際の秒数")
    rtf.add_argument("--seconds", type=float, default=20.0, help="測る実際の秒数（既定 20）")
    sub.add_parser("objects", help="/go2/objects を 1 通出す")
    args = ap.parse_args(argv)
    if args.command == "run" and not args.timeout > 0:
        ap.error("--timeout は 0 より大きい秒数にしてください")
    if args.command == "rtf" and not args.seconds > 0:
        ap.error("--seconds は 0 より大きい秒数にしてください")
    return args


def main(argv: list[str] | None = None) -> int:
    args = _parse(argv)

    def on_signal(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_signal)
    return {"run": _run, "rtf": _rtf, "objects": _objects}[args.command](args)


if __name__ == "__main__":
    sys.exit(main())
