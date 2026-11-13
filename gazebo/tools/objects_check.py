#!/usr/bin/env python3
"""Gazebo で「物の周りを一周」「物まで歩く」の合格の条件（物の設計書 §2、open-objects spec §10.8）と、
検出器の正しさ（spec §11.11・§13.3）を、LLM なしで測る（R24）。

app コンテナ（ROS 2 Humble）で動かす。式は run_formula にそのまま渡す（翻訳を通らない）。
- run：/model/robot/odometry（nav_msgs/Odometry、Gazebo の本当の位置。odom == Gazebo の world）と /clock を
  記録しながら、子プロセスで `python3 -m voice_ltl.run_formula <式> --timeout <秒>` を走らせる。
  子が終わったら TAIL_S 秒（3 秒）だけ記録を続け（止めた後の惰性と、その後に倒れるのも入れる）、summarize() の結果を
  JSON 1 行で出す（rtf_stats を "rtf" に）。合格なら終了コード 0、不合格は 1、配置か目標が違えば 2。
  子の出力は標準エラーにそのまま流す。/go2/objects も受け、最後の 1 通の "source" を summarize に渡す。
  --target <正解の id>：その正解の足跡で手前・周回の線を測る（式の id は検出器の id でよい。M10 の resolve で引く）。
  --layout <名前>：配置ファイル（既定は GO2_LAYOUT、なければ household）。/gazebo/objects_truth の "layout" が
  違えば、子を始めずに終了コード 2。app のサービスは GO2_LAYOUT を渡さないので、docker compose run では
  --layout か -e GO2_LAYOUT=<名前> で渡す（外で GO2_LAYOUT を付けても、コンテナの中には届かない）。
- watch：run と同じく記録するが、子は始めない（app が Go2 を動かす。H6 の声の段）。--seconds で、または
  settled()（動いた後に 10 秒止まった）で終わり、summarize(…, exit_code=None, log=None) を "child": null で出す。
  合格 0、不合格 1、odometry なし 2。
- accuracy：Go2 が止まり、/go2/detector_status の still_inferences が 3 以上になるまで（か --seconds）待ち、
  最後の /go2/objects（"source": "detector" でなければ 2）と /gazebo/objects_truth を accuracy() で比べる。
  /sim/world のボタンの箱と、状態の filled・ids_retired も使う。合格 0、不合格 1、データなし 2。
- resolve：/go2/objects の行で、正解の --target と組になり measured: true のものを待ち、その id を出す
  （{"target", "id", "measured", "listed_after_still_s"}）。見つかれば 0、時間切れ 1、データなし 2。
  listed_after_still_s は、このコマンドが見た止まり（歩いた後に止まった時刻）から、その行が measured: true で
  初めて届くまでの秒数（ListedWait）。止まった後や行が出た後に始めると null（わけは標準エラー）なので、
  目標が見えるようになる動きの前に、別に始めておく（H6 M4）。
- rtf：実際の時間の窓（--seconds 秒、monotonic）の間に /clock が進んだ秒数を、窓の長さで割る。{"rtf": …} を JSON 1 行で。
- objects：/go2/objects を 1 通、JSON 1 行で出す。どの行も objects.parse を通れば 0、
  通らない行があるか物がなければ 1、届かなければ 2。

合格の線（THRESHOLDS）：
- --target なし（§2 のまま）：
  - lap：box_1 の中心の周りの回転数（winding_number）が 0.9 以上 1.5 未満（裁定 C1）、
    箱の縁から胴の中心まで 0.6 m 以上。
  - near：最後の位置で、椅子の中心まで 1.5 m 以内、椅子の方向との角度差 45° 以内、椅子の縁まで 0.45 m 以上。
  - どちらも：最初の動作が「立つ」。
- --target あり（spec §10.8。物の縁から測る）：
  - near：最後の胴の中心から目標の足跡まで 1.5 m 以内（final_edge）、最後の向きと足跡のどこかの点との差 45° 以内
    （final_footprint_angle_deg）、回の間のすべての足跡（目標とほかの物）までの最小 0.30 m 以上（min_clearance）。
    final_edge が 0.50 m 未満なら notes に書く（記録だけ）。
  - lap：目標の中心の周りの巻き数 0.9 以上 1.5 未満、目標の足跡まで 0.6 m 以上（min_edge）。
  - 目標が chair_1（near）なら §2 の椅子の線も、box_1（lap）なら §2 の箱の値も（同じ線なので失敗は 1 回）。
    目標が box_1・near・source が logical のときだけ、箱の中心から 1.5 m・45°（O16、O28）。
  - 「立つ」が要るのは最初の記録が伏せ（z < STAND_Z）のときだけ（裁定 C1）。
- どちらも：傾き max(|roll|, |pitch|) が 1.0 rad 未満、run_formula の終了コードが 0。
  watch（exit_code と log が None）は、終了コードと最初の動作を確かめない。
回転数と縁までの距離は、初めて胴の高さが STAND_Z 以上になった（立った）ときからの記録だけで測る。
傾きは記録の全部で、最後の位置は最後の記録で測る。

使い方（Gazebo を立ち上げてから。このリポジトリで）:
    docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 -e SIM_KIND=gazebo app \\
        python3 gazebo/tools/objects_check.py run --expect lap "F around_box_1"
    docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 -e SIM_KIND=gazebo app \\
        python3 gazebo/tools/objects_check.py run --expect near --target sofa_1 "F near_sofa_1"
    docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 app \\
        python3 gazebo/tools/objects_check.py resolve --target sofa_1
    docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 app \\
        python3 gazebo/tools/objects_check.py accuracy --target sofa_1
    docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 app python3 gazebo/tools/objects_check.py rtf --seconds 20
    docker compose run --rm --no-deps -T -e ROS_DOMAIN_ID=77 app python3 gazebo/tools/objects_check.py objects
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import statistics
import subprocess
import sys
import threading
import time
from collections.abc import Mapping, Sequence
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]          # このリポジトリ（コンテナでは /work）
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from voice_ltl import layout as layouts  # noqa: E402
from voice_ltl.objects import (DEFAULT_LAYOUT, OBJECTS_TOPIC, Obj, edge_distance,  # noqa: E402
                               footprint_angle, parse, parse_source, winding_number, wrap_rad)

Sample = tuple[float, float, float, float, float, float]   # (t, x, y, yaw, z, tilt=max(|roll|,|pitch|))

THRESHOLDS = {"lap": {"winding": (0.9, 1.5), "min_edge_box_1": 0.6},
              "near": {"final_dist_chair_1": 1.5, "final_angle_chair_1_deg": 45.0, "min_edge_chair_1": 0.45},
              "all": {"max_tilt": 1.0, "first_action": "立つ", "exit_code": 0},
              # open-objects spec §10.8: measured from the target's footprint, for every size
              "target_near": {"max_edge": 1.5, "max_angle_deg": 45.0, "min_clearance": 0.30},
              "target_lap": {"winding": (0.9, 1.5), "min_edge": 0.6},
              # O16: 「箱の前まで」 from the box's centre, in logical mode only (O28)
              "box_centre": {"final_dist_box_1": 1.5, "final_angle_box_1_deg": 45.0},
              # spec §13.3 (provisional; may be revisited once, with the reason recorded)
              "accuracy": {"max_pos_err_target": 0.15, "max_pos_err": 0.35, "median_pos_err": 0.25,
                           "max_size_err": 0.4, "min_label_acc": 0.8, "min_recall": 0.6, "max_false_pos": 1}}

STAND_Z = 0.25               # 胴の高さがこれ以上なら立っている [m]
START_MARK = "開始："          # Session のログの「開始：立つ」
ODOM_TOPIC = "/model/robot/odometry"
CLOCK_TOPIC = "/clock"
TRUTH_TOPIC = "/gazebo/objects_truth"     # == gazebo.bridge_core.TRUTH_TOPIC (spec §11.3)
STATUS_TOPIC = "/go2/detector_status"     # the detector's 1 Hz status (spec §11.9)
WORLD_TOPIC = "/sim/world"                # {"boxes": [[x, y, size], ...], "collided": bool}
DEFAULT_LAYOUT_NAME = "household"         # GO2_LAYOUT unset: the Gazebo default from S7 on (spec §12.5)
TAIL_S = 3.0                 # 子が終わった後も記録する秒数（実際の時間）。終わった後に倒れるのも
                             # max_tilt に入れる（1 秒では、終わって 1.2〜2 秒で倒れた 2 回を見逃した）
CHILD_MARGIN_S = 150.0       # run_formula の --timeout に足す余裕（Go2 を待つ 90 秒と物を待つ 3 秒を含む）
STOP_WAIT_S = 10.0           # 子に SIGINT を送ってから待つ秒数。過ぎたら kill
MSG_WAIT_S = 10.0            # /go2/objects・/clock・/gazebo/objects_truth の最初の 1 通を待つ秒数
NOTE_FINAL_EDGE = 0.50       # spec §10.8: the final d_e to the target is recorded against 0.50 (never a failure)
PAIR_MIN_RADIUS = 0.75       # rows pair within max(0.75, truth radius) of a truth centre (spec §11.11)
COVER_TOL = 0.001            # covers(): 1 mm
ACCURACY_RANGE_M = 5.0       # accuracy --range: truth farther than this is out of recall
STILL_INFERENCES = 3         # accuracy waits for this many inferences in the current still interval
SETTLED_POS_M = 0.05         # settled(): the last quiet_s stay within this of the last sample
SETTLED_YAW_DEG = 3.0
RTF_WINDOW_S = 10.0          # rtf_stats(): wall-clock windows
WATCH_CHECK_S = 1.0          # watch checks settled() this often
_EPS = 1e-9


def summarize(samples: list[Sample], expect: str, exit_code: int | None, log: list[str] | None,
              layout=DEFAULT_LAYOUT, target: Obj | None = None, others: Sequence[Obj] = (),
              source: str | None = None, rtf: dict | None = None) -> dict:
    """記録（samples）・run_formula の終了コードと出力（log）から、合格の線の値と合否を出す。
    target なし：§2 の値（layout の box_1・chair_1）。target あり：その足跡から測る線（spec §10.8）と、
    chair_1・box_1 なら §2 の線（目標が box_1・near・source が logical なら箱の中心の線。O16）。
    exit_code・log が None（watch）なら、終了コードと最初の動作は確かめない。rtf があれば "rtf" に入れる。"""
    if expect not in ("lap", "near"):
        raise ValueError(f"expect は 'lap' か 'near'（{expect!r}）")
    if target is None:
        result = _section2(samples, expect, exit_code, log, layout)
    else:
        result = _target(samples, expect, exit_code, log, target, others, source)
    if rtf is not None:
        result["rtf"] = rtf
    return result


def first_action_required(samples: list[Sample]) -> str | None:
    """裁定 C1：最初の記録が伏せ（z < STAND_Z）なら「立つ」、立っていれば None（回る・歩くから始めてよい）。
    記録がなければ「立つ」（厳しいほう）。"""
    if not samples or samples[0][4] < STAND_Z:
        return THRESHOLDS["all"]["first_action"]
    return None


def last_source(payloads: Sequence[object]) -> str | None:
    """最後の /go2/objects（JSON を読んだもの。読めなかったら None）の "source"（objects.parse_source）。"""
    return parse_source(payloads[-1]) if payloads else None


def settled(samples: list[Sample], moved_m: float = 0.3, quiet_s: float = 10.0) -> bool:
    """watch の早い終わり：胴が最初の記録から moved_m 以上動いたことがあり、最後の quiet_s 秒（シミュレーションの
    時間）の記録がどれも最後の記録から SETTLED_POS_M・SETTLED_YAW_DEG の内にある。"""
    if not samples:
        return False
    x0, y0 = samples[0][1], samples[0][2]
    if not any(math.hypot(s[1] - x0, s[2] - y0) >= moved_m for s in samples):
        return False
    t, x, y, yaw = samples[-1][:4]
    since = t
    for s in reversed(samples):
        if (math.hypot(s[1] - x, s[2] - y) > SETTLED_POS_M
                or abs(math.degrees(wrap_rad(s[3] - yaw))) > SETTLED_YAW_DEG):
            break
        since = s[0]
    return t - since >= quiet_s - _EPS


def pair_rows(det: Sequence[Obj], truth: Sequence[Obj]) -> list[tuple[Obj, Obj]]:
    """検出の行と正解の行の組（spec §11.11）：名前によらず、中心の距離が近い順に 1 対 1、
    max(PAIR_MIN_RADIUS, 正解の半径) 以内。組にした順（近い順）に返す。"""
    det, truth = tuple(det), tuple(truth)
    candidates = []
    for i, d in enumerate(det):
        for j, t in enumerate(truth):
            dist = math.hypot(d.x - t.x, d.y - t.y)
            if dist <= max(PAIR_MIN_RADIUS, t.radius) + _EPS:
                candidates.append((dist, i, j))
    used_det, used_truth, pairs = set(), set(), []
    for _, i, j in sorted(candidates):
        if i in used_det or j in used_truth:
            continue
        used_det.add(i)
        used_truth.add(j)
        pairs.append((det[i], truth[j]))
    return pairs


def covers(det: Obj, truth: Obj, filled: bool, cam_xy: tuple[float, float]) -> tuple[bool, float]:
    """検出の足跡（軸に沿った四角）が正解の足跡を覆うか（O12）と、補った奥行きの足りない分（裁定 C2）。
    filled でなければ、4 辺とも COVER_TOL の内で含めば真、足りない分は 0。filled なら、奥行きの軸は
    |正解の中心 − cam_xy| が大きいほうの軸で、もう一方の軸の 2 辺と奥行きの手前の辺を含めば真（見えた所だけ）。
    足りない分は奥行きの軸で max(0, 正解の奥の辺 − 検出の奥の辺)（カメラから離れる向き）で、失敗にはしない。"""
    d_lo, d_hi = _extent(det)
    t_lo, t_hi = _extent(truth)

    def side(axis: int, low: bool) -> bool:
        if low:
            return d_lo[axis] <= t_lo[axis] + COVER_TOL
        return d_hi[axis] >= t_hi[axis] - COVER_TOL

    if not filled:
        return all(side(a, True) and side(a, False) for a in (0, 1)), 0.0
    depth = 0 if abs(truth.x - cam_xy[0]) >= abs(truth.y - cam_xy[1]) else 1
    other = 1 - depth
    away = (truth.x - cam_xy[0], truth.y - cam_xy[1])[depth] >= 0     # the object lies on the + side
    ok = side(other, True) and side(other, False) and side(depth, away)
    short = t_hi[depth] - d_hi[depth] if away else d_lo[depth] - t_lo[depth]
    return ok, max(0.0, short)


def accuracy(det_rows: Sequence[Obj], truth_rows: Sequence[Obj], lay: layouts.Layout,
             button_boxes: Sequence[Sequence[float]], robot_xy: tuple[float, float], targets: Sequence[str],
             thresholds: Mapping[str, float] = THRESHOLDS["accuracy"], filled: frozenset[str] = frozenset(),
             range_m: float = ACCURACY_RANGE_M) -> dict:
    """検出の正しさ（spec §11.11・§13.3）。範囲の中の正解：見えていて、robot_xy から range_m 以内で、配置で
    detectable。組は pair_rows（正解とボタンの箱（[x, y, 辺]）を一緒に）で、範囲の外の正解とも組になる
    （誤検出にしないが recall に数えない）。ボタンの箱と組になった行は button_boxes_seen（D9）。
    組ごと：pos_err・size_err・label_ok（配置の accept）・covers と fill_short_m（filled は検出の id。
    カメラの位置は robot_xy で代える）。線は thresholds（組がなければ組の線は測らない。範囲に正解がなければ
    recall は None で不合格）。"""
    truth = tuple(truth_rows)
    boxes = tuple(Obj(f"sim_box_{k}", "box", float(b[0]), float(b[1]), (float(b[2]), float(b[2])))
                  for k, b in enumerate(button_boxes, 1))
    paired = pair_rows(det_rows, truth + boxes)
    seen_boxes = sorted(d.id for d, t in paired if t in boxes)
    matches = [(d, t) for d, t in paired if t not in boxes]
    matched_ids = {d.id for d, _ in paired}
    false_ids = sorted(d.id for d in det_rows if d.id not in matched_ids)

    in_scope = sorted(t.id for t in truth if _in_scope(t, lay, robot_xy, range_m))
    found = {t.id for _, t in matches}
    target_set = set(targets)
    pairs = []
    for d, t in sorted(matches, key=lambda m: m[1].id):
        ok, short = covers(d, t, d.id in filled, robot_xy)
        entry = lay.find(t.id)
        pairs.append({"truth": t.id, "id": d.id, "label": d.label,
                      "label_ok": d.label in (entry.accept if entry is not None else (t.label,)),
                      "pos_err": math.hypot(d.x - t.x, d.y - t.y),
                      "size_err": max(abs(d.size[0] - t.size[0]), abs(d.size[1] - t.size[1])),
                      "covers": ok, "filled": d.id in filled, "fill_short_m": short,
                      "in_scope": t.id in in_scope, "target": t.id in target_set})

    pos = [p["pos_err"] for p in pairs]
    max_pos = max(pos) if pos else None
    median_pos = statistics.median(pos) if pos else None
    max_size = max((p["size_err"] for p in pairs), default=None)
    label_acc = sum(p["label_ok"] for p in pairs) / len(pairs) if pairs else None
    recall = len(found & set(in_scope)) / len(in_scope) if in_scope else None

    lines = _Lines()
    by_truth = {p["truth"]: p for p in pairs}
    for target in targets:
        p = by_truth.get(target)
        if p is None:
            lines.failures.append(f"max_pos_err_target: {target} not paired")
        else:
            lines.at_most("max_pos_err_target", p["pos_err"], thresholds["max_pos_err_target"], target)
    if pairs:
        lines.at_most("max_pos_err", max_pos, thresholds["max_pos_err"])
        lines.at_most("median_pos_err", median_pos, thresholds["median_pos_err"])
        lines.at_most("max_size_err", max_size, thresholds["max_size_err"])
        lines.at_least("min_label_acc", label_acc, thresholds["min_label_acc"])
    if recall is None:
        lines.failures.append("min_recall: no detectable truth in range")
    else:
        lines.at_least("min_recall", recall, thresholds["min_recall"])
    if len(false_ids) > thresholds["max_false_pos"]:
        lines.failures.append(f"max_false_pos: {len(false_ids)} > {thresholds['max_false_pos']}")

    for p in pairs:
        for key in ("pos_err", "size_err", "fill_short_m"):
            p[key] = _r(p[key])
    return {"pairs": pairs, "in_scope": in_scope, "missed": sorted(set(in_scope) - found),
            "recall": _r(recall), "label_acc": _r(label_acc), "max_pos_err": _r(max_pos),
            "median_pos_err": _r(median_pos), "max_size_err": _r(max_size),
            "false_pos": len(false_ids), "false_pos_ids": false_ids, "button_boxes_seen": seen_boxes,
            "covers_all": all(p["covers"] for p in pairs), "targets": list(targets),
            "passed": not lines.failures, "failures": lines.failures}


def accuracy_check(det_payload: object, truth_payload: object, lay: layouts.Layout, world: object = None,
                   robot_xy: tuple[float, float] | None = None, targets: Sequence[str] = (),
                   range_m: float = ACCURACY_RANGE_M, status: Mapping | None = None,
                   first_status: Mapping | None = None) -> tuple[int, dict]:
    """accuracy の判定（終了コード, JSON 1 行の中身）。データなし（2）：/go2/objects がない、その "source" が
    detector でない、正解がない、正解の "layout" が lay.name でない、odometry がない。ほかは accuracy() を
    （ボタンの箱は world から、filled は status から）流し、合格 0・不合格 1。ids_retired は確かめの間の増え。"""
    error = None
    if det_payload is None:
        error = f"no {OBJECTS_TOPIC}"
    elif parse_source(det_payload) != "detector":
        error = (f"{OBJECTS_TOPIC} has source {_shown(_raw_source(det_payload))}, not \"detector\" "
                 "(start the stack with OBJECTS_SOURCE=detector and --profile detector)")
    elif truth_payload is None:
        error = f"no {TRUTH_TOPIC}"
    elif layout_mismatch(truth_payload, lay.name) is not None:
        error = layout_mismatch(truth_payload, lay.name)
    elif robot_xy is None:
        error = f"no {ODOM_TOPIC}"
    if error is not None:
        return 2, {"error": error}
    status = status if isinstance(status, Mapping) else {}
    filled = status.get("filled")
    filled = frozenset(f for f in filled if isinstance(f, str)) if isinstance(filled, list) else frozenset()
    result = accuracy(parse(det_payload), parse(truth_payload), lay, _button_boxes(world), robot_xy, targets,
                      filled=filled, range_m=range_m)
    result.update({"detector": status.get("detector"), "still_inferences": status.get("still_inferences"),
                   "ids_retired": _retired(first_status, status), "notes": status.get("notes", [])})
    return (0 if result["passed"] else 1), result


def resolve(det_rows: Sequence[Obj], truth_rows: Sequence[Obj], target: str) -> str | None:
    """正解の id target と組になった検出の行の id（M10。pair_rows と同じ組み方）。組がなければ None。"""
    return next((d.id for d, t in pair_rows(det_rows, truth_rows) if t.id == target), None)


class ListedWait:
    """resolve's wait (M10, spec §13.5), without ROS. message() takes each /go2/objects message in order, the
    latest truth, the feed's still_wall (a stop this tool saw, _Feed) and the wall time; it is True once the wait
    is over: a truth for another layout (error), or the target's row has measured: true (id, measured).
    after (listed_after_still_s) = that wall time minus still_wall, only when the Go2 stood still since a stop
    this tool saw and an earlier message had no measured row for the target. Else after is None and note says
    why: a resolve started after the stop, or after the row was listed, must not time the latency from its own
    start (pre-final review F11). So start resolve before the motion that should bring the target into view."""

    def __init__(self, target: str, layout_name_: str):
        self.target = target
        self.layout = layout_name_
        self.id: str | None = None
        self.measured: bool | None = None
        self.after: float | None = None
        self.error: str | None = None
        self.note: str | None = None
        self._unlisted = False              # an earlier message had no measured row for the target

    def message(self, objects_payload: object, truth_payload: object, still_wall: float | None,
                now: float) -> bool:
        self.error = layout_mismatch(truth_payload, self.layout)
        if self.error is not None:
            return True
        rows = parse(objects_payload)
        obj_id = resolve(rows, parse(truth_payload), self.target)
        row = next((o for o in rows if o.id == obj_id), None)
        self.id, self.measured = obj_id, (row.measured if row is not None else None)
        if row is None or row.measured is not True:
            self._unlisted = True
            return False
        if not self._unlisted:
            self.note = "the row was measured in the first /go2/objects message this tool got"
        elif still_wall is None:
            self.note = ("the Go2 was not standing still after a stop this tool saw "
                         "(it started with the Go2 already still, or the Go2 is moving)")
        else:
            self.after = round(now - still_wall, 2)
        return True


def layout_mismatch(truth_payload: object, name: str) -> str | None:
    """/gazebo/objects_truth の "layout" が name でなければ、わけの 1 行（終了コード 2）。同じなら None。"""
    got = truth_payload.get("layout") if isinstance(truth_payload, dict) else None
    if got == name:
        return None
    if not isinstance(got, str):
        return f"{TRUTH_TOPIC} names no layout; this check uses the layout {name!r}"
    return (f"{TRUTH_TOPIC} is for the layout {got[:64]!r}, not {name!r}: "
            f"pass --layout {got[:64]} (with docker compose run, -e GO2_LAYOUT={got[:64]} also works: "
            f"the app service passes no GO2_LAYOUT)")


def layout_name(arg: str | None, environ: Mapping[str, str] = os.environ) -> str:
    """--layout、なければ GO2_LAYOUT、なければ household。"""
    return arg or environ.get("GO2_LAYOUT") or DEFAULT_LAYOUT_NAME


def pick_target(lay: layouts.Layout, target_id: str) -> tuple[Obj, tuple[Obj, ...]]:
    """配置の正解の足跡で、目標とほかの物。配置にない id なら ValueError（配置の id を並べる）。"""
    entry = lay.find(target_id)
    if entry is None:
        ids = ", ".join(e.id for e in lay.entries) or "none"
        raise ValueError(f"{target_id!r} is not in the layout {lay.name!r} (the ids are {ids})")
    return entry.obj(), tuple(e.obj() for e in lay.entries if e.id != target_id)


def rtf_stats(clock: list[tuple[float, float]], samples: list[Sample]) -> dict:
    """回の RTF（spec §13.5）：実際の時間の RTF_WINDOW_S 秒の窓ごとの real_time_factor の中央値と、
    歩いている窓（窓のシミュレーションの時間の odometry が、窓の最初の記録から SETTLED_POS_M か
    SETTLED_YAW_DEG を超えて動いた）と止まっている窓の中央値。満たない最後の窓と、記録のない窓の分けは数えない。"""
    out = {"window_s": RTF_WINDOW_S, "windows": 0, "median": None, "walking": None, "still": None,
           "walking_windows": 0, "still_windows": 0}
    if not clock:
        return out
    walls = [w for w, _ in clock]
    all_rtf, walking, still = [], [], []
    start = walls[0]
    while start + RTF_WINDOW_S <= walls[-1] + _EPS:
        end = start + RTF_WINDOW_S
        rtf = real_time_factor(clock, start, end)
        if rtf is not None:
            all_rtf.append(rtf)
            moving = _moving(samples, _sim_at(clock, start), _sim_at(clock, end))
            if moving is True:
                walking.append(rtf)
            elif moving is False:
                still.append(rtf)
        start = end

    def median(values: list[float]) -> float | None:
        return round(statistics.median(values), 3) if values else None

    out.update({"windows": len(all_rtf), "median": median(all_rtf), "walking": median(walking),
                "still": median(still), "walking_windows": len(walking), "still_windows": len(still)})
    return out


def odometry_sample(msg) -> Sample:
    """nav_msgs/Odometry 1 通 → Sample。t はシミュレーションの時刻（header.stamp）。"""
    stamp, p, q = msg.header.stamp, msg.pose.pose.position, msg.pose.pose.orientation
    roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return (stamp.sec + stamp.nanosec * 1e-9, p.x, p.y, yaw, p.z, max(abs(roll), abs(pitch)))


def real_time_factor(ticks: list[tuple[float, float]], start: float, end: float) -> float | None:
    """実際の時間の窓 [start, end] の間に /clock が進んだ秒数 ÷ (end − start)。ticks は (実際の時刻
    monotonic, シミュレーションの時刻)。始まりは start までの最後の 1 通（無ければ最初の 1 通）、終わりは
    end までの最後の 1 通。窓の中で /clock が止まった時間も、進まなかった時間として数える。"""
    upto = [sim for wall, sim in ticks if wall <= end]
    if not upto or not end > start:
        return None
    before = [sim for wall, sim in ticks if wall <= start]
    return (upto[-1] - (before[-1] if before else ticks[0][1])) / (end - start)


def valid_message(payload: object) -> bool:
    """/go2/objects の 1 通で、物が 1 つ以上あり、どの行も objects.parse を通る（落ちる行がない）。"""
    rows = payload.get("objects") if isinstance(payload, dict) else None
    return isinstance(rows, list) and 0 < len(parse(payload)) == len(rows)


# ---- 内部 ----

class _Lines:
    """合格の線の確かめ。失敗は「<名前>: <値> <比べ> <線>」の 1 行ずつ。"""

    def __init__(self):
        self.failures: list[str] = []

    def at_least(self, name: str, value: float | None, limit: float, what: str = "") -> None:
        if value is None:
            self.failures.append(f"{name}: not measured")
        elif not value >= limit - _EPS:
            self.failures.append(f"{name}: {_what(what)}{value:.3f} < {limit:g}")

    def at_most(self, name: str, value: float | None, limit: float, what: str = "") -> None:
        if value is None:
            self.failures.append(f"{name}: not measured")
        elif not value <= limit + _EPS:
            self.failures.append(f"{name}: {_what(what)}{value:.3f} > {limit:g}")

    def band(self, name: str, value: float | None, band: tuple[float, float]) -> None:
        lo, hi = band
        if value is None:
            self.failures.append(f"{name}: not measured")
        elif not lo <= value < hi:
            self.failures.append(f"{name}: {value:.3f} not in [{lo:g}, {hi:g})")

    def every(self, samples: list[Sample], exit_code: int | None, first: str | None,
              required: str | None, log_given: bool) -> None:
        """どの回にも：傾き、最初の動作（log があり、required のときだけ）、終了コード（None でなければ）。"""
        limits = THRESHOLDS["all"]
        max_tilt = _max_tilt(samples)
        if max_tilt is None:
            self.failures.append("max_tilt: not measured")
        elif not max_tilt < limits["max_tilt"]:
            self.failures.append(f"max_tilt: {max_tilt:.3f} >= {limits['max_tilt']:g}")
        if log_given and required is not None and first != required:
            self.failures.append(f"first_action: {first!r} != {required!r}")
        if exit_code is not None and exit_code != limits["exit_code"]:
            self.failures.append(f"exit_code: {exit_code} != {limits['exit_code']}")


def _section2(samples: list[Sample], expect: str, exit_code: int | None, log: list[str] | None,
              layout) -> dict:
    """物の設計書 §2 の値と合否（box_1・chair_1 で固定。今までのまま）。"""
    box, chair = _find(layout, "box_1"), _find(layout, "chair_1")
    path = _standing_path(samples)
    last = samples[-1] if samples else None

    winding = winding_number(path, (box.x, box.y)) if box is not None and path else None
    edge_box = _min_edge(path, box)
    edge_chair = _min_edge(path, chair)
    if chair is not None and last is not None:
        _, x, y, yaw, _, _ = last
        dist = math.hypot(chair.x - x, chair.y - y)
        angle = abs(math.degrees(wrap_rad(math.atan2(chair.y - y, chair.x - x) - yaw)))
    else:
        dist = angle = None
    first = _first_action(log) if log is not None else None

    lap, near = THRESHOLDS["lap"], THRESHOLDS["near"]
    lines = _Lines()
    if expect == "lap":
        lines.band("winding", winding, lap["winding"])
        lines.at_least("min_edge_box_1", edge_box, lap["min_edge_box_1"])
    else:
        lines.at_most("final_dist_chair_1", dist, near["final_dist_chair_1"])
        lines.at_most("final_angle_chair_1_deg", angle, near["final_angle_chair_1_deg"])
        lines.at_least("min_edge_chair_1", edge_chair, near["min_edge_chair_1"])
    lines.every(samples, exit_code, first, THRESHOLDS["all"]["first_action"], log is not None)

    return {"expect": expect, "exit_code": exit_code, "first_action": first,
            "winding_box_1": _r(winding), "min_edge_box_1": _r(edge_box),
            "min_edge_chair_1": _r(edge_chair), "final_dist_chair_1": _r(dist),
            "final_angle_chair_1_deg": _r(angle, 1), "max_tilt": _r(_max_tilt(samples)),
            "passed": not lines.failures, "failures": lines.failures}


def _target(samples: list[Sample], expect: str, exit_code: int | None, log: list[str] | None,
            target: Obj, others: Sequence[Obj], source: str | None) -> dict:
    """spec §10.8 の線（target の足跡から）と、chair_1・box_1 の §2 の線。"""
    path = _standing_path(samples)
    last = samples[-1] if samples else None
    footprints = (target, *(o for o in others if o.id != target.id))

    winding = winding_number(path, (target.x, target.y)) if path else None
    min_edge = _min_edge(path, target)
    clearance, clearance_id = None, None
    for o in footprints:
        d = _min_edge(path, o)
        if d is not None and (clearance is None or d < clearance):
            clearance, clearance_id = d, o.id
    if last is not None:
        _, x, y, yaw, _, _ = last
        final_edge = edge_distance(x, y, target)
        final_angle = math.degrees(footprint_angle(x, y, yaw, target))
        centre_dist = math.hypot(target.x - x, target.y - y)
        centre_angle = abs(math.degrees(wrap_rad(math.atan2(target.y - y, target.x - x) - yaw)))
    else:
        final_edge = final_angle = centre_dist = centre_angle = None
    first = _first_action(log) if log is not None else None
    required = first_action_required(samples)

    near, lap = THRESHOLDS["target_near"], THRESHOLDS["target_lap"]
    lines = _Lines()
    extra: dict = {}
    if expect == "near":
        lines.at_most("final_edge", final_edge, near["max_edge"])
        lines.at_most("final_footprint_angle_deg", final_angle, near["max_angle_deg"])
        lines.at_least("min_clearance", clearance, near["min_clearance"])
        if target.id == "chair_1":                  # §2: the chair keeps its centre lines (D6)
            s2 = THRESHOLDS["near"]
            lines.at_most("final_dist_chair_1", centre_dist, s2["final_dist_chair_1"])
            lines.at_most("final_angle_chair_1_deg", centre_angle, s2["final_angle_chair_1_deg"])
            lines.at_least("min_edge_chair_1", min_edge, s2["min_edge_chair_1"])
            extra = {"final_dist_chair_1": _r(centre_dist), "final_angle_chair_1_deg": _r(centre_angle, 1),
                     "min_edge_chair_1": _r(min_edge)}
        elif target.id == "box_1" and source == "logical":      # O16, O28
            centre = THRESHOLDS["box_centre"]
            lines.at_most("final_dist_box_1", centre_dist, centre["final_dist_box_1"])
            lines.at_most("final_angle_box_1_deg", centre_angle, centre["final_angle_box_1_deg"])
            extra = {"final_dist_box_1": _r(centre_dist), "final_angle_box_1_deg": _r(centre_angle, 1)}
    else:
        lines.band("winding", winding, lap["winding"])
        lines.at_least("min_edge", min_edge, lap["min_edge"])
        if target.id == "box_1":       # §2's box lines are these lines; the values under their §2 names
            extra = {"winding_box_1": _r(winding), "min_edge_box_1": _r(min_edge)}
    lines.every(samples, exit_code, first, required, log is not None)

    notes = []
    if expect == "near" and final_edge is not None and final_edge < NOTE_FINAL_EDGE:
        notes.append(f"final_edge {final_edge:.3f} < {NOTE_FINAL_EDGE:g}: the target may set the front "
                     "obstacle bit (recorded only)")
    return {"expect": expect, "target": target.id, "source": source, "exit_code": exit_code,
            "first_action": first, "first_action_required": required,
            "winding": _r(winding), "min_edge": _r(min_edge), "final_edge": _r(final_edge),
            "final_footprint_angle_deg": _r(final_angle, 1), "min_clearance": _r(clearance),
            "min_clearance_id": clearance_id, **extra, "max_tilt": _r(_max_tilt(samples)),
            "passed": not lines.failures, "failures": lines.failures, "notes": notes}


def _find(layout, obj_id: str) -> Obj | None:
    return next((o for o in layout if o.id == obj_id), None)


def _from_standing(samples: list[Sample]) -> list[Sample]:
    """初めて立った（z ≥ STAND_Z）記録から後ろ。立たなければ []。"""
    k = next((i for i, s in enumerate(samples) if s[4] >= STAND_Z), len(samples))
    return samples[k:]


def _standing_path(samples: list[Sample]) -> list[tuple[float, float]]:
    return [(s[1], s[2]) for s in _from_standing(samples)]


def _min_edge(path: list[tuple[float, float]], obj: Obj | None) -> float | None:
    if obj is None or not path:
        return None
    return min(edge_distance(x, y, obj) for x, y in path)


def _max_tilt(samples: list[Sample]) -> float | None:
    return max(abs(s[5]) for s in samples) if samples else None


def _first_action(log: list[str]) -> str | None:
    """「開始：」がある最初の行の、その後ろ。"""
    for line in log:
        _, mark, rest = line.partition(START_MARK)
        if mark:
            return rest.strip()
    return None


def _extent(o: Obj) -> tuple[tuple[float, float], tuple[float, float]]:
    """足跡の (x の最小, y の最小), (x の最大, y の最大)。"""
    a, b = o.size[0] / 2, o.size[1] / 2
    return (o.x - a, o.y - b), (o.x + a, o.y + b)


def _in_scope(t: Obj, lay: layouts.Layout, robot_xy: tuple[float, float], range_m: float) -> bool:
    entry = lay.find(t.id)
    return (t.visible and entry is not None and entry.detectable
            and math.hypot(t.x - robot_xy[0], t.y - robot_xy[1]) <= range_m + _EPS)


def _button_boxes(world: object) -> list[tuple[float, float, float]]:
    """/sim/world の "boxes"（[x, y, 辺] の列）。形の違う項目は捨てる。"""
    boxes = world.get("boxes") if isinstance(world, dict) else None
    out = []
    for b in boxes if isinstance(boxes, list) else ():
        if (isinstance(b, list) and len(b) == 3
                and all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in b)
                and b[2] > 0):
            out.append((float(b[0]), float(b[1]), float(b[2])))
    return out


def _retired(first: Mapping | None, last: Mapping | None):
    """状態の ids_retired の、確かめの間の増え（数なら差、id の列なら増えた id）。分からなければ None。"""
    a = first.get("ids_retired") if isinstance(first, Mapping) else None
    b = last.get("ids_retired") if isinstance(last, Mapping) else None
    if isinstance(a, int) and isinstance(b, int) and not isinstance(a, bool) and not isinstance(b, bool):
        return b - a
    if isinstance(a, list) and isinstance(b, list):
        return sorted(str(v) for v in b if v not in a)
    return None


def _raw_source(payload: object) -> object:
    return payload.get("source") if isinstance(payload, dict) else None


def _shown(v: object) -> str:
    return json.dumps(v if not isinstance(v, str) else v[:64], ensure_ascii=False)


def _sim_at(clock: list[tuple[float, float]], wall: float) -> float:
    """実際の時刻 wall までの最後の /clock（無ければ最初の 1 通）。"""
    upto = [sim for w, sim in clock if w <= wall]
    return upto[-1] if upto else clock[0][1]


def _moving(samples: list[Sample], t0: float, t1: float) -> bool | None:
    """シミュレーションの時間 [t0, t1] の記録が、その最初の記録から動いたか。記録がなければ None。"""
    window = [s for s in samples if t0 - _EPS <= s[0] <= t1 + _EPS]
    if not window:
        return None
    _, x, y, yaw = window[0][:4]
    return any(math.hypot(s[1] - x, s[2] - y) > SETTLED_POS_M
               or abs(math.degrees(wrap_rad(s[3] - yaw))) > SETTLED_YAW_DEG for s in window)


def _what(what: str) -> str:
    return f"{what} " if what else ""


def _r(v: float | None, nd: int = 3) -> float | None:
    return None if v is None else round(v, nd)


def _say(text: str) -> None:
    print(f"[objects_check] {text}", file=sys.stderr, flush=True)


def _print(payload: dict) -> None:
    print(json.dumps(payload, ensure_ascii=False), flush=True)


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


def _watch(child, spin_until, timeout: float) -> None:
    """子（run_formula）が終わるまで記録し（spin_until(done, 秒)）、終わったら TAIL_S 秒だけ続ける。
    timeout + CHILD_MARGIN_S 秒で終わらなければ止める。Ctrl+C・SIGTERM は「中断」として飲み込む（JSON は出す）。
    どう終わっても子は finally で止める（もう終わっていれば何もしない）。ほかの例外はその後そのまま上げる。"""
    try:
        if not spin_until(lambda: child.poll() is not None, timeout + CHILD_MARGIN_S):
            _say(f"run_formula が {timeout + CHILD_MARGIN_S:g} 秒で終わらないので止めます")
            _stop_child(child)
        spin_until(lambda: False, TAIL_S)
    except KeyboardInterrupt:
        _say("中断しました。run_formula を止めます")
    finally:
        _stop_child(child)


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


class _Feed:
    """1 つのコマンドが受けたもの（コールバックは足すだけ）。JSON のトピックは読んだ dict（読めなければ None）。
    gate（gazebo.bridge_core.StillGate）があれば odometry の速さを渡し、止まり始めの実際の時刻（clock、既定は
    time.monotonic）を still_wall に。ただし、このコマンドが見た止まり（動いていた後の止まり）だけ：最初の
    odometry から止まっていれば、止まった時刻はわからないので、次に動くまで None（pre-final review F11）。"""

    def __init__(self, gate=None, clock=time.monotonic):
        self.samples: list[Sample] = []
        self.ticks: list[tuple[float, float]] = []      # (実際の時刻 monotonic, シミュレーションの時刻)
        self.objects: list[object] = []
        self.truth: list[object] = []
        self.status: list[object] = []
        self.world: list[object] = []
        self.gate = gate
        self.still_wall: float | None = None
        self._clock = clock
        self._moved = False                             # a not-still odometry seen since this command started

    def on_odom(self, msg) -> None:
        sample = odometry_sample(msg)
        self.samples.append(sample)
        if self.gate is not None:
            was = self.gate.still_since
            tw = msg.twist.twist
            self.gate.observe(sample[0], (tw.linear.x, tw.linear.y, tw.linear.z),
                              (tw.angular.x, tw.angular.y, tw.angular.z))
            if self.gate.still_since is None:
                self.still_wall = None
                self._moved = True
            elif was is None and self._moved:
                self.still_wall = self._clock()

    def still(self) -> bool:
        return bool(self.gate is not None and self.samples and self.gate.allows(self.samples[-1][0]))

    def robot_xy(self) -> tuple[float, float] | None:
        return (self.samples[-1][1], self.samples[-1][2]) if self.samples else None


def _last(items: list[object]) -> object:
    return items[-1] if items else None


def _json(text: str) -> object:
    try:
        return json.loads(text)
    except ValueError:
        return None


def _listen(node, feed: _Feed, odom: bool = False, clock: bool = False, objects: bool = False,
            truth: bool = False, status: bool = False, world: bool = False) -> _Feed:
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String

    if odom:
        from nav_msgs.msg import Odometry
        node.create_subscription(Odometry, ODOM_TOPIC, feed.on_odom, qos_profile_sensor_data)
    if clock:
        from rosgraph_msgs.msg import Clock
        node.create_subscription(
            Clock, CLOCK_TOPIC,
            lambda m: feed.ticks.append((time.monotonic(), m.clock.sec + m.clock.nanosec * 1e-9)),
            qos_profile_sensor_data)
    for wanted, topic, into in ((objects, OBJECTS_TOPIC, feed.objects), (truth, TRUTH_TOPIC, feed.truth),
                                (status, STATUS_TOPIC, feed.status), (world, WORLD_TOPIC, feed.world)):
        if wanted:
            node.create_subscription(String, topic, lambda m, into=into: into.append(_json(m.data)),
                                     qos_profile_sensor_data)
    return feed


def _load_layout(name: str | None) -> layouts.Layout | None:
    """配置を読む。読めなければわけを JSON 1 行で出して None（終了コード 2）。"""
    try:
        return layouts.load(layout_name(name))
    except layouts.LayoutError as e:
        _print({"error": str(e)})
        return None


def _layout_and_target(args: argparse.Namespace):
    """(配置, 目標, ほかの物)。目標がなければ (配置, None, ())。だめなら None（わけは出した）。"""
    lay = _load_layout(args.layout)
    if lay is None:
        return None
    if getattr(args, "target", None) is None:
        return lay, None, ()
    try:
        target, others = pick_target(lay, args.target)
    except ValueError as e:
        _print({"error": str(e)})
        return None
    return lay, target, others


def _truth_layout_error(rclpy, node, feed: _Feed, name: str) -> str | None:
    """正解の最初の 1 通を MSG_WAIT_S まで待ち、"layout" が違えばわけ。来なければ記録して None
    （run・watch の線は配置ファイルの足跡で測る）。"""
    if not _spin_until(rclpy, node, lambda: bool(feed.truth), MSG_WAIT_S):
        _say(f"no {TRUTH_TOPIC} in {MSG_WAIT_S:g} s: the layout {name!r} is not checked against Gazebo")
        return None
    return layout_mismatch(feed.truth[-1], name)


def _run(args: argparse.Namespace) -> int:
    chosen = _layout_and_target(args)
    if chosen is None:
        return 2
    lay, target, others = chosen
    rclpy, node = _start_ros("go2_objects_check")
    feed = _listen(node, _Feed(), odom=True, clock=True, objects=True, truth=True)
    lines: list[str] = []
    child = reader = None
    try:
        error = _truth_layout_error(rclpy, node, feed, lay.name)
        if error is None:
            # 別のセッションにするのは、Ctrl+C が子に二重に届かないようにするため（子へはこちらから 1 回だけ送る）
            child = subprocess.Popen([sys.executable, "-m", "voice_ltl.run_formula", args.formula,
                                      "--timeout", f"{args.timeout:g}"],
                                     cwd=ROOT, stdout=subprocess.PIPE, text=True, bufsize=1,
                                     start_new_session=True)
            reader = threading.Thread(target=_pump, args=(child.stdout, lines), daemon=True)
            reader.start()
            _watch(child, lambda done, timeout: _spin_until(rclpy, node, done, timeout), args.timeout)
    except KeyboardInterrupt:          # only before the child started: _watch takes it afterwards
        error = "interrupted before run_formula started"
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    if child is None:
        _print({"formula": args.formula, "error": error})
        return 2
    reader.join(5.0)
    samples = list(feed.samples)
    result = summarize(samples, args.expect, child.wait(), list(lines), layout=lay.objs(), target=target,
                       others=others, source=last_source(feed.objects), rtf=rtf_stats(list(feed.ticks), samples))
    _print({"formula": args.formula, "layout": lay.name, **result, "samples": len(samples)})
    return 0 if result["passed"] else 1


def _watch_cmd(args: argparse.Namespace) -> int:
    chosen = _layout_and_target(args)
    if chosen is None:
        return 2
    lay, target, others = chosen
    rclpy, node = _start_ros("go2_objects_check_watch")
    feed = _listen(node, _Feed(), odom=True, clock=True, objects=True, truth=True)
    error = None
    try:
        error = _truth_layout_error(rclpy, node, feed, lay.name)
        end = time.monotonic() + args.seconds
        while error is None and time.monotonic() < end:
            _spin_until(rclpy, node, lambda: False, min(WATCH_CHECK_S, max(0.0, end - time.monotonic())))
            if settled(feed.samples):
                break
    except KeyboardInterrupt:
        _say("中断しました")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    if error is not None:
        _print({"child": None, "error": error})
        return 2
    samples = list(feed.samples)
    if not samples:
        _print({"child": None, "error": f"no {ODOM_TOPIC}"})
        return 2
    result = summarize(samples, args.expect, None, None, layout=lay.objs(), target=target, others=others,
                       source=last_source(feed.objects), rtf=rtf_stats(list(feed.ticks), samples))
    _print({"child": None, "layout": lay.name, **result, "samples": len(samples)})
    return 0 if result["passed"] else 1


def _accuracy(args: argparse.Namespace) -> int:
    from gazebo.bridge_core import StillGate

    lay = _load_layout(args.layout)
    if lay is None:
        return 2
    missing = [t for t in args.target if lay.find(t) is None]
    if missing:
        _print({"error": f"{', '.join(missing)} not in the layout {lay.name!r} "
                         f"(the ids are {', '.join(e.id for e in lay.entries) or 'none'})"})
        return 2
    rclpy, node = _start_ros("go2_objects_check_accuracy")
    feed = _listen(node, _Feed(StillGate()), odom=True, objects=True, truth=True, status=True, world=True)
    start = time.monotonic()

    def ready() -> bool:
        status = _last(feed.status)
        count = status.get("still_inferences") if isinstance(status, dict) else None
        return (feed.still() and isinstance(count, int) and not isinstance(count, bool)
                and count >= STILL_INFERENCES)

    try:
        is_ready = _spin_until(rclpy, node, ready, args.seconds)
    except KeyboardInterrupt:
        _say("中断しました")
        is_ready = False
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    code, out = accuracy_check(_last(feed.objects), _last(feed.truth), lay, _last(feed.world), feed.robot_xy(),
                               args.target, args.range, _last(feed.status),
                               feed.status[0] if feed.status else None)
    _print({**out, "layout": lay.name, "ready": is_ready, "waited_s": round(time.monotonic() - start, 1)})
    return code


def _resolve(args: argparse.Namespace) -> int:
    from gazebo.bridge_core import StillGate

    lay = _load_layout(args.layout)
    if lay is None:
        return 2
    if lay.find(args.target) is None:
        _print({"error": f"{args.target!r} is not in the layout {lay.name!r}"})
        return 2
    rclpy, node = _start_ros("go2_objects_check_resolve")
    feed = _listen(node, _Feed(StillGate()), odom=True, objects=True, truth=True)
    wait = ListedWait(args.target, lay.name)
    seen = [0]

    def done() -> bool:
        if not feed.truth:
            return False
        while seen[0] < len(feed.objects):
            seen[0] += 1
            if wait.message(feed.objects[seen[0] - 1], feed.truth[-1], feed.still_wall, time.monotonic()):
                return True
        return False

    try:
        found = _spin_until(rclpy, node, done, args.timeout)
    except KeyboardInterrupt:
        _say("中断しました")
        found = False
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    if wait.error is not None or not feed.objects or not feed.truth:
        missing = OBJECTS_TOPIC if not feed.objects else TRUTH_TOPIC
        _print({"target": args.target, "error": wait.error or f"no {missing}"})
        return 2
    if found and wait.note is not None:
        _say(f"listed_after_still_s を出せません：{wait.note}")
    _print({"target": args.target, "id": wait.id, "measured": wait.measured, "listed_after_still_s": wait.after})
    return 0 if found else 1


def _rtf(args: argparse.Namespace) -> int:
    rclpy, node = _start_ros("go2_objects_check_rtf")
    feed = _listen(node, _Feed(), clock=True)
    ticks = feed.ticks
    try:
        if not _spin_until(rclpy, node, lambda: bool(ticks), MSG_WAIT_S):
            print(json.dumps({"rtf": None, "error": f"no {CLOCK_TOPIC} in {MSG_WAIT_S:g} s"}), flush=True)
            return 2
        start = time.monotonic()   # 窓は実際の時間で測る（/clock が止まっても窓は進む）
        _spin_until(rclpy, node, lambda: time.monotonic() >= start + args.seconds, args.seconds + 5.0)
        end = time.monotonic()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    rtf = real_time_factor(ticks, start, end)
    print(json.dumps({"rtf": None if rtf is None else round(rtf, 2)}), flush=True)
    return 0 if rtf is not None else 2


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
                                 description="Gazebo の Go2 で、物の周りを一周・物まで歩くの合格の条件と、検出の正しさを測る")
    sub = ap.add_subparsers(dest="command", required=True)
    layout_help = "配置ファイルの名前（gazebo/layouts/<名前>.json。既定は GO2_LAYOUT、なければ household）"
    run = sub.add_parser("run", help="run_formula を走らせて odometry を記録し、合否を JSON 1 行で出す")
    run.add_argument("--expect", choices=("lap", "near"), required=True,
                     help="lap：周りを一周、near：手前まで（--target がなければ box_1・chair_1 の §2 の線）")
    run.add_argument("--target", help="測る正解の id（例 sofa_1）。式の id は検出器の id でよい")
    run.add_argument("--layout", help=layout_help)
    run.add_argument("--timeout", type=float, default=300.0,
                     help="run_formula に渡す --timeout [s]（既定 300）")
    run.add_argument("formula", help='LTLf の式（例："F around_box_1"）')
    watch = sub.add_parser("watch", help="子を始めずに記録し（app が動かす）、合否を JSON 1 行で出す")
    watch.add_argument("--expect", choices=("lap", "near"), required=True, help="lap：周りを一周、near：手前まで")
    watch.add_argument("--target", required=True, help="測る正解の id（例 sofa_1）")
    watch.add_argument("--layout", help=layout_help)
    watch.add_argument("--seconds", type=float, default=300.0,
                       help="記録する実際の秒数の上限（既定 300。動いた後に 10 秒止まれば早く終わる）")
    acc = sub.add_parser("accuracy", help="検出器の /go2/objects を正解と比べ、JSON 1 行で出す")
    acc.add_argument("--seconds", type=float, default=15.0, help="止まって推論が 3 回になるのを待つ上限（既定 15）")
    acc.add_argument("--range", type=float, default=ACCURACY_RANGE_M,
                     help=f"recall に数える正解の Go2 からの距離 [m]（既定 {ACCURACY_RANGE_M:g}）")
    acc.add_argument("--target", nargs="+", action="extend", default=[], help="位置の線を厳しくする正解の id")
    acc.add_argument("--layout", help=layout_help)
    res = sub.add_parser("resolve", help="正解の id と組になる /go2/objects の id を出す")
    res.add_argument("--target", required=True, help="正解の id（例 sofa_1）")
    res.add_argument("--timeout", type=float, default=30.0, help="待つ実際の秒数（既定 30）")
    res.add_argument("--layout", help=layout_help)
    rtf = sub.add_parser("rtf", help="/clock の秒数 ÷ 実際の秒数")
    rtf.add_argument("--seconds", type=float, default=20.0, help="測る実際の秒数（既定 20）")
    sub.add_parser("objects", help="/go2/objects を 1 通出す")
    args = ap.parse_args(argv)
    if args.command == "run" and not args.timeout > 0:
        ap.error("--timeout は 0 より大きい秒数にしてください")
    if args.command in ("rtf", "watch", "accuracy") and not args.seconds > 0:
        ap.error("--seconds は 0 より大きい秒数にしてください")
    if args.command == "resolve" and not args.timeout > 0:
        ap.error("--timeout は 0 より大きい秒数にしてください")
    if args.command == "accuracy" and not args.range > 0:
        ap.error("--range は 0 より大きい距離にしてください")
    return args


def main(argv: list[str] | None = None) -> int:
    args = _parse(argv)

    def on_signal(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_signal)
    return {"run": _run, "watch": _watch_cmd, "accuracy": _accuracy, "resolve": _resolve,
            "rtf": _rtf, "objects": _objects}[args.command](args)


if __name__ == "__main__":
    sys.exit(main())
