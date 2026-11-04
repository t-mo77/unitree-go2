"""式から直接 Go2 を動かす道具（設計書 §9.1「基本動作の確認」）。LLM も音声も使わない。

    python3 -m voice_ltl.run_formula "<LTLf の式>" [--timeout 120]

- 翻訳だけを、その式を 1 つ返す FakeLLM に置き換える。あとは Web のページと同じ道を通る：
  check()（文法と単語帳）→ 合成（監督器）→ 監督器と zip の実行器 → RosLink（実機と同じトピック）とセンサー。
- 標準出力に、計画・各動作の開始と完了（Session の、人が読むログ）と、最後の status を日本語で出す。
- 終了コード：status が done なら 0、それ以外（error、時間切れ、Go2 が見つからない、式が却下）は 1。
- status が done か error になったら終わる。--timeout 秒たっても終わらなければ、止めて終わる。
- 終わるときは RosLink.close(3)（StopMove を 3 回）。Ctrl+C・SIGTERM でも同じ。

コンテナで（Go2・にせ Go2・Gazebo のどれかが動いているとき。Web のページ（app）とは同時に使わない。
/api/sport/request の送り手が 2 つになるため、起動を断る）:
    docker compose run --rm app python3 -m voice_ltl.run_formula "F(moved_forward & X F(turned_left & X F lying))"
"""

import argparse
import sys
import time
from pathlib import Path
from typing import Optional

from voice_ltl.grammar import check
from voice_ltl.llm import FakeLLM
from voice_ltl.world import PROPOSITIONS

WORKDIR = Path("/work/out")   # 合成の作業場所（server.py と同じ）
PERIOD = 0.1                  # 制御の周期（10 Hz）
FINAL = ("done", "error")     # これになったら終わる
INSTRUCTION = "(run_formula)"  # Session に渡す指示の文（ログに出るだけ。式は FakeLLM が返す）


def _new_lines(seen: list[str], log: list[str]) -> list[str]:
    """log のうち、まだ出していない行。Session のログは最後の LOG_LINES 行だけを持つので、
    いっぱいになると古い行が前から消える。seen の後ろと log の前が重なる所から先を返す。"""
    for k in range(min(len(seen), len(log)), -1, -1):
        if seen[len(seen) - k:] == log[:k]:
            return log[k:]
    return log


def _parse(argv: Optional[list[str]]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="python3 -m voice_ltl.run_formula",
        description="LTLf の式から、LLM を使わずに Go2 を動かす（終了コード：done なら 0、それ以外は 1）")
    parser.add_argument("formula", help='LTLf の式（例："F(moved_forward & X F lying)"）')
    parser.add_argument("--timeout", type=float, default=120.0,
                        help="これだけ（秒）たっても終わらなければ止めて終わる（既定 120）")
    args = parser.parse_args(argv)
    if not args.timeout > 0:
        parser.error("--timeout は 0 より大きい秒数にしてください")
    return args


def run(formula: str, timeout: float) -> int:
    # 式は Session の中でも check() を通る。ここでも先に通すのは、却下の理由をそのまま出して、
    # ROS 2 を起動せずに終わるため（FakeLLM は 1 回しか答えないので、聞き直すと理由が消える）
    errors = check(formula, PROPOSITIONS)
    if errors:
        print(f"式が文法か単語帳に合いません：{formula}", flush=True)
        for e in errors:
            print(f"  {e}", flush=True)
        return 1

    from go2_nl_control.perception import PerceptionConfig

    from voice_ltl.ros_link import RosLink
    from voice_ltl.session import Session

    try:
        link = RosLink(PerceptionConfig(), node_name="go2_run_formula")
    except RuntimeError as exc:   # 二重起動
        print(exc, flush=True)
        return 1
    stop_times = 3
    try:
        try:
            link.wait_ready()
        except RuntimeError as exc:   # Go2 が見つからない・別の送り手がいる
            print(exc, flush=True)
            if link.other_senders():  # そちらが Go2 を動かしているので、StopMove で割り込まない
                stop_times = 0
            return 1
        llm = FakeLLM([{"formula": formula, "unsupported": []}])
        session = Session(link, llm, None, WORKDIR, background=False)
        session.submit_text(INSTRUCTION)
        return _follow(link, session, timeout)
    except KeyboardInterrupt:   # Ctrl+C・SIGTERM・SIGHUP（RosLink が KeyboardInterrupt にする）
        print("結果：中断しました", flush=True)
        return 1
    finally:
        link.close(stop_times)


def _follow(link, session, timeout: float) -> int:
    """10 Hz で受信と Session.tick を回し、ログを出す。status が done / error になるか、時間切れで終わる。"""
    seen: list[str] = []

    def show() -> dict:
        nonlocal seen
        snap = session.snapshot()
        for line in _new_lines(seen, snap["log"]):
            print(line, flush=True)
        seen = snap["log"]
        return snap

    snap = show()
    end = time.monotonic() + timeout
    next_t = time.monotonic()
    while snap["status"] not in FINAL:
        if time.monotonic() >= end:
            session.stop()   # StopMove を送り、計画を捨てる（終わりに close(3) も送る）
            show()
            print(f"結果：時間切れ（{timeout:g} 秒たっても終わりませんでした。status {snap['status']}）",
                  flush=True)
            return 1
        link.spin_once()
        session.tick(time.time())
        snap = show()
        next_t += PERIOD
        delay = next_t - time.monotonic()
        if delay < 0:   # 遅れたら、今から数え直す
            next_t, delay = time.monotonic(), 0.0
        time.sleep(delay)
    status = snap["status"]
    print(f"結果：{status}（{snap['message']}）", flush=True)
    return 0 if status == "done" else 1


def main(argv: Optional[list[str]] = None) -> int:
    try:
        args = _parse(argv)
    except SystemExit as exc:   # --help は 0、使い方の誤りは 1（argparse の 2 にしない）
        return 0 if exc.code in (0, None) else 1
    return run(args.formula, args.timeout)


if __name__ == "__main__":
    sys.exit(main())
