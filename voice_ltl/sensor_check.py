"""実機の LiDAR を確かめる道具（docs/real_robot.md）。

RosLink で /utlidar/cloud（と /sportmodestate）を受け、アプリと同じ判定（Sensing）の
前方・後方の「あり / なし / 途切れ」と一番近い距離を、0.5 秒ごとに 1 行ずつ出す。Ctrl+C で終わる。

Go2 には何も送らない。RosLink は /api/sport/request の送り手を作るが、このツールは一度も送らず、
終わるときも close(0) で StopMove を送らない。送り手が 1 つ見えるので、動かしている間は
サーバー（voice_ltl.server）を起動しない（起動を断られる）。

使い方（Go2 と有線でつないだ Linux PC で。サーバーを起動する前に）:
    docker compose -f compose.robot.yaml run --rm --no-deps app python3 -m voice_ltl.sensor_check
取り付けの値を試すとき（合ったら nl_control/go2_nl_control/perception.py の既定値に書く）:
    ... python3 -m voice_ltl.sensor_check --lidar-x 0.29 --lidar-z -0.05 --lidar-pitch 2.88
"""

import argparse
import functools
import time
from dataclasses import replace
from typing import Callable, Iterable, Optional

from go2_nl_control.perception import PerceptionConfig
from go2_nl_control.supervisor import BLOCKED, CLEAR, SENSOR_LOST

from voice_ltl.sensing import Reading

PERIOD_S = 0.5   # 1 行を出す間隔 [s]
SPIN_S = 0.05    # 受信を処理する間隔 [s]

# 画面（static/app.js の SENSE_JA）と同じ言葉
SENSE_JA = {CLEAR: "なし", BLOCKED: "あり", SENSOR_LOST: "途切れ"}

_print = functools.partial(print, flush=True)


def _meters(d: Optional[float]) -> str:
    return "—" if d is None else f"{d:.2f} m"


def format_line(r: Reading) -> str:
    """例: 「前方 あり 0.41 m ｜ 後方 なし —」。距離は胴の中心から（物が無ければ —）。"""
    front, back = SENSE_JA.get(r.front, r.front), SENSE_JA.get(r.back, r.back)
    return f"前方 {front} {_meters(r.front_dist)} ｜ 後方 {back} {_meters(r.back_dist)}"


def header(cfg: PerceptionConfig) -> list[str]:
    return [
        f"LiDAR の判定を {PERIOD_S} 秒ごとに 1 行出します。Go2 には何も送りません。Ctrl+C で終わります。",
        f"取り付け: lidar_x {cfg.lidar_x:.3f} m・lidar_z {cfg.lidar_z:.3f} m・"
        f"lidar_pitch {cfg.lidar_pitch:.3f} rad（--lidar-x・--lidar-z・--lidar-pitch で試せます）",
        f"距離は胴の中心から。{cfg.stop_distance:.2f} m より近いと「あり」、"
        f"{cfg.resume_distance:.2f} m より遠いと「なし」に戻ります（その間は前の判定のまま）。",
        "Go2 はリモコンで立たせてから見てください。"
        "「途切れ」は /utlidar/cloud か /sportmodestate が 0.5 秒届いていないことです。",
    ]


def config_from_args(argv: Optional[Iterable[str]] = None) -> PerceptionConfig:
    """既定は PerceptionConfig()（サーバーと同じ）。取り付けの 3 つだけ変えて試せる。"""
    base = PerceptionConfig()
    p = argparse.ArgumentParser(prog="python3 -m voice_ltl.sensor_check",
                                description="実機の LiDAR の判定を確かめる（Go2 には何も送らない）")
    p.add_argument("--lidar-x", type=float, default=base.lidar_x, help="前後のずれ [m]")
    p.add_argument("--lidar-z", type=float, default=base.lidar_z, help="高さのずれ [m]")
    p.add_argument("--lidar-pitch", type=float, default=base.lidar_pitch,
                   help="y 軸まわりの下向きの傾き [rad]")
    a = p.parse_args(None if argv is None else list(argv))
    return replace(base, lidar_x=a.lidar_x, lidar_z=a.lidar_z, lidar_pitch=a.lidar_pitch)


def run(link, out: Callable[[str], None] = _print, clock: Callable[[], float] = time.time,
        sleep: Callable[[float], None] = time.sleep) -> None:
    """Ctrl+C まで、受信を処理しながら PERIOD_S ごとに 1 行出す。終わるときは何も送らずに閉じる。

    link は RosLink と同じ spin_once / reading / close を持つもの。
    """
    try:
        due = clock()
        while True:
            link.spin_once()
            now = clock()
            if now >= due:
                out(format_line(link.reading(now)))
                due += PERIOD_S
                if due <= now:          # 遅れたら追いつこうとせず、次は今から PERIOD_S 後
                    due = now + PERIOD_S
            sleep(SPIN_S)
    except KeyboardInterrupt:   # Ctrl+C（RosLink は SIGTERM・SIGHUP もこれにする）
        pass
    finally:
        link.close(0)           # StopMove も送らない（このツールは Go2 を動かしていない）


def main(argv: Optional[Iterable[str]] = None) -> int:
    cfg = config_from_args(argv)
    from voice_ltl.ros_link import RosLink   # rclpy は道具を動かすときだけ読み込む
    try:
        link = RosLink(cfg, node_name="go2_sensor_check")
    except RuntimeError as exc:   # 同じコンテナでサーバーなどが動いている
        _print(exc)
        return 1
    for line in header(cfg):
        _print(line)
    run(link)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
