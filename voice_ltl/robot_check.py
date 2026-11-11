"""実機の Go2 と有線でつながったかを、まとめて確かめる道具（docs/real_robot.md）。

Go2 には何も送らない。/api/sport/request の送り手も作らず、受け手と送り手の数を数えるだけ。
サーバー（voice_ltl.server）を起動する前に流す（サーバーも送り手なので、動いていると「だめ」と出る）。

確かめること:
1. 設定：compose.robot.yaml で動いている（SIM=0）、env.txt の GO2_IFACE、ROS_DOMAIN_ID が 0（Go2 と同じ）、
   RMW が CycloneDDS、CYCLONEDDS_URI がその差し口を指している（URI の中身は出さない）。
2. 差し口：その名前の差し口があり、オンで、リンクがあり（ケーブルと Go2 の電源）、マルチキャストができ、
   192.168.123.0/24 の IPv4 がついている（.161 は Go2 本体なので使えない）。
3. --seconds 秒（既定 5）の間に /sportmodestate と /utlidar/cloud が届いた回数、最後の error_code と姿勢。
4. /api/sport/request の受け手（Go2 の運動の制御）がいて、ほかの送り手がいない。
1・2 にだめがあれば、ROS 2 は起動せずに終わる。終了コード：だめが無ければ 0、あれば 1。

使い方（Go2 と有線でつないだ Linux PC で）:
    docker compose -f compose.robot.yaml run --rm --no-deps app python3 -m voice_ltl.robot_check
"""

import argparse
import fcntl
import functools
import ipaddress
import os
import socket
import struct
import time
from dataclasses import dataclass
from typing import Callable, Iterable, Mapping, Optional

from go2_nl_control.actions import UNKNOWN, state_from_error_code

from voice_ltl.server import sim_mode
from voice_ltl.session import POSTURE_JA

# voice_ltl.ros_link と同じトピック（ros_link は rclpy を読み込むので、ここには書き写す。テストで一致を見る）
REQUEST_TOPIC = "/api/sport/request"
STATE_TOPIC = "/sportmodestate"
LIDAR_TOPIC = "/utlidar/cloud"

GO2_NET = ipaddress.ip_network("192.168.123.0/24")
GO2_ADDRESS = "192.168.123.161"     # Go2 本体（公式 Quick Start）
PC_EXAMPLE = "192.168.123.99/24"    # 公式 unitree_ros2 の例
DEFAULT_SECONDS = 5.0
HELP = "docs/real_robot.md の「8. 困ったとき」を見てください"

OK, WARN, BAD = "ok", "warn", "bad"
_LABEL = {OK: "［OK］", WARN: "［注意］", BAD: "［だめ］"}

_print = functools.partial(print, flush=True)


@dataclass(frozen=True)
class Check:
    level: str   # OK・WARN・BAD
    text: str    # 1 行。だめ・注意なら直し方も


@dataclass(frozen=True)
class IfaceInfo:
    up: bool                 # オン（IFF_UP）
    running: bool            # リンクがある（IFF_RUNNING：ケーブルの先が生きている）
    multicast: bool          # IFF_MULTICAST
    address: Optional[str]   # IPv4（無ければ None）
    netmask: Optional[str]


def format_check(c: Check) -> str:
    return f"{_LABEL[c.level]} {c.text}"


def exit_code(checks: Iterable[Check]) -> int:
    return 1 if any(c.level == BAD for c in checks) else 0


# ---------- 1. 設定 ----------
def check_settings(env: Mapping[str, str]) -> list[Check]:
    out: list[Check] = []
    if sim_mode(env):
        out.append(Check(BAD, "SIM=1 で動いています（compose.yaml）。実機は "
                              "`docker compose -f compose.robot.yaml run --rm --no-deps app python3 -m voice_ltl.robot_check` "
                              "で流します（network_mode: host で、PC の差し口が見えます）"))
    iface = (env.get("GO2_IFACE") or "").strip()
    if iface:
        out.append(Check(OK, f"GO2_IFACE：差し口 {iface}"))
    else:
        out.append(Check(BAD, "env.txt の GO2_IFACE が空です。Go2 につないだ差し口の名前"
                              "（PC で `ip -br addr` を見て、192.168.123.x がついたもの。例 enp3s0）を書きます"))
    domain = (env.get("ROS_DOMAIN_ID") or "").strip()
    if domain in ("", "0"):
        out.append(Check(OK, "ROS_DOMAIN_ID 0（Go2 と同じ）"))
    else:
        out.append(Check(BAD, f"ROS_DOMAIN_ID が {domain} です。Go2 は 0 なので、env.txt や -e から外します"))
    rmw = (env.get("RMW_IMPLEMENTATION") or "").strip()
    if rmw == "rmw_cyclonedds_cpp":
        out.append(Check(OK, "RMW_IMPLEMENTATION：rmw_cyclonedds_cpp"))
    else:
        out.append(Check(BAD, f"RMW_IMPLEMENTATION が {rmw or '空'} です。rmw_cyclonedds_cpp にします"
                              "（コンテナでは ~/unitree_ros2/setup.sh が決めます。上書きしていないか確かめます）"))
    if iface:
        uri = env.get("CYCLONEDDS_URI") or ""
        if f'name="{iface}"' in uri:
            out.append(Check(OK, f"CYCLONEDDS_URI：差し口 {iface} を使う"))
        else:
            out.append(Check(BAD, f"CYCLONEDDS_URI が差し口 {iface} を指していません。"
                                  "コンテナでは ~/unitree_ros2/setup.sh が GO2_IFACE から作ります"
                                  "（CYCLONEDDS_URI を env.txt や -e で渡していないか確かめます）"))
    return out


# ---------- 2. 差し口 ----------
def _ioctl(sock: socket.socket, request: int, name: str) -> bytes:
    return fcntl.ioctl(sock.fileno(), request, struct.pack("256s", name.encode()[:15]))


def read_iface(name: str) -> Optional[IfaceInfo]:
    """Linux の差し口の様子。その名前が無ければ None。"""
    if name not in {n for _, n in socket.if_nameindex()}:
        return None
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        flags = struct.unpack("H", _ioctl(s, 0x8913, name)[16:18])[0]   # SIOCGIFFLAGS
        try:
            address = socket.inet_ntoa(_ioctl(s, 0x8915, name)[20:24])   # SIOCGIFADDR
            netmask = socket.inet_ntoa(_ioctl(s, 0x891B, name)[20:24])   # SIOCGIFNETMASK
        except OSError:   # IPv4 がついていない
            address = netmask = None
    return IfaceInfo(up=bool(flags & 0x1), running=bool(flags & 0x40), multicast=bool(flags & 0x1000),
                     address=address, netmask=netmask)


def check_iface(name: str, info: Optional[IfaceInfo]) -> list[Check]:
    if info is None:
        return [Check(BAD, f"差し口 {name} が見つかりません。PC で `ip -br addr` を見て名前を確かめます。"
                           "compose.robot.yaml（network_mode: host）で起動していないと、コンテナから見えません")]
    if not info.up:
        return [Check(BAD, f"差し口 {name} がオフです。Ubuntu の「設定 → ネットワーク」で有線をオンにします")]
    if not info.running:
        return [Check(BAD, f"差し口 {name} にリンクがありません。ケーブルが両方に挿さっているか、"
                           "Go2 の電源が入っているかを確かめます")]
    if not info.multicast:
        return [Check(BAD, f"差し口 {name} でマルチキャストが使えません（DDS が Go2 を見つけられません）。"
                           f"`sudo ip link set {name} multicast on` で戻します")]
    if info.address is None:
        return [Check(BAD, f"差し口 {name} に IPv4 がついていません。{PC_EXAMPLE} などに固定します")]
    addr = ipaddress.ip_address(info.address)
    if addr not in GO2_NET:
        return [Check(BAD, f"差し口 {name} の IPv4 {info.address} が {GO2_NET} にありません。"
                           f"{PC_EXAMPLE} などに固定します（差し口の名前の取り違えも確かめます）")]
    if info.address == GO2_ADDRESS:
        return [Check(BAD, f"差し口 {name} の IPv4 が {GO2_ADDRESS} です。これは Go2 本体のものなので、"
                           f"{PC_EXAMPLE} など別の値にします")]
    prefix = ipaddress.ip_network(f"0.0.0.0/{info.netmask}").prefixlen if info.netmask else None
    if prefix != GO2_NET.prefixlen:
        return [Check(WARN, f"差し口 {name}：{info.address}（ネットマスク {info.netmask}）。公式どおり "
                            "255.255.255.0（/24）にしておくと、ほかの経路と混ざりません")]
    return [Check(OK, f"差し口 {name}：{info.address}/{prefix}・リンクあり")]


# ---------- 3・4. 届くもの ----------
def check_topics(seconds: float, n_state: int, n_cloud: int, error_code: Optional[int],
                 request_subs: int, request_pubs: int, require_cloud: bool = True) -> list[Check]:
    out: list[Check] = []
    if n_state:
        out.append(Check(OK, f"{STATE_TOPIC}：{seconds:g} 秒で {n_state} 回（{n_state / seconds:.0f} Hz）"))
    else:
        out.append(Check(BAD, f"{STATE_TOPIC} が {seconds:g} 秒届きません（Go2 が見えていません）。{HELP}"))
    if error_code is not None:
        state = state_from_error_code(error_code)
        text = f"error_code {error_code}（{POSTURE_JA[state]}）"
        if state == UNKNOWN:
            out.append(Check(WARN, f"{text}。アプリは姿勢がわからないので、計画を始められないことがあります。"
                                   "リモコンで伏せさせてから、もう一度見ます（docs/real_robot.md の error_code の表）"))
        else:
            out.append(Check(OK, text))
    if n_cloud:
        out.append(Check(OK, f"{LIDAR_TOPIC}：{seconds:g} 秒で {n_cloud} 回（{n_cloud / seconds:.0f} Hz）"))
    elif require_cloud:
        out.append(Check(BAD, f"{LIDAR_TOPIC} が {seconds:g} 秒届きません。届くまで、アプリは移動を保留します。{HELP}"))
    else:
        out.append(Check(WARN, "障害物判定はGo2本体へ委譲します。PCのLiDAR受信は移動条件に使いません。"))
    if request_subs:
        out.append(Check(OK, f"{REQUEST_TOPIC} の受け手（Go2 の運動の制御）：{request_subs}"))
    else:
        out.append(Check(BAD, f"{REQUEST_TOPIC} の受け手がいません。Unitree のアプリの「サービスの状態」で、"
                              "運動の制御のサービス（mcf。古いソフトでは sport_mode）が止まっていないか確かめます"))
    if request_pubs:
        out.append(Check(BAD, f"{REQUEST_TOPIC} の送り手が {request_pubs} つ見えます。このままだとサーバーは起動を断ります。"
                              "サーバー・sensor_check・ほかのプログラムを終了します。どれも動かしていないのに見えるなら、"
                              "その数を書き留めて知らせてください（実機では未確認）"))
    else:
        out.append(Check(OK, f"{REQUEST_TOPIC} のほかの送り手：なし"))
    return out


def listen(seconds: float) -> list[Check]:
    """seconds 秒受けてから、数を check_topics にかける。何も送らない（送り手も作らない）。"""
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2
    from unitree_go.msg import SportModeState
    from unitree_api.msg import Request
    from voice_ltl.publisher_guard import PublisherGuard

    rclpy.init()
    node = rclpy.create_node("go2_robot_check")
    got = {"state": 0, "cloud": 0, "code": None}
    guard = PublisherGuard()

    def on_state(msg: SportModeState) -> None:
        got["state"] += 1
        got["code"] = int(msg.error_code)

    def on_cloud(_msg: PointCloud2) -> None:
        got["cloud"] += 1

    try:
        node.create_subscription(SportModeState, STATE_TOPIC, on_state, qos_profile_sensor_data)
        node.create_subscription(PointCloud2, LIDAR_TOPIC, on_cloud, qos_profile_sensor_data)
        node.create_subscription(Request, REQUEST_TOPIC, guard.on_request, qos_profile_sensor_data)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
        subs = max(0, node.count_subscribers(REQUEST_TOPIC) - 1)  # 自分の受信監視を除く
        total_pubs = node.count_publishers(REQUEST_TOPIC)
        guard.freeze(node)
        pubs = guard.conflicts(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    checks = check_topics(seconds, got["state"], got["cloud"], got["code"], subs, pubs,
                          require_cloud=os.environ.get("GO2_OBSTACLE_AUTHORITY") != "go2")
    if total_pubs != pubs:
        checks.append(Check(WARN, f"待機中のnative DDS送り手 {total_pubs - pubs} 個（監視中の指令送信なし）。"
                            "実際に指令を送った場合や新しい送り手が増えた場合は競合として拒否します。"))
    return checks


def config_from_args(argv: Optional[Iterable[str]] = None) -> float:
    """受ける秒数（--seconds、既定 5）。"""
    p = argparse.ArgumentParser(prog="python3 -m voice_ltl.robot_check",
                                description="実機の Go2 と有線でつながったかを確かめる（Go2 には何も送らない）")
    p.add_argument("--seconds", type=float, default=DEFAULT_SECONDS, help="トピックを受ける秒数")
    a = p.parse_args(None if argv is None else list(argv))
    if not 0.0 < a.seconds < 600.0:
        p.error("--seconds は 0 より大きく 600 より小さい秒数にします")
    return a.seconds


def main(argv: Optional[Iterable[str]] = None, env: Mapping[str, str] = os.environ,
         out: Callable[[str], None] = _print) -> int:
    seconds = config_from_args(argv)
    out("実機の Go2 との接続を確かめます。Go2 には何も送りません。")
    checks = check_settings(env)
    iface = (env.get("GO2_IFACE") or "").strip()
    if iface:
        checks += check_iface(iface, read_iface(iface))
    for c in checks:
        out(format_check(c))
    if exit_code(checks):
        out("上の［だめ］を直してから、もう一度流します（ROS 2 はまだ起動していません）。")
        return 1
    out(f"{seconds:g} 秒、トピックを受けます…")
    topic_checks = listen(seconds)
    for c in topic_checks:
        out(format_check(c))
    code = exit_code(topic_checks)
    out("全部 OK です。サーバーを起動できます。" if code == 0 else "上の［だめ］を直してから、もう一度流します。")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
