"""Linux PCでDockerを使わず起動する。既定は実機と分離した偽Go2。"""

import argparse
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
from xml.sax.saxutils import quoteattr

from dotenv import dotenv_values

ROOT = Path(__file__).resolve().parents[1]


def environment(mode, original=None):
    env = dict(os.environ if original is None else original)
    # env.txtはシェルとして実行しない。ファイルの値をこのアプリだけへ渡す。
    for key, value in dotenv_values(ROOT / "env.txt").items():
        if value is not None:
            env[key] = value
    env["GO2_NATIVE"] = "1"
    # このPCのCUDA 13とWhisperのCUDA 12依存を混在させず、既定はCPUで動かす。
    env.setdefault("WHISPER_DEVICE", "cpu")
    env["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
    if mode in ("fake", "test"):
        # HumbleはROS_LOCALHOST_ONLY=1だとloをXMLに追加するため、明示指定と重複する。
        # localhostへの制限は下のCycloneDDS XMLで行う。
        env.update(SIM="1", SIM_KIND="fake", ROS_DOMAIN_ID="73", ROS_LOCALHOST_ONLY="0",
                   GO2_OBSTACLE_AUTHORITY="pc", GO2_LIDAR_PROFILE="simulation")
        env["CYCLONEDDS_URI"] = (
            '<CycloneDDS><Domain><General><Interfaces>'
            '<NetworkInterface name="lo" multicast="false"/>'
            '</Interfaces><AllowMulticast>false</AllowMulticast></General>'
            '<Discovery><ParticipantIndex>auto</ParticipantIndex>'
            '<Peers><Peer Address="127.0.0.1"/></Peers></Discovery>'
            '</Domain></CycloneDDS>'
        )
    else:
        env.setdefault("GO2_OBSTACLE_AUTHORITY", "go2")
        if mode == "sensors":
            env["GO2_OBSTACLE_AUTHORITY"] = "pc"
        env.setdefault("GO2_LIDAR_PROFILE", "go2")
        iface = (env.get("GO2_IFACE") or "").strip()
        if not iface:
            raise ValueError("env.txtのGO2_IFACEに有線LAN名を設定してください。")
        env.update(SIM="0", ROS_DOMAIN_ID="0", ROS_LOCALHOST_ONLY="0")
        env["CYCLONEDDS_URI"] = (
            '<CycloneDDS><Domain><General><Interfaces>'
            f'<NetworkInterface name={quoteattr(iface)} priority="default" multicast="default"/>'
            '</Interfaces></General></Domain></CycloneDDS>'
        )
    # 前回の音声アプリで取得済みのsmallモデルがあれば再利用する。
    cache = ROOT.parent / ".models" / "whisper"
    if cache.is_dir():
        env.setdefault("HF_HUB_CACHE", str(cache))
    # VS Code同梱のCodexも、通常のターミナルから見つけられるようにする。
    if not shutil.which("codex", path=env.get("PATH")):
        candidates = list((Path.home() / ".vscode" / "extensions").glob(
            "openai.chatgpt-*/bin/linux-x86_64/codex"))
        candidates = [p for p in candidates if os.access(p, os.X_OK)]
        if candidates:
            binary = max(candidates, key=lambda p: p.stat().st_mtime)
            env["PATH"] = str(binary.parent) + os.pathsep + env.get("PATH", "")
    return env


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", nargs="?", default="fake",
                        choices=("fake", "robot", "check", "sensors", "test"))
    args = parser.parse_args()
    processes = []
    try:
        env = environment(args.mode)
        if args.mode == "test":
            command = [sys.executable, "-m", "pytest", "-q", "-m", "not slow", "tests", "nl_control/tests"]
        elif args.mode in ("check", "sensors"):
            module = "robot_check" if args.mode == "check" else "sensor_check"
            command = [sys.executable, "-m", f"voice_ltl.{module}"]
        else:
            if args.mode == "fake":
                processes.append(subprocess.Popen(
                    [sys.executable, "-m", "sim.fake_go2"], env=env, cwd=ROOT,
                    start_new_session=True))
                print("偽Go2: localhostのみ・ROSドメイン73。実機のドメイン0と分離しています。", flush=True)
            command = [sys.executable, "-m", "voice_ltl.server"]
        processes.append(subprocess.Popen(command, env=env, cwd=ROOT, start_new_session=True))
        return processes[-1].wait()
    except KeyboardInterrupt:
        return 130
    except (OSError, ValueError) as exc:
        print(f"起動エラー: {exc}", file=sys.stderr)
        return 1
    finally:
        for process in reversed(processes):
            stop(process)


if __name__ == "__main__":
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, interrupt)
    sys.exit(main())
