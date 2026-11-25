"""UWB リモコン追尾の専用ページ。通常の動作指示は扱わない。

UWB の測距と方位を購読し、進む方向と速度を計算して Go2 に送る。
シミュレーション（SIM=1）では、にせ UWB リモコン（sim.fake_uwb）を同じプロセスで動かし、
ページの地図をクリック・ドラッグしてリモコンを動かせる。
"""

import asyncio
import logging
import math
import os
import threading
import time
from contextlib import closing
from dataclasses import dataclass

import numpy as np
import rclpy
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, Response as HttpResponse
from fastapi.staticfiles import StaticFiles
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from starlette.concurrency import run_in_threadpool
from unitree_go.msg import SportModeState, UwbState

from go2_nl_control.actions import ACTIONS, LYING, MOVE, STANDING
from go2_nl_control.executor import STOP, Command
from go2_nl_control.supervisor import SENSOR_LOST
from voice_ltl.camera import SimCamera, VideoHubCamera
from voice_ltl.avoid import AvoidConfig, choose_heading, free_length, obstacle_points
from voice_ltl.perception_config import perception_config
from voice_ltl.ros_link import LIDAR_TOPIC, STATE_TIMEOUT, STATE_TOPIC, RosLink
from voice_ltl.server import STATIC, LocalOnly, listen_address, sim_mode, ui_port, wait_ready_seconds
from voice_ltl.session import STAND_LOCK

UWB_STATE_TOPIC = "/uwbstate"
UWB_STATE_TIMEOUT = 1.0
HEARTBEAT_TIMEOUT = 3.0
STOPPED = "この画面からの追尾は停止しています"
STOP_FAILED = "Go2 に停止指令を送れません。純正リモコンで非常停止してください"
STOP_UNCONFIRMED = "Go2 の停止を確認できません。純正リモコンで非常停止してください"
STOP_CONFIRM_S = 0.3
MAX_TURN_WITHOUT_PROGRESS = math.radians(210)
TURN_PROGRESS_M = 0.15
UWB_FRESH_S = 0.5     # 自前の制御は、これより古い UWB の測定では動かない [s]
CLOUD_FRESH_S = 0.5   # これより古い点群では、よける向きを選ばない [s]
MAP_GRID = 0.1        # 地図に出す点の間引きの格子 [m]
MAP_MAX_POINTS = 3000  # 地図に出す点の数の上限
CONTROL_PERIOD = 0.1  # 自前の制御が Move を送る間隔 [s]（Move は 1 秒で切れる）
log = logging.getLogger(__name__)


def check_robot_state(link):
    link.spin_once()
    if not link.state_fresh(time.time()):
        raise RuntimeError("Go2 の状態が届いていません")


def check_robot(link):
    check_robot_state(link)
    if link.posture() != STANDING:
        raise RuntimeError("Go2 を立たせてから追尾を開始してください")


class FollowPositions:
    """追尾開始前にも見られる、位置表示専用の購読。"""

    fake = None   # シミュレーションのにせ UWB リモコン（sim.fake_uwb.FakeUwb）
    perception = None  # LiDAR の取り付け（None なら点群を受けない）
    cloud = None  # (受けた時刻, 機体座標の障害物の点 M×2, 受けたときの Go2 の位置か None)
    motion = None

    def __init__(self, simulate_uwb=False, perception=None):
        self.pose = None
        self.uwb = None
        self.perception = perception
        self.node = rclpy.create_node("go2_follow_positions")
        self.node.create_subscription(SportModeState, STATE_TOPIC, self.on_pose, qos_profile_sensor_data)
        self.node.create_subscription(UwbState, UWB_STATE_TOPIC, self.on_uwb, qos_profile_sensor_data)
        if perception is not None:
            self.node.create_subscription(PointCloud2, LIDAR_TOPIC, self.on_cloud, qos_profile_sensor_data)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        if simulate_uwb:
            from sim.fake_uwb import FakeUwb
            self.fake = FakeUwb()
            self.executor.add_node(self.fake)
        self.thread = threading.Thread(target=self.executor.spin, name="go2-follow-positions", daemon=True)
        self.thread.start()

    def on_pose(self, msg):
        now = time.monotonic()
        pose = (float(msg.position[0]), float(msg.position[1]), float(msg.imu_state.rpy[2]))
        self.pose = (now, pose) if all(map(math.isfinite, pose)) else None
        speed = math.hypot(float(msg.velocity[0]), float(msg.velocity[1]))
        yaw_speed = abs(float(msg.yaw_speed))
        previous = self.motion
        still_since = None
        if speed <= 0.05 and yaw_speed <= 0.05:
            still_since = now
            if previous is not None and now - previous[0] <= STATE_TIMEOUT and previous[1] is not None:
                still_since = previous[1]
        self.motion = (now, still_since)

    def stationary_after(self, start):
        sample = self.motion
        return (sample is not None and sample[1] is not None
                and time.monotonic() - sample[0] <= STATE_TIMEOUT
                and sample[0] - max(sample[1], start) >= STOP_CONFIRM_S)

    def on_uwb(self, msg):
        now = time.monotonic()
        self.uwb = (now, float(msg.distance_est), float(msg.orientation_est),
                    int(msg.channel), int(msg.error_state), self._recent_pose(now))

    def on_cloud(self, msg):
        points = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        xyz = np.column_stack([points[name] for name in ("x", "y", "z")])
        now = time.monotonic()
        self.cloud = (now, obstacle_points(xyz, self.perception), self._recent_pose(now))

    def obstacles(self, max_age=CLOUD_FRESH_S):
        """新しい点群から求めた、機体座標の障害物の点（M×2）。届いていない・古ければ None。"""
        cloud = self.cloud
        if cloud is None or time.monotonic() - cloud[0] > max_age:
            return None
        return cloud[1]

    def obstacle_snapshot(self):
        """地図用：障害物の点（オドメトリ座標・間引き）と、Go2 から見た近さ。点群が無ければ None。"""
        cloud = self.cloud
        if cloud is None or time.monotonic() - cloud[0] > CLOUD_FRESH_S:
            return None
        _, body, pose = cloud
        result = {"points": [], "nearest": None, "nearest_bearing": None,
                  "front_free": free_length(body, 0.0)}
        if math.isinf(result["front_free"]):
            result["front_free"] = None
        if len(body):
            ranges = np.hypot(body[:, 0], body[:, 1])
            i = int(np.argmin(ranges))
            result["nearest"] = float(ranges[i])
            result["nearest_bearing"] = float(math.atan2(body[i, 1], body[i, 0]))
            if pose is not None:
                shown = np.unique(np.round(body / MAP_GRID), axis=0) * MAP_GRID
                shown = shown[::max(1, -(-len(shown) // MAP_MAX_POINTS))]
                x, y, yaw = pose
                c, s = math.cos(yaw), math.sin(yaw)
                world = np.column_stack([x + c * shown[:, 0] - s * shown[:, 1],
                                         y + s * shown[:, 0] + c * shown[:, 1]])
                result["points"] = np.round(world, 2).tolist()
        return result

    def _recent_pose(self, now):
        pose = self.pose
        return pose[1] if pose and now - pose[0] <= STATE_TIMEOUT else None

    def measurement(self, max_age=UWB_FRESH_S):
        """新しい UWB の測定 (距離 [m], 方向 [rad]、左が正)。古い・エラー・不正なら None。"""
        sample = self.uwb
        if sample is None or time.monotonic() - sample[0] > max_age:
            return None
        _, distance, bearing, _, error, _ = sample
        if error != 0 or not (distance > 0 and math.isfinite(distance) and math.isfinite(bearing)):
            return None
        return distance, math.atan2(math.sin(bearing), math.cos(bearing))

    def snapshot(self):
        now = time.monotonic()
        pose = self._recent_pose(now)
        result = {"go2": pose, "uwb": None}
        if self.fake is not None:
            result["sim_tag"] = self.fake.tag.position
        if self.perception is not None:
            result["obstacles"] = self.obstacle_snapshot()
        sample = self.uwb
        if sample is None or now - sample[0] > UWB_STATE_TIMEOUT:
            return result
        _, distance, bearing, channel, error, pose_at_sample = sample
        uwb = {"channel": channel, "error": error, "distance": None, "bearing": None, "position": None}
        if error == 0 and distance > 0 and math.isfinite(distance) and math.isfinite(bearing):
            bearing = math.atan2(math.sin(bearing), math.cos(bearing))
            uwb.update(distance=distance, bearing=bearing)
            if pose is not None and pose_at_sample is not None:
                x, y, yaw = pose_at_sample
                uwb["position"] = (x + distance * math.cos(yaw + bearing),
                                   y + distance * math.sin(yaw + bearing))
        result["uwb"] = uwb
        return result

    def close(self):
        self.executor.shutdown()
        self.thread.join()
        if self.fake is not None:
            self.fake.destroy_node()
        self.node.destroy_node()


@dataclass(frozen=True)
class WalkConfig:
    stop_distance: float = 1.0          # リモコンにこれより近づいたら止まる [m]
    resume_margin: float = 0.3          # 止まった後は stop_distance + これより離れたら歩き出す [m]
    max_speed: float = 0.4              # 前進の速さの上限 [m/s]
    speed_gain: float = 0.8             # 前進の速さ = これ × (距離 − stop_distance)
    max_turn: float = 0.8               # 旋回の速さの上限 [rad/s]
    turn_gain: float = 1.5              # 旋回の速さ = これ × 方向
    turn_only: float = math.radians(35)  # 方向のずれがこれより大きい間は、その場で回るだけ
    face_deadband: float = math.radians(10)
    acc: float = 0.6                    # 前進の加速度の上限 [m/s^2]（減速は制限しない）
    turn_acc: float = 2.0               # 旋回の加速度の上限 [rad/s^2]


def _clamp(value, limit):
    return max(-limit, min(limit, value))


def walk_velocity(distance, bearing, front_blocked, walking, cfg=WalkConfig()):
    """UWB の距離・方向から (前進 [m/s], 旋回 [rad/s], 歩いているか) を求める。

    止まる距離に行き違いの幅（resume_margin）を持たせ、止まる・歩くを繰り返さないようにする。
    前に障害物があれば前進しない（その場の旋回だけ）。後ろ・横へは動かない。
    """
    limit = cfg.stop_distance + (0.0 if walking else cfg.resume_margin)
    walking = distance > limit
    if not walking:
        return 0.0, 0.0, False
    turn = 0.0 if abs(bearing) < cfg.face_deadband else _clamp(cfg.turn_gain * bearing, cfg.max_turn)
    forward = 0.0
    if walking and abs(bearing) < cfg.turn_only and not front_blocked:
        forward = min(cfg.max_speed, cfg.speed_gain * (distance - cfg.stop_distance)) * math.cos(bearing)
    return max(0.0, forward), turn, walking


def _ramp_up(current, target, step):
    """target へ近づける。速くなる向きだけ step で制限し、遅くなる・止まる向きはすぐ従う。"""
    if current * target >= 0 and abs(target) > abs(current):
        return current + math.copysign(min(step, abs(target) - abs(current)), target)
    return target


class WalkToUwb:
    """このプログラムが UWB の位置へ Go2 を歩かせる。"""

    def __init__(self, link, positions, cfg=WalkConfig(), avoid=AvoidConfig()):
        self.link = link
        self.positions = positions
        self.use_pc_lidar = link.obstacle_authority == "pc"
        self.cfg = cfg
        self.avoid = avoid
        self.side = 0   # 障害物をよけている側（choose_heading）
        self.heading = None  # 地図用：いま進もうとしている向き（オドメトリ座標の角度）
        self.lock = threading.Lock()
        self.command_lock = threading.Lock()
        self.halt = threading.Event()
        self.standing_up = False
        self.stop_unconfirmed = False
        self.stop_epoch = 0
        self.owner = None
        self.thread = None
        self.fault = None
        self.message = ""
        self.velocity = (0.0, 0.0)
        self.walking = False
        self.turn_budget_used = 0.0
        self.progress_distance = math.inf

    def start(self, owner, stop_epoch=None):
        with self.lock:
            if stop_epoch is not None and self.stop_epoch != stop_epoch:
                raise RuntimeError(STOPPED)
            if self.stop_unconfirmed:
                raise RuntimeError(STOP_UNCONFIRMED)
            if self.owner is not None:
                raise RuntimeError("追尾を開始済みです")
            if self.standing_up:
                raise RuntimeError("Go2 が立ち上がるまで待ってください")
            check_robot(self.link)
            if self.positions.measurement() is None:
                raise RuntimeError("UWB リモコンの測位が届いていません")
            if self.use_pc_lidar and self._pc_obstacles() is None:
                raise RuntimeError("LiDAR の点群が届いていません")
            if owner.is_set():
                raise RuntimeError("ブラウザとの通信が途切れました")
            self.owner = owner
            self.fault = None
            self.message = "追尾を始めます"
            self.velocity = (0.0, 0.0)
            self.walking = False
            self.side = 0
            self.turn_budget_used = 0.0
            self.progress_distance = math.inf
            self.halt.clear()
            self.thread = threading.Thread(target=self._run, name="go2-walk-to-uwb", daemon=True)
            self.thread.start()

    def stand(self):
        with self.lock:
            if self.stop_unconfirmed:
                raise RuntimeError(STOP_UNCONFIRMED)
            if self.owner is not None or self.standing_up:
                raise RuntimeError("追尾や立ち上がりが終わってから操作してください")
            check_robot_state(self.link)
            posture = self.link.posture()
            if posture == STANDING:
                return "Go2 は立っています"
            steps = ACTIONS["stand"].sequence
            if self.link.error_code() == STAND_LOCK:
                steps = steps[1:]
            elif posture != LYING:
                raise RuntimeError("リモコンで Go2 を伏せさせてから立たせてください")
            self.halt.clear()
            self.standing_up = True
        try:
            for api, delay in steps:
                check_robot_state(self.link)
                if not self._send_motion(Command(api)) or self.halt.wait(delay):
                    raise RuntimeError("立ち上がりの指令を中断しました")
            check_robot(self.link)
            return "Go2 が立ちました"
        finally:
            with self.lock:
                self.standing_up = False

    def _run(self):
        try:
            while not self.halt.wait(CONTROL_PERIOD):
                self._tick(CONTROL_PERIOD)
        except RuntimeError as exc:
            self.fault = str(exc)
        except Exception:
            log.exception("追尾の制御で異常が起きました")
            self.fault = "追尾の制御で異常が起きました"
        if self.fault is not None and not self.halt.is_set():
            try:
                self._send_stop()
            except RuntimeError as exc:
                self.fault = str(exc)

    def _tick(self, dt):
        check_robot(self.link)
        measured = self.positions.measurement()
        if measured is None:
            self.heading = None
            target = (0.0, 0.0)
            self.walking = False
            self.message = "UWB リモコンを見失いました。止まって待っています"
        else:
            points = self._pc_obstacles() if self.use_pc_lidar else None
            if self.use_pc_lidar and points is None:
                self.heading = None
                self.walking = False
                self.message = "LiDAR の点群が途切れました。止まって待っています"
                target = (0.0, 0.0)
            else:
                distance, bearing = measured
                target = self._steer(distance, bearing, points)
                if self.progress_distance - distance >= TURN_PROGRESS_M:
                    self.progress_distance = distance
                    self.turn_budget_used = 0.0
                self.turn_budget_used += abs(target[1]) * dt
                if self.turn_budget_used >= MAX_TURN_WITHOUT_PROGRESS:
                    raise RuntimeError("リモコンに近づけないまま旋回が続いたため追尾を中止しました")
        previous = self.velocity
        self.velocity = (_ramp_up(previous[0], target[0], self.cfg.acc * dt),
                         _ramp_up(previous[1], target[1], self.cfg.turn_acc * dt))
        if self.velocity != (0.0, 0.0):
            self._send_motion(Command(MOVE, {"x": round(self.velocity[0], 3), "y": 0.0,
                                              "z": round(self.velocity[1], 3)}))
        elif previous != (0.0, 0.0):
            self._send_stop()

    def _pc_obstacles(self):
        if self.link.reading(time.time()).front == SENSOR_LOST:
            return None
        return self.positions.obstacles()

    def _steer(self, distance, bearing, points):
        """リモコンの距離・方向から前進と旋回の速度を決める。"""
        self.heading = None
        blocked = self.use_pc_lidar and self.link.reading(time.time()).front_blocked
        forward, turn, self.walking = walk_velocity(distance, bearing, blocked, self.walking, self.cfg)
        if not self.walking:
            self.side = 0
            self.message = f"リモコンのそばで待っています（{distance:.1f} m）"
            return forward, turn

        heading = bearing
        if self.use_pc_lidar:
            travel = distance - self.cfg.stop_distance
            heading, free, self.side = choose_heading(points, bearing, travel, self.side, self.avoid)
            if heading is None:
                self.message = f"回り込める道が見つかりません。止まっています（リモコンまで {distance:.1f} m）"
                return 0.0, 0.0
            forward, turn, self.walking = walk_velocity(distance, heading, blocked, True, self.cfg)
            # 空きが少ないほど遅く進む（空きの手前 0.1 m で止まる速さ）。
            forward = min(forward, max(0.0, self.cfg.speed_gain * (free - 0.1)))

        pose = self.link.pose()
        if pose is not None:
            self.heading = pose[2] + heading
        if not self.use_pc_lidar:
            self.message = f"追尾指令を送っています（リモコンまで {distance:.1f} m、障害物判定は Go2 本体）"
        elif blocked:
            self.message = f"前に障害物があるため、向きを変えています（リモコンまで {distance:.1f} m）"
        elif self.side:
            self.message = (f"障害物を{'左' if self.side > 0 else '右'}によけて、"
                            f"リモコンへ向かっています（{distance:.1f} m）")
        else:
            self.message = f"リモコンへ歩いています（{distance:.1f} m）"
        return forward, turn

    def _send_motion(self, command):
        with self.command_lock:
            if self.halt.is_set():
                return False
            self.link.send(command)
            return True

    def _send_stop(self):
        with self.command_lock:
            failure = None
            for attempt in range(3):
                if attempt:
                    time.sleep(0.05)
                try:
                    self.link.send(STOP)
                    failure = None
                except Exception as exc:
                    failure = exc
                    log.exception("停止指令の送信に失敗しました")
            if failure is not None:
                raise RuntimeError(STOP_FAILED) from failure

    def plan(self):
        """地図用：追尾中に進もうとしている向き（オドメトリ座標の角度）。無ければ None。"""
        return self.heading if self.owner is not None else None

    def status(self, owner):
        with self.lock:
            if self.owner is not owner:
                return None
            if self.fault is not None:
                raise RuntimeError(self.fault)
            return self.message

    def stop(self, owner=None, *, confirm=False):
        with self.lock:
            if owner is not None and self.owner is not owner:
                return False
            if owner is None:
                self.stop_epoch += 1
            self.halt.set()
            stopped_at = time.monotonic()
            if confirm:
                self.stop_unconfirmed = True
            try:
                self._send_stop()
            except RuntimeError:
                self.stop_unconfirmed = True
                raise
            finally:
                if self.thread is not None:
                    self.thread.join()
                    self.thread = None
                self.owner = None
                self.heading = None
                self.velocity = (0.0, 0.0)
            if confirm:
                deadline = stopped_at + 2.0
                while not self.positions.stationary_after(stopped_at):
                    if time.monotonic() >= deadline:
                        raise RuntimeError(STOP_UNCONFIRMED)
                    time.sleep(0.05)
                self.stop_unconfirmed = False
            return True

    def close(self):
        self.stop()


class SpinningLink:
    """RosLink の受信を裏で処理し続ける RosLink の代わり。

    RosLink は spin_once を呼ばれたときだけ受信を処理する。追尾ページは音声アプリと違って
    制御ループを持たないので、放っておくと自分の指令の受信が古くなり、ほかの送り手と取り違える。
    spin_once は 1 つのロックで順に呼ぶ。裏で見つけた異常は、次の spin_once で 1 回だけ伝える。
    """

    def __init__(self, link, period=0.05):
        self._link = link
        self._lock = threading.Lock()
        self._error = None
        self._halt = threading.Event()
        self._thread = threading.Thread(target=self._run, args=(period,), name="go2-follow-spin", daemon=True)
        self._thread.start()

    def __getattr__(self, name):
        return getattr(self._link, name)

    def _run(self, period):
        while not self._halt.wait(period):
            try:
                with self._lock:
                    self._link.spin_once()
            except RuntimeError as exc:
                self._error = exc
            except Exception:
                log.exception("ROS の受信処理で異常が起きました")

    def spin_once(self):
        with self._lock:
            error, self._error = self._error, None
            if error is not None:
                raise error
            self._link.spin_once()

    def close(self):
        self._halt.set()
        self._thread.join(timeout=2)


class SimTools:
    """シミュレーションの UWB リモコンを動かす。"""

    def __init__(self, tag):
        self.tag = tag

    def place(self, x, y):
        return self.tag.place(x, y)


def create_follow_app(track, page_port, positions, sim=None, camera=None):
    app = FastAPI(docs_url=None, redoc_url=None, openapi_url=None)
    app.add_middleware(LocalOnly, page_port=page_port)
    app.mount("/static", StaticFiles(directory=STATIC), name="static")

    @app.get("/")
    def index():
        return FileResponse(STATIC / "follow.html", headers={"Cache-Control": "no-store"})

    @app.get("/api/config")
    def config():
        return {"control": "own", "sim": sim is not None, "camera": camera is not None}

    @app.get("/api/camera")
    def camera_image():
        frame = camera.latest() if camera is not None else None
        if frame is None:
            return HttpResponse(status_code=204, headers={"Cache-Control": "no-store"})
        _, kind, data = frame
        return HttpResponse(content=data, media_type=kind, headers={"Cache-Control": "no-store"})

    @app.post("/api/stand")
    def stand():
        try:
            return {"message": track.stand()}
        except RuntimeError as exc:
            return JSONResponse({"message": str(exc)}, status_code=409)

    if sim is not None:
        app.add_api_route("/api/sim/stand", stand, methods=["POST"])

        @app.post("/api/sim/uwb")
        def place_uwb(body: dict):
            try:
                return {"position": sim.place(body.get("x"), body.get("y"))}
            except (ValueError, TypeError) as exc:
                return JSONResponse({"message": str(exc)}, status_code=400)

    @app.get("/api/position")
    def position():
        data = positions.snapshot()
        plan = getattr(track, "plan", None)
        if callable(plan):
            data["heading"] = plan()
        return JSONResponse(data, headers={"Cache-Control": "no-store"})

    @app.post("/api/stop")
    def stop():
        try:
            track.stop(confirm=True)
        except RuntimeError as exc:
            return JSONResponse({"message": str(exc)}, status_code=503)
        return {"message": "Go2 の停止を確認しました"}

    @app.websocket("/ws/follow")
    async def follow(socket: WebSocket):
        stop_epoch = track.stop_epoch
        await socket.accept()
        owner = threading.Event()
        heartbeat = None
        message = STOPPED

        async def receive_heartbeat():
            try:
                if await socket.receive_text() == "continue":
                    return True
            except WebSocketDisconnect:
                pass
            owner.set()
            return False

        try:
            if await asyncio.wait_for(socket.receive_text(), HEARTBEAT_TIMEOUT) != "start":
                return
            heartbeat = asyncio.create_task(receive_heartbeat())
            await run_in_threadpool(track.start, owner, stop_epoch)
            while (status := await run_in_threadpool(track.status, owner)) is not None:
                await socket.send_json({"message": status, "active": True})
                if not await asyncio.wait_for(heartbeat, HEARTBEAT_TIMEOUT):
                    break
                heartbeat = asyncio.create_task(receive_heartbeat())
        except asyncio.TimeoutError:
            message = "ブラウザとの通信が途切れました"
        except RuntimeError as exc:
            message = str(exc)
        except (WebSocketDisconnect, OSError):
            return
        finally:
            if heartbeat is not None:
                heartbeat.cancel()
                await asyncio.gather(heartbeat, return_exceptions=True)
            try:
                await run_in_threadpool(track.stop, owner)
            except RuntimeError as exc:
                message = str(exc)
                log.error("%s", message)
        try:
            await socket.send_json({"message": message})
            await socket.close()
        except (WebSocketDisconnect, OSError, RuntimeError):
            pass

    return app


def main():
    logging.basicConfig(level=logging.INFO)
    try:
        host, port = listen_address(os.environ)
        page_port = ui_port(os.environ)
        wait_s = wait_ready_seconds(os.environ)
        simulating = sim_mode(os.environ)
        perception = perception_config()
        link = RosLink(perception, obstacle_authority="pc" if simulating else "go2")
    except (ValueError, RuntimeError) as exc:
        raise SystemExit(str(exc)) from None
    stop_times = 0
    try:
        link.wait_ready(wait_s)
        link.prepare_onboard_avoidance()
        stop_times = 3
        with (closing(FollowPositions(simulate_uwb=simulating, perception=perception)) as positions,
              closing(SpinningLink(link)) as spinning):
            with (closing(WalkToUwb(spinning, positions)) as track,
                  closing(SimCamera(positions.node) if simulating else VideoHubCamera()) as camera):
                sim = SimTools(positions.fake.tag) if simulating else None
                log.info("UWB 追尾のページ: http://localhost:%d", page_port)
                uvicorn.run(create_follow_app(track, page_port, positions, sim, camera),
                            host=host, port=port, timeout_graceful_shutdown=2)
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from None
    except KeyboardInterrupt:
        pass
    finally:
        link.close(stop_times=stop_times)


if __name__ == "__main__":
    main()
