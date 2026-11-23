"""UWB リモコン追尾の専用ページ。通常の動作指示は扱わない。

追尾のやり方は FOLLOW_CONTROL で選ぶ:
- utrack: Go2 本体の UTrack に任せる（実機の既定）。
- own: このプログラムが UWB の距離と方向から Move を送り、リモコンへ歩いていく
  （シミュレーションの既定。シミュレーションには UTrack が無いので own だけ）。
シミュレーション（SIM=1）では、にせ UWB リモコン（sim.fake_uwb）を同じプロセスで動かし、
ページの地図をクリック・ドラッグしてリモコンを動かせる。
"""

import asyncio
import json
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
from unitree_api.msg import Request, Response
from unitree_go.msg import SportModeState, UwbState

from go2_nl_control.actions import BALANCE_STAND, MOVE, STAND_UP, STANDING
from go2_nl_control.executor import STOP, Command
from go2_nl_control.supervisor import SENSOR_LOST
from voice_ltl.camera import SimCamera, VideoHubCamera
from voice_ltl.avoid import AvoidConfig, choose_heading, free_length, obstacle_points
from voice_ltl.onboard_avoidance import OnboardAvoidance
from voice_ltl.perception_config import perception_config
from voice_ltl.ros_link import LIDAR_TOPIC, STATE_TIMEOUT, STATE_TOPIC, RosLink
from voice_ltl.server import STATIC, LocalOnly, listen_address, sim_mode, ui_port, wait_ready_seconds

REQUEST_TOPIC = "/api/uwbswitch/request"
RESPONSE_TOPIC = "/api/uwbswitch/response"
UWB_STATE_TOPIC = "/uwbstate"
UWB_STATE_TIMEOUT = 1.0
SWITCH_SET, SWITCH_GET, IS_TRACKING = 1001, 1002, 1003
REPLY_TIMEOUT = 1.0
HEARTBEAT_TIMEOUT = 3.0
STOPPED = "この画面からの追尾は停止しています"
STOP_FAILED = "Go2 の追尾解除を確認できません。リモコンで停止してください"
UWB_FRESH_S = 0.5     # 自前の制御は、これより古い UWB の測定では動かない [s]
CLOUD_FRESH_S = 0.5   # これより古い点群では、よける向きを選ばない [s]
MAP_GRID = 0.1        # 地図に出す点の間引きの格子 [m]
MAP_MAX_POINTS = 3000  # 地図に出す点の数の上限
CONTROL_PERIOD = 0.1  # 自前の制御が Move を送る間隔 [s]（Move は 1 秒で切れる）
FOLLOW_CONTROLS = ("utrack", "own")
log = logging.getLogger(__name__)


def follow_control(env):
    """FOLLOW_CONTROL（utrack / own）。空ならシミュレーションは own、実機は utrack。"""
    simulating = sim_mode(env)
    control = (env.get("FOLLOW_CONTROL") or "").strip().lower() or ("own" if simulating else "utrack")
    if control not in FOLLOW_CONTROLS:
        raise ValueError(f"FOLLOW_CONTROL は utrack か own にしてください（今: {control!r}）")
    if simulating and control == "utrack":
        raise ValueError("シミュレーションには Go2 本体の追尾（UTrack）がありません。FOLLOW_CONTROL=own にしてください")
    return control


def check_robot(link):
    link.spin_once()
    if link.reading(time.time()).front == SENSOR_LOST:
        raise RuntimeError("Go2 の状態が届いていません")
    if link.posture() != STANDING:
        raise RuntimeError("Go2 を立たせてから追尾を開始してください")


class FollowPositions:
    """追尾開始前にも見られる、位置表示専用の購読。"""

    fake = None   # シミュレーションのにせ UWB リモコン（sim.fake_uwb.FakeUwb）
    perception = None  # LiDAR の取り付け（None なら点群を受けない）
    cloud = None  # (受けた時刻, 機体座標の障害物の点 M×2, 受けたときの Go2 の位置か None)

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
        pose = (float(msg.position[0]), float(msg.position[1]), float(msg.imu_state.rpy[2]))
        self.pose = (time.monotonic(), pose) if all(map(math.isfinite, pose)) else None

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


class UTrack:
    def __init__(self, link, avoidance):
        self.link = link
        self.avoidance = avoidance
        self.node = rclpy.create_node("go2_uwb_follow")
        self.publisher = self.node.create_publisher(Request, REQUEST_TOPIC, 10)
        self.node.create_subscription(Response, RESPONSE_TOPIC, self._receive, 10)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.lock = threading.Lock()
        self.owner = None
        self.reply = None
        self.identity = None
        self.was_tracking = False

    def _receive(self, msg):
        identity = msg.header.identity
        if (identity.id, identity.api_id) == self.identity:
            self.reply = msg

    def _call(self, api, parameters):
        request = Request()
        identity = time.monotonic_ns()
        request.header.identity.id = identity
        request.header.identity.api_id = api
        request.parameter = json.dumps(parameters)
        self.identity = (identity, api)
        self.reply = None
        self.publisher.publish(request)
        end = time.monotonic() + REPLY_TIMEOUT
        while self.reply is None and (remaining := end - time.monotonic()) > 0:
            self.executor.spin_once(timeout_sec=remaining)
        if self.reply is None:
            raise RuntimeError(f"Go2 の追尾サービスが応答しません（API {api}）")
        code = self.reply.header.status.code
        if code != 0:
            raise RuntimeError(f"Go2 が追尾の要求を拒否しました（API {api}, code {code}）")
        return self.reply.data

    def _enabled(self, api):
        try:
            value = json.loads(self._call(api, {}))["enable"]
        except (ValueError, TypeError, KeyError):
            raise RuntimeError(f"追尾サービスの状態を読み取れません（API {api}）") from None
        if type(value) not in (bool, int) or value not in (0, 1):
            raise RuntimeError(f"追尾サービスの状態が不正です（API {api}）")
        return bool(value)

    def _check_robot(self):
        check_robot(self.link)

    def start(self, owner):
        with self.lock:
            if self.owner is not None:
                raise RuntimeError("追尾を開始済みか、前回の解除を確認できていません")
            self._check_robot()
            if not self.publisher.get_subscription_count() or not self.node.count_publishers(RESPONSE_TOPIC):
                raise RuntimeError("Go2 の UWB 追尾サービスが見つかりません")
            self.avoidance.verify_enabled()
            if self._enabled(SWITCH_GET) or self._enabled(IS_TRACKING):
                raise RuntimeError("Go2 の追尾がすでに有効です。Unitree アプリかリモコンで解除してください")
            self._check_robot()
            if owner.is_set():
                raise RuntimeError("ブラウザとの通信が途切れました")
            # 応答が失われても、ここからは必ず解除を試みる。
            self.owner = owner
            self.was_tracking = False
            self._call(SWITCH_SET, {"enable": 1})

    def status(self, owner):
        with self.lock:
            if self.owner is not owner:
                return None
            self._check_robot()
            tracking = self._enabled(IS_TRACKING)
            if self.was_tracking and not tracking:
                raise RuntimeError("Go2 の追尾が途切れました")
            self.was_tracking = tracking
            return "UWB リモコンを追尾中" if tracking else "追尾モード有効。UWB リモコンを待っています"

    def stop(self, owner=None):
        with self.lock:
            if self.owner is None or (owner is not None and self.owner is not owner):
                return False
            try:
                self._call(SWITCH_SET, {"enable": 0})
            except RuntimeError:
                # 解除の要求または応答が失われた場合も、通常の停止指令を試す。
                try:
                    self.link.send(STOP)
                except Exception:
                    log.exception("Sport.StopMove の送信に失敗しました")
                try:
                    self._call(SWITCH_SET, {"enable": 0})
                except RuntimeError:
                    raise RuntimeError(STOP_FAILED) from None
            self.owner = None
            return True

    def close(self):
        try:
            self.stop()
        finally:
            self.executor.shutdown()
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
    face_deadband: float = math.radians(10)  # 止まっている間、これより小さいずれは直さない
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
    if not walking and abs(bearing) < cfg.face_deadband:
        turn = 0.0
    else:
        turn = _clamp(cfg.turn_gain * bearing, cfg.max_turn)
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
    """このプログラムが UWB の位置へ Go2 を歩かせる。UTrack と同じ start・status・stop を持つ。"""

    def __init__(self, link, positions, cfg=WalkConfig(), avoid=AvoidConfig()):
        self.link = link
        self.positions = positions
        self.cfg = cfg
        self.avoid = avoid
        self.side = 0   # 障害物をよけている側（choose_heading）
        self.heading = None  # 地図用：いま進もうとしている向き（オドメトリ座標の角度）
        self.lock = threading.Lock()
        self.halt = threading.Event()
        self.owner = None
        self.thread = None
        self.fault = None
        self.message = ""
        self.velocity = (0.0, 0.0)
        self.walking = False

    def start(self, owner):
        with self.lock:
            if self.owner is not None:
                raise RuntimeError("追尾を開始済みか、前回の解除を確認できていません")
            check_robot(self.link)
            if self.positions.measurement() is None:
                raise RuntimeError("UWB リモコンの測位が届いていません")
            if owner.is_set():
                raise RuntimeError("ブラウザとの通信が途切れました")
            self.owner = owner
            self.fault = None
            self.message = "追尾を始めます"
            self.velocity = (0.0, 0.0)
            self.walking = False
            self.side = 0
            self.halt.clear()
            self.thread = threading.Thread(target=self._run, name="go2-walk-to-uwb", daemon=True)
            self.thread.start()

    def _run(self):
        try:
            while not self.halt.wait(CONTROL_PERIOD):
                self._tick(CONTROL_PERIOD)
        except RuntimeError as exc:
            self.fault = str(exc)
        except Exception:
            log.exception("追尾の制御で異常が起きました")
            self.fault = "追尾の制御で異常が起きました"
        if self.fault is not None:
            self._send_stop()

    def _tick(self, dt):
        check_robot(self.link)
        measured = self.positions.measurement()
        if measured is None:
            self.heading = None
            target = (0.0, 0.0)
            self.walking = False
            self.message = "UWB リモコンを見失いました。止まって待っています"
        else:
            distance, bearing = measured
            target = self._steer(distance, bearing)
        previous = self.velocity
        self.velocity = (_ramp_up(previous[0], target[0], self.cfg.acc * dt),
                         _ramp_up(previous[1], target[1], self.cfg.turn_acc * dt))
        if self.velocity != (0.0, 0.0):
            self.link.send(Command(MOVE, {"x": round(self.velocity[0], 3), "y": 0.0,
                                          "z": round(self.velocity[1], 3)}))
        elif previous != (0.0, 0.0):
            self.link.send(STOP)

    def _steer(self, distance, bearing):
        """リモコンの距離・方向から (前進, 旋回) を決め、画面の文を書く。障害物はよけて回り込む。"""
        self.heading = None
        blocked = self.link.reading(time.time()).front_blocked
        forward, turn, walking = walk_velocity(distance, bearing, blocked, self.walking, self.cfg)
        if not walking:
            self.walking = False
            self.side = 0
            self.message = f"リモコンのそばで待っています（{distance:.1f} m）"
            return forward, turn
        travel = distance - self.cfg.stop_distance
        heading, free, self.side = choose_heading(self.positions.obstacles(), bearing, travel,
                                                  self.side, self.avoid)
        if heading is None:
            self.walking = True
            self.message = f"回り込める道が見つかりません。止まっています（リモコンまで {distance:.1f} m）"
            return 0.0, 0.0
        forward, turn, self.walking = walk_velocity(distance, heading, blocked, True, self.cfg)
        pose = self.link.pose()
        if pose is not None:
            self.heading = pose[2] + heading
        # 空きが少ないほど遅く進む（空きの手前 0.1 m で止まる速さ）
        forward = min(forward, max(0.0, self.cfg.speed_gain * (free - 0.1)))
        if blocked:
            self.message = f"前に障害物があるため、向きを変えています（リモコンまで {distance:.1f} m）"
        elif self.side:
            self.message = (f"障害物を{'左' if self.side > 0 else '右'}によけて、"
                            f"リモコンへ向かっています（{distance:.1f} m）")
        else:
            self.message = f"リモコンへ歩いています（{distance:.1f} m）"
        return forward, turn

    def _send_stop(self):
        self.velocity = (0.0, 0.0)
        try:
            self.link.send(STOP)
        except Exception:
            log.exception("Sport.StopMove の送信に失敗しました")
            raise RuntimeError(STOP_FAILED) from None

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

    def stop(self, owner=None):
        with self.lock:
            if self.owner is None or (owner is not None and self.owner is not owner):
                return False
            self.halt.set()
            if self.thread is not None:
                self.thread.join(timeout=2)
            self.owner = None
            self.heading = None
            self._send_stop()
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
    """シミュレーションだけの操作：Go2 を立たせる・にせ UWB リモコンを動かす。"""

    def __init__(self, link, tag, track):
        self.link = link
        self.tag = tag
        self.track = track
        self.lock = threading.Lock()

    def stand(self):
        if self.track.owner is not None:
            raise RuntimeError("追尾中は立ち上がりを送りません")
        if not self.lock.acquire(blocking=False):
            raise RuntimeError("立ち上がりの途中です")
        try:
            self.link.send(Command(STAND_UP))
            time.sleep(3.0)
            self.link.send(Command(BALANCE_STAND))
        finally:
            self.lock.release()

    def place(self, x, y):
        return self.tag.place(x, y)


def create_follow_app(track, page_port, positions, control="utrack", sim=None, camera=None):
    app = FastAPI(docs_url=None, redoc_url=None, openapi_url=None)
    app.add_middleware(LocalOnly, page_port=page_port)
    app.mount("/static", StaticFiles(directory=STATIC), name="static")

    @app.get("/")
    def index():
        return FileResponse(STATIC / "follow.html", headers={"Cache-Control": "no-store"})

    @app.get("/api/config")
    def config():
        return {"control": control, "sim": sim is not None, "camera": camera is not None}

    @app.get("/api/camera")
    def camera_image():
        frame = camera.latest() if camera is not None else None
        if frame is None:
            return HttpResponse(status_code=204, headers={"Cache-Control": "no-store"})
        _, kind, data = frame
        return HttpResponse(content=data, media_type=kind, headers={"Cache-Control": "no-store"})

    if sim is not None:
        @app.post("/api/sim/uwb")
        def place_uwb(body: dict):
            try:
                return {"position": sim.place(body.get("x"), body.get("y"))}
            except (ValueError, TypeError) as exc:
                return JSONResponse({"message": str(exc)}, status_code=400)

        @app.post("/api/sim/stand")
        def stand():
            try:
                sim.stand()
            except RuntimeError as exc:
                return JSONResponse({"message": str(exc)}, status_code=409)
            return {"message": "立ち上がりを送りました"}

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
            track.stop()
        except RuntimeError as exc:
            return JSONResponse({"message": str(exc)}, status_code=503)
        return {"message": STOPPED}

    @app.websocket("/ws/follow")
    async def follow(socket: WebSocket):
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
            await run_in_threadpool(track.start, owner)
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
        control = follow_control(os.environ)
        simulating = sim_mode(os.environ)
        link = RosLink(perception_config())
    except (ValueError, RuntimeError) as exc:
        raise SystemExit(str(exc)) from None
    try:
        link.wait_ready(wait_s)
        with (closing(FollowPositions(simulate_uwb=simulating,
                                      perception=perception_config() if control == "own" else None)) as positions,
              closing(SpinningLink(link)) as spinning):
            if control == "own":
                # 実機（障害物の判断が Go2 本体）では、Move を本体回避サービス経由で送る。
                link.prepare_onboard_avoidance()
                track = WalkToUwb(spinning, positions)
                avoidance = None
            else:
                avoidance = OnboardAvoidance()
                track = UTrack(spinning, avoidance)
            sim = SimTools(spinning, positions.fake.tag, track) if simulating else None
            camera = SimCamera(positions.node) if simulating else VideoHubCamera()
            try:
                with closing(track), closing(camera):
                    log.info("UWB 追尾のページ: http://localhost:%d（%s）", page_port,
                             "このプログラムの制御" if control == "own" else "Go2 本体の UTrack")
                    uvicorn.run(create_follow_app(track, page_port, positions, control, sim, camera),
                                host=host, port=port, timeout_graceful_shutdown=2)
            finally:
                if avoidance is not None:
                    avoidance.close()
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from None
    except KeyboardInterrupt:
        pass
    finally:
        link.close(stop_times=3 if control == "own" else 0)


if __name__ == "__main__":
    main()
