"""Go2 本体の UWB 追尾専用ページ。通常の動作指示は扱わない。"""

import asyncio
import json
import logging
import math
import os
import threading
import time
from contextlib import closing

import rclpy
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from starlette.concurrency import run_in_threadpool
from unitree_api.msg import Request, Response
from unitree_go.msg import SportModeState, UwbState

from go2_nl_control.actions import STANDING
from go2_nl_control.executor import STOP
from go2_nl_control.supervisor import SENSOR_LOST
from voice_ltl.onboard_avoidance import OnboardAvoidance
from voice_ltl.perception_config import perception_config
from voice_ltl.ros_link import STATE_TIMEOUT, STATE_TOPIC, RosLink
from voice_ltl.server import STATIC, LocalOnly, listen_address, ui_port, wait_ready_seconds

REQUEST_TOPIC = "/api/uwbswitch/request"
RESPONSE_TOPIC = "/api/uwbswitch/response"
UWB_STATE_TOPIC = "/uwbstate"
UWB_STATE_TIMEOUT = 1.0
SWITCH_SET, SWITCH_GET, IS_TRACKING = 1001, 1002, 1003
REPLY_TIMEOUT = 1.0
HEARTBEAT_TIMEOUT = 3.0
STOPPED = "この画面からの追尾は停止しています"
STOP_FAILED = "Go2 の追尾解除を確認できません。リモコンで停止してください"
log = logging.getLogger(__name__)


class FollowPositions:
    """追尾開始前にも見られる、位置表示専用の購読。"""

    def __init__(self):
        self.pose = None
        self.uwb = None
        self.node = rclpy.create_node("go2_follow_positions")
        self.node.create_subscription(SportModeState, STATE_TOPIC, self.on_pose, qos_profile_sensor_data)
        self.node.create_subscription(UwbState, UWB_STATE_TOPIC, self.on_uwb, qos_profile_sensor_data)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, name="go2-follow-positions", daemon=True)
        self.thread.start()

    def on_pose(self, msg):
        pose = (float(msg.position[0]), float(msg.position[1]), float(msg.imu_state.rpy[2]))
        self.pose = (time.monotonic(), pose) if all(map(math.isfinite, pose)) else None

    def on_uwb(self, msg):
        now = time.monotonic()
        self.uwb = (now, float(msg.distance_est), float(msg.orientation_est),
                    int(msg.channel), int(msg.error_state), self._recent_pose(now))

    def _recent_pose(self, now):
        pose = self.pose
        return pose[1] if pose and now - pose[0] <= STATE_TIMEOUT else None

    def snapshot(self):
        now = time.monotonic()
        pose = self._recent_pose(now)
        result = {"go2": pose, "uwb": None}
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
        self.link.spin_once()
        if self.link.reading(time.time()).front == SENSOR_LOST:
            raise RuntimeError("Go2 の状態が届いていません")
        if self.link.posture() != STANDING:
            raise RuntimeError("Go2 を立たせてから追尾を開始してください")

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


def create_follow_app(track, page_port, positions):
    app = FastAPI(docs_url=None, redoc_url=None, openapi_url=None)
    app.add_middleware(LocalOnly, page_port=page_port)
    app.mount("/static", StaticFiles(directory=STATIC), name="static")

    @app.get("/")
    def index():
        return FileResponse(STATIC / "follow.html", headers={"Cache-Control": "no-store"})

    @app.get("/api/position")
    def position():
        return JSONResponse(positions.snapshot(), headers={"Cache-Control": "no-store"})

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
        link = RosLink(perception_config())
    except (ValueError, RuntimeError) as exc:
        raise SystemExit(str(exc)) from None
    try:
        link.wait_ready(wait_s)
        with (closing(OnboardAvoidance()) as avoidance,
              closing(FollowPositions()) as positions,
              closing(UTrack(link, avoidance)) as track):
            log.info("UWB 追尾のページ: http://localhost:%d", page_port)
            uvicorn.run(create_follow_app(track, page_port, positions), host=host, port=port)
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from None
    except KeyboardInterrupt:
        pass
    finally:
        link.close(stop_times=0)


if __name__ == "__main__":
    main()
