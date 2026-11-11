"""The Web page and its API (design §8), and main(): ROS 2 + Session + the 10 Hz loop.

- GET /                     the page (static/index.html)
- POST /api/audio           body = recorded audio -> Session.submit_audio, 202; empty body 400,
                            over MAX_AUDIO_BYTES 413
- POST /api/text            {"text": str} -> Session.submit_text, 202 (typed instructions)
- POST /api/stop            Session.stop, 204
- POST /api/obstacle        {"cmd": "front" | "back" | "clear"}; 404 when not simulating
- GET /api/diagram/X.svg    X = FOCUS | SUP | TRIMMED | SPEC | PLANT; 404 until it is drawn
- WS /ws                    page_snapshot() every 0.2 s: Session.snapshot() plus
                            "message_kind", "diagram_names" and, when simulating, "world"

Only the page itself may use it (LocalOnly): every request needs the Host the browser uses for
the page, localhost:<GO2_UI_PORT> or 127.0.0.1:<GO2_UI_PORT> (DNS rebinding cannot pass), and
a POST and the WebSocket also need that page as their Origin (another site cannot send). Anything
else gets 403. A tool such as curl sends -H 'Origin: http://localhost:8810'.

Run in the container: python3 -m voice_ltl.server (compose passes env.txt as variables).
The page is http://localhost:8810 on the host; GO2_UI_PORT changes the port (listen_address).
It logs the safety caution of design §5 (SAFETY_CAUTION) on start-up, and the page shows it.

What the page says it is connected to (link_kind, snapshot "link_kind"): SIM=0 is the real Go2
(「実機」). With SIM=1, SIM_KIND names the simulator: fake (the default; compose sets it) is the
fake Go2 (`docker compose --profile fake up`), gazebo is Gazebo, started as
`SIM_KIND=gazebo docker compose --profile gazebo up`. Both send the same topics, so only
SIM_KIND tells them apart.
"""

import asyncio
import logging
import os
import re
import threading
import time
from pathlib import Path
from typing import TYPE_CHECKING, Literal, Mapping

import uvicorn
from fastapi import FastAPI, HTTPException, Request, Response, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, PlainTextResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from starlette.concurrency import run_in_threadpool
from starlette.datastructures import Headers
from starlette.types import ASGIApp, Receive, Scope, Send

from voice_ltl.llm import MSG_LOGIN, make_llm
from voice_ltl.objects import MSG_POSITION_DRIFT
from voice_ltl.session import (MSG_BUSY, MSG_DONE, MSG_EMPTY, MSG_NO_AUDIO, MSG_OTHER_SENDERS,
                               MSG_POSTURE_CHANGED, MSG_SETTLING, MSG_STOPPED, MSG_UNSATISFIABLE, MSG_WAITING, Session)

if TYPE_CHECKING:
    from voice_ltl.ros_link import RosLink, SimControl

_logger = logging.getLogger(__name__)

STATIC = Path(__file__).resolve().parent / "static"
DIAGRAM_NAMES = ("FOCUS", "SUP", "TRIMMED", "SPEC", "PLANT")
SIM_KINDS = ("fake", "gazebo")
WS_PERIOD = 0.2       # [s] between two snapshots on /ws
CONTROL_PERIOD = 0.1  # [s] the control loop (10 Hz)
UI_PORT = 8810        # the page's port on the host (GO2_UI_PORT, R25)
CONTAINER_PORT = 8000  # inside the container when simulating; compose publishes it as UI_PORT
WORKDIR = Path(__file__).resolve().parents[1] / "out"
MAX_AUDIO_BYTES = 10 * 1024 * 1024  # [byte] the largest recording /api/audio takes (SEC-2): 413
# Design §5: shown before motion (README and docs/real_robot.md 「安全の注意」). Logged on start-up; the page has it too.
SAFETY_CAUTION = "前方 1 m と周りを空けてください。リモコンを持った人が、必ずそばにいてください。"
REFUSED = "このページの外からの操作は受け付けません"
_PAGE_HOST = re.compile(r"(?:localhost|127\.0\.0\.1)(?::([0-9]{1,5}))?", re.IGNORECASE | re.ASCII)


class TextIn(BaseModel):
    text: str


class ObstacleIn(BaseModel):
    cmd: Literal["front", "back", "clear"]


def page_host(host: str | None, page_port: int) -> bool:
    """host (a Host header, "localhost:8810") names the page: localhost or 127.0.0.1 on its port."""
    m = _PAGE_HOST.fullmatch(host or "")
    return m is not None and int(m[1] or 80) == page_port


def page_origin(origin: str | None, page_port: int) -> bool:
    """origin (an Origin header, "http://localhost:8810") is the page itself."""
    scheme, sep, host = (origin or "").partition("://")
    return scheme == "http" and sep == "://" and page_host(host, page_port)


class LocalOnly:
    """SEC-1: only the page itself. Every request needs the page's Host (no DNS rebinding); a
    request that is not GET/HEAD, and the WebSocket, also need the page's Origin (no other site,
    and no Origin at all, as a browser always sends one there). Anything else gets 403."""

    def __init__(self, app: ASGIApp, page_port: int):
        self.app = app
        self.page_port = page_port

    async def __call__(self, scope: Scope, receive: Receive, send: Send) -> None:
        if scope["type"] not in ("http", "websocket") or self._allowed(scope):
            await self.app(scope, receive, send)
        elif scope["type"] == "http":
            await PlainTextResponse(REFUSED, status_code=403)(scope, receive, send)
        elif "websocket.http.response" in scope.get("extensions", {}):
            await WebSocket(scope, receive, send).send_denial_response(
                PlainTextResponse(REFUSED, status_code=403))
        else:  # a close before accept: uvicorn answers the handshake with 403
            await WebSocket(scope, receive, send).close(code=1008)

    def _allowed(self, scope: Scope) -> bool:
        headers = Headers(scope=scope)
        if not page_host(headers.get("host"), self.page_port):
            return False
        if scope["type"] == "http" and scope["method"] in ("GET", "HEAD"):
            return True
        return page_origin(headers.get("origin"), self.page_port)


def message_kind(status: str, message: str) -> str | None:
    """The kind of notice the page shows for the snapshot's message (design §8 お知らせ):
    "stopped", "done", "unsat", "login", "unheard" (聞き取れなかった; the plan goes on), "busy"
    (R27/R28: the instruction starts by itself), "waiting", "error", "info"; None for no message.
    Decided from Session's own messages first, so that 聞き取れなかった after an error is not an
    error, then from the status. MSG_UNSATISFIABLE may carry a reason after it (R15:
    NO_PATH_REASON); MSG_POSITION_DRIFT is an error whatever the status (R11)."""
    if not message:
        return None
    if message == MSG_STOPPED:
        return "stopped"
    if message in (MSG_EMPTY, MSG_NO_AUDIO):
        return "unheard"
    if message in (MSG_BUSY, MSG_SETTLING):
        return "busy"
    if message == MSG_WAITING:
        return "waiting"
    if message == MSG_DONE and status == "done":
        return "done"
    if message.startswith(MSG_UNSATISFIABLE):
        return "unsat"
    if message.startswith(MSG_LOGIN):
        return "login"
    # stopped or refused whatever the status: the page says 続けられませんでした after a plan began
    if message in (MSG_OTHER_SENDERS, MSG_POSTURE_CHANGED, MSG_POSITION_DRIFT):
        return "error"
    return "error" if status == "error" else "info"


def page_snapshot(session: Session, sim: "SimControl | None") -> dict:
    """What /ws sends: Session.snapshot(), the kind of notice for its message, which diagram
    tabs have a picture and, when simulating, the world (SimControl.latest_world())."""
    snap = session.snapshot()
    snap["message_kind"] = message_kind(snap["status"], snap["message"])
    snap["diagram_names"] = list(session.diagrams)
    if sim is not None:
        snap["world"] = sim.latest_world()
    return snap


def create_app(session: Session, sim: "SimControl | None", page_port: int = UI_PORT) -> FastAPI:
    """page_port: the port in the browser's address of the page (GO2_UI_PORT), which is not
    the port the server listens on in the container."""
    app = FastAPI(title="Go2 voice LTL", docs_url=None, redoc_url=None, openapi_url=None)
    app.add_middleware(LocalOnly, page_port=page_port)
    app.mount("/static", StaticFiles(directory=STATIC), name="static")

    @app.get("/")
    def index() -> FileResponse:
        return FileResponse(STATIC / "index.html", headers={"Cache-Control": "no-store"})

    @app.post("/api/audio", status_code=202)
    async def audio(request: Request) -> Response:
        too_big = HTTPException(413, f"音声が大きすぎます（{MAX_AUDIO_BYTES // 2**20} MB まで）")
        declared = request.headers.get("content-length", "")
        if declared.isdigit() and int(declared) > MAX_AUDIO_BYTES:
            raise too_big
        body = bytearray()
        async for chunk in request.stream():  # counted as it comes: a chunked body has no length
            body += chunk
            if len(body) > MAX_AUDIO_BYTES:
                raise too_big
        if not body:  # the page does not send a short tap (UI-3)
            raise HTTPException(400, "音声が空です")
        await run_in_threadpool(session.submit_audio, bytes(body))
        return Response(status_code=202)

    @app.post("/api/text", status_code=202)
    def text(req: TextIn) -> Response:
        session.submit_text(req.text)
        return Response(status_code=202)

    @app.post("/api/stop", status_code=204)
    def stop() -> Response:
        session.stop()
        return Response(status_code=204)

    @app.post("/api/obstacle", status_code=204)
    def obstacle(req: ObstacleIn) -> Response:
        if sim is None:
            raise HTTPException(404, "シミュレーションではないので、箱は置けません")
        sim.obstacle(req.cmd)
        return Response(status_code=204)

    @app.get("/api/diagram/{name}.svg")
    def diagram(name: str) -> Response:
        svg = session.diagrams.get(name) if name in DIAGRAM_NAMES else None
        if svg is None:
            raise HTTPException(404, "この図はまだありません")
        return Response(svg, media_type="image/svg+xml", headers={"Cache-Control": "no-store"})

    @app.websocket("/ws")
    async def ws(websocket: WebSocket) -> None:
        await websocket.accept()  # LocalOnly has checked its Host and Origin
        try:
            while True:
                await websocket.send_json(page_snapshot(session, sim))
                await asyncio.sleep(WS_PERIOD)
        except WebSocketDisconnect:
            pass

    return app


def control_loop(link: "RosLink", session: Session, stop: threading.Event,
                 period: float = CONTROL_PERIOD) -> None:
    """Every period: handle what ROS 2 received, then one Session.tick. Ends when stop is set."""
    failing = False
    next_t = time.monotonic()
    while not stop.is_set():
        try:
            link.spin_once()
            session.tick(time.time())
            failing = False
        except Exception:
            _logger.exception("control loop failed")
            if not failing:  # once per run of failures: a broken tick must not leave the Go2 moving
                failing = True
                try:
                    session.stop()
                except Exception:
                    _logger.exception("StopMove after a failed tick failed")
        next_t += period
        delay = next_t - time.monotonic()
        if delay < 0:  # fell behind: start counting again from now
            next_t, delay = time.monotonic(), 0.0
        stop.wait(delay)


def sim_mode(env: Mapping[str, str]) -> bool:
    """SIM=1 (compose): simulation. SIM=0, empty or unset: a real Go2 (the safe side)."""
    return (env.get("SIM") or "0").strip() != "0"


def ui_port(env: Mapping[str, str]) -> int:
    """The page's port on the host: GO2_UI_PORT, 8810 when empty or unset. ValueError if bad."""
    raw = (env.get("GO2_UI_PORT") or "").strip()
    if not raw:
        return UI_PORT
    port = int(raw) if raw.isdecimal() else 0
    if not 1 <= port <= 65535:
        raise ValueError(f"GO2_UI_PORT は 1〜65535 の数にしてください（今: {raw!r}）")
    return port


def link_kind(env: Mapping[str, str]) -> str:
    """What the page says it is connected to: "robot" when not simulating (SIM=0), else
    SIM_KIND: "fake" (the fake Go2; empty or unset) or "gazebo". ValueError for anything else."""
    if not sim_mode(env):
        return "robot"
    raw = env.get("SIM_KIND") or ""
    kind = raw.strip().lower() or "fake"
    if kind not in SIM_KINDS:
        raise ValueError(f"SIM_KIND は fake か gazebo にしてください（今: {raw!r}）")
    return kind


def listen_address(env: Mapping[str, str]) -> tuple[str, int]:
    """Where uvicorn listens. SIM=1: in the container on 0.0.0.0:8000, which compose publishes
    as 127.0.0.1:GO2_UI_PORT. Otherwise the host network (a real Go2): 127.0.0.1:GO2_UI_PORT,
    so only the PC itself opens the page."""
    if sim_mode(env) and env.get("GO2_NATIVE") != "1":
        return "0.0.0.0", CONTAINER_PORT
    return "127.0.0.1", ui_port(env)


WAIT_READY_S = 5.0          # [s] how long main() waits for the Go2 (GO2_WAIT_READY_S, R38)
WAIT_READY_GAZEBO_S = 90.0  # [s] the same for Gazebo, which can take 20-40 s to start


def wait_ready_seconds(env: Mapping[str, str]) -> float:
    """How long to wait for the Go2 at start-up [s]: GO2_WAIT_READY_S; when empty or unset, 90 with
    SIM=1 and SIM_KIND=gazebo (one `docker compose --profile gazebo up` starts the app and Gazebo
    together), else 5. run_formula uses it too. ValueError unless a positive number."""
    raw = (env.get("GO2_WAIT_READY_S") or "").strip()
    if not raw:
        return WAIT_READY_GAZEBO_S if link_kind(env) == "gazebo" else WAIT_READY_S
    try:
        seconds = float(raw)
    except ValueError:
        seconds = 0.0
    if not 0.0 < seconds < float("inf"):  # also nan
        raise ValueError(f"GO2_WAIT_READY_S は 0 より大きい秒数にしてください（今: {raw!r}）")
    return seconds


def _serve() -> None:
    from voice_ltl.perception_config import perception_config

    from voice_ltl.ros_link import RosLink, SimControl
    from voice_ltl.stt import Transcriber

    simulating = sim_mode(os.environ)
    try:
        page_port = ui_port(os.environ)
        host, port = listen_address(os.environ)
        kind = link_kind(os.environ)
        wait_s = wait_ready_seconds(os.environ)
    except ValueError as exc:
        _logger.error("%s", exc)
        raise SystemExit(1) from None
    try:
        link = RosLink(perception_config())
    except RuntimeError as exc:  # already running
        _logger.error("%s", exc)
        raise SystemExit(1) from None
    stop = threading.Event()
    loop: threading.Thread | None = None
    stop_times = 3  # StopMove x3 on the way out
    try:
        sim = SimControl(link.node) if simulating else None
        transcriber = Transcriber((os.environ.get("WHISPER_MODEL") or "").strip() or "small")
        session = Session(link, make_llm(os.environ), transcriber, workdir=WORKDIR, link_kind=kind)
        _logger.info("Go2 を待ちます（最長 %g 秒。GO2_WAIT_READY_S で変えられます）", wait_s)
        try:
            link.wait_ready(wait_s)
            prepare = getattr(link, "prepare_onboard_avoidance", None)
            if callable(prepare):
                prepare()
        except RuntimeError as exc:  # Go2 not found, or another program sends commands
            _logger.error("%s", exc)
            if link.other_senders():  # that program drives the Go2: do not cut in with StopMove
                stop_times = 0
            raise SystemExit(1) from None
        loop = threading.Thread(target=control_loop, args=(link, session, stop),
                                name="control-loop", daemon=True)
        loop.start()
        if simulating:
            _logger.info("安全の注意（実機に移したとき）: %s", SAFETY_CAUTION)
        else:
            _logger.warning("安全の注意（実機）: %s", SAFETY_CAUTION)
        _logger.info("ページ: http://localhost:%d （%s）", page_port,
                     "シミュレーション" if simulating else "実機")
        # A short graceful shutdown, so StopMove x3 goes out well before docker's SIGKILL.
        uvicorn.run(create_app(session, sim, page_port), host=host, port=port,
                    timeout_graceful_shutdown=2)
    finally:
        stop.set()
        if loop is not None:
            loop.join(timeout=2.0)  # no tick may send a command after close's StopMoves
        link.close(stop_times)


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    try:
        _serve()
    except KeyboardInterrupt:  # Ctrl+C / SIGTERM / SIGHUP: close(3) has already run
        pass


if __name__ == "__main__":
    main()
