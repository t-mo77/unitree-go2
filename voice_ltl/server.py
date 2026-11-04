"""The Web page and its API (design §8), and main(): ROS 2 + Session + the 10 Hz loop.

- GET /                     the page (static/index.html)
- POST /api/audio           body = recorded audio -> Session.submit_audio, 202; empty body 400
- POST /api/text            {"text": str} -> Session.submit_text, 202 (typed instructions)
- POST /api/stop            Session.stop, 204
- POST /api/obstacle        {"cmd": "front" | "back" | "clear"}; 404 when not simulating
- GET /api/diagram/X.svg    X = FOCUS | SUP | TRIMMED | SPEC | PLANT; 404 until it is drawn
- WS /ws                    Session.snapshot() every 0.2 s, plus "diagram_names" and, when
                            simulating, "world" (SimControl.latest_world())

Run in the container: python3 -m voice_ltl.server (compose passes env.txt as variables).
The page is http://localhost:8810 on the host; GO2_UI_PORT changes the port (listen_address).

What the page says it is connected to (link_kind, snapshot "link_kind"): SIM=0 is the real Go2
(「実機」). With SIM=1, SIM_KIND names the simulator: fake (the default; compose sets it) is the
fake Go2 (`docker compose --profile fake up`), gazebo is Gazebo, started as
`SIM_KIND=gazebo docker compose --profile gazebo up`. Both send the same topics, so only
SIM_KIND tells them apart.
"""

import asyncio
import logging
import os
import threading
import time
from pathlib import Path
from typing import TYPE_CHECKING, Literal, Mapping

import uvicorn
from fastapi import FastAPI, HTTPException, Request, Response, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from starlette.concurrency import run_in_threadpool

from voice_ltl.llm import make_llm
from voice_ltl.session import Session

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
WORKDIR = Path("/work/out")


class TextIn(BaseModel):
    text: str


class ObstacleIn(BaseModel):
    cmd: Literal["front", "back", "clear"]


def create_app(session: Session, sim: "SimControl | None") -> FastAPI:
    app = FastAPI(title="Go2 voice LTL", docs_url=None, redoc_url=None, openapi_url=None)
    app.mount("/static", StaticFiles(directory=STATIC), name="static")

    @app.get("/")
    def index() -> FileResponse:
        return FileResponse(STATIC / "index.html", headers={"Cache-Control": "no-store"})

    @app.post("/api/audio", status_code=202)
    async def audio(request: Request) -> Response:
        body = await request.body()
        if not body:  # a short tap records nothing; the page says so and does not send it
            raise HTTPException(400, "音声が空です")
        await run_in_threadpool(session.submit_audio, body)
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
        await websocket.accept()
        try:
            while True:
                snap = session.snapshot()
                snap["diagram_names"] = list(session.diagrams)  # which tabs have a picture
                if sim is not None:
                    snap["world"] = sim.latest_world()
                await websocket.send_json(snap)
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
    if sim_mode(env):
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
    from go2_nl_control.perception import PerceptionConfig

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
        link = RosLink(PerceptionConfig())
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
        except RuntimeError as exc:  # Go2 not found, or another program sends commands
            _logger.error("%s", exc)
            if link.other_senders():  # that program drives the Go2: do not cut in with StopMove
                stop_times = 0
            raise SystemExit(1) from None
        loop = threading.Thread(target=control_loop, args=(link, session, stop),
                                name="control-loop", daemon=True)
        loop.start()
        _logger.info("ページ: http://localhost:%d （%s）", page_port,
                     "シミュレーション" if simulating else "実機")
        # A short graceful shutdown, so StopMove x3 goes out well before docker's SIGKILL.
        uvicorn.run(create_app(session, sim), host=host, port=port, timeout_graceful_shutdown=2)
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
