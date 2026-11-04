"""The Web page and its API (design §8), and main(): ROS 2 + Session + the 10 Hz loop.

- GET /                     the page (static/index.html)
- POST /api/audio           body = recorded audio -> Session.submit_audio, 202
- POST /api/text            {"text": str} -> Session.submit_text, 202 (typed instructions)
- POST /api/stop            Session.stop, 204
- POST /api/obstacle        {"cmd": "front" | "back" | "clear"}; 404 when not simulating
- GET /api/diagram/X.svg    X = PLANT | SPEC | TRIMMED | SUP; 404 until it is drawn
- WS /ws                    Session.snapshot() every 0.2 s, plus "diagram_names" and, when
                            simulating, "world" (SimControl.latest_world())

Run in the container: python3 -m voice_ltl.server (compose passes env.txt as variables).
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
DIAGRAM_NAMES = ("PLANT", "SPEC", "TRIMMED", "SUP")
WS_PERIOD = 0.2       # [s] between two snapshots on /ws
CONTROL_PERIOD = 0.1  # [s] the control loop (10 Hz)
PORT = 8000
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


def _serve() -> None:
    from go2_nl_control.perception import PerceptionConfig

    from voice_ltl.ros_link import RosLink, SimControl
    from voice_ltl.stt import Transcriber

    simulating = sim_mode(os.environ)
    host = "0.0.0.0" if simulating else "127.0.0.1"
    try:
        link = RosLink(PerceptionConfig())
    except RuntimeError as exc:  # already running
        _logger.error("%s", exc)
        raise SystemExit(1) from None
    stop = threading.Event()
    loop: threading.Thread | None = None
    try:
        sim = SimControl(link.node) if simulating else None
        transcriber = Transcriber((os.environ.get("WHISPER_MODEL") or "").strip() or "small")
        session = Session(link, make_llm(os.environ), transcriber, workdir=WORKDIR)
        try:
            link.wait_ready()
        except RuntimeError as exc:  # Go2 not found, or another program sends commands
            _logger.error("%s", exc)
            raise SystemExit(1) from None
        loop = threading.Thread(target=control_loop, args=(link, session, stop),
                                name="control-loop", daemon=True)
        loop.start()
        _logger.info("ページ: http://localhost:%d （%s）", PORT,
                     "シミュレーション" if simulating else "実機")
        # A short graceful shutdown, so StopMove x3 goes out well before docker's SIGKILL.
        uvicorn.run(create_app(session, sim), host=host, port=PORT, timeout_graceful_shutdown=2)
    finally:
        stop.set()
        if loop is not None:
            loop.join(timeout=2.0)  # no tick may send a command after close's StopMoves
        link.close(3)


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    try:
        _serve()
    except KeyboardInterrupt:  # Ctrl+C / SIGTERM / SIGHUP: close(3) has already run
        pass


if __name__ == "__main__":
    main()
