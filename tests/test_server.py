import json
import logging
import re
import shutil
import subprocess
import sys
import threading
import time
import types

import pytest
from fastapi.testclient import TestClient
from go2_nl_control.actions import STOP_MOVE
from starlette.testclient import WebSocketDenialResponse

import voice_ltl.server as server
from sim.sim_core import SimGo2
from voice_ltl.llm import MSG_LOGIN, FakeLLM
from voice_ltl.server import control_loop, create_app
from voice_ltl.session import (MSG_BUSY, MSG_DONE, MSG_EMPTY, MSG_NO_AUDIO, MSG_POSTURE_UNKNOWN,
                               MSG_SETTLING, MSG_STOPPED, MSG_UNSATISFIABLE, MSG_WAITING,
                               DirectSimLink, Session)
import voice_ltl.session as session_module

PAGE = "http://localhost:8810"          # the page as the browser opens it (GO2_UI_PORT 8810)
WS = "ws://localhost:8810/ws"
AUDIO = {"content-type": "audio/webm"}


def make_client(tmp_path, origin=PAGE, heard=None):
    """A client of the page at PAGE. Its requests carry Origin `origin` (None: no Origin).
    heard, a list, gets the size of each recording the session transcribes."""
    def transcribe(audio):
        if heard is not None:
            heard.append(len(audio))
        return "立って挨拶して"

    sess = Session(DirectSimLink(SimGo2()), FakeLLM([{"formula": "F hello", "unsupported": []}]),
                   transcribe, tmp_path, background=False)
    headers = {"origin": origin} if origin is not None else {}
    return TestClient(create_app(sess, sim=None, page_port=8810), base_url=PAGE, headers=headers), sess


def test_text_endpoint_runs_pipeline(tmp_path):
    c, sess = make_client(tmp_path)
    assert c.post("/api/text", json={"text": "立って挨拶して"}).status_code == 202
    sess.tick(0.0); assert sess.snapshot()["formula"] == "F hello"
    for name in ("FOCUS", "SUP", "TRIMMED", "SPEC", "PLANT"):
        assert c.get(f"/api/diagram/{name}.svg").headers["content-type"].startswith("image/svg")


def test_audio_and_stop_and_obstacle_404(tmp_path):
    c, sess = make_client(tmp_path)
    assert c.post("/api/audio", content=b"RIFF....", headers={"content-type": "audio/wav"}).status_code == 202
    assert c.post("/api/stop").status_code == 204
    assert c.post("/api/obstacle", json={"cmd": "front"}).status_code == 404


def test_empty_audio_is_refused(tmp_path):
    c, sess = make_client(tmp_path)
    assert c.post("/api/audio", content=b"", headers={"content-type": "audio/webm"}).status_code == 400
    snap = sess.snapshot()
    assert snap["status"] == "idle" and snap["transcript"] == "" and snap["log"] == []


def test_ws_sends_snapshot(tmp_path):
    c, _ = make_client(tmp_path)
    with c.websocket_connect(WS) as ws:
        snap = ws.receive_json()
    assert {"status", "log", "debug_log", "steps", "link_kind", "stt", "llm_backend",
            "waiting_since", "server_now", "block_timeout", "stop_word", "message_kind"} <= set(snap)


def test_index_has_push_to_talk(tmp_path):
    c, _ = make_client(tmp_path)
    html = c.get("/").text
    assert "スペース" in html and "app.js" in html


def test_index_has_the_designed_parts(tmp_path):
    """The page of the Claude Design canvas (docs/reference/ui-design): its fixed parts."""
    c, _ = make_client(tmp_path)
    html = c.get("/").text
    for text in [
        "/static/style.css", "Go2 Voice", "止める", "マイクを使う",       # top bar, talk band
        "あなたの指示", "ロジック（LTLf）", "LLM の試行", "計画",        # the cards
        "状態遷移図", 'role="tablist"', 'role="tab"', 'aria-selected="true"',
        ">計画の道<", ">監督器<", ">trim 後<", ">要求<", ">環境<",      # the diagram tabs
        "姿勢", "前方", "後方", "上から見た図",                           # the Go2 card
        "前に箱を置く", "後ろに箱を置く", "箱を全部どける",
        "<details", "記録", "詳しく",                                      # the log
        'role="status"', 'role="alert"',                                   # the messages
    ]:
        assert text in html, text


@pytest.mark.parametrize("env,address", [
    ({"SIM": "1"}, ("0.0.0.0", 8000)),                         # compose publishes it on the host
    ({"SIM": "1", "GO2_UI_PORT": "9123"}, ("0.0.0.0", 8000)),
    ({"SIM": "0"}, ("127.0.0.1", 8810)),                       # host network (a real Go2)
    ({}, ("127.0.0.1", 8810)),
    ({"SIM": "0", "GO2_UI_PORT": ""}, ("127.0.0.1", 8810)),    # empty = the default
    ({"SIM": "0", "GO2_UI_PORT": " 9123 "}, ("127.0.0.1", 9123)),
])
def test_listen_address(env, address):
    assert server.listen_address(env) == address


def test_the_focus_tab_comes_first_and_is_selected(tmp_path):
    c, _ = make_client(tmp_path)
    html = c.get("/").text
    names = [html.index(f'data-name="{n}"') for n in ("FOCUS", "SUP", "TRIMMED", "SPEC", "PLANT")]
    assert names == sorted(names)
    tab = html[html.rindex("<button", 0, names[0]):html.index(">", names[0])]
    assert 'aria-selected="true"' in tab


@pytest.mark.parametrize("env,kind", [
    ({"SIM": "1"}, "fake"),                                    # compose's default (SIM_KIND=fake)
    ({"SIM": "1", "SIM_KIND": ""}, "fake"),
    ({"SIM": "1", "SIM_KIND": "fake"}, "fake"),
    ({"SIM": "1", "SIM_KIND": " Gazebo "}, "gazebo"),
    ({"SIM": "0"}, "robot"),
    ({"SIM": "0", "SIM_KIND": "gazebo"}, "robot"),             # SIM=0 is a real Go2, whatever SIM_KIND says
    ({}, "robot"),
])
def test_link_kind(env, kind):
    assert server.link_kind(env) == kind


def test_bad_sim_kind_is_refused():
    with pytest.raises(ValueError, match="SIM_KIND"):
        server.link_kind({"SIM": "1", "SIM_KIND": "real"})


@pytest.mark.parametrize("bad", ["abc", "0", "65536", "-1", "80.5"])
def test_bad_ui_port_is_refused(bad):
    with pytest.raises(ValueError, match="GO2_UI_PORT"):
        server.listen_address({"SIM": "0", "GO2_UI_PORT": bad})


class SpinningLink(DirectSimLink):
    """DirectSimLink with RosLink's spin_once, for control_loop."""

    def __init__(self):
        super().__init__(SimGo2())
        self.spins = 0

    def spin_once(self):
        self.spins += 1


class BrokenTick(Session):
    """tick() raises on ticks 1-3 and from tick 6 on; ticks 4 and 5 work."""

    ticks = 0

    def tick(self, now):
        self.ticks += 1
        if self.ticks <= 3 or self.ticks >= 6:
            raise RuntimeError("壊れた tick")


def test_control_loop_sends_stopmove_when_tick_fails(tmp_path):
    link = SpinningLink()
    sess = BrokenTick(link, FakeLLM([]), None, tmp_path, background=False)
    stop = threading.Event()
    loop = threading.Thread(target=control_loop, args=(link, sess, stop, 0.01), daemon=True)
    loop.start()
    deadline = time.monotonic() + 5
    while sess.ticks < 10 and time.monotonic() < deadline:
        time.sleep(0.01)
    stop.set(); loop.join(2)
    assert not loop.is_alive() and sess.ticks >= 10 and link.spins >= 10
    assert link.sim.accepted == [STOP_MOVE, STOP_MOVE]     # once per run of failures
    assert sess.snapshot()["status"] == "idle"


class BrokenSpin(SpinningLink):
    """spin_once() raises every time: what ROS 2 received never reaches the session."""

    def spin_once(self):
        super().spin_once()
        raise RuntimeError("壊れた spin_once")


def test_control_loop_sends_stopmove_when_spin_fails(tmp_path):
    """TD-5: a failing link stops the plan too (StopMove once), and the loop keeps going."""
    link = BrokenSpin()
    sess = Session(link, FakeLLM([]), None, tmp_path, background=False)
    stop = threading.Event()
    loop = threading.Thread(target=control_loop, args=(link, sess, stop, 0.01), daemon=True)
    loop.start()
    deadline = time.monotonic() + 5
    while link.spins < 5 and time.monotonic() < deadline:
        time.sleep(0.01)
    stop.set(); loop.join(2)
    assert not loop.is_alive() and link.spins >= 5
    assert link.sim.accepted == [STOP_MOVE]


def test_stop_button_sends_stopmove_to_the_go2(tmp_path):
    """TD-5: /api/stop reaches the Go2 as StopMove, and the plan that ran is gone."""
    c, sess = make_client(tmp_path)
    assert c.post("/api/text", json={"text": "立って挨拶して"}).status_code == 202
    t = 0.0
    while sess.snapshot()["status"] != "executing" and t < 5:
        sess.link.advance(t); sess.tick(t); t += 0.1
    assert sess.snapshot()["status"] == "executing"
    sent = len(sess.link.sim.accepted)
    assert c.post("/api/stop").status_code == 204
    assert sess.link.sim.accepted[sent:] == [STOP_MOVE]
    snap = sess.snapshot()
    assert snap["status"] == "idle" and snap["message"] == MSG_STOPPED and snap["current"] is None


@pytest.mark.parametrize("others,stop_times", [
    (1, 0),   # another program drives the Go2: close without cutting in with StopMove
    (0, 3),   # Go2 not found: the usual StopMove x3
])
def test_refusing_to_start(monkeypatch, tmp_path, others, stop_times):
    made = []

    class FakeRosLink:
        def __init__(self, perception):
            self.node, self.closed_with = None, None
            made.append(self)

        def wait_ready(self):
            raise RuntimeError("別の送り手がいます" if others else "Go2 が見つかりません")

        def other_senders(self):
            return others

        def close(self, stop_times=3):
            self.closed_with = stop_times

    ros_link = types.ModuleType("voice_ltl.ros_link")
    ros_link.RosLink, ros_link.SimControl = FakeRosLink, None
    stt = types.ModuleType("voice_ltl.stt")
    stt.Transcriber = lambda model: (lambda audio: "")
    monkeypatch.setitem(sys.modules, "voice_ltl.ros_link", ros_link)
    monkeypatch.setitem(sys.modules, "voice_ltl.stt", stt)
    monkeypatch.setattr(server, "make_llm", lambda env: FakeLLM([]))
    monkeypatch.setattr(server, "WORKDIR", tmp_path)
    monkeypatch.setenv("SIM", "0")
    monkeypatch.delenv("GO2_UI_PORT", raising=False)
    sessions = []
    monkeypatch.setattr(server, "Session", lambda *a, **kw: sessions.append(kw) or Session(*a, **kw))
    with pytest.raises(SystemExit):
        server._serve()
    assert [link.closed_with for link in made] == [stop_times]
    assert [kw.get("link_kind") for kw in sessions] == ["robot"]   # SIM=0: the page says 実機


# ---------------- SEC-1 / UI-4: only the page itself ----------------
FOREIGN_ORIGINS = [
    "https://evil.example", "http://evil.example", "null",
    "http://localhost:8811", "http://127.0.0.1:8000",          # another port: another site
    "https://localhost:8810",                                   # the page is http
    "http://localhost:8810.evil.example", "http://evil.example/http://localhost:8810",
    "http://user@localhost:8810", "http://[::1]:8810", "http://localhost",
]
FOREIGN_HOSTS = [
    "attacker.example:8810", "attacker.example",                # DNS rebinding
    "localhost:8000", "127.0.0.1:8811", "localhost",            # not the page's port
    "127.0.0.1.nip.io:8810", "localhost.evil.example:8810", "[::1]:8810", "",
]
POSTS = [
    ("/api/audio", {"content": b"RIFF....", "headers": AUDIO}),
    ("/api/text", {"json": {"text": "立って挨拶して"}}),
    ("/api/stop", {}),
    ("/api/obstacle", {"json": {"cmd": "front"}}),
]


def untouched(sess, heard):
    snap = sess.snapshot()
    return (heard == [] and snap["transcript"] == "" and snap["log"] == []
            and snap["status"] == "idle" and sess.link.sim.accepted == [])


@pytest.mark.parametrize("path,kw", POSTS, ids=[p for p, _ in POSTS])
@pytest.mark.parametrize("origin", FOREIGN_ORIGINS + [None])
def test_post_from_another_origin_is_refused(tmp_path, path, kw, origin):
    heard = []
    c, sess = make_client(tmp_path, origin=origin, heard=heard)
    r = c.post(path, **kw)
    assert r.status_code == 403
    assert untouched(sess, heard)


@pytest.mark.parametrize("host", FOREIGN_HOSTS)
def test_another_host_is_refused(tmp_path, host):
    """DNS rebinding: a page of another site whose name now points at 127.0.0.1."""
    heard = []
    c, sess = make_client(tmp_path, heard=heard)
    for path in ("/", "/static/app.js", "/static/style.css", "/api/diagram/SUP.svg"):
        assert c.get(path, headers={"host": host}).status_code == 403, path
    for path, kw in POSTS:
        headers = {**kw.get("headers", {}), "host": host, "origin": f"http://{host}"}
        assert c.post(path, **{**kw, "headers": headers}).status_code == 403, path
    assert untouched(sess, heard)


@pytest.mark.parametrize("page", ["http://localhost:8810", "http://127.0.0.1:8810",
                                  "http://LOCALHOST:8810"])
def test_the_page_itself_is_served_by_both_local_names(tmp_path, page):
    c, sess = make_client(tmp_path, origin=page)
    c.base_url = page
    assert c.get("/").status_code == 200
    assert c.get("/static/app.js").status_code == 200
    assert c.post("/api/text", json={"text": "立って挨拶して"}).status_code == 202
    assert c.post("/api/stop").status_code == 204
    with c.websocket_connect(page.replace("http", "ws") + "/ws") as ws:
        assert ws.receive_json()["link_kind"] == "fake"


def test_the_page_port_is_go2_ui_port(tmp_path):
    """In the container the server listens on 8000, but the browser says GO2_UI_PORT."""
    sess = Session(DirectSimLink(SimGo2()), FakeLLM([]), None, tmp_path, background=False)
    c = TestClient(create_app(sess, sim=None, page_port=9123), base_url="http://127.0.0.1:9123",
                   headers={"origin": "http://127.0.0.1:9123"})
    assert c.get("/").status_code == 200 and c.post("/api/stop").status_code == 204
    assert c.get("/", headers={"host": "127.0.0.1:8810"}).status_code == 403
    assert c.post("/api/stop", headers={"origin": PAGE}).status_code == 403


@pytest.mark.parametrize("origin", FOREIGN_ORIGINS + [None])
def test_websocket_from_another_origin_is_refused(tmp_path, origin):
    c, _ = make_client(tmp_path, origin=origin)
    with pytest.raises(WebSocketDenialResponse) as refused:
        with c.websocket_connect(WS):
            pass
    assert refused.value.status_code == 403


@pytest.mark.parametrize("host", FOREIGN_HOSTS)
def test_websocket_to_another_host_is_refused(tmp_path, host):
    c, _ = make_client(tmp_path)
    with pytest.raises(WebSocketDenialResponse) as refused:
        with c.websocket_connect(WS, headers={"host": host, "origin": f"http://{host}"}):
            pass
    assert refused.value.status_code == 403


def test_a_get_from_another_site_is_not_refused_for_its_origin(tmp_path):
    """GET checks the Host only: an <img> of another site gets the picture, but cannot read it."""
    c, _ = make_client(tmp_path, origin="https://evil.example")
    assert c.get("/").status_code == 200


# ---------------- SEC-2: a limit on the recording ----------------
def test_audio_limit_is_10_mb():
    assert server.MAX_AUDIO_BYTES == 10 * 1024 * 1024


def test_audio_at_the_limit_is_taken(tmp_path):
    heard = []
    c, _ = make_client(tmp_path, heard=heard)
    assert c.post("/api/audio", content=bytes(server.MAX_AUDIO_BYTES), headers=AUDIO).status_code == 202
    assert heard == [server.MAX_AUDIO_BYTES]


def test_audio_over_the_limit_is_refused(tmp_path):
    heard = []
    c, sess = make_client(tmp_path, heard=heard)
    r = c.post("/api/audio", content=bytes(server.MAX_AUDIO_BYTES + 1), headers=AUDIO)
    assert r.status_code == 413
    assert untouched(sess, heard)


def test_chunked_audio_over_the_limit_is_refused(tmp_path):
    """No Content-Length (chunked): the server counts while it reads."""
    heard = []
    c, sess = make_client(tmp_path, heard=heard)
    chunks = iter([bytes(1024 * 1024)] * 11)
    r = c.post("/api/audio", content=chunks, headers=AUDIO)
    assert r.status_code == 413
    assert untouched(sess, heard)


def test_audio_declared_over_the_limit_is_refused_before_it_is_read(tmp_path):
    heard = []
    c, sess = make_client(tmp_path, heard=heard)
    r = c.post("/api/audio", content=b"RIFF....",
               headers={**AUDIO, "content-length": str(server.MAX_AUDIO_BYTES + 1)})
    assert r.status_code == 413
    assert untouched(sess, heard)


# ---------------- UI-1: the kind of notice for the session's message ----------------
NOTICE_KINDS = [
    # (status, message, message_kind); every MSG_* of session.py is here
    ("idle", MSG_STOPPED, "stopped"),
    ("done", MSG_DONE, "done"),
    ("error", MSG_UNSATISFIABLE, "unsat"),
    ("error", MSG_LOGIN, "login"),
    ("idle", MSG_EMPTY, "unheard"),
    ("executing", MSG_EMPTY, "unheard"),
    ("error", MSG_EMPTY, "unheard"),        # after an error, a speechless press is still 聞き取れない
    ("done", MSG_EMPTY, "unheard"),
    ("idle", MSG_NO_AUDIO, "unheard"),
    ("error", MSG_NO_AUDIO, "unheard"),
    ("thinking", MSG_BUSY, "busy"),         # R28: the session goes on by itself
    ("thinking", MSG_SETTLING, "busy"),     # R27
    ("waiting", MSG_WAITING, "waiting"),
    ("error", MSG_POSTURE_UNKNOWN, "error"),
    ("error", "「キッチン」は単語帳にないので、式にできません", "error"),
    ("error", "15秒待っても進めません", "error"),
    ("idle", "", None),
    ("executing", "", None),
]


@pytest.mark.parametrize("status,message,kind", NOTICE_KINDS)
def test_message_kind(status, message, kind):
    assert server.message_kind(status, message) == kind


def test_every_session_message_has_its_notice_kind():
    """A new MSG_* in session.py needs its row in NOTICE_KINDS (and a kind the page knows)."""
    covered = {m for _, m, _ in NOTICE_KINDS}
    names = [n for n in vars(session_module) if n.startswith("MSG_")]
    assert names and all(getattr(session_module, n) in covered for n in names), names


def test_ws_tells_the_kind_of_notice(tmp_path):
    c, sess = make_client(tmp_path)
    sess.submit_text("")
    with c.websocket_connect(WS) as ws:
        snap = ws.receive_json()
    assert snap["message"] == MSG_EMPTY and snap["message_kind"] == "unheard"


# ---------------- SAFETY-3 / REAL-4: the safety caution (design §5) ----------------
def test_the_caution_is_the_readme_caution():
    readme = (server.STATIC.parents[1] / "README.md").read_text(encoding="utf-8")
    for line in ("前方 1 m と周りを空けてください。", "リモコンを持った人が、必ずそばにいてください。"):
        assert line in readme and line in server.SAFETY_CAUTION


def tag(html, element_id):
    """The opening tag of the element with this id."""
    m = re.search(rf'<[a-z]+ [^>]*\bid="{element_id}"[^>]*>', html)
    assert m, element_id
    return m.group(0)


def test_the_page_shows_the_caution_until_it_knows_it_simulates(tmp_path):
    c, _ = make_client(tmp_path)
    html = c.get("/").text
    assert server.SAFETY_CAUTION in html
    assert " hidden" not in tag(html, "caution")              # the safe side: shown at first
    assert " hidden" in tag(html, "caution-sim")
    start = html.index(tag(html, "caution"))
    assert "<button" not in html[start:html.index("</aside>", start)]   # it cannot be closed


@pytest.mark.parametrize("sim,level", [("0", logging.WARNING), ("1", logging.INFO)])
def test_serve_logs_the_caution_and_serves_on_the_page_port(monkeypatch, tmp_path, caplog,
                                                           sim, level):
    class FakeRosLink:
        def __init__(self, perception):
            self.node = None

        def wait_ready(self):
            pass

        def close(self, stop_times=3):
            pass

    served = {}
    ros_link = types.ModuleType("voice_ltl.ros_link")
    ros_link.RosLink, ros_link.SimControl = FakeRosLink, lambda node: None
    stt = types.ModuleType("voice_ltl.stt")
    stt.Transcriber = lambda model: (lambda audio: "")
    monkeypatch.setitem(sys.modules, "voice_ltl.ros_link", ros_link)
    monkeypatch.setitem(sys.modules, "voice_ltl.stt", stt)
    monkeypatch.setattr(server, "make_llm", lambda env: FakeLLM([]))
    monkeypatch.setattr(server, "WORKDIR", tmp_path)
    monkeypatch.setattr(server, "control_loop", lambda link, session, stop: None)
    monkeypatch.setattr(server.uvicorn, "run", lambda app, **kw: served.update(app=app, **kw))
    monkeypatch.setenv("SIM", sim)
    monkeypatch.setenv("GO2_UI_PORT", "9123")
    caplog.set_level(logging.INFO, logger="voice_ltl.server")
    server._serve()
    said = [r for r in caplog.records if server.SAFETY_CAUTION in r.getMessage()]
    assert [r.levelno for r in said] == [level]
    c = TestClient(served["app"], base_url="http://localhost:9123")
    assert c.get("/").status_code == 200
    assert c.get("/", headers={"host": "localhost:8000"}).status_code == 403


# ---------------- the page's script (app.js) in node ----------------
NODE = shutil.which("node")
needs_node = pytest.mark.skipif(NODE is None, reason="node is not installed")

# app.js runs in a vm context with stand-ins for the browser: `any` answers every property
# and call with itself (so drawing and listeners do nothing); getElementById keeps what the
# page writes to an element; fetch is recorded in `sent`; MediaRecorder hands over one chunk
# of Recorder.size bytes when it stops. Timers never fire.
HARNESS = r"""
const fs = require("fs"), vm = require("vm");
const [appJs, testJs] = process.argv.slice(1);
const any = new Proxy(function () {}, {
  get: (t, k) => k === Symbol.toPrimitive ? () => "" : k === Symbol.iterator ? function* () {}
    : k === "then" ? undefined : any,
  set: () => true, apply: () => any, construct: () => any,
});
const element = () => new Proxy({ dataset: {}, hidden: false, textContent: "" }, {
  get: (t, k) => (k in t ? t[k] : any), set: (t, k, v) => { t[k] = v; return true; },
});
const byId = {}, sent = [];
class Recorder {
  static isTypeSupported() { return true; }
  constructor() { this.mimeType = "audio/webm;codecs=opus"; }
  start() {}
  stop() { this.ondataavailable({ data: new Blob([new Uint8Array(Recorder.size)]) }); this.onstop(); }
}
const ctx = {
  document: { getElementById: (id) => (byId[id] ??= element()), createElement: element,
    querySelectorAll: () => [], querySelector: () => null, documentElement: any },
  navigator: {}, location: any, WebSocket: any, DOMParser: any, URL: any, HTMLElement: class {},
  getComputedStyle: () => any, addEventListener: () => {}, MediaRecorder: Recorder, Blob,
  setTimeout: () => 0, clearTimeout: () => {}, setInterval: () => 0, clearInterval: () => {},
  fetch: (url) => { sent.push(url); return Promise.resolve({ status: 202, ok: true }); },
  byId, sent, Recorder,
};
ctx.window = ctx;
vm.createContext(ctx);
vm.runInContext(fs.readFileSync(appJs, "utf8"), ctx, { filename: "app.js" });
process.stdout.write(JSON.stringify(vm.runInContext(testJs, ctx, { filename: "test.js" })));
"""


def run_page(script, **data):
    """Run app.js (HARNESS), then `script` with each of data as a global; the value of script."""
    prelude = "".join(f"var {k} = {json.dumps(v, ensure_ascii=False)};\n" for k, v in data.items())
    out = subprocess.run([NODE, "-e", HARNESS, str(server.STATIC / "app.js"), prelude + script],
                         capture_output=True, text=True, timeout=60)
    assert out.returncode == 0, out.stderr
    return json.loads(out.stdout)


def page_snapshot(tmp_path, **changes):
    """What /ws sends for a fresh session, with changes; message_kind as the server tells it."""
    _, sess = make_client(tmp_path)
    snap = {**server.page_snapshot(sess, None), **changes}
    snap["message_kind"] = server.message_kind(snap["status"], snap["message"])
    return snap


@needs_node
def test_page_notice_for_each_kind(tmp_path):
    cases = [page_snapshot(tmp_path, status=s, message=m) for s, m, _ in NOTICE_KINDS]
    notices = run_page("cases.map((s) => noticeFor(s))", cases=cases)
    again = "もう一度、スペースを押しながら話してください。"
    for (status, message, kind), n in zip(NOTICE_KINDS, notices):
        if kind is None:
            assert n is None
            continue
        assert n["kind"] == {"stopped": "stop", "busy": "waiting", "unheard": "heard",
                             "error": "cannot"}.get(kind, kind), (status, message)
        assert (n["body"] == again) == (kind == "unheard"), (status, message, n)
        assert n["role"] == ("status" if kind in ("done", "busy", "waiting", "unheard") else "alert")


@needs_node
@pytest.mark.parametrize("message", [MSG_BUSY, MSG_SETTLING])
def test_page_busy_notice_has_no_countdown_and_asks_nothing(tmp_path, message):
    """UI-1: while a gesture runs or the posture settles, the page does not ask to speak again."""
    snap = page_snapshot(tmp_path, status="thinking", message=message)
    n = run_page("noticeFor(snap)", snap=snap)
    assert n["kind"] == "waiting" and n["role"] == "status" and n["title"] == message
    assert "after" not in n and "話して" not in n["body"]


@needs_node
@pytest.mark.parametrize("kind,robot,line", [("robot", False, True), ("fake", True, False),
                                             ("gazebo", True, False)])
def test_page_caution_follows_link_kind(tmp_path, kind, robot, line):
    """hidden of #caution (the robot's, cannot be closed) and #caution-sim (one small line)."""
    snap = page_snapshot(tmp_path, link_kind=kind)
    shown = run_page('render(snap); [byId["caution"].hidden, byId["caution-sim"].hidden]', snap=snap)
    assert shown == [robot, line]


PRESS = r"""
function press(ms, bytes) {
  sent.length = 0;
  document.getElementById("hint").textContent = "";
  ptt.stream = {};
  Date.now = () => 1000; startRecording();
  Date.now = () => 1000 + ms; Recorder.size = bytes; stopRecording();
  return { sent: sent.slice(), hint: byId["hint"].textContent, talk: byId["talk"].dataset.state,
           status: byId["status-text"].textContent, transcript: byId["transcript"].textContent };
}
render(snap);
presses.map(([ms, bytes]) => press(ms, bytes));
"""


@needs_node
@pytest.mark.parametrize("status", ["idle", "thinking", "executing"])
def test_page_does_not_send_a_too_short_recording(tmp_path, status):
    """UI-3: under 0.3 s or under 2 KB is not sent; the page says so and shows the rest as it was."""
    snap = page_snapshot(tmp_path, status=status, transcript="立って挨拶して")
    presses = [(100, 50_000), (299, 50_000), (1500, 2047), (1500, 300), (300, 2048), (1500, 30_000)]
    results = run_page(PRESS, snap=snap, presses=presses)
    short = {"sent": [], "hint": "短すぎて聞き取れませんでした"}
    for (ms, size), r in zip(presses, results):
        if ms < 300 or size < 2048:
            assert {k: r[k] for k in short} == short, (ms, size)
        else:
            assert r["sent"] == ["/api/audio"] and r["hint"] == "", (ms, size)
        assert r["talk"] == ("thinking" if status == "thinking" else "idle")
        assert r["transcript"] == "「立って挨拶して」"
        assert r["status"] == {"idle": "指示を待っています", "thinking": "考えています",
                               "executing": "実行中"}[status]
