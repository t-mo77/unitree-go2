import sys
import threading
import time
import types

import pytest
from fastapi.testclient import TestClient
from go2_nl_control.actions import STOP_MOVE

import voice_ltl.server as server
from sim.sim_core import SimGo2
from voice_ltl.llm import FakeLLM
from voice_ltl.server import control_loop, create_app
from voice_ltl.session import DirectSimLink, Session


def make_client(tmp_path):
    sess = Session(DirectSimLink(SimGo2()), FakeLLM([{"formula": "F hello", "unsupported": []}]), lambda b: "立って挨拶して", tmp_path, background=False)
    return TestClient(create_app(sess, sim=None)), sess


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
    with c.websocket_connect("/ws") as ws:
        snap = ws.receive_json()
    assert {"status", "log", "debug_log", "steps", "link_kind", "stt", "llm_backend",
            "waiting_since", "block_timeout", "stop_word"} <= set(snap)


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
