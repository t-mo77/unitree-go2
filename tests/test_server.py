from fastapi.testclient import TestClient

from sim.sim_core import SimGo2
from voice_ltl.llm import FakeLLM
from voice_ltl.server import create_app
from voice_ltl.session import DirectSimLink, Session


def make_client(tmp_path):
    sess = Session(DirectSimLink(SimGo2()), FakeLLM([{"formula": "F hello", "unsupported": []}]), lambda b: "立って挨拶して", tmp_path, background=False)
    return TestClient(create_app(sess, sim=None)), sess


def test_text_endpoint_runs_pipeline(tmp_path):
    c, sess = make_client(tmp_path)
    assert c.post("/api/text", json={"text": "立って挨拶して"}).status_code == 202
    sess.tick(0.0); assert sess.snapshot()["formula"] == "F hello"
    assert c.get("/api/diagram/SUP.svg").headers["content-type"].startswith("image/svg")


def test_audio_and_stop_and_obstacle_404(tmp_path):
    c, sess = make_client(tmp_path)
    assert c.post("/api/audio", content=b"RIFF....", headers={"content-type": "audio/wav"}).status_code == 202
    assert c.post("/api/stop").status_code == 204
    assert c.post("/api/obstacle", json={"cmd": "front"}).status_code == 404


def test_ws_sends_snapshot(tmp_path):
    c, _ = make_client(tmp_path)
    with c.websocket_connect("/ws") as ws:
        assert "status" in ws.receive_json()


def test_index_has_push_to_talk(tmp_path):
    c, _ = make_client(tmp_path)
    html = c.get("/").text
    assert "スペース" in html and "app.js" in html
