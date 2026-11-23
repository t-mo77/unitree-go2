import struct
import zlib
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from voice_ltl import camera


def decode_png(data):
    """png_bytes が作った PNG（filter 0・IDAT 1 つ）を配列に戻す。"""
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    pos, chunks = 8, {}
    while pos < len(data):
        (length,) = struct.unpack(">I", data[pos:pos + 4])
        kind, body = data[pos + 4:pos + 8], data[pos + 8:pos + 8 + length]
        (crc,) = struct.unpack(">I", data[pos + 8 + length:pos + 12 + length])
        assert crc == zlib.crc32(kind + body) & 0xFFFFFFFF
        chunks[kind] = body
        pos += 12 + length
    width, height, _, color = struct.unpack(">IIBB", chunks[b"IHDR"][:10])
    channels = 3 if color == 2 else 1
    raw = np.frombuffer(zlib.decompress(chunks[b"IDAT"]), dtype=np.uint8).reshape(height, -1)
    assert (raw[:, 0] == 0).all()
    return raw[:, 1:].reshape(height, width, channels).squeeze()


def image(encoding, pixels, pad=0):
    h, w = pixels.shape[:2]
    rows = pixels.reshape(h, -1)
    rows = np.hstack([rows, np.zeros((h, pad), dtype=np.uint8)])
    return SimpleNamespace(encoding=encoding, height=h, width=w, step=rows.shape[1], data=rows.tobytes())


def test_png_round_trip_rgb_and_gray():
    rgb = np.arange(2 * 3 * 3, dtype=np.uint8).reshape(2, 3, 3)
    assert (decode_png(camera.png_bytes(rgb)) == rgb).all()
    gray = np.array([[0, 128], [255, 7]], dtype=np.uint8)
    assert (decode_png(camera.png_bytes(gray)) == gray).all()


def test_image_pixels_handles_bgr_alpha_and_row_padding():
    rgb = np.arange(2 * 2 * 3, dtype=np.uint8).reshape(2, 2, 3)
    assert (camera.image_pixels(image("rgb8", rgb, pad=2)) == rgb).all()
    assert (camera.image_pixels(image("bgr8", rgb[:, :, ::-1].copy())) == rgb).all()
    rgba = np.dstack([rgb, np.full((2, 2), 9, dtype=np.uint8)])
    assert (camera.image_pixels(image("rgba8", rgba)) == rgb).all()
    with pytest.raises(ValueError):
        camera.image_pixels(image("16UC1", rgb))


def test_sim_camera_encodes_once_and_expires(monkeypatch):
    clock = SimpleNamespace(now=10.0)
    monkeypatch.setattr(camera, "time", SimpleNamespace(monotonic=lambda: clock.now))
    cam = camera.SimCamera(Mock())
    assert cam.latest() is None
    rgb = np.zeros((2, 2, 3), dtype=np.uint8)
    cam._on_image(image("rgb8", rgb))
    first = cam.latest()
    assert first[1] == "image/png" and cam.latest() is first
    clock.now += camera.FRAME_STALE_S + 0.1
    assert cam.latest() is None


def test_video_hub_keeps_only_the_answer_to_its_request(monkeypatch):
    clock = SimpleNamespace(now=10.0)
    monkeypatch.setattr(camera, "time", SimpleNamespace(monotonic=lambda: clock.now))
    cam = camera.VideoHubCamera.__new__(camera.VideoHubCamera)
    cam._frame, cam._warned, cam._demand = None, False, 0.0
    cam._waiting = (5, camera.GET_IMAGE_SAMPLE)
    jpeg = b"\xff\xd8\xff\xe0rest"

    def reply(identity, code=0, data=jpeg):
        return SimpleNamespace(header=SimpleNamespace(
            identity=SimpleNamespace(id=identity, api_id=camera.GET_IMAGE_SAMPLE),
            status=SimpleNamespace(code=code)),
            binary=list(np.frombuffer(data, dtype=np.int8)))

    cam._on_response(reply(4))                 # 別の要求への返事
    assert cam.latest() is None and cam._demand == clock.now
    cam._on_response(reply(5))
    assert cam.latest() == (10.0, "image/jpeg", jpeg)
    cam._waiting = (6, camera.GET_IMAGE_SAMPLE)
    cam._on_response(reply(6, code=3104))      # 断られても前の画像は残す
    assert cam.latest()[2] == jpeg


def test_camera_endpoint():
    from fastapi.testclient import TestClient
    from voice_ltl import follow
    cam = Mock()
    cam.latest.return_value = (1.0, "image/jpeg", b"\xff\xd8abc")
    client = TestClient(follow.create_follow_app(Mock(), 8810, Mock(), camera=cam),
                        base_url="http://localhost:8810")
    assert client.get("/api/config").json()["camera"] is True
    response = client.get("/api/camera")
    assert response.content == b"\xff\xd8abc" and response.headers["content-type"] == "image/jpeg"
    assert response.headers["cache-control"] == "no-store"
    cam.latest.return_value = None
    assert client.get("/api/camera").status_code == 204
