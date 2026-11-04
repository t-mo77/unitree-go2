import io
import os
import wave
from pathlib import Path

import av
import numpy as np
import pytest

from voice_ltl.stt import Transcriber

FIXTURE = Path(__file__).parent / "fixtures" / "stand_hello.wav"


@pytest.mark.slow
def test_transcribes_japanese_fixture():
    text = Transcriber(os.environ.get("WHISPER_MODEL", "small"))(FIXTURE.read_bytes())
    assert "挨拶" in text


def test_empty_audio_is_silent():
    assert Transcriber.is_silent(b"") and Transcriber.is_silent(b"not audio")


def test_silent_audio_skips_model():
    t = Transcriber()
    assert t(b"") == ""
    assert t._model is None


def test_fixture_is_not_silent():
    assert not Transcriber.is_silent(FIXTURE.read_bytes())


def _wav(seconds, rate=16000):
    buf = io.BytesIO()
    with wave.open(buf, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(rate)
        w.writeframes(np.zeros(int(rate * seconds), dtype=np.int16).tobytes())
    return buf.getvalue()


def _webm_opus(seconds, rate=48000):
    buf = io.BytesIO()
    with av.open(buf, "w", format="webm") as out:
        stream = out.add_stream("libopus", rate=rate)
        stream.layout = "mono"
        n = int(rate * seconds)
        step = 960
        pts = 0
        for i in range(0, n, step):
            samples = np.zeros((1, min(step, n - i)), dtype=np.int16)
            frame = av.AudioFrame.from_ndarray(samples, format="s16", layout="mono")
            frame.sample_rate = rate
            frame.pts = pts
            pts += frame.samples
            for packet in stream.encode(frame):
                out.mux(packet)
        for packet in stream.encode(None):
            out.mux(packet)
    return buf.getvalue()


def test_short_wav_is_silent():
    assert Transcriber.is_silent(_wav(0.2))
    assert not Transcriber.is_silent(_wav(0.5))


def test_short_webm_opus_is_silent_and_skips_model():
    t = Transcriber()
    audio = _webm_opus(0.1)
    assert Transcriber.is_silent(audio)
    assert t(audio) == ""
    assert t._model is None


def test_long_enough_webm_opus_is_readable():
    assert not Transcriber.is_silent(_webm_opus(1.0))
