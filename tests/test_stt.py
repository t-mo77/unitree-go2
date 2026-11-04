import io
import os
import wave
from pathlib import Path
from types import SimpleNamespace

import av
import numpy as np
import pytest

from voice_ltl import stt
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


# ---------------- UI-2: no speech must not become text ----------------
class FakeModel:
    """A WhisperModel that gives fixed segments (text, no_speech_prob, avg_logprob)."""

    def __init__(self, *segments):
        self.segments = [SimpleNamespace(text=t, no_speech_prob=p, avg_logprob=lp)
                         for t, p, lp in segments]

    def transcribe(self, audio, **kwargs):
        return iter(self.segments), None


def transcribe_with(*segments):
    t = Transcriber()
    t._model = FakeModel(*segments)
    return t(_wav(0.5))


def test_hallucination_list_is_fixed():
    assert stt.HALLUCINATIONS == (
        "ご視聴ありがとうございました",
        "ご視聴ありがとうございます",
        "最後までご視聴いただきありがとうございました",
        "ご清聴ありがとうございました",
        "チャンネル登録よろしくお願いします",
        "チャンネル登録をお願いします",
        "チャンネル登録お願いします",
        "チャンネル登録",
        "高評価よろしくお願いします",
        "ありがとうございました",
        "お疲れ様でした",
    )
    assert (stt.NO_SPEECH_PROB, stt.LOW_LOGPROB) == (0.6, -1.0)


@pytest.mark.parametrize("phrase", stt.HALLUCINATIONS)
def test_a_result_of_only_hallucinated_phrases_is_empty(phrase):
    # measured on faint noise with the small model: no_speech_prob 0.87-0.93, avg_logprob -0.86..-0.98
    assert transcribe_with((f" {phrase}。", 0.9, -0.9)) == ""
    assert transcribe_with((f"{phrase}！", 0.9, -0.9), ("　ご視聴ありがとうございました", 0.9, -0.9)) == ""


def test_a_hallucinated_segment_after_speech_is_dropped():
    assert transcribe_with(("止まって", 0.2, -0.6), ("ご視聴ありがとうございました", 0.9, -0.9)) == "止まって"


def test_punctuation_alone_is_empty():
    assert transcribe_with(("。", 0.5, -0.5)) == ""


def test_a_no_speech_segment_is_dropped():
    # no_speech_prob > 0.6 and avg_logprob < -1.0: Whisper's own sign of no speech
    assert transcribe_with(("第3位です", 0.7, -1.2)) == ""
    assert transcribe_with(("第3位です", 0.7, -1.2), ("挨拶して", 0.1, -0.4)) == "挨拶して"


@pytest.mark.parametrize("text,no_speech,logprob", [
    ("止まって", 0.681, -0.513),   # measured: a clear 「止まって」 (macOS say, small int8)
    ("停止", 0.601, -0.594),       # measured: a clear 「停止」
    ("前に進んで", 0.459, -1.023),  # low confidence alone is kept
    ("挨拶して", 0.082, -0.425),
])
def test_speech_is_kept(text, no_speech, logprob):
    assert transcribe_with((text, no_speech, logprob)) == text


def test_a_hallucinated_phrase_inside_an_instruction_is_kept():
    text = "挨拶してご視聴ありがとうございました"
    assert transcribe_with((text, 0.3, -0.5)) == text
