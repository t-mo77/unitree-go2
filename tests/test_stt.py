import os
from pathlib import Path

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
