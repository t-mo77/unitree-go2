"""Speech to text with faster-whisper (Japanese).

A press with no speech must give "" (UI-2): Whisper otherwise makes up text such as
「ご視聴ありがとうございました」, which would end the running plan as a new instruction. So
segments Whisper itself marks as no speech are dropped, and a result made only of the phrases
it makes up (HALLUCINATIONS) is empty. Measured with the small model (int8): faint noise gives
that phrase at no_speech_prob 0.87-0.93 but avg_logprob -0.86 to -0.98, while a clear 「止まって」
has no_speech_prob 0.68, so no_speech_prob alone would drop a stop word.
"""

import io
import time
import unicodedata

import av

MIN_SECONDS = 0.3
NO_SPEECH_PROB = 0.6   # a segment above this ...
LOW_LOGPROB = -1.0     # ... and below this is no speech (Whisper's own thresholds)
# Texts Whisper makes up for no speech (from video subtitles in its training data)
HALLUCINATIONS = (
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
_LONGEST_FIRST = sorted(HALLUCINATIONS, key=len, reverse=True)


def _no_words(text: str) -> bool:
    """True if text has nothing but spaces, punctuation and HALLUCINATIONS phrases."""
    t = "".join(ch for ch in unicodedata.normalize("NFKC", text)
                if not (ch.isspace() or unicodedata.category(ch)[0] in "PSZ"))
    for phrase in _LONGEST_FIRST:
        t = t.replace(phrase, "")
    return not t


def _is_speech(segment) -> bool:
    if segment.no_speech_prob > NO_SPEECH_PROB and segment.avg_logprob < LOW_LOGPROB:
        return False
    return not _no_words(segment.text)


class Transcriber:
    def __init__(self, model_size: str = "small", compute_type: str = "int8"):
        self.model_size = model_size
        self.compute_type = compute_type
        self._model = None  # loaded on first real use
        self.last_load_seconds: float | None = None  # model load of the latest call; None if none

    @staticmethod
    def is_silent(audio: bytes) -> bool:
        """True if audio is empty, unreadable by PyAV, or shorter than 0.3 s."""
        if not audio:
            return True
        try:
            with av.open(io.BytesIO(audio)) as container:
                stream = container.streams.audio[0]
                if container.duration:
                    seconds = container.duration / av.time_base
                elif stream.duration and stream.time_base:
                    seconds = float(stream.duration * stream.time_base)
                else:  # webm from MediaRecorder has no duration: decode and count
                    seconds = 0.0
                    for frame in container.decode(stream):
                        if frame.sample_rate:
                            seconds += frame.samples / frame.sample_rate
        except Exception:
            # 意図: PyAV が読めない音声（壊れた・未対応の形式）は、無音と同じに扱う。
            # 呼び出し側は "" を受け取り、文字起こしをせずに次の入力を待つ。
            return True
        return seconds < MIN_SECONDS

    def __call__(self, audio: bytes) -> str:
        self.last_load_seconds = None
        if self.is_silent(audio):
            return ""
        if self._model is None:
            from faster_whisper import WhisperModel

            loading = time.monotonic()
            self._model = WhisperModel(self.model_size, compute_type=self.compute_type)
            self.last_load_seconds = time.monotonic() - loading
        segments, _ = self._model.transcribe(
            io.BytesIO(audio), language="ja", vad_filter=False, beam_size=5
        )
        text = "".join(s.text for s in segments if _is_speech(s)).strip()
        return "" if _no_words(text) else text
