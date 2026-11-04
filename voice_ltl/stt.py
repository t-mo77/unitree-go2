"""Speech to text with faster-whisper (Japanese)."""

import io

import av

MIN_SECONDS = 0.3


class Transcriber:
    def __init__(self, model_size: str = "small", compute_type: str = "int8"):
        self.model_size = model_size
        self.compute_type = compute_type
        self._model = None  # loaded on first real use

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
            return True
        return seconds < MIN_SECONDS

    def __call__(self, audio: bytes) -> str:
        if self.is_silent(audio):
            return ""
        if self._model is None:
            from faster_whisper import WhisperModel

            self._model = WhisperModel(self.model_size, compute_type=self.compute_type)
        segments, _ = self._model.transcribe(
            io.BytesIO(audio), language="ja", vad_filter=False, beam_size=5
        )
        return "".join(s.text for s in segments).strip()
