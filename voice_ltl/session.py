"""The conductor: instruction -> formula -> supervisor -> one Go2 action at a time.

- submit_text / submit_audio: a stop word (is_stop_text) sends StopMove at once (no LLM).
  Anything else stops the running plan, then is translated (LLM) on a worker thread.
  Results go through a queue and tick() takes them. Every new instruction and every stop
  bumps the generation; results of an older generation are dropped.
- tick(now), 10 Hz: a finished translation waits until the Go2 is ready: a posture change
  or gesture it is still doing has had its planned time (R28), and its posture is known
  (R27: Standing Lock gets BalanceStand; still unknown after 3 s is refused). Then the
  supervisor is synthesized (no LLM) on the worker from that posture.
- While a supervisor runs: sensor changes become uncontrollable events on it; the next
  controllable event on the shortest path runs on the zip's Executor, one Step at a time;
  when the Step ends, the event that happened (the action and the label from the current
  sensors) moves the supervisor; done needs the Go2 in the planned final posture (§7.6).
  A posture that reads UNKNOWN for a moment there (a real Go2 between two error_codes) is
  waited for, up to POSTURE_SETTLE_S.
- snapshot() is what the page shows: the steps of the plan, what the session is connected to,
  the transcription, the wait, and two logs: "log" in words for people, "debug_log" with the
  session's and the executor's own lines (event names and all).
"""

import dataclasses
import json
import logging
import queue
import re
import threading
import time
import unicodedata
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Callable, Optional, Protocol

from go2_nl_control.actions import (ACTIONS, BALANCE_STAND, LYING, SITTING, STANDING, UNKNOWN,
                                    ActionSpec)
from go2_nl_control.executor import STOP, Command, ExecConfig, Executor
from go2_nl_control.perception import PerceptionConfig
from go2_nl_control.supervisor import CLEAR, SENSOR_LOST, Step
from go2_nl_control.translator import STOP_WORDS, is_stop_command

from voice_ltl.automata import SpecError, Synthesis, synthesize
from voice_ltl.diagrams import render_focus_svg, render_svg
from voice_ltl.llm import LLM, ClaudeCLI, CodexCLI, LLMError
from voice_ltl.planner import PlanError, Runtime
from voice_ltl.sensing import Reading, Sensing
from voice_ltl.translator import Translation, translate
from voice_ltl.world import PlantState, event_action, go2_step, go2_world, step_value

if TYPE_CHECKING:
    from sim.sim_core import SimGo2

_logger = logging.getLogger(__name__)

# The executor holds a forward=True action while its env is BLOCKED. tick() gives
# backward the back sensor as its env, so a box behind holds backward the same way.
EXEC_ACTIONS: dict[str, ActionSpec] = {
    **ACTIONS, "backward": dataclasses.replace(ACTIONS["backward"], forward=True)}

POSTURE_JA = {LYING: "伏せ", STANDING: "立ち", SITTING: "おすわり", UNKNOWN: "不明"}
ACTION_JA = {
    "stand": "立つ", "lie_down": "伏せる", "sit": "おすわり", "rise_sit": "立ち上がる",
    "hello": "挨拶", "stretch": "伸び", "content": "喜ぶ", "heart": "ハート", "scrape": "おじぎ",
    "dance": "ダンス", "forward": "前進", "backward": "後退", "turn_left": "左に回る",
    "turn_right": "右に回る",
}
SENSOR_JA = {"front_blocked": "前方に物が出た", "front_clear": "前方の物が消えた",
             "back_blocked": "後方に物が出た", "back_clear": "後方の物が消えた"}
FOCUS_TITLE = "計画の道"
MSG_EMPTY = "聞き取れませんでした"
MSG_UNSATISFIABLE = "この指示は満たせません"
MSG_DONE = "終わりました"
MSG_STOPPED = "止めました"
MSG_WAITING = "今は進める動作がないので待っています"
MSG_NO_AUDIO = "音声を文字にできませんでした"
MSG_BUSY = "前の動作が終わるのを待っています"
MSG_SETTLING = "Go2 の姿勢が決まるのを待っています"
MSG_POSTURE_UNKNOWN = "Go2 の姿勢がわかりません。リモコンで伏せさせてから、もう一度話してください"
LOG_LINES = 200

POSTURE_WAIT_S = 3.0   # R27: how long an unknown posture may take to settle before a plan
POSTURE_SETTLE_S = 1.0  # mid-plan and at the end: how long an UNKNOWN posture is waited for
STAND_LOCK = 1002      # error_code of Standing Lock: StandUp done, BalanceStand not yet (UNKNOWN)
# R28: a posture change or gesture runs to its end in the Go2 (StopMove does not end it). Each
# such command keeps the Go2 busy for the time its action's sequence waits after it.
BUSY_S: dict[int, float] = {api: wait for spec in ACTIONS.values() for api, wait in spec.sequence}
_EPS = 1e-9

# ---------------- stop words ----------------
_STOP_REPEATS = [(re.compile(rf"{re.escape(w)}(?:[\W_]*{re.escape(w)})+", re.IGNORECASE), w)
                 for w in STOP_WORDS]


def _edge_char(ch: str) -> bool:
    return ch.isspace() or unicodedata.category(ch)[0] in "PS"   # punctuation, symbols


def normalize_stop_text(text: str) -> str:
    """The text as the zip's is_stop_command should see it: NFKC (ｽﾄｯﾌﾟ -> ストップ,
    ＳＴＯＰ -> STOP), no spaces / punctuation / symbols at either end (「止まって！」), and one
    stop word said again and again (「止まって止まって止まって」) written once."""
    t = unicodedata.normalize("NFKC", text)
    i, j = 0, len(t)
    while i < j and _edge_char(t[i]):
        i += 1
    while j > i and _edge_char(t[j - 1]):
        j -= 1
    t = t[i:j]
    for pattern, word in _STOP_REPEATS:
        t = pattern.sub(word, t)
    return t


def is_stop_text(text: str) -> bool:
    """A stop word: StopMove at once, no LLM (the zip's rule after normalize_stop_text)."""
    return is_stop_command(normalize_stop_text(text))


class Link(Protocol):
    """The Go2 as the session sees it (RosLink on ROS 2, DirectSimLink in tests).
    A link may also have error_code() -> int | None; only then is Standing Lock (1002) turned
    into a standing Go2 with BalanceStand (R27). Without it an unknown posture is only waited for."""

    def send(self, cmd: Command) -> None: ...
    def posture(self) -> str: ...
    def pose(self) -> tuple[float, float, float] | None: ...
    def reading(self, now: float) -> Reading: ...


class DirectSimLink:
    """For tests: a SimGo2 without ROS. Call advance(now) before each Session.tick(now)."""

    def __init__(self, sim: "SimGo2", perception: PerceptionConfig = PerceptionConfig()):
        self.sim = sim
        self.sensing = Sensing(perception)
        self.now = 0.0  # time of the last advance(); commands are handled at this time

    def advance(self, now: float) -> None:
        self.now = now
        self.sim.step(now)
        self.sensing.update(self.sim.cloud_lidar(), now)

    def send(self, cmd: Command) -> None:
        param = "" if cmd.parameter is None else json.dumps(cmd.parameter)
        self.sim.handle(cmd.api_id, param, self.now)

    def posture(self) -> str:
        return self.sim.posture

    def error_code(self) -> int | None:
        return self.sim.error_code

    def pose(self) -> tuple[float, float, float] | None:
        return (self.sim.x, self.sim.y, self.sim.yaw)

    def reading(self, now: float) -> Reading:
        return self.sensing.current(now)


@dataclass
class _Outcome:
    """What the worker hands to tick(): a supervisor to run, or why there is none."""

    translation: Optional[Translation] = None
    synthesis: Optional[Synthesis] = None
    initial: Optional[PlantState] = None
    error: str = ""
    diagrams: dict = field(default_factory=dict)


@dataclass
class _Prep:
    """A translation waiting for the Go2 to be ready before its supervisor is made."""

    gen: int
    translation: Translation
    since: Optional[float] = None   # when the wait for a known posture began
    balance_sent: bool = False      # BalanceStand sent for Standing Lock
    told: str = ""                  # the waiting message already shown


def _blocked(reading: Reading | None, posture: str) -> tuple[bool, bool]:
    """(front, back) blocked as the supervisor is told (R24). Only a standing Go2 can move, and
    lying or sitting the LiDAR sees the ground and the Go2's own legs: in any other posture both
    count as clear. A box really there is a front_blocked / back_blocked once the Go2 stands."""
    if posture != STANDING:
        return False, False
    if reading is None:  # no reading yet counts as blocked (safe side)
        return True, True
    return reading.front_blocked, reading.back_blocked


def _sup_highlight(rt: Runtime) -> tuple[int, list[tuple[int, str]]]:
    """The runtime's state and the (state, event) arrows of its remaining plan."""
    state, events = rt.state, []
    out: dict[tuple[int, str], int] = {(s, e): d for s, e, d in rt.sup.trans}
    for event in rt.plan() or []:
        events.append((state, event))
        state = out[(state, event)]
    return rt.state, events


def _backend(llm: LLM) -> str | None:
    """Which LLM translates: "claude" or "codex" (None for a test's FakeLLM)."""
    if isinstance(llm, ClaudeCLI):
        return "claude"
    if isinstance(llm, CodexCLI):
        return "codex"
    return None


class Session:
    def __init__(self, link: Link, llm: LLM, transcribe: Callable[[bytes], str] | None,
                 workdir: Path, background: bool = True, link_kind: str = "fake"):
        """link_kind is what the page says the link is: "fake" (the fake Go2), "gazebo" or
        "robot" (server.link_kind decides it)."""
        self.link = link
        self.llm = llm
        self.transcribe = transcribe
        self.workdir = Path(workdir)
        self.background = background
        self.link_kind = link_kind
        self.llm_backend = _backend(llm)
        self.executor = Executor(EXEC_ACTIONS, ExecConfig())
        self.synthesis: Synthesis | None = None
        self.runtime: Runtime | None = None
        # SVG strings keyed FOCUS / SUP / TRIMMED / SPEC / PLANT; PLANT, SPEC and TRIMMED are
        # drawn once per synthesis, FOCUS and SUP again whenever the supervisor moves. Drawing
        # never runs inside tick() when background=True (design §7.8).
        self.diagrams: dict[str, str] = {}
        self.diagram_version = 0  # bumped whenever self.diagrams changes
        self._syn_id = 0          # which synthesis self.diagrams belong to
        self._sup_slot: tuple | None = None  # newest FOCUS + SUP drawing request (latest wins)
        self._sup_lock = threading.Lock()    # guards _sup_slot between tick and the drawing thread
        self._sup_wake = threading.Event()

        self._lock = threading.RLock()
        self._gen = 0
        self._jobs: queue.Queue = queue.Queue()
        self._results: queue.Queue = queue.Queue()
        self._status = "idle"
        self._transcribing_gen: int | None = None
        self._transcript = ""
        self._stt: dict | None = None     # {"model", "seconds"} of the transcript; None if typed
        self._translation: Translation | None = None
        self._message = ""
        self._stop_word: str | None = None  # the word that made the last stop (None: the button)
        self._log: deque[str] = deque(maxlen=LOG_LINES)        # in words, for people
        self._debug_log: deque[str] = deque(maxlen=LOG_LINES)  # the session's own lines
        self._last_report = ""            # why the executor gave up, for the message

        self._prep: _Prep | None = None  # a translation waiting for the Go2 to be ready
        self._busy_until = float("-inf")  # R28: the Go2 is doing a posture change / gesture
        self._running = False           # a supervisor is being followed
        self._action: str | None = None  # the action loaded in the executor
        self._plan: list[str] = []
        self._done: list[str] = []       # the actions of this instruction's plan done so far
        self._plan_posture = LYING       # posture the supervisor believes in
        self._sensed = (True, True)      # (front, back) blocked, as told to the supervisor
        self._waiting_since: float | None = None
        self._clock: float | None = None  # `now` of the latest tick(): the clock waiting_since is on
        self._unknown_since: float | None = None  # since when a check saw the posture UNKNOWN

        self._reading: Reading | None = None
        self._posture = UNKNOWN
        self._pose: tuple[float, float, float] | None = None

        if background:
            threading.Thread(target=self._work, name="session-worker", daemon=True).start()
            threading.Thread(target=self._draw_loop, name="session-diagrams", daemon=True).start()

    # ---------------- input ----------------
    def submit_text(self, text: str) -> None:
        """Empty -> 「聞き取れませんでした」; a stop word -> stop(); else a new instruction."""
        gen = self._accept_text(text)
        if gen is not None:
            self._dispatch(("translate", gen, text))

    def submit_audio(self, audio: bytes) -> None:
        """Transcribe (on the worker), then the same as submit_text. The plan keeps running
        until the text is known."""
        with self._lock:
            self._gen += 1                   # the newest input wins
            self._transcribing_gen = self._gen
            if self._status == "thinking":   # that translation is dropped now
                self._prep = None
                self._status = "idle"
            gen = self._gen
        self._dispatch(("audio", gen, audio))

    def stop(self) -> None:
        """StopMove once, forget the plan and any translation on the way, and go idle."""
        self._stop(None)

    def _stop(self, word: str | None) -> None:
        """stop(); `word` is the stop word heard, None for the button."""
        with self._lock:
            self._gen += 1                   # drops any translation on the way
            self._transcribing_gen = None
            self._prep = None
            self._discard_plan("止める")
            self.link.send(STOP)
            self._status = "idle"
            self._message = MSG_STOPPED
            self._stop_word = word
            self._note(MSG_STOPPED)

    def _accept_text(self, text: str, stt: dict | None = None) -> int | None:
        """The quick part of submit_text. Returns the new generation to translate, or None.
        stt: how the text was transcribed (None when it was typed)."""
        with self._lock:
            self._transcript = text
            self._stt = stt
            if not text.strip():
                self._message = MSG_EMPTY
                self._note(MSG_EMPTY)
                return None
            if is_stop_text(text):
                self._note(f"止める言葉: {text}", f"止める言葉：「{text}」")
                self._stop(text)
                return None
            self._gen += 1
            self._prep = None
            if self._running:
                self._discard_plan("新しい指示")
                self.link.send(STOP)
            self._done = []
            self._translation = None
            self.synthesis = None
            self.runtime = None
            self._syn_id += 1
            self._set_diagrams({})
            self._message = ""
            self._status = "thinking"
            self._note(f"指示: {text}", f"指示：「{text}」")
            return self._gen

    # ---------------- worker ----------------
    def _dispatch(self, job: tuple) -> None:
        if self.background:
            self._jobs.put(job)
        else:
            self._run(job)

    def _work(self) -> None:
        """One job at a time: LLM calls never overlap, and a stale job is skipped."""
        while True:
            job = self._jobs.get()
            try:
                self._run(job)
            except Exception:  # keep the worker alive for the next instruction
                _logger.exception("session job failed")

    def _run(self, job: tuple) -> None:
        kind, gen, payload = job
        if kind == "audio":
            self._transcribe_then_translate(gen, payload)
        elif kind == "translate":
            self._produce(gen, self._translate, payload)
        else:  # "synthesize": (translation, initial state) once the Go2 is ready
            self._produce(gen, self._synthesize, gen, *payload)

    def _stale(self, gen: int) -> bool:
        with self._lock:
            return gen != self._gen

    def _transcribe_then_translate(self, gen: int, audio: bytes) -> None:
        if self._stale(gen):
            return
        try:
            if self.transcribe is None:
                raise RuntimeError("no transcriber")
            started = time.monotonic()
            text = self.transcribe(audio)
            total = time.monotonic() - started
            load = getattr(self.transcribe, "last_load_seconds", None)   # stt.Transcriber's
            # "seconds" is the transcription alone; the one-time model load is reported apart
            stt = {"model": getattr(self.transcribe, "model_size", None),
                   "seconds": round(max(0.0, total - (load or 0.0)), 1)}
            if load:
                stt["load_seconds"] = round(load, 1)
        except Exception:  # faster-whisper and PyAV raise many types
            _logger.exception("transcription failed")
            with self._lock:
                if gen == self._gen:
                    self._transcribing_gen = None
                    self._message = MSG_NO_AUDIO
                    self._note(MSG_NO_AUDIO)
            return
        with self._lock:
            if gen != self._gen:
                return
            self._transcribing_gen = None
            new_gen = self._accept_text(text, stt)
        if new_gen is not None:
            self._produce(new_gen, self._translate, text)

    def _produce(self, gen: int, make: Callable[..., Optional[_Outcome]], *args) -> None:
        """Run one step of planning for `gen` and hand its outcome to tick()."""
        if self._stale(gen):
            return
        try:
            outcome = make(*args)
        except Exception as exc:  # a bug must not leave the session thinking forever
            _logger.exception("planning failed")
            outcome = _Outcome(error=f"処理の途中でエラーが起きました（{type(exc).__name__}）")
        if outcome is not None:
            self._results.put((gen, outcome))

    def _translate(self, text: str) -> _Outcome:
        """A translation alone: tick() makes its supervisor once the Go2 is ready."""
        try:
            tr = translate(text, self.llm)
        except LLMError as exc:
            return _Outcome(error=str(exc))
        if not tr.ok:
            return _Outcome(tr, error=tr.message)
        return _Outcome(tr)

    def _synthesize(self, gen: int, tr: Translation, initial: PlantState) -> _Outcome | None:
        try:
            # R13: the formula goes in as it is; go2_world's forbidden events keep it safe.
            syn = synthesize(tr.formula, go2_world(initial), self.workdir / f"{gen:04d}")
        except SpecError as exc:
            return _Outcome(tr, error=str(exc))
        if self._stale(gen):  # a newer instruction or a stop came during synthesis: no drawing
            return None
        diagrams = self._draw_all(syn)
        if syn.empty:
            return _Outcome(tr, syn, initial, MSG_UNSATISFIABLE, diagrams)
        return _Outcome(tr, syn, initial, diagrams=diagrams)

    @staticmethod
    def _draw_all(syn: Synthesis) -> dict[str, str]:
        """The five diagrams of a new synthesis (on the worker thread). {} if drawing fails."""
        try:
            state, events = _sup_highlight(Runtime(syn.sup)) if not syn.empty else (None, [])
            return {
                "FOCUS": render_focus_svg(syn.sup, FOCUS_TITLE, state, events),
                "PLANT": render_svg(syn.plant, "PLANT"),
                "SPEC": render_svg(syn.spec, "SPEC"),
                "TRIMMED": render_svg(syn.trimmed, "TRIMMED"),
                "SUP": render_svg(syn.sup, "SUP", state, events),
            }
        except Exception:  # a missing `dot` must not stop the robot from following the plan
            _logger.exception("drawing diagrams failed")
            return {}

    # ---------------- control loop ----------------
    def tick(self, now: float) -> None:
        self._clock = now
        """One control period (10 Hz): read the Go2, take finished translations and supervisors,
        get the Go2 ready for a new plan, run the plan."""
        with self._lock:
            self._reading = self.link.reading(now)
            self._posture = self.link.posture()
            self._pose = self.link.pose()
            self._take_results()
            if self._prep is not None:
                self._get_ready(now)
                self._take_results()  # without a worker thread the supervisor is made already
            if self._running:
                self._step(now, self._reading)

    def _take_results(self) -> None:
        while True:
            try:
                gen, outcome = self._results.get_nowait()
            except queue.Empty:
                return
            if gen != self._gen:
                continue
            if outcome.synthesis is None and not outcome.error:  # translated: wait for the Go2
                self._show_translation(outcome.translation)
                self._prep = _Prep(gen, outcome.translation)
            else:
                self._start(outcome)

    def _get_ready(self, now: float) -> None:
        """Make the supervisor only once the Go2 is ready. R28: a posture change or gesture it
        is still doing has had its planned time. R27: its posture is known; Standing Lock (1002)
        gets BalanceStand and is planned as standing once that ends; a posture still unknown
        after POSTURE_WAIT_S is refused, and nothing is sent."""
        p = self._prep
        if p.gen != self._gen:
            self._prep = None
            return
        if now < self._busy_until - _EPS:   # busy with our own BalanceStand: settling too
            self._tell(p, MSG_SETTLING if p.balance_sent else MSG_BUSY)
            return
        posture = self._posture
        if posture == UNKNOWN:
            if p.since is None:
                p.since = now
            if not p.balance_sent and self._error_code() == STAND_LOCK:
                p.balance_sent, p.since = True, now   # its own POSTURE_WAIT_S to reach 1013
                self._note("立ち固め（1002）なので BalanceStand を送ります")
                self._send(Command(BALANCE_STAND), now)
            elif now - p.since >= POSTURE_WAIT_S - _EPS:
                self._prep = None
                self._finish("error", MSG_POSTURE_UNKNOWN)
            else:
                self._tell(p, MSG_SETTLING)
            return
        self._prep = None
        self._message = ""
        initial = PlantState(posture, *_blocked(self._reading, posture))
        self._dispatch(("synthesize", p.gen, (p.translation, initial)))

    def _tell(self, p: _Prep, message: str) -> None:
        if p.told != message:
            p.told = self._message = message
            self._note(message)

    def _error_code(self) -> int | None:
        read = getattr(self.link, "error_code", None)   # RosLink has none (R27)
        return read() if callable(read) else None

    def _show_translation(self, tr: Translation | None) -> None:
        if tr is not None and tr is not self._translation and tr.formula:
            self._note(f"式: {tr.formula}", f"式：{tr.formula}")
        self._translation = tr

    def _start(self, outcome: _Outcome) -> None:
        self._show_translation(outcome.translation)
        if outcome.synthesis is not None:
            self.synthesis = outcome.synthesis
            self._syn_id += 1
            self._set_diagrams(outcome.diagrams)
        if outcome.error:
            self._finish("error", outcome.error)
            return
        self.runtime = Runtime(outcome.synthesis.sup)
        self._plan_posture = outcome.initial.posture
        self._sensed = (outcome.initial.front, outcome.initial.back)
        self._running = True
        self._waiting_since = None
        self._unknown_since = None
        self._done = []
        self._status = "executing"
        self._message = ""
        self._refresh_plan()
        self._note(f"監督器: {outcome.synthesis.sup.size} 状態", self._plan_text())

    def _step(self, now: float, reading: Reading) -> None:
        if not self._apply_sensor_changes(reading):
            return
        if self._action is None and not self._load_next(now):
            return
        for cmd in self.executor.tick(now, self._env(self._action, reading), self._pose):
            self._send(cmd, now)
        self._take_executor_events()
        ex = self.executor
        if len(ex.steps) == 1 and ex.idx == 1:
            self._complete()
        elif not ex.steps:  # the executor gave up (held too long, or a move took too long)
            self._finish("error", self._last_report, note=False)  # already in the log

    def _apply_sensor_changes(self, reading: Reading) -> bool:
        """A change in front / back "blocked" since the last tick is an uncontrollable event."""
        front, back = self._sensed
        now_front, now_back = _blocked(reading, self._posture)
        events = []
        if now_front != front:
            events.append("front_blocked" if now_front else "front_clear")
        if now_back != back:
            events.append("back_blocked" if now_back else "back_clear")
        for event in events:
            try:
                self.runtime.apply(event)
            except PlanError:  # supcon keeps every uncontrollable event, so this is a bug
                self._abort(f"監督器がセンサーの変化を受け付けませんでした（{event}）")
                return False
            self._note(f"センサー: {event}", SENSOR_JA[event])
        self._sensed = (now_front, now_back)
        if events:
            self._refresh_plan()
            self._redraw_sup()
        return True

    def _load_next(self, now: float) -> bool:
        if self.runtime.done:
            actual = self._settled_posture(now)   # R28: the Go2 may have ignored a command
            if actual is None:
                return False
            if actual != self._plan_posture:
                self._abort(self._posture_mismatch(actual))
            else:
                self._finish("done", MSG_DONE)
            return False
        event = self.runtime.next_event()
        if event is None:
            if self._waiting_since is None:
                self._waiting_since = now
                self._status = "waiting"
                self._message = MSG_WAITING
                self._note(MSG_WAITING)
            elif now - self._waiting_since > self.executor.cfg.block_timeout:
                self._abort(f"{self.executor.cfg.block_timeout:g}秒待っても進めません")
            return False
        actual = self._settled_posture(now)
        if actual is None:
            return False
        if actual != self._plan_posture:
            self._abort(self._posture_mismatch(actual))
            return False
        if self._waiting_since is not None:
            self._waiting_since = None
            self._message = ""
        action = event_action(event)
        self.executor.load([Step(action, step_value(action))])
        self._take_executor_events()
        self._action = action
        self._status = "executing"
        self._say(f"開始：{ACTION_JA[action]}")
        return True

    def _settled_posture(self, now: float) -> str | None:
        """The Go2's posture, to compare with the plan's. A real Go2 reads UNKNOWN for a moment
        while its error_code changes from one value to the next: that is waited for (None) up
        to POSTURE_SETTLE_S, and only then compared as it is."""
        actual = self.link.posture()
        if actual != UNKNOWN:
            self._unknown_since = None
            return actual
        if self._unknown_since is None:
            self._unknown_since = now
        if now - self._unknown_since < POSTURE_SETTLE_S - _EPS:
            return None
        return actual

    @staticmethod
    def _env(action: str, reading: Reading) -> str:
        if action == "forward":
            return reading.front
        if action == "backward":
            return reading.back
        return SENSOR_LOST if SENSOR_LOST in (reading.front, reading.back) else CLEAR

    def _complete(self) -> None:
        """The loaded Step ended: move the supervisor by the event that happened."""
        action, self._action = self._action, None
        # The supervisor only offers events of the plant, so go2_step finds this action.
        event, _ = go2_step(PlantState(self._plan_posture, *self._sensed), action)
        try:
            self.runtime.apply(event)
        except PlanError:  # e.g. an obstacle appeared during a gesture the formula forbids then
            self._abort(f"監督器が許していない結果になりました（{event}）")
            return
        before = self._plan_posture
        self._plan_posture = ACTIONS[action].transitions[before]
        self._done.append(action)
        moved = (f"（{POSTURE_JA[before]} → {POSTURE_JA[self._plan_posture]}）"
                 if before != self._plan_posture else "")
        self._note(f"イベント: {event}", f"完了：{ACTION_JA[action]}{moved}")
        self._refresh_plan()
        self._redraw_sup()

    # ---------------- helpers ----------------
    def _send(self, cmd: Command, now: float) -> None:
        """Send a command of the plan. A posture change or gesture keeps the Go2 busy (R28)."""
        self.link.send(cmd)
        busy = BUSY_S.get(cmd.api_id)
        if busy is not None:
            self._busy_until = max(self._busy_until, now + busy)

    def _posture_mismatch(self, actual: str) -> str:
        return (f"姿勢が計画と違います（計画: {POSTURE_JA[self._plan_posture]}"
                f" / 実際: {POSTURE_JA.get(actual, actual)}）")

    def _discard_plan(self, reason: str) -> None:
        """Forget the plan. The caller sends StopMove (the executor's own is not sent)."""
        if self._action is not None:
            self.executor.cancel(reason)
            self._take_executor_events()
        self._action = None
        self._running = False
        self._waiting_since = None
        self._plan = []

    def _abort(self, message: str) -> None:
        logged = self._action is not None  # cancelling a loaded Step logs "中止: <message>"
        self._discard_plan(message)
        self.link.send(STOP)
        self._finish("error", message, note=not logged)

    def _finish(self, status: str, message: str, note: bool = True) -> None:
        self._action = None
        self._running = False
        self._waiting_since = None
        self._plan = []
        self._status = status
        self._message = message
        if note:
            self._note(message)

    def _refresh_plan(self) -> None:
        path = self.runtime.plan() if self.runtime is not None else None
        self._plan = [event_action(e) for e in path] if path else []

    def _plan_text(self) -> str:
        """The plan as a log line, when a supervisor starts."""
        if self._plan:
            return "計画：" + " → ".join(ACTION_JA[a] for a in self._plan)
        if self.runtime.done:
            return "計画：指示はもう満たされているので、動きません"
        return "計画：今は進める動作がありません"

    def _set_diagrams(self, diagrams: dict[str, str]) -> None:
        self.diagrams = diagrams
        self.diagram_version += 1

    def _redraw_sup(self) -> None:
        """The supervisor moved: draw FOCUS and SUP again with the state and the rest of the
        plan. With a background thread the control loop only hands over the request."""
        if self.runtime is None or self.synthesis is None or not self.diagrams:
            return
        state, events = _sup_highlight(self.runtime)
        request = (self._syn_id, self.runtime.sup, state, events)
        if not self.background:
            self._draw_sup(request)
            return
        with self._sup_lock:
            self._sup_slot = request  # a newer request replaces one not yet drawn
        self._sup_wake.set()

    def _draw_loop(self) -> None:
        while True:
            self._sup_wake.wait()
            self._sup_wake.clear()
            with self._sup_lock:  # take and empty in one step: a request made meanwhile waits
                request, self._sup_slot = self._sup_slot, None
            if request is not None:
                try:
                    self._draw_sup(request)
                except Exception:
                    _logger.exception("drawing SUP failed")

    def _draw_sup(self, request: tuple) -> None:
        """FOCUS and SUP of one request, shown together (they always show the same state)."""
        syn_id, sup, state, events = request
        focus = render_focus_svg(sup, FOCUS_TITLE, state, events)
        svg = render_svg(sup, "SUP", state, events)
        with self._lock:
            if syn_id == self._syn_id and self.diagrams:  # not for an older instruction
                self._set_diagrams({**self.diagrams, "FOCUS": focus, "SUP": svg})

    def _take_executor_events(self) -> None:
        while self.executor.events:
            line = self.executor.events.pop(0)
            if self._action == "backward":  # the zip names every forward=True hold "前方"
                line = line.replace("前方に障害物", "後方に障害物")
            if line.startswith("中止: "):    # why it gave up: the message, if that ends the plan
                self._last_report = self._in_words(line[len("中止: "):])
            self._note(line, self._executor_line(line))

    def _in_words(self, text: str) -> str:
        """The executor's text with the loaded Step (`forward(0.5m)`) as its Japanese name."""
        if self._action is None:
            return text
        step = Step(self._action, step_value(self._action)).text(EXEC_ACTIONS)
        return text.replace(step, ACTION_JA[self._action])

    def _executor_line(self, line: str) -> str:
        """An executor line for the log people read ("" leaves it out). The executor gets one
        Step at a time, so its 計画 / 完了 / 計画をすべて終えました lines come once per Step:
        the session's own 開始 / 完了 lines say that instead."""
        if line.startswith(("計画: ", "完了: ")) or line == "計画をすべて終えました":
            return ""
        if self._action is None:
            return line
        label = ACTION_JA[self._action]
        step = Step(self._action, step_value(self._action)).text(EXEC_ACTIONS)
        if line.startswith(hold := f"一時停止（{step}）: "):
            return f"一時停止：{label}（{line[len(hold):]}）"
        if line == f"再開（{step}）":
            return f"再開：{label}"
        if line.startswith("中止: "):
            return f"中止：{label}（{self._in_words(line[len('中止: '):])}）"
        return line

    def _note(self, line: str, human: str | None = None) -> None:
        """`line` goes to debug_log and the program's log; `human` (by default the same line)
        to the log people read. human="" keeps the line out of that log."""
        self._debug_log.append(line)
        _logger.info(line)
        text = line if human is None else human
        if text:
            self._log.append(text)

    def _say(self, human: str) -> None:
        """A line only for the log people read (debug_log has the executor's own line)."""
        self._log.append(human)

    def _steps(self) -> list[dict]:
        """This instruction's plan: the actions done, the one running, and the rest of the
        shortest path. The running one stays even when a sensor change took it off the path."""
        rows = [(a, "done") for a in self._done]
        ahead = list(self._plan)
        if self._action is not None:
            rows.append((self._action, "current"))
            if ahead and ahead[0] == self._action:
                ahead.pop(0)
        rows += [(a, "pending") for a in ahead]
        return [{"action": a, "label": ACTION_JA[a], "status": st} for a, st in rows]

    def _wait_start(self) -> float | None:
        """Since when the plan waits (block_timeout ends it): the session waiting for a
        controllable event, or the executor holding a move (an obstacle, a lost sensor)."""
        if self._waiting_since is not None:
            return self._waiting_since
        if self._action is not None:
            return self.executor.paused_since
        return None

    # ---------------- view ----------------
    def snapshot(self) -> dict:
        with self._lock:
            r = self._reading
            tr = self._translation
            return {
                "status": "transcribing" if self._transcribing_gen == self._gen else self._status,
                "transcript": self._transcript,
                "attempts": [dataclasses.asdict(a) for a in tr.attempts] if tr else [],
                "formula": tr.formula if tr else None,
                "unsupported": list(tr.unsupported) if tr else [],
                "message": self._message,
                "plan": list(self._plan),
                "current": self._action,
                "posture": self._posture,
                "pose": list(self._pose) if self._pose is not None else None,
                "front": r.front if r else SENSOR_LOST,
                "back": r.back if r else SENSOR_LOST,
                "front_dist": r.front_dist if r else None,
                "back_dist": r.back_dist if r else None,
                "sup_state": self.runtime.state if self.runtime is not None else None,
                "diagram_version": self.diagram_version,
                "steps": self._steps(),
                "link_kind": self.link_kind,
                "stt": dict(self._stt) if self._stt is not None else None,
                "llm_backend": self.llm_backend,
                "waiting_since": self._wait_start(),
                "server_now": self._clock,   # same clock as waiting_since: the page needs no clock of its own
                "block_timeout": self.executor.cfg.block_timeout,
                "stop_word": self._stop_word,
                "log": list(self._log),
                "debug_log": list(self._debug_log),
            }
