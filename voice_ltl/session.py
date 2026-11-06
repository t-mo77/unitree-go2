"""The conductor: instruction -> formula -> supervisor -> one Go2 action at a time.

- submit_text / submit_audio: a stop word (is_stop_text) sends StopMove at once (no LLM).
  Anything else stops the running plan, then is translated (LLM) on a worker thread.
  Results go through a queue and tick() takes them. Every new instruction and every stop
  bumps the generation; results of an older generation are dropped. A recording changes
  nothing until its text is known (R36): a stop word then always stops, however old; an empty
  or unreadable one leaves the instruction on the way alone; an instruction is dropped only if
  an input that came after it has already taken effect. While another program sends commands
  to the Go2, an instruction is refused (REAL-2); a stop word is not.
- tick(now), 10 Hz: a finished translation waits until the Go2 is ready: a posture change
  or gesture it is still doing has had its planned time (R28), and its posture is known
  (R27: Standing Lock gets BalanceStand; still unknown after 3 s is refused). Then the
  supervisor is synthesized (no LLM) on the worker from that posture.
- While a supervisor runs: sensor changes become uncontrollable events on it; the next
  controllable event on the shortest path runs on the zip's Executor, one Step at a time;
  when the Step ends, the event that happened (the action and the label from the current
  sensors) moves the supervisor; done needs the Go2 in the planned final posture (§7.6).
  A posture that reads UNKNOWN for a moment there (a real Go2 between two error_codes) is
  waited for, up to POSTURE_SETTLE_S. During a Step too the Go2 must stay in a posture its
  action allows, or the plan ends with StopMove (SAFETY-2).
- Objects (design §6): the link's object list is frozen with the pose when an instruction
  arrives (the Scene; the prompt and the check use it). A formula with object propositions
  (near_box_1, around_box_1, ...) runs on the grid plant (grid.py) anchored at the Go2's pose
  when the plan starts, and its steps on GridExecutor (to cell centres and grid headings).
  After each step the Go2 is snapped to the nearest cell and heading, and the supervisor
  follows where it is: the planned event, a jump to the state it really reached, or a stop
  (R10, R11). Any other formula runs on go2_world and the zip's Executor, as before.
- snapshot() is what the page shows: the steps of the plan, what the session is connected to,
  the transcription, the wait, and two logs: "log" in words for people, "debug_log" with the
  session's and the executor's own lines (event names and all). Secret-like text in it (an
  API key, a token) is hidden (R37).
"""

import dataclasses
import json
import logging
import math
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
from voice_ltl.grid import GridError, GridWorld, grid_world
from voice_ltl.grid_executor import GridExecutor, GridStep
from voice_ltl.llm import LLM, MSG_REDACTED, ClaudeCLI, CodexCLI, LLMError, _is_secretish
# MSG_TOO_SPREAD reaches the page as a GridError's text; it is named here with the other copy
from voice_ltl.objects import (MSG_NO_POSE, MSG_POSITION_DRIFT, MSG_TOO_SPREAD, NO_PATH_REASON,  # noqa: F401
                               Obj, Scene, formula_atoms, formula_object_props, parse_rows, snapshot_rows)
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
MSG_POSTURE_CHANGED = "姿勢が変わったので止めました"
MSG_OTHER_SENDERS = "ほかのプログラムが Go2 に指令を送っています。そちらを終了してから指示してください"
MSG_NO_PATH = MSG_UNSATISFIABLE + NO_PATH_REASON   # R15: an object the plan can never reach
LOG_LINES = 200

POSTURE_WAIT_S = 3.0   # R27: how long an unknown posture may take to settle before a plan
POSTURE_SETTLE_S = 1.0  # mid-plan and at the end: how long an UNKNOWN posture is waited for
STAND_LOCK = 1002      # error_code of Standing Lock: StandUp done, BalanceStand not yet (UNKNOWN)
# R11: after each step on the grid the Go2 is snapped to the nearest cell centre and heading; it
# is stopped when it is farther than this from them.
POSITION_TOLERANCE_M = 0.3
POSITION_TOLERANCE_DEG = 30.0
# R28: a posture change or gesture runs to its end in the Go2 (StopMove does not end it). Each
# such command keeps the Go2 busy for the time its action's sequence waits after it.
BUSY_S: dict[int, float] = {api: wait for spec in ACTIONS.values() for api, wait in spec.sequence}
_EPS = 1e-9
_ACTIVE = ("thinking", "executing", "waiting")   # an instruction is on the way


def _label(action: str) -> str:
    """The action's name on the page: ACTION_JA's, or the name itself for an action added to
    world.ACTION_NAMES without a line in ACTION_JA (TD-1)."""
    return ACTION_JA.get(action, action)


def _allowed_postures(action: str) -> set[str]:
    """The postures the Go2 may be in while `action` runs (SAFETY-2): standing for a move or a
    gesture; for a posture change the postures it starts from (stand: also UNKNOWN, the Standing
    Lock it passes through) and the one it ends in."""
    transitions = ACTIONS[action].transitions
    return set(transitions) | set(transitions.values())


# ---------------- secrets on the page (R37) ----------------
# Inside a line: an API key (sk-...) or a long run of letters and digits (a token). Underscores
# are not in the run, so event names (forward_to_moved_forward_standing) stay as they are.
_SECRET_RUN = re.compile(r"sk-[A-Za-z0-9_-]+|[A-Za-z0-9+/=-]{24,}")


def _redact(text: str | None) -> str | None:
    """text for the page with secret-like parts hidden: a line llm.py would hide in a log
    (token=..., Bearer ...) becomes MSG_REDACTED, and in other lines each secret-like run does."""
    if not text:
        return text
    return "\n".join(MSG_REDACTED if _is_secretish(line) else _SECRET_RUN.sub(MSG_REDACTED, line)
                     for line in text.split("\n"))


MAX_SUBSTITUTION_LEN = 60


def _shown_substitutions(tr: Translation) -> list[dict[str, str]]:
    """tr.substitutions for the page and the log: secrets hidden first, then each part clipped."""
    def show(text: str) -> str:
        text = _redact(text)
        return text if len(text) <= MAX_SUBSTITUTION_LEN else text[:MAX_SUBSTITUTION_LEN] + "…"
    return [{"said": show(s["said"]), "as": show(s["as"])} for s in tr.substitutions]


def _redacted(value):
    """_redact() on a string, and on each string of a list."""
    if isinstance(value, str):
        return _redact(value)
    if isinstance(value, list):
        return [_redacted(v) for v in value]
    return value

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
    into a standing Go2 with BalanceStand (R27). Without it an unknown posture is only waited for.
    A link may also have other_senders() -> int: how many other programs send commands to the
    Go2; while there are any, instructions are refused (REAL-2).
    A link may also have objects() -> list[dict]: the objects the front camera has seen, as rows of
    /go2/objects in odom (design §6.2; the session checks them again with objects.parse_rows).
    Without it there are no objects: the Scene of every instruction is empty."""

    def send(self, cmd: Command) -> None: ...
    def posture(self) -> str: ...
    def pose(self) -> tuple[float, float, float] | None: ...
    def reading(self, now: float) -> Reading: ...


class DirectSimLink:
    """For tests: a SimGo2 without ROS. Call advance(now) before each Session.tick(now)."""

    def __init__(self, sim: "SimGo2", perception: PerceptionConfig = PerceptionConfig()):
        from sim.sim_core import FakeCamera   # tests only: the app never imports the simulator

        self.sim = sim
        self.sensing = Sensing(perception)
        self.camera = FakeCamera()   # sees sim.objects as the Gazebo camera does, and remembers them
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

    def other_senders(self) -> int:
        return 0  # only this session drives the SimGo2

    def pose(self) -> tuple[float, float, float] | None:
        return (self.sim.x, self.sim.y, self.sim.yaw)

    def reading(self, now: float) -> Reading:
        return self.sensing.current(now)

    def objects(self) -> list[dict]:
        return [o.to_row() for o in self.camera.see(self.sim, self.now)]


@dataclass
class _Outcome:
    """What the worker hands to tick(): a supervisor to run, or why there is none."""

    translation: Optional[Translation] = None
    synthesis: Optional[Synthesis] = None
    initial: Optional[PlantState] = None
    error: str = ""
    diagrams: dict = field(default_factory=dict)
    grid: Optional[GridWorld] = None   # the grid plant of a formula with object propositions


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
        self.grid_executor = GridExecutor(EXEC_ACTIONS)   # for plans on the grid (R12)
        self._exec: Executor = self.executor              # the one of the running plan; set in _start
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
        # R36: every input (typed, recorded, the stop button) is numbered as it comes
        self._arrivals = 0
        self._took_effect = 0                   # number of the latest input that took effect
        self._transcribing: int | None = None   # number of the newest recording not yet transcribed
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
        self._snap_wait_since: float | None = None  # R11: a step ended while a sensor was lost
        self._clock: float | None = None  # `now` of the latest tick(): the clock waiting_since is on
        self._unknown_since: float | None = None  # since when a check saw the posture UNKNOWN

        self._reading: Reading | None = None
        self._posture = UNKNOWN
        self._pose: tuple[float, float, float] | None = None
        self._objects: tuple[Obj, ...] = ()     # what the link lists now (the page's live list)
        self._object_rows: list[dict] = []      # self._objects for the snapshot, made in tick()
        self._scene: Scene | None = None        # the objects and the pose frozen for the instruction
        self._grid: GridWorld | None = None     # the grid of the plan (None: go2_world, no objects)

        if background:
            threading.Thread(target=self._work, name="session-worker", daemon=True).start()
            threading.Thread(target=self._draw_loop, name="session-diagrams", daemon=True).start()

    # ---------------- input ----------------
    def submit_text(self, text: str) -> None:
        """Empty -> 「聞き取れませんでした」; a stop word -> stop(); else a new instruction."""
        with self._lock:
            gen = self._accept_text(text, None, self._arrive())
            scene = self._scene
        if gen is not None:
            self._dispatch(("translate", gen, (text, scene)))

    def submit_audio(self, audio: bytes) -> None:
        """Transcribe (on the worker), then the same as submit_text. Until the text is known
        nothing changes: the plan keeps running and a translation on the way goes on (R36)."""
        with self._lock:
            n = self._transcribing = self._arrive()
        self._dispatch(("audio", n, audio))

    def stop(self) -> None:
        """StopMove once, forget the plan and any translation on the way, and go idle."""
        with self._lock:
            self._stop(None, self._arrive())

    def _arrive(self) -> int:
        self._arrivals += 1
        return self._arrivals

    def _stop(self, word: str | None, n: int) -> None:
        """stop(); `word` is the stop word heard, None for the button; `n` the input's number."""
        with self._lock:
            self._took_effect = max(self._took_effect, n)
            self._gen += 1                   # drops any translation on the way
            self._prep = None
            self._discard_plan("止める")
            self.link.send(STOP)
            self._status = "idle"
            self._message = MSG_STOPPED
            self._stop_word = word
            self._note(MSG_STOPPED)

    def _accept_text(self, text: str, stt: dict | None, n: int) -> int | None:
        """The quick part of submit_text. Returns the new generation to translate, or None.
        stt: how the text was transcribed (None when it was typed); n: the input's number."""
        with self._lock:
            if is_stop_text(text):           # never too old to stop (R36)
                self._transcript, self._stt = text, stt
                self._note(f"止める言葉: {text}", f"止める言葉：「{text}」")
                self._stop(text, n)
                return None
            if not text.strip():
                self._heard_nothing(MSG_EMPTY)
                return None
            if n < self._took_effect:        # an input that came after it took effect first
                self._note(f"後から来た入力が先に届いたので使いません: {text}", "")
                return None
            self._took_effect = n
            self._transcript, self._stt = text, stt
            refused = self._other_senders() > 0   # REAL-2, as nl_control.py does per instruction
            self._scene = self._freeze_scene()
            self._gen += 1
            self._prep = None
            if self._running:
                self._discard_plan("新しい指示")
                self.link.send(STOP)
            self._done = []
            self._translation = None
            self.synthesis = None
            self.runtime = None
            self._grid = None
            self._syn_id += 1
            self._set_diagrams({})
            self._note(f"指示: {text}", f"指示：「{text}」")
            if refused:
                self._finish("error", MSG_OTHER_SENDERS)
                return None
            self._message = ""
            self._status = "thinking"
            return self._gen

    def _heard_nothing(self, message: str) -> None:
        """An empty or unreadable input (R36): the instruction on the way, if any, goes on with its
        own message, and the transcript stays. With none, `message` is shown."""
        self._note(message)
        if self._status not in _ACTIVE:
            self._message = message

    def _other_senders(self) -> int:
        read = getattr(self.link, "other_senders", None)
        return read() if callable(read) else 0

    def _link_objects(self) -> tuple[Obj, ...]:
        """The link's objects, checked again (R21); () for a link without objects()."""
        read = getattr(self.link, "objects", None)
        return parse_rows(read()) if callable(read) else ()

    def _freeze_scene(self) -> Scene:
        """The objects and the pose as the instruction arrives (design §6.3, R2): the prompt,
        the check and the grid's objects use them, whatever the link lists later."""
        return Scene(self._link_objects(), self.link.pose())

    # ---------------- worker ----------------
    def _dispatch(self, job: tuple) -> None:
        if self.background:
            self._jobs.put(job)
        else:
            self._run(job)

    def _work(self) -> None:
        """One job at a time: LLM calls never overlap, and a stale translation or synthesis is
        skipped (a recording is always transcribed)."""
        while True:
            job = self._jobs.get()
            try:
                self._run(job)
            except Exception:  # keep the worker alive for the next instruction
                _logger.exception("session job failed")

    def _run(self, job: tuple) -> None:
        kind, key, payload = job   # key: the generation; for "audio" the recording's number
        if kind == "audio":
            self._transcribe_then_translate(key, payload)
        elif kind == "translate":   # (text, the Scene frozen for it)
            self._produce(key, self._translate, *payload)
        else:  # "synthesize": (translation, initial state, pose) once the Go2 is ready
            self._produce(key, self._synthesize, key, *payload)

    def _stale(self, gen: int) -> bool:
        with self._lock:
            return gen != self._gen

    def _transcribe_then_translate(self, n: int, audio: bytes) -> None:
        """Every recording is transcribed, however many came after it: it may be a stop word."""
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
                self._transcribed(n)
                self._heard_nothing(MSG_NO_AUDIO)
            return
        with self._lock:
            self._transcribed(n)
            new_gen = self._accept_text(text, stt, n)
            scene = self._scene
        if new_gen is not None:
            self._produce(new_gen, self._translate, text, scene)

    def _transcribed(self, n: int) -> None:
        if self._transcribing == n:
            self._transcribing = None

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

    def _translate(self, text: str, scene: Scene | None) -> _Outcome:
        """A translation alone: tick() makes its supervisor once the Go2 is ready."""
        try:
            tr = translate(text, self.llm, scene=scene)
        except LLMError as exc:
            return _Outcome(error=str(exc))
        if not tr.ok:
            return _Outcome(tr, error=tr.message)
        return _Outcome(tr)

    def _synthesize(self, gen: int, tr: Translation, initial: PlantState,
                    pose: tuple[float, float, float] | None) -> _Outcome | None:
        """The supervisor of tr.formula from `initial`. A formula without object propositions
        runs on go2_world as before; one with them on the grid anchored at `pose` (the Go2 now,
        R2), with the objects of the Scene frozen for the instruction."""
        props = formula_object_props(tr.formula)
        gw = None
        try:
            if not props:
                world = go2_world(initial)
            elif pose is None:
                return _Outcome(tr, error=MSG_NO_POSE)
            else:
                gw = grid_world(initial.posture, initial.front, initial.back, pose,
                                tr.scene.objects if tr.scene else (), formula_atoms(tr.formula))
                world = gw.world
            # R13: the formula goes in as it is; the world's forbidden events keep it safe.
            syn = synthesize(tr.formula, world, self.workdir / f"{gen:04d}")
        except (GridError, SpecError) as exc:
            return _Outcome(tr, error=str(exc), grid=gw)
        if self._stale(gen):  # a newer instruction or a stop came during synthesis: no drawing
            return None
        diagrams = self._draw_all(syn)
        if syn.empty:
            return _Outcome(tr, syn, initial, self._empty_message(props, gw), diagrams, gw)
        return _Outcome(tr, syn, initial, diagrams=diagrams, grid=gw)

    @staticmethod
    def _empty_message(props: frozenset[str], gw: GridWorld | None) -> str:
        """Why there is no supervisor. R15: on the grid, an object proposition that no state or
        event can make true (no free cell near the object, or no ring around it) is the reason."""
        if gw is not None:
            reachable = set(gw.world.initial_label).union(*gw.world.event_label.values())
            if any(p not in reachable for p in props):
                return MSG_NO_PATH
        return MSG_UNSATISFIABLE

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
            self._objects = self._link_objects()
            self._object_rows = snapshot_rows(self._objects, self._pose)
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
        # R2: the grid's cell (0, 0) and heading 0 are the Go2 as it starts the plan
        self._dispatch(("synthesize", p.gen, (p.translation, initial, self._pose)))

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
            if tr.substitutions:
                read_as = "、".join(f"「{s['said']}」→ {s['as']}" for s in _shown_substitutions(tr))
                self._note(f"置き換え: {read_as}", f"こう受け取りました：{read_as}")
        self._translation = tr

    def _start(self, outcome: _Outcome) -> None:
        self._show_translation(outcome.translation)
        self._grid = outcome.grid
        # the old plan is discarded and its executor's lines taken: only here does it change
        self._exec = self.grid_executor if outcome.grid is not None else self.executor
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
        self._snap_wait_since = None
        self._unknown_since = None
        self._done = []
        self._status = "executing"
        self._message = ""
        self._refresh_plan()
        self._note(f"監督器: {outcome.synthesis.sup.size} 状態", self._plan_text())

    def _step(self, now: float, reading: Reading) -> None:
        if self._action is not None and not self._posture_fits_action(now):
            return
        if not self._apply_sensor_changes(reading):
            return
        if self._action is None and not self._load_next(now):
            return
        for cmd in self._exec.tick(now, self._env(self._action, reading), self._pose):
            self._send(cmd, now)
        self._take_executor_events()
        ex = self._exec
        if len(ex.steps) == 1 and ex.idx == 1:
            if self._grid is not None and SENSOR_LOST in (reading.front, reading.back):
                self._wait_to_snap(now)   # R11: the pose may be old; snap once the sensors are back
                return
            self._snap_wait_since = None
            self._complete()
        elif not ex.steps:  # the executor gave up (held too long, or a move took too long)
            self._finish("error", self._last_report, note=False)  # already in the log

    def _posture_fits_action(self, now: float) -> bool:
        """SAFETY-2: while a Step runs, the Go2 stays in a posture its action allows. A remote's
        lie-down and stand-up (or a fall) during a move must not let the move walk on by itself:
        a known posture outside them ends the plan with StopMove. UNKNOWN (a real Go2 between two
        error_codes) is waited for up to POSTURE_SETTLE_S, as between Steps."""
        actual = self._posture
        if actual in _allowed_postures(self._action):
            self._unknown_since = None
            return True
        if actual == UNKNOWN:
            if self._unknown_since is None:
                self._unknown_since = now
            if now - self._unknown_since < POSTURE_SETTLE_S - _EPS:
                return True
        self._note(f"姿勢の変化: {self._action} の途中で {actual}", "")
        self._abort(MSG_POSTURE_CHANGED)
        return False

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
            elif now - self._waiting_since > self._exec.cfg.block_timeout:
                self._abort(f"{self._exec.cfg.block_timeout:g}秒待っても進めません")
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
        self._exec.load([self._step_for(action)])
        self._take_executor_events()
        self._action = action
        self._status = "executing"
        self._say(f"開始：{_label(action)}")
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

    def _step_for(self, action: str) -> Step:
        """The Step for `action`. On the grid it also carries where the action ends: the
        centre of the planned cell and the yaw of the planned heading (R12)."""
        if self._grid is None:
            return Step(action, step_value(action))
        grid = self._grid.grid
        _, dst = self._grid.step(self.synthesis.plant_state(self.runtime.state), action)
        return GridStep(action, step_value(action), grid.center(dst.cell), grid.heading_yaw(dst.heading))

    def _wait_to_snap(self, now: float) -> None:
        """A step on the grid ended while a sensor is lost (R11): its event waits for the
        sensors, up to block_timeout, as a held move does."""
        timeout = self._exec.cfg.block_timeout
        if self._snap_wait_since is None:
            self._snap_wait_since = now
            self._note("センサーが途切れているので、位置合わせを待ちます", "")
        elif now - self._snap_wait_since > timeout:
            self._abort(f"{timeout:g}秒待っても進めません")

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
        before = self._plan_posture
        event = self._grid_event(action) if self._grid is not None else self._go2_event(action)
        if event is None:   # the plan was aborted
            return
        self._done.append(action)
        moved = (f"（{POSTURE_JA[before]} → {POSTURE_JA[self._plan_posture]}）"
                 if before != self._plan_posture else "")
        self._note(f"イベント: {event}", f"完了：{_label(action)}{moved}")
        self._refresh_plan()
        self._redraw_sup()

    def _go2_event(self, action: str) -> str | None:
        """go2_world: the event is the action with the label of the current sensors."""
        # The supervisor only offers events of the plant, so go2_step finds this action.
        event, _ = go2_step(PlantState(self._plan_posture, *self._sensed), action)
        try:
            self.runtime.apply(event)
        except PlanError:  # e.g. an obstacle appeared during a gesture the formula forbids then
            self._abort(f"監督器が許していない結果になりました（{event}）")
            return None
        self._plan_posture = ACTIONS[action].transitions[self._plan_posture]
        return event

    def _grid_event(self, action: str) -> str | None:
        """The grid (R10, R11): the Go2 is snapped to the nearest cell centre and heading. Where
        the plan said, the planned event moves the supervisor. In another cell or heading the
        supervisor jumps to the state of the action landing there (with the event of that
        label), if it has one. Too far from any cell centre or heading, or no such state: the
        plan stops with MSG_POSITION_DRIFT."""
        gw, syn, rt = self._grid, self.synthesis, self.runtime
        src, q = syn.plant_state(rt.state), syn.spec_state(rt.state)
        event, planned = gw.step(src, action)
        s = gw.grid.snap(self._pose) if self._pose is not None else None
        if (s is None or s.offset > POSITION_TOLERANCE_M
                or s.yaw_error > math.radians(POSITION_TOLERANCE_DEG)):
            self._note(f"位置のずれ: {action} の後 {s}" if s else f"位置のずれ: {action} の後、位置がわかりません", "")
            self._abort(MSG_POSITION_DRIFT)
            return None
        if (s.cell, s.heading) == (planned.cell, planned.heading):
            try:
                rt.apply(event)
            except PlanError:
                self._abort(f"監督器が許していない結果になりました（{event}）")
                return None
        else:
            seen = gw.observe(src, action, s.cell, s.heading)
            event = gw.event(seen, action) if seen is not None else None
            q2 = syn.spec_after(q, event) if event is not None else None
            target = syn.sup_state_for(seen, q2) if q2 is not None else None
            if target is None:
                self._note(f"位置のずれ: 計画 {planned.cell},{planned.heading} / 実際 {s.cell},{s.heading}", "")
                self._abort(MSG_POSITION_DRIFT)
                return None
            rt.jump(target)
            self._note(f"位置合わせ: {planned.cell},{planned.heading} -> {s.cell},{s.heading}", "")
        new = syn.plant_state(rt.state)
        self._sensed = (new.front, new.back)   # R1: a stand lands with no obstacle bits
        self._plan_posture = new.posture
        return event

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
            self._exec.cancel(reason)
            self._take_executor_events()
        self._action = None
        self._running = False
        self._waiting_since = None
        self._snap_wait_since = None
        self._plan = []

    def _abort(self, message: str) -> None:
        # cancelling a Step the executor is still running logs "中止: <message>"
        logged = self._action is not None and self._exec.busy
        self._discard_plan(message)
        self.link.send(STOP)
        self._finish("error", message, note=not logged)

    def _finish(self, status: str, message: str, note: bool = True) -> None:
        self._action = None
        self._running = False
        self._waiting_since = None
        self._snap_wait_since = None
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
            return "計画：" + " → ".join(_label(a) for a in self._plan)
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
        while self._exec.events:
            line = self._exec.events.pop(0)
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
        return text.replace(step, _label(self._action))

    def _executor_line(self, line: str) -> str:
        """An executor line for the log people read ("" leaves it out). The executor gets one
        Step at a time, so its 計画 / 完了 / 計画をすべて終えました lines come once per Step:
        the session's own 開始 / 完了 lines say that instead."""
        if line.startswith(("計画: ", "完了: ")) or line == "計画をすべて終えました":
            return ""
        if self._action is None:
            return line
        label = _label(self._action)
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
        return [{"action": a, "label": _label(a), "status": st} for a, st in rows]

    def _wait_start(self) -> float | None:
        """Since when the plan waits (block_timeout ends it): the session waiting for a
        controllable event, or the executor holding a move (an obstacle, a lost sensor)."""
        if self._waiting_since is not None:
            return self._waiting_since
        if self._snap_wait_since is not None:
            return self._snap_wait_since
        if self._action is not None:
            return self._exec.paused_since
        return None

    # ---------------- view ----------------
    def snapshot(self) -> dict:
        with self._lock:
            r = self._reading
            tr = self._translation
            # the newest recording is still being transcribed, and nothing came after it
            transcribing = (self._transcribing is not None
                            and self._transcribing > self._took_effect)
            return {
                "status": "transcribing" if transcribing else self._status,
                "transcript": _redact(self._transcript),
                "attempts": [{k: _redacted(v) for k, v in dataclasses.asdict(a).items()}
                             for a in tr.attempts] if tr else [],
                "formula": _redact(tr.formula) if tr else None,
                "unsupported": _redacted(list(tr.unsupported)) if tr else [],
                "substitutions": _shown_substitutions(tr) if tr else [],
                "message": _redact(self._message),
                "plan": list(self._plan),
                "current": self._action,
                "posture": self._posture,
                "pose": list(self._pose) if self._pose is not None else None,
                "objects": list(self._object_rows),
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
                "block_timeout": self._exec.cfg.block_timeout,
                "stop_word": self._stop_word,
                "log": [_redact(line) for line in self._log],
                "debug_log": [_redact(line) for line in self._debug_log],
            }
