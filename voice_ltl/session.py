"""The conductor: instruction -> formula -> supervisor -> one Go2 action at a time.

- submit_text / submit_audio: a stop word sends StopMove at once (no LLM). Anything else
  stops the running plan, then is translated (LLM) and synthesized (no LLM) on a worker
  thread. Results go through a queue and tick() takes them. Every new instruction and
  every stop bumps the generation; results of an older generation are dropped.
- tick(now), 10 Hz: sensor changes become uncontrollable events on the supervisor; the
  next controllable event on the shortest path runs on the zip's Executor, one Step at a
  time; when the Step ends, the event that happened (the action and the label from the
  current sensors) moves the supervisor (design §7.6).
"""

import dataclasses
import json
import logging
import queue
import threading
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Callable, Optional, Protocol

from go2_nl_control.actions import ACTIONS, LYING, SITTING, STANDING, UNKNOWN, ActionSpec
from go2_nl_control.executor import STOP, Command, ExecConfig, Executor
from go2_nl_control.perception import PerceptionConfig
from go2_nl_control.supervisor import CLEAR, SENSOR_LOST, Step
from go2_nl_control.translator import is_stop_command

from voice_ltl.automata import SpecError, Synthesis, synthesize
from voice_ltl.diagrams import render_svg
from voice_ltl.llm import LLM, LLMError
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
MSG_EMPTY = "聞き取れませんでした"
MSG_UNSATISFIABLE = "この指示は満たせません"
MSG_DONE = "終わりました"
MSG_STOPPED = "止めました"
MSG_WAITING = "今は進める動作がないので待っています"
MSG_NO_AUDIO = "音声を文字にできませんでした"
LOG_LINES = 200


class Link(Protocol):
    """The Go2 as the session sees it (RosLink on ROS 2, DirectSimLink in tests)."""

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


def _plan_posture(posture: str) -> str:
    # StandUp -> BalanceStand works from anywhere, so an unknown posture plans as lying.
    return LYING if posture == UNKNOWN else posture


def _sup_highlight(rt: Runtime) -> tuple[int, list[tuple[int, str]]]:
    """The runtime's state and the (state, event) arrows of its remaining plan."""
    state, events = rt.state, []
    out: dict[tuple[int, str], int] = {(s, e): d for s, e, d in rt.sup.trans}
    for event in rt.plan() or []:
        events.append((state, event))
        state = out[(state, event)]
    return rt.state, events


class Session:
    def __init__(self, link: Link, llm: LLM, transcribe: Callable[[bytes], str] | None,
                 workdir: Path, background: bool = True):
        self.link = link
        self.llm = llm
        self.transcribe = transcribe
        self.workdir = Path(workdir)
        self.background = background
        self.executor = Executor(EXEC_ACTIONS, ExecConfig())
        self.synthesis: Synthesis | None = None
        self.runtime: Runtime | None = None
        # SVG strings keyed PLANT / SPEC / TRIMMED / SUP; PLANT, SPEC and TRIMMED are drawn
        # once per synthesis, SUP again whenever the supervisor moves. Drawing never runs
        # inside tick() when background=True (design §7.8).
        self.diagrams: dict[str, str] = {}
        self.diagram_version = 0  # bumped whenever self.diagrams changes
        self._syn_id = 0          # which synthesis self.diagrams belong to
        self._sup_slot: tuple | None = None  # newest SUP drawing request (latest wins)
        self._sup_wake = threading.Event()

        self._lock = threading.RLock()
        self._gen = 0
        self._jobs: queue.Queue = queue.Queue()
        self._results: queue.Queue = queue.Queue()
        self._status = "idle"
        self._transcribing_gen: int | None = None
        self._transcript = ""
        self._translation: Translation | None = None
        self._message = ""
        self._log: deque[str] = deque(maxlen=LOG_LINES)
        self._last_report = ""

        self._running = False           # a supervisor is being followed
        self._action: str | None = None  # the action loaded in the executor
        self._plan: list[str] = []
        self._plan_posture = LYING       # posture the supervisor believes in
        self._sensed = (True, True)      # (front, back) blocked, as told to the supervisor
        self._waiting_since: float | None = None

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
                self._status = "idle"
            gen = self._gen
        self._dispatch(("audio", gen, audio))

    def stop(self) -> None:
        """StopMove once, forget the plan and any translation on the way, and go idle."""
        with self._lock:
            self._gen += 1                   # drops any translation on the way
            self._transcribing_gen = None
            self._discard_plan("止める")
            self.link.send(STOP)
            self._status = "idle"
            self._message = MSG_STOPPED
            self._note(MSG_STOPPED)

    def _accept_text(self, text: str) -> int | None:
        """The quick part of submit_text. Returns the new generation to translate, or None."""
        with self._lock:
            self._transcript = text
            if not text.strip():
                self._message = MSG_EMPTY
                self._note(MSG_EMPTY)
                return None
            if is_stop_command(text):
                self._note(f"止める言葉: {text}")
                self.stop()
                return None
            self._gen += 1
            if self._running:
                self._discard_plan("新しい指示")
                self.link.send(STOP)
            self._translation = None
            self.synthesis = None
            self.runtime = None
            self._syn_id += 1
            self._set_diagrams({})
            self._message = ""
            self._status = "thinking"
            self._note(f"指示: {text}")
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
        else:
            self._translate_and_synthesize(gen, payload)

    def _stale(self, gen: int) -> bool:
        with self._lock:
            return gen != self._gen

    def _transcribe_then_translate(self, gen: int, audio: bytes) -> None:
        if self._stale(gen):
            return
        try:
            if self.transcribe is None:
                raise RuntimeError("no transcriber")
            text = self.transcribe(audio)
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
            new_gen = self._accept_text(text)
        if new_gen is not None:
            self._translate_and_synthesize(new_gen, text)

    def _translate_and_synthesize(self, gen: int, text: str) -> None:
        if self._stale(gen):
            return
        try:
            outcome = self._make_supervisor(gen, text)
        except Exception as exc:  # a bug must not leave the session thinking forever
            _logger.exception("planning failed")
            outcome = _Outcome(error=f"処理の途中でエラーが起きました（{type(exc).__name__}）")
        if outcome is not None:
            self._results.put((gen, outcome))

    def _make_supervisor(self, gen: int, text: str) -> _Outcome | None:
        try:
            tr = translate(text, self.llm)
        except LLMError as exc:
            return _Outcome(error=str(exc))
        if not tr.ok:
            return _Outcome(tr, error=tr.message)
        if self._stale(gen):
            return None
        initial = self._initial_state()
        try:
            # R13: the formula goes in as it is; go2_world's forbidden events keep it safe.
            syn = synthesize(tr.formula, go2_world(initial), self.workdir / f"{gen:04d}")
        except SpecError as exc:
            return _Outcome(tr, error=str(exc))
        diagrams = self._draw_all(syn)
        if syn.empty:
            return _Outcome(tr, syn, initial, MSG_UNSATISFIABLE, diagrams)
        return _Outcome(tr, syn, initial, diagrams=diagrams)

    @staticmethod
    def _draw_all(syn: Synthesis) -> dict[str, str]:
        """The four diagrams of a new synthesis (on the worker thread). {} if drawing fails."""
        try:
            state, events = _sup_highlight(Runtime(syn.sup)) if not syn.empty else (None, [])
            return {
                "PLANT": render_svg(syn.plant, "PLANT"),
                "SPEC": render_svg(syn.spec, "SPEC"),
                "TRIMMED": render_svg(syn.trimmed, "TRIMMED"),
                "SUP": render_svg(syn.sup, "SUP", state, events),
            }
        except Exception:  # a missing `dot` must not stop the robot from following the plan
            _logger.exception("drawing diagrams failed")
            return {}

    def _initial_state(self) -> PlantState:
        with self._lock:
            reading = self._reading  # the last tick's; none yet counts as blocked (safe side)
        front, back = (reading.front_blocked, reading.back_blocked) if reading else (True, True)
        return PlantState(_plan_posture(self.link.posture()), front, back)

    # ---------------- control loop ----------------
    def tick(self, now: float) -> None:
        """One control period (10 Hz): read the Go2, take a finished translation, run the plan."""
        with self._lock:
            self._reading = self.link.reading(now)
            self._posture = self.link.posture()
            self._pose = self.link.pose()
            self._take_results()
            if self._running:
                self._step(now, self._reading)

    def _take_results(self) -> None:
        while True:
            try:
                gen, outcome = self._results.get_nowait()
            except queue.Empty:
                return
            if gen == self._gen:
                self._start(outcome)

    def _start(self, outcome: _Outcome) -> None:
        tr = outcome.translation
        self._translation = tr
        if tr is not None and tr.formula:
            self._note(f"式: {tr.formula}")
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
        self._status = "executing"
        self._message = ""
        self._refresh_plan()
        self._note(f"監督器: {outcome.synthesis.sup.size} 状態")

    def _step(self, now: float, reading: Reading) -> None:
        if not self._apply_sensor_changes(reading):
            return
        if self._action is None and not self._load_next(now):
            return
        for cmd in self.executor.tick(now, self._env(self._action, reading), self._pose):
            self.link.send(cmd)
        self._take_executor_events()
        ex = self.executor
        if len(ex.steps) == 1 and ex.idx == 1:
            self._complete()
        elif not ex.steps:  # the executor gave up (held too long, or a move took too long)
            self._finish("error", self._last_report, note=False)  # already in the log

    def _apply_sensor_changes(self, reading: Reading) -> bool:
        """A change in front / back "blocked" since the last tick is an uncontrollable event."""
        front, back = self._sensed
        events = []
        if reading.front_blocked != front:
            events.append("front_blocked" if reading.front_blocked else "front_clear")
        if reading.back_blocked != back:
            events.append("back_blocked" if reading.back_blocked else "back_clear")
        for event in events:
            try:
                self.runtime.apply(event)
            except PlanError:  # supcon keeps every uncontrollable event, so this is a bug
                self._abort(f"監督器がセンサーの変化を受け付けませんでした（{event}）")
                return False
            self._note(f"センサー: {event}")
        self._sensed = (reading.front_blocked, reading.back_blocked)
        if events:
            self._refresh_plan()
            self._redraw_sup()
        return True

    def _load_next(self, now: float) -> bool:
        if self.runtime.done:
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
        if _plan_posture(self._posture) != self._plan_posture:
            self._abort(f"姿勢が計画と違います（計画: {POSTURE_JA[self._plan_posture]}"
                        f" / 実際: {POSTURE_JA.get(self._posture, self._posture)}）")
            return False
        if self._waiting_since is not None:
            self._waiting_since = None
            self._message = ""
        action = event_action(event)
        self.executor.load([Step(action, step_value(action))])
        self._take_executor_events()
        self._action = action
        self._status = "executing"
        return True

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
        self._plan_posture = ACTIONS[action].transitions[self._plan_posture]
        self._note(f"イベント: {event}")
        self._refresh_plan()
        self._redraw_sup()

    # ---------------- helpers ----------------
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

    def _set_diagrams(self, diagrams: dict[str, str]) -> None:
        self.diagrams = diagrams
        self.diagram_version += 1

    def _redraw_sup(self) -> None:
        """The supervisor moved: draw SUP again with the state and the rest of the plan.
        With a background thread the control loop only hands over the request."""
        if self.runtime is None or self.synthesis is None or not self.diagrams:
            return
        state, events = _sup_highlight(self.runtime)
        request = (self._syn_id, self.runtime.sup, state, events)
        if not self.background:
            self._draw_sup(request)
            return
        self._sup_slot = request  # a newer request replaces one not yet drawn
        self._sup_wake.set()

    def _draw_loop(self) -> None:
        while True:
            self._sup_wake.wait()
            self._sup_wake.clear()
            request, self._sup_slot = self._sup_slot, None
            if request is not None:
                try:
                    self._draw_sup(request)
                except Exception:
                    _logger.exception("drawing SUP failed")

    def _draw_sup(self, request: tuple) -> None:
        syn_id, sup, state, events = request
        svg = render_svg(sup, "SUP", state, events)
        with self._lock:
            if syn_id == self._syn_id and self.diagrams:  # not for an older instruction
                self._set_diagrams({**self.diagrams, "SUP": svg})

    def _take_executor_events(self) -> None:
        while self.executor.events:
            line = self.executor.events.pop(0)
            if self._action == "backward":  # the zip names every forward=True hold "前方"
                line = line.replace("前方に障害物", "後方に障害物")
            self._last_report = line
            self._note(line)

    def _note(self, line: str) -> None:
        self._log.append(line)
        _logger.info(line)

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
                "log": list(self._log),
            }
