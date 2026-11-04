"""Adversarial runs of the whole loop: Session + the fake Go2 through DirectSimLink (Task C2).

Every trial has a fixed seed and drives the session at 10 Hz (link.advance(now), then
session.tick(now)) for at most 120 s of simulated time. At random moments it throws in
new instructions (FakeLLM answers with a formula from a fixed set, an unsupported reply,
an unsatisfiable formula or an LLMError), stop words, the stop button, boxes in front of
or behind the Go2, box removal, and quiet time. After a random phase of at most 60 s a
last instruction is left alone until the session settles. Half of the stops (chosen by a
stream of their own, so the schedule stays the same) wait up to 6 s for a move to be
running: a new plan may start seconds after its instruction (R28), so stops that land on a
move would otherwise be rare.

The trials run once per module. Each property is its own test over all trials, so one
broken property does not hide the others. Only Session's public names are used
(submit_text, tick, snapshot, stop); what the Go2 really did is read from the fake Go2.

Bugs found here and fixed in Session (Task F1; the step-by-step tests are at the bottom):
- BUG-C2-1: a new instruction during a gesture or posture change sent the next command
  while the Go2 was still busy; the Go2 ignored it, and the session still said 終わりました.
  Now the new plan waits for the planned time of that action, and done needs the planned
  final posture (R28).
- BUG-C2-2: the Standing Lock posture (UNKNOWN, after a stop during stand) was planned as
  lying, so `F lying` was done at once while the Go2 stood. Now the session never plans
  from UNKNOWN: it waits, sends BalanceStand for 1002, or refuses (R27).
"""

import math
import random
from collections import Counter
from dataclasses import dataclass, field

import numpy as np
import pytest
from go2_nl_control.actions import (BALANCE_STAND, CONTENT, DANCE1, HEART, HELLO, LYING, MOVE,
                                    RISE_SIT, SCRAPE, SIT, SITTING, STAND_DOWN, STAND_UP,
                                    STANDING, STOP_MOVE, STRETCH, UNKNOWN)
from go2_nl_control.perception import PerceptionConfig

from sim.sim_core import SimGo2
from voice_ltl.llm import FakeLLM, LLMError
from voice_ltl.ltlf import holds
from voice_ltl.session import DirectSimLink, Session
from voice_ltl.world import PROPOSITIONS

DT = 0.1
TRIAL_S = 120.0
RANDOM_PHASE_S = (20.0, 60.0)  # the random phase ends somewhere in here; then the last instruction
QUIET_AFTER_S = 2.0            # once the last instruction settles, check this long that nothing is sent
SEEDS = range(2026_1104, 2026_1104 + 64)
MAX_BOXES = 3
NEAR_BOX = (0.55, 0.68)        # [m] to the box centre: blocked at once (0.55 still leaves a tick to react)
FAR_BOX = (0.7, 1.6)           # [m]: blocked only when the Go2 walks up to it
SAFE_GAP = PerceptionConfig().stop_distance     # 0.5 m
GEOMETRY_TOLERANCE = 0.02      # the sensor uses the 5th nearest point; the geometry the nearest

FORMULAS = {
    "F hello": "挨拶して",
    "F(moved_forward & X F moved_forward)": "1メートル進んで",
    "F moved_backward": "後ろに下がって",
    "G !danced & F stretched": "ダンスはしないで伸びをして",
    "(!sitting U hello) & F sitting": "挨拶するまで座らないで、そのあと座って",
    "F lying": "伏せて",
    "F(turned_left & X F turned_left)": "左に180度回って",
}
UNSUPPORTED = ({"formula": None, "unsupported": ["キッチン"]}, "キッチンへ行って")
UNSATISFIABLE = ({"formula": "F sitting & G !sitting", "unsupported": []}, "座って、でも絶対に座らないで")
LLM_DOWN = (LLMError("LLM の返事が 60 秒で返ってきませんでした"), "挨拶して")
STOP_TEXTS = ("止まって", "ストップ", "停止")
STOP_WAITS_FOR_MOVE = 0.5      # share of the random phase's stops that wait for a move to be running
STOP_WAIT_S = 6.0              # how long such a stop waits at most

# The test's own reading of what the Go2 did (not taken from voice_ltl.world).
POSTURE_PROP = {LYING: "lying", STANDING: "standing", SITTING: "sitting"}   # UNKNOWN: none
ACTION_API = {"stand": (STAND_UP, BALANCE_STAND), "lie_down": (STAND_DOWN,), "sit": (SIT,),
              "rise_sit": (RISE_SIT,), "hello": (HELLO,), "stretch": (STRETCH,),
              "content": (CONTENT,), "heart": (HEART,), "scrape": (SCRAPE,), "dance": (DANCE1,)}
ACTION_PROP = {"hello": "hello", "stretch": "stretched", "content": "content", "heart": "heart",
               "scrape": "scraped", "dance": "danced", "forward": "moved_forward",
               "backward": "moved_backward", "turn_left": "turned_left",
               "turn_right": "turned_right"}
MOVES = ("forward", "backward", "turn_left", "turn_right")

PROPERTIES = {
    "P1": "にせ Go2 が箱にぶつからない",
    "P2": "伏せ・おすわりのときに Move が受け付けられない",
    "P3": "前方（後方）0.5 m 以内に物があるあいだ、前向き（後ろ向き）の Move を送らない",
    "P4": "session.tick / submit_text / stop が例外を出さない",
    "P5": "止める言葉・止めるボタン・動作中の新しい指示の直後は StopMove、止めたあとは何も送らず動かない",
    "P6": "最後の指示を邪魔しなければ done / error / waiting になり、最後は done か error",
    "P7": "done のとき、Session が記録したイベントのラベル列で式が真",
    "P7b": "done のとき、にせ Go2 が実際にしたことのラベル列で式が真（既知のバグ以外）",
    "P8": "done / error のあと、次の指示・停止まで何も送らない",
    "BUG-C2-1": "Go2 が動作中で無視した指令を、Session が実行済みとして done にする",
    "BUG-C2-2": "姿勢 UNKNOWN（立ち固め 1002 など）のまま計画を始める（R27）",
    "harness": "テストの前提（FakeLLM の返事の数、ログの読み取り）",
}


# ---------------------------------------------------------------- recording link
@dataclass
class Sent:
    t: float
    api_id: int
    x: float          # Move's forward speed (0 for other commands)
    front: str        # Sensing's judgement when the command was sent
    back: str
    gap_front: float  # nearest box surface ahead / behind in the sensor's corridor (geometry)
    gap_back: float
    posture: str      # the fake Go2's posture just before the command
    accepted: bool


def corridor_gaps(sim: SimGo2) -> tuple[float, float]:
    """Nearest box surface ahead of / behind the Go2's centre, within the sensor's corridor."""
    half_width = PerceptionConfig().half_width
    front = back = math.inf
    c, s = math.cos(sim.yaw), math.sin(sim.yaw)
    for b in sim.boxes:
        h = b.size / 2.0
        u = np.linspace(-h, h, 31)
        xs = np.concatenate([b.x + u, b.x + u, np.full(31, b.x - h), np.full(31, b.x + h)])
        ys = np.concatenate([np.full(31, b.y - h), np.full(31, b.y + h), b.y + u, b.y + u])
        dx, dy = xs - sim.x, ys - sim.y
        bx, by = c * dx + s * dy, -s * dx + c * dy
        near = np.abs(by) <= half_width
        ahead, behind = bx[near & (bx > 0)], -bx[near & (bx < 0)]
        if ahead.size:
            front = min(front, float(ahead.min()))
        if behind.size:
            back = min(back, float(behind.min()))
    return front, back


class RecordingLink(DirectSimLink):
    """DirectSimLink that keeps every command with what the Go2 and the sensors said then."""

    def __init__(self, sim: SimGo2):
        super().__init__(sim)
        self.sent: list[Sent] = []

    def send(self, cmd) -> None:
        reading = self.reading(self.now)
        posture, before = self.sim.posture, len(self.sim.accepted)
        gaps = corridor_gaps(self.sim) if cmd.api_id == MOVE else (math.inf, math.inf)
        super().send(cmd)
        x = float((cmd.parameter or {}).get("x", 0.0)) if cmd.api_id == MOVE else 0.0
        self.sent.append(Sent(self.now, cmd.api_id, x, reading.front, reading.back, *gaps,
                              posture, len(self.sim.accepted) > before))


# ---------------------------------------------------------------- schedule
@dataclass(frozen=True)
class Event:
    tick: int
    kind: str      # instruct | stop_word | stop_button | box_front | box_back | clear | quiet
    arg: object = None


def _pick_instruction(rng: random.Random, n: int):
    roll = rng.random()
    if roll < 0.10:
        reply, text = UNSUPPORTED
    elif roll < 0.18:
        reply, text = UNSATISFIABLE
    elif roll < 0.26:
        reply, text = LLM_DOWN
    else:
        formula = rng.choice(sorted(FORMULAS))
        reply, text = {"formula": formula, "unsupported": []}, FORMULAS[formula]
    return reply, f"{text}（{n}）"   # unique text, so every instruction has its own log line


def make_schedule(rng: random.Random) -> tuple[list[Event], list, int]:
    """Random events of one trial, the FakeLLM replies in order, and the tick of the last instruction."""
    events: list[Event] = []
    replies: list = []

    def instruct(tick: int) -> None:
        reply, text = _pick_instruction(rng, len(replies) + 1)
        replies.append(reply)
        events.append(Event(tick, "instruct", (text, reply)))

    phase_end = round(rng.uniform(*RANDOM_PHASE_S) / DT)
    tick = rng.choice([0, 1, rng.randint(2, 20)])     # 0: before the first tick (no reading yet)
    instruct(tick)
    while True:
        roll = rng.random()
        if roll < 0.25:
            gap = rng.uniform(0.0, 0.5)               # often lands in the middle of an action
        elif roll < 0.6:
            gap = rng.uniform(0.5, 4.0)
        else:
            gap = rng.uniform(4.0, 15.0)
        tick += round(gap / DT)
        if tick >= phase_end:
            break
        kind = rng.choices(
            ["instruct", "stop_word", "stop_button", "box_front", "box_back", "clear", "quiet"],
            weights=[30, 10, 5, 14, 12, 10, 10])[0]
        if kind == "instruct":
            instruct(tick)
        elif kind == "stop_word":
            events.append(Event(tick, kind, rng.choice(STOP_TEXTS)))
        elif kind in ("box_front", "box_back"):
            events.append(Event(tick, kind, round(rng.uniform(*(NEAR_BOX if rng.random() < 0.35
                                                                else FAR_BOX)), 2)))
        else:
            events.append(Event(tick, kind))
    instruct(phase_end)
    return events, replies, phase_end


# ---------------------------------------------------------------- labels
def label_of_event(event: str) -> frozenset[str] | None:
    """The label inside an event name (`<action>_to_<props joined by _ in sorted order>`)."""
    if "_to_" not in event:
        return None

    def split(s: str):
        for p in PROPOSITIONS:
            if s == p:
                return [p]
            if s.startswith(p + "_"):
                tail = split(s[len(p) + 1:])
                if tail is not None:
                    return [p] + tail
        return None

    props = split(event.split("_to_", 1)[1])
    return None if props is None else frozenset(props)


def obstacle_props(front_blocked: bool, back_blocked: bool) -> frozenset[str]:
    return frozenset(({"obstacle_front"} if front_blocked else set())
                     | ({"obstacle_back"} if back_blocked else set()))


def did_action(action: str, sends: list[Sent], pose_a, pose_b) -> bool:
    """Whether the fake Go2 really did `action` between two poses, given the commands it accepted."""
    if action in ACTION_API:
        return any(s.accepted and s.api_id == ACTION_API[action][-1] for s in sends)
    (xa, ya, yawa), (xb, yb, yawb) = pose_a, pose_b
    along = (xb - xa) * math.cos(yawa) + (yb - ya) * math.sin(yawa)
    turned = yawb - yawa                     # the fake Go2 does not wrap its yaw
    return {"forward": along >= 0.4, "backward": along <= -0.4,
            "turn_left": turned >= math.radians(80),
            "turn_right": turned <= -math.radians(80)}.get(action, False)


# ---------------------------------------------------------------- one trial
@dataclass
class Episode:
    """One instruction, from submit_text to the next instruction or stop."""
    text: str
    formula: str | None
    t0: float
    posture0: str           # the fake Go2's posture when the instruction came
    obstacles0: frozenset
    sent0: int
    pose0: tuple
    start_posture: str | None = None             # the fake Go2's posture when the plan started
    events: list = field(default_factory=list)   # (t, event, posture, pose, obstacles, sent index)
    checked: bool = False


@dataclass
class Trial:
    seed: int
    schedule: list
    violations: dict = field(default_factory=lambda: {k: [] for k in PROPERTIES})
    stats: dict = field(default_factory=dict)
    done_formulas: Counter = field(default_factory=Counter)

    def flag(self, prop: str, t: float, message: str) -> None:
        if len(self.violations[prop]) < 5:
            self.violations[prop].append(f"seed={self.seed} t={t:.1f}s: {message}")

    def steps_until(self, t: float) -> str:
        return "; ".join(f"{e.tick * DT:.1f}s {e.kind}{'' if e.arg is None else ' ' + _short(e.arg)}"
                         for e in self.schedule if e.tick * DT <= t + 1e-9)


def _short(arg) -> str:
    if isinstance(arg, tuple):
        text, reply = arg
        what = type(reply).__name__ if isinstance(reply, Exception) else (reply["formula"] or "unsupported")
        return f"{text} -> {what}"
    return str(arg)


def _new_lines(prev: list[str], cur: list[str]) -> list[str]:
    """Lines added to the session's log (a bounded deque) since the previous snapshot."""
    for k in range(min(len(prev), len(cur)), -1, -1):
        if prev[len(prev) - k:] == cur[:k]:
            return cur[k:]
    return cur


class Runner:
    def __init__(self, seed: int, workdir):
        schedule, replies, self.final_tick = make_schedule(random.Random(seed))
        self.trial = Trial(seed, schedule)
        self.wait_rng = random.Random(seed + 1)         # its own stream: the schedule stays the same
        self.held: list[tuple[int, Event]] = []         # (last tick, stop) waiting for a move
        self.by_tick: dict[int, list[Event]] = {}
        for e in schedule:
            self.by_tick.setdefault(e.tick, []).append(e)
        self.sim = SimGo2()
        self.link = RecordingLink(self.sim)
        self.llm = FakeLLM(replies)
        self.n_instructions = 0
        self.session = Session(self.link, self.llm, None, workdir, background=False)
        self.now = 0.0
        self.ticked = False
        self.log: list[str] = []
        self.status = "idle"
        self.episode: Episode | None = None
        self.stopped: tuple[int, tuple] | None = None   # (sent index, pose) after a stop
        self.quiet_from: int | None = None              # sent index when done / error was seen
        self.final_seen: list[str] = []
        self.stats = dict(instructions=0, stops=0, boxes=0, events=0, done=0, error=0, waiting=0,
                          held_moves=0, stops_mid_move=0, switches_mid_run=0)

    # ---- helpers ----
    def snapshot(self) -> dict:
        snap = self.session.snapshot()
        self.status = snap["status"]
        return snap

    def flag(self, prop: str, message: str) -> None:
        self.trial.flag(prop, self.now, message)

    def guarded(self, what: str, fn, *args) -> None:
        try:
            fn(*args)
        except Exception as exc:  # noqa: BLE001 - any exception is the finding
            self.flag("P4", f"{what} raised {type(exc).__name__}: {exc}; "
                            f"steps: {self.trial.steps_until(self.now)}")

    def pose(self) -> tuple:
        return (self.sim.x, self.sim.y, self.sim.yaw)

    def obstacles(self) -> frozenset:
        if not self.ticked:
            return obstacle_props(True, True)        # Session counts "no reading yet" as blocked
        r = self.link.reading(self.now)
        return obstacle_props(r.front_blocked, r.back_blocked)

    def first_sent_after(self, i0: int) -> str:
        return str(self.link.sent[i0].api_id) if len(self.link.sent) > i0 else "nothing"

    # ---- inputs (between ticks) ----
    def feed(self, tick: int) -> None:
        """This tick's events, and the stops held for a move (fired while one runs, or at
        their last tick; never once the last instruction is in)."""
        current = self.session.snapshot()["current"]
        for last, e in list(self.held):
            if current in MOVES or tick >= last:
                self.held.remove((last, e))
                if tick < self.final_tick:
                    fired = Event(tick, e.kind, e.arg)
                    self.trial.schedule.append(fired)    # shown where it really happened
                    self.trial.schedule.sort(key=lambda ev: ev.tick)
                    self.apply(fired)
        for e in self.by_tick.get(tick, []):
            if (e.kind in ("stop_word", "stop_button") and tick < self.final_tick
                    and self.wait_rng.random() < STOP_WAITS_FOR_MOVE):
                self.trial.schedule.remove(e)
                self.held.append((tick + round(STOP_WAIT_S / DT), e))
            else:
                self.apply(e)

    def apply(self, e: Event) -> None:
        if e.kind == "instruct":
            text, reply = e.arg
            self.stats["instructions"] += 1
            running = self.status in ("executing", "waiting")
            i0 = len(self.link.sent)
            posture0, obstacles0, pose0 = self.sim.posture, self.obstacles(), self.pose()
            self.guarded("submit_text", self.session.submit_text, text)
            self.n_instructions += 1
            if running:
                self.stats["switches_mid_run"] += 1
                if self.first_sent_after(i0) != str(STOP_MOVE):
                    self.flag("P5", f"new instruction while {self.status}: first command was "
                                    f"{self.first_sent_after(i0)}")
            formula = reply.get("formula") if isinstance(reply, dict) else None
            self.episode = Episode(text, formula, self.now, posture0, obstacles0,
                                   len(self.link.sent), pose0)
            self.stopped = self.quiet_from = None
        elif e.kind in ("stop_word", "stop_button"):
            self.stats["stops"] += 1
            if self.session.snapshot()["current"] in MOVES:
                self.stats["stops_mid_move"] += 1
            i0 = len(self.link.sent)
            if e.kind == "stop_word":
                self.guarded("submit_text", self.session.submit_text, e.arg)
            else:
                self.guarded("stop", self.session.stop)
            if self.first_sent_after(i0) != str(STOP_MOVE):
                self.flag("P5", f"{e.kind}: first command after it was {self.first_sent_after(i0)}")
            self.stopped = (len(self.link.sent), self.pose())
            self.episode = self.quiet_from = None
        elif e.kind in ("box_front", "box_back"):
            if len(self.sim.boxes) >= MAX_BOXES:
                self.sim.clear_boxes()
            self.sim.place_box("front" if e.kind == "box_front" else "back", e.arg)
            self.stats["boxes"] += 1
        elif e.kind == "clear":
            self.sim.clear_boxes()
        self.snapshot()

    # ---- after each tick ----
    def observe(self, tick: int) -> None:
        before = self.status
        snap = self.snapshot()
        status = snap["status"]
        new = _new_lines(self.log, snap["log"])
        self.log = snap["log"]
        if status != before and status in ("done", "error", "waiting"):
            self.stats[status] += 1
        self.stats["held_moves"] += sum(line.startswith("一時停止") for line in new)
        ep = self.episode
        if ep is not None and not ep.checked:
            for line in new:
                if line.startswith("監督器: ") and ep.start_posture is None:
                    # the session plans from the posture it reads in this very tick (R27, R28)
                    ep.start_posture = self.sim.posture
                    if ep.start_posture not in POSTURE_PROP:
                        self.flag("BUG-C2-2", f"'{ep.text}' planned while the fake Go2 is "
                                              f"{ep.start_posture} (error_code {self.sim.error_code}); "
                                              f"steps: {self.trial.steps_until(self.now)}")
                if line.startswith("イベント: "):
                    self.stats["events"] += 1
                    ep.events.append((self.now, line[len("イベント: "):], self.sim.posture,
                                      self.pose(), self.obstacles(), len(self.link.sent)))
            if status == "done":
                ep.checked = True
                self.check_done(ep)
        # P5: after a stop, nothing but StopMove and no movement until the next instruction
        if self.stopped is not None:
            idx, pose = self.stopped
            for s in self.link.sent[idx:]:
                if s.api_id != STOP_MOVE:
                    self.flag("P5", f"after a stop the session sent {s.api_id} at t={s.t:.1f}s")
            if max(abs(a - b) for a, b in zip(self.pose(), pose)) > 1e-6:
                self.flag("P5", f"the Go2 moved after a stop: {pose} -> {self.pose()}")
                pose = self.pose()
            self.stopped = (len(self.link.sent), pose)
        # P8: after done / error, silence until the next instruction or stop
        if self.quiet_from is not None and len(self.link.sent) > self.quiet_from:
            self.flag("P8", f"after done/error the session sent "
                            f"{[s.api_id for s in self.link.sent[self.quiet_from:]]}")
        if status in ("done", "error"):
            self.quiet_from = len(self.link.sent)
        if tick > self.final_tick:
            self.final_seen.append(status)

    def check_done(self, ep: Episode) -> None:
        """P7 (the session's own events) and P7b (what the fake Go2 really did) on a done episode."""
        if ep.formula is None:
            self.flag("P7", f"'{ep.text}' ended done without a formula")
            return
        self.trial.done_formulas[ep.formula] += 1
        if ep.start_posture is None:
            self.flag("harness", f"'{ep.text}' ended done without a 監督器 line")
            return
        if ep.start_posture not in POSTURE_PROP:
            return                                   # already flagged BUG-C2-2 when it started
        # Session's rule: the plan starts from the posture once the Go2 is ready (R27, R28),
        # which is also what the fake Go2 really was then.
        start = frozenset({POSTURE_PROP[ep.start_posture]}) | ep.obstacles0
        claimed = [start]
        for _, ev, *_ in ep.events:
            label = label_of_event(ev)
            if label is None:
                self.flag("harness", f"cannot read the label of event {ev}")
                return
            claimed.append(label)
        if not holds(ep.formula, claimed):
            self.flag("P7", f"'{ep.formula}' done but false on the session's events "
                            f"{[ev for _, ev, *_ in ep.events]}; steps: {self.trial.steps_until(self.now)}")
        # what the fake Go2 did: its posture, the commands it accepted, how far it moved
        truth = [start]
        ignored = [False]
        idx_prev, pose_prev = ep.sent0, ep.pose0
        did = []
        for _, ev, posture, pose, obstacles, idx in ep.events:
            action = ev.split("_to_", 1)[0]
            window = self.link.sent[idx_prev:idx]
            done = did_action(action, window, pose_prev, pose)
            props = {POSTURE_PROP[posture]} if posture in POSTURE_PROP else set()
            if done and action in ACTION_PROP:
                props.add(ACTION_PROP[action])
            truth.append(frozenset(props) | obstacles)
            ignored.append(any(s.api_id in ACTION_API.get(action, ()) and not s.accepted
                               for s in window))
            did.append(f"{action}{'' if done or action not in ACTION_PROP else '(not done)'}->{posture}")
            idx_prev, pose_prev = idx, pose
        if holds(ep.formula, truth):
            return
        # Which known bug explains it: put back the session's belief where it is the bug's own.
        ignored_cmds = [c if ign else t for t, c, ign in zip(truth, claimed, ignored)]
        if any(ignored) and holds(ep.formula, ignored_cmds):
            kind = "BUG-C2-1"
        else:
            kind = "P7b"
        self.flag(kind, f"'{ep.formula}' done at t={self.now:.1f}s but the fake Go2 did "
                        f"{did or 'nothing'} from {ep.posture0} (instruction at t={ep.t0:.1f}s); "
                        f"steps: {self.trial.steps_until(self.now)}")

    # ---- the run ----
    def run(self) -> Trial:
        for e in self.by_tick.get(0, []):
            self.apply(e)
        settled_tick = None
        for tick in range(1, round(TRIAL_S / DT) + 1):
            self.now = round(tick * DT, 6)
            self.link.advance(self.now)
            if self.sim.collided:
                self.flag("P1", f"collided at pose {self.pose()} boxes {self.sim.boxes}; "
                                f"steps: {self.trial.steps_until(self.now)}")
            self.guarded("tick", self.session.tick, self.now)
            self.ticked = True
            self.observe(tick)
            self.feed(tick)
            if tick > self.final_tick and settled_tick is None and self.status in ("done", "error"):
                settled_tick = tick
            if settled_tick is not None and tick - settled_tick >= round(QUIET_AFTER_S / DT):
                break
        self.check_end()
        self.trial.stats = self.stats
        return self.trial

    def check_end(self) -> None:
        for s in self.link.sent:
            if s.api_id == MOVE and s.accepted and s.posture in (LYING, SITTING):
                self.flag("P2", f"Move accepted at t={s.t:.1f}s while {s.posture}")
            if s.api_id == MOVE and s.x > 0 and (s.front != "CLEAR"
                                                 or s.gap_front < SAFE_GAP - GEOMETRY_TOLERANCE):
                self.flag("P3", f"forward Move x={s.x} at t={s.t:.1f}s with front {s.front}, "
                                f"box {s.gap_front:.3f} m ahead")
            if s.api_id == MOVE and s.x < 0 and (s.back != "CLEAR"
                                                 or s.gap_back < SAFE_GAP - GEOMETRY_TOLERANCE):
                self.flag("P3", f"backward Move x={s.x} at t={s.t:.1f}s with back {s.back}, "
                                f"box {s.gap_back:.3f} m behind")
        seen = self.final_seen
        if not any(s in ("done", "error", "waiting") for s in seen):
            self.flag("P6", f"after the last instruction the status was only {sorted(set(seen))}; "
                            f"steps: {self.trial.steps_until(self.now)}")
        elif seen[-1] not in ("done", "error"):
            self.flag("P6", f"the last instruction is still '{seen[-1]}' after {len(seen) * DT:.0f}s; "
                            f"steps: {self.trial.steps_until(self.now)}")
        if len(self.llm.prompts) != self.n_instructions:
            self.flag("harness", f"{len(self.llm.prompts)} LLM calls for {self.n_instructions} instructions")


# ---------------------------------------------------------------- fuzz tests
@pytest.fixture(scope="module")
def trials(tmp_path_factory) -> list[Trial]:
    return [Runner(seed, tmp_path_factory.mktemp(f"seed{seed}")).run() for seed in SEEDS]


def _assert_none(trials, prop):
    found = [v for tr in trials for v in tr.violations[prop]]
    assert not found, f"{PROPERTIES[prop]}: {len(found)}\n" + "\n".join(found[:6])


def test_fuzz_reaches_the_interesting_cases(trials):
    """The random runs really reach the situations the properties are about."""
    total = {k: sum(tr.stats[k] for tr in trials) for k in trials[0].stats}
    assert len(trials) >= 30
    assert total["instructions"] >= 250 and total["stops"] >= 60 and total["boxes"] >= 100, total
    assert total["events"] >= 200, total            # the session's "イベント:" lines were read
    assert total["done"] >= 100 and total["error"] >= 50 and total["waiting"] >= 5, total
    assert total["held_moves"] >= 5 and total["stops_mid_move"] >= 5, total
    assert total["switches_mid_run"] >= 50, total
    done = sum((tr.done_formulas for tr in trials), Counter())
    assert set(done) == set(FORMULAS), done         # P7 / P7b saw every formula end done
    _assert_none(trials, "harness")


@pytest.mark.parametrize("prop", ["P1", "P2", "P3", "P4", "P5", "P6", "P7", "P7b", "P8"])
def test_property_holds_in_every_trial(trials, prop):
    _assert_none(trials, prop)


def test_fuzz_no_ignored_command_counted_as_done(trials):
    _assert_none(trials, "BUG-C2-1")


def test_fuzz_unknown_posture_not_taken_as_lying(trials):
    _assert_none(trials, "BUG-C2-2")


# ---------------------------------------------------------------- the bugs, step by step
class Precondition(Exception):
    """The scene the bug needs was not reached: a plain failure, not the expected xfail."""


def require(cond: bool, what: str) -> None:
    if not cond:
        raise Precondition(what)


def make(tmp_path, *formulas):
    link = DirectSimLink(SimGo2())
    llm = FakeLLM([{"formula": f, "unsupported": []} for f in formulas])
    return Session(link, llm, None, tmp_path, background=False), link


def drive(session, link, seconds):
    """Advance the simulated time: link.advance(now) then session.tick(now), every DT."""
    end = link.now + seconds
    while link.now < end - 1e-9:
        now = round(link.now + DT, 6)
        link.advance(now)
        session.tick(now)


# BUG-C2-1 (fixed, R28): the new plan's first command waits until the interrupted gesture / sit
# has had its planned time, so the Go2 takes it and 終わりました is true.
@pytest.mark.parametrize("first,busy_api,second,expected_api", [
    ("F hello", HELLO, "F stretched", STRETCH),     # hello: 3 s in the Go2, new instruction after 0.8 s
    ("F sitting", SIT, "F hello", HELLO),           # sit: 2 s in the Go2, new instruction after 0.8 s
])
def test_bug_c2_1_command_ignored_by_busy_go2_is_reported_done(tmp_path, first, busy_api,
                                                              second, expected_api):
    sess, link = make(tmp_path, first, second)
    sess.submit_text("一つ目の指示"); drive(sess, link, 5.0)    # stand ends at 4.1 s, then the action
    require(link.sim.accepted[-1] == busy_api, f"{busy_api} is running in the Go2: {link.sim.accepted}")
    n = len(link.sim.accepted)
    sess.submit_text("二つ目の指示"); drive(sess, link, 15.0)
    snap = sess.snapshot()
    assert snap["status"] == "done" and expected_api in link.sim.accepted[n:], (
        f"status {snap['status']} ({snap['message']}); the Go2 should have accepted {expected_api}: "
        f"accepted after the instruction {link.sim.accepted[n:]}, posture {link.sim.posture}")


# BUG-C2-2 (fixed, R27): after a stop during stand the Go2 is in Standing Lock (1002, UNKNOWN);
# the session sends BalanceStand, plans from standing, and F lying really lies down.
def test_bug_c2_2_standing_lock_is_taken_as_lying(tmp_path):
    sess, link = make(tmp_path, "F hello", "F lying")
    sess.submit_text("挨拶して"); drive(sess, link, 1.0)          # StandUp accepted at 0.1 s
    sess.submit_text("止まって"); drive(sess, link, 3.0)          # StandUp still ends: 1002 at 1.6 s
    require(link.sim.error_code == 1002 and sess.snapshot()["posture"] == UNKNOWN,
            f"Standing Lock reached: error_code {link.sim.error_code}")
    sess.submit_text("伏せて"); drive(sess, link, 10.0)
    snap = sess.snapshot()
    assert snap["status"] == "done" and link.sim.posture == LYING, (
        f"status {snap['status']} ({snap['message']}); the Go2 is {link.sim.posture} "
        f"(error_code {link.sim.error_code}); accepted {link.sim.accepted}")
