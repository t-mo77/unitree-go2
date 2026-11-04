"""Go2 vocabulary and the environment automaton (posture x front/back obstacle)."""

from dataclasses import dataclass

from go2_nl_control.actions import ACTIONS, LYING, SITTING, STANDING

from voice_ltl.automata import World

ACTION_NAMES = (
    "stand", "lie_down", "sit", "rise_sit", "hello", "stretch", "content",
    "heart", "scrape", "dance", "forward", "backward", "turn_left", "turn_right",
)

ACTION_PROP = {
    "stand": None, "lie_down": None, "sit": None, "rise_sit": None,
    "hello": "hello", "stretch": "stretched", "content": "content",
    "heart": "heart", "scrape": "scraped", "dance": "danced",
    "forward": "moved_forward", "backward": "moved_backward",
    "turn_left": "turned_left", "turn_right": "turned_right",
}

POSTURE_PROP = {LYING: "lying", STANDING: "standing", SITTING: "sitting"}
SENSOR_PROPS = ("obstacle_front", "obstacle_back")

PROPOSITIONS = (
    tuple(POSTURE_PROP.values())
    + tuple(p for p in ACTION_PROP.values() if p)
    + SENSOR_PROPS
)

PROP_DESCRIPTION_JA = {
    "lying": "伏せている",
    "standing": "立っている",
    "sitting": "おすわりしている",
    "hello": "挨拶した",
    "stretched": "伸びをした",
    "content": "喜んだ",
    "heart": "ハートを描いた",
    "scraped": "おじぎをした",
    "danced": "ダンスをした",
    "moved_forward": "前へ進んだ",
    "moved_backward": "後ろへ下がった",
    "turned_left": "左へ向きを変えた",
    "turned_right": "右へ向きを変えた",
    "obstacle_front": "前に障害物がある",
    "obstacle_back": "後ろに障害物がある",
}

UNCONTROLLABLE = ("front_blocked", "front_clear", "back_blocked", "back_clear")
SAFETY_FORMULA = "G(!(obstacle_front & moved_forward)) & G(!(obstacle_back & moved_backward))"

STEP_DISTANCE = 0.5  # [m] per forward/backward step
STEP_ANGLE = 90.0    # [deg] per turn step
MAX_REPEAT = 6


@dataclass(frozen=True)
class PlantState:
    posture: str
    front: bool
    back: bool


def go2_label(state: PlantState, action: str | None) -> frozenset[str]:
    label = {POSTURE_PROP[state.posture]}
    if action is not None and ACTION_PROP[action]:
        label.add(ACTION_PROP[action])
    if state.front:
        label.add("obstacle_front")
    if state.back:
        label.add("obstacle_back")
    return frozenset(label)


def event_name(action: str, label: frozenset[str]) -> str:
    return f"{action}_to_" + "_".join(sorted(label))


def event_action(event: str) -> str | None:
    if event in UNCONTROLLABLE:
        return None
    return event.split("_to_", 1)[0]


def go2_step(state: PlantState, action: str) -> tuple[str, PlantState] | None:
    """Event name and next state, or None when the action cannot run here."""
    dst_posture = ACTIONS[action].transitions.get(state.posture)
    if dst_posture is None or dst_posture not in POSTURE_PROP:
        return None
    dst = PlantState(dst_posture, state.front, state.back)
    return event_name(action, go2_label(dst, action)), dst


def go2_sensor_events(state: PlantState) -> list[tuple[str, PlantState]]:
    front = ("front_clear", PlantState(state.posture, False, state.back)) if state.front \
        else ("front_blocked", PlantState(state.posture, True, state.back))
    back = ("back_clear", PlantState(state.posture, state.front, False)) if state.back \
        else ("back_blocked", PlantState(state.posture, state.front, True))
    return [front, back]


def step_value(action: str) -> float:
    if action in ("forward", "backward"):
        return STEP_DISTANCE
    if action in ("turn_left", "turn_right"):
        return STEP_ANGLE
    return 0


def go2_world(initial: PlantState) -> World:
    states = [initial]
    transitions = []
    event_label: dict[str, frozenset[str]] = {}
    i = 0
    while i < len(states):
        src = states[i]
        i += 1
        outgoing = []
        for action in ACTION_NAMES:
            step = go2_step(src, action)
            if step is not None:
                ev, dst = step
                event_label[ev] = go2_label(dst, action)
                outgoing.append((src, ev, dst, "c"))
        outgoing += [(src, ev, dst, "u") for ev, dst in go2_sensor_events(src)]
        for _, _, dst, _ in outgoing:
            if dst not in states:
                states.append(dst)
        transitions += outgoing
    return World(
        states=states,
        initial=initial,
        transitions=transitions,
        marked=list(states),
        event_label=event_label,
        initial_label=go2_label(initial, None),
        propositions=PROPOSITIONS,
    )
