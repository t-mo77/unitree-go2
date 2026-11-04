from go2_nl_control.actions import LYING, SITTING, STANDING

from voice_ltl.world import (
    ACTION_NAMES, PROP_DESCRIPTION_JA, PROPOSITIONS, PlantState, event_action,
    go2_label, go2_sensor_events, go2_step, go2_world, step_value,
)


def test_go2_world_counts():
    w = go2_world(PlantState(LYING, False, False))
    assert len(w.states) == 12 and w.states[0] == PlantState(LYING, False, False)
    assert len(w.transitions) == 80
    assert sum(1 for t in w.transitions if t[3] == "c") == 56
    assert sum(1 for t in w.transitions if t[3] == "u") == 24
    assert len({t[1] for t in w.transitions}) == 60
    assert set(w.marked) == set(w.states)


def test_labels_and_events():
    s = PlantState(STANDING, True, False)
    assert go2_label(s, "hello") == {"standing", "hello", "obstacle_front"}
    ev, dst = go2_step(PlantState(LYING, False, False), "stand")
    assert dst == PlantState(STANDING, False, False) and event_action(ev) == "stand"
    assert go2_step(PlantState(LYING, False, False), "hello") is None
    assert go2_step(PlantState(SITTING, False, False), "forward") is None
    assert event_action("front_clear") is None


def test_sensor_events():
    evs = dict(go2_sensor_events(PlantState(STANDING, False, True)))
    assert evs == {"front_blocked": PlantState(STANDING, True, True),
                   "back_clear": PlantState(STANDING, False, False)}


def test_vocabulary_is_safe_and_complete():
    assert len(PROPOSITIONS) == 15 and set(PROP_DESCRIPTION_JA) == set(PROPOSITIONS)
    assert not {"flip", "jump", "handstand"} & set(ACTION_NAMES)
    assert step_value("forward") == 0.5 and step_value("turn_left") == 90.0 and step_value("hello") == 0
    assert all(type(step_value(a)) is float for a in ACTION_NAMES)


def test_forward_is_in_the_plant_even_when_blocked_but_forbidden():
    # The supervisor forbids moving into an obstacle; the plant itself still has the event.
    w = go2_world(PlantState(STANDING, True, False))
    forward = [t for t in w.transitions if t[0] == w.initial and event_action(t[1]) == "forward"]
    assert len(forward) == 1 and forward[0][3] == "c"
    assert forward[0][1] in w.forbidden
    assert w.event_label[forward[0][1]] >= {"obstacle_front", "moved_forward"}
