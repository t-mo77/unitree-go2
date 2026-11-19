import ast
import dataclasses
import json
import math
import sys
from pathlib import Path

import pytest

from voice_ltl import grammar, objects, vocabulary
from voice_ltl.objects import (
    DEFAULT_LAYOUT, DEFAULT_SIZE, KEYWORDS, Obj, Scene, around_cw_prop, around_prop, describe,
    edge_distance, format_message, formula_atoms, formula_object_props, label_in, msg_ambiguous,
    msg_not_found, msg_too_far, near_prop, parse, parse_rows, snapshot_rows, split_prop, where,
    winding_number, wrap_rad,
)
from voice_ltl.objects import (   # open object kinds (spec §8, §9, §10.10 item 13, §11.8)
    kinds_in, labels_in, ID_RE, SOURCES, core_distance, default_size, direction, footprint_angle,
    measure_radius, msg_lap_too_big, msg_not_listed, msg_too_big_here, name_of, parse_source, spine_angle,
    spine_point,
)


def row(obj_id, label, **extra):
    """A wire row at (1, 0) without a size, plus any extra keys."""
    return {"id": obj_id, "label": label, "x": 1.0, "y": 0.0, **extra}


def distance_of(scene, obj_id):
    o = scene.find(obj_id)
    return math.hypot(o.x - scene.pose[0], o.y - scene.pose[1])


def test_prop_names_and_split():
    assert (near_prop("box_1"), around_prop("chair_12"), around_cw_prop("box_1")) == \
        ("near_box_1", "around_chair_12", "around_cw_box_1")
    assert split_prop("around_cw_box_1") == ("around_cw", "box_1")
    assert split_prop("around_box_1") == ("around", "box_1")
    for bad in ("near_box_0", "near_box_100", "near_Box_1", "moved_forward", "near_sim_box_1"):
        assert split_prop(bad) is None
    assert split_prop("near_table_1") == ("near", "table_1")         # a vocabulary kind now


def test_formula_atoms():
    assert formula_atoms("F(near_chair_1 & X F hello) & G !last") == {"near_chair_1", "hello"}
    assert formula_object_props("F around_box_1 & F near_chair_1 & F hello") == {"around_box_1", "near_chair_1"}


def test_keywords_are_the_grammar_keywords():
    assert KEYWORDS == grammar.KEYWORDS


def test_parse_keeps_only_valid_rows():
    good = {"id": "box_1", "label": "box", "x": 2.5, "y": 1.5, "size": [0.5, 0.5], "seen": 3.0, "visible": True}
    rows = [good, {**good, "id": "ground_plane"}, {**good, "id": "box_2", "label": "chair"},
            {**good, "id": "box_3", "x": float("nan")}, {**good, "id": "box_4", "size": [-1, 0.5]},
            {**good, "id": "box_5", "x": True}, {**good, "id": "chair_1", "label": "chair", "size": None},
            {**good, "x": 9.0}, "not a dict"]
    objs = parse_rows(rows)
    assert [o.id for o in objs] == ["box_1", "chair_1"] and objs[0].x == 2.5
    assert objs[1].size == (0.45, 0.45)                              # the vocabulary's chair, as (L, L)
    assert parse({"frame": "map", "objects": [good]}) == () and parse([good]) == ()


def test_parse_checks_size_visible_and_seen():
    good = {"id": "box_1", "label": "box", "x": 2.5, "y": 1.5}
    rows = [{**good, "id": "box_2", "size": [0.5]}, {**good, "id": "box_3", "size": [0.5, 3.5]},
            {**good, "id": "box_4", "size": "0.5 0.5"}, {**good, "id": "box_5", "visible": "yes"},
            {**good, "id": "box_6", "seen": float("inf")}, {**good, "id": "box_7", "seen": False},
            {**good, "id": "box_8", "size": [3.0, 0.1], "visible": False, "seen": 7},
            {**good, "id": "box_9", "x": 10**400}, {**good, "id": "box_10", "seen": 10**400},
            {**good, "id": "box_11", "y": -10**400}, {**good, "id": "box_12", "size": [10**400, 0.5]}]
    assert [o.id for o in parse_rows(rows)] == ["box_8"]   # an int too big for a float drops, never raises
    huge = '{"frame":"odom","objects":[{"id":"box_1","label":"box","x":1' + "0" * 400 + ',"y":0}]}'
    assert parse(json.loads(huge)) == ()
    (o,) = parse_rows([good])
    assert (o.size, o.visible, o.seen) == (DEFAULT_SIZE, True, None)
    assert parse_rows(None) == () and parse_rows({"objects": [good]}) == ()
    assert parse({"frame": "odom", "objects": "box_1"}) == () and parse({"frame": "odom"}) == ()


def test_format_and_parse_round_trip():
    msg = format_message(DEFAULT_LAYOUT, 12.0)
    assert msg["frame"] == "odom" and msg["stamp"] == 12.0
    assert parse(json.loads(json.dumps(msg))) == DEFAULT_LAYOUT


def test_parse_rows_is_idempotent_on_wire_rows():
    hidden = dataclasses.replace(DEFAULT_LAYOUT[1], visible=False, seen=4.25)
    rows = [o.to_row() for o in (DEFAULT_LAYOUT[0], hidden)]
    assert parse_rows(rows) == (DEFAULT_LAYOUT[0], hidden)
    assert parse_rows([o.to_row() for o in parse_rows(rows)]) == parse_rows(rows)


def test_to_row_rounds_to_millimetres():
    row = Obj("box_1", "box", 1.23456, -0.00049, (0.5004, 0.25), seen=10.98765).to_row()
    assert row == {"id": "box_1", "label": "box", "x": 1.235, "y": -0.0, "size": [0.5, 0.25],
                   "seen": 10.988, "visible": True}


def test_radius_is_the_circumscribed_circle():
    assert DEFAULT_LAYOUT[0].radius == pytest.approx(0.35355, abs=1e-4)


@pytest.mark.parametrize("bearing,word", [(0, "前"), (22.4, "前"), (-22.4, "前"), (22.6, "左前"), (-22.6, "右前"),
    (90, "左"), (135, "左後ろ"), (179, "後ろ"), (-135, "右後ろ"), (-90, "右")])   # never exactly on a boundary
def test_where_sectors(bearing, word):
    b = math.radians(bearing)
    assert where(Obj("box_1", "box", 2 * math.cos(b), 2 * math.sin(b)), (0.0, 0.0, 0.0)) == f"{word} 2.0 m"


def test_where_is_relative_to_the_pose():
    box = Obj("box_1", "box", 1.0, 3.0)
    assert where(box, (1.0, 1.0, math.pi / 2)) == "前 2.0 m"          # straight ahead after a left turn
    assert where(box, (1.0, 1.0, -math.pi / 2 + 4 * math.pi)) == "後ろ 2.0 m"   # an unwrapped yaw
    assert where(box, None) is None


@pytest.mark.parametrize("pose", [(float("nan"), 0.0, 0.0), (0.0, float("nan"), 0.0), (0.0, 0.0, float("nan")),
                                  (float("inf"), 0.0, 0.0), (0.0, float("-inf"), 0.0), (0.0, 0.0, float("inf"))])
def test_where_without_a_finite_pose_is_none(pose):   # Session.snapshot() calls it at 5 Hz: never raises
    box = DEFAULT_LAYOUT[0]
    assert where(box, pose) is None
    assert describe(box, pose) == "box_1（箱）"
    assert snapshot_rows((box,), pose)[0]["where"] is None


@pytest.mark.parametrize("x,y", [(float("nan"), 1.0), (1.0, float("inf")), (float("-inf"), 1.0), (1e308, 0.0)])
def test_where_of_an_object_without_a_finite_position_is_none(x, y):   # Obj built directly, not parsed
    assert where(Obj("box_1", "box", x, y), (-1e308, 0.0, 0.0)) is None


@pytest.mark.parametrize("yaw", [1e308, -1e308, sys.float_info.max, 3e306, 1e20])
def test_where_with_a_huge_finite_yaw_never_raises(yaw):   # degrees(1e308) is inf: the yaw is wrapped first
    box = DEFAULT_LAYOUT[0]
    w = where(box, (0.0, 0.0, yaw))
    assert w is not None and w.endswith(" 2.9 m") and w.split()[0] in objects._DIRECTION_WORDS
    assert describe(box, (0.0, 0.0, yaw)).startswith("box_1（箱）：") and snapshot_rows((box,), (0.0, 0.0, yaw))


def test_where_with_a_yaw_many_turns_away_reads_the_same():
    box = Obj("box_1", "box", 1.0, 3.0)
    assert where(box, (1.0, 1.0, math.pi / 2 + 2 * math.pi * 1e6)) == "前 2.0 m"
    assert where(box, (1.0, 1.0, -math.pi / 2 - 2 * math.pi * 1e6)) == "後ろ 2.0 m"


def test_default_layout_reads_left_front_and_right_front():
    box, chair = DEFAULT_LAYOUT
    assert where(box, (0, 0, 0)) == "左前 2.9 m" and where(chair, (0, 0, 0)) == "右前 2.9 m"
    assert describe(box, (0, 0, 0)) == "box_1（箱）：左前 2.9 m" and describe(box, None) == "box_1（箱）"
    assert describe(dataclasses.replace(box, visible=False), (0, 0, 0)).endswith("（いまは見えていません）")


def test_scene_lists_the_8_nearest_and_their_props():
    objs = tuple(Obj(f"box_{k}", "box", float(k), 0.0) for k in range(1, 11))
    scene = Scene(objs, (10.0, 0.0, 0.0))
    assert [o.id for o in scene.listed()] == [f"box_{k}" for k in (10, 9, 8, 7, 6, 5, 4, 3)]
    assert scene.propositions()[:3] == ("near_box_10", "around_box_10", "around_cw_box_10")
    assert Scene(DEFAULT_LAYOUT).descriptions()["near_chair_1"] == "chair_1（椅子）の手前のマスにいて、椅子の方を向いている"


def test_scene_without_pose_lists_by_id_and_finds_among_all():
    objs = tuple(Obj(f"chair_{k}", "chair", 0.0, float(k)) for k in range(9, 0, -1)) + DEFAULT_LAYOUT[:1]
    scene = Scene(objs)
    assert [o.id for o in scene.listed()] == ["box_1"] + [f"chair_{k}" for k in range(1, 8)]
    assert scene.find("chair_9") == objs[0] and scene.find("chair_10") is None
    assert len(scene.propositions()) == 24 and set(scene.descriptions()) == set(scene.propositions())
    assert Scene().listed() == () and Scene().propositions() == () and Scene().descriptions() == {}


def test_scene_without_pose_sorts_ids_by_number_within_a_label():
    objs = tuple(Obj(f"box_{k}", "box", 0.0, float(k)) for k in (10, 2, 1)) + \
        tuple(Obj(f"chair_{k}", "chair", 0.0, -float(k)) for k in (11, 9))
    assert [o.id for o in Scene(objs).listed()] == ["box_1", "box_2", "box_10", "chair_9", "chair_11"]


def test_scene_with_pose_breaks_distance_ties_by_number():
    objs = (Obj("box_10", "box", 2.0, 0.0), Obj("box_2", "box", -2.0, 0.0), Obj("box_1", "box", 0.0, 3.0))
    assert [o.id for o in Scene(objs, (0.0, 0.0, 0.0)).listed()] == ["box_2", "box_10", "box_1"]


def test_descriptions_copy():
    d = Scene(DEFAULT_LAYOUT[:1]).descriptions()
    assert d == {"near_box_1": "box_1（箱）の手前のマスにいて、箱の方を向いている",
                 "around_box_1": "box_1（箱）の周りを左回りに 1 周し終えた",
                 "around_cw_box_1": "box_1（箱）の周りを右回りに 1 周し終えた"}


def test_label_words():
    assert label_in("箱の周りを一周") == "box" and label_in("いすまで行って") == "chair"
    assert label_in("障害物の周りを一周する") is None and label_in("この周りを一周する（周回）") is None


def test_label_words_match_only_whole_words():   # R16: 見つかりません only for the object's name
    for text in ("箱", "木箱", "段ボール", "段ボール箱", "ボックス", "ボックスまで行って"):
        assert label_in(text) == "box", text
    for text in ("椅子", "イス", "黒いイスの周り", "イスまで歩いて", "チェア", "チェアーに近づいて",
                 "いす", "いすまで", "このいすの周りを一周", "机といす",
                 "ボックス・椅子", "オフィスチェア"):    # the last word wins; オフィスチェア is a vocabulary word (spec §9.5)
        assert label_in(text) == "chair", text
    for text in ("アイスを取ってくる", "アドバイスする", "スイスに行く", "ボイスを変える", "イスラエル",   # katakana word
                 "チェアマン", "ボックスステップ", "チェックボックス", "段ボールハウス",
                 "かわいすぎるダンス", "いすわる", "ちいさいすべりだい", "はいすいこう",            # hiragana
                 "箱根に行って", "椅子取りゲーム",                                                 # kanji compound
                 "車椅子まで"):                                                                  # a not_word
        assert label_in(text) is None, text


def test_messages():                                     # exact copy, spec §9.9 and §10.6
    assert msg_not_found("box") == "「箱」が見つかりません。Go2 の前に置くか、Go2 をそちらに向けてから、もう一度言ってください。"
    assert msg_not_found("desk", "机").startswith("「机」が見つかりません。")
    assert msg_too_far("chair") == "「椅子」が遠すぎます（6 m まで）。"
    assert msg_ambiguous("box", 2, ("左前", "右前")) == \
        "「箱」が 2 つあるので、どれか決められません。「左前の箱」「右前の箱」のように言ってください。"
    assert msg_ambiguous("box", 2, ("前", "前")) == \
        "「箱」が 2 つあるので、どれか決められません。「近いほうの箱」「遠いほうの箱」のように言ってください。"
    assert msg_ambiguous("box", 4) == "「箱」が 4 つあるので、どれか決められません。「左の箱」のように言ってください。"
    assert msg_ambiguous("box", 12, ("左前", "右前", "前", "後ろ")).startswith("「箱」が 12 個あるので、")
    assert msg_not_listed("potted_plant") == "「観葉植物」はほかの物より遠いので、選べません（近い順に 8 個まで）。"
    assert msg_lap_too_big("sofa") == "「ソファ」は大きすぎて、周りを回る計画が作れません。"
    assert msg_too_big_here("bed") == "「ベッド」は大きいので、ここからは計画できません。近づいてから、もう一度言ってください。"
    assert msg_ambiguous("chair", 3) == "「椅子」が 3 つあるので、どれか決められません。「左の椅子」のように言ってください。"


def test_messages_name_the_users_word_and_pick_the_hint():   # {name} = word if given, else name_of(label)
    assert msg_too_far("chair", "いす") == "「いす」が遠すぎます（6 m まで）。"
    assert msg_not_listed("trash_can", "ごみ箱").startswith("「ごみ箱」はほかの物より遠いので")
    assert msg_ambiguous("chair", 3, ("左前", "前", "右前"), word="いす") == \
        "「いす」が 3 つあるので、どれか決められません。「左前のいす」「前のいす」「右前のいす」のように言ってください。"
    assert msg_ambiguous("box", 9).startswith("「箱」が 9 つ") and msg_ambiguous("box", 10).startswith("「箱」が 10 個")
    today = "のように言ってください。"
    for dirs in ((), ("左前",), ("左前", None), (None, None), ("左前", "前", "前"), ("左", "前", "右", "後ろ"),
                 ("前", "前", "前")):
        assert msg_ambiguous("box", 2, dirs).endswith("「左の箱」" + today), dirs
    assert msg_not_found("no_such") == "「no such」が見つかりません。Go2 の前に置くか、Go2 をそちらに向けてから、もう一度言ってください。"


def test_fixed_copy():
    assert objects.MSG_TOO_SPREAD == "物どうしが離れすぎていて、一度には計画できません"
    assert objects.MSG_NO_POSE == "Go2 の位置がわからないので、物を使う指示はできません"
    assert objects.MSG_POSITION_DRIFT == "位置がずれたので止めました。"
    assert objects.NO_PATH_REASON == "（物のまわりに通れる道がありません）"


def test_winding_and_edge_distance():
    ring = [(math.cos(t) + 1, math.sin(t)) for t in (2 * math.pi * k / 40 for k in range(41))]
    assert winding_number(ring, (1, 0)) == pytest.approx(1.0) and winding_number(ring[::-1], (1, 0)) == pytest.approx(-1.0)
    assert abs(winding_number([(0, 0), (1, 0), (2, 0)], (1, 5))) < 0.5
    box = DEFAULT_LAYOUT[0]
    assert edge_distance(2.5 + 0.25 + 0.6, 1.5, box) == pytest.approx(0.6) and edge_distance(2.5, 1.5, box) == 0.0


def test_edge_distance_to_a_corner_and_a_short_path():
    box = DEFAULT_LAYOUT[0]
    assert edge_distance(2.5 + 0.25 + 0.3, 1.5 + 0.25 + 0.4, box) == pytest.approx(0.5)
    assert winding_number([(0, 0)], (1, 1)) == 0.0 and winding_number([], (1, 1)) == 0.0


def test_snapshot_rows_shape():
    (row,) = snapshot_rows(DEFAULT_LAYOUT[:1], (0, 0, 0))
    assert row == {"id": "box_1", "label": "box", "name": "箱", "x": 2.5, "y": 1.5, "size": [0.5, 0.5],
                   "visible": True, "seen": None, "where": "左前 2.9 m"}
    assert snapshot_rows(DEFAULT_LAYOUT, None)[1]["where"] is None and snapshot_rows((), None) == []


def test_snapshot_rows_keep_every_to_row_key(monkeypatch):
    (row,) = snapshot_rows(DEFAULT_LAYOUT[:1], (0, 0, 0))
    assert set(row) == set(DEFAULT_LAYOUT[0].to_row()) | {"name", "where"}
    real = Obj.to_row
    monkeypatch.setattr(Obj, "to_row", lambda self: {**real(self), "height": 0.5})   # a key added later
    (row,) = snapshot_rows(DEFAULT_LAYOUT[:1], None)
    assert row["height"] == 0.5 and set(row) == set(real(DEFAULT_LAYOUT[0])) | {"height", "name", "where"}


def test_wrap_rad_is_public_and_wraps_to_minus_pi_exclusive():
    assert objects._wrap_rad is wrap_rad                  # the old private name still works
    assert wrap_rad(math.pi) == math.pi and wrap_rad(-math.pi) == math.pi and wrap_rad(0.0) == 0.0
    assert wrap_rad(3 * math.pi) == pytest.approx(math.pi) and wrap_rad(0.5 + 4 * math.pi) == pytest.approx(0.5)
    assert wrap_rad(-0.5 - 6 * math.pi) == pytest.approx(-0.5)


def test_shared_constants():   # the values the other tasks code against (plan Global Constraints, design §6.2)
    assert objects.OBJECTS_TOPIC == "/go2/objects" and objects.FRAME == "odom"
    assert objects.OBJECTS_HZ == 2.0 and objects.OBJECTS_STALE_S == 2.0
    assert (objects.CAMERA_HFOV, objects.CAMERA_NEAR, objects.CAMERA_FAR, objects.CAMERA_HZ) == (2.094, 0.1, 8.0, 5.0)
    assert objects.CAMERA_IN_BODY == (0.32715, -0.00003, 0.04297)
    assert objects.MAX_OBJECT_DISTANCE == 6.0 and objects.MAX_PROMPT_OBJECTS == 8
    assert objects.DEFAULT_SIZE == (0.5, 0.5)


def test_imports_only_the_standard_library():
    tree = ast.parse(Path(objects.__file__).read_text(encoding="utf-8"))
    mods = {a.name.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
    mods |= {n.module.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and n.module}
    assert mods <= set(sys.stdlib_module_names) | {"__future__"}


def test_ids_and_props_follow_the_vocabulary():
    assert split_prop("near_potted_plant_1") == ("near", "potted_plant_1")
    assert split_prop("around_cw_potted_plant_12") == ("around_cw", "potted_plant_12")
    assert split_prop("around_cw_box_1") == ("around_cw", "box_1") and split_prop("near_table_1") == ("near", "table_1")
    for bad in ("near_box_1_2", "near_box_01", "near_box_100", "near_box_0", "near_sim_box_1", "near_Box_1",
                "near_person_1", "moved_forward"):
        assert split_prop(bad) is None, bad
    assert ID_RE.fullmatch("sim_box_1") is None and ID_RE.fullmatch("cw_box_1") is None
    assert split_prop("around_cw_fire_extinguisher_99") == ("around_cw", "fire_extinguisher_99")


def test_every_vocabulary_label_makes_ids_and_props():
    assert objects.LABELS == vocabulary.LABELS and objects.Kind is vocabulary.Kind
    for label in sorted(objects.LABELS):
        m = ID_RE.fullmatch(f"{label}_7")
        assert m is not None and m.groups() == (label, "7"), label
        for prop in ("near", "around", "around_cw"):
            assert split_prop(f"{prop}_{label}_99") == (prop, f"{label}_99"), label
    for gone in ("LABEL_JA", "LABEL_WORDS", "_LABEL_RES"):
        assert not hasattr(objects, gone), gone


def test_rows_accept_only_vocabulary_labels_and_sort_by_number():
    rows = [row("potted_plant_3", "potted_plant"), row("sim_box_1", "sim_box"), row("person_1", "person"),
            row("box_10", "box"), row("box_2", "box")]
    assert [o.id for o in parse_rows(rows)] == ["potted_plant_3", "box_10", "box_2"]
    assert sorted(["box_10", "box_2", "bean_bag_1"], key=objects._id_key) == ["bean_bag_1", "box_2", "box_10"]


def test_names_sizes_and_radius_come_from_the_vocabulary():
    assert Obj("potted_plant_1", "potted_plant", 0, 0).name == "観葉植物" and name_of("no_such") == "no such"
    assert parse_rows([{"id": "chair_1", "label": "chair", "x": 1, "y": 2}])[0].size == (0.45, 0.45)
    assert default_size("sofa") == (1.8, 1.8) and default_size("no_such") == DEFAULT_SIZE
    assert measure_radius("box") == pytest.approx(0.6536, abs=1e-4)
    assert measure_radius("sofa") == pytest.approx(1.3062, abs=1e-4) and measure_radius("ball") == 0.6
    assert measure_radius("no_such") == 0.6 and default_size("box") == (0.5, 0.5)


def test_confidence_and_measured_are_optional_and_checked():
    good = row("sofa_1", "sofa", confidence=0.71, measured=True)
    (o,) = parse_rows([good]); assert (o.confidence, o.measured) == (0.71, True)
    for bad in ({"confidence": True}, {"confidence": 1.2}, {"confidence": -0.1}, {"confidence": float("nan")},
                {"confidence": "0.5"}, {"measured": "yes"}, {"measured": 1}):
        assert parse_rows([{**good, **bad}]) == (), bad
    assert DEFAULT_LAYOUT[0].to_row() == {"id": "box_1", "label": "box", "x": 2.5, "y": 1.5, "size": [0.5, 0.5],
                                          "seen": None, "visible": True}          # byte-identical without them
    assert {"confidence", "measured"} <= set(o.to_row())
    (back,) = parse_rows([o.to_row()]); assert back == o                          # the wire keeps both
    (edge,) = parse_rows([{**good, "confidence": 0, "measured": False}])
    assert (edge.confidence, edge.measured) == (0.0, False) and edge.to_row()["measured"] is False
    (plain,) = parse_rows([row("sofa_2", "sofa")])
    assert (plain.confidence, plain.measured) == (None, None) and not {"confidence", "measured"} & set(plain.to_row())


def test_message_source():
    msg = format_message(DEFAULT_LAYOUT, 1.0, source="detector")
    assert msg["source"] == "detector" and parse_source(msg) == "detector" and parse(msg) == DEFAULT_LAYOUT
    assert "source" not in format_message(DEFAULT_LAYOUT, 1.0) and parse_source({"frame": "odom"}) is None
    assert parse({**msg, "source": "yoloe"}) == () and parse_source({**msg, "source": "yoloe"}) is None
    assert SOURCES == ("logical", "detector", "fake")
    for bad in (None, 1, ["detector"], "Detector"):
        assert parse({**msg, "source": bad}) == () and parse_source({**msg, "source": bad}) is None, bad
    assert parse_source("detector") is None and parse_source(None) is None
    assert list(format_message(DEFAULT_LAYOUT, 1.0)) == ["frame", "stamp", "objects"]   # today's message, as is
    with pytest.raises(ValueError):
        format_message(DEFAULT_LAYOUT, 1.0, source="yoloe")


def test_label_in_is_longest_and_last():                 # O1, spec §9.5
    for text, label in (("ゴミ箱", "trash_can"), ("ごみ箱", "trash_can"), ("くず箱", "trash_can"), ("本棚", "bookshelf"),
                        ("本箱", "bookshelf"), ("食器棚", "cabinet"), ("戸棚", "cabinet"), ("下駄箱", "shoe_rack"),
                        ("丸椅子", "stool"), ("椅子の上の箱", "box"), ("段ボール箱", "box"), ("木箱", "box"),
                        ("机といす", "chair"), ("観葉植物", "potted_plant"), ("オフィスチェア", "chair")):
        assert label_in(text) == label, text
    for text in ("車椅子", "家具", "家電", "人", "グリーン", "キッチン", "台所", "2階", "1本", "2台"):
        assert label_in(text) is None, text
    assert labels_in("ゴミ箱と椅子") == {"trash_can", "chair"}


def test_kinds_in_delegates_to_the_vocabulary():
    assert kinds_in("ゴミ箱と椅子") == vocabulary.kinds_in("ゴミ箱と椅子")
    assert [(k.label, k.word) for k in kinds_in("机といす")] == [("desk", "机"), ("chair", "いす")]
    assert kinds_in("") == () and labels_in("家具") == frozenset() and label_in("") is None


def test_scene_focus_lists_the_named_kind_first():
    boxes = tuple(Obj(f"box_{k}", "box", -1.5, -1.75 + 0.5 * (k - 1), (0.3, 0.3)) for k in range(1, 9))
    plant = Obj("potted_plant_1", "potted_plant", 3.0, 0.0, (0.4, 0.4))
    crowded = Scene(boxes + (plant,), (0.0, 0.0, 0.0))
    assert plant not in crowded.listed() and len(crowded.listed()) == 8
    focused = dataclasses.replace(crowded, focus=frozenset({"potted_plant"}))
    assert plant in focused.listed() and "box_8" not in [o.id for o in focused.listed()]
    assert [o.id for o in focused.listed()] == sorted((o.id for o in focused.listed()),
                                                      key=lambda i: distance_of(focused, i))   # still nearest first
    assert "near_potted_plant_1" in focused.propositions() and "near_potted_plant_1" in focused.descriptions()
    assert crowded.focus == frozenset() and Scene().focus == frozenset()


def test_scene_focus_without_a_pose_with_many_of_the_kind_and_empty():
    chairs = tuple(Obj(f"chair_{k}", "chair", 0.0, float(k)) for k in range(9, 0, -1))
    plant = Obj("potted_plant_1", "potted_plant", 9.0, 9.0)
    no_pose = Scene(chairs + (plant,), focus=frozenset({"potted_plant"}))   # no pose: id order, the kind first
    assert [o.id for o in no_pose.listed()] == [f"chair_{k}" for k in range(1, 8)] + ["potted_plant_1"]
    plants = tuple(Obj(f"potted_plant_{k}", "potted_plant", float(k), 0.0) for k in range(1, 11))
    box = Obj("box_1", "box", 0.5, 0.0)
    many = Scene(plants + (box,), (0.0, 0.0, 0.0), frozenset({"potted_plant"}))   # 10 of the kind: the nearest 8
    assert [o.id for o in many.listed()] == [f"potted_plant_{k}" for k in range(1, 9)]
    two_kinds = dataclasses.replace(many, focus=frozenset({"potted_plant", "box"}))
    assert [o.id for o in two_kinds.listed()] == ["box_1"] + [f"potted_plant_{k}" for k in range(1, 8)]
    for scene in (Scene(chairs + (plant,)), Scene(plants + (box,), (0.0, 0.0, 0.0))):   # an empty focus: today
        assert dataclasses.replace(scene, focus=frozenset()).listed() == scene.listed()
        assert dataclasses.replace(scene, focus=frozenset({"sofa"})).listed() == scene.listed()   # none of it here
    few = Scene(DEFAULT_LAYOUT, (0.0, 0.0, 0.0), frozenset({"chair"}))
    assert few.listed() == Scene(DEFAULT_LAYOUT, (0.0, 0.0, 0.0)).listed()


def test_direction_is_the_first_word_of_where():
    for o in DEFAULT_LAYOUT:
        assert direction(o, (0, 0, 0)) == where(o, (0, 0, 0)).split()[0]
    assert direction(DEFAULT_LAYOUT[0], None) is None
    assert direction(DEFAULT_LAYOUT[0], (float("nan"), 0.0, 0.0)) is None
    assert direction(Obj("box_1", "box", 1e308, 0.0), (-1e308, 0.0, 0.0)) is None
    assert direction(Obj("box_1", "box", 1.0, 3.0), (1.0, 1.0, math.pi / 2)) == "前"


def test_footprint_geometry():                           # spec §10.10 item 13
    sofa = Obj("sofa_1", "sofa", 0.0, 0.0, (2.0, 0.9))   # spine along x, e = 0.55, m = 0.45
    assert spine_point(0.3, 2.0, sofa) == (0.3, 0.0) and spine_point(3.0, 1.0, sofa) == (0.55, 0.0)
    assert core_distance(0.3, 1.5, sofa) == pytest.approx(1.05)            # beside the long side: d_k == d_e
    box = Obj("box_1", "box", 0.0, 0.0, (0.5, 0.5))
    assert core_distance(2.0, 0.0, box) == pytest.approx(2.0 - 0.25)       # square: d_k == d_c - a
    assert spine_angle(0.3, 1.5, -math.pi / 2, sofa) == pytest.approx(0.0)
    assert footprint_angle(0.0, 2.0, -math.pi / 2 + 0.3, sofa) == 0.0      # still pointing into the sofa
    assert footprint_angle(3.0, 0.0, math.pi / 2, box) == pytest.approx(math.pi / 2 - math.atan2(0.25, 2.75), abs=1e-9)
    assert footprint_angle(0.1, 0.1, 1.0, box) == 0.0                      # inside


def test_footprint_geometry_off_the_origin_and_along_y():
    tall = Obj("bed_1", "bed", 1.0, -2.0, (1.0, 2.0))                        # spine along y, e = 0.5, m = 0.5
    assert spine_point(3.0, -1.8, tall) == (1.0, -1.8) and spine_point(1.2, 5.0, tall) == (1.0, -1.5)
    assert spine_point(1.0, -9.0, tall) == (1.0, -2.5)
    assert core_distance(3.0, -1.8, tall) == pytest.approx(edge_distance(3.0, -1.8, tall))   # beside the long side
    d_e = edge_distance(2.0, 0.0, tall)                                      # off a corner: d_e <= d_k <= d_e + 0.414 m
    assert d_e < core_distance(2.0, 0.0, tall) <= d_e + (math.sqrt(2) - 1) * 0.5 + 1e-12
    assert core_distance(1.0, -2.0, tall) == pytest.approx(-0.5)            # the centre: inside the core
    assert spine_point(0.7, 0.2, box := Obj("box_1", "box", 0.5, 0.5, (0.5, 0.5))) == (0.5, 0.5)   # a square: the centre
    assert core_distance(0.5, 2.5, box) == pytest.approx(math.hypot(0.0, 2.0) - 0.25)
    assert spine_angle(3.0, -1.8, math.pi, tall) == pytest.approx(0.0)       # facing the long side straight on
    assert spine_angle(3.0, -1.8, 0.0, tall) == pytest.approx(math.pi)       # back to it
    assert spine_angle(1.0, 1.0, -math.pi / 2 + 0.2, tall) == pytest.approx(0.2)   # past the spine end: the end
    assert footprint_angle(1.0, 1.0, -math.pi / 2, tall) == 0.0 and footprint_angle(1.0, 1.0, math.pi / 2, tall) \
        == pytest.approx(math.pi - math.atan2(0.5, 2.0))                     # facing away: to the nearer hull side
    assert footprint_angle(1.5, -2.0, 0.0, tall) == 0.0                      # on the edge: d_e == 0
