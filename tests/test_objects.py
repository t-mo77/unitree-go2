import ast
import dataclasses
import json
import math
import sys
from pathlib import Path

import pytest

from voice_ltl import grammar, objects
from voice_ltl.objects import (
    DEFAULT_LAYOUT, DEFAULT_SIZE, KEYWORDS, Obj, Scene, around_cw_prop, around_prop, describe,
    edge_distance, format_message, formula_atoms, formula_object_props, label_in, msg_ambiguous,
    msg_not_found, msg_too_far, near_prop, parse, parse_rows, snapshot_rows, split_prop, where,
    winding_number,
)


def test_prop_names_and_split():
    assert (near_prop("box_1"), around_prop("chair_12"), around_cw_prop("box_1")) == \
        ("near_box_1", "around_chair_12", "around_cw_box_1")
    assert split_prop("around_cw_box_1") == ("around_cw", "box_1")
    assert split_prop("around_box_1") == ("around", "box_1")
    for bad in ("near_box_0", "near_box_100", "near_Box_1", "moved_forward", "near_table_1"):
        assert split_prop(bad) is None


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
    assert objs[1].size == DEFAULT_SIZE
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


def test_descriptions_copy():
    d = Scene(DEFAULT_LAYOUT[:1]).descriptions()
    assert d == {"near_box_1": "box_1（箱）の手前のマスにいて、箱の方を向いている",
                 "around_box_1": "box_1（箱）の周りを左回りに 1 周し終えた",
                 "around_cw_box_1": "box_1（箱）の周りを右回りに 1 周し終えた"}


def test_label_words():
    assert label_in("箱の周りを一周") == "box" and label_in("いすまで行って") == "chair"
    assert label_in("障害物の周りを一周する") is None and label_in("この周りを一周する（周回）") is None


def test_label_words_match_only_whole_words():   # R16: 見つかりません only for the object's name
    for text in ("箱", "木箱", "段ボール", "段ボール箱", "ボックス", "ボックスまで行って", "ボックス・椅子"):
        assert label_in(text) == "box", text
    for text in ("椅子", "車椅子まで", "イス", "黒いイスの周り", "イスまで歩いて", "チェア", "チェアーに近づいて",
                 "いす", "いすまで", "このいすの周りを一周", "机といす"):
        assert label_in(text) == "chair", text
    for text in ("アイスを取ってくる", "アドバイスする", "スイスに行く", "ボイスを変える", "イスラエル",   # katakana word
                 "チェアマン", "オフィスチェア", "ボックスステップ", "チェックボックス", "段ボールハウス",
                 "かわいすぎるダンス", "いすわる", "ちいさいすべりだい", "はいすいこう",            # hiragana
                 "箱根に行って", "椅子取りゲーム"):                                                # kanji compound
        assert label_in(text) is None, text


def test_messages():   # exact copy above
    assert msg_not_found("box") == "「箱」が見つかりません。Go2 の前に置いてから、もう一度言ってください。"
    assert msg_too_far("chair") == "「椅子」が遠すぎます（6 m まで）。"
    assert msg_ambiguous("box", 2) == "「箱」が 2 つあるので、どれか決められません。「左の箱」のように言ってください。"
    assert msg_ambiguous("chair", 3) == "「椅子」が 3 つあるので、どれか決められません。「左の椅子」のように言ってください。"


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


def test_imports_only_the_standard_library():
    tree = ast.parse(Path(objects.__file__).read_text(encoding="utf-8"))
    mods = {a.name.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
    mods |= {n.module.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and n.module}
    assert mods <= set(sys.stdlib_module_names) | {"__future__"}
