"""voice_ltl/vocabulary.json and its loader voice_ltl/vocabulary.py (open-objects spec §7, §8, §9.4, §9.5)."""

import ast
import copy
import itertools
import json
import sys
from pathlib import Path

import pytest

import voice_ltl.vocabulary as V
from voice_ltl import translator, world

DATA = json.loads(V.VOCABULARY_PATH.read_text(encoding="utf-8"))


def entry(data, label):
    return next(e for e in data["labels"] if e["label"] == label)


def test_committed_file_loads():
    v = V.VOCABULARY
    assert len(v.entries) == 50 and sum(len(e.words) for e in v.entries) == 165
    assert sum(len(e.detect) for e in v.entries) == 87 and sum(len(e.en) for e in v.entries) == 44
    assert v.detect_ignore == ("person", "dog", "cat") and v.not_words == ("長椅子", "車椅子", "テレビ番組")
    assert {"box", "chair", "table", "desk", "bookshelf", "refrigerator", "trash_can", "potted_plant",
            "sofa", "lamp", "suitcase", "ball", "television"} <= V.LABELS
    assert V.name_of("trash_can") == "ゴミ箱" and V.name_of("no_such") == "no such"
    assert V.size_of("sofa") == (1.8, 0.9) and V.height_of("chair") == 0.9 and V.DETECT["couch"] == "sofa"


def test_module_names_and_limits():
    assert V.VOCABULARY_PATH == Path(V.__file__).with_name("vocabulary.json")
    assert (V.MAX_KINDS, V.MAX_SIZE, V.DETECT_WARN) == (200, 3.0, 100)
    assert V.FIXED_RESERVED == frozenset({"near", "around", "cw", "to", "last", "true", "false", "end"})
    assert V.ONE_CHAR_WORDS == frozenset("箱棚机籠鞄")
    assert V.SUFFIXES == ("", "まで", "の周り", "のところ", "の前", "を", "に", "へ")
    assert V.LABELS == V.VOCABULARY.labels and V.DETECT_IGNORE == frozenset({"person", "dog", "cat"})
    assert V.size_of("no_such") is None and V.height_of("no_such") is None
    assert V.VOCABULARY.entry("box") == V.Entry("box", "箱", ("箱", "はこ", "ボックス", "段ボール", "ダンボール"),
                                                ("cardboard box", "box"), ("carton",), (0.5, 0.5), 0.5)
    assert V.VOCABULARY.entry("no_such") is None


def test_imports_only_the_standard_library():   # the Jazzy bridge and the detector import it
    tree = ast.parse(Path(V.__file__).read_text(encoding="utf-8"))
    mods = {a.name.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
    mods |= {(n.module or "").split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)}
    assert mods <= set(sys.stdlib_module_names) | {"__future__"}


# ---- one broken copy per row; each breaks exactly one rule (spec §7.3) ----

def _version_2(d):
    d["version"] = 2


def _entry_key_word(d):
    entry(d, "chair")["word"] = ["椅子"]


def _top_level_key(d):
    d["not_word"] = ["長椅子"]


def _missing_height(d):
    del entry(d, "stool")["height"]


def _label(value):
    def mutate(d):
        entry(d, "stool")["label"] = value
    return mutate


def _without(label):
    def mutate(d):
        d["labels"].remove(entry(d, label))
    return mutate


def _ja_with_brackets(d):
    entry(d, "chair")["ja"] = "「椅子」"


def _ja_not_a_word(d):
    entry(d, "stool")["ja"] = "腰掛け"


def _ja_13_characters(d):
    e = entry(d, "bean_bag")
    e["ja"] = "ビーズクッションチェアーズ"
    e["words"].append(e["ja"])


def _word(label, word):
    def mutate(d):
        entry(d, label)["words"].append(word)
    return mutate


def _21_words(d):
    entry(d, "chair")["words"] += [f"テスト{c}" for c in "アイウエオカキクケコサシ"]


def _detect_upper(d):
    entry(d, "chair")["detect"] = ["Chair", "office chair"]


def _detect(label, prompt):
    def mutate(d):
        entry(d, label)["detect"].append(prompt)
    return mutate


def _9_en(d):
    entry(d, "potted_plant")["en"] = ["plant", "house plant", "flower pot", "plant pot", "pot plant",
                                      "indoor plant", "green plant", "foliage plant", "leafy plant"]


def _couch_on_bench(d):
    entry(d, "bench")["en"] = ["couch"]


def _set(label, key, value):
    def mutate(d):
        entry(d, label)[key] = value
    return mutate


def _not_word_hoge(d):
    d["not_words"].append("ほげ")


def _201_kinds(d):
    box = entry(d, "box")
    d["labels"] += [dict(copy.deepcopy(box), label=f"pad{i}") for i in range(1, 152)]


BROKEN = [
    pytest.param(1, _version_2, id="r1-version-2"),
    pytest.param(1, _entry_key_word, id="r1-entry-key-word"),
    pytest.param(1, _top_level_key, id="r1-top-level-key"),
    pytest.param(1, _missing_height, id="r1-missing-height"),
    pytest.param(2, _label("Box"), id="r2-Box"),
    pytest.param(2, _label("cw_box"), id="r2-cw_box"),
    pytest.param(2, _label("sim_box"), id="r2-sim_box"),
    pytest.param(2, _label("near_box"), id="r2-near_box"),
    pytest.param(2, _label("abcdefghijklmnopqrstuvwxy"), id="r2-25-characters"),
    pytest.param(2, _label("chair"), id="r2-duplicate"),
    pytest.param(2, _without("box"), id="r2-no-box"),
    pytest.param(2, _without("chair"), id="r2-no-chair"),
    pytest.param(3, _ja_with_brackets, id="r3-brackets"),
    pytest.param(3, _ja_not_a_word, id="r3-not-a-word"),
    pytest.param(3, _ja_13_characters, id="r3-13-characters"),
    pytest.param(4, _word("television", "TV"), id="r4-TV"),
    pytest.param(4, _word("box", "ア" * 21), id="r4-21-characters"),
    pytest.param(4, _word("bookshelf", "本"), id="r4-one-character"),
    pytest.param(4, _word("sofa", "椅子"), id="r4-word-of-another-kind"),
    pytest.param(4, _word("chair", "車椅子"), id="r4-not-word-as-word"),
    pytest.param(4, _21_words, id="r4-21-words"),
    pytest.param(5, _word("ball", "箱を"), id="r5-self-resolution"),
    pytest.param(7, _detect_upper, id="r7-upper-case"),
    pytest.param(7, _detect("chair", "a" * 41), id="r7-41-characters"),
    pytest.param(7, _detect("stool", "chair"), id="r7-another-kinds-prompt"),
    pytest.param(7, _detect("stool", "person"), id="r7-detect-ignore"),
    pytest.param(7, _detect("table", "kitchen table"), id="r7-4-prompts"),
    pytest.param(8, _9_en, id="r8-9-en"),
    pytest.param(8, _couch_on_bench, id="r8-alias-of-another-kind"),
    pytest.param(9, _set("box", "size", [0.5, 3.1]), id="r9-too-long"),
    pytest.param(9, _set("box", "size", [0.3, 0.5]), id="r9-short-first"),
    pytest.param(9, _set("box", "height", 0), id="r9-height-0"),
    pytest.param(10, _not_word_hoge, id="r10-holds-no-word"),
    pytest.param(11, _201_kinds, id="r11-201-kinds"),
]


@pytest.mark.parametrize("rule,mutate", BROKEN)
def test_each_rule_rejects_a_broken_copy(rule, mutate):
    data = copy.deepcopy(DATA)
    mutate(data)
    with pytest.raises(V.VocabularyError, match=rf"rule {rule}:"):
        V.parse(data)


def test_the_unbroken_copy_parses():   # so each row above fails only by its own change
    v = V.parse(copy.deepcopy(DATA))
    assert v.entries == V.VOCABULARY.entries and v.labels == V.LABELS


def test_reserved_segments_come_from_world():                         # C5: a set built from the code, no count
    want = {s for n in world.ACTION_NAMES + world.PROPOSITIONS + world.UNCONTROLLABLE for s in n.split("_")}
    assert world.RESERVED_LABEL_SEGMENTS == frozenset(want) and {"turned", "obstacle", "blocked", "stand"} <= want
    bad = copy.deepcopy(DATA)
    entry(bad, "stool")["label"] = "turned_stool"
    with pytest.raises(V.VocabularyError, match="rule 2:"):
        V.check_reserved_segments(V.parse(bad), world.RESERVED_LABEL_SEGMENTS, "world.py")


def test_no_word_reads_as_a_handling_verb():
    assert not [w for e in V.VOCABULARY.entries for w in e.words if translator._handles_object(w)]
    bad = copy.deepcopy(DATA)
    entry(bad, "cabinet")["words"].append("押し入れ")
    with pytest.raises(V.VocabularyError, match="rule 6:"):
        V.check_words(V.parse(bad), lambda w: not translator._handles_object(w), 6, "reads as a handling verb")


# From the rev-2 check of design-vocabulary.md §2.4.
RESERVED_PHRASES = tuple("""
立って 挨拶して ダンスして ダンス 座って おすわり 伏せて 寝そべって しゃがんで 伸びをして 喜んで ハートを描いて
お辞儀して おじぎ 前 後ろ 右 左 左前 右前 一周して 周りを回って 回れ右 進んで 下がって バックして バック 歩いて 走って
障害物 物 もの 何か キッチン 台所 階段を上って ジャンプ 握手 お手 おかわり こっちに来て 部屋 壁 床 家具 家電 あれ それ
1本 2台 3個 1枚 2つ 一人 二人で 1匹 1袋 1冊 2脚 本 台 人 花 木 草 球 鉢 袋 電球 地球 花火 花瓶 箱根 植物 植物園
トランプ スカート ベンチャー ボールペン かわいすぎる アイス たなばた したな はこぶ はこんで ボックスステップ
テーブルクロス デスクトップ 自転車置き場 棚卸し 机上 バケツリレー コーンスープ ランプシェード テレビ番組 長椅子 車椅子
荷物 にもつ トイレ 便器 窓 ドア カーテン 絨毯 ラグ マット クッション 布団 押し入れ 運動器具 ライト 電気 照明 スタンド
チャリティー 洗濯機置き場 冷凍庫 紙袋 ランドセル サーキュレーター オットマン 台車 はしご 梯子 踏み台 こたつ コタツ 炬燵
ちゃぶ台 座卓 衣装ケース コートハンガー ポールハンガー コート掛け 洋服掛け まり くま グリーン クーラー モニター
キーボード トランク バギー ハンガー アイロン ボード バイク タンク ケース ルンバ 腰掛け すいそう しょうかき
""".split())

EXPECTED = (
    ("本棚の本", "bookshelf"), ("椅子の上の箱", "box"), ("箱の横の椅子", "chair"), ("机といす", "chair"),
    ("ゴミ箱と椅子", "chair"), ("ゴミ箱まで", "trash_can"), ("段ボール箱", "box"), ("木箱", "box"),
    ("テレビ台", "television"), ("ドラム式洗濯機", "washing_machine"), ("液晶テレビ", "television"),
    ("石油ストーブ", "heater"), ("食器棚", "cabinet"), ("戸棚", "cabinet"), ("シューズボックス", "shoe_rack"),
    ("ダストボックス", "trash_can"), ("くずかご", "trash_can"), ("ソファベッド", "sofa"), ("ベビーベッド", "crib"),
    ("バスケットボール", "ball"), ("段ボール", "box"), ("丸椅子", "stool"), ("座椅子", "chair"), ("チェアー", "chair"),
    ("学習机", "desk"), ("ｲｽ", "chair"),
)


def test_phrase_lists_have_the_planned_sizes():
    assert len(RESERVED_PHRASES) == len(set(RESERVED_PHRASES)) == 154
    assert len(EXPECTED) == len(dict(EXPECTED)) == 26


@pytest.mark.parametrize("phrase", RESERVED_PHRASES)
def test_reserved_phrases_are_no_kind(phrase):
    assert V.kinds_in(phrase) == ()


@pytest.mark.parametrize("phrase,label", EXPECTED)
def test_expected_phrases_resolve_to_their_kind(phrase, label):
    assert V.kinds_in(phrase)[-1].label == label


def test_every_word_resolves_to_its_own_kind():          # rule 5, also checked at load
    for e in V.VOCABULARY.entries:
        for w in e.words:
            for s in V.SUFFIXES:
                assert V.kinds_in(w + s)[-1].label == e.label, (w, s)


def test_kinds_in_is_longest_and_last():
    assert [(k.label, k.word, k.start, k.end) for k in V.kinds_in("ゴミ箱と椅子")] == \
        [("trash_can", "ゴミ箱", 0, 3), ("chair", "椅子", 4, 6)]
    assert V.kinds_in("チェアーまで")[0].word == "チェア" and V.kinds_in("ｲｽ")[0].label == "chair"
    assert V.kinds_in("車椅子まで") == () and [k.label for k in V.kinds_in("観葉植物")] == ["potted_plant"]


def test_kinds_in_words_and_offsets():
    assert V.kinds_in("チェアーまで") == (V.Kind("chair", "チェア", 0, 4),)       # the ー is part of the hit
    assert V.kinds_in("ソファーの前") == (V.Kind("sofa", "ソファー", 0, 4),)      # a word that ends in ー stays
    assert V.kinds_in("車椅子と椅子") == (V.Kind("chair", "椅子", 4, 6),)         # the mask keeps the offsets
    assert V.kinds_in("") == () and V.kinds_in(None) == ()


def test_hypernyms_and_living_beings_are_not_kinds():
    for w in ("家具", "家電", "物", "もの", "障害物", "何か", "人", "グリーン", "キッチン", "台所", "2階", "1本", "2台"):
        assert V.kinds_in(w) == (), w
    assert not {"person", "dog", "cat"} & V.LABELS and all(V.lookup(x) is None for x in ("person", "dog", "cat"))


@pytest.mark.parametrize("name,label", [
    ("desk", "desk"), ("Desk", "desk"), ("dining_table", "table"), ("coffee table", "table"),
    ("potted plant", "potted_plant"), ("Potted-Plants", "potted_plant"), ("houseplant", "potted_plant"),
    ("fridge", "refrigerator"), ("couch", "sofa"), ("tv", "television"), ("bookcase", "bookshelf"),
    ("cupboard", "cabinet"), ("trash can", "trash_can"), ("trash bin", "trash_can"), ("boxes", "box"),
    ("chairs", "chair"), ("benches", "bench"), ("tables", "table"),
    ("person", None), ("dog", None), ("kitchen", None), ("sim_box", None), ("", None), ("furniture", None),
    ("cup", None), ("tv_stand", None), (42, None)])
def test_lookup(name, label):
    assert V.lookup(name) == label


def test_normalise_alias():
    assert V.normalise_alias("  Potted - Plant\t") == "potted_plant"
    assert V.normalise_alias("-trash--can-") == "trash_can" and V.normalise_alias("ＴＶ") == "tv"


def test_event_names_never_collide_for_label_pairs():   # spec §7.3 / §8, 1225 pairs
    labels = sorted(V.LABELS)
    pairs = list(itertools.combinations(labels, 2))
    assert len(pairs) == len(labels) * (len(labels) - 1) // 2
    for a, b in pairs:
        props = (f"near_{a}_1", f"around_{a}_1", f"around_cw_{a}_1", f"near_{b}_1", f"around_{b}_1", "standing")
        subsets = [frozenset(s) for k in range(4) for s in itertools.combinations(props, k)]
        names = [world.event_name(action, s) for action in ("forward", "turn_left") for s in subsets]
        assert len(set(names)) == len(names), (a, b)


def test_schema_matches_the_loader():                   # the schema is for docs and tests only
    schema = json.loads((V.VOCABULARY_PATH.parent / "vocabulary.schema.json").read_text(encoding="utf-8"))
    kind = schema["$defs"]["kind"]
    assert set(kind["required"]) == {"label", "ja", "words", "detect", "size", "height"}
    assert set(kind["properties"]) == {"label", "ja", "words", "detect", "en", "size", "height"}
    assert set(schema["properties"]) == {"version", "labels", "not_words", "detect_ignore"}
    assert kind["properties"]["label"]["pattern"] == f"^{V.LABEL_SHAPE.pattern}$"


def test_errors_name_the_file_rule_label_and_value(tmp_path):        # Review Focus 1
    bad = copy.deepcopy(DATA)
    entry(bad, "sofa")["words"].append("椅子")
    p = tmp_path / "vocabulary.json"
    p.write_text(json.dumps(bad, ensure_ascii=False), encoding="utf-8")
    with pytest.raises(V.VocabularyError) as e:
        V.load(p)
    assert all(s in str(e.value) for s in ("vocabulary.json", "rule 4", "sofa", "椅子"))
    p.write_text('{"version": 1, "labels": [],}', encoding="utf-8")
    with pytest.raises(V.VocabularyError, match=r"vocabulary\.json: rule 1: -: .*line 1 column"):
        V.load(p)


def test_error_message_form(tmp_path):
    e = V.VocabularyError("vocabulary.json", 4, "sofa", "椅子", "also a word of chair")
    assert str(e) == "vocabulary.json: rule 4: sofa: 椅子: also a word of chair" and isinstance(e, ValueError)
    assert (e.source, e.rule, e.label, e.value, e.reason) == ("vocabulary.json", 4, "sofa", "椅子",
                                                             "also a word of chair")
    with pytest.raises(V.VocabularyError, match=r"rule 1: -: .*: cannot read"):
        V.load(tmp_path / "missing.json")
    p = tmp_path / "vocabulary.json"
    p.write_text('{"version": 1, "version": 1, "labels": []}', encoding="utf-8")
    with pytest.raises(V.VocabularyError, match=r"rule 1: -: version: .*twice"):
        V.load(p)


def test_committed_file_is_the_default_path():
    assert V.load().entries == V.VOCABULARY.entries
    assert V.VOCABULARY.source == "voice_ltl/vocabulary.json"


def test_many_detect_prompts_warn(monkeypatch, caplog):
    monkeypatch.setattr(V, "DETECT_WARN", 86)
    V.parse(copy.deepcopy(DATA))
    assert "87 detect prompts" in caplog.text
