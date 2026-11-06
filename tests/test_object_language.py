"""The language side of objects (Task C): the prompt's ## Objects and ### 物, and translate() with a Scene."""

import dataclasses
import json

from voice_ltl import objects, translator
from voice_ltl.grammar import check
from voice_ltl.llm import FakeLLM
from voice_ltl.objects import DEFAULT_LAYOUT, Obj, Scene, msg_ambiguous, msg_not_found, msg_too_far
from voice_ltl.prompt import build_prompt
from voice_ltl.translator import translate
from voice_ltl.world import PROPOSITIONS

SCENE = Scene(DEFAULT_LAYOUT, (0.0, 0.0, 0.0))


def llm_prompt_of(scene):
    return build_prompt("x", scene=scene)


def example(instruction, formula, unsupported, substitutions):
    """One example as the prompt writes it: the instruction is a JSON string."""
    answer = json.dumps({"formula": formula, "unsupported": unsupported,
                         "substitutions": substitutions}, ensure_ascii=False)
    return f"指示: {json.dumps(instruction, ensure_ascii=False)}\n答え: {answer}"


def section(prompt, head, next_head):
    return prompt[prompt.index(head):prompt.index(next_head)]


def test_objects_section_sits_between_vocabulary_and_rules():
    p = build_prompt("箱の周りを一周して", scene=SCENE)
    assert p.index("## Vocabulary") < p.index("## Objects") < p.index("## Rules") < p.index("## Query")
    assert "- box_1（箱）：左前 2.9 m" in p and "- chair_1（椅子）：右前 2.9 m" in p
    assert "- near_chair_1: chair_1（椅子）の手前のマスにいて、椅子の方を向いている" in p
    assert p.index("### 物") < p.index("### 断るもの")
    assert p[p.index("## Query"):] == '## Query\n指示: "箱の周りを一周して"\n答え:'
    assert "Objects にある物は「### 物」に従います" in p and "障害物の周りを回る" in p


def test_objects_section_text_and_the_vocabulary_lines():
    p = build_prompt("x", scene=SCENE)
    assert section(p, "## Objects", "## Rules") == (
        "## Objects\n"
        "Go2 のカメラで見つけた物です。向きと距離は、指示を受けたときの Go2 から見たものです。\n"
        "- box_1（箱）：左前 2.9 m\n"
        "- chair_1（椅子）：右前 2.9 m\n\n")
    vocab = section(p, "## Vocabulary", "## Objects")
    for prop, desc in SCENE.descriptions().items():          # 6 object lines after the 15 static ones
        assert f"- {prop}: {desc}\n" in vocab
    assert vocab.index("- obstacle_back: ") < vocab.index("- near_box_1: ")
    assert vocab.count("\n- ") == len(PROPOSITIONS) + len(SCENE.propositions())


def test_no_objects_says_none_and_hidden_ones_are_marked():
    assert "## Objects\nいま見えている物はありません。" in build_prompt("x")
    assert "## Objects\nいま見えている物はありません。" in build_prompt("x", scene=Scene((), (0, 0, 0)))
    hidden = Scene((dataclasses.replace(DEFAULT_LAYOUT[0], visible=False),), (0, 0, 0))
    assert "box_1（箱）：左前 2.9 m（いまは見えていません）" in build_prompt("x", scene=hidden)


def test_at_most_8_objects_are_listed():
    # counted inside ## Objects: the line before the object examples also names 「box_9（箱）：前 2.0 m」
    scene = Scene(tuple(Obj(f"box_{k}", "box", float(k), 0.0) for k in range(1, 11)), (0, 0, 0))
    p = build_prompt("x", scene=scene)
    assert section(p, "## Objects", "## Rules").count("（箱）：") == 8
    assert "- box_8（箱）：前 8.0 m" in p and "box_9（箱）：前 9.0 m" not in p and "near_box_10" not in p


def test_prompt_with_8_objects_and_a_reask_stays_under_20000_characters():
    scene = Scene(tuple(Obj(f"chair_{k}", "chair", float(k), 0.5, visible=False) for k in range(10, 18)),
                  (0, 0, 0))
    assert len(build_prompt("あ" * 300, "F hello " * 12500, "x" * 5000, scene=scene)) < 20_000


def test_refusal_line_and_the_object_rules():
    p = build_prompt("x")
    assert ("- 物の一覧（Objects）にない場所・物・人や、まわりを見て決める動き（キッチン、ボール、一覧にない物、"
            "障害物の周りを回る、階段を上る、おいで、こっちに来て、駆け寄って、ついて来て など）。"
            "Objects にある物は「### 物」に従います。\n") in p
    rules = section(p, "### 物（Objects にある物を使う指示）\n", "### 断るもの")
    assert "near_〈id〉（その物の手前にいて、物の方を向いている）" in rules
    assert '{"said": 指示の中の言葉, "as": "左回りに 1 周"}' in rules
    assert '{"said": "2周", "as": "1 周（上限）"}' in rules
    assert "物を言わない「一周して」「その場で一周」は、今までどおりその場で回る動作です。" in rules
    assert "「右の箱」「左前の椅子」のように向きで言われたら、Objects の向きで選びます。" in rules
    assert "向きに「前」が入る物（前・左前・右前）が Objects にちょうど 1 つのときだけ" in rules
    assert rules.endswith("「前に障害物があったら」のような条件は、今までどおり obstacle_front・obstacle_back です。\n\n")


def test_object_examples_follow_the_others_with_their_scene():
    from voice_ltl.prompt import _EXAMPLES, _OBJECT_EXAMPLES, EXAMPLE_SCENE
    p = build_prompt("x")
    intro = ("（ここからの例は、Objects が「box_9（箱）：前 2.0 m」「chair_9（椅子）：右前 2.5 m」のときです。"
             "本当の指示では Objects の id を使います）")
    assert p.index(example(*_EXAMPLES[-1])) < p.index(intro) < p.index(example(*_OBJECT_EXAMPLES[0]))
    for e in _OBJECT_EXAMPLES:
        assert example(*e) in p
    assert example("箱の周りを一周して", "F around_box_9", [],
                   [{"said": "周りを一周", "as": "左回りに 1 周"}]) in p
    assert example("立って、椅子まで歩いて、挨拶して", "F(near_chair_9 & X F hello)", [], []) in p
    assert example("障害物の周りを回って", None, ["障害物の周りを回る"], []) in p
    assert [o.id for o in EXAMPLE_SCENE.listed()] == ["box_9", "chair_9"]
    assert EXAMPLE_SCENE.pose == (0.0, 0.0, 0.0)


def test_object_examples_pass_the_check_with_the_example_scene():
    from voice_ltl.prompt import _OBJECT_EXAMPLES, EXAMPLE_SCENE
    for _, formula, unsupported, subs in _OBJECT_EXAMPLES:
        if formula is not None:
            assert check(formula, PROPOSITIONS + EXAMPLE_SCENE.propositions()) == [] and unsupported == []
        else:
            assert unsupported and subs == []


def test_listed_props_pass_and_others_are_reasked():
    llm = FakeLLM([{"formula": "F near_box_2", "unsupported": []}, {"formula": "F near_box_1", "unsupported": []}])
    t = translate("箱まで歩いて", llm, scene=SCENE)
    assert t.ok and t.formula == "F near_box_1" and t.scene is SCENE
    assert any("near_box_2" in e for e in t.attempts[0].errors)
    assert "- box_1（箱）：左前 2.9 m" in llm.prompts[1] and "near_box_2" in llm.prompts[1]


def test_object_props_are_refused_without_a_scene():
    t = translate("箱まで歩いて", FakeLLM([{"formula": "F near_box_1", "unsupported": []}] * 3))
    assert not t.ok and t.scene is None and any("near_box_1" in e for e in t.attempts[0].errors)


def test_missing_object_is_not_found():                   # and no 「この指示」 sentence after it
    empty = Scene()
    t = translate("箱の周りを一周して", FakeLLM([{"formula": None, "unsupported": ["箱"]}]), scene=empty)
    assert t.message == msg_not_found("box")
    assert not t.ok and t.unsupported == ["箱"] and t.scene is empty


def test_two_boxes_are_ambiguous():                       # Review Focus 5
    two = Scene((Obj("box_1", "box", 2, 1), Obj("box_2", "box", 2, -1)), (0, 0, 0))
    t = translate("箱まで行って", FakeLLM([{"formula": None, "unsupported": ["箱"]}]), scene=two)
    assert t.message == msg_ambiguous("box", 2)
    assert "box_1（箱）：左前" in llm_prompt_of(two) and "box_2（箱）：右前" in llm_prompt_of(two)


def test_three_boxes_say_three_and_each_label_is_told_once():   # P6
    three = Scene(tuple(Obj(f"box_{k}", "box", 2, k - 2) for k in (1, 2, 3)), (0, 0, 0))
    t = translate("x", FakeLLM([{"formula": None, "unsupported": ["箱", "段ボール", "椅子"]}]), scene=three)
    assert t.message == msg_ambiguous("box", 3) + msg_not_found("chair")


def test_one_box_with_the_word_keeps_todays_message_and_others_are_joined():
    t = translate("x", FakeLLM([{"formula": None, "unsupported": ["箱を持つ", "キッチン"]}]), scene=SCENE)
    assert t.message == "「箱を持つ、キッチン」は単語帳にないので、まだできません。"
    t = translate("x", FakeLLM([{"formula": None, "unsupported": ["椅子", "キッチン"]}]), scene=Scene())
    assert t.message == msg_not_found("chair") + "「キッチン」は単語帳にないので、まだできません。"
    t = translate("x", FakeLLM([{"formula": None, "unsupported": ["箱を持つ"]}]), scene=Scene())
    assert t.message == "「箱を持つ」は単語帳にないので、まだできません。"   # holding is not an object name


def test_one_box_keeps_todays_message_and_numbers_still_get_theirs():
    t = translate("x", FakeLLM([{"formula": None, "unsupported": ["箱"]}]), scene=SCENE)
    assert t.message == "「箱」は単語帳にないので、まだできません。"
    t = translate("x", FakeLLM([{"formula": None, "unsupported": ["椅子", "0メートル"]}]))
    assert t.message.startswith(msg_not_found("chair") + "「0メートル」：この数は指定できません")
    assert "単語帳" not in t.message


def test_object_handling_words_are_never_object_items():  # R16, design §3
    for item in ("箱を押す", "箱を運ぶ", "箱を拾う", "椅子を動かす", "箱をどける", "椅子をどかす"):
        t = translate("x", FakeLLM([{"formula": None, "unsupported": [item]}]), scene=Scene())
        assert t.message == f"「{item}」は単語帳にないので、まだできません。", item


def test_words_that_only_contain_a_label_word_keep_todays_message():   # R16, review I1
    t = translate("x", FakeLLM([{"formula": None, "unsupported": ["アイスを取ってくる"]}]), scene=Scene())
    assert t.message == "「アイスを取ってくる」は単語帳にないので、まだできません。"
    for item in ("アイスを取ってくる", "アドバイスする", "かわいすぎるダンス", "ボックスステップ", "箱根に行く"):
        for scene in (None, Scene(), Scene(tuple(Obj(f"chair_{k}", "chair", 2, k) for k in (1, 2)), (0, 0, 0))):
            t = translate("x", FakeLLM([{"formula": None, "unsupported": [item]}]), scene=scene)
            assert t.message == f"「{item}」は単語帳にないので、まだできません。", (item, scene)


def test_far_object_is_refused_early():                   # P5
    far = Scene((Obj("box_1", "box", 7.0, 0.0),), (0, 0, 0))
    t = translate("箱まで歩いて", FakeLLM([{"formula": "F near_box_1", "unsupported": []}]), scene=far)
    assert not t.ok and t.message == msg_too_far("box") and t.formula is None
    assert t.scene is far and len(t.attempts) == 1


def test_the_first_far_object_in_the_formula_is_named():
    scene = Scene((Obj("box_1", "box", 7.0, 0.0), Obj("chair_1", "chair", 1.0, 1.0),
                   Obj("chair_2", "chair", 0.0, -6.5)), (0.5, 0.0, 1.0))
    t = translate("x", FakeLLM([{"formula": "F(near_chair_1 & X F(near_chair_2 & X F around_box_1))",
                                 "unsupported": []}]), scene=scene)
    assert t.message == msg_too_far("chair")
    t = translate("x", FakeLLM([{"formula": "F near_box_1", "unsupported": []}]),
                  scene=dataclasses.replace(scene, pose=(1.5, 0.0, 0.0)))
    assert t.ok                                            # 5.5 m from the pose, not from the origin


def test_far_objects_are_left_to_the_grid_without_a_pose():
    no_pose = Scene((Obj("box_1", "box", 7.0, 0.0),))
    t = translate("x", FakeLLM([{"formula": "F near_box_1", "unsupported": []}]), scene=no_pose)
    assert t.ok and t.scene is no_pose


def test_without_a_scene_nothing_changes():
    t = translate("x", FakeLLM([{"formula": "F hello", "unsupported": []}]))
    assert t.ok and t.scene is None


def test_hiragana_handling_verbs_are_never_object_items():   # R16, wave-2 addendum J3
    for item in ("箱をもつ", "箱をもって", "箱もって", "箱をおす", "箱をおして", "箱をはこぶ", "いすをはこんで",
                 "箱をひろう", "箱をひろって", "椅子をうごかす", "椅子をうごかして", "箱をどける", "箱をどけて",
                 "椅子をどかす", "椅子をどかして", "箱をもってきて", "箱をおしてください"):
        for scene in (Scene(), Scene((Obj("box_1", "box", 2, 1), Obj("box_2", "box", 2, -1),
                                      Obj("chair_1", "chair", 2, 1.5), Obj("chair_2", "chair", 2, -1.5)), (0, 0, 0))):
            t = translate("x", FakeLLM([{"formula": None, "unsupported": [item]}]), scene=scene)
            assert t.message == f"「{item}」は単語帳にないので、まだできません。", (item, scene)


def test_words_that_only_look_like_handling_verbs_keep_the_object_message():
    # もっと (more), おすわり (sit), おすすめ, おもって (think), なおして (fix) hold no handling verb
    for item, label in (("もっと箱に近づく", "box"), ("箱の前でおすわり", "box"), ("椅子のそばでおすわりして", "chair"),
                        ("おすすめの箱", "box"), ("箱だとおもって", "box"), ("椅子をなおして", "chair"),
                        ("箱のまわりをもっと回る", "box")):
        t = translate("x", FakeLLM([{"formula": None, "unsupported": [item]}]), scene=Scene())
        assert t.message == msg_not_found(label), item


def test_polite_negative_and_imperative_handling_verbs_are_never_object_items():   # R16, review J Minor 2
    for item in ("箱をおします", "箱をもちます", "箱をおすな", "箱をおさないで", "箱をはこべ", "箱をもちあげて",
                 "箱をもつの", "箱をはこびました", "箱をひろいません", "椅子をうごかさないで", "箱をもたないで",
                 "箱をどけます", "椅子をどけろ", "椅子をどかせ", "箱をひろえ", "箱をおせ", "いすをもちあげる"):
        for scene in (Scene(), Scene((Obj("box_1", "box", 2, 1), Obj("box_2", "box", 2, -1),
                                      Obj("chair_1", "chair", 2, 1.5), Obj("chair_2", "chair", 2, -1.5)), (0, 0, 0))):
            t = translate("x", FakeLLM([{"formula": None, "unsupported": [item]}]), scene=scene)
            assert t.message == f"「{item}」は単語帳にないので、まだできません。", (item, scene)


def test_other_words_around_the_handling_verbs_keep_the_object_message():
    # でもって (and then), とおす / とおして (pass through), にもつ (luggage), もちろん (of course),
    # おしえて (tell), おしまい (the end), おもて (front), ひろい (wide), おしい (close) hold no handling verb
    for item, label in (("でもって箱まで", "box"), ("それでもって箱まで", "box"), ("箱のあいだをとおして", "box"),
                        ("箱のあいだをとおします", "box"), ("椅子のあいだをとおさないで", "chair"),
                        ("にもつの横の箱", "box"), ("箱とにもつのあいだ", "box"), ("もちろん箱まで", "box"),
                        ("箱のことをおしえて", "box"), ("おしまいに箱を回る", "box"), ("おもての箱", "box"),
                        ("ひろい部屋の箱", "box"), ("おしいところの椅子", "chair")):
        t = translate("x", FakeLLM([{"formula": None, "unsupported": [item]}]), scene=Scene())
        assert t.message == msg_not_found(label), item


def test_handling_verbs_share_the_word_start_of_the_label_words():   # one boundary rule (objects.word_start)
    assert translator.word_start is objects.word_start
