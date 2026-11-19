"""The docs of the open object kinds (open-objects plan Task 4.1; spec §11.2, §12.7; rulings D5, Q4, O24, D12).

The licence facts stand in both docs/simulation.md and docs/design.md (spec §11.2, review M5): ultralytics, the
YOLOE weights and the CLIP tokenizer fork under AGPL-3.0; the MobileCLIP text encoder weights research-only and
non-commercial; adding a LICENSE file is the user's separate decision. docs/simulation.md attributes every Fuel model
that docker/fuel_models.json pins (name, version, author, URL, licence), and no other; the plain box and chair are not
recognised by the detector (Q4). README keeps the commands only (M12). No doc carries a home or temp path."""

import json
import re
from pathlib import Path
from urllib.parse import quote

ROOT = Path(__file__).resolve().parent.parent
README = ROOT / "README.md"
SIMULATION = ROOT / "docs" / "simulation.md"
DESIGN = ROOT / "docs" / "design.md"
VOCABULARY = ROOT / "docs" / "vocabulary.md"
SPEC = ROOT / "docs" / "superpowers" / "specs" / "2026-11-11-go2-open-objects-design.md"
OBJECTS_DESIGN = ROOT / "docs" / "superpowers" / "specs" / "2026-11-05-go2-objects-design.md"
FUEL = ROOT / "docker" / "fuel_models.json"

AGPL = ("検出器の container は、ultralytics と YOLOE の重みと CLIP の tokenizer（ultralytics/CLIP の fork）を、"
        "AGPL-3.0 で使います。")
MOBILECLIP = ("文字の符号化器の重み `mobileclip_blt.ts`（Apple の MobileCLIP の text encoder）は、Apple の条件で"
              "研究のため（Research Purposes）だけに使えて、商用・製品づくりには使えません（research-only, non-commercial）。")
OUT_OF_IMAGES = ("ultralytics と torch は検出器のイメージと `detector/yoloe.py` にだけあり、app のイメージにも Gazebo の"
                 "イメージにも、本来の Linux の準備にも入りません。検出器は取り替え口（`detector/protocol.py`）の後ろにあります。")
LICENSE_FILE = "リポジトリに LICENSE ファイルを足すかどうかは、利用者の別の判断です（この仕事では足していません）。"
Q4 = "ただの形の箱（box_1）と椅子（chair_1）は、検出器の目標ではなく、検出器で見つかることを当てにできません。"
REAL_ROBOT = "実機の道（PR #4 の Linux 直接実行と `compose.robot.yaml`）は、この仕事で変えていません。"
DETECTOR_UP = "OBJECTS_SOURCE=detector SIM_KIND=gazebo docker compose --profile gazebo --profile detector up -d"
# back to logical (the final review, runtime-2): stop the detector with its profile first, then start logical again;
# `up` without the detector profile leaves the detector running beside a logical Gazebo (two /go2/objects senders)
DETECTOR_DOWN_FIRST = "docker compose --profile gazebo --profile detector down  # logical に戻す前に、検出器ごと止める"
LOGICAL_AGAIN = "SIM_KIND=gazebo docker compose --profile gazebo up -d    # logical で起動し直す"
DETECTOR_BUILD = "docker compose --profile detector build detector"
GAZEBO_BUILDS = ("docker build -t go2-voice-ltl-gazebo:patch-0005 -f docker/Dockerfile.gazebo docker",
                 "docker compose --profile gazebo build gazebo")
OLD_ROOM = "GO2_LAYOUT=default"
ATTRIBUTION = "## Gazebo の物の出典"


def text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def section(doc: str, heading: str) -> str:
    start = doc.index(heading)
    nxt = doc.find("\n## ", start + len(heading))
    return doc[start:] if nxt < 0 else doc[start:nxt]


def test_licence_sentences_stand_in_both_docs():
    for path in (SIMULATION, DESIGN):
        doc = text(path)
        for sentence in (AGPL, MOBILECLIP, OUT_OF_IMAGES, LICENSE_FILE):
            assert sentence in doc, (path.name, sentence)
        assert "利用者は研究に使うことを承知しています（O24）" in doc, path.name
    sim = text(SIMULATION)
    for alternative in ("prompt-free の YOLOE", "OWLv2", "Grounding DINO"):
        assert alternative in sim, alternative


def test_every_pinned_fuel_model_is_attributed_and_no_other():
    pins = json.loads(text(FUEL))
    rows = [line for line in section(text(SIMULATION), ATTRIBUTION).splitlines()
            if line.startswith("| ") and "fuel.gazebosim.org" in line]
    assert len(rows) == len(pins["models"])
    for model in pins["models"]:
        name = model.get("name", model["folder"])
        url = f"{pins['base']}/{quote(name)}/{model['version']}"
        row = next((r for r in rows if f"「{name}」" in r), None)
        assert row is not None, name
        for part in (f"版 {model['version']}", url, model["licence"], model["licence_url"]):
            assert part in row, (name, part)
        for author in model["author"].split(", "):
            assert author in row, (name, author)
    attribution = section(text(SIMULATION), ATTRIBUTION)
    assert "Construction%20Cone" in attribution and "Wan Yi Seow" in attribution
    assert "ファイルは変えず、名前・位置・向き・static だけを指定して置いています。" in attribution
    for gone in ("TrashBin", "LampAndStand"):                  # left the room (R-H3h): not in the table
        assert not any(gone in r for r in rows), gone
        assert gone in attribution                              # ... but the section says why


def test_the_detector_does_not_see_the_plain_box_and_chair():
    for path in (SIMULATION, VOCABULARY, DESIGN):
        assert Q4 in text(path), path.name


def test_commands_for_the_household_room_and_the_detector():
    readme, sim = text(README), text(SIMULATION)
    for doc in (readme, sim):
        assert DETECTOR_UP in doc and DETECTOR_BUILD in doc and OLD_ROOM in doc and "GO2_LAYOUT" in doc
        for build in GAZEBO_BUILDS:
            assert build in doc, build
        assert doc.index(GAZEBO_BUILDS[0]) < doc.index(GAZEBO_BUILDS[1])
        assert DETECTOR_DOWN_FIRST in doc and LOGICAL_AGAIN in doc
        assert doc.index(DETECTOR_UP) < doc.index(DETECTOR_DOWN_FIRST) < doc.index(LOGICAL_AGAIN)


def test_the_real_robot_path_is_unchanged_in_the_docs():
    for path in (README, SIMULATION, DESIGN):
        assert REAL_ROBOT in text(path), path.name


def test_the_objects_message_example_names_source_and_measured():
    sim = text(SIMULATION)
    example = next(line for line in sim.splitlines() if '"frame": "odom"' in line and '"objects"' in line)
    assert '"source": "logical"' in example and '"measured": true' in example


def test_the_specs_point_to_each_other_and_carry_the_measured_room():
    assert "2026-11-11-go2-open-objects-design.md" in text(OBJECTS_DESIGN)
    spec = text(SPEC)
    for words in ("traffic_cone_1", "Construction Cone", "(−1.62, −1.14)", "(−4.6, 0)", "(−0.3, −4.45)",
                  "−166.6〜−172.4°", "−95.1〜−97.6°", "16 の見え方", "4.51 GB", "Wan Yi Seow"):
        assert words in spec, words


# The voice plan's recording step names the agent temp folder twice on one line (line 941). It was on main before this
# work and is left alone: the one known exception, pinned by its count so that a new temp path there still fails.
KNOWN_TEMP = {"docs/superpowers/plans/2026-11-04-voice-ltl-go2.md": 2}


def test_no_doc_keeps_a_fill_marker():
    """Task 4.1 left `<!-- fill:… -->` markers for numbers still being measured (H6, then H6b); all are filled."""
    for path in [README, *sorted((ROOT / "docs").rglob("*.md"))]:
        assert "<!-- fill:" not in text(path), path.relative_to(ROOT).as_posix()


def test_no_doc_carries_a_home_or_temp_path():
    home = re.compile(r"/Users/[A-Za-z0-9_.-]|/private/tmp/|/var/folders/")
    temp = re.compile(r"/tmp/claude-\d+/|claude-\d+/-Users-")      # the agent temp folder and its scratchpad paths
    for path in [README, *sorted((ROOT / "docs").rglob("*.md"))]:
        name = path.relative_to(ROOT).as_posix()
        assert not home.search(text(path)), name
        assert len(temp.findall(text(path))) == KNOWN_TEMP.get(name, 0), name
