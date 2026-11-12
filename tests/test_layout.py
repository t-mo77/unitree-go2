"""Layout files gazebo/layouts/<name>.json and their loader voice_ltl/layout.py (open-objects spec §12.5).

Footprints are written in model axes (as measured on the model); the loader turns them into world axes
once (Entry.world_size / world_offset / centre), and every user takes them from there.
"""

import ast
import copy
import dataclasses
import json
import sys
from pathlib import Path

import pytest

from voice_ltl import layout, objects

ROOT = Path(__file__).resolve().parent.parent

GOOD = {"version": 1, "objects": [
    {"id": "box_1", "model": "primitive:box", "x": 2.5, "y": 1.5, "yaw_deg": 0,
     "footprint": {"offset": [0, 0], "size": [0.5, 0.5]}, "height": 0.5, "detectable": False, "accept": ["box"]},
    {"id": "sofa_1", "model": "model://Sofa", "x": -4.4, "y": 0.0, "yaw_deg": 0, "z": 0.0,
     "footprint": {"offset": [0.02, 0], "size": [1.044, 1.8]}, "height": 0.943, "detectable": True,
     "accept": ["sofa"]},
]}
FOLDERS = frozenset({"Sofa"})


def entry_with(**kw):
    base = layout.Entry(id="bookshelf_1", label="bookshelf", model="model://Bookshelf", x=0.0, y=0.0, yaw_deg=0,
                        offset=(0.0, 0.0), size=(0.9, 0.3), height=1.8, z=0.0, detectable=False,
                        accept=("bookshelf",))
    return dataclasses.replace(base, **kw)


def test_default_layout_equals_the_objects_default_layout():
    assert layout.load("default").objs() == objects.DEFAULT_LAYOUT


def test_default_layout_entries():
    lay = layout.load("default")
    assert lay.name == "default" and [e.id for e in lay.entries] == ["box_1", "chair_1"]
    box, chair = lay.entries
    assert (box.model, box.label, box.x, box.y, box.yaw_deg, box.offset, box.size, box.height, box.z,
            box.detectable, box.accept) == ("primitive:box", "box", 2.5, 1.5, 0, (0.0, 0.0), (0.5, 0.5), 0.5,
                                             0.0, False, ("box",))
    assert (chair.model, chair.x, chair.y, chair.size, chair.height, chair.detectable, chair.accept) == \
        ("primitive:chair", 2.5, -1.5, (0.45, 0.45), 0.9, False, ("chair",))
    assert box.folder is None and chair.folder is None
    assert lay.find("chair_1") is chair and lay.find("chair_2") is None


def test_world_axes_footprint():
    e = entry_with(x=1.0, y=2.0, yaw_deg=90, offset=(0.19, 0.0), size=(0.9, 0.3))
    assert e.world_size() == (0.3, 0.9) and e.world_offset() == pytest.approx((0.0, 0.19))
    assert e.centre() == pytest.approx((1.0, 2.19)) and e.obj().size == (0.3, 0.9)
    assert entry_with(yaw_deg=180, offset=(0.02, 0.0)).world_offset() == pytest.approx((-0.02, 0.0))


@pytest.mark.parametrize("yaw_deg,size,offset", [(0, (0.9, 0.3), (0.19, 0.05)), (90, (0.3, 0.9), (-0.05, 0.19)),
                                                 (180, (0.9, 0.3), (-0.19, -0.05)), (270, (0.3, 0.9), (0.05, -0.19)),
                                                 (-90, (0.3, 0.9), (0.05, -0.19)), (360, (0.9, 0.3), (0.19, 0.05))])
def test_every_quarter_turn(yaw_deg, size, offset):
    e = entry_with(x=1.0, y=-1.0, yaw_deg=yaw_deg, offset=(0.19, 0.05), size=(0.9, 0.3))
    assert e.world_size() == size and e.world_offset() == pytest.approx(offset)
    assert e.obj() == objects.Obj("bookshelf_1", "bookshelf", *e.centre(), size)
    assert e.obj().x == pytest.approx(1.0 + offset[0]) and e.obj().y == pytest.approx(-1.0 + offset[1])


def test_model_folder():
    assert entry_with(model="model://Sofa").folder == "Sofa"
    assert entry_with(model="primitive:box").folder is None


def test_a_good_layout_parses():
    lay = layout.parse(copy.deepcopy(GOOD), "test", folders=FOLDERS)
    sofa = lay.find("sofa_1")
    assert sofa.folder == "Sofa" and sofa.offset == (0.02, 0.0) and sofa.size == (1.044, 1.8)
    assert sofa.obj() == objects.Obj("sofa_1", "sofa", -4.4 + 0.02, 0.0, (1.044, 1.8))
    assert sofa.centre() == pytest.approx((-4.38, 0.0))
    assert all(isinstance(v, float) for e in lay.entries for v in (e.x, e.y, e.z, e.height, *e.offset, *e.size))


def _set(path, value):
    def change(d):
        *head, last = path
        for k in head:
            d = d[k]
        d[last] = value
    return change


def _drop(path):
    def change(d):
        *head, last = path
        for k in head:
            d = d[k]
        del d[last]
    return change


BAD = {
    "an id whose label is not a kind": (_set(("objects", 0, "id"), "spaceship_1"), "spaceship_1"),
    "a duplicate id": (lambda d: d["objects"].append(copy.deepcopy(d["objects"][0])), "box_1"),
    "primitive:table": (_set(("objects", 0, "model"), "primitive:table"), "box_1"),
    "model://Unknown": (_set(("objects", 1, "model"), "model://Unknown"), "sofa_1"),
    "yaw_deg 45": (_set(("objects", 0, "yaw_deg"), 45), "box_1"),
    "size 0": (_set(("objects", 0, "footprint", "size"), [0, 0.5]), "box_1"),
    "size 3.1": (_set(("objects", 1, "footprint", "size"), [3.1, 0.5]), "sofa_1"),
    "height 0": (_set(("objects", 0, "height"), 0), "box_1"),
    "accept without its own label": (_set(("objects", 1, "accept"), ["chair"]), "sofa_1"),
    "accept with an unknown label": (_set(("objects", 1, "accept"), ["sofa", "spaceship"]), "sofa_1"),
    "an unknown key": (_set(("objects", 0, "colour"), "red"), "box_1"),
    "version 2": (_set(("version",), 2), None),
    "z on a primitive": (_set(("objects", 0, "z"), 0.1), "box_1"),
}


@pytest.mark.parametrize("bad", ["an id whose label is not a kind", "a duplicate id", "primitive:table",
    "model://Unknown", "yaw_deg 45", "size 0", "size 3.1", "height 0", "accept without its own label",
    "accept with an unknown label", "an unknown key", "version 2", "z on a primitive"])
def test_loader_refuses(bad):
    change, obj_id = BAD[bad]
    data = copy.deepcopy(GOOD)
    change(data)
    with pytest.raises(layout.LayoutError) as e:
        layout.parse(data, "test", folders=FOLDERS)
    assert str(e.value).startswith("gazebo/layouts/test.json: ") and "\n" not in str(e.value)
    if obj_id is not None:
        assert f": {obj_id}: " in str(e.value)


@pytest.mark.parametrize("change", [
    _set(("objects", 0, "model"), "primitive:chair"),          # a primitive of another label
    _set(("objects", 1, "model"), "model://"), _set(("objects", 1, "model"), "model://So/fa"),
    _set(("objects", 1, "model"), "sofa"), _set(("objects", 0, "id"), "box_0"), _set(("objects", 0, "id"), "box_100"),
    _set(("objects", 0, "id"), "box_01"), _set(("objects", 0, "id"), 7), _set(("objects", 0, "x"), "2.5"),
    _set(("objects", 0, "y"), float("nan")), _set(("objects", 0, "yaw_deg"), True), _set(("objects", 0, "yaw_deg"), 90.5),
    _set(("objects", 0, "footprint", "size"), [0.5]), _set(("objects", 0, "footprint", "size"), [0.5, -0.5]),
    _set(("objects", 0, "footprint", "offset"), [0, "0"]), _set(("objects", 0, "footprint", "colour"), 1),
    _drop(("objects", 0, "footprint", "offset")), _set(("objects", 0, "footprint"), [0.5, 0.5]),
    _set(("objects", 0, "height"), 3.1), _set(("objects", 0, "height"), True), _set(("objects", 1, "z"), "0"),
    _set(("objects", 0, "detectable"), 1), _set(("objects", 0, "accept"), []), _set(("objects", 0, "accept"), "box"),
    _set(("objects", 0, "accept"), ["box", "box"]), _drop(("objects", 0, "height")), _drop(("objects", 0, "accept")),
    _set(("objects",), {}), _set(("objects", 0), "box_1"), _set(("version",), True), _drop(("version",)),
    _set(("extra",), 1),
])
def test_loader_refuses_more(change):
    data = copy.deepcopy(GOOD)
    change(data)
    with pytest.raises(layout.LayoutError):
        layout.parse(data, "test", folders=FOLDERS)


@pytest.mark.parametrize("data", [[], None, "default", {"version": 1}, {"objects": []}])
def test_loader_refuses_a_file_that_is_not_a_layout(data):
    with pytest.raises(layout.LayoutError):
        layout.parse(data, "test", folders=FOLDERS)


def test_errors_name_the_file_the_id_and_the_value():
    data = copy.deepcopy(GOOD)
    data["objects"][1]["yaw_deg"] = 45
    with pytest.raises(layout.LayoutError) as e:
        layout.parse(data, "test", folders=FOLDERS)
    assert str(e.value).startswith("gazebo/layouts/test.json: sofa_1: yaw_deg 45: ")
    data["objects"][1]["yaw_deg"] = 0
    data["objects"][1]["model"] = "model://Sofa2"
    with pytest.raises(layout.LayoutError) as e:
        layout.parse(data, "test", folders=FOLDERS)
    assert "sofa_1" in str(e.value) and "model://Sofa2" in str(e.value) and "docker/fuel_models.json" in str(e.value)


def test_model_folders_default_to_the_fuel_list():
    data = copy.deepcopy(GOOD)
    assert layout.parse(data, "test").find("sofa_1").folder == "Sofa"          # Sofa is in docker/fuel_models.json
    assert layout.parse({"version": 1, "objects": []}, "empty").entries == ()


def test_load_reads_the_file_and_reports_json_errors(tmp_path, monkeypatch):
    monkeypatch.setattr(layout, "LAYOUTS_DIR", tmp_path)
    (tmp_path / "good.json").write_text(json.dumps(GOOD), encoding="utf-8")
    (tmp_path / "comma.json").write_text('{"version": 1,\n "objects": [],\n}', encoding="utf-8")
    (tmp_path / "twice.json").write_text('{"version": 1, "version": 1, "objects": []}', encoding="utf-8")
    assert layout.names() == ("comma", "good", "twice")
    assert layout.load("good").find("sofa_1").size == (1.044, 1.8)
    with pytest.raises(layout.LayoutError) as e:
        layout.load("comma")
    assert "comma.json" in str(e.value) and "line 3 column 1" in str(e.value)
    with pytest.raises(layout.LayoutError) as e:
        layout.load("twice")
    assert "twice.json" in str(e.value) and "version" in str(e.value)


def test_unknown_layout_names_the_valid_layouts():      # Review Focus 5
    with pytest.raises(layout.LayoutError) as e:
        layout.load("houshold")
    assert "houshold" in str(e.value) and "default" in str(e.value)


@pytest.mark.parametrize("name", ["", "../voice_ltl/vocabulary", "/etc/passwd", "default.json", "Default", None])
def test_only_layout_stems_load(name):
    with pytest.raises(layout.LayoutError):
        layout.load(name)


def test_paths_and_names():
    assert layout.ROOT == ROOT and layout.LAYOUTS_DIR == ROOT / "gazebo" / "layouts"
    assert layout.FUEL_MODELS == ROOT / "docker" / "fuel_models.json"
    assert layout.PRIMITIVES == {"primitive:box": "box", "primitive:chair": "chair"}
    assert "default" in layout.names() and layout.names() == tuple(sorted(layout.names()))
    assert issubclass(layout.LayoutError, ValueError)


def test_fuel_models_list_is_the_spec_pins():
    m = {x["folder"]: x for x in json.loads(layout.FUEL_MODELS.read_text())["models"]}
    assert {k: v["version"] for k, v in m.items()} == {"Sofa": 3, "TrashBin": 2, "LampAndStand": 3, "Table": 4, "Suitcase1": 2}
    assert m["Sofa"]["sha256"] == "6d19d1f109427626d88667de047ca80a2c175b3f8f12dc2d0e75ddb3b88a0db1"
    assert layout.model_folders() == frozenset(m)


def test_fuel_models_list_pins_every_field():          # spec §12.2, §12.7
    data = json.loads(layout.FUEL_MODELS.read_text(encoding="utf-8"))
    assert data["version"] == 1 and data["base"] == "https://fuel.gazebosim.org/1.0/OpenRobotics/models"
    by = "https://creativecommons.org/licenses/by/4.0/"
    zero = "https://creativecommons.org/publicdomain/zero/1.0/"
    assert [(x["folder"], x["version"], x["bytes"], x["sha256"], x["author"], x["licence"], x["licence_url"])
            for x in data["models"]] == [
        ("Sofa", 3, 1311028, "6d19d1f109427626d88667de047ca80a2c175b3f8f12dc2d0e75ddb3b88a0db1", "Wan Yi Seow",
         "CC BY 4.0", by),
        ("TrashBin", 2, 1045623, "ff38c8c7d7163145083e75166201a0d8c594c3170451ecf753fc77e173c00de3", "Roselle Carmen",
         "CC BY 4.0", by),
        ("LampAndStand", 3, 676942, "f8610eb71132cc511634937e17fef21f6c15d9ddc9612dbedcd45979566485eb",
         "Roselle Carmen", "CC BY 4.0", by),
        ("Table", 4, 899222, "3a720ef2a8e2f416e0ed017b2e13936f08f6f7c97829c9ea4b0d15380e36163d", "Nate Koenig",
         "CC0 1.0", zero),
        ("Suitcase1", 2, 1802850, "8e0c23b7c34cd19f443253eecf7185ca1a949f955aba278f01029775972f8fe7",
         "Wan Yi Seow", "CC0 1.0", zero),                          # H1: the zip's sha256, model.config's author
    ]
    assert all(set(x) == {"folder", "version", "bytes", "sha256", "author", "licence", "licence_url"}
               for x in data["models"])


def test_compose_passes_go2_layout_to_the_fake_go2():   # sim service only
    compose = (ROOT / "compose.yaml").read_text(encoding="utf-8")
    app = compose[compose.index("\n  app:"):compose.index("\n  sim:")]
    sim = compose[compose.index("\n  sim:"):compose.index("\n  gazebo:")]
    assert "    environment:" in sim.splitlines()
    assert '      GO2_LAYOUT: "${GO2_LAYOUT:-default}"' in sim.splitlines()
    assert "GO2_LAYOUT" not in app                          # the app gets no GO2_LAYOUT (review M8)


def test_imports_only_the_standard_library():           # the Jazzy bridge and the detector import it
    tree = ast.parse(Path(layout.__file__).read_text(encoding="utf-8"))
    absolute = {a.name.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
    absolute |= {n.module.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and n.level == 0}
    relative = {a.name for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and n.level == 1 and n.module is None
                for a in n.names}
    relative |= {n.module for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and n.level == 1 and n.module}
    assert absolute <= set(sys.stdlib_module_names) | {"__future__"}
    assert relative <= {"objects", "vocabulary"}            # both standard library only (their own tests)
    assert not any(isinstance(n, ast.ImportFrom) and n.level > 1 for n in ast.walk(tree))
