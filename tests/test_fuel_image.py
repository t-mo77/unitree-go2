"""The thin Gazebo image with the pinned Fuel models (open-objects spec §12.2, §12.6; plan Task 2.6).

docker/Dockerfile.gazebo-fuel, docker/fetch_fuel_models.py (download, sha256, unpack) and
gazebo/tools/model_extent.py (a model's collision extent in model axes). Nothing here downloads or builds:
the downloads are replaced by a fake fetch, and the zips and models are tiny synthetic ones.
"""

import ast
import hashlib
import importlib.util
import io
import json
import struct
import subprocess
import sys
import zipfile
from pathlib import Path

import pytest

from gazebo.tools import model_extent

ROOT = Path(__file__).resolve().parent.parent
DOCKERFILE = ROOT / "docker" / "Dockerfile.gazebo-fuel"
FETCH = ROOT / "docker" / "fetch_fuel_models.py"
FUEL_MODELS = ROOT / "docker" / "fuel_models.json"
MODELS = ROOT / "tests" / "fixtures" / "models"
EXTENT = ROOT / "gazebo" / "tools" / "model_extent.py"

SDF = b'<?xml version="1.0"?><sdf version="1.9"><model name="m"><link name="l"/></model></sdf>'
CONFIG = b'<?xml version="1.0"?><model><name>m</name><sdf version="1.9">model.sdf</sdf></model>'


def _load_fetch():
    # docker/ is a build context, not a package: load the script by its path (dataclasses need sys.modules)
    spec = importlib.util.spec_from_file_location("fetch_fuel_models", FETCH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


fetch = _load_fetch()


def make_zip(path: Path, members: dict) -> Path:
    """A zip at path with the given members (name -> bytes; a name ending in / is a directory entry)."""
    with zipfile.ZipFile(path, "w") as zf:
        for name, data in members.items():
            zf.writestr(zipfile.ZipInfo(name), data)
    return path


def zip_bytes(members: dict) -> bytes:
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as zf:
        for name, data in members.items():
            zf.writestr(zipfile.ZipInfo(name), data)
    return buf.getvalue()


def every_file(root: Path) -> list:
    return sorted(str(p.relative_to(root)) for p in root.rglob("*"))


def write_list(path: Path, models: list) -> Path:
    path.write_text(json.dumps({"version": 1, "base": "https://fuel.example.test/1.0/Owner/models",
                                "models": models}), encoding="utf-8")
    return path


# --- docker/Dockerfile.gazebo-fuel ---------------------------------------------------------------------------

def test_dockerfile_pins_the_base_and_reads_the_list():
    text = DOCKERFILE.read_text(encoding="utf-8")
    lines = [line for line in text.splitlines() if line.strip() and not line.startswith("#")]
    assert lines == [
        "ARG BASE=go2-voice-ltl-gazebo:patch-0005",
        "FROM ${BASE}",
        "COPY fuel_models.json fetch_fuel_models.py /opt/go2_fuel/",
        "RUN python3 /opt/go2_fuel/fetch_fuel_models.py /opt/go2_fuel/fuel_models.json /opt/go2_models",
        "ENV GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH:+${GZ_SIM_RESOURCE_PATH}:}/opt/go2_models",
    ]
    header = [line for line in text.splitlines() if line.startswith("#")]
    assert text.startswith("#") and len(header) >= 2
    joined = "\n".join(header)
    assert "go2-voice-ltl-gazebo:patch-0005-fuel2" in joined                 # the tag (R-H3g: the room fix)
    assert ("docker build --build-arg BASE=go2-voice-ltl-gazebo:patch-0005 -t go2-voice-ltl-gazebo:patch-0005-fuel2 "
            "-f docker/Dockerfile.gazebo-fuel docker") in joined             # the build command (plan H1)
    assert "-t go2-voice-ltl-gazebo:patch-0005-fuel1" not in joined          # fuel1 is kept, never built over
    for name in ("fuel_models.json", "fetch_fuel_models.py"):               # the COPY sources are in the context
        assert (ROOT / "docker" / name).is_file()


# --- verify ----------------------------------------------------------------------------------------------------

def test_verify_refuses_a_mismatch_and_a_null_pin():
    data = b"zip bytes"
    good = hashlib.sha256(data).hexdigest()
    assert fetch.verify(data, good) is None
    with pytest.raises(fetch.FuelError, match="sha256"):
        fetch.verify(data + b"!", good)                                  # one more byte
    with pytest.raises(fetch.FuelError, match="no sha256 pin"):
        fetch.verify(data, None)                                         # Suitcase1 until H1 pins it
    for bad in (good.upper(), good[:-1], good + "0", "", 0, ["x"]):
        with pytest.raises(fetch.FuelError):
            fetch.verify(data, bad)
    assert issubclass(fetch.FuelError, ValueError)


# --- extract ---------------------------------------------------------------------------------------------------

def test_extract_refuses_zip_slip_and_strips_one_folder(tmp_path):
    dest = tmp_path / "models"
    for bad in ("../evil.txt", "Sofa/../../evil.txt", "/etc/evil.txt", "\\evil.txt", "C:/evil.txt",
                "Sofa/meshes/..\\..\\evil.txt"):
        z = make_zip(tmp_path / "bad.zip", {"model.config": CONFIG, "model.sdf": SDF, bad: b"x"})
        with pytest.raises(fetch.FuelError, match="unsafe path"):
            fetch.extract(z, dest, "Sofa")
        assert not dest.exists()                                         # nothing written, inside or outside
        assert every_file(tmp_path) == ["bad.zip"]
    # one top-level folder (its name need not be the folder's) is stripped
    z = make_zip(tmp_path / "wrapped.zip", {"Sofa_v3/": b"", "Sofa_v3/model.config": CONFIG,
                                            "Sofa_v3/model.sdf": SDF, "Sofa_v3/meshes/": b"",
                                            "Sofa_v3/meshes/Sofa.obj": b"v 0 0 0\n"})
    out = fetch.extract(z, dest, "Sofa")
    assert out == dest / "Sofa"
    assert every_file(out) == ["meshes", "meshes/Sofa.obj", "model.config", "model.sdf"]
    assert (out / "model.sdf").read_bytes() == SDF and (out / "meshes" / "Sofa.obj").read_bytes() == b"v 0 0 0\n"
    # a zip with the files at its top (as Fuel serves them) is unpacked as it is
    z = make_zip(tmp_path / "flat.zip", {"model.config": CONFIG, "model.sdf": SDF, "meshes/Table.obj": b"v 1 2 3\n",
                                         "thumbnails/1.png": b"png"})
    out = fetch.extract(z, dest, "Table")
    assert every_file(out) == ["meshes", "meshes/Table.obj", "model.config", "model.sdf", "thumbnails",
                               "thumbnails/1.png"]
    # an existing folder is never mixed with a new unpack
    with pytest.raises(fetch.FuelError, match="already exists"):
        fetch.extract(z, dest, "Table")
    # the folder is one plain name
    for folder in ("", "../Sofa", "a/b", ".hidden", "So fa"):
        with pytest.raises(fetch.FuelError, match="folder"):
            fetch.extract(z, tmp_path / "other", folder)
    assert not (tmp_path / "other").exists()


def test_extract_requires_model_files(tmp_path):
    dest = tmp_path / "models"
    cases = {
        "no model.sdf": {"model.config": CONFIG, "meshes/a.obj": b""},
        "no model.config": {"model.sdf": SDF},
        "two folders deep": {"a/b/model.config": CONFIG, "a/b/model.sdf": SDF},
        "a folder named model.sdf": {"model.config": CONFIG, "model.sdf/": b"", "model.sdf/x": b""},
        "two top folders": {"A/model.config": CONFIG, "A/model.sdf": SDF, "__MACOSX/A/._model.sdf": b""},
        "empty": {},
    }
    for why, members in cases.items():
        z = make_zip(tmp_path / "m.zip", members)
        with pytest.raises(fetch.FuelError, match="model.config and model.sdf"):
            fetch.extract(z, dest, "M")
        assert not dest.exists(), why
    (tmp_path / "junk.zip").write_bytes(b"not a zip")
    with pytest.raises(fetch.FuelError, match="not a zip"):
        fetch.extract(tmp_path / "junk.zip", dest, "M")


def test_extract_refuses_a_model_uri_of_another_folder(tmp_path):
    # Gazebo finds model://<name>/... under GZ_SIM_RESOURCE_PATH/<name>, so the folder must be that name (§12.6)
    sdf = b'<sdf version="1.6"><model name="Sofa"><link name="l"><visual name="v"><geometry><mesh>' \
          b'<uri>model://sofa/meshes/Sofa.obj</uri></mesh></geometry></visual></link></model></sdf>'
    z = make_zip(tmp_path / "m.zip", {"model.config": CONFIG, "model.sdf": sdf})
    with pytest.raises(fetch.FuelError, match="model://sofa"):
        fetch.extract(z, tmp_path / "models", "Sofa")
    good = sdf.replace(b"model://sofa/", b"model://Sofa/")
    z = make_zip(tmp_path / "m.zip", {"model.config": CONFIG, "model.sdf": good})
    assert (fetch.extract(z, tmp_path / "models", "Sofa") / "model.sdf").read_bytes() == good


def test_extract_refuses_links_and_oversized_zips(tmp_path, monkeypatch):
    z = tmp_path / "link.zip"
    with zipfile.ZipFile(z, "w") as zf:
        zf.writestr("model.config", CONFIG)
        zf.writestr("model.sdf", SDF)
        link = zipfile.ZipInfo("meshes")
        link.external_attr = (0o120777 << 16)                            # a symbolic link entry
        zf.writestr(link, "/etc")
    with pytest.raises(fetch.FuelError, match="link"):
        fetch.extract(z, tmp_path / "models", "M")
    monkeypatch.setattr(fetch, "MAX_UNPACKED_BYTES", 100)
    z = make_zip(tmp_path / "big.zip", {"model.config": CONFIG, "model.sdf": SDF, "meshes/big.obj": b"v" * 101})
    with pytest.raises(fetch.FuelError, match="unpacked"):
        fetch.extract(z, tmp_path / "models", "M")
    assert not (tmp_path / "models").exists()


# --- main ------------------------------------------------------------------------------------------------------

def test_main_downloads_verifies_and_unpacks(tmp_path, capsys):
    zips = {"Alpha": zip_bytes({"model.config": CONFIG, "model.sdf": SDF}),
            "Beta": zip_bytes({"Beta/model.config": CONFIG, "Beta/model.sdf": SDF, "Beta/meshes/b.obj": b"v 0 0 0"})}
    lst = write_list(tmp_path / "list.json", [
        {"folder": "Alpha", "version": 3, "sha256": hashlib.sha256(zips["Alpha"]).hexdigest(), "bytes": 1},
        {"folder": "Beta", "version": 12, "sha256": hashlib.sha256(zips["Beta"]).hexdigest(), "author": None},
    ])
    urls = []

    def fake_fetch(url):
        urls.append(url)
        return zips[url.rsplit("/", 1)[1][:-len(".zip")]]

    dest = tmp_path / "opt" / "go2_models"
    assert fetch.main(lst, dest, fetch=fake_fetch) == 0
    assert urls == ["https://fuel.example.test/1.0/Owner/models/Alpha/3/Alpha.zip",
                    "https://fuel.example.test/1.0/Owner/models/Beta/12/Beta.zip"]
    assert every_file(dest) == ["Alpha", "Alpha/model.config", "Alpha/model.sdf",
                                "Beta", "Beta/meshes", "Beta/meshes/b.obj", "Beta/model.config", "Beta/model.sdf"]
    out = capsys.readouterr().out
    assert "Alpha 3" in out and "Beta 12" in out


def test_a_fuel_name_apart_from_the_folder(tmp_path, capsys):
    # Construction Cone (H3 screening): Fuel serves the zip under the model's name, URL-encoded (the folder's URL
    # gives HTTP 404), and its model.sdf finds its files as model://construction_cone/..., so the folder is another
    # name (spec §12.6: "a name with a space needs the URL encoding and its own folder")
    sdf = (b'<?xml version="1.0"?><sdf version="1.5"><model name="Construction Cone"><link name="l"><visual name="v">'
           b'<geometry><mesh><uri>model://construction_cone/meshes/c.dae</uri></mesh></geometry></visual></link>'
           b'</model></sdf>')
    cone = zip_bytes({"model.config": CONFIG, "model.sdf": sdf, "meshes/c.dae": b"<COLLADA/>"})
    lst = write_list(tmp_path / "list.json", [
        {"folder": "construction_cone", "name": "Construction Cone", "version": 3,
         "sha256": hashlib.sha256(cone).hexdigest()},
        {"folder": "Beta", "name": None, "version": 1, "sha256": hashlib.sha256(cone).hexdigest()}])
    (m, beta) = fetch.read_list(lst).models
    assert (m.folder, m.name, m.fuel_name, m.version) == ("construction_cone", "Construction Cone", "Construction Cone", 3)
    assert (beta.name, beta.fuel_name) == (None, "Beta")                # no name: the folder is the Fuel name
    assert fetch.zip_url("https://f.example.test/m", m) == \
        "https://f.example.test/m/Construction%20Cone/3/Construction%20Cone.zip"
    urls = []

    def fake_fetch(url):
        urls.append(url)
        return cone

    dest = tmp_path / "models"
    assert fetch.main(write_list(tmp_path / "one.json", [json.loads(lst.read_text())["models"][0]]), dest,
                      fetch=fake_fetch) == 0
    assert urls == ["https://fuel.example.test/1.0/Owner/models/Construction%20Cone/3/Construction%20Cone.zip"]
    assert every_file(dest) == ["construction_cone", "construction_cone/meshes", "construction_cone/meshes/c.dae",
                                "construction_cone/model.config", "construction_cone/model.sdf"]
    assert "construction_cone 3" in capsys.readouterr().out
    # the folder still has to be the model:// name: a name is never a way round that check
    lst = write_list(tmp_path / "list.json", [
        {"folder": "Construction_Cone", "name": "Construction Cone", "version": 3,
         "sha256": hashlib.sha256(cone).hexdigest()}])
    assert fetch.main(lst, tmp_path / "other", fetch=lambda url: cone) != 0
    assert "model://construction_cone" in capsys.readouterr().err
    assert every_file(tmp_path / "other") == []
    # a name that points at another Fuel model: the other model's zip is fetched, and the sha256 pin stops it
    sofa = zip_bytes({"model.config": CONFIG, "model.sdf": SDF})
    lst = write_list(tmp_path / "list.json", [
        {"folder": "construction_cone", "name": "Sofa", "version": 3, "sha256": hashlib.sha256(cone).hexdigest()}])
    urls = []

    def other_model(url):
        urls.append(url)
        return sofa

    assert fetch.main(lst, tmp_path / "third", fetch=other_model) != 0
    assert urls == ["https://fuel.example.test/1.0/Owner/models/Sofa/3/Sofa.zip"]
    assert "sha256 mismatch" in capsys.readouterr().err
    assert every_file(tmp_path / "third") == []                         # nothing unpacked, the zip not kept


def test_zip_url_keeps_the_name_in_one_path_segment():
    pin = "0" * 64
    base = "https://fuel.example.test/1.0/Owner/models"
    # quote(safe=""): a slash, a query, a fragment and the rest are encoded, so the name stays one segment of the
    # Fuel path (read_list refuses such names before main gets here; this pins the encoding on its own)
    assert fetch.zip_url(base, fetch.Model("A", 1, pin, "a/b?c")) == f"{base}/a%2Fb%3Fc/1/a%2Fb%3Fc.zip"
    assert fetch.zip_url(base, fetch.Model("A", 2, pin, "//e.example/x#f&g")) == \
        f"{base}/%2F%2Fe.example%2Fx%23f%26g/2/%2F%2Fe.example%2Fx%23f%26g.zip"
    # quote leaves "." and ".." as they are (unreserved characters): a dot segment would leave the owner's path, so
    # zip_url refuses it too, and an empty name, whatever read_list did
    for name in ("", ".", ".."):
        with pytest.raises(fetch.FuelError, match="one path segment"):
            fetch.zip_url(base, fetch.Model("A", 1, pin, name))
    with pytest.raises(fetch.FuelError, match="one path segment"):
        fetch.zip_url(base, fetch.Model("..", 1, pin))                  # no name: the folder is the Fuel name


def test_main_checks_every_pin_before_downloading(tmp_path, capsys):
    calls = []
    lst = write_list(tmp_path / "list.json", [
        {"folder": "Alpha", "version": 3, "sha256": "0" * 64},
        {"folder": "Suitcase1", "version": 2, "sha256": None},
    ])
    assert fetch.main(lst, tmp_path / "models", fetch=calls.append) != 0
    assert calls == [] and not (tmp_path / "models").exists()
    err = capsys.readouterr().err
    assert "Suitcase1 2" in err and "no sha256 pin" in err


def test_main_stops_at_the_first_failure(tmp_path, capsys):
    good = zip_bytes({"model.config": CONFIG, "model.sdf": SDF})
    lst = write_list(tmp_path / "list.json", [
        {"folder": "Alpha", "version": 1, "sha256": hashlib.sha256(good + b"x").hexdigest()},
        {"folder": "Beta", "version": 1, "sha256": hashlib.sha256(good).hexdigest()},
    ])
    calls = []

    def fake_fetch(url):
        calls.append(url)
        return good

    assert fetch.main(lst, tmp_path / "models", fetch=fake_fetch) != 0
    assert len(calls) == 1                                               # Beta is never fetched
    assert "Alpha 1" in capsys.readouterr().err
    assert not (tmp_path / "models" / "Alpha").exists()
    assert every_file(tmp_path / "models") == []                         # the downloaded zip is removed too

    def broken(url):
        raise fetch.FuelError(f"{url}: HTTP 404")

    assert fetch.main(lst, tmp_path / "models", fetch=broken) != 0
    assert "HTTP 404" in capsys.readouterr().err

    no_sdf = zip_bytes({"model.config": CONFIG})                         # a right pin, a wrong zip
    lst = write_list(tmp_path / "list.json", [
        {"folder": "Gamma", "version": 1, "sha256": hashlib.sha256(no_sdf).hexdigest()}])
    assert fetch.main(lst, tmp_path / "models", fetch=lambda url: no_sdf) != 0
    assert "model.config and model.sdf" in capsys.readouterr().err
    assert every_file(tmp_path / "models") == []                         # the written zip is removed


def test_main_refuses_a_bad_list(tmp_path, capsys):
    pin = "0" * 64
    bad_lists = [
        "not json",
        json.dumps([]),
        json.dumps({"base": "http://fuel.example.test/models", "models": [{"folder": "A", "version": 1,
                                                                            "sha256": pin}]}),
        json.dumps({"base": "https://fuel.example.test/models", "models": []}),
        json.dumps({"base": "https://fuel.example.test/models", "models": [{"folder": "../A", "version": 1,
                                                                             "sha256": pin}]}),
        json.dumps({"base": "https://fuel.example.test/models", "models": [{"folder": "A", "version": True,
                                                                             "sha256": pin}]}),
        json.dumps({"base": "https://fuel.example.test/models", "models": [{"folder": "A", "version": 0,
                                                                             "sha256": pin}]}),
        json.dumps({"base": "https://fuel.example.test/models", "models": [{"folder": "A", "version": 1,
                                                                             "sha256": "abc"}]}),
        json.dumps({"base": "https://fuel.example.test/models", "models": [
            {"folder": "A", "version": 1, "sha256": pin}, {"folder": "A", "version": 2, "sha256": pin}]}),
    ]
    for text in bad_lists:
        path = tmp_path / "list.json"
        path.write_text(text, encoding="utf-8")
        calls = []
        assert fetch.main(path, tmp_path / "models", fetch=calls.append) != 0, text
        assert calls == []
        assert "list.json" in capsys.readouterr().err
    assert fetch.main(tmp_path / "missing.json", tmp_path / "models", fetch=calls.append) != 0
    # a Fuel name is one plain name: no path, URL, query, fragment or other host, and the error names the field
    bad_names = ("", " Cone", "Cone ", "a/b", "../A", "..", ".", "Cone\n", "%2e%2e", "https://evil.example/x",
                 "https://evil.example/x.zip", "//evil.example/x", "Cone?x=1", "Cone#f", "a&b", "user@evil.example",
                 "C:x", "a\\b", 3, ["A"], True)
    for name in bad_names:
        path = write_list(tmp_path / "list.json", [{"folder": "A", "name": name, "version": 1, "sha256": pin}])
        calls = []
        assert fetch.main(path, tmp_path / "models", fetch=calls.append) != 0, name
        assert calls == []
        err = capsys.readouterr().err
        assert "list.json" in err and f": A: name {name!r} is null or one Fuel model name" in err, name


def test_the_pinned_list_gives_the_fuel_urls():
    pinned = fetch.read_list(FUEL_MODELS)                               # a null pin is read; main refuses it
    assert [fetch.zip_url(pinned.base, m) for m in pinned.models] == [
        "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sofa/3/Sofa.zip",
        "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Table/4/Table.zip",
        "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Suitcase1/2/Suitcase1.zip",
        # the H3 screening's URL (HTTP 200, 622 427 B); the folder's URL .../construction_cone/3/... gave 404
        "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Construction%20Cone/3/Construction%20Cone.zip",
    ]
    assert [m.folder for m in pinned.models] == ["Sofa", "Table", "Suitcase1", "construction_cone"]


def test_fetch_cli_needs_two_arguments():
    run = subprocess.run([sys.executable, "-I", str(FETCH)], capture_output=True, text=True, timeout=60)
    assert run.returncode == 2 and "usage" in run.stderr


# --- gazebo/tools/model_extent.py ------------------------------------------------------------------------------

def test_model_extent_of_synthetic_models():
    assert model_extent.extent(MODELS / "box_pose") == {          # box 0.5 x 0.4 x 0.8 at (0.1, 0, 0.4)
        "size": [0.5, 0.4], "offset": [0.1, 0.0], "height": 0.8, "z_min": 0.0}
    assert model_extent.extent(MODELS / "obj_cube") == {          # OBJ cube 0..1 scaled 2, link turned 90°
        "size": [2.0, 2.0], "offset": [-1.0, 1.5], "height": 2.0, "z_min": -0.5}
    assert model_extent.extent(MODELS / "dae_triangles") == {     # DAE triangles in cm, nested nodes
        "size": [0.5, 1.0], "offset": [-0.15, 0.7], "height": 0.3, "z_min": 0.0}
    assert model_extent.extent(MODELS / "cylinders") == {         # a lying and a standing cylinder
        "size": [1.15, 0.2], "offset": [0.075, 0.0], "height": 0.6, "z_min": 0.0}


def _model(tmp_path: Path, geometry: str, files: dict | None = None, name: str = "m") -> Path:
    d = tmp_path / name
    d.mkdir()
    (d / "model.sdf").write_text(
        f'<?xml version="1.0"?><sdf version="1.9"><model name="{name}"><link name="l">'
        f'<collision name="c"><geometry>{geometry}</geometry></collision></link></model></sdf>', encoding="utf-8")
    for rel, data in (files or {}).items():
        (d / rel).parent.mkdir(parents=True, exist_ok=True)
        (d / rel).write_bytes(data)
    return d


def test_model_extent_reads_stl_and_spheres(tmp_path):
    tri = [(0.0, 0.0, 0.0), (0.4, 0.0, 0.0), (0.0, 0.2, 0.1)]
    binary = b"\0" * 80 + struct.pack("<I", 1) + struct.pack("<12fH", 0, 0, 1, *[c for v in tri for c in v], 0)
    d = _model(tmp_path, "<mesh><uri>model://m/meshes/t.stl</uri><scale>1 2 3</scale></mesh>",
               {"meshes/t.stl": binary})
    assert model_extent.extent(d) == {"size": [0.4, 0.4], "offset": [0.2, 0.2], "height": 0.3, "z_min": 0.0}
    ascii_stl = b"solid t\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 0.4 0 0\nvertex 0 0.2 0.1\n" \
                b"endloop\nendfacet\nendsolid t\n"
    d = _model(tmp_path, "<mesh><uri>meshes/t.STL</uri></mesh>", {"meshes/t.STL": ascii_stl}, name="a")
    assert model_extent.extent(d) == {"size": [0.4, 0.2], "offset": [0.2, 0.1], "height": 0.1, "z_min": 0.0}
    d = _model(tmp_path, "<sphere><radius>0.25</radius></sphere>", name="s")
    assert model_extent.extent(d) == {"size": [0.5, 0.5], "offset": [0.0, 0.0], "height": 0.5, "z_min": -0.25}


def test_model_extent_refuses_what_it_cannot_measure(tmp_path):
    cases = {
        "glb": ("<mesh><uri>meshes/a.glb</uri></mesh>", {"meshes/a.glb": b"glTF"}, "mesh format"),
        "plane": ("<plane><normal>0 0 1</normal><size>1 1</size></plane>", None, "plane"),
        "missing": ("<mesh><uri>meshes/none.obj</uri></mesh>", None, "none.obj"),
        "fuel url": ("<mesh><uri>https://fuel.gazebosim.org/1.0/x/models/y/1/files/meshes/a.obj</uri></mesh>",
                     None, "URI"),
        "submesh": ("<mesh><uri>meshes/a.obj</uri><submesh><name>part</name></submesh></mesh>",
                    {"meshes/a.obj": b"v 0 0 0\n"}, "submesh"),
        "empty obj": ("<mesh><uri>meshes/a.obj</uri></mesh>", {"meshes/a.obj": b"# nothing\n"}, "no vertices"),
        "y up dae": ("<mesh><uri>meshes/a.dae</uri></mesh>",
                     {"meshes/a.dae": (MODELS / "dae_triangles" / "meshes" / "tetra.dae").read_bytes()
                      .replace(b"Z_UP", b"Y_UP")}, "up_axis"),
    }
    for i, (why, (geometry, files, match)) in enumerate(cases.items()):
        d = _model(tmp_path, geometry, files, name=f"m{i}")
        with pytest.raises(model_extent.ExtentError, match=match):
            model_extent.extent(d)
    no_collision = tmp_path / "nc"
    no_collision.mkdir()
    (no_collision / "model.sdf").write_text('<sdf version="1.9"><model name="nc"><link name="l"><visual name="v">'
                                            '<geometry><box><size>1 1 1</size></box></geometry></visual></link>'
                                            '</model></sdf>', encoding="utf-8")
    with pytest.raises(model_extent.ExtentError, match="no collision"):
        model_extent.extent(no_collision)
    with pytest.raises(model_extent.ExtentError, match="model.sdf"):
        model_extent.extent(tmp_path / "absent")
    assert issubclass(model_extent.ExtentError, ValueError)


def test_model_extent_cli_prints_the_json(tmp_path):
    run = subprocess.run([sys.executable, "-I", str(EXTENT), str(MODELS / "box_pose")],
                         capture_output=True, text=True, timeout=60)
    assert run.returncode == 0, run.stderr
    assert json.loads(run.stdout) == {"size": [0.5, 0.4], "offset": [0.1, 0.0], "height": 0.8, "z_min": 0.0}
    run = subprocess.run([sys.executable, "-I", str(EXTENT), str(tmp_path / "absent")],
                         capture_output=True, text=True, timeout=60)
    assert run.returncode == 1 and "model_extent" in run.stderr and "Traceback" not in run.stderr


# --- both scripts --------------------------------------------------------------------------------------------

def test_scripts_import_only_the_standard_library():
    # fetch_fuel_models runs in the Gazebo image before anything is installed; model_extent runs there with -I
    for path in (FETCH, EXTENT):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        names = {a.name.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
        names |= {n.module.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and n.level == 0}
        assert not any(isinstance(n, ast.ImportFrom) and n.level for n in ast.walk(tree)), path
        assert names <= set(sys.stdlib_module_names) | {"__future__"}, (path, names - set(sys.stdlib_module_names))
