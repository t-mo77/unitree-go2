"""The rendered front camera and its bridge, only with OBJECTS_SOURCE=detector (open-objects Task 2.9; spec §2 goal 7,
§11.3, §11.4; design-detector.md §4.1).

With OBJECTS_SOURCE=logical (the default) the robot description and the main ros_gz_bridge arguments stay
byte-identical to 0f6e06a, so today's RTF and e2e records stay valid. The launch file is run here against stand-in
launch / launch_ros / xacro modules, so these tests need no ROS 2 Jazzy. Two kinds of test skip in the app image:
the ones that read 0f6e06a with git (the test containers mount the worktree without its git folder; the pins below
are checked wherever git can read the commit) and the ones that run the real xacro (the gazebo image only).
"""

import ast
import hashlib
import importlib.util
import subprocess
import sys
import types
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from detector.camera import CAMERA_CLIP, CAMERA_H, CAMERA_HFOV, CAMERA_RATE_HZ, CAMERA_W, CameraModel
from gazebo import bridge_core
from gazebo import launch_config
from gazebo.launch_config import (CAMERA_BRIDGE_ARGS, CAMERA_BRIDGE_NAME, GZ_BRIDGE_ARGS, camera_bridge_args,
                                  xacro_mappings)

ROOT = Path(__file__).resolve().parent.parent
XACRO = ROOT / "gazebo" / "go2_sensors.urdf.xacro"
LAUNCH = ROOT / "gazebo" / "launch" / "go2_gazebo.launch.py"
XACRO_NS = "{http://www.ros.org/wiki/xacro}"

BASE = "0f6e06a"     # origin/main before the open-objects work (spec §2 goal 7)
# The six arguments of the main parameter_bridge in 0f6e06a's gazebo/launch/go2_gazebo.launch.py (MODEL "robot",
# WORLD "empty"), and the sha256 of 0f6e06a's gazebo/go2_sensors.urdf.xacro. test_pins_are_those_of_0f6e06a checks both.
BRIDGE_ARGS_0F6E06A = (
    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
    "/gazebo/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
    "/gazebo/logical_camera@ros_gz_interfaces/msg/LogicalCameraImage[gz.msgs.LogicalCameraImage",
    "/model/robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
    "/world/empty/create@ros_gz_interfaces/srv/SpawnEntity",
    "/world/empty/remove@ros_gz_interfaces/srv/DeleteEntity",
)
XACRO_0F6E06A_SHA256 = "92666ba2cfac30b45ce61378cc1513e2ceb6bbc1e1c56c1f47e9eb30fe518812"

# What the camera adds to the xacro text, exactly
CAMERA_ARG = '<xacro:arg name="camera" default="false"/>'
CAMERA_IF_OPEN = '<xacro:if value="$(arg camera)">'
CAMERA_IF_CLOSE = "</xacro:if>"


def _at_base(rel: str) -> str:
    """The text of rel at BASE, or skip where git cannot read the commit."""
    try:
        out = subprocess.run(["git", "-c", "safe.directory=*", "-C", str(ROOT), "show", f"{BASE}:{rel}"],
                             capture_output=True, timeout=60, check=False)
    except (OSError, subprocess.TimeoutExpired) as e:
        pytest.skip(f"git cannot run here: {e}")
    if out.returncode != 0:
        pytest.skip(f"git cannot read {BASE} here (a worktree mounted without its git folder)")
    return out.stdout.decode("utf-8")


def _string(node: ast.AST, names: dict) -> str:
    """A str literal or an f-string over module-level str constants."""
    if isinstance(node, ast.Constant) and isinstance(node.value, str):
        return node.value
    if isinstance(node, ast.JoinedStr):
        parts = []
        for v in node.values:
            if isinstance(v, ast.Constant):
                parts.append(v.value)
            elif (isinstance(v, ast.FormattedValue) and isinstance(v.value, ast.Name) and v.conversion == -1
                  and v.format_spec is None):
                parts.append(names[v.value.id])
            else:
                raise AssertionError(ast.dump(v))
        return "".join(parts)
    raise AssertionError(ast.dump(node))


def _bridge_args_in(source: str) -> list:
    """The arguments of every parameter_bridge Node(...) in a launch file's source, read with ast."""
    tree = ast.parse(source)
    names = {t.id: n.value.value for n in tree.body if isinstance(n, ast.Assign)
             for t in n.targets if isinstance(t, ast.Name)
             if isinstance(n.value, ast.Constant) and isinstance(n.value.value, str)}
    found = []
    for call in ast.walk(tree):
        if not (isinstance(call, ast.Call) and isinstance(call.func, ast.Name) and call.func.id == "Node"):
            continue
        kw = {k.arg: k.value for k in call.keywords}
        if isinstance(kw.get("executable"), ast.Constant) and kw["executable"].value == "parameter_bridge":
            found.append(tuple(_string(e, names) for e in kw["arguments"].elts))
    return found


def _without_camera(text: str) -> str:
    """The xacro text with the camera argument and the camera block cut out (xacro drops them and nothing else when
    camera is false: no space or comment was added around them)."""
    assert text.count(CAMERA_ARG) == 1 and text.count(CAMERA_IF_OPEN) == 1
    start = text.index(CAMERA_IF_OPEN)
    end = text.index(CAMERA_IF_CLOSE, start) + len(CAMERA_IF_CLOSE)
    return (text[:start] + text[end:]).replace(CAMERA_ARG, "")


def _camera_block(root: ET.Element = None) -> ET.Element:
    """The xacro's only xacro:if (of the given parse of the xacro, or a new one)."""
    root = ET.parse(XACRO).getroot() if root is None else root
    blocks = list(root.iter(XACRO_NS + "if"))
    assert [b.get("value") for b in blocks] == ["$(arg camera)"]
    return blocks[0]


# ---- the launch file against stand-in ROS modules ----
class _Made:
    """Stands in for a launch / launch_ros class: keeps the arguments it was made with."""

    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs


class _Record:
    def __init__(self):
        self.nodes, self.xacro_calls, self.source_calls = [], [], []


def _load_launch(monkeypatch, source):
    """Load gazebo/launch/go2_gazebo.launch.py against stand-in ROS modules, with OBJECTS_SOURCE=source (None: unset).
    Returns the module and the record of the Nodes made, the xacro calls and the objects_source calls."""
    rec = _Record()

    class Node(_Made):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            rec.nodes.append(self)

    class Doc:
        def __init__(self, mappings):
            self.mappings = mappings

        def toxml(self):
            return f"<robot camera mappings {sorted(self.mappings.items())}/>"

    def process_file(path, **kwargs):
        rec.xacro_calls.append((path, kwargs))
        return Doc(dict(kwargs.get("mappings") or {}))

    fakes = {
        "xacro": {"process_file": process_file},
        "ament_index_python": {},
        "ament_index_python.packages": {"get_package_share_directory": lambda pkg: f"/share/{pkg}"},
        "launch": {"LaunchDescription": type("LaunchDescription", (_Made,), {})},
        "launch.actions": {n: type(n, (_Made,), {}) for n in
                           ("DeclareLaunchArgument", "ExecuteProcess", "IncludeLaunchDescription")},
        "launch.launch_description_sources": {"PythonLaunchDescriptionSource":
                                              type("PythonLaunchDescriptionSource", (_Made,), {})},
        "launch.conditions": {"IfCondition": type("IfCondition", (_Made,), {})},
        "launch.substitutions": {n: type(n, (_Made,), {}) for n in ("LaunchConfiguration", "PythonExpression")},
        "launch_ros": {},
        "launch_ros.actions": {"Node": Node},
    }
    for name, attrs in fakes.items():
        module = types.ModuleType(name)
        module.__dict__.update(attrs)
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.setattr(sys, "path", list(sys.path))       # the launch file adds the repo and nl_control

    real = bridge_core.objects_source

    def counting(*args, **kwargs):
        rec.source_calls.append(args)
        return real(*args, **kwargs)
    monkeypatch.setattr(bridge_core, "objects_source", counting)
    if source is None:
        monkeypatch.delenv("OBJECTS_SOURCE", raising=False)
    else:
        monkeypatch.setenv("OBJECTS_SOURCE", source)

    spec = importlib.util.spec_from_file_location("go2_gazebo_launch_under_test", LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, rec


def _run_launch(monkeypatch, source):
    """generate_launch_description() of the launch file loaded by _load_launch: (description, record)."""
    module, rec = _load_launch(monkeypatch, source)
    return module.generate_launch_description(), rec


def _bridges(nodes):
    return [n for n in nodes if n.kwargs.get("executable") == "parameter_bridge"]


# ---- tests ----
def test_xacro_has_no_camera_by_default():
    root = ET.parse(XACRO).getroot()
    assert {a.get("name"): a.get("default") for a in root.iter(XACRO_NS + "arg")} == {"GAZEBO": "true",
                                                                                     "camera": "false"}
    inside = list(_camera_block(root).iter("sensor"))
    outside = [s for s in root.iter("sensor") if s not in inside]
    assert [(s.get("name"), s.get("type")) for s in inside] == [("front_camera", "camera")]
    assert [(s.get("name"), s.get("type")) for s in outside] == [("utlidar", "gpu_lidar"),
                                                                 ("front_logical_camera", "logical_camera")]
    # Comments only inside the block, and no space added around the argument or the block: with them cut out, the
    # file is 0f6e06a's byte for byte, so xacro's output with camera false is too
    text = XACRO.read_text(encoding="utf-8")
    assert hashlib.sha256(_without_camera(text).encode("utf-8")).hexdigest() == XACRO_0F6E06A_SHA256


def test_camera_block_values():
    block = _camera_block()
    gazebos = block.findall("gazebo")
    assert [g.get("reference") for g in gazebos] == ["logical_camera"]      # the logical camera's link
    assert [e.tag for e in block] == ["gazebo"]                              # no new link or joint
    sensor = gazebos[0].find("sensor")
    cam = sensor.find("camera")
    values = {
        "horizontal_fov": cam.findtext("horizontal_fov"),
        "width": cam.findtext("image/width"), "height": cam.findtext("image/height"),
        "format": cam.findtext("image/format"),
        "near": cam.findtext("clip/near"), "far": cam.findtext("clip/far"),
        "camera_info_topic": cam.findtext("camera_info_topic"),
        "always_on": sensor.findtext("always_on"), "update_rate": sensor.findtext("update_rate"),
        "visualize": sensor.findtext("visualize"), "topic": sensor.findtext("topic"),
        "gz_frame_id": sensor.findtext("gz_frame_id"),
    }
    assert values == {"horizontal_fov": "2.094", "width": "640", "height": "480", "format": "R8G8B8",
                      "near": "0.1", "far": "10", "camera_info_topic": "gazebo/front_camera/camera_info",
                      "always_on": "true", "update_rate": "2", "visualize": "false",
                      "topic": "gazebo/front_camera/image", "gz_frame_id": "front_camera"}
    # one place for the numbers: detector/camera.py (D7 ruling)
    assert (int(values["width"]), int(values["height"])) == (CAMERA_W, CAMERA_H) == (640, 480)
    assert float(values["horizontal_fov"]) == CAMERA_HFOV == 2.094
    assert float(values["update_rate"]) == CAMERA_RATE_HZ == 2.0
    assert (float(values["near"]), float(values["far"])) == CAMERA_CLIP == (0.1, 10.0)
    k = CameraModel.from_hfov()
    assert (k.fx, k.fy, k.cx, k.cy) == pytest.approx((184.836, 184.836, 320.0, 240.0), abs=5e-4)
    assert sensor.find("noise") is None and cam.find("noise") is None                   # no noise
    assert cam.find("triggered") is None and sensor.find("triggered") is None          # periodic
    # the logical camera block is unchanged
    root = ET.parse(XACRO).getroot()
    logical = [s for s in root.iter("sensor") if s.get("type") == "logical_camera"]
    assert len(logical) == 1
    lc = logical[0]
    assert {t: lc.findtext(f"logical_camera/{t}") for t in ("near", "far", "horizontal_fov", "aspect_ratio")} == {
        "near": "0.1", "far": "8.0", "horizontal_fov": "2.094", "aspect_ratio": "1.778"}
    assert (lc.get("name"), lc.findtext("update_rate"), lc.findtext("topic"), lc.findtext("always_on"),
            lc.findtext("visualize")) == ("front_logical_camera", "5", "gazebo/logical_camera", "true", "false")


def test_main_bridge_arguments_are_unchanged(monkeypatch):
    assert GZ_BRIDGE_ARGS == BRIDGE_ARGS_0F6E06A
    assert all(type(a) is str for a in GZ_BRIDGE_ARGS)
    for source in (None, "logical", "detector"):
        _, rec = _run_launch(monkeypatch, source)
        main = _bridges(rec.nodes)[0]
        assert "name" not in main.kwargs, source                      # the same node as before
        assert main.kwargs["package"] == "ros_gz_bridge" and main.kwargs["output"] == "screen"
        assert main.kwargs["arguments"] == list(BRIDGE_ARGS_0F6E06A), source
        assert main.kwargs["parameters"] == [{"use_sim_time": True}]


def test_pins_are_those_of_0f6e06a():
    old_launch = _at_base("gazebo/launch/go2_gazebo.launch.py")
    assert _bridge_args_in(old_launch) == [BRIDGE_ARGS_0F6E06A]
    old_xacro = _at_base("gazebo/go2_sensors.urdf.xacro")
    assert hashlib.sha256(old_xacro.encode("utf-8")).hexdigest() == XACRO_0F6E06A_SHA256
    assert _without_camera(XACRO.read_text(encoding="utf-8")) == old_xacro


def test_camera_bridge_only_in_detector_mode(monkeypatch):
    assert CAMERA_BRIDGE_ARGS == ("/gazebo/front_camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
                                  "/gazebo/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo")
    assert CAMERA_BRIDGE_NAME == "front_camera_bridge"
    assert not set(CAMERA_BRIDGE_ARGS) & set(GZ_BRIDGE_ARGS)
    # the bridged topics are the xacro's (gz topics without the leading slash)
    sensor = _camera_block().find("gazebo/sensor")
    assert [a.split("@")[0] for a in CAMERA_BRIDGE_ARGS] == [
        "/" + sensor.findtext("topic"), "/" + sensor.findtext("camera/camera_info_topic")]
    assert camera_bridge_args("logical") is None
    assert camera_bridge_args("detector") == CAMERA_BRIDGE_ARGS
    assert xacro_mappings("logical") == {}
    assert xacro_mappings("detector") == {"camera": "true"}
    # xacro adds every xacro:arg default into the mappings it is given: a fresh dict each call
    assert xacro_mappings("detector") is not xacro_mappings("detector")
    assert xacro_mappings("logical") is not xacro_mappings("logical")
    for bad in ("", "Logical", "fake", "detector "):
        with pytest.raises(ValueError, match="OBJECTS_SOURCE"):
            xacro_mappings(bad)
        with pytest.raises(ValueError, match="OBJECTS_SOURCE"):
            camera_bridge_args(bad)

    by_source = {}
    for source in (None, "logical", "detector"):
        description, rec = _run_launch(monkeypatch, source)
        assert len(rec.source_calls) == 1, source                                 # read once
        assert rec.xacro_calls == [(str(XACRO), {"mappings": xacro_mappings(source or "logical")})], source
        entities = description.args[0]
        assert all(n in entities for n in rec.nodes), source
        rsp = next(n for n in rec.nodes if n.kwargs.get("executable") == "robot_state_publisher")
        assert rsp.kwargs["parameters"][0]["robot_description"] == (
            f"<robot camera mappings {sorted(xacro_mappings(source or 'logical').items())}/>")
        by_source[source] = (entities, _bridges(rec.nodes))
    for source in (None, "logical"):
        entities, bridges = by_source[source]
        assert len(bridges) == 1 and len(entities) == 9, source                    # as in 0f6e06a
    entities, bridges = by_source["detector"]
    assert len(bridges) == 2 and len(entities) == 10
    camera = bridges[1]
    assert camera.kwargs == {"package": "ros_gz_bridge", "executable": "parameter_bridge",
                             "name": "front_camera_bridge", "output": "screen",
                             "arguments": list(CAMERA_BRIDGE_ARGS), "parameters": [{"use_sim_time": True}]}


def test_invalid_objects_source_stops_the_launch(monkeypatch):
    for bad in ("lidar", "", "DETECTOR"):
        module, rec = _load_launch(monkeypatch, bad)
        with pytest.raises(ValueError, match=f"OBJECTS_SOURCE.*{bad!r}"):
            module.generate_launch_description()
        assert rec.xacro_calls == [] and rec.nodes == [], bad          # before the robot description and any node


def test_launch_config_imports_no_ros():
    tree = ast.parse(Path(launch_config.__file__).read_text(encoding="utf-8"))
    imported = {a.name.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.Import) for a in n.names}
    imported |= {n.module.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and n.module}
    assert imported <= {"__future__", "typing", "gazebo"}, imported


def _description(monkeypatch, folder: Path, **kwargs) -> str:
    """xacro.process_file(...).toxml() of folder/go2_sensors.urdf.xacro, by its relative name, so xacro's banner
    (it names the input file) is the same for every folder."""
    xacro = pytest.importorskip("xacro")
    packages = pytest.importorskip("ament_index_python.packages")
    try:
        packages.get_package_share_directory("go2_description")
    except packages.PackageNotFoundError:
        pytest.skip("go2_description is not installed (the gazebo image has it in ~/gz_ws)")
    monkeypatch.chdir(folder)
    return xacro.process_file(XACRO.name, **kwargs).toxml()


def test_logical_robot_description_is_unchanged(monkeypatch, tmp_path):
    now = _description(monkeypatch, XACRO.parent)
    old_folder = tmp_path / "base"
    old_folder.mkdir()
    (old_folder / XACRO.name).write_text(_at_base("gazebo/go2_sensors.urdf.xacro"), encoding="utf-8")
    old = _description(monkeypatch, old_folder)
    assert now == old
    assert _description(monkeypatch, XACRO.parent, mappings=xacro_mappings("logical")) == old   # what launch passes


def test_detector_robot_description_has_one_camera(monkeypatch):
    def sensors(xml):
        root = ET.fromstring(xml)
        return [(g.get("reference"), s.get("name"), s.get("type")) for g in root.iter("gazebo")
                for s in g.iter("sensor")]
    logical = sensors(_description(monkeypatch, XACRO.parent))
    detector = sensors(_description(monkeypatch, XACRO.parent, mappings=xacro_mappings("detector")))
    assert [s for s in logical if s[2] == "camera"] == []
    assert [s for s in detector if s[2] == "camera"] == [("logical_camera", "front_camera", "camera")]
    assert [s for s in detector if s[2] != "camera"] == logical       # the LiDAR and the logical camera either way
    assert ("logical_camera", "front_logical_camera", "logical_camera") in logical
