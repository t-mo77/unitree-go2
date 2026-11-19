"""The detector image and its compose service (open-objects spec §11.1, §11.2, §4; plan Task 2.8).

docker/Dockerfile.detector (design-detector.md §3.2), the `detector` service in compose.yaml (§3.6, profile
`detector`), the `detector` pytest marker, and the licence isolation of D5: only detector/yoloe*.py imports
torch or ultralytics, so neither reaches the app or the gazebo image. Nothing here builds or starts the
image (plan H2 builds it).
"""

import configparser
import fnmatch
import json
import os
import re
import shlex
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
DOCKERFILE = ROOT / "docker" / "Dockerfile.detector"
COMPOSE = ROOT / "compose.yaml"
PYTEST_INI = ROOT / "pytest.ini"

# spec §11.1 (pins from spike/detector.md §2)
TAG = "go2-voice-ltl-detector:yoloe-11s"
BASE = "ros:jazzy-ros-base-noble"
TORCH_VERSION = "2.14.1"
TORCHVISION_VERSION = "0.29.1"
ULTRALYTICS_VERSION = "8.4.210"
CLIP_COMMIT = "7ffa84b3bfa40c42ecc1c77147a855e69cb2dd40"
ASSETS = "https://github.com/ultralytics/assets/releases/download/v8.4.0"
TORCH_INDEX = "https://download.pytorch.org/whl/cpu"
CLIP_URL = "git+https://github.com/ultralytics/CLIP.git"
YOLOE_SHA256 = "8e439445c87338b79d9ce21dec109f4621e26df67e94d26ea1a98c1e64dce3e3"
MOBILECLIP_SHA256 = "a67804d1b0f07b8b9a20c1761ec0847f34660f5fa338ec70e8f3fce68ed95e54"

MARKER = "detector: needs the detector image (ultralytics); skipped elsewhere"

# spec §4 / D5: an import statement of torch or ultralytics, indented ones included. Anchored to the start of
# a statement, so this pattern's own source line (and any string or comment naming the packages) never matches.
IMPORT = re.compile(r"^\s*(?:import|from)\s+(?:torch|ultralytics)\b", re.M)


def instructions(text: str) -> list:
    """The Dockerfile as (KEYWORD, arguments) pairs: comment lines dropped, continuations joined, runs of
    whitespace collapsed to one space."""
    out, parts = [], []
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if line.endswith("\\"):
            parts.append(line[:-1].strip())
            continue
        parts.append(line)
        joined = " ".join(" ".join(parts).split())
        keyword, _, args = joined.partition(" ")
        out.append((keyword.upper(), args))
        parts = []
    assert not parts, "a continuation runs off the end of the file"
    return out


def env_pairs(args: str) -> dict:
    return dict(word.split("=", 1) for word in shlex.split(args))


# --- docker/Dockerfile.detector --------------------------------------------------------------------------------

def test_dockerfile_pins():
    text = DOCKERFILE.read_text(encoding="utf-8")
    header = []
    for line in text.splitlines():
        if not line.startswith("#"):
            break
        header.append(line)
    joined = "\n".join(header)
    assert TAG in joined and "AGPL-3.0" in joined and "ultralytics" in joined     # the tag and the licence (D5)
    assert "mobileclip_blt.ts" in joined and "Research Purposes" in joined     # spec §11.2: not AGPL, research only
    assert f"docker build -t {TAG} -f docker/Dockerfile.detector docker" in joined   # the build command (plan H2)

    steps = instructions(text)
    assert steps[0] == ("FROM", BASE)
    args = dict(a.split("=", 1) for k, a in steps if k == "ARG")
    assert args == {"TORCH_VERSION": TORCH_VERSION, "TORCHVISION_VERSION": TORCHVISION_VERSION,
                    "ULTRALYTICS_VERSION": ULTRALYTICS_VERSION, "CLIP_COMMIT": CLIP_COMMIT, "ASSETS": ASSETS,
                    "YOLOE_SHA256": YOLOE_SHA256, "MOBILECLIP_SHA256": MOBILECLIP_SHA256}
    for sha in (YOLOE_SHA256, MOBILECLIP_SHA256):
        assert re.fullmatch(r"[0-9a-f]{64}", sha)

    # design-detector.md §3.2 as written
    assert steps == [
        ("FROM", BASE),
        ("SHELL", '["/bin/bash", "-o", "pipefail", "-c"]'),
        ("ENV", "DEBIAN_FRONTEND=noninteractive"),
        ("ARG", f"TORCH_VERSION={TORCH_VERSION}"),
        ("ARG", f"TORCHVISION_VERSION={TORCHVISION_VERSION}"),
        ("ARG", f"ULTRALYTICS_VERSION={ULTRALYTICS_VERSION}"),
        ("ARG", f"CLIP_COMMIT={CLIP_COMMIT}"),
        ("ARG", f"ASSETS={ASSETS}"),
        ("ARG", f"YOLOE_SHA256={YOLOE_SHA256}"),
        ("ARG", f"MOBILECLIP_SHA256={MOBILECLIP_SHA256}"),
        ("RUN", "apt-get update && apt-get install -y --no-install-recommends python3-venv libgl1 libglib2.0-0t64 "
                "ros-jazzy-rmw-cyclonedds-cpp curl ca-certificates && rm -rf /var/lib/apt/lists/*"),
        ("RUN", "python3 -m venv --system-site-packages /opt/det"),
        ("ENV", "PATH=/opt/det/bin:$PATH VIRTUAL_ENV=/opt/det"),
        ("RUN", 'pip install --no-cache-dir "torch==${TORCH_VERSION}" "torchvision==${TORCHVISION_VERSION}" '
                f"--index-url {TORCH_INDEX}"),
        ("RUN", 'pip install --no-cache-dir "ultralytics==${ULTRALYTICS_VERSION}" '
                f'"clip @ {CLIP_URL}@${{CLIP_COMMIT}}" pytest==8.4.2'),
        ("RUN", "mkdir -p /opt/yoloe/config && cd /opt/yoloe "
                '&& curl -fsSL -o yoloe-11s-seg.pt "${ASSETS}/yoloe-11s-seg.pt" '
                '&& curl -fsSL -o mobileclip_blt.ts "${ASSETS}/mobileclip_blt.ts" '
                "&& printf '%s %s\\n' \"${YOLOE_SHA256}\" yoloe-11s-seg.pt \"${MOBILECLIP_SHA256}\" mobileclip_blt.ts "
                "| sha256sum -c -"),
        ("ENV", "YOLO_CONFIG_DIR=/opt/yoloe/config YOLO_AUTOINSTALL=False YOLO_OFFLINE=True "
                "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp PYTHONPATH=/work:/work/nl_control "
                "OMP_NUM_THREADS=1 MKL_NUM_THREADS=1"),
        ("RUN", "yolo settings weights_dir=/opt/yoloe sync=False > /dev/null"),
        ("WORKDIR", "/work"),
        ("ENTRYPOINT", '["bash", "-c", "source /opt/ros/jazzy/setup.bash && exec \\"$@\\"", "--"]'),
        ("CMD", '["python3", "-m", "detector.node"]'),
    ]

    # what the steps mean, against spec §11.1
    run = [a for k, a in steps if k == "RUN"]
    apt = run[0].split("--no-install-recommends ", 1)[1].split(" && ", 1)[0].split()
    assert apt == ["python3-venv", "libgl1", "libglib2.0-0t64", "ros-jazzy-rmw-cyclonedds-cpp", "curl",
                   "ca-certificates"]
    torch_pip = shlex.split(run[2])
    assert torch_pip[torch_pip.index("--index-url") + 1] == TORCH_INDEX                        # the CPU wheels
    weights = run[4].replace("${ASSETS}", args["ASSETS"])
    assert f'-o yoloe-11s-seg.pt "{ASSETS}/yoloe-11s-seg.pt"' in weights                     # both URLs
    assert f'-o mobileclip_blt.ts "{ASSETS}/mobileclip_blt.ts"' in weights
    assert weights.endswith("| sha256sum -c -") and "pipefail" in steps[1][1]                 # a bad byte fails
    assert "printf '%s  %s\\n'" in text                     # two spaces: the line format sha256sum -c reads
    assert weights.count("curl -fsSL") == 2                                                    # nothing else fetched
    env = {}
    for k, a in steps:
        if k == "ENV":
            env.update(env_pairs(a))
    assert env["YOLO_OFFLINE"] == "True" and env["YOLO_AUTOINSTALL"] == "False"               # nothing at run time
    assert env["OMP_NUM_THREADS"] == "1" and env["MKL_NUM_THREADS"] == "1"
    assert env["RMW_IMPLEMENTATION"] == "rmw_cyclonedds_cpp" and env["PYTHONPATH"] == "/work:/work/nl_control"
    assert env["YOLO_CONFIG_DIR"] == "/opt/yoloe/config"
    entry = json.loads(dict(steps)["ENTRYPOINT"])
    assert entry == ["bash", "-c", 'source /opt/ros/jazzy/setup.bash && exec "$@"', "--"]
    assert json.loads(dict(steps)["CMD"]) == ["python3", "-m", "detector.node"]
    assert "COPY" not in dict(steps) and "ADD" not in dict(steps)                              # nothing from the tree

    # 11m and the prompt-free -pf weights are not fetched (spec §11.1; O24's fallback is not built)
    assert "-pf" not in text and "11m" not in text


# --- compose.yaml ------------------------------------------------------------------------------------------------

def test_compose_detector_service():
    text = COMPOSE.read_text(encoding="utf-8")
    data = yaml.safe_load(text)
    detector = data["services"]["detector"]
    assert detector == {
        "profiles": ["detector"],
        "build": {"context": "./docker", "dockerfile": "Dockerfile.detector"},
        "image": TAG,
        "init": True,
        "volumes": ["./:/work", "/dev/null:/work/env.txt:ro", "detector-cache:/cache"],
        "environment": {"OBJECTS_SOURCE": "${OBJECTS_SOURCE:-logical}", "DETECTOR": "${DETECTOR:-yoloe}"},
        "cpus": "2.0",
        "restart": "on-failure:3",
        "command": "python3 -m detector.node",
    }
    assert "env_file" not in detector                                    # no login: compose never reads env.txt for it
    assert "/dev/null:/work/env.txt:ro" in detector["volumes"]           # and the repo's env.txt is masked inside
    # M9: the default ROS domain, like gazebo (neither service sets one)
    assert "ROS_DOMAIN_ID" not in detector["environment"]
    assert "ROS_DOMAIN_ID" not in data["services"]["gazebo"]["environment"]
    assert data["volumes"] == {"hf-cache": None, "codex-home": None, "detector-cache": None}
    # inserted before gazebo: the gazebo slices ("\n  gazebo:" to "\nvolumes:") of test_gazebo_objects.py stay gazebo's
    assert list(data["services"]) == ["app", "sim", "detector", "gazebo"]
    assert text.index("\n  sim:") < text.index("\n  detector:") < text.index("\n  gazebo:") < text.index("\nvolumes:")
    block = text[text.index("\n  detector:"):text.index("\n  gazebo:")]
    assert "<<:" not in block and "*go2" not in block                    # not the shared anchor (its env_file)
    assert "AGPL-3.0" in block and "--profile detector" in block         # the licence and how it is started


# --- pytest.ini --------------------------------------------------------------------------------------------------

def test_detector_marker_is_registered(pytestconfig):
    assert MARKER in pytestconfig.getini("markers")
    ini = configparser.ConfigParser(interpolation=None)
    ini.read(PYTEST_INI, encoding="utf-8")
    markers = [line.strip() for line in ini["pytest"]["markers"].splitlines() if line.strip()]
    assert markers == ["slow: loads a real model (downloads on first use)", MARKER]


# --- licence isolation (D5, spec §4) -----------------------------------------------------------------------------

def python_files():
    """Every *.py under the repository root. Dot folders are skipped: .git, and the gitignored .venv
    (README: the local app venv), .pytest_cache and .superpowers, none of which hold this repo's code."""
    for dirpath, dirnames, filenames in os.walk(ROOT):
        dirnames[:] = sorted(d for d in dirnames if not d.startswith("."))
        for name in sorted(filenames):
            if name.endswith(".py"):
                yield Path(dirpath) / name


def may_import(rel: Path) -> bool:
    return rel.parent == Path("detector") and fnmatch.fnmatch(rel.name, "yoloe*.py")


def test_licence_isolation():
    for line in ("import torch", "    import torch", "\tfrom torch import nn", "from torch.nn import functional",
                 "import ultralytics", "        from ultralytics import YOLOE", "import torch as th"):
        assert IMPORT.search(f"x = 1\n{line}\n"), line                   # indented imports included
    for line in ("# import torch", "x = 'import torch'", "import torchx", "from ultralytics_x import a",
                 'IMPORT = re.compile(r"^\\s*(?:import|from)\\s+(?:torch|ultralytics)\\b", re.M)',
                 "importlib.util.find_spec('ultralytics')"):
        assert not IMPORT.search(f"x = 1\n{line}\n"), line
    assert may_import(Path("detector/yoloe.py")) and may_import(Path("detector/yoloe_embeddings.py"))
    assert not may_import(Path("detector/node.py")) and not may_import(Path("tests/test_detector_yoloe.py"))
    assert not may_import(Path("detector/sub/yoloe.py")) and not may_import(Path("gazebo/yoloe.py"))

    scanned, importers = [], []
    for path in python_files():
        rel = path.relative_to(ROOT)
        scanned.append(rel.as_posix())
        if IMPORT.search(path.read_text(encoding="utf-8")) and not may_import(rel):
            importers.append(rel.as_posix())
    assert importers == []
    for seen in ("detector/geometry.py", "detector/tracker.py", "voice_ltl/objects.py", "gazebo/bridge_core.py",
                 "sim/fake_go2.py", "nl_control/go2_nl_control/supervisor.py", "tests/test_detector_image.py"):
        assert seen in scanned, seen                                     # the walk saw the whole tree
