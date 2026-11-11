#!/usr/bin/env python3
"""docker/fuel_models.json に固定した Gazebo Fuel のモデルを取って、1 つのフォルダに開く（open-objects spec §12.2, §12.6）。

docker/Dockerfile.gazebo-fuel が、イメージを作るときに 1 回だけ動かす:
    python3 /opt/go2_fuel/fetch_fuel_models.py /opt/go2_fuel/fuel_models.json /opt/go2_models
モデルごとに {base}/{folder}/{version}/{folder}.zip を取り、zip の sha256 を固定の値と比べ（null の固定か違いなら止まる）、
<dest>/<folder> に開く。固定はどれも、取る前に確かめる。どこかで外れたら 0 でない終了コードで終わるので、build も止まる。
開くときは、絶対パスと .. を断り、zip の上に 1 つだけフォルダがあればそれを外し（Fuel の zip は上に
model.config と model.sdf がそのまま並ぶ）、model.config と model.sdf を求め、model.sdf の model://<名前> が
フォルダの名前と同じことを確かめる（Gazebo は GZ_SIM_RESOURCE_PATH/<名前> でファイルを探す）。
標準ライブラリだけを使う（Gazebo のイメージで、ほかに何も入れずに動く）。
"""

from __future__ import annotations

import hashlib
import http.client
import json
import re
import shutil
import stat
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
import zipfile
from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Callable

TIMEOUT_S = 60.0
ATTEMPTS = 3                        # a dropped connection or a 5xx is tried again; a 4xx or a wrong sha256 is not
MAX_ZIP_BYTES = 64 * 1024 * 1024    # the five pinned zips are 0.7-1.8 MB
MAX_UNPACKED_BYTES = 512 * 1024 * 1024
MODEL_FILES = ("model.config", "model.sdf")

_FOLDER_SHAPE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]*")      # the same shape as voice_ltl/layout.py
_SHA256_SHAPE = re.compile(r"[0-9a-f]{64}")
_DRIVE = re.compile(r"[A-Za-z]:")
_MODEL_URI = re.compile(rb"model://([^/<>\"'\s]*)")


class FuelError(ValueError):
    """A list, download or zip that the build must not use (one line naming what and why)."""


@dataclass(frozen=True)
class Model:
    folder: str
    version: int
    sha256: str | None


@dataclass(frozen=True)
class FuelList:
    base: str
    models: tuple[Model, ...]


def verify(data: bytes, sha256: str | None) -> None:
    """Raise FuelError unless sha256(data) is the pin (64 lowercase hex characters; None is no pin)."""
    if sha256 is None:
        raise FuelError("no sha256 pin (record the zip's sha256 in docker/fuel_models.json first)")
    if not (isinstance(sha256, str) and _SHA256_SHAPE.fullmatch(sha256)):
        raise FuelError(f"the sha256 pin {sha256!r} is not 64 lowercase hex characters")
    got = hashlib.sha256(data).hexdigest()
    if got != sha256:
        raise FuelError(f"sha256 mismatch: the download ({len(data)} B) is {got}, the pin is {sha256}")


def _check_folder(folder: object) -> str:
    if not (isinstance(folder, str) and _FOLDER_SHAPE.fullmatch(folder)):
        raise FuelError(f"the folder {folder!r} is not one plain name ([A-Za-z0-9][A-Za-z0-9_.-]*)")
    return folder


def _parts(name: str) -> tuple[str, ...]:
    """The path parts of a zip member name; FuelError for an absolute path or a .. part."""
    if name.startswith(("/", "\\")) or _DRIVE.match(name):
        raise FuelError(f"unsafe path in the zip: {name!r} is absolute")
    parts = tuple(p for p in re.split(r"[/\\]", name) if p not in ("", "."))
    if ".." in parts:
        raise FuelError(f"unsafe path in the zip: {name!r} has ..")
    return parts


def _plan(zf: zipfile.ZipFile, zip_name: str) -> dict[PurePosixPath, zipfile.ZipInfo]:
    """The files to write (path in the model folder -> member), after every check that needs no writing."""
    members = []
    for info in zf.infolist():
        parts = _parts(info.filename)
        if stat.S_ISLNK(info.external_attr >> 16):
            raise FuelError(f"{zip_name}: {info.filename!r} is a link")
        if parts:
            members.append((parts, info))
    files = [(parts, info) for parts, info in members if not info.is_dir()]
    tops = {parts[0] for parts, _ in members}
    if len(tops) == 1 and files and all(len(parts) > 1 for parts, _ in files):
        files = [(parts[1:], info) for parts, info in files]         # one top-level folder: strip it
    plan: dict[PurePosixPath, zipfile.ZipInfo] = {}
    for parts, info in files:
        rel = PurePosixPath(*parts)
        if rel in plan:
            raise FuelError(f"{zip_name}: {info.filename!r} is in the zip twice")
        plan[rel] = info
    for rel, info in plan.items():
        if any(parent in plan for parent in rel.parents):
            raise FuelError(f"{zip_name}: {info.filename!r} is inside a member that is a file")
    missing = [name for name in MODEL_FILES if PurePosixPath(name) not in plan]
    if missing:
        raise FuelError(f"{zip_name}: the model needs model.config and model.sdf at its top "
                        f"(missing {', '.join(missing)}; the zip has {len(plan)} files)")
    unpacked = sum(info.file_size for info in plan.values())
    if unpacked > MAX_UNPACKED_BYTES:
        raise FuelError(f"{zip_name}: the unpacked size {unpacked} B is over the limit {MAX_UNPACKED_BYTES} B")
    return plan


def _check_model_uris(sdf: bytes, folder: str, zip_name: str) -> None:
    for name in sorted({m.group(1) for m in _MODEL_URI.finditer(sdf)}):
        if name.decode("utf-8", "replace") != folder:
            shown = name.decode("utf-8", "replace")
            raise FuelError(f"{zip_name}: model.sdf refers to model://{shown}/..., but the folder is {folder} "
                            f"(Gazebo looks for model://<name> in a folder of that name)")


def extract(zip_path, dest: Path, folder: str) -> Path:
    """Unpack the model zip to dest/folder and return that path (spec §12.6).

    Refuses absolute paths, .. parts and links; strips one top-level folder when every member is inside it;
    requires model.config and model.sdf at the top of the model; requires every model://<name> in model.sdf
    to name the folder. Nothing is written unless every check passes; dest/folder must not exist yet."""
    folder = _check_folder(folder)
    zip_path, dest = Path(zip_path), Path(dest)
    target = dest / folder
    if target.exists():
        raise FuelError(f"{target} already exists (a model is unpacked once, into a new folder)")
    try:
        with zipfile.ZipFile(zip_path) as zf:
            plan = _plan(zf, zip_path.name)
            with zf.open(plan[PurePosixPath("model.sdf")]) as f:
                _check_model_uris(f.read(), folder, zip_path.name)
            target.mkdir(parents=True)
            root = target.resolve()
            for rel, info in plan.items():
                out = target.joinpath(*rel.parts)
                if root not in out.resolve().parents:                 # cannot happen after _parts; kept as a guard
                    raise FuelError(f"unsafe path in the zip: {info.filename!r}")
                out.parent.mkdir(parents=True, exist_ok=True)
                with zf.open(info) as src, open(out, "wb") as dst:
                    shutil.copyfileobj(src, dst)
    except zipfile.BadZipFile as e:
        raise FuelError(f"{zip_path.name}: not a zip file ({e})") from None
    return target


def read_list(list_path) -> FuelList:
    """Read docker/fuel_models.json: an https base and models with a folder, a version and a sha256 (or null).

    Other fields (bytes, author, licence) are for the docs and are not read here."""
    list_path = Path(list_path)
    try:
        data = json.loads(list_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as e:
        raise FuelError(f"{list_path}: not JSON (line {e.lineno} column {e.colno}: {e.msg})") from None
    except OSError as e:
        raise FuelError(f"{list_path}: cannot read ({e.strerror or e})") from None
    if not isinstance(data, dict):
        raise FuelError(f"{list_path}: the list is one JSON object")
    base = data.get("base")
    if not (isinstance(base, str) and base.startswith("https://") and not base.endswith("/")):
        raise FuelError(f"{list_path}: base {base!r} is an https:// URL without a trailing /")
    raw = data.get("models")
    if not (isinstance(raw, list) and raw):
        raise FuelError(f"{list_path}: models is a non-empty list")
    models = []
    for m in raw:
        if not isinstance(m, dict):
            raise FuelError(f"{list_path}: each model is an object, not {m!r}")
        try:
            folder = _check_folder(m.get("folder"))
        except FuelError as e:
            raise FuelError(f"{list_path}: {e}") from None
        version = m.get("version")
        if not (isinstance(version, int) and not isinstance(version, bool) and version >= 1):
            raise FuelError(f"{list_path}: {folder}: version {version!r} is a whole number >= 1")
        sha256 = m.get("sha256")
        if not (sha256 is None or (isinstance(sha256, str) and _SHA256_SHAPE.fullmatch(sha256))):
            raise FuelError(f"{list_path}: {folder}: sha256 {sha256!r} is null or 64 lowercase hex characters")
        if any(x.folder == folder for x in models):
            raise FuelError(f"{list_path}: {folder}: the folder is listed twice")
        models.append(Model(folder, version, sha256))
    return FuelList(base, tuple(models))


def zip_url(base: str, model: Model) -> str:
    """Fuel's REST form {base}/{folder}/{version}/{folder}.zip (spec §12.2)."""
    name = urllib.parse.quote(model.folder)
    return f"{base}/{name}/{model.version}/{name}.zip"


def download(url: str) -> bytes:
    """The bytes at url (at most MAX_ZIP_BYTES), tried ATTEMPTS times on a dropped connection or a 5xx."""
    request = urllib.request.Request(url, headers={"User-Agent": "go2-voice-ltl fetch_fuel_models"})
    for attempt in range(1, ATTEMPTS + 1):
        try:
            with urllib.request.urlopen(request, timeout=TIMEOUT_S) as response:
                data = response.read(MAX_ZIP_BYTES + 1)
            if len(data) > MAX_ZIP_BYTES:
                raise FuelError(f"{url}: more than {MAX_ZIP_BYTES} B")
            return data
        except urllib.error.HTTPError as e:
            if not (500 <= e.code < 600 and attempt < ATTEMPTS):
                raise FuelError(f"{url}: HTTP {e.code} {e.reason}") from None
        except (urllib.error.URLError, http.client.HTTPException, TimeoutError, ConnectionError) as e:
            if attempt == ATTEMPTS:
                raise FuelError(f"{url}: {getattr(e, 'reason', e)} (after {ATTEMPTS} tries)") from None
        time.sleep(2.0 * attempt)
    raise AssertionError("unreachable")


def main(list_path, dest, *, fetch: Callable[[str], bytes] = download) -> int:
    """Fetch, verify and unpack every listed model into dest; 0 when all are in place, 1 at the first failure."""
    list_path, dest = Path(list_path), Path(dest)
    try:
        fuel = read_list(list_path)
    except FuelError as e:
        print(f"fetch_fuel_models: {e}", file=sys.stderr)
        return 1
    unpinned = [m for m in fuel.models if m.sha256 is None]
    if unpinned:                                                     # before any download
        for m in unpinned:
            print(f"fetch_fuel_models: {m.folder} {m.version}: no sha256 pin in {list_path} "
                  f"(fetch the zip once and record its sha256 and bytes first)", file=sys.stderr)
        return 1
    try:
        dest.mkdir(parents=True, exist_ok=True)
    except OSError as e:
        print(f"fetch_fuel_models: {dest}: {e}", file=sys.stderr)
        return 1
    for m in fuel.models:
        url = zip_url(fuel.base, m)
        zip_path = dest / f".{m.folder}-{m.version}.zip"
        try:
            data = fetch(url)
            verify(data, m.sha256)
            zip_path.write_bytes(data)
            target = extract(zip_path, dest, m.folder)
        except (FuelError, OSError) as e:
            print(f"fetch_fuel_models: {m.folder} {m.version}: {url}: {e}", file=sys.stderr)
            return 1
        finally:
            zip_path.unlink(missing_ok=True)
        print(f"fetch_fuel_models: {m.folder} {m.version}: {len(data)} B, sha256 {m.sha256} ok -> {target}")
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("usage: fetch_fuel_models.py <fuel_models.json> <dest folder>", file=sys.stderr)
        sys.exit(2)
    sys.exit(main(sys.argv[1], sys.argv[2]))
