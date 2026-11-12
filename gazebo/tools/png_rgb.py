"""8-bit RGB PNG files with the standard library (zlib, struct) and numpy only: no PIL, no cv2 (plan review I8).

record_frames.py writes the camera images with write_png inside the Gazebo image (which has neither OpenCV nor a
verified PIL) and detector_check.py reads them with read_png in the app and detector images.
- write_png(path, rgb): H x W x 3 uint8 -> colour type 2, bit depth 8, no interlace, filter 0 on every row, one
  IDAT chunk.
- read_png(path) -> H x W x 3 uint8: 8-bit RGB (colour type 2) or RGBA (6, the alpha is dropped), not interlaced,
  any of the filter types 0-4, IDAT in one or more chunks. Anything else (another bit depth or colour type,
  interlacing, a bad CRC, truncated or short data, a missing IHDR / IDAT / IEND, an unknown critical chunk) raises
  ValueError naming the file and the reason.
"""

from __future__ import annotations

import os
import struct
import zlib

import numpy as np

SIGNATURE = b"\x89PNG\r\n\x1a\n"
COMPRESS_LEVEL = 6
_CHANNELS = {2: 3, 6: 4}             # colour type -> samples per pixel (8 bits each)


def write_png(path: str | os.PathLike, rgb: np.ndarray) -> None:
    """Write an H x W x 3 uint8 image (RGB) as a PNG file."""
    a = np.asarray(rgb)
    if a.dtype != np.uint8 or a.ndim != 3 or a.shape[2] != 3 or a.shape[0] < 1 or a.shape[1] < 1:
        raise ValueError(f"write_png needs an H x W x 3 uint8 array, not {a.shape} {a.dtype}")
    h, w = a.shape[:2]
    raw = np.zeros((h, 1 + 3 * w), dtype=np.uint8)          # column 0: filter type 0 (None) on every row
    raw[:, 1:] = a.reshape(h, 3 * w)
    data = (SIGNATURE + _chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + _chunk(b"IDAT", zlib.compress(raw.tobytes(), COMPRESS_LEVEL)) + _chunk(b"IEND", b""))
    with open(path, "wb") as f:
        f.write(data)


def read_png(path: str | os.PathLike) -> np.ndarray:
    """Read an 8-bit RGB or RGBA PNG file as an H x W x 3 uint8 array (see the module docstring)."""
    name = os.fspath(path)
    with open(path, "rb") as f:
        data = f.read()
    w, h, bpp, compressed = _parse(data, name)
    try:
        raw = zlib.decompress(compressed)
    except zlib.error as e:
        raise ValueError(f"{name}: the image data does not decompress ({e})") from None
    stride = w * bpp
    if len(raw) != h * (stride + 1):
        raise ValueError(f"{name}: {len(raw)} bytes of image data, a {w} x {h} image needs {h * (stride + 1)}")
    rows = np.frombuffer(raw, dtype=np.uint8).reshape(h, stride + 1)
    out = np.empty((h, stride), dtype=np.uint8)
    prior = np.zeros(stride, dtype=np.uint8)
    for i in range(h):
        out[i] = _unfilter(int(rows[i, 0]), rows[i, 1:], prior, bpp, w, name, i)
        prior = out[i]
    return np.ascontiguousarray(out.reshape(h, w, bpp)[:, :, :3])


def _chunk(kind: bytes, body: bytes) -> bytes:
    return struct.pack(">I", len(body)) + kind + body + struct.pack(">I", zlib.crc32(kind + body) & 0xFFFFFFFF)


def _parse(data: bytes, name: str) -> tuple[int, int, int, bytes]:
    """(width, height, bytes per pixel, the joined IDAT data) after checking the chunks and the header."""
    if not data.startswith(SIGNATURE):
        raise ValueError(f"{name}: not a PNG file (no PNG signature)")
    pos, header, idat, ended = len(SIGNATURE), None, [], False
    while pos < len(data):
        if pos + 8 > len(data):
            raise ValueError(f"{name}: truncated chunk header at byte {pos}")
        length, kind = struct.unpack(">I4s", data[pos:pos + 8])
        body, crc = data[pos + 8:pos + 8 + length], data[pos + 8 + length:pos + 12 + length]
        if len(body) != length or len(crc) != 4:
            raise ValueError(f"{name}: truncated {kind!r} chunk at byte {pos}")
        if zlib.crc32(kind + body) & 0xFFFFFFFF != struct.unpack(">I", crc)[0]:
            raise ValueError(f"{name}: bad CRC in the {kind!r} chunk at byte {pos}")
        pos += 12 + length
        if header is None and kind != b"IHDR":
            raise ValueError(f"{name}: the first chunk is {kind!r}, not IHDR")
        if kind == b"IHDR":
            if header is not None or length != 13:
                raise ValueError(f"{name}: a second or malformed IHDR chunk")
            header = struct.unpack(">IIBBBBB", body)
        elif kind == b"IDAT":
            idat.append(body)
        elif kind == b"IEND":
            ended = True
            break
        elif not kind[0] & 0x20 and kind != b"PLTE":       # a critical chunk this reader does not know
            raise ValueError(f"{name}: unsupported critical chunk {kind!r}")
    if header is None:
        raise ValueError(f"{name}: no IHDR chunk")
    if not ended:
        raise ValueError(f"{name}: no IEND chunk (truncated file)")
    if not idat:
        raise ValueError(f"{name}: no IDAT chunk")
    w, h, depth, colour, compression, filtering, interlace = header
    if w < 1 or h < 1:
        raise ValueError(f"{name}: empty image {w} x {h}")
    if depth != 8 or colour not in _CHANNELS:
        raise ValueError(f"{name}: bit depth {depth}, colour type {colour}: only 8-bit RGB (2) or RGBA (6) is read")
    if compression != 0 or filtering != 0:
        raise ValueError(f"{name}: unknown compression {compression} or filter method {filtering}")
    if interlace != 0:
        raise ValueError(f"{name}: interlaced (Adam7) images are not read")
    return w, h, _CHANNELS[colour], b"".join(idat)


def _unfilter(kind: int, line: np.ndarray, prior: np.ndarray, bpp: int, w: int, name: str, row: int) -> np.ndarray:
    """One reconstructed scanline (PNG filter types 0-4)."""
    if kind == 0:
        return line
    if kind == 1:                                            # Sub: a running sum per channel, mod 256
        return (np.cumsum(line.reshape(w, bpp), axis=0, dtype=np.int64) & 0xFF).astype(np.uint8).reshape(-1)
    if kind == 2:                                            # Up
        return ((line.astype(np.int64) + prior) & 0xFF).astype(np.uint8)
    if kind not in (3, 4):
        raise ValueError(f"{name}: row {row} has filter type {kind} (only 0-4 exist)")
    cur, up, out = line.tobytes(), prior.tobytes(), bytearray(len(line))
    for i in range(len(cur)):                                # Average and Paeth depend on the reconstructed left
        a = out[i - bpp] if i >= bpp else 0
        b = up[i]
        if kind == 3:
            out[i] = (cur[i] + ((a + b) >> 1)) & 0xFF
            continue
        c = up[i - bpp] if i >= bpp else 0
        p = a + b - c
        pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
        out[i] = (cur[i] + (a if pa <= pb and pa <= pc else (b if pb <= pc else c))) & 0xFF
    return np.frombuffer(bytes(out), dtype=np.uint8)
