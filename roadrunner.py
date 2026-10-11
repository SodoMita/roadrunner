#!/usr/bin/env python3
"""ROADRUNNER — a terminal car game whose complete current state lives in one SVG.

By default a sibling GAME.history/ folder also keeps a full SVG snapshot per
turn and a JSONL input/outcome log. Use `new --no-history` to disable it.
The game file is a normal SVG image you can open in a browser. It contains:
  * an embedded raster (PNG) where the BLACK line is the road,
  * a <g:vehicle> element carrying position, rotation, velocity, torque,
    weight, linear drag and angular drag,
  * a <g:path> element with the car's trail (a single dot at first),
  * <g:road> / <g:rules> / <g:status> elements describing the track and
    the restriction conditions of the game.

You play from the terminal. Every `run` call:
  1. lets you directly set vehicle values (only fields allowed by the rules),
  2. lets you apply a force and/or a torque,
  3. advances the physics simulation,
  4. extends the trail, rewrites the SVG and reports coverage / off-road stats.

WIN  : the trail covers at least `min-road-coverage` of the road while at most
       `max-path-off-road` of the trail lies outside the road.
LOSE : the off-road allowance is exceeded, or the time limit runs out.

Usage:
  roadrunner.py new    GAME.svg [--style easy|medium|hard] [--seed N]
                       [--width 900] [--height 600] [--road-width 9]
                       [--rules RULES.json] [--force] [--no-history]
  roadrunner.py run    GAME.svg [--fx F] [--fy F] [--force M] [--angle A]
                       [--frame local|global] [--torque T]
                       [--set key=value]... [--steps N] [--quiet]
  roadrunner.py status GAME.svg [--map]
  roadrunner.py retry  GAME.svg [--turns K | --to N] [--keep] [--quiet]
  roadrunner.py history GAME.svg [--json] [--tail N]
  roadrunner.py rules

TURN HISTORY (made for LLM players): every game keeps a folder next to the
file, GAME.history/, with one complete snapshot SVG per turn:
turn_0000.svg is the freshly created game, turn_0001.svg the state after
the first `run`, and so on. Each snapshot is a full game file you can open
in a viewer. A `history.jsonl` log records one JSON object per turn: the CLI
input, values actually applied after clamping, and outcome. `retry` restores
an earlier turn so you can try different controls; use `--keep` to archive
the abandoned attempt for comparison.

Exit codes: 0 = ok (still running), 2 = usage error, 3 = game already over,
            4 = file error, 10 = you won, 30 = you lost. `retry` also maps
            its exit code to the restored state (0/10/30).
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import os
import random
import re
import shutil
import struct
import sys
import xml.etree.ElementTree as ET
import zlib
from datetime import datetime, timezone

# --------------------------------------------------------------------------
# namespaces & constants
# --------------------------------------------------------------------------
SVG_NS = "http://www.w3.org/2000/svg"
GAME_NS = "http://arena.ai/roadrunner/game/v1"
ET.register_namespace("", SVG_NS)
ET.register_namespace("g", GAME_NS)


def _g(tag: str) -> str:
    return f"{{{GAME_NS}}}{tag}"


def _s(tag: str) -> str:
    return f"{{{SVG_NS}}}{tag}"


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


DEFAULT_RULES = {
    # which vehicle values may be changed with --set (others need force/torque)
    "settable": ["weight", "linear-drag", "angular-drag"],
    "max_force": 120.0,          # magnitude limit for the applied force
    "max_torque": 6.0,           # |torque| limit
    "max_steps_per_call": 600,   # max simulation steps per `run` call
    "max_path_off_road": 0.10,   # max fraction of the trail allowed off-road
    "min_road_coverage": 0.98,   # fraction of the road the trail must cover
    "time_limit_s": 300.0,       # total simulated seconds (null = unlimited)
    "cover_radius": None,        # px; None -> road_width/2 + 2
    "off_road_tolerance": 2.0,   # px grace when judging "on road"
    "dt": None,                  # seconds per step; None -> 1/60
}

# physics constants
RGIR2 = 0.5        # moment of inertia = weight * RGIR2 (gyration radius ~0.7px)
GRIP_LAT = 20.0    # lateral (grip) drag multiplier vs. linear drag
LATERAL_FORCE = 0.25  # lateral force input is weak: it's a "drift" input
MAX_SPEED = 300.0
MAX_OMEGA = 10.0
MAX_POINTS = 10000  # trail cap; longer trails are resampled

# turn history
HISTORY_TURN_RE = re.compile(r"^turn_(\d+)\.svg$")
HISTORY_LOG = "history.jsonl"
REDO_DIRNAME = "redo"

STYLES = {
    # road is the graph y(x) = H/2 + A1*sin(f1*x + p1) + A2*sin(f2*x + p2)
    "easy":   {"A1": 60, "f1": 0.0050, "A2": 18, "f2": 0.0140, "min_length": 700},
    "medium": {"A1": 120, "f1": 0.0065, "A2": 45, "f2": 0.0210, "min_length": 950},
    "hard":   {"A1": 150, "f1": 0.0100, "A2": 55, "f2": 0.0310, "min_length": 1350},
}

SET_ALIASES = {
    "x": "x",
    "y": "y",
    "theta": "theta", "rot": "theta", "rotation": "theta", "heading": "theta",
    "vx": "vx", "vy": "vy",
    "omega": "omega", "spin": "omega",
    "weight": "weight", "mass": "weight",
    "linear-drag": "linear_drag", "l-drag": "linear_drag", "drag": "linear_drag",
    "angular-drag": "angular_drag", "ang-drag": "angular_drag", "a-drag": "angular_drag",
    "torque": "torque",
}
SETTABLE_FRIENDLY = {
    "x": "x", "y": "y", "theta": "theta", "vx": "vx", "vy": "vy", "omega": "omega",
    "weight": "weight", "linear_drag": "linear-drag",
    "angular_drag": "angular-drag", "torque": "torque",
}


# --------------------------------------------------------------------------
# data model
# --------------------------------------------------------------------------
class Vehicle:
    __slots__ = ("x", "y", "theta", "vx", "vy", "omega", "torque",
                 "weight", "linear_drag", "angular_drag", "sim_time")

    def __init__(self, x=0.0, y=0.0, theta=0.0, vx=0.0, vy=0.0, omega=0.0,
                 torque=0.0, weight=6.0, linear_drag=1.5, angular_drag=0.5,
                 sim_time=0.0):
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)
        self.vx = float(vx)
        self.vy = float(vy)
        self.omega = float(omega)
        self.torque = float(torque)
        self.weight = float(weight)
        self.linear_drag = float(linear_drag)
        self.angular_drag = float(angular_drag)
        self.sim_time = float(sim_time)

    def to_attrs(self) -> dict:
        return {
            "x": f"{self.x:.2f}", "y": f"{self.y:.2f}",
            "theta": f"{self.theta:.4f}",
            "vx": f"{self.vx:.2f}", "vy": f"{self.vy:.2f}",
            "omega": f"{self.omega:.3f}", "torque": f"{self.torque:.3f}",
            "weight": f"{self.weight:.2f}",
            "linear-drag": f"{self.linear_drag:.3f}",
            "angular-drag": f"{self.angular_drag:.3f}",
            "sim-time": f"{self.sim_time:.2f}",
        }

    @classmethod
    def from_attrs(cls, a: dict) -> "Vehicle":
        return cls(x=a["x"], y=a["y"], theta=a["theta"], vx=a["vx"], vy=a["vy"],
                   omega=a["omega"], torque=a["torque"], weight=a["weight"],
                   linear_drag=a["linear-drag"], angular_drag=a["angular-drag"],
                   sim_time=a.get("sim-time", "0"))


class Rules:
    def __init__(self, d: dict | None):
        d = {**DEFAULT_RULES, **(d or {})}
        unknown = set(d) - set(DEFAULT_RULES)
        if unknown:
            raise ValueError(f"unknown rule keys: {sorted(unknown)}")
        self.settable = [SET_ALIASES.get(k.strip().lower()) for k in d["settable"]]
        self.settable = [k for k in self.settable if k is not None]
        self.max_force = float(d["max_force"])
        self.max_torque = float(d["max_torque"])
        self.max_steps_per_call = int(d["max_steps_per_call"])
        self.max_path_off_road = float(d["max_path_off_road"])
        self.min_road_coverage = float(d["min_road_coverage"])
        self.time_limit_s = float(d["time_limit_s"]) if d["time_limit_s"] is not None else None
        self.cover_radius = float(d["cover_radius"]) if d["cover_radius"] is not None else None
        self.off_road_tolerance = float(d["off_road_tolerance"])
        self.dt = float(d["dt"]) if d["dt"] is not None else 1.0 / 60.0

    def cover_radius_for(self, road_width: float) -> float:
        return self.cover_radius if self.cover_radius is not None else road_width / 2.0 + 2.0

    def to_attrs(self) -> dict:
        return {
            "settable": ",".join(SETTABLE_FRIENDLY[k] for k in self.settable),
            "max-force": f"{self.max_force:.1f}",
            "max-torque": f"{self.max_torque:.1f}",
            "max-steps-per-call": str(self.max_steps_per_call),
            "max-path-off-road": f"{self.max_path_off_road:.3f}",
            "min-road-coverage": f"{self.min_road_coverage:.3f}",
            "time-limit-s": "" if self.time_limit_s is None else f"{self.time_limit_s:.1f}",
            "cover-radius": "" if self.cover_radius is None else f"{self.cover_radius:.1f}",
            "off-road-tolerance": f"{self.off_road_tolerance:.1f}",
            "dt": f"{self.dt:.7f}",
        }

    @classmethod
    def from_attrs(cls, a: dict) -> "Rules":
        d = {
            "settable": [s for s in a.get("settable", "").split(",") if s],
            "max_force": a.get("max-force", DEFAULT_RULES["max_force"]),
            "max_torque": a.get("max-torque", DEFAULT_RULES["max_torque"]),
            "max_steps_per_call": a.get("max-steps-per-call", DEFAULT_RULES["max_steps_per_call"]),
            "max_path_off_road": a.get("max-path-off-road", DEFAULT_RULES["max_path_off_road"]),
            "min_road_coverage": a.get("min-road-coverage", DEFAULT_RULES["min_road_coverage"]),
            "time_limit_s": a.get("time-limit-s") or None,
            "cover_radius": a.get("cover-radius") or None,
            "off_road_tolerance": a.get("off-road-tolerance", DEFAULT_RULES["off_road_tolerance"]),
            "dt": a.get("dt") or None,
        }
        return cls(d)


class Road:
    """A deterministic road: centerline (dense 1px points) + raster grids."""

    def __init__(self, seed, style, W, H, road_width):
        self.seed = seed
        self.style = style
        self.W = W
        self.H = H
        self.road_width = road_width
        self.source = "proc"     # "proc" (procedural) | "image" (picture track)
        self.center = []         # dense centerline points
        self.samples = []        # coverage samples (road pixels)
        self.grid = bytearray(W * H)     # 1 = counted road pixel
        self.excluded = bytearray(W * H)  # 1 = road pixel, unreachable, not counted
        self.offgrid = bytearray(W * H)   # 1 = within off-road tolerance of road
        self.length = 0.0
        self.start = (0.0, 0.0, 0.0)
        self.reach_gap = 0.0     # max off-road gap (px) a component may cross
        self.gaps = {}           # component id -> gap to main component

    @classmethod
    def build(cls, seed, style, W, H, road_width, off_tol) -> "Road":
        st = STYLES[style]
        dx = 4.0
        x0, x1 = int(W * 0.05), int(W * 0.95)
        # keep the curve inside the canvas with a safe margin
        max_amp = H / 2.0 - max(50.0, road_width)
        for attempt in range(64):
            rng = random.Random(seed * 100003 + attempt)
            p1 = rng.uniform(0.0, 2.0 * math.pi)
            p2 = rng.uniform(0.0, 2.0 * math.pi)
            amp_scale = min(1.0 + 0.25 * (attempt % 4),
                            max_amp / (st["A1"] + st["A2"]))
            A1, A2 = st["A1"] * amp_scale, st["A2"] * amp_scale
            f1, f2 = st["f1"], st["f2"]

            def ty(x):
                return (H / 2.0 + A1 * math.sin(f1 * x + p1)
                        + A2 * math.sin(f2 * x + p2))

            xs = [x0 + i * dx for i in range(int((x1 - x0) / dx) + 1)]
            pts = [(x, ty(x)) for x in xs]
            L = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                    for a, b in zip(pts, pts[1:]))
            if L < st["min_length"]:
                continue
            road = cls(seed, style, W, H, road_width)
            road.center = densify(pts, 1.0)
            if len(road.center) < 80:
                continue
            road.samples = road.center[::2]
            road.length = float(len(road.center) - 1)
            road.grid = stamp_grid(road.center, W, H, road_width / 2.0)
            road.offgrid = stamp_grid(road.center, W, H, road_width / 2.0 + off_tol)
            (xa, ya), (xb, yb) = road.center[0], road.center[1]
            road.start = (xa, ya, math.atan2(yb - ya, xb - xa))
            return road
        raise RuntimeError(f"could not generate a valid road (seed={seed}, style={style})")

    @classmethod
    def build_image(cls, img_grid, W, H, road_width, off_tol, reach_gap) -> "Road":
        """Build a picture track from a binary road grid (1 = dark line pixel).
        Only the road parts reachable from the start component without crossing
        an off-road gap bigger than `reach_gap` count toward the win; the rest
        is stored in `excluded` (drawn gray, ignored)."""
        comp, pixels_by_comp = road_components(img_grid, W, H)
        if not pixels_by_comp:
            raise RuntimeError("image has no road (no dark pixels found)")
        sizes = [len(p) for p in pixels_by_comp]
        main_id = sizes.index(max(sizes)) + 1
        dist = offroad_dist(img_grid, W, H, pixels_by_comp[main_id - 1])
        gaps = component_gaps(img_grid, W, H, pixels_by_comp, main_id, dist)
        excluded = bytearray(W * H)
        for cid, g in gaps.items():
            if cid != main_id and g > reach_gap:
                for p in pixels_by_comp[cid - 1]:
                    excluded[p] = 1
        grid = bytearray(W * H)
        samples = []
        for i in range(W * H):
            if img_grid[i] and not excluded[i]:
                grid[i] = 1
                samples.append((i % W, i // W))
        road = cls(0, "image", W, H, road_width)
        road.source = "image"
        road.excluded = excluded
        road.grid = grid
        road.samples = samples
        road.center = []
        road.length = float(len(samples))
        road.offgrid = grid_dilated(grid, W, H, road_width / 2.0 + off_tol)
        road.reach_gap = float(reach_gap)
        road.gaps = gaps
        # start: on the straightest ~24px stretch of the main component
        # (topmost among the straight ones, so the first move is not a
        # convex-cap corner-cut); heading along the line, pointing "down"
        main_pix = pixels_by_comp[main_id - 1]
        CELL = 8
        gw, gh = (W + CELL - 1) // CELL, (H + CELL - 1) // CELL
        mgrid = [[] for _ in range(gw * gh)]
        for p in main_pix:
            x, y = p % W, p // W
            mgrid[(y // CELL) * gw + (x // CELL)].append(p)

        def m_near(x, y, r):
            c0x = max(0, (x - r) // CELL)
            c1x = min(gw - 1, (x + r) // CELL)
            c0y = max(0, (y - r) // CELL)
            c1y = min(gh - 1, (y + r) // CELL)
            out = []
            for cy in range(c0y, c1y + 1):
                row = cy * gw
                for cx in range(c0x, c1x + 1):
                    out.extend(mgrid[row + cx])
            return out

        def straightness(x, y, rad, min_span, max_ext):
            """(perpendicular spread, PCA angle) of the line within rad px."""
            pts = [(q % W, q // W) for q in m_near(x, y, rad)]
            if len(pts) < 8:
                return None
            mx = sum(a for a, _ in pts) / len(pts)
            my = sum(b for _, b in pts) / len(pts)
            sxx = syy = sxy = 0.0
            for a, b in pts:
                dx, dy = a - mx, b - my
                sxx += dx * dx
                syy += dy * dy
                sxy += dx * dy
            if sxx + syy < 1e-9:
                return None
            t = math.atan2(2 * sxy, sxx - syy) / 2
            ct, stn = math.cos(t), math.sin(t)
            proj = [(a - mx) * ct + (b - my) * stn for a, b in pts]
            if max(proj) - min(proj) < min_span:
                return None
            perp = [(a - mx) * -stn + (b - my) * ct for a, b in pts]
            if max(perp) - min(perp) > max_ext:
                return None            # junction, arc, or two parallel lines
            dev = math.sqrt(sum(p * p for p in perp) / len(pts))
            return dev, t

        def main_px_near(x, y, r):
            """any main-component pixel within r of (x,y)?"""
            c0 = max(0, int((x - r) // CELL))
            c1 = min(gw - 1, int((x + r) // CELL))
            r0 = max(0, int((y - r) // CELL))
            r1 = min(gh - 1, int((y + r) // CELL))
            for cy in range(r0, r1 + 1):
                row = cy * gw
                for cx in range(c0, c1 + 1):
                    if mgrid[row + cx]:
                        return True
            return False

        # corridor: how far the road keeps straight along the tangent
        def corridor(x, y, t):
            ct, stn = math.cos(t), math.sin(t)
            run = 0.0
            for sgn in (1, -1):
                s = 4.0
                while s <= 70:
                    if main_px_near(x + sgn * s * ct, y + sgn * s * stn, 2.5):
                        s += 4
                    else:
                        break
                run += s - 4
            return run

        scored = []
        for p in main_pix[::6]:
            x, y = p % W, p // W
            r = straightness(x, y, 20, 30, 4.5)
            if r is None:
                continue
            L = corridor(x, y, r[1])
            if L >= 50:
                scored.append((-L, y, x, y, r[1]))
        if scored:
            scored.sort()
            _, _, sx, sy, theta = scored[0]
            if math.sin(theta) < 0:
                theta += math.pi
            road.start = (float(sx), float(sy), theta)
        else:
            sx = sy = None
            for p in main_pix:
                x, y = p % W, p // W
                if sy is None or y < sy or (y == sy and x < sx):
                    sx, sy = x, y
            sxx = syy = sxy = 0.0
            for p in main_pix:
                x, y = p % W, p // W
                if (x - sx) ** 2 + (y - sy) ** 2 <= 64:
                    dx, dy = x - sx, y - sy
                    sxx += dx * dx
                    syy += dy * dy
                    sxy += dx * dy
            theta = math.atan2(2 * sxy, sxx - syy) / 2
            if math.cos(theta) < 0:
                theta += math.pi
            road.start = (float(sx), float(sy), theta)
        return road


def densify(pts, step: float):
    out = [pts[0]]
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        d = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(round(d / step)))
        for i in range(1, n + 1):
            out.append((x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n))
    return out


def _disk_pattern(r: float):
    R = int(math.ceil(r))
    r2 = r * r
    return [(dx, dy) for dy in range(-R, R + 1) for dx in range(-R, R + 1)
            if dx * dx + dy * dy <= r2]


def stamp_grid(center, W: int, H: int, r: float) -> bytearray:
    grid = bytearray(W * H)
    pattern = _disk_pattern(r)
    for (x, y) in center:
        xi, yi = int(round(x)), int(round(y))
        for dx, dy in pattern:
            xx, yy = xi + dx, yi + dy
            if 0 <= xx < W and 0 <= yy < H:
                grid[yy * W + xx] = 1
    return grid


# --------------------------------------------------------------------------
# PNG (pure python, RGB, filter 0)
# --------------------------------------------------------------------------
def _png_chunk(tag: bytes, data: bytes) -> bytes:
    c = tag + data
    return struct.pack(">I", len(data)) + c + struct.pack(">I", zlib.crc32(c) & 0xFFFFFFFF)


def road_png_bytes(center, W: int, H: int, road_width: float) -> bytes:
    rows = [bytearray(b"\xff" * (W * 3)) for _ in range(H)]
    pattern = _disk_pattern(road_width / 2.0)
    for (x, y) in center:
        xi, yi = int(round(x)), int(round(y))
        for dx, dy in pattern:
            xx, yy = xi + dx, yi + dy
            if 0 <= xx < W and 0 <= yy < H:
                row = rows[yy]
                j = xx * 3
                row[j] = row[j + 1] = row[j + 2] = 0
    raw = b"".join(b"\x00" + bytes(r) for r in rows)
    ihdr = struct.pack(">IIBBBBB", W, H, 8, 2, 0, 0, 0)
    return (b"\x89PNG\r\n\x1a\n"
            + _png_chunk(b"IHDR", ihdr)
            + _png_chunk(b"IDAT", zlib.compress(raw, 9))
            + _png_chunk(b"IEND", b""))


def road_png_from_grid(grid, excluded, W: int, H: int) -> bytes:
    """Render the road raster: black = counted road, gray = unreachable road."""
    rows = [bytearray(b"\xff" * (W * 3)) for _ in range(H)]
    for i in range(W * H):
        y, x = divmod(i, W)
        j = x * 3
        if excluded[i]:
            rows[y][j] = rows[y][j + 1] = rows[y][j + 2] = 150
        elif grid[i]:
            rows[y][j] = rows[y][j + 1] = rows[y][j + 2] = 0
    raw = b"".join(b"\x00" + bytes(r) for r in rows)
    ihdr = struct.pack(">IIBBBBB", W, H, 8, 2, 0, 0, 0)
    return (b"\x89PNG\r\n\x1a\n"
            + _png_chunk(b"IHDR", ihdr)
            + _png_chunk(b"IDAT", zlib.compress(raw, 9))
            + _png_chunk(b"IEND", b""))


def png_decode(data: bytes):
    """Minimal 8-bit PNG decoder (gray/RGB/RGBA, filters 0-4, no interlace).
    Returns (W, H, gray bytearray)."""
    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("not a PNG file")
    pos, W, H, ch, idat = 8, None, None, None, b""
    while pos + 8 <= len(data):
        (ln,) = struct.unpack(">I", data[pos:pos + 4])
        tag = data[pos + 4:pos + 8]
        chunk = data[pos + 8:pos + 8 + ln]
        if tag == b"IHDR":
            W, H, depth, ct, _c, _f, inter = struct.unpack(">IIBBBBB", chunk)
            if depth != 8 or inter != 0:
                raise ValueError("unsupported PNG (need 8-bit, no interlace)")
            ch = {0: 1, 2: 3, 4: 4, 6: 4}.get(ct)
            if ch is None:
                raise ValueError("unsupported PNG color type")
        elif tag == b"IDAT":
            idat += chunk
        elif tag == b"IEND":
            break
        pos += 12 + ln
    if W is None:
        raise ValueError("malformed PNG (missing IHDR)")
    raw = zlib.decompress(idat)
    stride = W * ch
    out = bytearray(W * H * ch)
    prev = bytearray(stride)
    p = 0
    for y in range(H):
        f = raw[p]
        p += 1
        line = bytearray(raw[p:p + stride])
        p += stride
        if f == 1:
            for i in range(ch, stride):
                line[i] = (line[i] + line[i - ch]) & 0xFF
        elif f == 2:
            for i in range(stride):
                line[i] = (line[i] + prev[i]) & 0xFF
        elif f == 3:
            for i in range(stride):
                left = line[i - ch] if i >= ch else 0
                line[i] = (line[i] + ((left + prev[i]) >> 1)) & 0xFF
        elif f == 4:
            for i in range(stride):
                left = line[i - ch] if i >= ch else 0
                up = prev[i]
                upleft = prev[i - ch] if i >= ch else 0
                pp = left + up - upleft
                da, db, dc = abs(pp - left), abs(pp - up), abs(pp - upleft)
                pr = left if (da <= db and da <= dc) else (up if db <= dc else upleft)
                line[i] = (line[i] + pr) & 0xFF
        elif f != 0:
            raise ValueError(f"bad PNG filter {f}")
        out[y * stride:(y + 1) * stride] = line
        prev = line
    if ch == 1:
        return W, H, out
    gray = bytearray(W * H)
    for i in range(W * H):
        j = i * ch
        gray[i] = (out[j] + out[j + 1] + out[j + 2]) // 3
    return W, H, gray


# --------------------------------------------------------------------------
# road graph analysis (picture tracks: which parts are reachable)
# --------------------------------------------------------------------------
def road_components(grid, W, H):
    """8-connected components of road pixels.
    Returns (comp bytearray with 1..n ids, pixels_by_comp list of index lists)."""
    from collections import deque
    comp = bytearray(W * H)
    pixels_by_comp = []
    for i in range(W * H):
        if not grid[i] or comp[i]:
            continue
        cid = len(pixels_by_comp)
        pixels_by_comp.append([])
        comp[i] = cid + 1
        q = deque([i])
        while q:
            p = q.popleft()
            pixels_by_comp[cid].append(p)
            x, y = p % W, p // W
            for dx in (-1, 0, 1):
                nx = x + dx
                if nx < 0 or nx >= W:
                    continue
                for dy in (-1, 0, 1):
                    ny = y + dy
                    if ny < 0 or ny >= H:
                        continue
                    j = ny * W + nx
                    if grid[j] and not comp[j]:
                        comp[j] = cid + 1
                        q.append(j)
    return comp, pixels_by_comp


def offroad_dist(grid, W, H, seeds):
    """BFS through NON-road pixels, seeded from road pixels in `seeds`.
    dist[p] = number of off-road pixels on the shortest path from p to the
    seed road (p itself counted); road pixels stay 0."""
    from collections import deque
    dist = bytearray(W * H)
    q = deque()
    for p in seeds:
        x, y = p % W, p // W
        for dx in (-1, 0, 1):
            nx = x + dx
            if nx < 0 or nx >= W:
                continue
            for dy in (-1, 0, 1):
                ny = y + dy
                if ny < 0 or ny >= H:
                    continue
                j = ny * W + nx
                if not grid[j] and not dist[j]:
                    dist[j] = 1
                    q.append(j)
    while q:
        p = q.popleft()
        d = dist[p]
        x, y = p % W, p // W
        if d >= 250:
            continue
        nd = d + 1
        for dx in (-1, 0, 1):
            nx = x + dx
            if nx < 0 or nx >= W:
                continue
            for dy in (-1, 0, 1):
                ny = y + dy
                if ny < 0 or ny >= H:
                    continue
                j = ny * W + nx
                if not grid[j] and not dist[j]:
                    dist[j] = nd
                    q.append(j)
    return dist


def component_gaps(grid, W, H, pixels_by_comp, main_id, dist):
    """Minimum off-road gap (in px) between each component and the main one."""
    gaps = {main_id: 0}
    for idx, pix in enumerate(pixels_by_comp):
        cid = idx + 1
        if cid == main_id:
            continue
        g = 250
        for p in pix:
            x, y = p % W, p // W
            for dx in (-1, 0, 1):
                nx = x + dx
                if nx < 0 or nx >= W:
                    continue
                for dy in (-1, 0, 1):
                    ny = y + dy
                    if ny < 0 or ny >= H:
                        continue
                    j = ny * W + nx
                    if not grid[j] and 0 < dist[j] < g:
                        g = dist[j]
        gaps[cid] = g
    return gaps


def grid_dilated(grid, W, H, r):
    out = bytearray(W * H)
    pattern = _disk_pattern(r)
    for i in range(W * H):
        if not grid[i]:
            continue
        x, y = i % W, i // W
        for dx, dy in pattern:
            xx, yy = x + dx, y + dy
            if 0 <= xx < W and 0 <= yy < H:
                out[yy * W + xx] = 1
    return out


# --------------------------------------------------------------------------
# metrics
# --------------------------------------------------------------------------
def compute_metrics(path, road: Road, rules: Rules):
    """Return (coverage, off_ratio) for the given trail."""
    W, H = road.W, road.H

    off = 0
    for (x, y) in path:
        xi, yi = int(round(x)), int(round(y))
        if not (0 <= xi < W and 0 <= yi < H) or not road.offgrid[yi * W + xi]:
            off += 1
    off_ratio = off / len(path) if path else 0.0

    R = int(math.ceil(rules.cover_radius_for(road.road_width)))
    pg = bytearray(W * H)
    pattern = _disk_pattern(R)
    for (x, y) in path:
        xi, yi = int(round(x)), int(round(y))
        if 0 <= xi < W and 0 <= yi < H:
            for dx, dy in pattern:
                xx, yy = xi + dx, yi + dy
                if 0 <= xx < W and 0 <= yy < H:
                    pg[yy * W + xx] = 1
    hit = 0
    for (sx, sy) in road.samples:
        if pg[int(sy) * W + int(sx)]:
            hit += 1
    coverage = hit / len(road.samples) if road.samples else 0.0
    return coverage, off_ratio


# --------------------------------------------------------------------------
# physics
# --------------------------------------------------------------------------
def step_vehicle(v: Vehicle, fx: float, fy: float, dt: float):
    """One integration step. fx, fy are world-frame forces."""
    c, s = math.cos(v.theta), math.sin(v.theta)
    # world -> local (x_local = forward, y_local = right)
    f_l = fx * c + fy * s
    f_lat = -fx * s + fy * c
    v_l = v.vx * c + v.vy * s
    v_lat = -v.vx * s + v.vy * c
    # longitudinal dynamics + lateral grip
    a_l = (f_l - v.linear_drag * v_l) / v.weight
    a_lat = (LATERAL_FORCE * f_lat - GRIP_LAT * v.linear_drag * v_lat) / v.weight
    # local -> world
    dvx = (a_l * c - a_lat * s) * dt
    dvy = (a_l * s + a_lat * c) * dt
    v.vx += dvx
    v.vy += dvy
    # rotation
    alpha = (v.torque - v.angular_drag * v.omega) / (v.weight * RGIR2)
    v.omega = clamp(v.omega + alpha * dt, -MAX_OMEGA, MAX_OMEGA)
    v.theta += v.omega * dt
    # integrate position
    v.x += v.vx * dt
    v.y += v.vy * dt
    sp = math.hypot(v.vx, v.vy)
    if sp > MAX_SPEED:
        v.vx *= MAX_SPEED / sp
        v.vy *= MAX_SPEED / sp
    v.x = clamp(v.x, -2000.0, 2000.0)
    v.y = clamp(v.y, -2000.0, 2000.0)


def run_sim(v: Vehicle, path, fx: float, fy: float, steps: int, dt: float):
    for _ in range(steps):
        step_vehicle(v, fx, fy, dt)
        v.sim_time += dt
        path.append((round(v.x, 2), round(v.y, 2)))
    if len(path) > MAX_POINTS:
        k = (len(path) + MAX_POINTS - 1) // MAX_POINTS
        path = path[::k]
    return path


# --------------------------------------------------------------------------
# game file (SVG) I/O
# --------------------------------------------------------------------------
class Game:
    def __init__(self, road: Road, rules: Rules, vehicle: Vehicle, path,
                 state, reason, history: str = "on"):
        self.road = road
        self.rules = rules
        self.vehicle = vehicle
        self.path = path
        self.state = state      # running | won | lost
        self.reason = reason    # "" | "off-road" | "time"
        self.history = history  # "on" | "off" (per-turn snapshots)

    @property
    def file_name(self):
        return getattr(self, "_file_name", "game.svg")


def points_str(path) -> str:
    return " ".join(f"{x:.1f},{y:.1f}" for (x, y) in path)


def write_game(path: str, game: Game, coverage: float, off_ratio: float):
    road, rules, v = game.road, game.rules, game.vehicle
    ET.register_namespace("", SVG_NS)
    ET.register_namespace("g", GAME_NS)

    root = ET.Element(_s("svg"), {
        "version": "1.1",
        "width": str(road.W), "height": str(road.H),
        "viewBox": f"0 0 {road.W} {road.H}",
    })
    ET.SubElement(root, _g("meta"), {"version": "1", "game": "roadrunner",
                                     "history": game.history})
    ra = {
        "width": str(road.W), "height": str(road.H),
        "road-width": f"{road.road_width:.1f}",
        "length": f"{road.length:.0f}",
    }
    if road.source == "image":
        ra.update({
            "source": "image",
            "start-x": f"{road.start[0]:.2f}", "start-y": f"{road.start[1]:.2f}",
            "start-theta": f"{road.start[2]:.4f}",
            "reach-gap": f"{road.reach_gap:.0f}",
            "gaps": ";".join(f"{k}:{v}" for k, v in sorted(road.gaps.items())),
        })
        png = road_png_from_grid(road.grid, road.excluded, road.W, road.H)
    else:
        ra.update({"seed": str(road.seed), "style": road.style})
        png = road_png_bytes(road.center, road.W, road.H, road.road_width)
    ET.SubElement(root, _g("road"), ra)
    ET.SubElement(root, _g("rules"), rules.to_attrs())
    ET.SubElement(root, _g("vehicle"), v.to_attrs())
    ET.SubElement(root, _g("path"), {
        "points-count": str(len(game.path)),
        "points": points_str(game.path),
    })
    ET.SubElement(root, _g("status"), {
        "state": game.state,
        "reason": game.reason,
        "coverage": f"{coverage:.4f}",
        "off-road": f"{off_ratio:.4f}",
    })

    b64 = base64.b64encode(png).decode("ascii")
    ET.SubElement(root, _s("image"), {
        "href": f"data:image/png;base64,{b64}",
        "x": "0", "y": "0", "width": str(road.W), "height": str(road.H),
    })

    pts = points_str(game.path)
    ET.SubElement(root, _s("polyline"), {
        "points": pts, "fill": "none", "stroke": "#2563eb",
        "stroke-width": "1.4", "stroke-linecap": "round",
        "stroke-linejoin": "round", "opacity": "0.9",
    })
    (x0, y0) = game.path[0]
    ET.SubElement(root, _s("circle"), {
        "cx": f"{x0:.1f}", "cy": f"{y0:.1f}", "r": "3",
        "fill": "#2563eb", "stroke": "#1e3a8a", "stroke-width": "0.8",
    })

    deg = math.degrees(v.theta) % 360.0
    marker = ET.SubElement(root, _s("g"), {
        "transform": f"translate({v.x:.1f},{v.y:.1f}) rotate({deg:.2f})"
    })
    ET.SubElement(marker, _s("rect"), {
        "x": "-7", "y": "-4.5", "width": "14", "height": "9", "rx": "2",
        "fill": "#ef4444", "stroke": "#7f1d1d", "stroke-width": "1",
    })
    ET.SubElement(marker, _s("polygon"), {
        "points": "8,0 -1,4 -1,-4", "fill": "#fbbf24",
        "stroke": "#92400e", "stroke-width": "0.6",
    })

    speed = math.hypot(v.vx, v.vy)
    g = ET.SubElement(root, _s("g"), {
        "font-family": "monospace", "font-size": "12", "fill": "#111827",
    })
    ET.SubElement(g, _s("text"), {"x": "8", "y": "18"}).text = (
        f"roadrunner  {game.state.upper()}"
        + (f" ({game.reason})" if game.reason else "")
    )
    ET.SubElement(g, _s("text"), {"x": "8", "y": "33"}).text = (
        f"coverage {coverage:.1%}   off-road {off_ratio:.1%}   "
        f"time {v.sim_time:.1f}s   speed {speed:.1f} px/s"
    )

    tree = ET.ElementTree(root)
    tree.write(path, encoding="utf-8", xml_declaration=True)
    game._file_name = os.path.basename(path)


# --------------------------------------------------------------------------
# turn history (GAME.history/: one snapshot SVG per turn + history.jsonl)
# --------------------------------------------------------------------------
def _utcnow() -> str:
    return (datetime.now(timezone.utc).isoformat(timespec="seconds")
            .replace("+00:00", "Z"))


def history_dir(game_path: str) -> str:
    """game.svg -> game.history/ (next to the game file)."""
    base = os.path.basename(game_path)
    stem = base[:-4] if base.lower().endswith(".svg") else base
    return os.path.join(os.path.dirname(os.path.abspath(game_path)),
                        stem + ".history")


def list_turns(hdir: str):
    """[(turn_index, filename)] present in the folder, sorted by turn."""
    out = []
    if os.path.isdir(hdir):
        for name in os.listdir(hdir):
            m = HISTORY_TURN_RE.match(name)
            if m:
                out.append((int(m.group(1)), name))
    return sorted(out)


def save_snapshot(hdir: str, turn: int, game: Game,
                  coverage: float, off_ratio: float) -> str:
    """Write GAME.history/turn_NNNN.svg — a complete, playable game file."""
    os.makedirs(hdir, exist_ok=True)
    fname = os.path.join(hdir, f"turn_{turn:04d}.svg")
    tmp = fname + ".tmp"
    prev = getattr(game, "_file_name", "game.svg")
    try:
        write_game(tmp, game, coverage, off_ratio)
    finally:
        game._file_name = prev
    os.replace(tmp, fname)
    return fname


def append_history_row(hdir: str, row: dict):
    with open(os.path.join(hdir, HISTORY_LOG), "a", encoding="utf-8") as f:
        f.write(json.dumps(row, separators=(",", ":")) + "\n")


def read_history_rows(hdir: str):
    rows = []
    try:
        with open(os.path.join(hdir, HISTORY_LOG), "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    rows.append(json.loads(line))
                except ValueError:
                    pass
    except OSError:
        pass
    return rows


def rewrite_history(hdir: str, keep_until: int):
    """Drop log entries above `keep_until` (called after a rewind)."""
    rows = [r for r in read_history_rows(hdir)
            if int(r.get("turn", -1)) <= keep_until]
    with open(os.path.join(hdir, HISTORY_LOG), "w", encoding="utf-8") as f:
        for r in rows:
            f.write(json.dumps(r, separators=(",", ":")) + "\n")


def clear_history(hdir: str):
    for _i, name in list_turns(hdir):
        try:
            os.remove(os.path.join(hdir, name))
        except OSError:
            pass
    log = os.path.join(hdir, HISTORY_LOG)
    if os.path.isfile(log):
        os.remove(log)
    rd = os.path.join(hdir, REDO_DIRNAME)
    if os.path.isdir(rd):
        shutil.rmtree(rd, ignore_errors=True)


def read_game(path: str) -> Game:
    try:
        root = ET.parse(path).getroot()
    except (OSError, ET.ParseError) as e:
        die(f"cannot read game file '{path}': {e}", 4)
    need = ("road", "rules", "vehicle", "path", "status")
    els = {}
    for tag in need:
        el = root.find(_g(tag))
        if el is None:
            die(f"'{path}' is not a roadrunner game file (missing <g:{tag}>)", 4)
        els[tag] = el
    a = lambda t, k: els[t].get(k)
    if a("vehicle", "x") is None:
        die(f"'{path}' is not a roadrunner game file (missing attributes)", 4)

    rules = Rules.from_attrs({k: v for k, v in els["rules"].items()})
    if a("road", "source") == "image":
        img_el = root.find(_s("image"))
        href = img_el.get("href") if img_el is not None else None
        if not href or not href.startswith("data:image/png;base64,"):
            die(f"'{path}' has no embedded road image", 4)
        W2, H2, gray = png_decode(base64.b64decode(href.split(",", 1)[1]))
        grid = bytearray(W2 * H2)
        excluded = bytearray(W2 * H2)
        for i, gv in enumerate(gray):
            if gv < 64:
                grid[i] = 1
            elif gv < 220:
                excluded[i] = 1
        road = Road(0, "image", W2, H2, float(a("road", "road-width")))
        road.source = "image"
        road.excluded = excluded
        road.grid = grid
        road.samples = [(i % W2, i // W2) for i in range(W2 * H2) if grid[i]]
        road.length = float(len(road.samples))
        road.offgrid = grid_dilated(
            grid, W2, H2, road.road_width / 2.0 + rules.off_road_tolerance)
        road.start = (float(a("road", "start-x")), float(a("road", "start-y")),
                      float(a("road", "start-theta")))
        road.reach_gap = float(a("road", "reach-gap") or "40")
        for part in (a("road", "gaps") or "").split(";"):
            if ":" in part:
                k, gv = part.split(":")
                road.gaps[int(k)] = int(gv)
    else:
        if a("road", "seed") is None:
            die(f"'{path}' is not a roadrunner game file (missing attributes)", 4)
        road = Road.build(
            int(a("road", "seed")), a("road", "style"),
            int(a("road", "width")), int(a("road", "height")),
            float(a("road", "road-width")),
            rules.off_road_tolerance,
        )
    v = Vehicle.from_attrs({k: v2 for k, v2 in els["vehicle"].items()})
    pts_raw = a("path", "points") or ""
    try:
        trail = [tuple(map(float, p.split(","))) for p in pts_raw.split()]
    except ValueError:
        die(f"'{path}' has a malformed <g:path points>", 4)
    if not trail:
        trail = [road.start[:2]]
    meta = root.find(_g("meta"))
    history = meta.get("history", "on") if meta is not None else "on"
    if history not in ("on", "off"):
        history = "on"
    game = Game(road, rules, v, trail, a("status", "state") or "running",
                a("status", "reason") or "", history)
    game._file_name = os.path.basename(path)
    return game


# --------------------------------------------------------------------------
# ascii map (terminal view)
# --------------------------------------------------------------------------
def dir_char(theta: float) -> str:
    a = math.degrees(theta) % 360
    if a < 22.5 or a >= 337.5:
        return "→"
    if a < 67.5:
        return "↘"
    if a < 112.5:
        return "↓"
    if a < 157.5:
        return "↙"
    if a < 202.5:
        return "←"
    if a < 247.5:
        return "↖"
    if a < 292.5:
        return "↑"
    return "↗"


def ascii_map(game: Game) -> str:
    road, v = game.road, game.vehicle
    cols, rows = 88, 26
    cell_road = set()
    W, H = road.W, road.H
    for idx, val in enumerate(road.grid):
        if val:
            yy, xx = divmod(idx, W)
            c = min(cols - 1, int(xx * cols / W))
            r = min(rows - 1, int(yy * rows / H))
            cell_road.add((c, r))
    cell_path_on, cell_path_off = set(), set()
    for (x, y) in game.path:
        if not (0 <= x < W and 0 <= y < H):
            continue
        c = min(cols - 1, int(x * cols / W))
        r = min(rows - 1, int(y * rows / H))
        xi, yi = int(round(x)), int(round(y))
        (cell_path_on if road.offgrid[yi * W + xi] else cell_path_off).add((c, r))
    vc = min(cols - 1, int(v.x * cols / W))
    vr = min(rows - 1, int(v.y * rows / H))
    lines = []
    for r in range(rows):
        line = []
        for c in range(cols):
            ch = "."
            if (c, r) in cell_road:
                ch = "#"
            if (c, r) in cell_path_on:
                ch = "@"
            if (c, r) in cell_path_off:
                ch = "x"
            line.append(ch)
        if (vr, vc) == (r, None) or (vr == r and 0 <= vc < cols and (vr, vc) is not None):
            if vr == r:
                line[vc] = "A"
        lines.append("".join(line))
    border = "+" + "-" * cols + "+"
    out = [border]
    for ln in lines:
        out.append("|" + ln + "|")
    out.append(border)
    out.append("  # road    @ trail on road    x trail off road    A vehicle "
               f"({dir_char(v.theta)})")
    return "\n".join(out)


# --------------------------------------------------------------------------
# reporting
# --------------------------------------------------------------------------
def speed(v: Vehicle) -> float:
    return math.hypot(v.vx, v.vy)


def vehicle_record(v: Vehicle) -> dict:
    """Stable, JSON-friendly state values for the per-turn history log."""
    return {
        "x": round(v.x, 2), "y": round(v.y, 2),
        "theta_rad": round(v.theta, 5),
        "theta_degrees": round(math.degrees(v.theta), 3),
        "vx": round(v.vx, 2), "vy": round(v.vy, 2),
        "omega": round(v.omega, 4), "torque": round(v.torque, 3),
        "weight": round(v.weight, 3),
        "linear_drag": round(v.linear_drag, 3),
        "angular_drag": round(v.angular_drag, 3),
    }


def print_summary(game: Game, coverage: float, off_ratio: float,
                  notes=None, input_desc=None):
    road, v, rules = game.road, game.vehicle, game.rules
    name = game.file_name
    print(f"→ roadrunner  {name}")
    if input_desc:
        print(f"  input:   {input_desc}")
    for n in notes or []:
        print(f"  note:    {n}")
    print(f"  vehicle: pos ({v.x:.1f}, {v.y:.1f})  "
          f"heading {math.degrees(v.theta) % 360:.1f}°  "
          f"speed {speed(v):.1f} px/s  spin {v.omega:.2f} rad/s  "
          f"torque {v.torque:+.2f}")
    print(f"  tuning:  weight {v.weight:.1f}  lin-drag {v.linear_drag:.2f}  "
          f"ang-drag {v.angular_drag:.2f}")
    tl = "" if rules.time_limit_s is None else f" / {rules.time_limit_s:.0f}s"
    print(f"  road:    covered {coverage:.1%} (need {rules.min_road_coverage:.0%})   "
          f"off-road {off_ratio:.1%} (limit {rules.max_path_off_road:.0%})   "
          f"time {v.sim_time:.1f}s{tl}")
    print(f"  state:   {game.state.upper()}"
          + (f"  ({game.reason})" if game.reason else ""))


# --------------------------------------------------------------------------
# commands
# --------------------------------------------------------------------------
def die(msg: str, code: int = 2):
    print(f"error: {msg}", file=sys.stderr)
    sys.exit(code)


def cmd_new(args):
    if os.path.exists(args.game) and not args.force:
        die(f"'{args.game}' exists; use --force to overwrite it", 4)
    if args.rules:
        try:
            with open(args.rules, "r", encoding="utf-8") as f:
                rules_dict = json.load(f)
        except (OSError, ValueError) as e:
            die(f"cannot read rules file: {e}", 4)
    else:
        rules_dict = {}
    try:
        rules = Rules(rules_dict)
    except ValueError as e:
        die(str(e), 4)
    if args.image:
        try:
            with open(args.image, "rb") as f:
                W2, H2, gray = png_decode(f.read())
        except (OSError, ValueError) as e:
            die(f"cannot read image '{args.image}': {e}", 4)
        args.width, args.height = W2, H2
        rw = args.road_width if args.road_width else 3.0
        img_grid = bytearray(1 if b < 128 else 0 for b in gray)
        try:
            road = Road.build_image(img_grid, W2, H2, rw,
                                    rules.off_road_tolerance, args.reach_gap)
        except RuntimeError as e:
            die(str(e), 4)
        seed = 0
    else:
        rw = args.road_width if args.road_width else 9.0
        seed = args.seed if args.seed is not None else random.randrange(1, 10 ** 6)
        try:
            road = Road.build(seed, args.style, args.width, args.height,
                              rw, rules.off_road_tolerance)
        except RuntimeError as e:
            die(str(e), 4)
    (sx, sy, st) = road.start
    v = Vehicle(x=sx, y=sy, theta=st)
    game = Game(road, rules, v, [(round(sx, 2), round(sy, 2))], "running", "",
                history=("off" if args.no_history else "on"))
    cov, off = compute_metrics(game.path, road, rules)
    write_game(args.game, game, cov, off)
    hdir = history_dir(args.game)
    # A forced new game starts a fresh timeline, even if history is disabled;
    # otherwise stale snapshots could be mistaken for this game's history.
    if os.path.isdir(hdir):
        clear_history(hdir)
        if game.history == "off":
            try:
                os.rmdir(hdir)
            except OSError:
                pass
    if game.history == "on":
        os.makedirs(hdir, exist_ok=True)
        save_snapshot(hdir, 0, game, cov, off)
        append_history_row(hdir, {
            "turn": 0, "at": _utcnow(), "event": "new",
            "style": road.style, "seed": (road.seed if road.source != "image"
                                          else None),
            "sim_time": 0.0, "vehicle_after": vehicle_record(v),
            "coverage": round(cov, 6),
            "off_road": round(off, 6), "state": "running",
            "desc": "initial state",
        })

    print(f"created {args.game}")
    if road.source == "image":
        n_exc = sum(road.excluded)
        print(f"  road:    picture track  canvas {road.W}x{road.H}  "
              f"{road.length:.0f}px of road line")
        print(f"            winnable: {road.length:.0f}px  "
              f"({n_exc}px in unreachable parts, gap limit "
              f"{road.reach_gap:.0f}px)")
        frag_gaps = sorted(g for g in road.gaps.values() if g != 0)
        if frag_gaps:
            print(f"            disconnected fragments: {len(frag_gaps)} "
                  f"(gaps: {', '.join(f'{g}px' for g in frag_gaps)})")
        else:
            print("            one connected piece, no gaps")
    else:
        print(f"  road:    style={args.style}  seed={seed}  "
              f"{road.length:.0f}px long  width={rw:.0f}px  "
              f"canvas {road.W}x{road.H}")
    print(f"  car:     parked at ({sx:.1f}, {sy:.1f}), "
          f"heading {math.degrees(st) % 360:.0f}°")
    print(f"  rules:   settable: {', '.join(SETTABLE_FRIENDLY[k] for k in rules.settable) or '—'}")
    print(f"            max force {rules.max_force:.0f}  max torque {rules.max_torque:.1f}  "
          f"max steps/call {rules.max_steps_per_call}")
    print(f"            cover {rules.min_road_coverage:.0%} of road, "
          f"≤ {rules.max_path_off_road:.0%} off-road"
          + (f", time limit {rules.time_limit_s:.0f}s" if rules.time_limit_s else ""))
    if game.history == "on":
        print(f"  history: {os.path.basename(hdir)}/turn_0000.svg (one SVG per turn)")
        print(f"  retry:   roadrunner.py retry {args.game}   "
              f"# rewind a turn and replay it")
    print(f"  start:   roadrunner.py run {args.game} --fx 60 --torque 1.0 --steps 60")
    print(f"  watch:   roadrunner.py status {args.game} --map")


def cmd_run(args):
    try:
        game = read_game(args.game)
    except SystemExit:
        raise
    v, rules = game.vehicle, game.rules
    if game.state in ("won", "lost"):
        cov, off = compute_metrics(game.path, game.road, rules)
        print(f"{game.file_name}: game is already over ({game.state}"
              + (f", {game.reason}" if game.reason else "") + ").")
        print("  create a fresh one with:  roadrunner.py new "
              f"{game.file_name} --force")
        if game.history == "on" and list_turns(history_dir(args.game)):
            print("  or rewind to an earlier turn:  roadrunner.py retry "
                  f"{game.file_name}")
        return 3

    # history: if this game has no snapshots yet (e.g. created before the
    # history feature), capture the current state as turn 0 first, so the
    # first recorded turn below can be retried like any other.
    hdir = history_dir(args.game)
    history_on = game.history == "on"
    if history_on and not list_turns(hdir):
        cov0, off0 = compute_metrics(game.path, game.road, rules)
        save_snapshot(hdir, 0, game, cov0, off0)
        append_history_row(hdir, {
            "turn": 0, "at": _utcnow(), "event": "init",
            "sim_time": round(v.sim_time, 2),
            "vehicle_after": vehicle_record(v),
            "coverage": round(cov0, 6),
            "off_road": round(off0, 6), "state": game.state,
            "desc": "history started mid-game",
        })

    notes = []
    set_notes = []
    set_applied = {}

    # 1) direct value setting (restricted by rules)
    for spec in args.set or []:
        if "=" not in spec:
            die(f"bad --set '{spec}', expected key=value", 2)
        key, raw = spec.split("=", 1)
        key = key.strip().lower()
        canon = SET_ALIASES.get(key)
        if canon is None:
            die(f"unknown vehicle field '{key}' "
                f"(fields: {', '.join(SET_ALIASES)})", 2)
        if canon not in rules.settable:
            allowed = ", ".join(SETTABLE_FRIENDLY[k] for k in rules.settable) or "nothing"
            die(f"'{SETTABLE_FRIENDLY[canon]}' cannot be set directly in this game; "
                f"allowed: {allowed}", 2)
        try:
            val = float(raw)
        except ValueError:
            die(f"bad value for {key}: '{raw}'", 2)
        if canon == "theta":
            before = v.theta
            setattr(v, "theta", math.radians(val))
            set_notes.append(f"theta {before:.2f} → {math.radians(val):.2f} rad")
        elif canon == "x":
            setattr(v, "x", clamp(val, -2000, 2000))
            set_notes.append(f"x → {v.x:.1f}")
        elif canon == "y":
            setattr(v, "y", clamp(val, -2000, 2000))
            set_notes.append(f"y → {v.y:.1f}")
        elif canon in ("vx", "vy"):
            setattr(v, canon, clamp(val, -MAX_SPEED, MAX_SPEED))
            set_notes.append(f"{canon} → {getattr(v, canon):.1f}")
        elif canon == "omega":
            setattr(v, "omega", clamp(val, -MAX_OMEGA, MAX_OMEGA))
            set_notes.append(f"omega → {v.omega:.2f}")
        elif canon == "weight":
            setattr(v, "weight", clamp(val, 0.5, 100.0))
            set_notes.append(f"weight → {v.weight:.1f}")
        elif canon in ("linear_drag", "angular_drag"):
            setattr(v, canon, clamp(val, 0.0, 50.0))
            set_notes.append(f"{SETTABLE_FRIENDLY[canon]} → {getattr(v, canon):.2f}")
        elif canon == "torque":
            setattr(v, "torque", clamp(val, -rules.max_torque, rules.max_torque))
            set_notes.append(f"torque → {v.torque:.2f}")
        if canon == "theta":
            set_applied["theta_degrees"] = round(math.degrees(v.theta), 4)
        else:
            set_applied[SETTABLE_FRIENDLY[canon]] = getattr(v, canon)

    # 2) torque (engine input, persists in state)
    if args.torque is not None:
        t = args.torque
        if abs(t) > rules.max_torque:
            notes.append(f"torque clamped to {rules.max_torque:+.1f}")
            t = clamp(t, -rules.max_torque, rules.max_torque)
        v.torque = t

    # 3) force (applied for this call only)
    if args.force is not None:
        ang = math.radians(args.angle or 0.0)
        f_local, f_lat = args.force * math.cos(ang), args.force * math.sin(ang)
    else:
        f_local, f_lat = args.fx or 0.0, args.fy or 0.0
    if args.frame == "global":
        fx, fy = f_local, f_lat
    else:
        c, s = math.cos(v.theta), math.sin(v.theta)
        fx, fy = f_local * c - f_lat * s, f_local * s + f_lat * c
    mag = math.hypot(fx, fy)
    if mag > rules.max_force > 0:
        notes.append(f"force clamped to {rules.max_force:.0f}")
        fx *= rules.max_force / mag
        fy *= rules.max_force / mag
        mag = rules.max_force

    steps = int(clamp(args.steps, 0, rules.max_steps_per_call))
    if args.steps > rules.max_steps_per_call:
        notes.append(f"steps clamped to {rules.max_steps_per_call}")

    # 4) simulate
    game.path = run_sim(v, game.path, fx, fy, steps, rules.dt)

    # 5) evaluate
    coverage, off_ratio = compute_metrics(game.path, game.road, rules)
    if off_ratio > rules.max_path_off_road:
        game.state, game.reason = "lost", "off-road"
    elif rules.time_limit_s is not None and v.sim_time > rules.time_limit_s:
        game.state, game.reason = "lost", "time"
    elif coverage >= rules.min_road_coverage:
        game.state, game.reason = "won", ""

    write_game(args.game, game, coverage, off_ratio)

    if args.force is not None:
        fdesc = f"force {args.force:+.1f} @ {args.angle:.0f}°"
    else:
        fdesc = f"fx {args.fx or 0.0:+.1f} fy {args.fy or 0.0:+.1f}"
    in_desc = (
        f"{fdesc} ({args.frame})  "
        f"torque {v.torque:+.2f}  steps {steps} ({steps * rules.dt:.2f}s)"
    )
    if set_notes:
        in_desc += "  |  " + "  ".join(set_notes)

    # history: snapshot the post-run state as the newest turn + log the input
    if history_on:
        turns = list_turns(hdir)
        turn_no = turns[-1][0] + 1 if turns else 0
        save_snapshot(hdir, turn_no, game, coverage, off_ratio)
        append_history_row(hdir, {
            "turn": turn_no, "at": _utcnow(), "event": "run",
            "desc": in_desc,
            "input": {"fx": args.fx, "fy": args.fy, "force": args.force,
                      "angle": args.angle, "frame": args.frame,
                      "torque": args.torque, "set": args.set or [],
                      "steps": args.steps},
            "applied": {"force_world": {"fx": round(fx, 3),
                                           "fy": round(fy, 3)},
                        "torque": round(v.torque, 3), "steps": steps,
                        "set": set_applied},
            "sim_time": round(v.sim_time, 2),
            "vehicle_after": vehicle_record(v),
            "coverage": round(coverage, 6), "off_road": round(off_ratio, 6),
            "state": game.state, "reason": game.reason or None,
        })
        notes.append(f"history: turn {turn_no} saved → "
                     f"{os.path.basename(hdir)}/turn_{turn_no:04d}.svg")

    if not args.quiet:
        print_summary(game, coverage, off_ratio, notes, in_desc)
        if game.state == "won":
            print()
            print("  ╔══════════════════════════════════════════════════╗")
            print(f"  ║  WIN! road {coverage:.1%} covered, only "
                  f"{off_ratio:.1%} off-road  ║")
            print(f"  ║  open {game.file_name} to admire your trail   ║")
            print("  ╚══════════════════════════════════════════════════╝")
        elif game.state == "lost":
            print()
            print(f"  *** LOST: {game.reason} ***  start over with "
                  f"'roadrunner.py new {game.file_name} --force'"
                  + (f"  —  or rewind the turn and retry it: "
                     f"'roadrunner.py retry {game.file_name}'"
                     if history_on else ""))
    if game.state == "won":
        return 10
    if game.state == "lost":
        return 30
    return 0


def cmd_status(args):
    game = read_game(args.game)
    coverage, off_ratio = compute_metrics(game.path, game.road, game.rules)
    print_summary(game, coverage, off_ratio)
    print(f"  settable: {', '.join(SETTABLE_FRIENDLY[k] for k in game.rules.settable) or 'nothing'}")
    if game.road.source == "image":
        n_exc = sum(game.road.excluded)
        print(f"  road:     picture track  {game.road.length:.0f}px winnable"
              + (f"  ({n_exc}px unreachable)" if n_exc else "")
              + f"  (trail: {len(game.path)} points)")
    else:
        print(f"  road:     style={game.road.style}  seed={game.road.seed}  "
              f"{game.road.length:.0f}px  (trail: {len(game.path)} points)")
    hdir = history_dir(args.game)
    turns = list_turns(hdir) if game.history == "on" else []
    if turns:
        print(f"  turn:     {turns[-1][0]}   (snapshots: {hdir}/ - "
              f"{len(turns)} saved, retry = 'roadrunner.py retry "
              f"{game.file_name}')")
    elif game.history == "on":
        print("  turn:     —   (turn snapshots start on the next run)")
    if args.map:
        print()
        print(ascii_map(game))
    return 0


def cmd_retry(args):
    """Rewind the game to an earlier turn (default: one turn back) and
    restore that snapshot over the game file, so the turn can be replayed."""
    if args.turns is not None and args.to is not None:
        die("use either --turns or --to, not both", 2)
    hdir = history_dir(args.game)
    turns = list_turns(hdir)
    if not turns:
        hint = ""
        if os.path.isfile(args.game):
            try:
                if read_game(args.game).history == "off":
                    hint = " — this game was created with --no-history"
            except SystemExit:
                pass
        die(f"no history found for '{args.game}'{hint}", 4)
    cur = turns[-1][0]
    if args.turns is not None and args.turns < 1:
        die("--turns must be a positive integer", 2)
    if args.to is not None:
        target = args.to
    elif args.turns is not None:
        target = cur - args.turns
    else:
        target = cur - 1
    if target >= cur:
        die(f"already at turn {cur}; nothing to rewind", 2)
    if target < 0:
        die(f"cannot rewind to turn {target}; history starts at turn "
            f"{turns[0][0]}", 2)
    have = dict(turns)
    if target not in have:
        die(f"no snapshot for turn {target} "
            f"(snapshots: turn {turns[0][0]}..{cur})", 4)
    snap = os.path.join(hdir, have[target])
    restored = read_game(snap)          # validate before touching anything
    discarded = [(i, name) for i, name in turns if i > target]
    archived_to = None
    if args.keep:
        # Keep each abandoned timeline in its own uniquely named folder. Never
        # overwrite an earlier attempt: comparisons are the point of retries.
        redo_root = os.path.join(hdir, REDO_DIRNAME)
        os.makedirs(redo_root, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        branch_name = f"attempt_{stamp}_from_{target:04d}"
        branch_dir = os.path.join(redo_root, branch_name)
        os.makedirs(branch_dir)
        for _i, name in discarded:
            shutil.move(os.path.join(hdir, name), os.path.join(branch_dir, name))
        log = os.path.join(hdir, HISTORY_LOG)
        if os.path.isfile(log):
            shutil.copyfile(log, os.path.join(branch_dir, HISTORY_LOG))
        archived_to = os.path.relpath(branch_dir, os.path.dirname(hdir))
    else:
        for _i, name in discarded:
            os.remove(os.path.join(hdir, name))
    shutil.copyfile(snap, args.game)
    rewrite_history(hdir, target)

    coverage, off_ratio = compute_metrics(restored.path, restored.road,
                                          restored.rules)
    restored._file_name = os.path.basename(args.game)
    if not args.quiet:
        print(f"rewound {cur - target} turn(s):  turn {cur} → turn {target}"
              + (f"  (attempt archived: {archived_to}/)"
                 if archived_to else ""))
        print_summary(restored, coverage, off_ratio)
        print(f"  next:    roadrunner.py run {args.game} "
              f"--fx 60 --torque 0.5 --steps 60")
    if restored.state == "won":
        return 10
    if restored.state == "lost":
        return 30
    return 0


def cmd_history(args):
    """Show the per-turn log (turns, inputs, outcomes)."""
    hdir = history_dir(args.game)
    rows = read_history_rows(hdir)
    snaps = list_turns(hdir)
    if not rows and not snaps:
        die(f"no history found for '{args.game}'", 4)
    if args.tail is not None:
        if args.tail < 0:
            die("--tail must be zero or a positive integer", 2)
        rows = rows[-args.tail:] if args.tail else []
    if args.json:
        for r in rows:
            print(json.dumps(r, separators=(",", ":")))
        return 0
    print(f"history: {hdir}   ({len(snaps)} snapshot SVGs, "
          f"{len(rows)} log entries shown)")
    print()
    print(f"  {'turn':>4}  {'sim-time':>8}  {'coverage':>8}  {'off-road':>8}  "
          f"{'state':>8}   turn input")
    print("  " + "-" * 74)
    for r in rows:
        t = r.get("turn", "?")
        st = r.get("sim_time", 0.0) or 0.0
        cov = r.get("coverage", 0.0) or 0.0
        off = r.get("off_road", 0.0) or 0.0
        state = r.get("state", "?")
        desc = r.get("desc") or "(initial state)"
        print(f"  {str(t):>4}  {st:>7.1f}s  {cov:>8.1%}  {off:>8.1%}  "
              f"{state:>8}   {desc}")
    redo_root = os.path.join(hdir, REDO_DIRNAME)
    branches = []
    if os.path.isdir(redo_root):
        for name in sorted(os.listdir(redo_root)):
            branch_dir = os.path.join(redo_root, name)
            if not os.path.isdir(branch_dir):
                continue
            branch_turns = list_turns(branch_dir)
            if not branch_turns:
                continue
            branch_rows = read_history_rows(branch_dir)
            final_turn = branch_turns[-1][0]
            final_row = next((r for r in reversed(branch_rows)
                              if r.get("turn") == final_turn), {})
            branches.append((name, branch_turns, final_row))
    if branches:
        print()
        print(f"archived attempts in {REDO_DIRNAME}/ (inspect their SVGs to compare):")
        for name, branch_turns, r in branches:
            turn_range = (str(branch_turns[0][0]) if len(branch_turns) == 1
                          else f"{branch_turns[0][0]}–{branch_turns[-1][0]}")
            state = r.get("state", "unknown")
            cov = float(r.get("coverage", 0.0) or 0.0)
            off = float(r.get("off_road", 0.0) or 0.0)
            print(f"  {name}/  turns {turn_range}: {state}, "
                  f"coverage {cov:.1%}, off-road {off:.1%}")
    return 0


def cmd_rules(args):
    d = {k: (list(v) if isinstance(v, list) else v) for k, v in DEFAULT_RULES.items()}
    print(json.dumps(d, indent=2))
    return 0


# --------------------------------------------------------------------------
# cli
# --------------------------------------------------------------------------
EXAMPLES = """examples:
  %(prog)s new game.svg --style medium --seed 42
  %(prog)s run game.svg --fx 60 --torque 1.2 --steps 60
  %(prog)s run game.svg --force 80 --angle -30 --frame global
  %(prog)s run game.svg --set linear-drag=0.6 --fx 40
  %(prog)s run game.svg --torque 0 --steps 30            # brake (coast)
  %(prog)s status game.svg --map
  %(prog)s history game.svg                          # per-turn log
  %(prog)s retry game.svg                            # rewind one turn
  %(prog)s retry game.svg --to 4 --keep              # back to turn 4, keep attempts
  %(prog)s rules > my_rules.json

history: every `run` also writes the full state to GAME.history/turn_XXXX.svg
(openable in a browser) and appends the input + outcome to history.jsonl,
so a player can inspect or rewind any turn. `new --no-history` opts out.

force: fx is forward, fy is to the right (local frame by default).
torque > 0 turns clockwise on screen; torque persists in the car state,
force is applied only during the steps of this call."""


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="roadrunner.py",
        description="ROADRUNNER — fill the black road with your trail. "
                    "The whole game lives in one SVG file.",
        epilog=EXAMPLES,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("new", help="create a new game file")
    p.add_argument("game", help="SVG file to create (e.g. game.svg)")
    p.add_argument("--seed", type=int, default=None, help="road seed (default: random)")
    p.add_argument("--style", choices=sorted(STYLES), default="medium")
    p.add_argument("--width", type=int, default=900)
    p.add_argument("--height", type=int, default=600)
    p.add_argument("--road-width", type=float, default=None,
                   help="road line width (default: 9 procedural, 3 picture)")
    p.add_argument("--image", default=None,
                   help="use this PNG as the road (dark pixels = the line)")
    p.add_argument("--reach-gap", type=float, default=40.0,
                   help="picture tracks: road parts separated by a bigger "
                        "off-road gap are excluded from the win (default 40px)")
    p.add_argument("--rules", default=None, help="JSON rules file (see 'rules')")
    p.add_argument("--no-history", action="store_true",
                   help="do not keep per-turn snapshots (GAME.history/)")
    p.add_argument("--force", action="store_true", help="overwrite existing file")
    p.set_defaults(func=cmd_new)

    p = sub.add_parser("run", aliases=["step", "go"],
                       help="apply controls, step the simulation, update the file")
    p.add_argument("game", help="game SVG file")
    p.add_argument("--fx", type=float, default=None, help="forward force (local frame)")
    p.add_argument("--fy", type=float, default=None, help="right force (local frame)")
    p.add_argument("--force", type=float, default=None,
                   help="force magnitude (with --angle from forward)")
    p.add_argument("--angle", type=float, default=0.0,
                   help="force angle in degrees from forward (positive = right)")
    p.add_argument("--frame", choices=["local", "global"], default="local")
    p.add_argument("--torque", type=float, default=None,
                   help="engine torque for this call (persists in state)")
    p.add_argument("--set", action="append", metavar="KEY=VALUE",
                   help="directly set a vehicle value if the rules allow "
                        "(x y theta vx vy omega weight linear-drag angular-drag torque)")
    p.add_argument("--steps", type=int, default=60,
                   help="simulation steps for this call (default 60 = 1s)")
    p.add_argument("--quiet", action="store_true", help="print nothing")
    p.set_defaults(func=cmd_run)

    p = sub.add_parser("status", aliases=["show"], help="print stats (and ASCII map)")
    p.add_argument("game", help="game SVG file")
    p.add_argument("--map", action="store_true", help="draw an ASCII map")
    p.set_defaults(func=cmd_status)

    p = sub.add_parser("retry", aliases=["undo", "rewind"],
                       help="rewind the game to an earlier turn and replay it "
                            "for a better result")
    p.add_argument("game", help="game SVG file")
    p.add_argument("--turns", type=int, default=None,
                   help="how many turns to go back (default 1)")
    p.add_argument("--to", type=int, default=None,
                   help="rewind to this exact turn number")
    p.add_argument("--keep", action="store_true",
                   help="archive the discarded timeline under "
                        "HISTORY/redo/ for later comparison")
    p.add_argument("--quiet", action="store_true", help="print nothing")
    p.set_defaults(func=cmd_retry)

    p = sub.add_parser("history", aliases=["log"],
                       help="show the per-turn input/outcome log")
    p.add_argument("game", help="game SVG file")
    p.add_argument("--json", action="store_true",
                   help="print the raw history.jsonl entries")
    p.add_argument("--tail", type=int, default=None,
                   help="show only the last N turns")
    p.set_defaults(func=cmd_history)

    p = sub.add_parser("rules", help="print the default rules JSON")
    p.set_defaults(func=cmd_rules)
    return ap


def main(argv=None):
    args = build_parser().parse_args(argv)
    sys.exit(args.func(args))


if __name__ == "__main__":
    main()
