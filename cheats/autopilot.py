#!/usr/bin/env python3
"""Test autopilot for roadrunner: drives a game file to completion via the CLI.

Also handy as a proof that a given track is solvable under a given ruleset.

  python3 autopilot.py game.svg [--calls 60] [--steps 60]

Two drivers:
  * procedural roads: tangent-pursuit + sideslip lead + cascaded spin control
  * picture tracks:  "chase the nearest uncovered road pixel" with a forward
    corridor (so parallel lines are not mixed up), U-turns at dead ends via
    direct targeting, and small off-road gap crossings when the next road
    cluster is separated from the car's line (within the track's reach gap).
"""
import math
import os
import subprocess
import sys

import roadrunner as rr


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# picture-track controller gains (tunable via env for sweeps)
KP = float(os.environ.get("AP_KP", 3.0))        # heading err -> spin target
CAP0 = float(os.environ.get("AP_CAP0", 0.45))   # spin cap at speed
CAP1 = float(os.environ.get("AP_CAP1", 0.50))   # extra cap at low speed
KIN = float(os.environ.get("AP_KIN", 8.0))      # spin tracking gain
KD2 = float(os.environ.get("AP_KD2", 16.0))     # damping vs (omega - need)
SLEW = float(os.environ.get("AP_SLEW", 2.7))    # max |d target| per second


def run_call(file, fx, torque, steps):
    return subprocess.run(
        [sys.executable, "roadrunner.py", "run", file,
         "--fx", f"{fx:.1f}", "--torque", f"{torque:.2f}",
         "--steps", str(steps), "--quiet"],
    ).returncode


# ---------------------------------------------------------------------------
# procedural roads
# ---------------------------------------------------------------------------
def drive_procedural(file, calls, steps):
    for i in range(calls):
        game = rr.read_game(file)
        if game.state in ("won", "lost"):
            print(f"game over after {i} calls: {game.state} {game.reason}")
            return 0
        v, road, rules = game.vehicle, game.road, game.rules
        best_i, best_d = 0, float("inf")
        for j, (sx, sy) in enumerate(road.samples):
            d = (sx - v.x) ** 2 + (sy - v.y) ** 2
            if d < best_d:
                best_i, best_d = j, d
        speed = math.hypot(v.vx, v.vy)

        def tang(idx):
            p, q = road.samples[idx], road.samples[min(idx + 1, len(road.samples) - 1)]
            return math.atan2(q[1] - p[1], q[0] - p[0])
        t0 = tang(best_i)
        t1 = tang(min(best_i + 12, len(road.samples) - 1))
        kappa = wrap(t1 - t0) / 24.0
        nx, ny = road.samples[best_i]
        dx, dy = road.samples[min(best_i + 1, len(road.samples) - 1)]
        tl = math.hypot(dx, dy) or 1.0
        lat = (v.x - nx) * (-dy / tl) + (v.y - ny) * (dx / tl)
        omega_need = kappa * speed
        grip = rr.GRIP_LAT * v.linear_drag / v.weight
        desired = t0 + omega_need / grip - 0.8 * lat / 24.0
        err = wrap(desired - v.theta)
        omega_target = omega_need + rr.clamp(3.0 * err, -0.45, 0.45)
        torque = 8.0 * (omega_target - v.omega) + v.angular_drag * omega_need
        torque = max(-rules.max_torque, min(rules.max_torque, torque))
        vmax = max(18.0, min(70.0, 30.0 + 40.0 * (1 - min(abs(err), 1.0))))
        vmax = min(vmax, max(16.0, 0.55 / abs(kappa)) if abs(kappa) > 1e-4 else vmax)
        fx = max(0.0, min(rules.max_force, (vmax - speed) * 1.5))
        run_call(file, fx, torque, steps)
        new = rr.read_game(file)
        cov, off = rr.compute_metrics(new.path, new.road, new.rules)
        print(f"call {i + 1:3d}  coverage {cov:6.1%}  off-road {off:6.2%}  "
              f"pos ({new.vehicle.x:6.1f},{new.vehicle.y:6.1f})  "
              f"speed {math.hypot(new.vehicle.vx, new.vehicle.vy):5.1f}", flush=True)
        if new.state in ("won", "lost"):
            print(f"game over after {i + 1} calls: {new.state} {new.reason}")
            return 0
    print("autopilot stopped (call limit)")
    return 1


# ---------------------------------------------------------------------------
# picture tracks
# ---------------------------------------------------------------------------
def _pca_tangent(px, py, pts):
    sxx = syy = sxy = 0.0
    for (ax, ay) in pts:
        dx, dy = ax - px, ay - py
        sxx += dx * dx
        syy += dy * dy
        sxy += dx * dy
    if sxx + syy < 1e-9:
        return None
    return math.atan2(2 * sxy, sxx - syy) / 2


def drive_tour(file, calls, steps):
    """Follow a precomputed tour (make_tour.make_tour) with pure pursuit."""
    import make_tour
    g0 = rr.read_game(file)
    road, rules = g0.road, g0.rules
    tour, tinfo = make_tour.make_tour(road, reach_gap=road.reach_gap)
    n = len(tour)
    prefix = [0.0] * n
    for i in range(1, n):
        prefix[i] = prefix[i - 1] + math.hypot(tour[i][0] - tour[i - 1][0],
                                               tour[i][1] - tour[i - 1][1])
    total = prefix[-1]

    v0 = g0.vehicle
    tpos = min(range(n), key=lambda i: (tour[i][0] - v0.x) ** 2
               + (tour[i][1] - v0.y) ** 2)
    t0_f = t_a_f = None
    last_target = None
    for i in range(calls):
        game = rr.read_game(file)
        if game.state in ("won", "lost"):
            cov, off = rr.compute_metrics(game.path, road, rules)
            print(f"game over after {i} calls: {game.state} {game.reason}  "
                  f"(coverage {cov:.1%}, off-road {off:.2%})")
            return 0
        v = game.vehicle
        speed = math.hypot(v.vx, v.vy)
        lo = max(0, tpos - 5)
        hi = min(n - 1, tpos + 60)
        tpos = min(range(lo, hi + 1),
                   key=lambda j: (tour[j][0] - v.x) ** 2 + (tour[j][1] - v.y) ** 2)

        def chord(j, d):
            j2 = min(n - 1, j + d)
            j1 = max(0, j - d)
            if j2 == j1:
                return None
            return math.atan2(tour[j2][1] - tour[j1][1],
                              tour[j2][0] - tour[j1][0])
        t0 = chord(tpos, 6)
        t_a = chord(tpos + 10, 6)
        if t0 is None:
            t0 = t_a = v.theta
        if t_a is None:
            t_a = t0
        if t0_f is None:
            t0_f, t_a_f = t0, t_a
        else:
            t0_f += 0.5 * wrap(t0 - t0_f)
            t_a_f += 0.5 * wrap(t_a - t_a_f)
        t0, t_a = t0_f, t_a_f
        wp = tour[min(n - 1, tpos + 12)]
        (wx, wy) = wp
        a_wp = math.atan2(wy - v.y, wx - v.x)
        d_wp = math.hypot(wx - v.x, wy - v.y)
        rx, ry = -math.sin(t0), math.cos(t0)
        lat = (v.x - tour[tpos][0]) * rx + (v.y - tour[tpos][1]) * ry
        kappa = wrap(t_a - t0) / 12.0
        omega_need = kappa * speed
        grip = rr.GRIP_LAT * v.linear_drag / v.weight
        desired = t_a + omega_need / grip - 0.8 * lat / 12.0
        if d_wp < 24:
            desired += 0.3 * wrap(a_wp - t_a)
        err = wrap(desired - v.theta)
        if speed < 6 and abs(err) > math.radians(45) and d_wp > 8:
            # misaligned: no thrust, proportional spot-turn (decelerates as
            # the heading closes in, so it does not overshoot)
            raw_target = rr.clamp(0.34 * err, -1.2, 1.2)
            fx = 0.0
        else:
            cap = CAP0 + CAP1 * (1 - min(speed / 25.0, 1.0))
            raw_target = omega_need + rr.clamp(KP * err, -cap, cap)
        if last_target is not None:
            max_d = SLEW * steps * rules.dt
            raw_target = rr.clamp(raw_target, last_target - max_d,
                                  last_target + max_d)
        last_target = raw_target
        omega_target = raw_target
        if os.environ.get("AP_DEBUG") and i < 14:
            print(f"    dbg tpos {tpos}/{n}  head {math.degrees(v.theta) % 360:7.1f} "
                  f"ta {math.degrees(t_a) % 360:7.1f}  lat {lat:+5.2f} "
                  f"kap {kappa:+.3f}  dwp {d_wp:5.1f}  des "
                  f"{math.degrees(desired) % 360:7.1f}  tgt {round(omega_target, 2)}",
                  file=sys.stderr, flush=True)
        torque = KIN * (omega_target - v.omega) + v.angular_drag * omega_need
        torque = max(-rules.max_torque, min(rules.max_torque, torque))
        vmax = max(10.0, min(55.0, 26.0 + 34.0 * (1 - min(abs(err), 1.0))))
        if abs(kappa) > 1e-4:
            vmax = min(vmax, max(6.0, 0.55 / abs(kappa)))
        if not (speed < 6 and abs(err) > math.radians(45) and d_wp > 8):
            fx = max(0.0, min(rules.max_force, (vmax - speed) * 1.5))
        run_call(file, fx, torque, steps)
        new = rr.read_game(file)
        cov, off = rr.compute_metrics(new.path, new.road, new.rules)
        print(f"call {i + 1:3d}  coverage {cov:6.1%}  off {off:6.2%}  "
              f"pos ({new.vehicle.x:6.1f},{new.vehicle.y:6.1f})  "
              f"t {tpos / n:5.1%}  spd {speed:5.1f}  err {math.degrees(err):+6.1f}",
              flush=True)
        if new.state in ("won", "lost"):
            print(f"game over after {i + 1} calls: {new.state} {new.reason}  "
                  f"(coverage {cov:.1%}, off-road {off:.2%})")
            return 0
        if tpos >= n - 3 and d_wp < 20:
            print("tour complete")
            break
    print("autopilot stopped (call limit)")
    return 1



def drive_image(file, calls, steps):
    g0 = rr.read_game(file)
    road, rules = g0.road, g0.rules
    W, H, samples = road.W, road.H, road.samples
    n = len(samples)
    R = int(math.ceil(rules.cover_radius_for(road.road_width)))
    R2 = R * R

    # spatial grid over the road samples
    CELL = 8
    gw, gh = (W + CELL - 1) // CELL, (H + CELL - 1) // CELL
    cells = [[] for _ in range(gw * gh)]
    for i, (sx, sy) in enumerate(samples):
        cells[(sy // CELL) * gw + (sx // CELL)].append(i)

    def near(x, y, rmax):
        """sample indices within rmax of (x,y), nearest first"""
        c0x = max(0, int(x - rmax) // CELL)
        c1x = min(gw - 1, int(x + rmax) // CELL)
        c0y = max(0, int(y - rmax) // CELL)
        c1y = min(gh - 1, int(y + rmax) // CELL)
        out = []
        for cy in range(c0y, c1y + 1):
            row = cy * gw
            for cx in range(c0x, c1x + 1):
                out.extend(cells[row + cx])
        out.sort(key=lambda i: (samples[i][0] - x) ** 2 + (samples[i][1] - y) ** 2)
        return out

    # component of every sample (for gap checks on crossing)
    full = bytearray(W * H)
    for i in range(W * H):
        if road.grid[i] or road.excluded[i]:
            full[i] = 1
    comp, _ = rr.road_components(full, W, H)
    s_comp = [comp[sx + sy * W] for (sx, sy) in samples]
    gaps = road.gaps
    reach_gap = road.reach_gap

    uncovered = bytearray(b"\x01") * n
    prev_len = 0
    WINDOW = 36.0
    last_target = None
    t0_f = t_a_f = None
    for i in range(calls):
        game = rr.read_game(file)
        if game.state in ("won", "lost"):
            cov, off = rr.compute_metrics(game.path, road, rules)
            print(f"game over after {i} calls: {game.state} {game.reason}  "
                  f"(coverage {cov:.1%}, off-road {off:.2%})")
            return 0
        v = game.vehicle
        speed = math.hypot(v.vx, v.vy)

        # mark samples covered by the trail added since the last call
        for (x, y) in game.path[prev_len:]:
            xi, yi = int(round(x)), int(round(y))
            for j in near(xi, yi, R):
                sx, sy = samples[j]
                if (sx - xi) ** 2 + (sy - yi) ** 2 <= R2:
                    uncovered[j] = 0
        prev_len = len(game.path)
        rem = sum(uncovered)
        if rem == 0:
            print(f"all {n} road samples covered after {i} calls")
            break

        # the line we are on right now (all road pixels nearby)
        win = near(v.x, v.y, 8)
        p0 = samples[win[0]] if win else None
        t0 = None
        if p0 is not None:
            t0 = _pca_tangent(p0[0], p0[1], [samples[j] for j in win])
            if t0 is None:
                t0 = v.theta
            elif math.cos(t0 - v.theta) < 0:
                t0 += math.pi
        ref = t0 if t0 is not None else v.theta

        # car standing between two lines (gap / finger slot)?
        between_idx = None
        if p0 is not None and len(win) >= 12:
            ct, stn = math.cos(t0), math.sin(t0)
            perp = [((samples[j][0] - p0[0]) * -stn
                     + (samples[j][1] - p0[1]) * ct) for j in win]
            if max(perp) - min(perp) > 5.0:
                between_idx = win[0]     # nearest line pixel, any coverage

        # is our local line still unfinished within 60px ahead of us?
        # (no local line at all -> fall straight through to cross mode,
        # which steers directly back at the nearest uncovered road point)
        line_ahead = p0 is not None and between_idx is None
        if p0 is not None and between_idx is None:
            ct, stn = math.cos(t0), math.sin(t0)
            for j in near(p0[0], p0[1], 60):
                if not uncovered[j]:
                    continue
                sx, sy = samples[j]
                fd = (sx - p0[0]) * ct + (sy - p0[1]) * stn
                if fd < 2 or fd > 60:
                    continue
                if abs((sx - p0[0]) * -stn + (sy - p0[1]) * ct) < 8:
                    line_ahead = True
                    break

        mode = None
        omega_need = 0.0
        kappa = 0.0
        t_a = lat = d_wp = a_wp = None
        if between_idx is not None:
            # between two lines: go straight for the nearest line pixel
            wx, wy = samples[between_idx]
            desired = math.atan2(wy - v.y, wx - v.x)
            a_wp = desired
            d_wp = math.hypot(wx - v.x, wy - v.y)
            mode = "between"
        elif line_ahead:
            # follow: nearest uncovered, preferring points along the local
            # line (score penalizes lateral direction)
            p_near = None
            best_score = None
            for j in near(v.x, v.y, WINDOW):
                if not uncovered[j]:
                    continue
                wx, wy = samples[j]
                d0 = math.hypot(wx - v.x, wy - v.y)
                if d0 < 4:
                    continue
                a_wp = math.atan2(wy - v.y, wx - v.x)
                if speed > 5 and abs(wrap(a_wp - v.theta)) > math.radians(100):
                    continue
                score = d0 + 30.0 * abs(wrap(a_wp - ref))
                if best_score is None or score < best_score:
                    best_score = score
                    p_near = j
            if p_near is not None:
                wx, wy = samples[p_near]
                d_wp = math.hypot(wx - v.x, wy - v.y)
                a_wp = math.atan2(wy - v.y, wx - v.x)
                if p0 is None:
                    p0 = (v.x, v.y)
                    t0 = ref
                    t_a = t0
                    lat = 0.0
                else:
                    # tangent ~14px ahead: PCA of the forward half of 20px
                    fwd = []
                    for j in near(p0[0], p0[1], 20):
                        sx, sy = samples[j]
                        if ((sx - p0[0]) * math.cos(t0)
                                + (sy - p0[1]) * math.sin(t0)) > 3:
                            fwd.append(j)
                    t_a_raw = t0
                    if len(fwd) >= 5:
                        c0 = samples[fwd[0]]
                        t2 = _pca_tangent(c0[0], c0[1],
                                          [samples[j] for j in fwd])
                        if t2 is not None:
                            if math.cos(t2 - t0) < 0:
                                t2 += math.pi
                            t_a_raw = t2
                    # low-pass the noisy tangents between calls
                    if t0_f is None:
                        t0_f, t_a_f = t0, t_a_raw
                    else:
                        t0_f += 0.5 * wrap(t0 - t0_f)
                        t_a_f += 0.5 * wrap(t_a_raw - t_a_f)
                    t_a = t_a_f
                    kappa = wrap(t_a - t0_f) / 14.0
                    rx, ry = -math.sin(t0_f), math.cos(t0_f)
                    lat = (v.x - p0[0]) * rx + (v.y - p0[1]) * ry
                omega_need = kappa * speed
                grip = rr.GRIP_LAT * v.linear_drag / v.weight
                # pull only when the waypoint is off our local line
                # (branch / corner / gap); on-line it only adds noise
                rnx, rny = -math.sin(t0_f), math.cos(t0_f)
                lat_wp = (wx - p0[0]) * rnx + (wy - p0[1]) * rny
                on_line = abs(lat_wp) < 6.0 and d_wp < 22
                pull = 0.0 if on_line else 0.35
                if win and s_comp[win[0]] != s_comp[p_near]:
                    pull = max(pull, 0.55)   # different piece of road
                desired = t_a + omega_need / grip - 0.8 * lat / 14.0
                if d_wp < 26:
                    desired += pull * wrap(a_wp - t_a)
                mode = "follow"
        if mode is None:
            # local line is done here: dead end (U-turn), sharp corner, or a
            # separated cluster (small off-road crossing) -> aim at the
            # nearest uncovered point outright
            cand = sorted((j for j in range(n) if uncovered[j]),
                          key=lambda j: (samples[j][0] - v.x) ** 2
                          + (samples[j][1] - v.y) ** 2)
            Q = next((j for j in cand
                      if math.hypot(samples[j][0] - v.x,
                                    samples[j][1] - v.y) >= 4),
                     cand[0] if cand else None)
            if Q is None:
                break
            g = gaps.get(s_comp[Q], 250)
            if g > reach_gap:
                cov, off = rr.compute_metrics(game.path, road, rules)
                print(f"call {i + 1:3d}  stopped: remaining cluster is "
                      f"{g}px off-road from the car's component "
                      f"(> reach gap {reach_gap:.0f}px); "
                      f"coverage {cov:.1%} off {off:.2%}")
                break
            wx, wy = samples[Q]
            desired = math.atan2(wy - v.y, wx - v.x)
            a_wp = desired
            d_wp = math.hypot(wx - v.x, wy - v.y)
            mode = "uturn" if d_wp < 16 else "cross"

        err = wrap(desired - v.theta)
        if speed < 6 and abs(err) > math.radians(60) and d_wp > 8:
            # tangled: no thrust, spot-turn toward the waypoint
            torque = rules.max_torque * (1 if err > 0 else -1)
            run_call(file, 0.0, torque, steps)
            new = rr.read_game(file)
            cov, off = rr.compute_metrics(new.path, new.road, new.rules)
            print(f"call {i + 1:3d}  spin    coverage {cov:6.1%}  off {off:6.2%}  "
                  f"rem {rem:6d}  pos ({new.vehicle.x:6.1f},{new.vehicle.y:6.1f})  "
                  f"spd {speed:5.1f}  err {math.degrees(err):+6.1f}", flush=True)
            if new.state in ("won", "lost"):
                print(f"game over after {i + 1} calls: {new.state} {new.reason}  "
                      f"(coverage {cov:.1%}, off-road {off:.2%})")
                return 0
            continue
        cap = CAP0 + CAP1 * (1 - min(speed / 25.0, 1.0))
        raw_target = omega_need + rr.clamp(KP * err, -cap, cap)
        # slew-limit the spin target between calls (control delay is 1 call)
        if last_target is not None:
            max_d = SLEW * steps * rules.dt
            raw_target = rr.clamp(raw_target, last_target - max_d,
                                  last_target + max_d)
        last_target = raw_target
        omega_target = raw_target
        if os.environ.get("AP_DEBUG") and i < 12:
            def _d(a):
                return f"{math.degrees(a) % 360:7.1f}" if a is not None else "    nan"
            print(f"    dbg {mode}: head {_d(v.theta)} t0 {_d(t0)} "
                  f"ta {_d(t_a)}  lat {lat if lat is None else round(lat, 2)} "
                  f"need {round(omega_need, 3)}  dwp {d_wp if d_wp is None else round(d_wp, 1)} "
                  f"awp {_d(a_wp)}  des {_d(desired)}  tgt {round(omega_target, 2)} "
                  f"p0 {p0}",
                  file=sys.stderr, flush=True)
        torque = (KIN * (omega_target - v.omega)
                  + v.angular_drag * omega_need
                  - KD2 * (v.omega - omega_need))
        torque = max(-rules.max_torque, min(rules.max_torque, torque))
        vmax = max(10.0, min(55.0, 26.0 + 34.0 * (1 - min(abs(err), 1.0))))
        if abs(kappa) > 1e-4:
            vmax = min(vmax, max(8.0, 0.55 / abs(kappa)))
        if mode == "follow" and p0 is not None and t0_f is not None:
            # how much does the line rotate over the next ~30px? (walk the
            # real line: each step predicts from the last found direction)
            tot = 0.0
            cx, cy = p0[0], p0[1]
            sx0, sy0 = cx, cy
            cur = t0_f
            for _ in range(12):
                tx = cx + 3.0 * math.cos(cur)
                ty = cy + 3.0 * math.sin(cur)
                nn = near(tx, ty, 5.0)
                if not nn:
                    break
                (nx_, ny_) = samples[nn[0]]
                if math.hypot(nx_ - cx, ny_ - cy) < 0.5:
                    break
                ang = math.atan2(ny_ - cy, nx_ - cx)
                d_a = wrap(ang - cur)
                if abs(d_a) > 1.0:      # the line doubled back: full U
                    tot = max(tot, 2.2)
                    break
                tot += abs(d_a)
                cur = ang
                cx, cy = nx_, ny_
                if tot > 2.6 or (cx - sx0) ** 2 + (cy - sy0) ** 2 > 961:
                    break
            if tot > 0.3:
                vmax = min(vmax, max(5.0, 30.0 * 0.55 / tot))
        if mode == "cross":
            vmax = min(vmax, 14.0)
        if mode == "between":
            vmax = min(vmax, 10.0)
        fx = max(0.0, min(rules.max_force, (vmax - speed) * 1.5))
        run_call(file, fx, torque, steps)
        new = rr.read_game(file)
        cov, off = rr.compute_metrics(new.path, new.road, new.rules)
        print(f"call {i + 1:3d}  {mode:6s}  coverage {cov:6.1%}  off {off:6.2%}  "
              f"rem {rem:6d}  pos ({new.vehicle.x:6.1f},{new.vehicle.y:6.1f})  "
              f"spd {speed:5.1f}  err {math.degrees(err):+6.1f}  "
              f"torq {torque:+5.2f}", flush=True)
        if new.state in ("won", "lost"):
            print(f"game over after {i + 1} calls: {new.state} {new.reason}  "
                  f"(coverage {cov:.1%}, off-road {off:.2%})")
            return 0
    print("autopilot stopped (call limit)")
    return 1


def main():
    file = sys.argv[1]
    calls = 300
    steps = 20
    if "--calls" in sys.argv:
        calls = int(sys.argv[sys.argv.index("--calls") + 1])
    if "--steps" in sys.argv:
        steps = int(sys.argv[sys.argv.index("--steps") + 1])

    g0 = rr.read_game(file)
    if g0.road.source == "image":
        return drive_tour(file, calls, steps)
    return drive_procedural(file, calls, steps)


if __name__ == "__main__":
    sys.exit(main())
