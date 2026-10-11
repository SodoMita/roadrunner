"""Precompute a driving tour for a picture track.

Greedy pixel walk over the road pixels:

  * step to the nearest not-yet-walked road pixel within 3px, preferring the
    one that continues the current direction;
  * a road pixel counts as *covered* once the tour comes within 3px of it, so
    the second pixel row of a 2px line is skipped (the car's R=4 trail covers
    it anyway);
  * at a dead end (finger tip, crotch V) the nearest *uncovered* road pixel is
    found and a straight (possibly off-road) connector is driven to it;
  * fragments separated by a gap <= reach_gap are visited the same way.

The result is an ordered list of integer (x, y) points (1px steps plus
connectors) that the autopilot follows with plain pure pursuit.
"""
import math


def _grid(pts, W, H, cell=4):
    gw = (W + cell - 1) // cell
    gh = (H + cell - 1) // cell
    cells = [[] for _ in range(gw * gh)]
    for (x, y) in pts:
        cells[(y // cell) * gw + (x // cell)].append((x, y))
    return cells, gw, gh, cell


def make_tour(road, reach_gap=40.0):
    """road: roadrunner.Road (source == 'image').
    Returns (tour, info): tour = list of (int, int) points covering every
    counted road pixel (each within 3px of the tour)."""
    import roadrunner as rr
    W, H = road.W, road.H

    road_px = set()
    for i in range(W * H):
        if road.grid[i]:
            road_px.add((i % W, i // W))

    full = bytearray(W * H)
    for i in range(W * H):
        if road.grid[i] or road.excluded[i]:
            full[i] = 1
    comp, pbc = rr.road_components(full, W, H)
    sp = (int(road.start[0]), int(road.start[1]))
    main_id = comp[sp[1] * W + sp[0]]
    # fragments within the reach gap merge in; beyond it stay ignored
    seed = set()
    for p in pbc[main_id - 1]:
        seed.add((p % W, p // W))
    if road.gaps:
        for cid, g in sorted(road.gaps.items()):
            if cid != main_id and 1 < g <= reach_gap:
                for p in pbc[cid - 1]:
                    seed.add((p % W, p // W))
    # (fragment pixels are reached via the uncovered-pixel jump)

    cells, gw, gh, cell = _grid(seed, W, H)

    def near(x, y, r):
        rr_ = r * r
        c0 = max(0, (x - r) // cell)
        c1 = min(gw - 1, (x + r) // cell)
        y0 = max(0, (y - r) // cell)
        y1 = min(gh - 1, (y + r) // cell)
        out = []
        for cy in range(y0, y1 + 1):
            row = cy * gw
            for cx in range(c0, c1 + 1):
                for (px, py) in cells[row + cx]:
                    if (px - x) ** 2 + (py - y) ** 2 <= rr_:
                        out.append((px, py))
        return out

    start = min(seed, key=lambda p: (p[0] - sp[0]) ** 2 + (p[1] - sp[1]) ** 2)

    walked = set()
    covered = set()          # road pixels within 3px of the tour
    tour = [start]
    walked.add(start)

    dx = dy = None
    n_seed = len(seed)
    n_cov = 0

    def mark_covered(px, py):
        nonlocal n_cov
        for (ax, ay) in near(px, py, 3):
            if (ax, ay) in seed and (ax, ay) not in covered:
                covered.add((ax, ay))
                n_cov += 1

    for (x0, y0) in near(*start, 3):
        if (x0, y0) in seed:
            covered.add((x0, y0))
            n_cov += 1
    for _ in range(n_seed * 6 + 1000):
        x, y = tour[-1]
        # candidates: seed pixels within 3px, not walked yet
        cands = [q for q in near(x, y, 3) if q not in walked]
        if not cands:
            cands = [q for q in near(x, y, 3) if q not in covered]
        if cands:
            if dx is not None:
                q = max(cands,
                        key=lambda qq: (qq[0] - x) * dx + (qq[1] - y) * dy)
            else:
                q = min(cands, key=lambda qq: (qq[0] - x) ** 2 + (qq[1] - y) ** 2)
            ndx, ndy = q[0] - x, q[1] - y
            l = math.hypot(ndx, ndy)
            if l > 0.1:
                dx, dy = ndx / l, ndy / l
            tour.append(q)
            walked.add(q)
            mark_covered(*q)
        else:
            # dead end: jump to the nearest uncovered seed pixel
            best = None
            for rmax in (4, 8, 16, 32, 64, 128, 4096):
                best = None
                for q in seed:
                    if q in covered or q in walked:
                        continue
                    d = (q[0] - x) ** 2 + (q[1] - y) ** 2
                    if d <= rmax * rmax:
                        if best is None or d < best[1]:
                            best = (q, d)
                if best:
                    break
            if not best:
                break
            q = best[0]
            # straight connector (1px Bresenham)
            bx, by = x, y
            steps = max(abs(q[0] - bx), abs(q[1] - by)) or 1
            for i in range(1, steps + 1):
                nxp = round(bx + (q[0] - bx) * i / steps)
                nyp = round(by + (q[1] - by) * i / steps)
                if (nxp, nyp) != tour[-1]:
                    tour.append((nxp, nyp))
                mark_covered(nxp, nyp)
            dx, dy = (q[0] - x) / (steps or 1), (q[1] - y) / (steps or 1)
        if n_cov == n_seed:
            break

    # drop consecutive duplicates
    out = [tour[0]]
    for p in tour[1:]:
        if p != out[-1]:
            out.append(p)
    uncovered_n = len(seed) - len(covered)
    return out, {"steps": len(out), "px_walked": len(tour),
                 "uncovered_px": uncovered_n, "seed": len(seed)}


if __name__ == "__main__":
    import sys
    import roadrunner as rr
    g = rr.read_game(sys.argv[1] if len(sys.argv) > 1 else "body.svg")
    import time
    t0 = time.time()
    tour, info = make_tour(g.road, reach_gap=g.road.reach_gap)
    print(f"tour: {info}  built in {time.time() - t0:.1f}s")
