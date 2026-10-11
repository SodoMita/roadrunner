#!/usr/bin/env python3
"""Debug helper: drive a game file with the autopilot controller and log per-call state."""
import math
import subprocess
import sys

sys.path.insert(0, ".")
import roadrunner as rr


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def main():
    file = sys.argv[1]
    from_call = int(sys.argv[2]) if len(sys.argv) > 2 else 0
    to_call = int(sys.argv[3]) if len(sys.argv) > 3 else 60

    def tang(road, idx):
        s = road.samples
        p, q = s[idx], s[min(idx + 1, len(s) - 1)]
        return math.atan2(q[1] - p[1], q[0] - p[0])

    for call in range(to_call):
        g = rr.read_game(file)
        if g.state != "running":
            print("game over:", g.state, g.reason)
            break
        v, road, rules = g.vehicle, g.road, g.rules
        samples = road.samples
        bi, bd = 0, 1e18
        for i, (sx, sy) in enumerate(samples):
            d = (sx - v.x) ** 2 + (sy - v.y) ** 2
            if d < bd:
                bd, bi = d, i
        speed = math.hypot(v.vx, v.vy)
        t0 = tang(road, bi)
        t1 = tang(road, min(bi + 12, len(samples) - 1))
        kappa = wrap(t1 - t0) / 24.0
        err = wrap(t1 - v.theta)
        nx, ny = samples[bi]
        px, py = samples[min(bi + 1, len(samples) - 1)]
        dx, dy = px - nx, py - ny
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
        rc = subprocess.run([sys.executable, "roadrunner.py", "run", file,
                             "--fx", f"{fx:.1f}", "--torque", f"{torque:.2f}",
                             "--steps", "20", "--quiet"]).returncode
        if call >= from_call:
            print(f"call {call + 1:3d} d={math.sqrt(bd):5.1f} lat={lat:+6.1f} "
                  f"t1={math.degrees(t1) % 360:6.1f} head={math.degrees(v.theta) % 360:6.1f} "
                  f"err={math.degrees(err):+6.1f} k={kappa:+.3f} torq={torque:+5.2f} "
                  f"om={v.omega:+5.2f} need={omega_need:+5.2f} spd={speed:5.1f} "
                  f"pos=({v.x:5.0f},{v.y:5.0f})")
        if rc in (10, 30):
            print(f"  -> game over after call {call + 1} (rc={rc})")
            break


if __name__ == "__main__":
    main()
