# Road Runner — a terminal game with SVG game states

The complete **current** game state — road, vehicle (position, heading,
velocity, torque, weight, linear drag, angular drag), trail, and rules — lives
in one `.svg` file. The road is an embedded raster PNG (black line on white);
the vehicle and trail are SVG elements, so you can open the game in a browser.

By default, a sibling `GAME.history/` folder also saves a complete SVG for
every turn (`turn_0000.svg`, `turn_0001.svg`, …) and a JSONL record of the
inputs and results. This makes the game easy to inspect and replay turn-by-turn.
Use `new --no-history` to opt out. No database or external service is needed.

```
$ python3 roadrunner.py new game.svg --style medium
created game.svg
  road:    style=medium  seed=4821  1015px long  width=9px  canvas 900x600
  ...
  start:   roadrunner.py run game.svg --fx 60 --torque 1.0 --steps 60

$ python3 roadrunner.py run game.svg --fx 60 --torque 1.0 --steps 60
  input:   fx +60.0 fy +0.0 (local)  torque +1.00  steps 60 (1.00s)
  vehicle: pos (55.1, 434.0)  heading 335.2°  speed 6.4 px/s  ...
  road:    covered 2.1% (need 98%)   off-road 0.0% (limit 10%)   time 1.0s / 300s
  state:   RUNNING
```

## How to play

The goal: drive the vehicle along the road and **cover at least 98% of it
with your trail, while keeping at most 10% of the trail off-road**.

Every turn you call `run` with your inputs. The engine then simulates physics
step by step (dt = 1/60 s) for the requested number of steps.

```
python3 roadrunner.py new FILE [--style easy|medium|hard] [--seed N]
                [--width N] [--height N] [--road-width N] [--rules FILE]
                [--force] [--no-history]
python3 roadrunner.py run FILE
                [--fx F] [--fy F] [--force F] [--angle A] [--frame local|global]
                [--torque T] [--set key=value ...] [--steps N] [--quiet]
python3 roadrunner.py status FILE [--map]     # show state (== show)
python3 roadrunner.py history FILE [--json] [--tail N]
python3 roadrunner.py retry FILE [--turns K | --to N] [--keep] [--quiet]
python3 roadrunner.py rules                  # print default rules JSON
```

Inputs:

| flag | meaning |
|---|---|
| `--fx`, `--fy` | constant force applied along the steps, default **vehicle-local** frame (forward/right). |
| `--force F --angle A` | same, magnitude + direction (0° = forward). |
| `--frame world` | interpret the force in world coordinates instead. |
| `--torque T` | steering torque in rad/s² (left +, right −). **Persists** on the car. |
| `--set key=value` | directly overwrite a vehicle property — **only the keys the rules allow** (`--set x=…` is rejected by default). |
| `--steps N` | how many 1/60 s steps to simulate (clamped by the rules). |

`status` prints the full state; `status --map` adds an ASCII map of the road
with the trail and the car so you can play blind. `rules` prints default rule
values; `status` and the game SVG show the active rules for a game.

## Turn history and retry (LLM-friendly workflow)

Every `run` call is a turn. The sidecar folder is named from the game file:
`game.svg` → `game.history/`.

* `turn_0000.svg` is the initial state. `turn_0001.svg` is the state after the
  first `run`, and each later SVG is the complete state after that turn.
* `history.jsonl` has one JSON object per state/turn. Each `run` record includes
  the requested CLI inputs, the applied world-frame force, clamped torque and
  step count, accepted `--set` values, final vehicle state, simulation time,
  coverage, off-road ratio, and game state.
* You can open any snapshot SVG or parse the JSONL directly. For a compact
  human-readable log, use `python3 roadrunner.py history game.svg`; use
  `--json` for machine-readable rows and `--tail N` to limit the output.
* `retry` restores a saved state and truncates the active timeline after it.
  The next `run` is a new attempt from that exact state, with new controls.
  `--keep` archives the abandoned timeline in a unique folder under
  `game.history/redo/`, so repeated attempts are not overwritten and can be
  compared by inspecting their SVGs and `history.jsonl`.

Example: after a bad turn, retry from the state before it, then try different
controls. `--to 4` restores turn 4 (so the next `run` is turn 5); by default,
`retry` goes back one turn.

```sh
python3 roadrunner.py history game.svg
python3 roadrunner.py retry game.svg --keep
python3 roadrunner.py run game.svg --fx 45 --torque 0.3 --steps 30
python3 roadrunner.py history game.svg       # compare the saved attempt
```

The CLI remains model-agnostic: an LLM, a person, or a future ML controller
can choose the `run` inputs. No built-in AI/model player is added here.

## Winning and losing

* **WIN** — coverage ≥ 98% and off-road trail ≤ 10%. (exit code `10`)
* **LOST** — off-road trail exceeds 10%, or the time limit runs out. (exit code `30`)

Coverage uses a 5 px radius around the trail; off-road counts trail points
farther than 2 px from the road centerline (road half-width + tolerance).

## The restriction conditions

Rules are written into the SVG when the game is created and cannot be changed
mid-game. Defaults (used when `--rules` is not given):

```json
{ "settable": ["weight", "linear-drag", "angular-drag"],
  "max_force": 120, "max_torque": 6.0, "max_steps_per_call": 600,
  "cover_target": 0.98, "off_ratio_max": 0.10, "time_limit_s": 300 }
```

Example custom rule files:

* `rules_chill.json` — also allows `--set theta`, bigger canvas budget,
  900 steps/call, 90% / 25% tolerances (beginner friendly).
* `rules_strict.json` — nothing directly settable, 70 force, 240 steps/call,
  99.5% / 4% tolerances (hardcore).

Create a custom game: `python3 roadrunner.py new game.svg --rules my_rules.json`.

## The file format

A game file is a normal, valid SVG. Game data lives in custom elements under
the `http://arena.ai/roadrunner/game/v1` namespace:

```xml
<g:meta    version="1" game="roadrunner" history="on"/>
<g:road    seed="7" style="easy" width="900" height="600" road-width="9.0" length="808"/>
<g:rules   settable="weight,linear-drag,angular-drag" max-force="120.0" max-torque="6.0"
           max-steps-per-call="600" min-road-coverage="0.980" max-path-off-road="0.100"
           time-limit-s="300.0" off-road-tolerance="2.0" dt="0.0166667"/>
<g:vehicle x="45.00" y="223.89" theta="-0.1096" vx="0.00" vy="0.00" omega="0.000"
           torque="0.000" weight="6.00" linear-drag="1.500" angular-drag="0.500" sim-time="0.00"/>
<g:path    points-count="1" points="45.0,223.9"/>
<g:status  state="running" reason="" coverage="0.0099" off-road="0.0000"/>
<image href="data:image/png;base64,...."/>        <!-- the road raster -->
```

* `<g:road>` stores the deterministic road spec (seed, style, canvas, road
  width); the embedded PNG raster is regenerated on read if missing.
* `<g:vehicle>` is the complete vehicle state: position, rotation, velocity,
  torque, weight, linear drag, angular drag, sim time.
* `<g:path>` is the trail as a space-separated point list — a single dot
  (one point) at the start, growing with every `run`.
* A visible vehicle `<rect>` (plus a small heading tick) and a visible trail
  `<polyline>` are refreshed on every save, so browsers show the live picture
  even though the *data* lives in the `g:` elements.

## Physics

Per 1/60 s step, in the vehicle frame:

```
lateral force  F_lat = -GRIP · v_lat · linear_drag · dt      (grip model, GRIP = 20)
forward  force  F_fwd = -v_fwd · linear_drag · dt
dx += v_fwd·cos θ …                                            (rotate to world)
v_fwd = (v_fwd + F_fwd) / weight
v_lat = (v_lat + F_lat) / weight
dω/dt  = torque / (weight·0.5) - angular_drag·ω
θ += ω·dt
```

Tuning `weight`, `linear-drag` and `angular-drag` (the settable keys) changes
how the car handles: light weight = twitchy, more drag = gripky.

## Exit codes (scripting)

| code | meaning |
|---|---|
| 0 | running |
| 2 | usage error / disallowed `--set` |
| 3 | game already over (win or loss) |
| 4 | file not found / invalid game file |
| 10 | **won** |
| 30 | **lost** |

`retry` returns the restored state's code (`0`, `10`, or `30`).

## Provable solvable

`autopilot.py` plays a game file purely through the public CLI and wins:

```
$ python3 roadrunner.py new g.svg --style hard --seed 7 --force
$ python3 autopilot.py g.svg
…
game over after 158 calls: won
```

Verified on 9 games (3 styles × 3 seeds), all won with ≤ 4.5% off-road —
the default rules are fair, not just solvable in principle.

## Tests

Run the history/retry CLI regression tests with:

```sh
python3 -m unittest discover -s tests -v
```
