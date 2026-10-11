# Generated line-art tracks

Two monochrome tracks were generated as PNG art, then resized to 632×424 for
faster image-track loading in the game:

| Track | Generated art | Playable PNG | Game SVG |
|---|---|---|---|
| Sweeping serpentine | `sweeping-serpentine.png` | `sweeping-serpentine-playable.png` | `../games/sweeping-serpentine.svg` |
| Rounded circuit | `rounded-circuit.png` | `rounded-circuit-playable.png` | `../games/rounded-circuit.svg` |
| Roadrunner character | `roadrunner-character.png` | `roadrunner-character-small.png` | `../games/roadrunner-character.svg` |

The rounded circuit is at turn 3; the serpentine is at turn 2 (two off-road
attempts were retried and archived under its `redo/` folder).

The current roadrunner-character game is **WON** at **100% coverage and
0.0000% off-road in 91 turns** (240.4 simulated seconds). It keeps the 1%
off-road cap. To meet the turn limit, this race uses a 900-step-per-call cap
and the rule-authorized 0.5 vehicle weight; see
`../games/roadrunner-character-under100-rules.json`. Its 92 snapshots (turns
0–91) are in `../games/roadrunner-character.history/`. The preceding full
coverage run took 501 turns and is archived with its history as
`../games/roadrunner-character-100pct-501-turns.svg` and
`../games/roadrunner-character-100pct-501-turns.history/`. The earlier
full-coverage attempts are also preserved as `../games/roadrunner-character-100pct-474s.svg`
(100%, 0.0356% off-road) and `../games/roadrunner-character-95pct.svg`
(95.1099%, 0.0413% off-road). The majority-target run is archived as
`../games/roadrunner-character-55pct.svg` with its turn history in
`../games/roadrunner-character-55pct.history/` (55.3% coverage, 6.3% off-road
under its older 55%/30% limits).

Continue playing from the project root, for example:

```sh
python3 roadrunner.py status games/roadrunner-character.svg --map
python3 roadrunner.py status games/rounded-circuit.svg --map
python3 roadrunner.py run games/rounded-circuit.svg --fx 60 --torque 0 --steps 60
python3 roadrunner.py history games/rounded-circuit.svg
```

The PNGs are dark-line-on-white picture tracks. Their `--road-width` settings
match the generated stroke widths: 18 px for the serpentine, 14 px for the
circuit.
