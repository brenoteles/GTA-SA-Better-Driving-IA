# Handoff: Real Traffic Fix (GTA SA Better Driving IA)

State as of 2026-11-14. Read this first, then `docs/sensores.html` (the progress checklist, published as
https://claude.ai/artifact/ApRAiDCM7Dbfki8r8famLs).

## What this is

An ASI mod (plugin-sdk, GTA San Andreas 1.0 US HOODLUM) that drives every AI car: car following, curves, lane
changes, overtaking, routes, junctions, actuated traffic lights. The goal is realistic traffic, few crashes and a fast
Grove Street to Bayside trip, proven by the in-game AutoTest.

- Repo: https://github.com/brenoteles/GTA-SA-Better-Driving-IA (branch `main`; this folder is its own repo).
- Game: `C:\Users\death\Games\GTA\GTA San Andreas`. The build copies `RealTrafficFix.SA.asi` to its `scripts` folder.
- Main code: `RealTrafficFix2/source/RealTrafficFix.cpp` (~7,300 lines) and `AutoTest.h` (~1,700).
- Settings: `RealTrafficFix2/RealTrafficFix.ini`. The test launcher copies it over the game's ini at every test start,
  so change the repo ini, not the game's.

## Working rules the user set

- Commit and push after every change.
- Keep `docs/sensores.html` as a checklist map grouped by area (CARLA, openpilot, SUMO/network, map/physics, tools,
  open issues). Update items in place (status, result, commit). Do not append paragraphs. Republish the artifact.
- Work with Gemini through the Antigravity CLI (read-only for Gemini). Check every Gemini claim in the code first:
  about 1 in 4 suggestions was wrong.
- Study the code and game data before a long in-game test.
- The traffic light poles added by the mod stay floating (5.5 m up) until the user says otherwise.
- Never overwrite another mod's files; never use `--dangerously-skip-permissions`; never store the Gemini API key.

## Build and test

Build (VS 2026, MSBuild):

```
MSBuild RealTrafficFix2/RealTrafficFix.vcxproj "-p:Configuration=Release GTA-SA" -p:Platform=Win32
```

The game must be closed (it locks the ASI).

Test (the launcher starts the game, keeps it focused until it loads, runs the trip, restores the config):

```
pythonw tools/AutoTestUI.pyw --car 495 --drivers rapido --start                  # Sandking, fast driver, with traffic
pythonw tools/AutoTestUI.pyw --car 560 --drivers sozinho --notraffic --start     # Sultan, 'alone', no traffic
pythonw tools/AutoTestUI.pyw --car 411 --drivers maximo --notraffic --start      # Infernus, fastest possible
```

- Started with `--start`, the launcher closes itself when the test ends. Never start a second one while a test runs:
  two launchers once overlapped. Check with `tasklist` that `gta_sa.exe` isn't running first.
- MixSets `NoPauseWhenUnfocus = 1` stays on (the user wants the game running when minimised).
- `--notraffic` sets the game's random car limit to 0 and the mod's spawner off every frame (the mod's own ini, read
  after the test's, used to switch the spawner back on). The log's `[pulse]` lines show `random cars 0 ... ambient
  created 0` when it works.

- Standard test: Sandking (495), fast driver, Grove Street to Bayside (~10.7 km).
- Steering tests: Sultan (560), profile `alone` (fast driver at 2x the road speed, same curves), no traffic.
- Logs: `scripts/autotest_<date>_<time>/autotest.log`. The summary at the end has one line per feature counter.
  `=== OFF ROAD` dumps hold a 15 s trace (speed, steer, yaw, radius, slip). `=== OSCILLATION` means the car
  went forwards/backwards (pedals), not weaving.

### Git push

The git credential stored for https expired. `gh` is logged in. Push without changing the config:

```
git -c credential.helper= -c "credential.helper=!gh auth git-credential" push origin HEAD
```

### Gemini (Antigravity)

```
C:/Users/death/AppData/Local/agy/bin/agy -p "<prompt with file paths>" --model gemini-4-argon-high --print-timeout 400s
```

Settings: `~/.gemini/antigravity-cli/settings.json` (read allowed for the GTA folder and the scratchpad; write and
commands denied). Pass paths, not code. For a diff review, write `git diff` to a file and pass its path.

## Steering and lane keeping (the latest work)

The user reported cars zig-zagging. Findings, in order:

1. The game steers by the angle to its next point (steering = heading error, same gain at any speed). At 170 km/h,
   11 degrees off was full lock (0.2 rad, about 3 g): lock-to-lock weaving.
2. A steering rate limit (openpilot style) made it worse: its lag grows with the correction. Replaced by a short
   low-pass filter (0 to 0.08 s by speed).
3. A first PID measured the error per road segment, against a straight line through each link: the line stepped
   sideways at every node and kicked the PID. It also handed the wheel back to the game at every junction node.
4. The game itself moves every AI car one lane left or right at random every 4 to 7 nodes on straight stretches
   (`PickNextNode*` at 0x427655, 0x42E962; 1 in 4 nodes at 0x427C7C). With the densified nodes that was a lane every
   few seconds, and the mod's keep-right moved it back: 284 "back to the right" in 6 minutes on an empty map.

5. A PID on that line (spring 1.2 rad/s) was soft in curves: the car went wide and took seconds to come back.
6. The game's lane number kept dropping one lane (418 phantom "back to the right" changes in 7 minutes on an empty map,
   the car not moving), and the controller followed it left while keep-right brought it back.

Current design (commits d8dcc66 to the latest):

- `NoRandomLaneChanges = 1`: the three `jne` are patched to `jmp`. The mod makes every lane change itself.
- `LaneControllerSteer`: predictive steering on the car's own model, after
  [ten-tenths](https://github.com/antoniooodev/ten-tenths) (single-track car in the road's frame: offset, heading
  error, yaw rate, a curvature preview; no PID).
  - Reference: a Catmull-Rom line through the lane point at every link (offset across the link's own direction, as
    the game places lanes), sampled every ~3 m, extended 30 m back (`BuildLaneLine`, `SplineInto`).
  - Each frame ~40 wheel angles are simulated 1.2 s ahead (24 steps of 0.05 s): yaw goes to
    `gain * v tan(d) / (L + K v^2)` with a 0.2 s lag, capped by the grip the car has shown (`limits.txt`); heading
    error and offset integrate against the line's curvature. A candidate is held 0.4 s, then follows the road's own
    curvature. Cost: offset (and lateral speed) error, terminal x4, a small frame-rate-free wheel-change cost.
  - `lkGain` (per vehicle) is learned while driving: the yaw the car really makes over what L/K say, on a steady wheel.
  - A lane change uses the target lane's line with the S as where the car should be across over the 1.2 s.
  - The racing line (racing drivers) is followed by the same predictive steering; pure pursuit only as a fallback.
  - The game steers only into sharp turns (over 40 degrees within 1.5 s), without road data, or below 29 km/h
    (blend to 47 km/h). Hand-overs are eased.
  - While the controller drives, the game's lane is held to the lane the car is in (only the mod's lane changes move
    it).
- AutoTest counters: `lane controller ...` (frames, distance to the centre, why the game steered), `weaving (3 yaw-rate
  reversals > 0.15 rad/s within 4 s, above 54 km/h)`, `game lane put back ...`.

Results (Sultan, `alone`, Grove Street to Bayside):

| Test | Steering | Result | Weaving (test car) | Off the centre (avg) |
|---|---|---|---|---|
| 160701 | PID per segment | arrived 6.60 min, 0 collisions | (not measured) | 1.06 m |
| 165953 | PID on the spline | arrived 6.85 min, 0 collisions | 3 | 0.78 m |
| 170849 | predictive, first version | arrived 7.11 min, 0 collisions | 2 | 0.86 m |

The user said the predictive version "already improved a lot". Test 170849 still had traffic by mistake (see below), so
its "out of the lane" count (20,843 frames) includes other cars. Later fixes (lane changes measured against the S,
candidates held 0.4 s, lighter wheel-change cost) are not measured yet.

Infernus test (in progress at handoff): model 411, driver `maximo` (max performance: ignores road speed, 95% of the
measured grip, racing line, runs red lights at speed), no traffic.

## Other recent changes (still to confirm in game)

- Yellow lights: decided at 3.5 to 2.5 m/s2 by aggression (it was inverted: calm drivers went through from 60 m).
- Lane change timing: indicator 0.4 to 1.0 s, change 2.6 to 3.2 s, at most 9 degrees and 0.49 g.
- The extra lanes (79 one-way stretches) are uninstalled: one in LS put cars on the pavement. The lane version of the
  path files is kept in the scratchpad (`paths_lanes`); the backup without them is what's installed. To do: fix the
  width measurement (pavement counted as asphalt?) before reinstalling.
- `RealTrafficFix.steer.txt` (game scripts folder): learned turn model per car model. If a model learns a wrong
  understeer (it happened with the old rate limit), edit its line back. Sandking: `495 3.22803 0.00010836 158 36`.

## SUMO

- `tools/sa_to_sumo.py` converts the game's road network; `tools/sumo_profiles.py` makes the mod's 5 driver profiles
  x 5 car classes (handling.cfg, weighted by cargrp.dat) as a SUMO vTypeDistribution; `tools/sumo_compare.py`
  compares runs. Install: `pip install eclipse-sumo`.
- Result (3,000 identical trips): city time gap 1.5x gave the same time loss, -15% collisions and -40% emergency
  braking. Keep-right, cooperation and junction gaps changed little. To do: try `CityGapFactor` 0.7 -> 1.05 in game.

## Open issues and next steps

See the "a fazer" items in `docs/sensores.html`. The main ones:

1. Confirm the steering with traffic (Sandking test) and watch the `weaving`, `lane controller` and `game lane put back`
   lines. Find what lowers the game's lane number (the hold hides it).
2. Lane changes: "back to the right" fires hundreds of times per test; most are phantom (the car doesn't move).
3. Off the route at (2803, -431) and (-997, 1275) in every Sultan test: the car takes a different way there.
4. The Sultan hit something stopped on the road near (-486, 604) at 145 km/h with no traffic and fell off the bridge
   (test 162908). Find what the "leader" at 9 km/h was.
5. CityGapFactor 1.05 in game (SUMO says fewer crashes at no cost).
6. Extra lanes: fix the measurement, reinstall.
7. CARLA ideas 4-7 (path envelope, conflict point negotiation, leader lock, sensor rounds), the FLA4 path format,
   and linking the dead-end "traps" in the network.
