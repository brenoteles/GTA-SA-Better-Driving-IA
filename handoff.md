# Handoff: Real Traffic Fix (GTA SA Better Driving IA)

State at the end of the 2026-11-14 session. Read this first, then `docs/sensores.html` (the progress checklist,
published as https://claude.ai/artifact/ApRAiDCM7Dbfki8r8famLs; it was last updated before the race work below).

## 2026-11-15 session: Cadrona with traffic regressed - read this first

Report page: `docs/avancos.html` (published https://claude.ai/artifact/5P8pM1RmV3BGzL4974qwYt): physics findings, what was
ported, every test since 25/10 (data: `docs/test_history.json`, from `python tools/test_history.py`, written by Gemini).

Done this session:
- `HoldUntilCollision` (RealTrafficFix.cpp): the race mode's fix generalised. AI cars with physics, over 120 m from the
  player, where `CColStore::HasCollisionLoaded` is false, are frozen (at most 10 s, then 30 s free), and the IPL/
  collision/path loading is asked for at one of them per frame. Near the player it held a car for good (the queue behind
  it never moved), hence the 120 m.
- Ambient spawn: a model not loaded is requested and another spot tried (it used to end the spawn turn); `AmbientSpawnAhead`
  skips 3 of 4 spots behind a moving player; spots need `HasCollisionLoaded`; the start speed is the mean speed of the cars
  going that way within 120 m. `SET_LOAD_COLLISION_FOR_CAR_FLAG` on spawned cars made them appear inside the ground: removed.
- Gemini (agy, gemini-3.8-flash-high) now has write access to this repo and command(*) in
  `~/.gemini/antigravity-cli/settings.json` (the user asked for full rights). It wrote the history JSON and the HTML; its
  draft had invented notes and wrong car models in the trip table, corrected before publishing: check its numbers.

Results (Cadrona 527, FAST, Grove -> Bayside, traffic):
- Before (02/11, 5 runs): 7.19-8.72 min, 1-2 test-car collisions, 0.67-4.62 traffic collisions/min.
- 15/11 00:06, everything on: 13.66 min, 14 collisions, 5.78/min (79); 867 holds for 946 cars made. Regression.
- 15/11 00:21, HoldUntilCollision and AmbientSpawnAhead off, stopped at 2.86 km: 3.39 min there with 0 collisions (the run
  with everything on was at 4.55 min with 2). Better, still slower than 02/11 (~89 km/h average then).

State: both new features are OFF in RealTrafficFix.ini (`HoldUntilCollision = 0`, `AmbientSpawnAhead = 0`); the other spawn
changes stay on. KinematicSteer and AiABS (from the proving ground, tested without traffic only) are on for all traffic.

Next:
1. Full Cadrona FAST run with the current ini; if still over ~8.7 min, try `KinematicSteer = 0`, then `AiABS = 0` (or keep
   both only for racing drivers).
2. Then retry HoldUntilCollision for far cars only if they really sink (it is only needed where the player is not).
3. The steer-based turn speed cap for the off-route spots (see the proving ground section below).

## Proving ground (2026-11-14, second session)

A new flat test platform and a bench to learn GTA SA's real car physics, then the results ported to the AI.

- Platform: `tools/make_gridmap.py` writes `modloader/RealTrafficFix ProvingGrid/` (model 19400 `pg_grid`, 100 m tiles
  with a 10 m grid texture, tarmac collision overlapping 0.5 m, `Loader.txt` with the gta.dat lines: modloader needs it
  for new .ide/.ipl). x 500..2900, y -2950..-2150, z 300, above the sea south of LS.
- Bench: `ProvingGround.h`, active only while `scripts/RealTrafficFix.proving.ini` exists. `python tools/proving.py
  start|push|stop|report`; `push` reloads the ini in the running game (no rebuild). Driver `cjai` = CJ at the wheel with
  the AI's physics (status forced to STATUS_PHYSICS; the AI's SteerAICarWithPhysics call 0x6B52A8 is replaced). `AB = 1`:
  a second car 40 m to the right with `[Steer.B]`/`[Speed.B]` (two hypotheses per run). Tests: slalom, zig-zag, moose,
  Z turns, chicanes, hairpin after top speed, brake, top speed, open-loop steps/ramps. Logs: `scripts/proving/<stamp>/`.
- What the physics is (code: gta-reversed CAutomobile::ProcessControl/ProcessCarWheelPair, CVehicle::ProcessWheel; and
  measured): below the grip each wheel cancels its contact's sideways speed every frame: no slip, the car turns at
  exactly v tan(steer)/L with no lag (measured L = wheelbase). AI grip = 10 x adhesive(4.5 tarmac) x tractionMult m/s2,
  load-independent (the player's grip depends on suspension load and the game scales the player's steering down with
  speed); in practice the front axle gives way first, ~1.6-2 g on an Infernus. No ABS: brake past a wheel's adhesion
  locks it (Infernus from ~70% pedal) and it loses its steering. The AI Infernus reaches 385 km/h.
- Best bench result (Infernus, kinematic steering, 42% of the formula grip, ABS, traction control): max offset 0.2-0.6 m
  on zig-zag/slalom/Z/chicanes/hairpin.
- Ported to the mod (ini: KinematicSteer, SteerBandwidth 1.5, AiABS, RacerGripShare 0.42): kinematic steering in
  LaneControllerSteer, ABS for every AI car, racers' curve grip from the formula, learned grip capped by it (this also
  lowers the stored limits file).
- Grove -> Bayside, Infernus `maximo`, no traffic: 3.53 min, 0 collisions, 188 km/h average (route 11 km). Still leaves
  the route at (2751, -1663), (1793, 892) and (-2743, 2336). At (1793, 892) the junction turn is steered by the game and
  is not in the speed plan: it accelerated on full lock at 2.3 g, understeered 37 m and spun. Next step (written, not
  tested, reverted): cap the speed to the turn the wheel asks for, v <= sqrt(G L / tan|steer|), in the AI hook.

## Where things stand (before the proving ground)

The user's current goal: an AI that races like they do. They race the AI side by side (race mode below), and the AI
should learn from their driving. Their verdict on the last versions:

- "freia demais nas curvas, precisa aprender a manter a velocidade, seguir melhores linhas"
- "nao consegue seguir a minha linha, nao e agressiva o suficiente e acaba perdendo a linha; quando perde, o carro vai
  muito mal e nao consegue voltar a ela"

The last commit (racer steering more aggressive + ghost only where it follows the route + FORCED fix) was tested once
(183122, Bullet, AI alone): it went off the route at (2790, -413) and (1792, 891), spun at (1788, 900) (slip 0.8 rad,
yaw 2 rad/s) and the game closed with no summary and no watchdog entry. Full gas 23% of the time (worse than 50%
before). **Treat the last commit as unproven, possibly a regression.** Next step: rerun it, read `drive.csv` around
those points, and if it's worse, revert the racer weights in `LaneControllerSteer` (line error x4, wheel cost 15,
candidate span 0.04, maxSteer 0.45) one at a time.

## What this is

An ASI mod (plugin-sdk, GTA San Andreas 1.0 US HOODLUM) that drives every AI car: car following, curves, lane
changes, overtaking, routes, junctions, actuated traffic lights. The goal is realistic traffic, few crashes and a fast
Grove Street to Bayside trip, proven by the in-game AutoTest.

- Repo: https://github.com/brenoteles/GTA-SA-Better-Driving-IA (branch `main`; this folder is its own repo).
- Game: `C:\Users\death\Games\GTA\GTA San Andreas`. The build copies `RealTrafficFix.SA.asi` to its `scripts` folder.
- Main code: `RealTrafficFix2/source/RealTrafficFix.cpp` (~7,500 lines) and `AutoTest.h` (~1,900).
- Settings: `RealTrafficFix2/RealTrafficFix.ini`. The test launcher copies it over the game's ini at every test start,
  so change the repo ini, not the game's.

## Working rules the user set

- Commit and push after every change.
- Keep `docs/sensores.html` as a checklist map grouped by area. Update items in place (status, result, commit). Do not
  append paragraphs. Republish the artifact.
- Work with Gemini through the Antigravity CLI (read-only for Gemini). Check every Gemini claim in the code first:
  about 1 in 4 suggestions was wrong.
- Study the code and game data before a long in-game test.
- The traffic light poles added by the mod stay floating (5.5 m up) until the user says otherwise.
- Never overwrite another mod's files; never use `--dangerously-skip-permissions`; never store the Gemini API key.
- No MTA: the user chose local races over installing MTA (MTA doesn't load ASI/modloader).

## Build and test

```
MSBuild RealTrafficFix2/RealTrafficFix.vcxproj "-p:Configuration=Release GTA-SA" -p:Platform=Win32
```

The game must be closed (it locks the ASI; the post-build step kills a running game and then fails to copy).

```
pythonw tools/AutoTestUI.pyw --car 495 --drivers rapido --start                        # Sandking, fast driver, traffic
pythonw tools/AutoTestUI.pyw --car 560 --drivers sozinho --notraffic --start           # Sultan, 'alone', no traffic
pythonw tools/AutoTestUI.pyw --car 541 --drivers maximo --notraffic --start            # Bullet, fastest, AI alone
pythonw tools/AutoTestUI.pyw --car 541 --drivers maximo --notraffic --race --start     # the user races the AI
```

- With `--start` the launcher closes itself when the test ends. Never start a second one while a test runs (two once
  overlapped). Check `tasklist | grep gta_sa` first. If you kill a test, also delete
  `scripts/RealTrafficFix.autotest.ini`.
- Never chain a launcher after `git push` in one command: the push once hung, and the launcher started later on its
  own and overlapped the next test.
- MixSets `NoPauseWhenUnfocus = 1` stays on (the user wants the game running when minimised).
- `--notraffic`: the game's random car limit 0 and the mod's spawner off, every frame. `[pulse]` lines show
  `random cars 0 ... ambient created 0` when it works.
- Logs in `scripts/autotest_<date>_<time>/`: `autotest.log` (summary at the end; `=== OFF ROAD` dumps a 15 s trace),
  `drive.csv` (the AI test car every 0.1 s: speed, target, planned curve speed, limiter, pedals, yaw, g, off-route),
  `race.csv` (race mode: both cars every 0.1 s).

Git push (the stored https credential expired; `gh` is logged in):

```
git -c credential.helper= -c "credential.helper=!gh auth git-credential" push origin HEAD
```

Gemini: `C:/Users/death/AppData/Local/agy/bin/agy -p "<prompt with file paths>" --model gemini-4-argon-high
--print-timeout 400s` (pass paths, not code; write `git diff` to a file for reviews).

## Race mode and learning from the player

- `--race`: the player in a second car beside the AI's (after MTA's race gamemode): frozen 3-2-1 countdown with sounds,
  place/gap on screen, back on the road at the last good spot (AI: on its route, heading along it, 30+ km/h) after a
  fall/roll/sink/8 s stuck/4 s lost, further back each time it repeats. The AI's reverse manoeuvre is cleared at GO.
- Away from the player the game unloads the map: the AI's car gets `SET_LOAD_COLLISION_FOR_CAR_FLAG`, and every frame
  `CIplStore::AddIplsNeededAtPosn`, `CColStore::AddCollisionNeededAtPosn`, `ThePaths.SetPathsNeededAtPosition`, and
  is held (frozen) until the collision there is loaded. Before this it fell through elevated roads.
- `tools/race_learn.py <race.csv> <model> <scripts> [--write] [--ghost]` (game closed: the mod rewrites the limits file
  every minute):
  - `--write`: the model's learned grip raised to the player's 75th percentile g in 30-250 m curves x 0.85 (never
    lowered; ceiling 3 g). Now: Infernus 411 2.59 g, Bullet 541 2.30 g.
  - `--ghost`: the player's line and speeds (every ~3 m) to `scripts/RealTrafficFix.ghost.<model>.txt`. Written from
    the best races: Bullet 180612 (3.44 min), Infernus 175443 (2.71 min).
- Ghost line (`UpdateGhost`, `GhostLine`): a racing driver on the player's line (within 12 m to start, 20 m to keep,
  same direction, and the line 40/80 m ahead within 8 m of its route) steers along it with the predictive steering
  and takes the player's speeds x 0.95 (braking for slower points ahead) instead of its own curve and heading limits.
  Summary line: `on the player's line (ghost)`.

Results:

| Race / test | Car | Player | AI | Notes |
|---|---|---|---|---|
| 173316 | Infernus | 2.79 min | stuck | fell off a bridge, respawn loop under it |
| 174918 | Infernus | 2.84 min | - | hit something on the SF bridge at 180 km/h, into the water |
| 175951 | Bullet | 3.45 min | 4.25 min | player full gas 89% / braking 1%; AI 51% / 18% |
| 180612 | Bullet | 3.44 min | 4.23 min | |
| 181747 | Bullet, AI alone | | 4.06 min | junction turns + start fix |
| 182318 | Bullet, AI alone | | 3.96 min | racers ignore lights; braking 13% (curve 9%) |
| 183122 | Bullet, AI alone | | did not finish | aggressive steering + ghost gate; see the top |

Where the AI loses to the player (race 180612, by 100 m bins): ~30 s in the first km (junction turns at 34 vs 70-85
km/h; ~1 s of reverse at GO, both since fixed), then 2-3 s in many curves. The player steers ~10x more than the AI at
130-220 km/h (p99 0.42 vs 0.043 rad at 130-170).

## Steering and lane keeping

- The game steers by heading error with the same gain at every speed (lock-to-lock weaving at 170 km/h), and moved
  every AI car a lane left/right at random every 4-7 nodes (`NoRandomLaneChanges = 1` patches 0x427655, 0x42E962,
  0x427C7C `jne` -> `jmp`).
- `LaneControllerSteer`: predictive steering on the car's own model, after
  [ten-tenths](https://github.com/antoniooodev/ten-tenths): a Catmull-Rom lane-centre line (extended 30 m back), ~40
  wheel angles simulated 1.2 s ahead (yaw lag 0.2 s, measured L/K, learned grip cap, curvature preview), held 0.4 s
  then following the road's curvature; cost on offset and lateral speed plus a wheel-change cost. `lkGain` learned
  while driving. Lane changes use the target lane's line with the S as the reference. The game steers only into sharp
  turns (over 40 degrees within 1.5 s), without road data, or below 29 km/h. While the controller drives, the game's
  lane is held to the lane the car is in.
- Racing drivers (last commit, unproven): line error x4, wheel cost 15 (others 60), candidates +-0.32 rad, up to 0.45
  rad of steering at speed (the game holds its AI to 0.2 from 126 km/h), ghost offset tolerated to 16 m.
- Racing line (`RacingLine`) is off: it cut off the side of a bridge at 200 km/h twice.
- Steering low-pass 0-0.08 s by speed (a rate limit made the weaving worse).

## Speed (curves) for racing drivers

- Fast-curve grip boost capped at the learned grip; the road's curvature where the car is (from the lane line) in the
  heading limit; an understeer aid (full lock, turning under 75% of the road: off the gas, braking).
- Junction turns: radius also from the turn's angle (racers 16 m for 90 degrees, others 10 m), 90% of the grip for
  racers (75% others), also for tight turns in the heading limit. Gentle bends near junction nodes use the long window
  (a 12 m window on node corners read a 220 m curve as ~100 m).
- Racers flat out (no jerk ramp, no speed-scaled ceiling; traction and stability control stay on), run red lights
  without slowing for any light (cross traffic still slows them).
- FORCED obstacle (a drop/wall from the ground under the sensors) only from the front rays and off the route: the side
  rays past bridge edges made walls that braked racers on bridges.

## Open issues and next steps

1. The last commit (see the top): rerun, check, revert parts if it's worse.
2. The interchange at (2790, -413) and the junction at (1788-1792, 880-900): every test leaves the route there, and the
   last one spun at (1788, 900). Look at the route and the lane line there.
3. The AI should keep speed in curves like the player: compare `drive.csv` (AI) with `race.csv` (player) per 100 m;
   where the ghost is on, the AI should be within ~5% of the player's speed.
4. More races with the user, then `race_learn.py --write --ghost` from their best one per car.
5. Traffic (non-racers): confirm the steering with traffic (Sandking test) and watch `weaving`, `lane controller`,
   `game lane put back`. "Back to the right" fires hundreds of times per test, most phantom.
6. From before: CityGapFactor 0.7 -> 1.05 (SUMO: -15% collisions at no cost), the extra lanes (uninstalled: one in LS
   ran on the pavement; fix the width measurement), CARLA ideas 4-7, the FLA4 path format, dead-end "traps".
7. `docs/sensores.html` is behind: add the race/ghost/steering items.

## SUMO

`tools/sa_to_sumo.py` (network), `tools/sumo_profiles.py` (the mod's driver profiles x car classes from
handling.cfg/cargrp.dat), `tools/sumo_compare.py`. City time gap 1.5x: same time loss, -15% collisions, -40% emergency
braking.
