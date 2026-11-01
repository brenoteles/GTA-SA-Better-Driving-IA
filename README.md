# GTA SA Better Driving IA

Realistic AI traffic for GTA San Andreas (1.0 US), built on top of
[Real Traffic Fix](https://www.mixmods.com.br/2019/12/Real-Traffic-Fix.html) by Valdir da Costa Júnior (MIT, see LICENSE).

## What it does

- Car following (IDM) with per-driver profiles: calm, safe, normal, fast, aggressive
- Curve speed from the planned route, learned grip limits, handbrake turns for faster drivers
- Lane changes, overtaking with a priority lane board, pass requests (headlight flash), zipper merges
- Planned routes (A*) for every car, also over roads the game leaves unused (dirt roads); the game is made to take
  the route's exit at every junction; rejoining the route after leaving it
- Junctions: first-come-first-served without lights, don't-block-the-box, conflict prediction
- Getting around broken-down/blocking vehicles, deadlock breaking (circles of waiting cars)
- Ambient spawner around the player out of sight, never more cars than the roads around hold; longer despawn distances
- Never goes round a blocked lane through the oncoming lanes

## Layout

| Path | |
|---|---|
| `RealTrafficFix2/source/RealTrafficFix.cpp` | the mod |
| `RealTrafficFix2/source/AutoTest.h` | in-game automated test (route trip, collision/jam logs, road scan) |
| `RealTrafficFix2/RealTrafficFix.ini` | settings |
| `RealTrafficFix2/RealTrafficFix.routefixes.txt` | lane data corrections |
| `tools/AutoTestUI.pyw` | test launcher UI (game folder from the `GTA_SA_DIR` environment variable) |
| `tools/collision_diagram.py`, `tools/route_scan.py` | collision diagrams, road scan analysis |

## Building

Visual Studio with [plugin-sdk](https://github.com/DK22Pac/plugin-sdk) (`PLUGIN_SDK_DIR` set), `RealTrafficFix.sln`,
configuration `Release GTA-SA`, platform Win32. Output: `RealTrafficFix.SA.asi`; with `GTA_SA_DIR` set the build copies
it to the game's `scripts` folder. Copy `RealTrafficFix.ini` and `RealTrafficFix.routefixes.txt` there too.

## Path data tools

| Path | |
|---|---|
| `tools/sa_paths.py` | reader/writer and analysis of the game's road network (`data/paths/NODES*.DAT`) |
| `tools/lights_everywhere.py` | traffic lights at every urban junction, coherent cycles (main axis / crossing roads) |
| `tools/densify_paths.py` | ~2x the road nodes: links split on a Catmull-Rom curve, navis spilled to areas with room (needs LoadAllPathNodes) |
| `tools/reclassify_paths.py` | road speed classes from the network's shape (long multi-lane unlit stretches -> freeway, short lit ones -> normal) |
| `tools/check_paths.py` | structural check of path files (link symmetry, navi references, lanes) |

`python tools/lights_everywhere.py "<game>/data/paths" "<game>/modloader/RealTrafficFix Paths/gta3.img"` writes the
changed `NODESn.DAT` files; modloader puts them in `gta3.img`. Delete that folder to go back to the original lights.
The game's files themselves aren't in this repository.
