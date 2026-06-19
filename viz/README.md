# MAPF visualiser

Two-step pipeline: a C++ tool runs a solver and dumps the solution to JSON; a
dependency-free Python script renders it.

## 1. Export a solution to JSON (inside the Docker container)

```sh
bazel run //solver:export_solution -- \
    --width 32 --height 20 --agents 40 --obstacles \
    --solver pibt_solver --seed 42 \
    --out viz/data/arena.json
```

Flags:
| flag | meaning | default |
|------|---------|---------|
| `--width`, `--height` | grid size | 32 × 20 |
| `--agents` | number of agents (random distinct start/goal) | 40 |
| `--obstacles` | add rectangular pillar obstacles (arena look) | off |
| `--solver` | any registered solver (`pibt_solver`, `astar_solver`, `bfs_solver`) | `pibt_solver` |
| `--seed` | RNG seed for the instance | 42 |
| `--out` | output JSON path (relative to repo root) | `viz/data/demo.json` |

## 2. Render (on the host, Python 3 standard library only)

```sh
python3 viz/visualize.py viz/data/arena.json --out viz/out
```

Produces:
* `viz/out/arena.html` — interactive animation: HUD (solver / agents / makespan /
  comp_time / time step), play–pause, time slider, speed, trails toggle. Open in
  any browser.
* `viz/out/arena_trails.svg` — static "all trajectories" image for the report.

## JSON schema

```json
{
  "width": 32, "height": 20, "solver": "pibt_solver",
  "makespan": 39, "comp_time_ms": 16.3,
  "obstacles": [[x, y], ...],
  "agents": [
    {"id": 0, "start": [x, y], "goal": [x, y], "path": [[x, y], ...]}
  ]
}
```

Coordinates are grid cells; an agent's `path` lists its cell at every time step
(equal length for all agents — they move in lockstep), so position at step `t`
is `path[t]`.
