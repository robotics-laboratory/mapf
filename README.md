# MAPF
Experiments in Multi-Agent Pathfinding.

The repository implements three solvers (BFS, A*, PIBT) on grid graphs, a benchmark
and ablation test suite, a JSON exporter, and a dependency-free Python visualizer that
turns exported solutions into an interactive HTML animation and an SVG trajectory image.

## Getting started

### Clone repository

```sh
git clone git@github.com:robotics-laboratory/mapf.git
```

### Docker 

#### Build and Run container

```sh
# Build and Run
docker compose build mapf
docker compose up -d mapf

# Attach to shell
docker exec -it mapf-${username} zsh

# Stop container
docker stop mapf-${username}
```

### Bazel

#### Build and Run target

```sh
# Build 
bazel build //...

# Run 
bazel run //... -- ${binary options}

# Test
bazel test --test_output=all //...
```

## Running solver

### Show information about solver binary options

```sh
bazel run //solver:main -- --help
```

### List available solver names

```sh
bazel run //solver:main -- --list-solvers
```

Solver input is a file with text formated [MAPFProblem](models/proto/models.proto#L22) protobuf message. [See](data/mapf_problem_1.pb.txt) example of file structure.

Solver output is a file with text formated [MAPFSolution](models/proto/models.proto#L27) protobuf message.

Use paths, relative to MODULE directory.

### Solver run command example

```sh
bazel run //solver:main -- -s bfs_solver -i data/mapf_problem_1.pb.txt -o data/mapf_solution_1.pb.txt
```

## Visualizing a solution

Solve an instance and write a JSON description (map, obstacles, every agent's path) suitable for the visualizer:

```sh
bazel run //solver:export_solution -- \
    --width 32 --height 20 --agents 40 --obstacles \
    --solver pibt_solver --out viz/data/arena.json
```

Render an interactive HTML animation and a static SVG trajectory image:

```sh
python3 viz/visualize.py viz/data/arena.json --out viz/out
```

Open `viz/out/arena.html` in a browser for the animation, or use `viz/out/arena_trails.svg` as a static figure.

## Repository contents

Guide to packages:

* __geom__: Geometry primitives (`Vec2`, floating point comparators). See __geom/tests__ for usage and __geom/proto__ for proto definitions.

* __graph__: Graph primitives (`Node`, `Edge`, `Endpoints`). See __graph/test__ for usage and __graph/proto__ for proto definitions.

* __models__: MAPF problem types (`AgentState`, `AgentTask`, `AgentPath`, `MAPFProblem`, `MAPFSolution`). See __models/test__ for usage and __models/proto__ for proto definitions.

* __solver__: Package with available MAPF-problem solvers. 

    * __solver/solvers__: Solver implementations (BFS, A*, PIBT). The `Solver` interface is defined [here](solver/solvers/include/solvers/solver_base.h).

    * __solver/factory__: Factory with registered solvers. After registering a solver in [the factory constructor](solver/factory/src/solver_factory.cpp), it automatically becomes available in CLI options.

    * __solver/main__: Solver binary. See [CLI options](solver/main/src/main.cpp) for the supported flags.

    * __solver/tools__: `export_solution` binary — solves an arena instance and writes a JSON file consumed by the visualizer.

    * __solver/tests__: GoogleTest correctness tests plus the benchmark, ablation and statistical experiments (`Experiment.OperatingEnvelope`, `Experiment.LargeScale`, `Experiment.Ablation`, `Experiment.Suboptimality`).

* __viz__: Standard-library Python visualizer. `visualize.py` renders a JSON solution to HTML + SVG; `plot_envelope.py` renders the operating-envelope plot; `plot_suboptimality.py` renders the statistical box-plots.

* __data__: Example MAPF problems (text-format protobuf).
