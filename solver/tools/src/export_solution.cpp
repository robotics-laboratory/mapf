
// example:
//   bazel run //solver:export_solution -- \
//       --width 32 --height 20 --agents 40 --obstacles --solver pibt_solver \
//       --out viz/data/arena.json
// output:
//   { "width", "height", "solver", "makespan", "comp_time_ms",
//     "obstacles": [[x,y], ...],
//     "agents": [ { "id", "start":[x,y], "goal":[x,y], "path":[[x,y], ...] }, ... ] }

#include "factory/solver_factory.h"

#include "graph/graph.h"
#include "models/models.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <vector>

namespace {

using mapf::graph::Edge;
using mapf::graph::Edges;
using mapf::graph::Endpoints;
using mapf::graph::Graph;
using mapf::graph::Node;
using mapf::graph::NodeId;
using mapf::graph::Nodes;
using mapf::models::AgentId;
using mapf::models::AgentTask;
using mapf::models::AgentTasks;
using mapf::models::MAPFProblem;

struct Config {
    uint32_t width = 32;
    uint32_t height = 20;
    uint32_t agents = 40;
    uint32_t seed = 42;
    bool obstacles = false;
    std::string solver = "pibt_solver";
    std::string out = "viz/data/demo.json";
};

Config ParseArgs(int argc, char* argv[]) {
    Config cfg;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&]() -> std::string { return (i + 1 < argc) ? argv[++i] : ""; };
        if (arg == "--width") {
            cfg.width = static_cast<uint32_t>(std::stoul(next()));
        } else if (arg == "--height") {
            cfg.height = static_cast<uint32_t>(std::stoul(next()));
        } else if (arg == "--agents") {
            cfg.agents = static_cast<uint32_t>(std::stoul(next()));
        } else if (arg == "--seed") {
            cfg.seed = static_cast<uint32_t>(std::stoul(next()));
        } else if (arg == "--solver") {
            cfg.solver = next();
        } else if (arg == "--obstacles") {
            cfg.obstacles = true;
        } else if (arg == "--out") {
            cfg.out = next();
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
        }
    }
    return cfg;
}

bool IsObstacle(const Config& cfg, uint32_t x, uint32_t y) {
    if (!cfg.obstacles) {
        return false;
    }
    constexpr uint32_t kBlock = 2;
    constexpr uint32_t kSpacing = 6;
    constexpr uint32_t kMargin = 3;
    if (x < kMargin || y < kMargin || x + kMargin >= cfg.width || y + kMargin >= cfg.height) {
        return false;
    }
    return (x % kSpacing < kBlock) && (y % kSpacing < kBlock);
}

Graph BuildArena(const Config& cfg, std::vector<NodeId>& free_cells) {
    Nodes nodes;
    Edges edges;
    auto id_of = [&](uint32_t x, uint32_t y) { return y * cfg.width + x; };

    for (uint32_t y = 0; y < cfg.height; ++y) {
        for (uint32_t x = 0; x < cfg.width; ++x) {
            if (IsObstacle(cfg, x, y)) {
                continue;
            }
            const NodeId id = id_of(x, y);
            nodes.emplace(id, Node(id));
            free_cells.push_back(id);
        }
    }
    for (uint32_t y = 0; y < cfg.height; ++y) {
        for (uint32_t x = 0; x < cfg.width; ++x) {
            if (IsObstacle(cfg, x, y)) {
                continue;
            }
            const NodeId id = id_of(x, y);
            if (x + 1 < cfg.width && !IsObstacle(cfg, x + 1, y)) {
                edges.emplace(Edge(id, id_of(x + 1, y)));
                edges.emplace(Edge(id_of(x + 1, y), id));
            }
            if (y + 1 < cfg.height && !IsObstacle(cfg, x, y + 1)) {
                edges.emplace(Edge(id, id_of(x, y + 1)));
                edges.emplace(Edge(id_of(x, y + 1), id));
            }
        }
    }
    return Graph(std::move(nodes), std::move(edges));
}

void WriteXY(std::ofstream& out, NodeId id, uint32_t width) {
    out << "[" << (id % width) << "," << (id / width) << "]";
}

}  // namespace

int main(int argc, char* argv[]) {
    const Config cfg = ParseArgs(argc, argv);

    std::vector<NodeId> free_cells;
    Graph graph = BuildArena(cfg, free_cells);

    if (cfg.agents > free_cells.size()) {
        std::cerr << "Not enough free cells (" << free_cells.size() << ") for " << cfg.agents
                  << " agents\n";
        return 1;
    }

    std::mt19937 rng(cfg.seed);
    std::vector<NodeId> starts = free_cells;
    std::shuffle(starts.begin(), starts.end(), rng);
    std::vector<NodeId> goals = free_cells;
    std::shuffle(goals.begin(), goals.end(), rng);

    AgentTasks tasks;
    for (uint32_t k = 0; k < cfg.agents; ++k) {
        tasks.emplace(k, AgentTask(k, Endpoints(starts[k], goals[k])));
    }
    MAPFProblem problem(std::move(graph), std::move(tasks));

    auto solver = mapf::solver::SolverFactory::Instance().CreateSolver(cfg.solver);
    if (!solver) {
        std::cerr << "Unknown solver: " << cfg.solver << "\n";
        return 1;
    }

    const auto t0 = std::chrono::high_resolution_clock::now();
    const auto solution = solver->FindSolution(problem);
    const auto t1 = std::chrono::high_resolution_clock::now();
    const double comp_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    long long makespan = -1;
    if (!solution.agent_paths.empty()) {
        makespan = static_cast<long long>(solution.agent_paths.begin()->second.path.size()) - 1;
    }
    if (makespan < 0) {
        std::cerr << "WARNING: solver '" << cfg.solver << "' found no solution for this instance\n";
    }

    std::filesystem::path out_path = cfg.out;
    if (out_path.is_relative()) {
        if (const char* workspace = std::getenv("BUILD_WORKSPACE_DIRECTORY")) {
            out_path = std::filesystem::path(workspace) / out_path;
        }
    }
    std::filesystem::create_directories(out_path.parent_path());

    std::ofstream out(out_path);
    if (!out.is_open()) {
        std::cerr << "Failed to open output file: " << out_path << "\n";
        return 1;
    }

    out << "{\n";
    out << "  \"width\": " << cfg.width << ",\n";
    out << "  \"height\": " << cfg.height << ",\n";
    out << "  \"solver\": \"" << cfg.solver << "\",\n";
    out << "  \"makespan\": " << makespan << ",\n";
    out << "  \"comp_time_ms\": " << comp_time_ms << ",\n";

    out << "  \"obstacles\": [";
    bool first = true;
    for (uint32_t y = 0; y < cfg.height; ++y) {
        for (uint32_t x = 0; x < cfg.width; ++x) {
            if (IsObstacle(cfg, x, y)) {
                out << (first ? "" : ",") << "[" << x << "," << y << "]";
                first = false;
            }
        }
    }
    out << "],\n";

    out << "  \"agents\": [\n";
    std::vector<AgentId> ids;
    ids.reserve(problem.agent_tasks.size());
    for (const auto& [id, _] : problem.agent_tasks) {
        ids.push_back(id);
    }
    std::sort(ids.begin(), ids.end());

    for (size_t idx = 0; idx < ids.size(); ++idx) {
        const AgentId id = ids[idx];
        const auto& task = problem.agent_tasks.at(id);
        out << "    {\"id\": " << id << ", \"start\": ";
        WriteXY(out, task.endpoints.from_node_id, cfg.width);
        out << ", \"goal\": ";
        WriteXY(out, task.endpoints.to_node_id, cfg.width);
        out << ", \"path\": [";
        auto path_it = solution.agent_paths.find(id);
        if (path_it != solution.agent_paths.end()) {
            const auto& path = path_it->second.path;
            for (size_t i = 0; i < path.size(); ++i) {
                if (i != 0) {
                    out << ",";
                }
                WriteXY(out, path[i], cfg.width);
            }
        }
        out << "]}" << (idx + 1 < ids.size() ? "," : "") << "\n";
    }
    out << "  ]\n";
    out << "}\n";
    out.close();

    std::cout << "Wrote " << out_path << "  (solver=" << cfg.solver << ", agents=" << cfg.agents
              << ", makespan=" << makespan << ", comp_time=" << comp_time_ms << " ms)\n";
    return 0;
}
