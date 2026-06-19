#include "solvers/astar_solver.h"
#include "solvers/bfs_solver.h"
#include "solvers/pibt_solver.h"

#include "graph/graph.h"
#include "models/metrics.h"
#include "models/models.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <numeric>
#include <random>
#include <string>
#include <vector>

using namespace mapf::graph;
using namespace mapf::models;
using namespace mapf::solver;

namespace {

Graph MakeGrid(uint32_t width, uint32_t height) {
    Nodes nodes;
    Edges edges;
    for (uint32_t y = 0; y < height; ++y) {
        for (uint32_t x = 0; x < width; ++x) {
            NodeId id = y * width + x;
            nodes.emplace(id, Node(id));
            if (x + 1 < width) {
                edges.emplace(Edge(id, id + 1));
                edges.emplace(Edge(id + 1, id));
            }
            if (y + 1 < height) {
                edges.emplace(Edge(id, id + width));
                edges.emplace(Edge(id + width, id));
            }
        }
    }
    return Graph(std::move(nodes), std::move(edges));
}

Graph MakeCorridorWithPocket() {
    Nodes nodes;
    Edges edges;
    for (uint32_t i = 0; i < 9; ++i) {
        nodes.emplace(i, Node(i));
    }
    for (uint32_t i = 0; i < 7; ++i) {
        edges.emplace(Edge(i, i + 1));
        edges.emplace(Edge(i + 1, i));
    }
    edges.emplace(Edge(3, 8));
    edges.emplace(Edge(8, 3));
    return Graph(std::move(nodes), std::move(edges));
}

MAPFProblem MakeRandomGridProblem(
    uint32_t width, uint32_t height, uint32_t num_agents, uint32_t seed) {
    auto graph = MakeGrid(width, height);

    std::vector<NodeId> ids(static_cast<size_t>(width) * height);
    std::iota(ids.begin(), ids.end(), 0u);

    std::mt19937 rng(seed);
    std::vector<NodeId> starts = ids;
    std::shuffle(starts.begin(), starts.end(), rng);
    std::vector<NodeId> goals = ids;
    std::shuffle(goals.begin(), goals.end(), rng);

    AgentTasks tasks;
    for (uint32_t k = 0; k < num_agents; ++k) {
        tasks.emplace(k, AgentTask(k, Endpoints(starts[k], goals[k])));
    }
    return MAPFProblem(std::move(graph), std::move(tasks));
}

struct RunResult {
    bool solved = false;
    double time_ms = 0.0;
    long long makespan = -1;
    long long soc = -1;
    long long nodes = -1;
};

long long MakespanOf(const MAPFSolution& solution) {
    if (solution.agent_paths.empty()) {
        return -1;
    }
    return static_cast<long long>(mapf::models::Makespan(solution));
}

long long SocOf(const MAPFSolution& solution) {
    if (solution.agent_paths.empty()) {
        return -1;
    }
    return static_cast<long long>(mapf::models::SumOfCosts(solution));
}

template<typename Solver>
RunResult TimeSolver(Solver& solver, const MAPFProblem& problem) {
    RunResult result;
    auto t0 = std::chrono::high_resolution_clock::now();
    auto solution = solver.FindSolution(problem);
    auto t1 = std::chrono::high_resolution_clock::now();
    result.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    result.solved = !solution.agent_paths.empty();
    result.makespan = MakespanOf(solution);
    result.soc = SocOf(solution);
    return result;
}

RunResult RunBFS(const MAPFProblem& problem) {
    BFSSolver solver;
    auto result = TimeSolver(solver, problem);
    result.nodes = static_cast<long long>(solver.GetNodesExpanded());
    return result;
}

RunResult RunAStar(const MAPFProblem& problem) {
    AStarSolver solver;
    auto result = TimeSolver(solver, problem);
    result.nodes = static_cast<long long>(solver.GetNodesExpanded());
    return result;
}

RunResult RunPIBT(const MAPFProblem& problem) {
    PIBTSolver solver;
    return TimeSolver(solver, problem);
}

std::string Cell(const RunResult& r, bool show_nodes) {
    if (!r.solved) {
        return "  --              ";
    }
    char buf[80];
    if (show_nodes) {
        std::snprintf(buf, sizeof(buf), "%8lld  %9.2f  %4lld  %4lld", r.nodes, r.time_ms,
                      r.makespan, r.soc);
    } else {
        std::snprintf(buf, sizeof(buf), "%9.3f  %4lld  %4lld", r.time_ms, r.makespan, r.soc);
    }
    return buf;
}

}  // namespace

TEST(Experiment, OperatingEnvelope) {
    std::printf("\n");
    std::printf("================================================================================\n");
    std::printf(" TABLE A -- head-to-head on small instances\n");
    std::printf("            (nodes expanded / time ms / makespan / sum-of-costs)\n");
    std::printf("================================================================================\n");
    std::printf("%-22s | %-30s | %-30s | %-22s\n", "instance", "BFS", "A*", "PIBT");
    std::printf("%-22s | %8s  %9s  %4s  %4s | %8s  %9s  %4s  %4s | %9s  %4s  %4s\n",
                "", "nodes", "time", "mksp", "soc", "nodes", "time", "mksp", "soc",
                "time", "mksp", "soc");
    std::printf("--------------------------------------------------------------------------------\n");

    struct SmallCase {
        std::string name;
        MAPFProblem problem;
    };
    std::vector<SmallCase> small_cases;
    small_cases.push_back({"3x3 grid, 2 agents",
                           MAPFProblem(MakeGrid(3, 3), AgentTasks{{0, {0, {0, 8}}}, {1, {1, {8, 0}}}})});
    small_cases.push_back({"corridor+pocket, 2", MAPFProblem(MakeCorridorWithPocket(),
                                                             AgentTasks{{0, {0, {0, 7}}}, {1, {1, {7, 0}}}})});
    small_cases.push_back({"4x4 grid, 3 agents",
                           MAPFProblem(MakeGrid(4, 4),
                                       AgentTasks{{0, {0, {0, 15}}}, {1, {1, {3, 12}}}, {2, {2, {12, 3}}}})});

    for (auto& c : small_cases) {
        auto bfs = RunBFS(c.problem);
        auto astar = RunAStar(c.problem);
        auto pibt = RunPIBT(c.problem);
        std::printf("%-22s | %-30s | %-30s | %-22s\n", c.name.c_str(), Cell(bfs, true).c_str(),
                    Cell(astar, true).c_str(), Cell(pibt, false).c_str());
        if (pibt.solved && astar.solved && astar.makespan > 0) {
            const double mksp_ratio =
                static_cast<double>(pibt.makespan) / static_cast<double>(astar.makespan);
            const double soc_ratio =
                astar.soc > 0
                    ? static_cast<double>(pibt.soc) / static_cast<double>(astar.soc)
                    : 0.0;
            std::printf("%-22s   PIBT/optimal: makespan = %.2fx,  SOC = %.2fx\n", "", mksp_ratio,
                        soc_ratio);
        }
    }

    std::printf("\n");
    std::printf("================================================================================\n");
    std::printf(" TABLE B -- scalability: joint-state space |V|^n vs PIBT (open grids)\n");
    std::printf("            time ms / makespan / sum-of-costs\n");
    std::printf("================================================================================\n");
    std::printf("%-22s | %14s | %-30s | %-24s\n", "instance", "|V|^n (approx)", "A*", "PIBT");
    std::printf("%-22s | %14s | %9s  %4s  %5s  %5s | %9s  %4s  %5s\n",
                "", "", "time", "mksp", "soc", "ok", "time", "mksp", "soc");
    std::printf("--------------------------------------------------------------------------------\n");

    struct ScaleCase {
        std::string name;
        uint32_t w, h, agents;
        bool run_astar;
    };
    std::vector<ScaleCase> scale_cases{
        {"4x4 grid, 4 agents", 4, 4, 4, true},
        {"8x8 grid, 16 agents", 8, 8, 16, false},
        {"8x8 grid, 32 agents", 8, 8, 32, false},
        {"16x16 grid, 50 agents", 16, 16, 50, false},
        {"16x16 grid, 100 agents", 16, 16, 100, false},
        {"32x32 grid, 200 agents", 32, 32, 200, false},
    };

    for (const auto& sc : scale_cases) {
        MAPFProblem problem = (sc.w == 4 && sc.h == 4 && sc.agents == 4)
            ? MAPFProblem(MakeGrid(4, 4), AgentTasks{{0, {0, {0, 15}}}, {1, {1, {15, 0}}},
                                                     {2, {2, {3, 12}}}, {3, {3, {12, 3}}}})
            : MakeRandomGridProblem(sc.w, sc.h, sc.agents, 12345u);

        const double log10_state_space =
            static_cast<double>(sc.agents) * std::log10(static_cast<double>(sc.w) * sc.h);

        std::string astar_cell = "  intractable                 ";
        if (sc.run_astar) {
            auto astar = RunAStar(problem);
            char buf[80];
            std::snprintf(buf, sizeof(buf), "%9.2f  %4lld  %5lld  %5s", astar.time_ms,
                          astar.makespan, astar.soc, astar.solved ? "yes" : "no");
            astar_cell = buf;
        }

        auto pibt = RunPIBT(problem);
        char pibt_cell[80];
        std::snprintf(pibt_cell, sizeof(pibt_cell), "%9.3f  %4lld  %5lld", pibt.time_ms,
                      pibt.makespan, pibt.soc);

        char statespace[32];
        std::snprintf(statespace, sizeof(statespace), "~10^%.0f", log10_state_space);

        std::printf("%-22s | %14s | %-30s | %-24s%s\n", sc.name.c_str(), statespace,
                    astar_cell.c_str(), pibt_cell, pibt.solved ? "" : "  (DNF)");
        EXPECT_TRUE(pibt.solved) << "PIBT should solve open-grid instance: " << sc.name;
    }
    std::printf("================================================================================\n\n");
}

TEST(Experiment, LargeScale) {
    constexpr uint32_t kWidth = 64, kHeight = 64;
    constexpr int kTrials = 5;

    std::printf("\n");
    std::printf("================================================================================\n");
    std::printf(" TABLE C -- PIBT large-scale stress test on a %ux%u open grid (%u cells)\n",
                kWidth, kHeight, kWidth * kHeight);
    std::printf("           %d random instances per row\n", kTrials);
    std::printf("================================================================================\n");
    std::printf("%-8s | %-9s | %-12s | %-12s | %-12s | %-12s\n", "agents", "success",
                "time avg ms", "time max ms", "makespan avg", "soc avg");
    std::printf("--------------------------------------------------------------------------------\n");

    for (uint32_t agents : {100u, 200u, 400u, 700u, 1000u}) {
        int solved = 0;
        double time_sum = 0.0, time_max = 0.0, makespan_sum = 0.0, soc_sum = 0.0;
        for (int s = 0; s < kTrials; ++s) {
            auto problem = MakeRandomGridProblem(kWidth, kHeight, agents, 1000u + s);
            auto r = RunPIBT(problem);
            if (r.solved) {
                ++solved;
                time_sum += r.time_ms;
                time_max = std::max(time_max, r.time_ms);
                makespan_sum += static_cast<double>(r.makespan);
                soc_sum += static_cast<double>(r.soc);
            }
        }
        const double time_avg = solved ? time_sum / solved : 0.0;
        const double makespan_avg = solved ? makespan_sum / solved : 0.0;
        const double soc_avg = solved ? soc_sum / solved : 0.0;
        std::printf("%-8u | %5d / %-2d | %12.1f | %12.1f | %12.1f | %12.1f\n", agents, solved,
                    kTrials, time_avg, time_max, makespan_avg, soc_avg);
        EXPECT_GT(solved, 0) << "PIBT should solve at least one " << agents << "-agent instance";
    }
    std::printf("================================================================================\n\n");
}

// ablation study
namespace {

MAPFProblem DiagonalSwap(uint32_t n) {
    auto graph = MakeGrid(n, n);
    auto id = [&](uint32_t x, uint32_t y) { return y * n + x; };
    AgentTasks tasks;
    tasks.emplace(0, AgentTask(0, Endpoints(id(0, 0), id(n - 1, n - 1))));
    tasks.emplace(1, AgentTask(1, Endpoints(id(n - 1, n - 1), id(0, 0))));
    tasks.emplace(2, AgentTask(2, Endpoints(id(n - 1, 0), id(0, n - 1))));
    tasks.emplace(3, AgentTask(3, Endpoints(id(0, n - 1), id(n - 1, 0))));
    return MAPFProblem(std::move(graph), std::move(tasks));
}

MAPFProblem ColumnSwap(uint32_t n) {
    auto graph = MakeGrid(n, n);
    auto id = [&](uint32_t x, uint32_t y) { return y * n + x; };
    AgentTasks tasks;
    uint32_t a = 0;
    for (uint32_t y = 0; y < n; ++y) {
        tasks.emplace(a, AgentTask(a, Endpoints(id(0, y), id(n - 1, y))));
        ++a;
        tasks.emplace(a, AgentTask(a, Endpoints(id(n - 1, y), id(0, y))));
        ++a;
    }
    return MAPFProblem(std::move(graph), std::move(tasks));
}

std::vector<long long> RunConfigDetailed(const PIBTConfig& cfg,
                                         const std::vector<MAPFProblem>& problems) {
    std::vector<long long> makespans;
    makespans.reserve(problems.size());
    for (const auto& problem : problems) {
        PIBTSolver solver(cfg);
        const auto solution = solver.FindSolution(problem);
        makespans.push_back(
            solution.agent_paths.empty()
                ? -1
                : static_cast<long long>(solution.agent_paths.begin()->second.path.size() - 1));
    }
    return makespans;
}

}  // namespace

TEST(Experiment, Ablation) {
    std::vector<MAPFProblem> problems;
    problems.push_back(DiagonalSwap(4));
    problems.push_back(DiagonalSwap(6));
    problems.push_back(DiagonalSwap(8));
    problems.push_back(ColumnSwap(5));
    problems.push_back(ColumnSwap(6));
    problems.push_back(ColumnSwap(8));
    const size_t n_sym = problems.size();

    for (uint32_t s = 1; s <= 15; ++s) {
        problems.push_back(MakeRandomGridProblem(8, 8, 16, s));
    }
    for (uint32_t s = 1; s <= 15; ++s) {
        problems.push_back(MakeRandomGridProblem(10, 10, 25, 100 + s));
    }
    for (uint32_t s = 1; s <= 10; ++s) {
        problems.push_back(MakeRandomGridProblem(12, 12, 30, 200 + s));
    }
    const size_t n_total = problems.size();
    const size_t n_rand = n_total - n_sym;

    using TB = PIBTConfig::TieBreak;
    using PM = PIBTConfig::PriorityMode;
    struct Row {
        const char* name;
        PIBTConfig cfg;
    };
    const std::vector<Row> rows = {
        {"hash      + dynamic (ours)", {TB::Hash, PM::Dynamic, 0xC0FFEEu}},
        {"hash      + static",        {TB::Hash, PM::Static, 0xC0FFEEu}},
        {"random    + dynamic",       {TB::Random, PM::Dynamic, 0xC0FFEEu}},
        {"random    + static",        {TB::Random, PM::Static, 0xC0FFEEu}},
        {"lowest-id + dynamic",       {TB::LowestId, PM::Dynamic, 0xC0FFEEu}},
        {"lowest-id + static",        {TB::LowestId, PM::Static, 0xC0FFEEu}},
    };

    std::vector<std::vector<long long>> results;
    for (const auto& row : rows) {
        results.push_back(RunConfigDetailed(row.cfg, problems));
    }

    auto avg_makespan_all = [&](size_t c) {
        double sum = 0.0;
        for (long long m : results[c]) {
            sum += static_cast<double>(m);
        }
        return sum / static_cast<double>(results[c].size());
    };

    auto avg_makespan_solved = [&](size_t c) -> std::pair<double, int> {
        double sum = 0.0;
        int solved = 0;
        for (long long m : results[c]) {
            if (m >= 0) {
                sum += static_cast<double>(m);
                ++solved;
            }
        }
        if (solved == 0) {
            return {0.0, 0};
        }
        return {sum / static_cast<double>(solved), solved};
    };

    std::printf("\n");
    std::printf("================================================================================\n");
    std::printf(" ABLATION -- PIBT tie-break x priority\n");
    std::printf("   %zu symmetric stressors + %zu random instances = %zu total\n", n_sym, n_rand,
                n_total);
    std::printf("================================================================================\n");
    std::printf("%-28s | %-15s | %-15s | %-15s | %s\n", "configuration", "symmetric",
                "random", "overall", "avg mksp (N)");
    std::printf("--------------------------------------------------------------------------------\n");
    for (size_t c = 0; c < rows.size(); ++c) {
        int sym = 0, rnd = 0;
        for (size_t i = 0; i < n_total; ++i) {
            const bool solved = results[c][i] >= 0;
            if (i < n_sym) {
                sym += solved;
            } else {
                rnd += solved;
            }
        }
        const int overall = sym + rnd;
        const double sym_pct = 100.0 * sym / static_cast<double>(n_sym);
        const double rnd_pct = 100.0 * rnd / static_cast<double>(n_rand);
        const double all_pct = 100.0 * overall / static_cast<double>(n_total);
        const auto [avg_mksp, n_solved] = avg_makespan_solved(c);
        std::printf("%-28s | %2d/%-2zu (%5.1f%%) | %2d/%-2zu (%5.1f%%) | %2d/%-2zu (%5.1f%%) | %5.2f (N=%d)\n",
                    rows[c].name, sym, n_sym, sym_pct, rnd, n_rand, rnd_pct, overall, n_total,
                    all_pct, avg_mksp, n_solved);
    }
    std::printf("================================================================================\n");
    std::printf(" Same-instance comparison over all %zu instances\n", n_total);
    std::printf("   (only the two configurations that solve every instance):\n");
    std::printf("   hash+dynamic (ours) = %.2f   vs   random+dynamic = %.2f\n", avg_makespan_all(0),
                avg_makespan_all(2));
    std::printf("================================================================================\n\n");

    for (size_t i = 0; i < n_total; ++i) {
        EXPECT_GE(results[0][i], 0) << "ours (hash+dynamic) failed instance " << i;
    }
}

// pibt vs a*

namespace {

struct ZoneSpec {
    const char* name;
    uint32_t w;
    uint32_t h;
    uint32_t agents;
    uint32_t seeds;
};

std::filesystem::path ResolveOutputPath(const std::filesystem::path& relative) {
    if (relative.is_absolute()) {
        return relative;
    }
    if (const char* workspace = std::getenv("BUILD_WORKSPACE_DIRECTORY")) {
        return std::filesystem::path(workspace) / relative;
    }
    if (const char* test_undeclared = std::getenv("TEST_UNDECLARED_OUTPUTS_DIR")) {
        return std::filesystem::path(test_undeclared) / relative.filename();
    }
    return relative;
}

}  // namespace

TEST(Experiment, Suboptimality) {
    const std::vector<ZoneSpec> zones = {
        {"3x3_2agents", 3, 3, 2, 50},
        {"4x4_2agents", 4, 4, 2, 50},
        {"4x4_3agents", 4, 4, 3, 50},
    };

    const std::filesystem::path out_path = ResolveOutputPath("viz/data/suboptimality.csv");
    std::filesystem::create_directories(out_path.parent_path());
    std::ofstream csv(out_path);
    ASSERT_TRUE(csv.is_open()) << "could not open " << out_path;

    csv << "zone,width,height,agents,seed,"
           "astar_nodes,astar_time_ms,astar_makespan,astar_soc,"
           "pibt_time_ms,pibt_makespan,pibt_soc,"
           "makespan_ratio,soc_ratio\n";

    struct ZoneStats {
        std::vector<double> makespan_ratio;
        std::vector<double> soc_ratio;
        std::vector<double> astar_time;
        std::vector<double> pibt_time;
    };
    std::vector<ZoneStats> stats(zones.size());

    int total_instances = 0;
    int pibt_solved = 0;

    for (size_t zi = 0; zi < zones.size(); ++zi) {
        const auto& z = zones[zi];
        for (uint32_t s = 1; s <= z.seeds; ++s) {
            auto problem = MakeRandomGridProblem(z.w, z.h, z.agents, s);
            auto astar = RunAStar(problem);
            auto pibt = RunPIBT(problem);

            ++total_instances;
            if (pibt.solved) {
                ++pibt_solved;
            }
            ASSERT_TRUE(astar.solved) << "A* failed on " << z.name << " seed=" << s;
            ASSERT_TRUE(pibt.solved) << "PIBT failed on " << z.name << " seed=" << s;

            const double mksp_ratio = astar.makespan > 0
                ? static_cast<double>(pibt.makespan) / static_cast<double>(astar.makespan)
                : 1.0;
            const double soc_ratio = astar.soc > 0
                ? static_cast<double>(pibt.soc) / static_cast<double>(astar.soc)
                : 1.0;

            stats[zi].makespan_ratio.push_back(mksp_ratio);
            stats[zi].soc_ratio.push_back(soc_ratio);
            stats[zi].astar_time.push_back(astar.time_ms);
            stats[zi].pibt_time.push_back(pibt.time_ms);

            csv << z.name << "," << z.w << "," << z.h << "," << z.agents << "," << s << ","
                << astar.nodes << "," << astar.time_ms << "," << astar.makespan << ","
                << astar.soc << "," << pibt.time_ms << "," << pibt.makespan << "," << pibt.soc
                << "," << mksp_ratio << "," << soc_ratio << "\n";
        }
    }
    csv.close();

    auto percentile = [](std::vector<double> xs, double q) {
        if (xs.empty()) return 0.0;
        std::sort(xs.begin(), xs.end());
        const double pos = q * (static_cast<double>(xs.size()) - 1.0);
        const size_t lo = static_cast<size_t>(pos);
        const size_t hi = std::min(lo + 1, xs.size() - 1);
        const double frac = pos - static_cast<double>(lo);
        return xs[lo] * (1.0 - frac) + xs[hi] * frac;
    };
    auto mean = [](const std::vector<double>& xs) {
        if (xs.empty()) return 0.0;
        double s = 0;
        for (double x : xs) s += x;
        return s / static_cast<double>(xs.size());
    };

    std::printf("\n");
    std::printf("================================================================================\n");
    std::printf(" TABLE D -- statistical PIBT vs A* (open grids, 50 random instances per zone)\n");
    std::printf("================================================================================\n");
    std::printf("CSV written to: %s\n", out_path.string().c_str());
    std::printf("--------------------------------------------------------------------------------\n");
    std::printf("%-13s | %s\n", "zone",
                " makespan ratio (PIBT/A*)        |  SOC ratio                       | mean t ms");
    std::printf("%-13s | %s\n", "",
                "  median   p75    max    mean   |  median   p75    max    mean    | A*       PIBT");
    std::printf("--------------------------------------------------------------------------------\n");
    for (size_t zi = 0; zi < zones.size(); ++zi) {
        const auto& s = stats[zi];
        std::printf("%-13s |  %5.3f  %5.3f  %5.3f  %5.3f  |  %5.3f  %5.3f  %5.3f  %5.3f  |"
                    " %7.2f %7.3f\n",
                    zones[zi].name,
                    percentile(s.makespan_ratio, 0.5), percentile(s.makespan_ratio, 0.75),
                    *std::max_element(s.makespan_ratio.begin(), s.makespan_ratio.end()),
                    mean(s.makespan_ratio),
                    percentile(s.soc_ratio, 0.5), percentile(s.soc_ratio, 0.75),
                    *std::max_element(s.soc_ratio.begin(), s.soc_ratio.end()),
                    mean(s.soc_ratio),
                    mean(s.astar_time), mean(s.pibt_time));
    }
    std::printf("================================================================================\n\n");

    EXPECT_EQ(pibt_solved, total_instances)
        << "PIBT should solve every open-grid instance in these zones";
}
