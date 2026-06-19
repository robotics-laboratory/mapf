#include "solvers/pibt_solver.h"

#include "solvers/goal_distances.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <optional>
#include <random>
#include <unordered_map>
#include <vector>

namespace mapf::solver {

namespace {

using models::AgentId;
using graph::NodeId;

constexpr uint64_t kUnreachable = std::numeric_limits<uint64_t>::max();

uint64_t MixHash(uint64_t value, uint64_t step) {
    uint64_t x = value * 0x9E3779B97F4A7C15ull + step * 0xD1B54A32D192ED03ull + 0x165667B19E3779F9ull;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
    return x ^ (x >> 31);
}

std::unordered_map<AgentId, double> DefaultInitialOffsets(
    const models::MAPFProblem& mapf_problem, const AgentGoalDistances& goal_distances) {
    struct AgentKey {
        AgentId agent_id;
        uint64_t start_distance;
    };

    std::vector<AgentKey> agents;
    agents.reserve(mapf_problem.agent_tasks.size());
    for (const auto& [agent_id, agent_task] : mapf_problem.agent_tasks) {
        uint64_t start_distance = 0;
        if (auto dist_it = goal_distances.find(agent_id); dist_it != goal_distances.end()) {
            const auto& node_distances = dist_it->second;
            auto node_it = node_distances.find(agent_task.endpoints.from_node_id);
            start_distance = node_it != node_distances.end() ? node_it->second : 0;
        }
        agents.push_back(AgentKey{agent_id, start_distance});
    }

    std::sort(agents.begin(), agents.end(), [](const AgentKey& lhs, const AgentKey& rhs) {
        if (lhs.start_distance != rhs.start_distance) {
            return lhs.start_distance > rhs.start_distance;
        }
        return lhs.agent_id < rhs.agent_id;
    });

    std::unordered_map<AgentId, double> offsets;
    offsets.reserve(agents.size());
    const double denominator = static_cast<double>(agents.size() + 1);
    for (size_t index = 0; index < agents.size(); ++index) {
        offsets[agents[index].agent_id] =
            static_cast<double>(agents.size() - index) / denominator;
    }
    return offsets;
}

struct PibtRun {
    const models::MAPFProblem& problem;
    const AgentGoalDistances& dist;

    std::unordered_map<AgentId, NodeId> current;
    std::unordered_map<AgentId, NodeId> next;
    std::unordered_map<NodeId, AgentId> occupant_now;
    std::unordered_map<NodeId, AgentId> reserved;

    size_t step = 0;
    PIBTConfig config;
    std::mt19937 rng;

    uint64_t DistanceToGoal(AgentId agent_id, NodeId node_id) const {
        auto dist_it = dist.find(agent_id);
        if (dist_it == dist.end()) {
            return kUnreachable;
        }
        auto node_it = dist_it->second.find(node_id);
        return node_it != dist_it->second.end() ? node_it->second : kUnreachable;
    }

    bool Decide(AgentId agent_id, std::optional<AgentId> pusher) {
        const NodeId from_node = current.at(agent_id);

        std::vector<NodeId> candidates;
        const auto& neighbours = problem.graph.GetNeighbours(from_node);
        candidates.reserve(neighbours.size() + 1);
        for (const NodeId neighbour : neighbours) {
            candidates.push_back(neighbour);
        }
        candidates.push_back(from_node);

        if (config.tie_break == PIBTConfig::TieBreak::Random) {
            std::shuffle(candidates.begin(), candidates.end(), rng);
            std::stable_sort(candidates.begin(), candidates.end(), [&](NodeId lhs, NodeId rhs) {
                return DistanceToGoal(agent_id, lhs) < DistanceToGoal(agent_id, rhs);
            });
        } else {
            std::sort(candidates.begin(), candidates.end(), [&](NodeId lhs, NodeId rhs) {
                const uint64_t lhs_dist = DistanceToGoal(agent_id, lhs);
                const uint64_t rhs_dist = DistanceToGoal(agent_id, rhs);
                if (lhs_dist != rhs_dist) {
                    return lhs_dist < rhs_dist;
                }
                if (config.tie_break == PIBTConfig::TieBreak::LowestId) {
                    return lhs < rhs;
                }
                return MixHash(lhs, step) < MixHash(rhs, step);
            });
        }

        for (const NodeId candidate : candidates) {
            if (reserved.contains(candidate)) {
                continue;
            }
            if (pusher.has_value() && candidate == current.at(*pusher)) {
                continue;
            }

            reserved.emplace(candidate, agent_id);
            next.emplace(agent_id, candidate);

            auto occupant_it = occupant_now.find(candidate);
            if (occupant_it != occupant_now.end()) {
                const AgentId blocker = occupant_it->second;
                if (blocker != agent_id && !next.contains(blocker)) {

                    if (Decide(blocker, agent_id)) {
                        return true;
                    }

                    reserved.erase(candidate);
                    next.erase(agent_id);
                    continue;
                }
            }
            return true;
        }
        return false;
    }
};

}  

models::MAPFSolution PIBTSolver::FindSolution(const models::MAPFProblem& mapf_problem) const {
    makespan_ = 0;

    const auto goal_distances = BuildGoalDistances(mapf_problem);

    PibtRun run{mapf_problem, goal_distances, {}, {}, {}, {}};
    run.config = config_;
    run.rng.seed(config_.seed);

    std::unordered_map<AgentId, NodeId> goal;
    std::unordered_map<AgentId, double> priority;
    std::unordered_map<AgentId, graph::NodeIdsList> paths;

    const auto offsets = DefaultInitialOffsets(mapf_problem, goal_distances);

    const size_t agent_count = mapf_problem.agent_tasks.size();
    run.current.reserve(agent_count);
    goal.reserve(agent_count);
    priority.reserve(agent_count);
    paths.reserve(agent_count);

    for (const auto& [agent_id, agent_task] : mapf_problem.agent_tasks) {
        const NodeId start_node = agent_task.endpoints.from_node_id;
        run.current.emplace(agent_id, start_node);
        goal.emplace(agent_id, agent_task.endpoints.to_node_id);
        priority.emplace(agent_id, offsets.at(agent_id));
        paths.emplace(agent_id, graph::NodeIdsList{start_node});
    }

    const size_t max_timesteps = 64 * std::max<size_t>(mapf_problem.graph.nodes.size(), 1);

    std::vector<AgentId> order;
    order.reserve(agent_count);
    for (const auto& [agent_id, _] : mapf_problem.agent_tasks) {
        order.push_back(agent_id);
    }

    bool solved = false;
    for (size_t step = 0; step < max_timesteps; ++step) {
        bool all_at_goal = true;
        for (const auto& [agent_id, node_id] : run.current) {
            if (node_id != goal.at(agent_id)) {
                all_at_goal = false;
                break;
            }
        }
        if (all_at_goal) {
            solved = true;
            break;
        }

        run.step = step;
        run.next.clear();
        run.reserved.clear();
        run.occupant_now.clear();
        run.occupant_now.reserve(agent_count);
        for (const auto& [agent_id, node_id] : run.current) {
            run.occupant_now.emplace(node_id, agent_id);
        }

        std::sort(order.begin(), order.end(), [&](AgentId lhs, AgentId rhs) {
            return priority.at(lhs) > priority.at(rhs);
        });

        for (const AgentId agent_id : order) {
            if (!run.next.contains(agent_id)) {
                run.Decide(agent_id, std::nullopt);
            }
        }

        for (const auto& [agent_id, next_node] : run.next) {
            run.current[agent_id] = next_node;
            paths.at(agent_id).push_back(next_node);
        }

        if (config_.priority == PIBTConfig::PriorityMode::Dynamic) {
            for (auto& [agent_id, value] : priority) {
                if (run.current.at(agent_id) == goal.at(agent_id)) {
                    value = offsets.at(agent_id);
                } else {
                    value += 1.0;
                }
            }
        }
    }

    if (!solved) {
        return {};
    }

    models::MAPFSolution solution;
    solution.agent_paths.reserve(paths.size());
    for (auto& [agent_id, path] : paths) {
        makespan_ = std::max(makespan_, path.size() - 1);
        solution.agent_paths.emplace(agent_id, models::AgentPath(agent_id, std::move(path)));
    }
    return solution;
}

}  // namespace mapf::solver
