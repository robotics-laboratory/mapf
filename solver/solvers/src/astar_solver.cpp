#include "solvers/astar_solver.h"

#include "solvers/bfs_solver.h"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

namespace mapf::solver {

namespace {

using AgentGoalDistances = std::unordered_map<models::AgentId, std::unordered_map<graph::NodeId, uint64_t>>;

AgentGoalDistances BuildGoalDistances(const models::MAPFProblem& mapf_problem) {
    std::unordered_map<graph::NodeId, std::vector<graph::NodeId>> reverse_adjacency;
    reverse_adjacency.reserve(mapf_problem.graph.nodes.size());
    for (const auto& edge : mapf_problem.graph.edges) {
        reverse_adjacency[edge.to_node_id].emplace_back(edge.from_node_id);
    }

    AgentGoalDistances goal_distances;
    goal_distances.reserve(mapf_problem.agent_tasks.size());

    for (const auto& [agent_id, agent_task] : mapf_problem.agent_tasks) {
        std::unordered_map<graph::NodeId, uint64_t> distances;
        distances.reserve(mapf_problem.graph.nodes.size());

        std::queue<graph::NodeId> bfs_queue;
        const graph::NodeId goal_node = agent_task.endpoints.to_node_id;
        distances[goal_node] = 0;
        bfs_queue.push(goal_node);

        while (!bfs_queue.empty()) {
            const graph::NodeId current_node = bfs_queue.front();
            bfs_queue.pop();
            const uint64_t current_dist = distances[current_node];

            auto reverse_it = reverse_adjacency.find(current_node);
            if (reverse_it == reverse_adjacency.end()) {
                continue;
            }

            for (const graph::NodeId prev_node : reverse_it->second) {
                if (distances.contains(prev_node)) {
                    continue;
                }
                distances[prev_node] = current_dist + 1;
                bfs_queue.push(prev_node);
            }
        }

        goal_distances.emplace(agent_id, std::move(distances));
    }

    return goal_distances;
}

uint64_t Heuristic(
    const models::AgentStates& state, const AgentGoalDistances& goal_distances) {
    uint64_t lower_bound = 0;

    for (const auto& [agent_id, agent_state] : state) {
        auto distances_it = goal_distances.find(agent_id);
        if (distances_it == goal_distances.end()) {
            return std::numeric_limits<uint64_t>::max();
        }

        auto agent_dist_it = distances_it->second.find(agent_state.node_id);
        if (agent_dist_it == distances_it->second.end()) {
            return std::numeric_limits<uint64_t>::max();
        }

        lower_bound = std::max(lower_bound, agent_dist_it->second);
    }

    return lower_bound;
}

models::MAPFSolution BuildSolutionFromParents(
    const models::MAPFProblem& mapf_problem,
    models::AgentStates current_state,
    const std::unordered_map<models::AgentStates, models::AgentStates>& parent) {
    std::unordered_map<models::AgentId, graph::NodeIdsList> paths;
    paths.reserve(mapf_problem.agent_tasks.size());
    for (const auto& [agent_id, _] : mapf_problem.agent_tasks) {
        paths[agent_id] = {current_state[agent_id].node_id};
    }

    auto parent_it = parent.find(current_state);
    while (parent_it != parent.end()) {
        const auto& prev_state = parent_it->second;
        for (const auto& [agent_id, _] : mapf_problem.agent_tasks) {
            paths[agent_id].emplace_back(prev_state.at(agent_id).node_id);
        }
        current_state = prev_state;
        parent_it = parent.find(current_state);
    }

    models::MAPFSolution solution;
    solution.agent_paths.reserve(paths.size());
    for (auto& [agent_id, path] : paths) {
        std::reverse(path.begin(), path.end());
        solution.agent_paths.emplace(agent_id, models::AgentPath(agent_id, std::move(path)));
    }
    return solution;
}

}  // namespace

models::MAPFSolution AStarSolver::FindSolution(const models::MAPFProblem& mapf_problem) const {
    models::AgentStates start_state;
    start_state.reserve(mapf_problem.agent_tasks.size());
    for (const auto& [agent_id, agent_task] : mapf_problem.agent_tasks) {
        start_state.emplace(
            agent_id, models::AgentState(agent_id, agent_task.endpoints.from_node_id));
    }

    models::AgentStates finish_state;
    finish_state.reserve(mapf_problem.agent_tasks.size());
    for (const auto& [agent_id, agent_task] : mapf_problem.agent_tasks) {
        finish_state.emplace(
            agent_id, models::AgentState(agent_id, agent_task.endpoints.to_node_id));
    }

    const auto goal_distances = BuildGoalDistances(mapf_problem);

    struct OpenNode {
        uint64_t f_score;
        uint64_t g_score;
        models::AgentStates state;
    };

    auto cmp = [](const OpenNode& lhs, const OpenNode& rhs) {
        if (lhs.f_score != rhs.f_score) {
            return lhs.f_score > rhs.f_score;
        }
        return lhs.g_score > rhs.g_score;
    };
    std::priority_queue<OpenNode, std::vector<OpenNode>, decltype(cmp)> open_set(cmp);

    std::unordered_map<models::AgentStates, uint64_t> cost_to_come;
    std::unordered_map<models::AgentStates, models::AgentStates> parent;

    const uint64_t start_h = Heuristic(start_state, goal_distances);
    if (start_h == std::numeric_limits<uint64_t>::max()) {
        return {};
    }
    open_set.push(OpenNode{start_h, 0, start_state});
    cost_to_come[start_state] = 0;

    nodes_expanded_ = 0;

    while (!open_set.empty()) {
        auto current = std::move(open_set.top());
        open_set.pop();
        ++nodes_expanded_;

        auto current_cost_it = cost_to_come.find(current.state);
        if (current_cost_it == cost_to_come.end() || current.g_score != current_cost_it->second) {
            continue;
        }

        if (current.state == finish_state) {
            return BuildSolutionFromParents(mapf_problem, std::move(current.state), parent);
        }

        std::unordered_map<models::AgentId, graph::NodeIdsSet> individual_moves;
        individual_moves.reserve(mapf_problem.agent_tasks.size());
        for (const auto& [agent_id, _] : mapf_problem.agent_tasks) {
            const graph::NodeId current_node = current.state.at(agent_id).node_id;
            const auto& neighbors = mapf_problem.graph.GetNeighbours(current_node);
            individual_moves[agent_id] = neighbors;
            individual_moves[agent_id].emplace(current_node);
        }

        const auto joint_moves = GenerateJointMoves(individual_moves);
        for (const auto& joint_move : joint_moves) {
            models::AgentStates next_state;
            next_state.reserve(joint_move.size());
            for (const auto& [agent_id, node_id] : joint_move) {
                next_state.emplace(agent_id, models::AgentState(agent_id, node_id));
            }

            if (HasCollision(current.state, next_state)) {
                continue;
            }

            const uint64_t tentative_g = current.g_score + 1;
            auto existing_cost_it = cost_to_come.find(next_state);
            if (existing_cost_it != cost_to_come.end() && tentative_g >= existing_cost_it->second) {
                continue;
            }

            const uint64_t h = Heuristic(next_state, goal_distances);
            if (h == std::numeric_limits<uint64_t>::max()) {
                continue;
            }

            cost_to_come[next_state] = tentative_g;
            parent[next_state] = current.state;
            open_set.push(OpenNode{tentative_g + h, tentative_g, std::move(next_state)});
        }
    }

    return {};
}

}  // namespace mapf::solver

