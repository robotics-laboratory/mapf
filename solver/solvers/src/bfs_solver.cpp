#include "solvers/bfs_solver.h"

#include <queue>
#include <unordered_set>
#include <unordered_map>

namespace mapf::solver {

std::vector<std::unordered_map<models::AgentId, graph::NodeId>> GenerateJointMoves(
    const std::unordered_map<models::AgentId, graph::NodeIdsSet>& individual_moves) {
    std::vector<std::unordered_map<models::AgentId, graph::NodeId>> joint_moves;
    joint_moves.push_back({});

    for (const auto& [agent_id, moves] : individual_moves) {
        std::vector<std::unordered_map<models::AgentId, graph::NodeId>> temp_joint_moves;
        for (const auto& joint_move : joint_moves) {
            for (auto move : moves) {
                auto new_joint_move = joint_move;
                new_joint_move.emplace(agent_id, std::move(move));
                temp_joint_moves.emplace_back(std::move(new_joint_move));
            }
        }
        joint_moves = std::move(temp_joint_moves);
    }

    return joint_moves;
}

bool HasCollision(
    const models::AgentStates& prev_positions, const models::AgentStates& current_positions) {
    std::unordered_set<graph::NodeId> occupied_positions;

    for (const auto& [agent_id, agent_state] : current_positions) {
        if (occupied_positions.contains(agent_state.node_id)) {
            return true;
        }
        occupied_positions.insert(agent_state.node_id);
    }

    for (const auto& [agent_id_i, _] : prev_positions) {
        for (const auto& [agent_id_j, _] : prev_positions) {
            if (agent_id_i == agent_id_j) {
                continue;
            }
            if (prev_positions.at(agent_id_i).node_id == current_positions.at(agent_id_j).node_id
                && prev_positions.at(agent_id_j).node_id
                       == current_positions.at(agent_id_i).node_id) {
                return true;
            }
        }
    }

    return false;
}

models::MAPFSolution BFSSolver::FindSolution(const models::MAPFProblem& mapf_problem) const {
    auto start_state = models::AgentStates();
    for (const auto& [agent_id, agent_task] : mapf_problem.agent_tasks) {
        start_state.emplace(
            agent_id, models::AgentState(agent_id, agent_task.endpoints.from_node_id));
    }

    auto finish_state = models::AgentStates();
    for (const auto& [agent_id, agent_task] : mapf_problem.agent_tasks) {
        finish_state.emplace(
            agent_id, models::AgentState(agent_id, agent_task.endpoints.to_node_id));
    }

    std::unordered_map<models::AgentStates, uint64_t> cost_to_come;
    std::unordered_map<models::AgentStates, models::AgentStates> parent;

    std::queue<models::AgentStates> open_set;
    open_set.push(start_state);
    cost_to_come[start_state] = 0;

    nodes_expanded_ = 0;

    while (!open_set.empty()) {
        auto current_state = open_set.front();
        open_set.pop();
        ++nodes_expanded_;

        if (current_state == finish_state) {
            std::unordered_map<models::AgentId, graph::NodeIdsList> paths;
            for (const auto& [agent_id, agent_task] : mapf_problem.agent_tasks) {
                paths[agent_id] = {current_state[agent_id].node_id};
            }

            auto parent_it = parent.find(current_state);
            while (parent_it != parent.end()) {
                auto prev_state = parent_it->second;
                for (const auto& [agent_id, agent_task] : mapf_problem.agent_tasks) {
                    paths[agent_id].emplace_back(prev_state[agent_id].node_id);
                }
                current_state = prev_state;
                parent_it = parent.find(current_state);
            }

            for (const auto& [agent_id, agent_task] : mapf_problem.agent_tasks) {
                auto& path = paths[agent_id];
                std::reverse(path.begin(), path.end());
            }

            models::MAPFSolution solution;
            for (const auto& [agent_id, agent_path] : paths) {
                solution.agent_paths.emplace(agent_id, models::AgentPath(agent_id, agent_path));
            }

            return solution;
        }

        std::unordered_map<models::AgentId, graph::NodeIdsSet> individual_moves;
        for (const auto& [agent_id, _] : mapf_problem.agent_tasks) {
            graph::NodeId current_node = current_state[agent_id].node_id;
            const auto& neighbors = mapf_problem.graph.GetNeighbours(current_node);

            individual_moves[agent_id] = neighbors;
            individual_moves[agent_id].emplace(current_node);
        }

        auto joint_moves = GenerateJointMoves(individual_moves);

        for (auto&& joint_move : joint_moves) {
            models::AgentStates next_state;
            next_state.reserve(joint_move.size());
            for (auto&& [agent_id, node_id] : joint_move) {
                next_state.emplace(
                    agent_id, models::AgentState(std::move(agent_id), std::move(node_id)));
            }

            if (HasCollision(current_state, next_state)) {
                continue;
            }

            if (cost_to_come.contains(next_state)) {
                continue;
            }

            open_set.push(next_state);
            cost_to_come[next_state] = cost_to_come[current_state] + 1;
            parent[next_state] = current_state;
        }
    }

    return {};
}

}  // namespace mapf::solver