#include "solvers/goal_distances.h"

#include <queue>
#include <vector>

namespace mapf::solver {

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

}  // namespace mapf::solver
