#include "models/metrics.h"

#include <cstddef>

namespace mapf::models {

size_t Makespan(const MAPFSolution& solution) {
    if (solution.agent_paths.empty()) {
        return 0;
    }
    const size_t path_len = solution.agent_paths.begin()->second.path.size();
    return path_len == 0 ? 0 : path_len - 1;
}

size_t SumOfCosts(const MAPFSolution& solution) {
    size_t total = 0;
    for (const auto& [_, agent_path] : solution.agent_paths) {
        const auto& path = agent_path.path;
        if (path.empty()) {
            continue;
        }
        const auto goal = path.back();
        size_t last_move = 0;
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            if (path[i] != goal) {
                last_move = i + 1;
            }
        }
        total += last_move;
    }
    return total;
}

}  // namespace mapf::models
