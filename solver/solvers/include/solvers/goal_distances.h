#pragma once

#include "graph/graph.h"
#include "models/models.h"

#include <cstdint>
#include <unordered_map>

namespace mapf::solver {

using AgentGoalDistances =
    std::unordered_map<models::AgentId, std::unordered_map<graph::NodeId, uint64_t>>;

AgentGoalDistances BuildGoalDistances(const models::MAPFProblem& mapf_problem);

}  // namespace mapf::solver
