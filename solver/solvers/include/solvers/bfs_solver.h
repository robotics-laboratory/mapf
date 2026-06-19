#pragma once

#include "solvers/solver_base.h"

#include "graph/graph.h"
#include "models/models.h"

#include <vector>

namespace mapf::solver {

std::vector<std::unordered_map<models::AgentId, graph::NodeId>> GenerateJointMoves(
    const std::unordered_map<models::AgentId, graph::NodeIdsSet>& individual_moves);

bool HasCollision(
    const models::AgentStates& prev_positions, const models::AgentStates& current_positions);

struct BFSSolver : public SolverBase {
    models::MAPFSolution FindSolution(const models::MAPFProblem& mapf_problem) const final;
    size_t GetNodesExpanded() const { return nodes_expanded_; }

  private:
    mutable size_t nodes_expanded_ = 0;
};

}  // namespace mapf::solver
