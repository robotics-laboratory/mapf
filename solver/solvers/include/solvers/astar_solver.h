#pragma once

#include "solvers/solver_base.h"

#include "models/models.h"

namespace mapf::solver {

struct AStarSolver : public SolverBase {
    models::MAPFSolution FindSolution(const models::MAPFProblem& mapf_problem) const final;
    size_t GetNodesExpanded() const { return nodes_expanded_; }

  private:
    mutable size_t nodes_expanded_ = 0;
};

}  // namespace mapf::solver

