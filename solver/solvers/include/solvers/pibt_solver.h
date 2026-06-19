#pragma once

#include "solvers/solver_base.h"

#include "models/models.h"

#include <cstdint>

namespace mapf::solver {

struct PIBTConfig {
    enum class TieBreak {
        Hash,
        LowestId,
        Random,
    };
    enum class PriorityMode {
        Dynamic,
        Static,
    };

    TieBreak tie_break = TieBreak::Hash;
    PriorityMode priority = PriorityMode::Dynamic;
    uint64_t seed = 0xC0FFEEu;
};

struct PIBTSolver : public SolverBase {
    PIBTSolver() = default;
    explicit PIBTSolver(PIBTConfig config) : config_(config) {}

    models::MAPFSolution FindSolution(const models::MAPFProblem& mapf_problem) const final;

    size_t GetMakespan() const { return makespan_; }

  private:
    PIBTConfig config_;
    mutable size_t makespan_ = 0;
};

}  // namespace mapf::solver
