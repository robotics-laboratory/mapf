#pragma once

#include "models/models.h"

#include <cstddef>

namespace mapf::models {

size_t Makespan(const MAPFSolution& solution);

size_t SumOfCosts(const MAPFSolution& solution);

}  // namespace mapf::models
