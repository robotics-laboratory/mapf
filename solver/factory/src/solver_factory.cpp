#include "factory/solver_factory.h"

#include "solvers/astar_solver.h"
#include "solvers/bfs_solver.h"
#include "solvers/pibt_solver.h"

#include <memory>

namespace mapf::solver {

SolverFactory::SolverFactory() {
#define REGISTER(NAME, SOLVER_CLS, ...) \
    factory_map_[#NAME] = []() { return std::make_shared<SOLVER_CLS>(__VA_ARGS__); }

    REGISTER(bfs_solver, BFSSolver);
    REGISTER(astar_solver, AStarSolver);
    REGISTER(pibt_solver, PIBTSolver);

#undef REGISTER
}

SolverFactory& SolverFactory::Instance() {
    static SolverFactory instance;
    return instance;
}

SolverPtr SolverFactory::CreateSolver(const std::string& name) const {
    if (auto it = factory_map_.find(name); it != factory_map_.end()) {
        return (it->second)();
    }
    return nullptr;
}

SolverFactory::SolverFactoryMap::const_iterator SolverFactory::find(const std::string& name) const {
    return factory_map_.find(name);
}

SolverFactory::SolverFactoryMap::const_iterator SolverFactory::cbegin() const {
    return factory_map_.cbegin();
}

SolverFactory::SolverFactoryMap::const_iterator SolverFactory::cend() const {
    return factory_map_.cend();
}

}  // namespace mapf::solver
