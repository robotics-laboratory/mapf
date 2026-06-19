#include "models/models.h"
#include "factory/solver_factory.h"

#include <boost/program_options.hpp>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <memory>

namespace po = boost::program_options;

struct ProgramOptions {
    static ProgramOptions Parse(int argc, char* argv[]) {
        ProgramOptions program_options;

        auto options_description = po::options_description("Options");
        options_description.add_options()("help", "produce help message")(
            "list-solvers,l",
            po::bool_switch(&program_options.list_solvers)->default_value(false),
            "list available solvers")(
            "solver,s",
            po::value<std::string>(&program_options.solver_name),
            "name of solver to be launched")(
            "mapf-problem-file,i",
            po::value<std::filesystem::path>(&program_options.mapf_problem_file),
            "path to file with MAPF problem description")(
            "mapf-solution-file,o",
            po::value<std::filesystem::path>(&program_options.mapf_solution_file),
            "path to file to MAPF solution");

        const char* workspace_dir_env = std::getenv("BUILD_WORKSPACE_DIRECTORY");
        std::filesystem::path workspace_dir = workspace_dir_env ? workspace_dir_env : "";
        if (workspace_dir.empty()) {
            throw std::runtime_error("Got empty workspace dir");
        }

        po::variables_map variables_map;
        po::store(po::parse_command_line(argc, argv, options_description), variables_map);
        po::notify(variables_map);

        if (variables_map.count("help")) {
            std::cout << options_description << std::endl;
            std::exit(0);
        }

        if (program_options.list_solvers) {
            std::cout << "Available solvers:" << std::endl;
            for (auto it = mapf::solver::SolverFactory::Instance().cbegin();
                 it != mapf::solver::SolverFactory::Instance().cend();
                 ++it) {
                std::cout << std::string(4, ' ') << it->first << std::endl;
            }
            std::exit(0);
        }

        if (!variables_map.count("solver")) {
            throw std::runtime_error("the option '--solver' is required but missing");
        }

        if (mapf::solver::SolverFactory::Instance().find(program_options.solver_name)
            == mapf::solver::SolverFactory::Instance().cend()) {
            throw std::runtime_error(
                std::format("No such solver in factory: {}", program_options.solver_name));
        }

        if (!variables_map.count("mapf-problem-file")) {
            throw std::runtime_error("the option '--mapf-problem-file' is required but missing");
        }

        if (!variables_map.count("mapf-solution-file")) {
            throw std::runtime_error("the option '--mapf-solution-file' is required but missing");
        }

        program_options.mapf_problem_file = workspace_dir / program_options.mapf_problem_file;
        program_options.mapf_solution_file = workspace_dir / program_options.mapf_solution_file;

        return program_options;
    }

    bool list_solvers;
    std::string solver_name;
    std::filesystem::path mapf_problem_file;
    std::filesystem::path mapf_solution_file;
};

int main(int argc, char* argv[]) {
    auto program_options = ProgramOptions::Parse(argc, argv);

    auto mapf_problem_proto = mapf::models::LoadProtoFromFile<mapf::models::proto::MAPFProblem>(
        program_options.mapf_problem_file);

    auto mapf_problem_solution = mapf::solver::SolverFactory::Instance()
                                     .CreateSolver(program_options.solver_name)
                                     ->FindProtoSolution(mapf_problem_proto);

    mapf::models::SaveProtoToFile(mapf_problem_solution, program_options.mapf_solution_file);

    return 0;
}