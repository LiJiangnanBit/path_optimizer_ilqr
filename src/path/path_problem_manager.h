#pragma once
#include "solver/problem_manager.h"
#include "path/free_space.h"
#include "path/reference_line.h"

namespace PathPlanning {

using namespace Solver;

class FrenetPathDynamics : public Dynamics<N_PATH_STATE , N_PATH_CONTROL > {
public:
    Variable<N_PATH_STATE> move_forward(const Variable<N_PATH_STATE>& state, const Variable<N_PATH_CONTROL>& control) const override;
    Variable<N_PATH_STATE> dx(const Variable<N_PATH_STATE>& state, const Variable<N_PATH_CONTROL>& control) const override;
    Variable<N_PATH_CONTROL> du(const Variable<N_PATH_STATE>& state, const Variable<N_PATH_CONTROL>& control) const override;
};

struct Config {
    double delta_s = 0.3;
    double planning_length = 0.0;
};

class PathProblemManager : public ProblemManager<N_PATH_STATE , N_PATH_CONTROL > {
public:
    void formulate_path_problem(const FreeSpace& free_space, const ReferenceLine& reference_line);

private:
    void sample_knots();
    void calculate_init_trajectory(const ReferenceLine& reference_line);
    void add_costs();
    Config _config;
};

}