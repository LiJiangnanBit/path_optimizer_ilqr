#pragma once
#include "solver/problem_manager.h"
#include "path/free_space.h"
#include "path/reference_line.h"

namespace PathPlanning {

using namespace Solver;

class FrenetPathDynamics : public Dynamics<N_PATH_STATE , N_PATH_CONTROL> {
public:
    FrenetPathDynamics(const ReferenceLine& reference_line) : _reference_line(reference_line) {}
    Variable<N_PATH_STATE> move_forward(const Node<N_PATH_STATE , N_PATH_CONTROL>& node, double move_dist) const override;
    Eigen::Matrix<double, N_PATH_STATE, N_PATH_STATE> dx(const Node<N_PATH_STATE , N_PATH_CONTROL>& node, double move_dist) const override;
    Eigen::Matrix<double, N_PATH_STATE, N_PATH_CONTROL> du(const Node<N_PATH_STATE , N_PATH_CONTROL>& node, double move_dist) const override;
private:
    const ReferenceLine& _reference_line;
};

struct Config {
    double delta_s = 0.3;
    double planning_length = 0.0;
};

class PathProblemManager : public ProblemManager<N_PATH_STATE, N_PATH_CONTROL > {
public:
    void formulate_path_problem(const FreeSpace& free_space, const ReferenceLine& reference_line, const PathPoint& init_state);
    const Config& config() const { return _config; }
    static std::vector<PathPoint> transform_to_path_points(
        const ReferenceLine& reference_line, const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory);

private:
    void sample_knots(const ReferenceLine& reference_line, const PathPoint& init_state);
    void calculate_init_trajectory(const ReferenceLine& reference_line, const PathPoint& init_state);
    void add_costs(const FreeSpace& free_space);
    Config _config;
};

}