#include <cmath>
#include "path_problem_manager.h"
#include "path_costs.h"

namespace PathPlanning {
using namespace Solver;

void PathProblemManager::formulate_path_problem(const FreeSpace& free_space, const ReferenceLine& reference_line) {
    // Initialize knots and costs.
    _config.planning_length = reference_line.length();
    sample_knots();
    // Set dynamics.
    _p_dynamics = std::make_shared<FrenetPathDynamics>(reference_line);
    // Init trajectory.
    calculate_init_trajectory(reference_line);
    // Add costs and constraints.
    add_costs();
}

void PathProblemManager::sample_knots() {
    _knots.clear();
    _costs.clear();
    for (double s = 0.0; s < _config.planning_length; s += _config.delta_s) {
        _knots.emplace_back(s);
        _costs.emplace_back(CostMap<N_PATH_STATE , N_PATH_CONTROL >());
    }
}

void PathProblemManager::calculate_init_trajectory(const ReferenceLine& reference_line) {
    _init_trajectory.clear();
    Node<N_PATH_STATE , N_PATH_CONTROL > node;
    for (const double s : _knots) {
        auto ref_pt = reference_line.get_reference_point(s);
        (*node.mutable_control())(0, 0) = ref_pt.kappa;
        node.set_sample(s);
        _init_trajectory.emplace_back(node);
    }
}

void PathProblemManager::add_costs() {
    CHECK(_costs.size() == num_steps());
    std::shared_ptr<PathCost> ref_l_cost_ptr(new RefLCost());
    std::shared_ptr<PathCost> kappa_cost_ptr(new KappaCost());
    std::shared_ptr<PathCost> kappa_rate_cost_0_ptr(new KappaRateCost0());
    std::shared_ptr<PathCost> kappa_rate_cost_1_ptr(new KappaRateCost1());
    for (std::size_t step = 0; step < num_steps(); ++step) {
        // Ref l cost for each step:
        _costs.at(step)[ref_l_cost_ptr->name()] = ref_l_cost_ptr;
        // Kappa cost for each step but the last one:
        if (step < num_steps() - 1) {
            _costs.at(step)[kappa_cost_ptr->name()] = kappa_cost_ptr;
        }
        // Kappa rate cost.
        // part 1:
        if (step < num_steps() - 2) {
            _costs.at(step)[kappa_rate_cost_0_ptr->name()] = kappa_rate_cost_0_ptr;
        }
        // part 2:
        if (step < num_steps() - 1 && step > 0) {
            _costs.at(step)[kappa_rate_cost_1_ptr->name()] = kappa_rate_cost_1_ptr;
        }
    }
}

Variable<N_PATH_STATE> FrenetPathDynamics::move_forward(const Node<N_PATH_STATE , N_PATH_CONTROL>& node, double move_dist) const {
    const double kappa_ref = _reference_line.get_reference_point(node.sample()).kappa;
    const double l = node.state()(L_INDEX);
    const double hd = node.state()(HD_INDEX);
    const double kappa = node.control()(KAPPA_INDEX);
    const double dl = (1 - kappa_ref * l) * tan(hd);
    const double dhd = (1 - kappa_ref * l) * kappa / cos(hd) - kappa_ref;
    Variable<N_PATH_STATE> ret;
    ret << l + dl * move_dist, hd + dhd * move_dist;
    return ret;
}
Eigen::Matrix<double, N_PATH_STATE, N_PATH_STATE> FrenetPathDynamics::dx(
    const Node<N_PATH_STATE , N_PATH_CONTROL>& node, double move_dist) const {
    const double kappa_ref = _reference_line.get_reference_point(node.sample()).kappa;
    const double l = node.state()(L_INDEX);
    const double hd = node.state()(HD_INDEX);
    const double kappa = node.control()(KAPPA_INDEX);
    Eigen::Matrix<double, N_PATH_STATE, N_PATH_STATE> ret;
    ret << 1 - move_dist * tan(hd) * kappa_ref,
        move_dist * (1 - kappa_ref * l) / cos(hd) / cos(hd),
        -move_dist * kappa * kappa_ref / cos(hd),
        1 + move_dist * (1 - kappa_ref * l) * kappa * tan(hd) / cos(hd);
    return ret;
}
Eigen::Matrix<double, N_PATH_STATE, N_PATH_CONTROL> FrenetPathDynamics::du(
    const Node<N_PATH_STATE , N_PATH_CONTROL>& node, double move_dist) const {
    const double kappa_ref = _reference_line.get_reference_point(node.sample()).kappa;
    const double l = node.state()(L_INDEX);
    const double hd = node.state()(HD_INDEX);
    Eigen::Matrix<double, N_PATH_STATE, N_PATH_CONTROL> ret;
    ret << 0, move_dist * (1 - kappa_ref * l) / cos(hd);
    return ret;
}

}