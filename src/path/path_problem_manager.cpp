#include <cmath>
#include "path_problem_manager.h"
#include "path_costs.h"
#include "tool.h"

namespace PathPlanning {
using namespace Solver;

constexpr double WEIGHT_REF_L = 0.001;
constexpr double WEIGHT_KAPPA = 10.0;
constexpr double WEIGHT_KAPPA_RATE = 50.0;
constexpr double WEIGHT_END_L = 0.1;

void PathProblemManager::formulate_path_problem(const FreeSpace& free_space, const ReferenceLine& reference_line) {
    // Initialize knots and costs.
    _config.planning_length = reference_line.length();
    sample_knots();
    // Set dynamics.
    _p_dynamics = std::make_shared<FrenetPathDynamics>(reference_line);
    // Init trajectory.
    calculate_init_trajectory(reference_line);
    // Add costs and constraints.
    add_costs(free_space);
}

void PathProblemManager::sample_knots() {
    _knots.clear();
    _costs.clear();
    for (double s = 0.0; s < _config.planning_length; s += _config.delta_s) {
        _knots.emplace_back(s);
        _costs.emplace_back(CostMap<N_PATH_STATE , N_PATH_CONTROL>());
    }
}

void PathProblemManager::calculate_init_trajectory(const ReferenceLine& reference_line) {
    _init_trajectory.clear();
    Node<N_PATH_STATE , N_PATH_CONTROL> node;
    for (const double s : _knots) {
        auto ref_pt = reference_line.get_reference_point(s);
        (*node.mutable_state())(K_INDEX) = ref_pt.kappa;
        node.set_sample(s);
        _init_trajectory.emplace_back(node);
    }
    for (std::size_t i = 0; i < _init_trajectory.size() - 1; ++i) {
        (*_init_trajectory.at(i).mutable_control())(KR_INDEX) =
            (_init_trajectory.at(i + 1).state()(K_INDEX) - _init_trajectory.at(i).state()(K_INDEX)) / _config.delta_s;
    }
}

void PathProblemManager::add_costs(const FreeSpace& free_space) {
    CHECK(_costs.size() == num_steps());
    std::shared_ptr<PathCost> ref_l_cost_ptr(new RefLCost(WEIGHT_REF_L));
    std::shared_ptr<PathCost> kappa_cost_ptr(new KappaCost(WEIGHT_KAPPA));
    std::shared_ptr<PathCost> kappa_rate_cost_ptr(new KappaRateCost(WEIGHT_KAPPA_RATE));
    std::shared_ptr<PathCost> end_ref_l_cost_ptr(new RefLCost(WEIGHT_END_L, "_end_state"));
    std::shared_ptr<PathCost> rear_boundary_constraint_ptr(new RearBoundaryConstraint(free_space, 0.5, 2.5));
    std::shared_ptr<PathCost> front_boundary_constraint_ptr(new FrontBoundaryConstraint(free_space, 0.5, 2.5));

    _dynamic_costs.emplace_back(rear_boundary_constraint_ptr);
    _dynamic_costs.emplace_back(front_boundary_constraint_ptr);
    
    for (std::size_t step = 0; step < num_steps(); ++step) {
        _costs.at(step)[ref_l_cost_ptr->name()] = ref_l_cost_ptr;
        _costs.at(step)[kappa_cost_ptr->name()] = kappa_cost_ptr;
        if (step < num_steps() - 1) {
            _costs.at(step)[kappa_rate_cost_ptr->name()] = kappa_rate_cost_ptr;
        }
        if (step > 0) {
            _costs.at(step)[rear_boundary_constraint_ptr->name()] = rear_boundary_constraint_ptr;
            _costs.at(step)[front_boundary_constraint_ptr->name()] = front_boundary_constraint_ptr;
        }
    }
    _costs.back()[end_ref_l_cost_ptr->name()] = end_ref_l_cost_ptr;
}

Variable<N_PATH_STATE> FrenetPathDynamics::move_forward(const Node<N_PATH_STATE , N_PATH_CONTROL>& node, double move_dist) const {
    const double kappa_ref = _reference_line.get_reference_point(node.sample()).kappa;
    const double l = node.state()(L_INDEX);
    const double hd = node.state()(HD_INDEX);
    const double kappa = node.state()(K_INDEX);
    const double kappa_rate = node.control()(KR_INDEX);

    const double dl = (1 - kappa_ref * l) * tan(hd);
    const double dhd = (1 - kappa_ref * l) * kappa / cos(hd) - kappa_ref;
    const double dk = (1 - kappa_ref * l) / cos(hd) * kappa_rate;

    Variable<N_PATH_STATE> ret;
    ret << l + dl * move_dist, constrainAngle(hd + dhd * move_dist), kappa + dk * move_dist;
    return ret;
}

Eigen::Matrix<double, N_PATH_STATE, N_PATH_STATE> FrenetPathDynamics::dx(
    const Node<N_PATH_STATE , N_PATH_CONTROL>& node, double move_dist) const {
    const double kappa_ref = _reference_line.get_reference_point(node.sample()).kappa;
    const double l = node.state()(L_INDEX);
    const double hd = node.state()(HD_INDEX);
    const double kappa = node.state()(K_INDEX);
    const double kappa_rate = node.control()(KR_INDEX);

    Eigen::Matrix<double, N_PATH_STATE, N_PATH_STATE> ret;
    ret << 1 - move_dist * tan(hd) * kappa_ref, move_dist * (1 - kappa_ref * l) / cos(hd) / cos(hd), 0.0,
        -move_dist * kappa * kappa_ref / cos(hd), 1 + move_dist * (1 - kappa_ref * l) * kappa * tan(hd) / cos(hd), move_dist * (1 - kappa_ref * l) / cos(hd),
        -move_dist * kappa_rate * kappa_ref / cos(hd), move_dist * (1 - kappa_ref * l) * kappa_rate * tan(hd) / cos(hd), 1.0;
    return ret;
}

Eigen::Matrix<double, N_PATH_STATE, N_PATH_CONTROL> FrenetPathDynamics::du(
    const Node<N_PATH_STATE , N_PATH_CONTROL>& node, double move_dist) const {
    const double kappa_ref = _reference_line.get_reference_point(node.sample()).kappa;
    const double l = node.state()(L_INDEX);
    const double hd = node.state()(HD_INDEX);
    const double kappa = node.state()(K_INDEX);
    Eigen::Matrix<double, N_PATH_STATE, N_PATH_CONTROL> ret;
    ret << 0.0, 0.0, move_dist * (1 - kappa_ref * l) / cos(hd);
    return ret;
}

}