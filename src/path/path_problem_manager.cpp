#include <cmath>
#include "path_problem_manager.h"
#include "path_costs.h"
#include "tool.h"

namespace PathPlanning {
using namespace Solver;

constexpr double WEIGHT_REF_L = 0.001;
constexpr double WEIGHT_KAPPA = 10.0;
constexpr double WEIGHT_KAPPA_RATE = 50.0;
constexpr double WEIGHT_END_L = 0.5;
constexpr double WEIGHT_END_HEADING_DIFF = 20.0;

void PathProblemManager::formulate_path_problem(const FreeSpace& free_space, const ReferenceLine& reference_line, const PathPoint& init_state, const PathPoint& end_state) {
    // Initialize knots and costs.
    if (not free_space.is_initialized()) {
        LOG(INFO) << "[PathProblem] Quit for uninitialized free_space.";
        return;
    }
    // Set dynamics.
    _p_dynamics = std::make_shared<FrenetPathDynamics>(reference_line);
    // Init trajectory.
    calculate_init_trajectory(reference_line, init_state, end_state);
    // Add costs and constraints.
    add_costs(free_space, end_state);
    _problem_formulated = true;
}

void PathProblemManager::calculate_init_trajectory(const ReferenceLine& reference_line, const PathPoint& init_state, const PathPoint& end_state) {
    _init_trajectory.clear();
    _knots.clear();
    _costs.clear();

    auto dynamics_by_kappa = [&reference_line](const Node<N_PATH_STATE, N_PATH_CONTROL>& node, double move_dist)->Node<N_PATH_STATE, N_PATH_CONTROL> {
        const double kappa_ref = reference_line.get_reference_point(node.sample()).kappa;
        const double l = node.state()(L_INDEX);
        const double hd = node.state()(HD_INDEX);
        const double kappa = node.state()(K_INDEX);
        const double dl = (1 - kappa_ref * l) * tan(hd);
        const double dhd = (1 - kappa_ref * l) * kappa / cos(hd) - kappa_ref;
        Node<N_PATH_STATE, N_PATH_CONTROL> ret;
        (*ret.mutable_state())(L_INDEX) = l + dl * move_dist;
        (*ret.mutable_state())(HD_INDEX) = constrainAngle(hd + dhd * move_dist);
        return ret;
    };
    
    const auto cart_state = init_state;
    Node<N_PATH_STATE , N_PATH_CONTROL> node;
    const auto init_proj = reference_line.get_projection(init_state);
    node.set_sample(init_proj.s);
    (*node.mutable_state())(L_INDEX) = init_proj.l;
    (*node.mutable_state())(HD_INDEX) = constrainAngle(init_state.theta - reference_line.get_reference_point(init_proj.s).theta);
    (*node.mutable_state())(K_INDEX) = init_state.kappa;
    (*node.mutable_control())(KR_INDEX) = init_state.dkappa;
    _init_trajectory.emplace_back(node);
    _knots.emplace_back(init_proj.s);
    _costs.emplace_back(CostMap<N_PATH_STATE , N_PATH_CONTROL>());
    const double pursuit_dist = 10.0;
    const auto end_proj = reference_line.get_projection(end_state);

    for (std::size_t i = 0; node.sample() + _config.delta_s < end_proj.s; ++i) {
        auto new_node = dynamics_by_kappa(node, _config.delta_s);
        new_node.set_sample(node.sample() + _config.delta_s);
        if (i == 0) {
            (*new_node.mutable_state())(K_INDEX) = init_state.dkappa * _config.delta_s + init_state.kappa;
        } else {
            // Kappa by pure pursuit.
            const auto xy = reference_line.get_xy_by_sl(SLPosition{new_node.sample(), new_node.state()(L_INDEX)});
            const double heading = constrainAngle(reference_line.get_reference_point(new_node.sample()).theta + new_node.state()(HD_INDEX));
            const auto pursuit_point = reference_line.get_reference_point(new_node.sample() + pursuit_dist);
            const double dist = distance(xy, pursuit_point);
            double kappa = std::fabs(dist) > 1.0 ? 2 * sin(atan2(pursuit_point.y - xy.y, pursuit_point.x - xy.x) - heading) / dist : 0.0;
            kappa = std::min(kappa, FLAGS_max_kappa);
            kappa = std::max(kappa, -FLAGS_max_kappa);
            (*new_node.mutable_state())(K_INDEX) = kappa;
        }
        node = std::move(new_node);
        _init_trajectory.emplace_back(node);
        _knots.emplace_back(node.sample());
        _costs.emplace_back(CostMap<N_PATH_STATE , N_PATH_CONTROL>());
    }

    for (std::size_t i = 0; i < _init_trajectory.size() - 1; ++i) {
        const double delta_s = _config.delta_s * (1 - reference_line.get_reference_point(_init_trajectory.at(i).sample()).kappa) / cos(_init_trajectory.at(i).state()(HD_INDEX));
        (*_init_trajectory.at(i).mutable_control())(KR_INDEX) =
            (_init_trajectory.at(i + 1).state()(K_INDEX) - _init_trajectory.at(i).state()(K_INDEX)) / delta_s;
    }
}

std::vector<PathPoint> PathProblemManager::transform_to_path_points(
    const ReferenceLine& reference_line, const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory) {
   std::vector<PathPoint> ret;
   for (const auto& pt : trajectory) {
        SLPosition sl;
        sl.s = pt.sample();
        sl.l = pt.state()(PathPlanning::L_INDEX);
        const auto xy = reference_line.get_xy_by_sl(sl);
        PathPoint path_point;
        path_point.s = sl.s;
        path_point.l = sl.l;
        path_point.x = xy.x;
        path_point.y = xy.y;
        path_point.theta = constrainAngle(reference_line.get_reference_point(sl.s).theta + pt.state()(HD_INDEX));
        path_point.kappa = pt.state()(K_INDEX);
        ret.emplace_back(std::move(path_point));
   }
   return ret;
}

void PathProblemManager::add_costs(const FreeSpace& free_space, const PathPoint& end_state) {
    CHECK(_costs.size() == num_steps());
    std::shared_ptr<PathCost> ref_l_cost_ptr(new RefLCost(WEIGHT_REF_L));
    std::shared_ptr<PathCost> kappa_cost_ptr(new KappaCost(WEIGHT_KAPPA));
    std::shared_ptr<PathCost> kappa_rate_cost_ptr(new KappaRateCost(WEIGHT_KAPPA_RATE));
    std::shared_ptr<PathCost> rear_boundary_constraint_ptr(new RearBoundaryConstraint(free_space, 0.5, 2.5));
    std::shared_ptr<PathCost> front_boundary_constraint_ptr(new FrontBoundaryConstraint(free_space, 0.5, 2.5));
    std::shared_ptr<PathCost> kappa_constraint_ptr(new KappaConstraint(0.5, 2.5));
    const auto& reference_line = *(free_space.reference_line_ptr());
    const double end_state_l = reference_line.get_projection(end_state).l;
    const double end_state_heading_diff = constrainAngle(end_state.theta - reference_line.get_reference_point(_knots.back()).theta);
    std::shared_ptr<PathCost> end_state_cost_ptr(new TargetStateCost(end_state_l, end_state_heading_diff, WEIGHT_END_L, WEIGHT_END_HEADING_DIFF, "end_state"));

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
            _costs.at(step)[kappa_constraint_ptr->name()] = kappa_constraint_ptr;
        }
    }
    _costs.back()[end_state_cost_ptr->name()] = end_state_cost_ptr;
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