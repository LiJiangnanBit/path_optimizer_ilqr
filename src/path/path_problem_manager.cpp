#include "path_problem_manager.h"
#include "path_costs.h"

namespace PathPlanning {
using namespace Solver;

void PathProblemManager::formulate_path_problem(const FreeSpace& free_space, const ReferenceLine& reference_line) {
    // Initialize knots and costs.
    _config.planning_length = reference_line.length();
    sample_knots();
    // Set dynamics.
    _p_dynamics = std::make_shared<FrenetPathDynamics>();
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
        (*node.mutable_control()->mutable_vector())(0, 0) = ref_pt.kappa;
        _init_trajectory.emplace_back(node);
    }
}

void PathProblemManager::add_costs() {
    CHECK(_costs.size() == num_steps());
    std::shared_ptr<constPathCost> ref_l_cost_ptr(new RefLCost());
    std::shared_ptr<PathCost> kappa_cost_ptr(new KappaCost());
    std::shared_ptr<KappaRateCost0> kappa_rate_cost_0_ptr(new KappaRateCost0());
    std::shared_ptr<KappaRateCost1> kappa_rate_cost_1_ptr(new KappaRateCost1());
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
        if (step < num_steps() - 1 && step > 0) {
            _costs.at(step)[kappa_rate_cost_1_ptr->name()] = kappa_rate_cost_1_ptr;
        }
    }
}

Variable<N_PATH_STATE> FrenetPathDynamics::move_forward(const Variable<N_PATH_STATE>& state, const Variable<N_PATH_CONTROL>& control) const {
    Variable<N_PATH_STATE> ret;
    
    return ret;
}
Variable<N_PATH_STATE> FrenetPathDynamics::dx(const Variable<N_PATH_STATE>& state, const Variable<N_PATH_CONTROL>& control) const {
    Variable<N_PATH_STATE> ret;
    
    return ret;
}
Variable<N_PATH_CONTROL> FrenetPathDynamics::du(const Variable<N_PATH_STATE>& state, const Variable<N_PATH_CONTROL>& control) const {
    Variable<N_PATH_CONTROL> ret;
    
    return ret;
}

}