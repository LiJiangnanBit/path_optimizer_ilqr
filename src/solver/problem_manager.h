#pragma once
#include <vector>
#include <memory>
#include <string>
#include <unordered_map>
#include <glog/logging.h>
#include "variable.h"

namespace Solver {

template <std::size_t N_STATE, std::size_t N_CONTROL>
class Cost {
public:
    Cost(const std::string& name) : _name(name) {}
    virtual ~Cost() = default;
    virtual double cost_value(const Trajectory<N_STATE, N_CONTROL>& trajectory, std::size_t step) {
        return 0.0;
    }
    virtual void calculate_derivatives(const Trajectory<N_STATE, N_CONTROL>& trajectory, std::size_t step, DerivativesInfo<N_STATE, N_CONTROL>* derivatives) = 0;
    const std::string name() const { return _name; }
    virtual void update(const Trajectory<N_STATE, N_CONTROL>& trajectory) {}
private:
    const std::string _name;
};

template<std::size_t N, std::size_t M>
class ExpBarrierFunction {
public:
    ExpBarrierFunction(double q1, double q2) : _q1(q1), _q2(q2) {}
    double value(double x) const {
        return _q1 * std::exp(_q2 * x);
    }
    Eigen::Matrix<double, N, 1> dx(double x, const Eigen::Matrix<double, N, 1>& dx) const {
        return _q1 * _q2 * std::exp(_q2 * x) * dx;
    }
    Eigen::Matrix<double, N, M> ddx(double x, const Eigen::Matrix<double, N, 1>& dx, const Eigen::Matrix<double, N, M>& ddx) const {
        return _q1 * _q2 * std::exp(_q2 * x) * ddx + _q1 * _q2 * _q2 * std::exp(_q2 * x) * dx * dx.transpose();
    }

private:
    double _q1 = 1.0;
    double _q2 = 1.0;
};

template<std::size_t N_STATE, std::size_t N_CONTROL>
using CostMap = std::unordered_map<std::string, std::shared_ptr<Cost<N_STATE, N_CONTROL>>>;

template <std::size_t N_STATE, std::size_t N_CONTROL>
class Dynamics {
public:
    Dynamics() = default;
    virtual ~Dynamics() = default;
    virtual Variable<N_STATE> move_forward(const Node<N_STATE, N_CONTROL>& node, double move_dist) const = 0;
    virtual Eigen::Matrix<double, N_STATE, N_STATE> dx(const Node<N_STATE, N_CONTROL>& node, double move_dist) const = 0;
    virtual Eigen::Matrix<double, N_STATE, N_CONTROL> du(const Node<N_STATE, N_CONTROL>& node, double move_dist) const = 0;
};

template <std::size_t N_STATE, std::size_t N_CONTROL>
class ProblemManager {
public:
    ProblemManager() = default;
    virtual ~ProblemManager() = default;
    bool add_cost_item(std::shared_ptr<const Cost<N_STATE, N_CONTROL>> cost_item, std::size_t step);
    double calculate_total_cost(const Trajectory<N_STATE, N_CONTROL>& trajectory) const;
    const std::vector<double>& knots() const { return _knots; }
    const std::vector<CostMap<N_STATE, N_CONTROL>>& costs() const { return _costs; }
    std::size_t num_steps() const { return _knots.size(); }
    const Dynamics<N_STATE, N_CONTROL>& dynamics() const { return *_p_dynamics; }
    const Trajectory<N_STATE, N_CONTROL>& init_trajectory() const { return _init_trajectory; }
    void update_dynamic_costs(const Trajectory<N_STATE, N_CONTROL>& trajectory);
    void calculate_derivatives(const Trajectory<N_STATE, N_CONTROL>& trajectory, std::size_t step, DerivativesInfo<N_STATE, N_CONTROL>* derivatives) const;
    bool problem_formulated() const { return _problem_formulated; }
protected:
    std::vector<double> _knots;
    std::vector<CostMap<N_STATE, N_CONTROL>> _costs;
    std::shared_ptr<Dynamics<N_STATE, N_CONTROL>> _p_dynamics;
    Trajectory<N_STATE, N_CONTROL> _init_trajectory;
    // Some costs may need to be refreshed.
    std::vector<std::shared_ptr<Cost<N_STATE, N_CONTROL>>> _dynamic_costs;
    bool _problem_formulated = false;
};

template <std::size_t N_STATE, std::size_t N_CONTROL>
bool ProblemManager<N_STATE, N_CONTROL>::add_cost_item(std::shared_ptr<const Cost<N_STATE, N_CONTROL>> cost_item, std::size_t step) {
    if (step >= _costs.size()) {
        LOG(ERROR) << "[Problem manager] Trying to add cost at wrong step! Total steps: " << num_steps() << ", input step; " << step;
        return false;
    }
    _costs.at(step)[cost_item->name()] = cost_item;
}

template <std::size_t N_STATE, std::size_t N_CONTROL>
void ProblemManager<N_STATE, N_CONTROL>::calculate_derivatives(const Trajectory<N_STATE, N_CONTROL>& trajectory, std::size_t step, DerivativesInfo<N_STATE, N_CONTROL>* derivatives) const {
    CHECK(step < _costs.size());
    DerivativesInfo<N_STATE, N_CONTROL> tmp_derivatives;
    for (const auto& cost_pair : _costs.at(step)) {
        cost_pair.second->calculate_derivatives(trajectory, step, &tmp_derivatives);
    }
    *derivatives = std::move(tmp_derivatives);
}

template <std::size_t N_STATE, std::size_t N_CONTROL>
double ProblemManager<N_STATE, N_CONTROL>::calculate_total_cost(
    const Trajectory<N_STATE, N_CONTROL>& trajectory) const {
    // CHECK_EQ(Trajectory.size(), num_steps());
    const std::size_t size = std::min(_costs.size(), trajectory.size());
    double total_cost = 0.0;
    std::unordered_map<std::string, double> cost_values;
    for (std::size_t i = 0; i < size; ++i) {
        for (const auto& cost_pair : _costs.at(i)) {
            total_cost += cost_pair.second->cost_value(trajectory, i);
            cost_values[cost_pair.second->name()] += cost_pair.second->cost_value(trajectory, i);
        }
    }
    // for (const auto& pair : cost_values) {
    //     LOG(INFO) << "cost name " << pair.first << ", value " << pair.second;
    // }
    // LOG(INFO) << "[Test] total value " << total_cost;
    return total_cost;
}

template <std::size_t N_STATE, std::size_t N_CONTROL>
void ProblemManager<N_STATE, N_CONTROL>::update_dynamic_costs(const Trajectory<N_STATE, N_CONTROL>& trajectory) {
    for (auto cost : _dynamic_costs) {
        cost->update(trajectory);
    }
}

}