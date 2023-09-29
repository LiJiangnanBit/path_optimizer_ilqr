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
    virtual Eigen::Matrix<double, N_STATE, 1> dx(
        const Trajectory<N_STATE, N_CONTROL>& trajectory, std::size_t step) {
            return Eigen::MatrixXd::Zero(N_STATE, 1);
        };
    virtual Eigen::Matrix<double, N_CONTROL, 1> du(
        const Trajectory<N_STATE, N_CONTROL>& trajectory, std::size_t step) {
            return Eigen::MatrixXd::Zero(N_CONTROL, 1);
        };
    virtual Eigen::Matrix<double, N_STATE, N_STATE> dxx(
        const Trajectory<N_STATE, N_CONTROL>& trajectory, std::size_t step) {
            return Eigen::MatrixXd::Zero( N_STATE, N_STATE);
        };
    virtual Eigen::Matrix<double, N_CONTROL, N_CONTROL> duu(
        const Trajectory<N_STATE, N_CONTROL>& trajectory, std::size_t step) {
            return Eigen::MatrixXd::Zero(N_CONTROL, N_CONTROL);
        };
    virtual Eigen::Matrix<double, N_CONTROL, N_STATE> dux(
        const Trajectory<N_STATE, N_CONTROL>& trajectory, std::size_t step) {
            return Eigen::MatrixXd::Zero(N_CONTROL, N_STATE);
        };
    const std::string name() const { return _name; } 
private:
    const std::string _name;
};

template<std::size_t N_STATE, std::size_t N_CONTROL>
using CostMap = std::unordered_map<std::string, std::shared_ptr<Cost<N_STATE, N_CONTROL>>>;

template <std::size_t N_STATE, std::size_t N_CONTROL>
class Dynamics {
public:
    Dynamics() = default;
    virtual ~Dynamics() = default;
    virtual Variable<N_STATE> move_forward(const Variable<N_STATE>& state, const Variable<N_CONTROL>& control) const = 0;
    virtual Variable<N_STATE> dx(const Variable<N_STATE>& state, const Variable<N_CONTROL>& control) const = 0;
    virtual Variable<N_CONTROL> du(const Variable<N_STATE>& state, const Variable<N_CONTROL>& control) const = 0;
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

    Eigen::Matrix<double, N_STATE, 1> dx(
        const Trajectory<N_STATE, N_CONTROL>& trajectory, std::size_t step) const;
    Eigen::Matrix<double, N_CONTROL, 1> du(
        const Trajectory<N_STATE, N_CONTROL>& trajectory, std::size_t step) const;
    Eigen::Matrix<double, N_STATE, N_STATE> dxx(
        const Trajectory<N_STATE, N_CONTROL>& trajectory, std::size_t step) const;
    Eigen::Matrix<double, N_CONTROL, N_CONTROL> duu (
        const Trajectory<N_STATE, N_CONTROL>& trajectory, std::size_t step) const;
    Eigen::Matrix<double, N_CONTROL, N_STATE> dux(
        const Trajectory<N_STATE, N_CONTROL>& trajectory, std::size_t step) const;
protected:
    std::vector<double> _knots;
    std::vector<CostMap<N_STATE, N_CONTROL>> _costs;
    std::shared_ptr<Dynamics<N_STATE, N_CONTROL>> _p_dynamics;
    Trajectory<N_STATE, N_CONTROL> _init_trajectory;
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
Eigen::Matrix<double, N_STATE, 1> ProblemManager<N_STATE, N_CONTROL>::dx(
    const Trajectory<N_STATE, N_CONTROL>& trajectory, std::size_t step) const {
    CHECK(step < _costs.size());
    Eigen::Matrix<double, N_STATE, 1> ret = Eigen::MatrixXd::Zero(N_STATE, 1);
    for (const auto& cost_pair : _costs.at(step)) {
        ret += cost_pair.second->dx(trajectory, step);
    }
    return ret;
}
template <std::size_t N_STATE, std::size_t N_CONTROL>
Eigen::Matrix<double, N_CONTROL, 1> ProblemManager<N_STATE, N_CONTROL>::du(
    const Trajectory<N_STATE, N_CONTROL>& trajectory, std::size_t step) const {
    CHECK(step < _costs.size());
    Eigen::Matrix<double, N_CONTROL, 1> ret = Eigen::MatrixXd::Zero(N_CONTROL, 1);
    for (const auto& cost_pair : _costs.at(step)) {
        ret += cost_pair.second->du(trajectory, step);
    }
    return ret;
}
template <std::size_t N_STATE, std::size_t N_CONTROL>
Eigen::Matrix<double, N_STATE, N_STATE> ProblemManager<N_STATE, N_CONTROL>::dxx(
    const Trajectory<N_STATE, N_CONTROL>& trajectory, std::size_t step) const {
    CHECK(step < _costs.size());
    Eigen::Matrix<double, N_STATE, N_STATE> ret = Eigen::MatrixXd::Zero(N_STATE, N_STATE);
    for (const auto& cost_pair : _costs.at(step)) {
        ret += cost_pair.second->dxx(trajectory, step);
    }
    return ret;
}
template <std::size_t N_STATE, std::size_t N_CONTROL>
Eigen::Matrix<double, N_CONTROL, N_CONTROL> ProblemManager<N_STATE, N_CONTROL>::duu (
    const Trajectory<N_STATE, N_CONTROL>& trajectory, std::size_t step) const {
    CHECK(step < _costs.size());
    Eigen::Matrix<double, N_CONTROL, N_CONTROL> ret = Eigen::MatrixXd::Zero(N_CONTROL, N_CONTROL);
    for (const auto& cost_pair : _costs.at(step)) {
        ret += cost_pair.second->duu(trajectory, step);
    }
    return ret;
}
template <std::size_t N_STATE, std::size_t N_CONTROL>
Eigen::Matrix<double, N_CONTROL, N_STATE> ProblemManager<N_STATE, N_CONTROL>::dux(
    const Trajectory<N_STATE, N_CONTROL>& trajectory, std::size_t step) const {
    CHECK(step < _costs.size());
    Eigen::Matrix<double, N_CONTROL, N_STATE> ret = Eigen::MatrixXd::Zero(N_CONTROL, N_STATE);
    for (const auto& cost_pair : _costs.at(step)) {
        ret += cost_pair.second->dux(trajectory, step);
    }
    return ret;
}

template <std::size_t N_STATE, std::size_t N_CONTROL>
double ProblemManager<N_STATE, N_CONTROL>::calculate_total_cost(
    const Trajectory<N_STATE, N_CONTROL>& trajectory) const {
    // CHECK_EQ(Trajectory.size(), num_steps());
    const std::size_t size = std::min(_costs.size(), trajectory.size());
    double total_cost = 0.0;
    for (std::size_t i = 0; i < size; ++i) {
        for (const auto& cost_pair : _costs.at(i)) {
            total_cost += cost_pair.second->cost_value(trajectory, i);
        }
    }
    return total_cost;
}

}