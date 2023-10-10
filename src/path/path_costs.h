#pragma once
#include <cmath>
#include <string>
#include "data_structure.h"
#include "solver/problem_manager.h"

namespace PathPlanning {

constexpr double vehicle_length = 4.8;
constexpr double vehicle_width = 1.9;
constexpr double rear_axle_to_center = 1.4;
constexpr double wheel_base = 3.6;

using namespace Solver;
using PathCost = Cost<N_PATH_STATE, N_PATH_CONTROL>;

class RefLCost : public PathCost {
public:
    RefLCost(double weight, const std::string& name = "") : PathCost("ref_l_cost" + name), _weight(weight) {}
    double cost_value(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        CHECK(step < trajectory.size());
        const double l = trajectory.at(step).state()(L_INDEX);
        return 0.5 * _weight * l * l;
    }

    void calculate_derivatives(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step, DerivativesInfo<N_PATH_STATE, N_PATH_CONTROL>* derivatives) override {
        CHECK(step < trajectory.size());
        const double l = trajectory.at(step).state()(L_INDEX);
        Eigen::Matrix<double, N_PATH_STATE, 1> dx{_weight * l, 0.0, 0.0};
        Eigen::Matrix<double, N_PATH_STATE, N_PATH_STATE> dxx;
        dxx << _weight, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        derivatives->lx += dx;
        derivatives->lxx += dxx;
    }
private:
    double _weight = 0.0;
};

class KappaCost : public PathCost {
public:
    KappaCost(double weight, const std::string& name = "") : PathCost("kappa_cost" + name), _weight(weight) {}
    double cost_value(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        CHECK(step < trajectory.size());
        const double kappa = trajectory.at(step).state()(K_INDEX);
        return 0.5 * _weight * kappa * kappa;
    }

    void calculate_derivatives(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step, DerivativesInfo<N_PATH_STATE, N_PATH_CONTROL>* derivatives) override {
        CHECK(step < trajectory.size());
        const double kappa = trajectory.at(step).state()(K_INDEX);
        Eigen::Matrix<double, N_PATH_STATE, 1> dx{0.0, 0.0, _weight * kappa};
        Eigen::Matrix<double, N_PATH_STATE, N_PATH_STATE> dxx;
        dxx <<  0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, _weight;
        derivatives->lx += dx;
        derivatives->lxx += dxx;
    }
private:
    double _weight = 0.0;
};

class KappaRateCost : public PathCost {
public:
    KappaRateCost(double weight, const std::string& name = "") : PathCost("kappa_rate_cost" + name), _weight(weight) {}
    double cost_value(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        CHECK(step < trajectory.size() - 1);
        const double kappa_rate = trajectory.at(step).control()(KR_INDEX);
        return 0.5 * _weight * kappa_rate * kappa_rate;
    }

    void calculate_derivatives(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step, DerivativesInfo<N_PATH_STATE, N_PATH_CONTROL>* derivatives) override {
        CHECK(step < trajectory.size() - 1);
        const double kappa_rate = trajectory.at(step).control()(KR_INDEX);
        Eigen::Matrix<double, N_PATH_CONTROL, 1> du{_weight * kappa_rate};
        Eigen::Matrix<double, N_PATH_CONTROL, N_PATH_CONTROL> duu{_weight};
        derivatives->lu += du;
        derivatives->luu += duu;
    }
private:
    double _weight = 0.0;
};

// ----------Constraints----------------

struct Bound {
    double lb = 0.0;
    double ub = 0.0;
};

class RearBoundaryConstraint : public PathCost {
public:
    RearBoundaryConstraint(const FreeSpace& free_space, double q1, double q2, double buffer = 0.0, const std::string& name = "")
        : PathCost("rear_boundary_constraint" + name), _free_space(free_space), _barrier_function(q1, q2), _buffer(buffer) {}
    void update(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory) override {
        if (not _bounds.empty()) {
            return;
        }
        Bound bound;
        for (const auto& pt : trajectory) {
            _free_space.get_l_bound_for_circle(pt.sample(), vehicle_width / 2.0, &bound.lb, &bound.ub);
            _bounds.emplace_back(bound);
            // LOG(INFO) << "s " << pt.sample() << ", lb " << bound.lb << ", ub " << bound.ub;
        }
    }
    double cost_value(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        CHECK(step < _bounds.size());
        const auto& bound = _bounds.at(step);
        const double l = trajectory.at(step).state()(L_INDEX);
        return _barrier_function.value(bound.lb + _buffer - l) + _barrier_function.value(l - bound.ub + _buffer);
    }

    void calculate_derivatives(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step, DerivativesInfo<N_PATH_STATE, N_PATH_CONTROL>* derivatives) override {
        CHECK(step < _bounds.size());
        const auto& bound = _bounds.at(step);
        const double l = trajectory.at(step).state()(L_INDEX);
        derivatives->lx += _barrier_function.dx(bound.lb + _buffer - l, {-1.0, 0.0, 0.0}) + _barrier_function.dx(l - bound.ub + _buffer, {1.0, 0.0, 0.0});
        derivatives->lxx += _barrier_function.ddx(bound.lb + _buffer - l, {-1.0, 0.0, 0.0}, Eigen::MatrixXd::Zero(N_PATH_STATE, N_PATH_STATE))
            + _barrier_function.ddx(l - bound.ub + _buffer, {1.0, 0.0, 0.0}, Eigen::MatrixXd::Zero(N_PATH_STATE, N_PATH_STATE));
    }

private:
    const FreeSpace& _free_space;
    double _buffer = 0.0;
    std::vector<Bound> _bounds;
    ExpBarrierFunction<N_PATH_STATE, N_PATH_STATE>  _barrier_function;
};

}