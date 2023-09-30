#pragma once
#include <cmath>
#include "data_structure.h"
#include "solver/problem_manager.h"

namespace PathPlanning {

using namespace Solver;
using PathCost = Cost<N_PATH_STATE, N_PATH_CONTROL>;
constexpr double weight_ref_l = 0.00001;
constexpr double weight_kappa = 1.0;
constexpr double weight_kappa_rate = 10.0;

class RefLCost : public PathCost {
public:
    RefLCost() : PathCost("ref_l_cost") {}
    double cost_value(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        CHECK(step < trajectory.size());
        const double l = trajectory.at(step).state()(L_INDEX);
        return 0.5 * weight_ref_l * l * l;
    }
    Eigen::Matrix<double, N_PATH_STATE, 1> dx(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        CHECK(step < trajectory.size());
        const double l = trajectory.at(step).state()(L_INDEX);
        Eigen::Matrix<double, N_PATH_STATE, 1> ret;
        ret << weight_ref_l * l, 0.0;
        return ret;
    }
    Eigen::Matrix<double, N_PATH_STATE, N_PATH_STATE> dxx(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        CHECK(step < trajectory.size());
        Eigen::Matrix<double, N_PATH_STATE, N_PATH_STATE> ret;
        ret << weight_ref_l, 0.0, 0.0, 0.0;
        return ret;
    }
};

class KappaCost : public PathCost {
public:
    KappaCost() : PathCost("kappa_cost") {}
    double cost_value(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        CHECK(step < trajectory.size());
        const double kappa = trajectory.at(step).control()(KAPPA_INDEX);
        return 0.5 * weight_kappa * kappa * kappa;
    }
    Eigen::Matrix<double, N_PATH_CONTROL, 1> du(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        CHECK(step < trajectory.size());
        const double kappa = trajectory.at(step).control()(KAPPA_INDEX);
        Eigen::Matrix<double, N_PATH_CONTROL, 1> ret;
        ret << weight_kappa * kappa;
        return ret;
    }
    Eigen::Matrix<double, N_PATH_CONTROL, N_PATH_CONTROL> duu(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        CHECK(step < trajectory.size());
        Eigen::Matrix<double, N_PATH_CONTROL, N_PATH_CONTROL> ret;
        ret << weight_kappa;
        return ret;
    }
};

// 0.5 * ((k1-k0)/ds)^2.
class KappaRateCost0 : public PathCost {
public:
    KappaRateCost0() : PathCost("kappa_rate_cost_0") {}
    double cost_value(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        CHECK(step < trajectory.size() - 1);
        update_info(trajectory, step);
        const double kappa_rate = (_kappa_1 - _kappa_0) / _ds;
        return 0.5 * weight_kappa_rate * kappa_rate * kappa_rate;
    }
    Eigen::Matrix<double, N_PATH_CONTROL, 1> du(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        CHECK(step < trajectory.size() - 1);
        update_info(trajectory, step);
        Eigen::Matrix<double, N_PATH_CONTROL, 1> ret;
        ret << weight_kappa_rate / _ds / _ds * (_kappa_0 - _kappa_1);
        return ret;
    }
    Eigen::Matrix<double, N_PATH_CONTROL, N_PATH_CONTROL> duu(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        CHECK(step < trajectory.size() - 1);
        update_info(trajectory, step);
        Eigen::Matrix<double, N_PATH_CONTROL, N_PATH_CONTROL> ret;
        ret << weight_kappa_rate / _ds / _ds;
    }
private:
    void update_info(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) {
        CHECK(step < trajectory.size() - 1);
        _ds = (trajectory.at(step + 1).sample() - trajectory.at(step).sample()) / cos(trajectory.at(step).state()(HD_INDEX));
        _kappa_0 = trajectory.at(step).control()(KAPPA_INDEX);
        _kappa_1 = trajectory.at(step + 1).control()(KAPPA_INDEX);
    }
    double _ds = 0.0;
    double _kappa_0 = 0.0;
    double _kappa_1 = 0.0;
};

class KappaRateCost1 : public PathCost {
public:
    KappaRateCost1() : PathCost("kappa_rate_cost_1") {}
    double cost_value(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        // Cost value is calculated in KappaRateCost0::cost_value.
        return 0.0;
    }
    Eigen::Matrix<double, N_PATH_CONTROL, 1> du(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        CHECK(step > 0 && step < trajectory.size());
        update_info(trajectory, step);
        Eigen::Matrix<double, N_PATH_CONTROL, 1> ret;
        ret << weight_kappa_rate / _ds / _ds * (_kappa_1 - _kappa_0);
        return ret;
    }
    Eigen::Matrix<double, N_PATH_CONTROL, N_PATH_CONTROL> duu(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        CHECK(step > 0 && step < trajectory.size());
        update_info(trajectory, step);
        Eigen::Matrix<double, N_PATH_CONTROL, N_PATH_CONTROL> ret;
        ret << weight_kappa_rate / _ds / _ds;
    }
private:
    void update_info(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) {
        CHECK(step < trajectory.size() - 1);
        _ds = (trajectory.at(step + 1).sample() - trajectory.at(step).sample()) / cos(trajectory.at(step).state()(HD_INDEX));
        _kappa_0 = trajectory.at(step).control()(KAPPA_INDEX);
        _kappa_1 = trajectory.at(step + 1).control()(KAPPA_INDEX);
    }
    double _ds = 0.0;
    double _kappa_0 = 0.0;
    double _kappa_1 = 0.0;
};
}