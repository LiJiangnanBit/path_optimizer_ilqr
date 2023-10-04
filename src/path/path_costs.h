#pragma once
#include <cmath>
#include "data_structure.h"
#include "solver/problem_manager.h"

namespace PathPlanning {

using namespace Solver;
using PathCost = Cost<N_PATH_STATE, N_PATH_CONTROL>;
constexpr double weight_ref_l = 0.001;
constexpr double weight_kappa = 10.0;
constexpr double weight_kappa_rate = 50.0;

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
        ret << weight_ref_l * l, 0.0, 0.0;
        return ret;
    }
    Eigen::Matrix<double, N_PATH_STATE, N_PATH_STATE> dxx(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        CHECK(step < trajectory.size());
        Eigen::Matrix<double, N_PATH_STATE, N_PATH_STATE> ret;
        ret << weight_ref_l, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        return ret;
    }
};

class KappaCost : public PathCost {
public:
    KappaCost() : PathCost("kappa_cost") {}
    double cost_value(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        CHECK(step < trajectory.size());
        const double kappa = trajectory.at(step).state()(K_INDEX);
        return 0.5 * weight_kappa * kappa * kappa;
    }
    Eigen::Matrix<double, N_PATH_STATE, 1> dx(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        CHECK(step < trajectory.size());
        const double kappa = trajectory.at(step).state()(K_INDEX);
        Eigen::Matrix<double, N_PATH_STATE, 1> ret;
        ret << 0.0, 0.0, weight_kappa * kappa;
        return ret;
    }
    Eigen::Matrix<double, N_PATH_STATE, N_PATH_STATE> dxx(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        CHECK(step < trajectory.size());
        Eigen::Matrix<double, N_PATH_STATE, N_PATH_STATE> ret;
        ret << 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, weight_kappa;
        return ret;
    }
};

class KappaRateCost : public PathCost {
public:
    KappaRateCost() : PathCost("kappa_rate_cost") {}
    double cost_value(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        CHECK(step < trajectory.size() - 1);
        const double kappa_rate = trajectory.at(step).control()(KR_INDEX);
        return 0.5 * weight_kappa_rate * kappa_rate * kappa_rate;
    }
    Eigen::Matrix<double, N_PATH_CONTROL, 1> du(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        CHECK(step < trajectory.size() - 1);
        const double kappa_rate = trajectory.at(step).control()(KR_INDEX);
        Eigen::Matrix<double, N_PATH_CONTROL, 1> ret;
        ret << weight_kappa_rate * kappa_rate;
        return ret;
    }
    Eigen::Matrix<double, N_PATH_CONTROL, N_PATH_CONTROL> duu(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        CHECK(step < trajectory.size() - 1);
        Eigen::Matrix<double, N_PATH_CONTROL, N_PATH_CONTROL> ret;
        ret << weight_kappa_rate;
        return ret;
    }
};

class EndStateCost : public PathCost {
public:
    EndStateCost() : PathCost("end_state_cost") {}
    double 
};
}