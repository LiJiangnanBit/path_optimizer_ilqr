#pragma once
#include <cmath>
#include <string>
#include "data_structure.h"
#include "solver/problem_manager.h"

namespace PathPlanning {

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
    Eigen::Matrix<double, N_PATH_STATE, 1> dx(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        CHECK(step < trajectory.size());
        const double l = trajectory.at(step).state()(L_INDEX);
        Eigen::Matrix<double, N_PATH_STATE, 1> ret;
        ret << _weight * l, 0.0, 0.0;
        return ret;
    }
    Eigen::Matrix<double, N_PATH_STATE, N_PATH_STATE> dxx(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        CHECK(step < trajectory.size());
        Eigen::Matrix<double, N_PATH_STATE, N_PATH_STATE> ret;
        ret << _weight, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        return ret;
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
    Eigen::Matrix<double, N_PATH_STATE, 1> dx(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        CHECK(step < trajectory.size());
        const double kappa = trajectory.at(step).state()(K_INDEX);
        Eigen::Matrix<double, N_PATH_STATE, 1> ret;
        ret << 0.0, 0.0, _weight * kappa;
        return ret;
    }
    Eigen::Matrix<double, N_PATH_STATE, N_PATH_STATE> dxx(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        CHECK(step < trajectory.size());
        Eigen::Matrix<double, N_PATH_STATE, N_PATH_STATE> ret;
        ret << 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, _weight;
        return ret;
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
    Eigen::Matrix<double, N_PATH_CONTROL, 1> du(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        CHECK(step < trajectory.size() - 1);
        const double kappa_rate = trajectory.at(step).control()(KR_INDEX);
        Eigen::Matrix<double, N_PATH_CONTROL, 1> ret;
        ret << _weight * kappa_rate;
        return ret;
    }
    Eigen::Matrix<double, N_PATH_CONTROL, N_PATH_CONTROL> duu(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        CHECK(step < trajectory.size() - 1);
        Eigen::Matrix<double, N_PATH_CONTROL, N_PATH_CONTROL> ret;
        ret << _weight;
        return ret;
    }
private:
    double _weight = 0.0;
};

}