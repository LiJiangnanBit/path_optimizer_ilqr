#pragma once
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include "solver/variable.h"

namespace PathPlanning {

using Solver::Variable;
using Solver::Node;

constexpr std::size_t N_PATH_STATE  = 2;
constexpr std::size_t N_PATH_CONTROL = 1;
constexpr std::size_t L_INDEX = 0;
constexpr std::size_t HD_INDEX = 1; // HD for heading diff.
constexpr std::size_t KAPPA_INDEX = 0;

class FrenetPathState : public Variable<3> {
public:
    double l() const { return _vector(0); }
    double theta_diff() const { return _vector(1); }
    double kappa() const { return _vector(2); }
    void set_l(double l) { _vector(0) = l; }
    void set_theta_diff(double theta_diff) { _vector(1) = theta_diff; }
    void set_kappa(double kappa) { _vector(2) = kappa; }
};

class PathControl : public Variable<1> {
public: 
    double dkappa() const { return _vector(0); }
    void set_dkappa(double dkappa) { _vector(0) = dkappa; }
};

// class PathNode : public Node<3, 1> {
// public:
//     const Variable<3>& state() const override { return _state; }
//     const Variable<1>& control() const override { return _control; }
//     Variable<3>* mutable_state() override { return &_state; }
//     Variable<1>* mutable_control() override { return &_control; }

// private:
//     FrenetPathState _state;
//     PathControl _control;
// };

struct XYPosition {
    double x = 0.0;
    double y = 0.0;
};

struct SLPosition {
    double s = 0.0;
    double l = 0.0;
};

struct PathPoint : public XYPosition, SLPosition {
    double theta = 0.0;
    double kappa = 0.0;
    double dkappa = 0.0;
    double theta_diff = 0.0;
    double dl = 0.0;
    double ddl = 0.0;
};

} // namespace PathPlanning
