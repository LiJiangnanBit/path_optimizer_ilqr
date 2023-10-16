#pragma once
#include <memory>
#include <vector>
#include <Eigen/Dense>
#include "solver/variable.h"

namespace PathPlanning {

using Solver::Variable;
using Solver::Node;

constexpr std::size_t N_PATH_STATE  = 3;
constexpr std::size_t N_PATH_CONTROL = 1;
constexpr std::size_t L_INDEX = 0;
constexpr std::size_t HD_INDEX = 1; // HD for heading diff.
constexpr std::size_t K_INDEX = 2;
constexpr std::size_t KR_INDEX = 0; // Control, kappa rate.

struct XYPosition {
    double x = 0.0;
    double y = 0.0;
};
using Vector = XYPosition;

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
