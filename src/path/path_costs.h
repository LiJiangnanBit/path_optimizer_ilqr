#pragma once
#include <cmath>
#include <string>
#include "data_structure.h"
#include "solver/problem_manager.h"
#include "gflags.h"

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

class RearBoundaryConstraint : public PathCost {
public:
    RearBoundaryConstraint(const FreeSpace& free_space, double q1, double q2, double buffer = 0.0, const std::string& name = "")
        : PathCost("rear_boundary_constraint" + name), _free_space(free_space), _barrier_function(q1, q2), _buffer(buffer) {}
    void update(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory) override {
        if (not _bounds.empty()) {
            return;
        }
        for (const auto& pt : trajectory) {
            _bounds.emplace_back(_free_space.get_circle_bound(pt.sample()));
        }
    }
    double cost_value(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        CHECK(step < _bounds.size());
        const auto& bound = _bounds.at(step);
        const double l = trajectory.at(step).state()(L_INDEX);
        return _barrier_function.value(bound.lb_l + _buffer - l) + _barrier_function.value(l - bound.ub_l + _buffer);
    }

    void calculate_derivatives(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step, DerivativesInfo<N_PATH_STATE, N_PATH_CONTROL>* derivatives) override {
        CHECK(step < _bounds.size());
        const auto& bound = _bounds.at(step);
        const double l = trajectory.at(step).state()(L_INDEX);
        derivatives->lx += _barrier_function.dx(bound.lb_l + _buffer - l, {-1.0, 0.0, 0.0}) + _barrier_function.dx(l - bound.ub_l + _buffer, {1.0, 0.0, 0.0});
        derivatives->lxx += _barrier_function.ddx(bound.lb_l + _buffer - l, {-1.0, 0.0, 0.0}, Eigen::MatrixXd::Zero(N_PATH_STATE, N_PATH_STATE))
            + _barrier_function.ddx(l - bound.ub_l + _buffer, {1.0, 0.0, 0.0}, Eigen::MatrixXd::Zero(N_PATH_STATE, N_PATH_STATE));
    }

private:
    const FreeSpace& _free_space;
    double _buffer = 0.0;
    std::vector<BoundaryPoint> _bounds;
    ExpBarrierFunction<N_PATH_STATE, N_PATH_STATE>  _barrier_function;
};

struct FrontBoundaryInfo {
    PathPoint front_ref_point;
    XYPosition front_xy;
    SLPosition front_sl;
    Vector norm_vec;
    double lb = 0.0;
    double ub = 0.0;
};

class FrontBoundaryConstraint : public PathCost {
public:
    FrontBoundaryConstraint(const FreeSpace& free_space, double q1, double q2, double buffer = 0.0, const std::string& name = "")
        : PathCost("front_boundary_constraint" + name), _free_space(free_space), _barrier_function(q1, q2), _buffer(buffer) {}
    void update(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory) override {
        const auto& reference_line = *_free_space.reference_line_ptr();
        if (_rear_ref_points.empty()) {
            for (const auto& pt : trajectory) {
                _rear_ref_points.emplace_back(reference_line.get_reference_point(pt.sample()));
            }
        }

        _info_vec.clear();
        const double rear_axle_to_front = FLAGS_vehicle_length * 0.5 + FLAGS_rear_axle_to_center;
        for (std::size_t i = 0; i < trajectory.size(); ++i) {
            const auto& pt = trajectory.at(i);
            const auto& rear_ref_pt = _rear_ref_points.at(i);
            const XYPosition rear_xy{rear_ref_pt.x + pt.state()(L_INDEX) * cos(rear_ref_pt.theta + M_PI_2), rear_ref_pt.y + pt.state()(L_INDEX) * sin(rear_ref_pt.theta + M_PI_2)};
            const double ego_heading = pt.state()(HD_INDEX) + rear_ref_pt.theta;
            const XYPosition front_xy{rear_xy.x + rear_axle_to_front * cos(ego_heading), rear_xy.y + rear_axle_to_front * sin(ego_heading)};
            const double hint_s = pt.sample() + rear_axle_to_front * cos(pt.state()(HD_INDEX));

            FrontBoundaryInfo info;
            info.front_xy = front_xy;
            info.front_sl = reference_line.get_projection_by_newton(front_xy, hint_s);
            info.front_ref_point = reference_line.get_reference_point(info.front_sl.s);
            info.norm_vec.x = cos(info.front_ref_point.theta + M_PI_2);
            info.norm_vec.y = sin(info.front_ref_point.theta + M_PI_2);
            _free_space.get_l_bound_for_circle(info.front_sl.s, FLAGS_vehicle_width / 2.0, &info.lb, &info.ub);
            if (i < 10) {
                LOG(INFO) << "i " << i << ", front s " << info.front_sl.s << ", front l " << info.front_sl.l << ", lb " << info.lb << ", ub " << info.ub;
            }
            _info_vec.emplace_back(std::move(info));
        }
    }
    double cost_value(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step) override {
        CHECK(step < _rear_ref_points.size());
        CHECK(step < _info_vec.size());
        const double rear_axle_to_front = FLAGS_vehicle_length * 0.5 + FLAGS_rear_axle_to_center;
        const auto& pt = trajectory.at(step);
        const auto& rear_ref_pt = _rear_ref_points.at(step);
        const XYPosition rear_xy{rear_ref_pt.x + pt.state()(L_INDEX) * cos(rear_ref_pt.theta + M_PI_2), rear_ref_pt.y + pt.state()(L_INDEX) * sin(rear_ref_pt.theta + M_PI_2)};
        const double ego_heading = pt.state()(HD_INDEX) + rear_ref_pt.theta;
        const XYPosition front_xy{rear_xy.x + rear_axle_to_front * cos(ego_heading), rear_xy.y + rear_axle_to_front * sin(ego_heading)};
        const auto& info = _info_vec.at(step);
        double front_l = (front_xy.x - info.front_ref_point.x) * info.norm_vec.x + (front_xy.y - info.front_ref_point.y) * info.norm_vec.y;
        if (step < 10) {
            LOG(INFO) << step << "front l " << front_l;
        }
        return _barrier_function.value(info.lb + _buffer - front_l) + _barrier_function.value(front_l - info.ub + _buffer);
    }

    void calculate_derivatives(const Trajectory<N_PATH_STATE, N_PATH_CONTROL>& trajectory, std::size_t step, DerivativesInfo<N_PATH_STATE, N_PATH_CONTROL>* derivatives) override {
        CHECK(step < _rear_ref_points.size());
        CHECK(step < _info_vec.size());
        const auto& info = _info_vec.at(step);
        const auto& norm_vec = info.norm_vec;
        const double front_l = info.front_sl.l;
        const auto& rear_ref_pt = _rear_ref_points.at(step);
        const auto& pt = trajectory.at(step);
        const double rear_axle_to_front = FLAGS_vehicle_length * 0.5 + FLAGS_rear_axle_to_center;
        const double ego_heading = pt.state()(HD_INDEX) + rear_ref_pt.theta;
        Eigen::Matrix<double, N_PATH_STATE, 1> dldx;
        dldx << norm_vec.x * cos(rear_ref_pt.theta + M_PI_2) + norm_vec.y * sin(rear_ref_pt.theta + M_PI_2),
            -norm_vec.x * rear_axle_to_front * sin(ego_heading) + norm_vec.y * rear_axle_to_front * cos(ego_heading),
            0.0;
        derivatives->lx += _barrier_function.dx(info.lb + _buffer - front_l, -dldx) + _barrier_function.dx(front_l - info.ub + _buffer, dldx);
        derivatives->lxx += _barrier_function.ddx(info.lb + _buffer - front_l, -dldx, Eigen::MatrixXd::Zero(N_PATH_STATE, N_PATH_STATE))
            + _barrier_function.ddx(front_l - info.ub + _buffer, dldx, Eigen::MatrixXd::Zero(N_PATH_STATE, N_PATH_STATE));
    }

private:
    const FreeSpace& _free_space;
    double _buffer = 0.0;
    ExpBarrierFunction<N_PATH_STATE, N_PATH_STATE> _barrier_function;
    std::vector<PathPoint> _rear_ref_points;
    std::vector<FrontBoundaryInfo> _info_vec;    
};

}