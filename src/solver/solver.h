#pragma once
#include <vector>
#include "variable.h"
#include "problem_manager.h"

namespace Solver {

constexpr double EPS = 1e-5;

enum class ILQRSolveStatus {
    SOLVED,
    REACHED_MAX_ITERATION,
    REACHED_MAX_MU,
    PROBLEM_NOT_FORMULATED,
};

enum class LQRSolveStatus {
    RUNNING,
    CONVERGED,
    BACKWARD_PASS_FAIL,
    FORWARD_PASS_FAIL,
    FORWARD_PASS_SMALL_STEP,
};

struct ILQRConfig {
    double min_alpha = 0.01;
    double accept_step_threshold = 0.5;
    std::size_t max_iter = 150;
    double delta_0 = 2.0;
    double min_mu = 1e-6;
    double max_mu = 1000.0;
    double convergence_threshold = 1e-2;
};

template <std::size_t N_STATE, std::size_t N_CONTROL>
class ILQRSolver {

public:
    ILQRSolver(ProblemManager<N_STATE, N_CONTROL>& problem_manager)
        : _problem_manager(problem_manager),
        _current_trajectory(problem_manager.init_trajectory()),
        _num_steps(problem_manager.num_steps()),
        _k(problem_manager.num_steps(), Eigen::MatrixXd::Zero(N_CONTROL, N_CONTROL)),
        _K(problem_manager.num_steps(), Eigen::MatrixXd::Zero(N_CONTROL, N_STATE)),
        _vx(Eigen::MatrixXd::Zero(N_STATE, 1)),
        _vxx(Eigen::MatrixXd::Zero(N_STATE, N_STATE)),
        _derivatives(problem_manager.num_steps()),
        _approx_cost_decay_info({0.0, 0.0}) {
            CHECK_EQ(_current_trajectory.size(), _num_steps);
            _problem_manager.update_dynamic_costs(_current_trajectory);
            _current_cost = problem_manager.calculate_total_cost(problem_manager.init_trajectory());
        };
    ILQRSolveStatus solve();
    Trajectory<N_STATE, N_CONTROL> final_trajectory() const { return _current_trajectory; }

private:
    void calculate_derivatives();
    void backward_pass();
    void forward_pass();
    inline void increase_mu();
    inline void decrease_mu();
    ProblemManager<N_STATE, N_CONTROL>& _problem_manager;
    Trajectory<N_STATE, N_CONTROL> _current_trajectory;
    double _current_cost = 0.0;
    std::size_t _num_steps = 0;
    std::size_t _iter = 0;
    std::vector<Eigen::Matrix<double, N_CONTROL, N_CONTROL>> _k;
    std::vector<Eigen::Matrix<double, N_CONTROL, N_STATE>> _K;
    Eigen::Matrix<double, N_STATE, 1> _vx;
    Eigen::Matrix<double, N_STATE, N_STATE> _vxx;
    std::vector<DerivativesInfo<N_STATE, N_CONTROL>> _derivatives;
    std::pair<double, double> _approx_cost_decay_info;
    double _mu = 0.0;
    double _delta = 0.0;
    ILQRConfig _ilqr_config;
    LQRSolveStatus _current_solve_status = LQRSolveStatus::RUNNING;
};

template <std::size_t N_STATE, std::size_t N_CONTROL>
ILQRSolveStatus ILQRSolver<N_STATE, N_CONTROL>::solve() {
    if (not _problem_manager.problem_formulated() || _current_trajectory.empty()) {
        return ILQRSolveStatus::PROBLEM_NOT_FORMULATED;
    }
    _iter = 0;
    _current_solve_status = LQRSolveStatus::RUNNING;
    for (; _iter < _ilqr_config.max_iter; ++_iter) {
        // _current_solve_status is updated in the following functions.
        calculate_derivatives();
        backward_pass();
        forward_pass();
        // Process mu.
        if (_current_solve_status == LQRSolveStatus::BACKWARD_PASS_FAIL
                || _current_solve_status == LQRSolveStatus::FORWARD_PASS_FAIL) {
            increase_mu();
        } else if (_current_solve_status == LQRSolveStatus::RUNNING) {
            decrease_mu();
        }
        // Process optimization result.
        if (_mu > _ilqr_config.max_mu) {
            return ILQRSolveStatus::REACHED_MAX_MU;
        } else if (_current_solve_status == LQRSolveStatus::CONVERGED) {
            return ILQRSolveStatus::SOLVED;
        }
    }
    return ILQRSolveStatus::REACHED_MAX_ITERATION;
}

template <std::size_t N_STATE, std::size_t N_CONTROL>
void ILQRSolver<N_STATE, N_CONTROL>::increase_mu() {
    _delta = std::max(_ilqr_config.delta_0, _delta * _ilqr_config.delta_0);
    _mu = std::max(_ilqr_config.min_mu, _mu * _delta);
    LOG(INFO) << "Iter " << _iter << ", increase mu to " << _mu;
}

template <std::size_t N_STATE, std::size_t N_CONTROL>
void ILQRSolver<N_STATE, N_CONTROL>::decrease_mu() {
    _delta = std::min(1.0 / _ilqr_config.delta_0, _delta / _ilqr_config.delta_0);
    _mu *= _delta;
    if (_mu < _ilqr_config.min_mu) {
        _mu = 0.0;
    }
    LOG(INFO) << "Iter " << _iter << ", decrease mu to " << _mu;
}

template <std::size_t N_STATE, std::size_t N_CONTROL>
void ILQRSolver<N_STATE, N_CONTROL>::calculate_derivatives() {
    // Derivatives need to be calculated only if the trajectory is updated.
    if (_current_solve_status != LQRSolveStatus::RUNNING && _current_solve_status != LQRSolveStatus::FORWARD_PASS_SMALL_STEP) {
        _current_solve_status = LQRSolveStatus::RUNNING;
        return;
    }
    _current_solve_status = LQRSolveStatus::RUNNING;
    _problem_manager.update_dynamic_costs(_current_trajectory);
    for (std::size_t i = 0; i < _num_steps; ++i) {
        auto& derivative = _derivatives.at(i);
        _problem_manager.calculate_derivatives(_current_trajectory, i, &derivative);
        if (i < _num_steps - 1) {
            const double move_dist = _problem_manager.knots().at(i + 1) - _problem_manager.knots().at(i);
            derivative.fx = _problem_manager.dynamics().dx(_current_trajectory.at(i), move_dist);
            derivative.fu = _problem_manager.dynamics().du(_current_trajectory.at(i), move_dist);
        }
        // LOG_IF(INFO, _iter == 0 && i < 5) << "[Test] i " << i << ", lx " << derivative.lx << " lu " << derivative.lu << " lxx " << derivative.lxx;
    }
}

template <std::size_t N_STATE, std::size_t N_CONTROL>
void ILQRSolver<N_STATE, N_CONTROL>::backward_pass() {
    _vx = _derivatives.back().lx;
    _vxx = _derivatives.back().lxx;
    _approx_cost_decay_info = {0.0, 0.0};
    for (int i = _num_steps - 2; i >= 0; --i) {
        auto& derivative = _derivatives.at(i);
        const auto qx = derivative.lx + derivative.fx.transpose() * _vx;
        // LOG(INFO) << "[Test] Iter " << _iter << ", i " << i << ", qx " << qx;
        const auto qu = derivative.lu + derivative.fu.transpose() * _vx;
        const auto qxx = derivative.lxx + derivative.fx.transpose() * _vxx * derivative.fx;
        const auto quu = derivative.luu + derivative.fu.transpose() * _vxx * derivative.fu
            + _mu * Eigen::Matrix<double, N_CONTROL, N_CONTROL>::Identity();
        const auto qux = derivative.lux + derivative.fu.transpose() * _vxx * derivative.fx;

        Eigen::LLT<Eigen::MatrixXd> llt_quu(quu);
        if (llt_quu.info() == Eigen::NumericalIssue) {
            LOG(INFO) << "[Backward pass] Non-PD quu at index " << i << ", mu " << _mu;
            _current_solve_status = LQRSolveStatus::BACKWARD_PASS_FAIL;
            return;
        }

        const auto quu_inverse = quu.inverse();
        _k.at(i) = -quu_inverse * qu;
        _K.at(i) = -quu_inverse * qux;

        _vx = qx + _K.at(i).transpose() * quu * _k.at(i) + _K.at(i).transpose() * qu + qux.transpose() * _k.at(i);
        _vxx = qxx + _K.at(i).transpose() * quu * _K.at(i) + _K.at(i).transpose() * qux + qux.transpose() * _K.at(i);

        if (i < _num_steps - 1) {
            _approx_cost_decay_info.first += _k.at(i).transpose() * qu;
            _approx_cost_decay_info.second += (0.5 * _k.at(i).transpose() * quu * _k.at(i))(0);
        }
    }
    LOG(INFO) << "[Backward pass] Iter " << _iter << " OK";
}

template <std::size_t N_STATE, std::size_t N_CONTROL>
void ILQRSolver<N_STATE, N_CONTROL>::forward_pass() {
    if (_current_solve_status != LQRSolveStatus::RUNNING) {
        return;
    }
    double alpha = 1.0;
    double new_cost = 0.0;
    LOG(INFO) << "[Forward pass] Iter " << _iter << ", current cost " << _current_cost;
    while (alpha > _ilqr_config.min_alpha) {
        // Forward simulate.
        Trajectory<N_STATE, N_CONTROL> new_trajectory = _current_trajectory;
        for (std::size_t i = 0; i < _num_steps - 1; ++i) {
            *(new_trajectory.at(i).mutable_control()) =
                _current_trajectory.at(i).control() + alpha * _k.at(i) + _K.at(i) * (new_trajectory.at(i).state() - _current_trajectory.at(i).state());
            *(new_trajectory.at(i + 1).mutable_state()) =
                _problem_manager.dynamics().move_forward(new_trajectory.at(i), _problem_manager.knots().at(i + 1) - _problem_manager.knots().at(i));
        }
        // Calculate actual cost decay. 
        new_cost = _problem_manager.calculate_total_cost(new_trajectory);
        const double actual_cost_decay = _current_cost - new_cost;
        // if (std::fabs(alpha - 1.0) < EPS && actual_cost_decay < -100.0) {
        //     for (std::size_t i = 0; i < new_trajectory.size(); ++i) {
        //         LOG(INFO) << i << " old state " << _current_trajectory.at(i).state() << ", control " << _current_trajectory.at(i).control();
        //         LOG(INFO) << i << " new state " << new_trajectory.at(i).state() << ", control " << new_trajectory.at(i).control();
        //     }
        // }
        if (std::fabs(alpha - 1.0) < EPS && std::fabs(actual_cost_decay) < _ilqr_config.convergence_threshold) {
            LOG(INFO) << "[Forward pass] Iter " << _iter << ", optimization has converged.";
            _current_solve_status = LQRSolveStatus::CONVERGED;
            return;
        }
        const double approx_cost_decay = -(alpha * _approx_cost_decay_info.first + alpha * alpha * _approx_cost_decay_info.second);
        LOG(INFO) << "[Forward pass] Iter " << _iter << ", alpha " << alpha << ", actual cost decay " << actual_cost_decay << ", approx " << approx_cost_decay;
        if (actual_cost_decay > 0.0
                && (approx_cost_decay < 0.0
                || actual_cost_decay / approx_cost_decay > _ilqr_config.accept_step_threshold)) {
        // if (true) {
            LOG(INFO) << "[Forward pass] Iter " << _iter << ", accept alpha " << alpha << ", cost " << new_cost;
            _current_cost = new_cost;
            _current_trajectory = new_trajectory;
            if (std::fabs(alpha - 1.0) > EPS) {
                _current_solve_status = LQRSolveStatus::FORWARD_PASS_SMALL_STEP;
            }
            return;
        }
        alpha *= 0.5;
    }
    LOG(INFO) << "[Forward pass] Forward pass fail.";
    _current_solve_status = LQRSolveStatus::FORWARD_PASS_FAIL;
}
}