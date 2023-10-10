#pragma once
#include <memory>
#include <vector>
#include <Eigen/Dense>

namespace Solver {

template <std::size_t N>
using Variable = Eigen::Matrix<double, N, 1>;

// template <std::size_t N>
// class Variable {
// public:
//     Variable() : _vector(Eigen::VectorXd::Zero(N)), _dim(N) {}
//     Variable(const Eigen::Matrix<double, N, 1>& vector) : _vector(vector), _dim(N) {}
//     virtual ~Variable() {}
//     const Eigen::Matrix<double, N, 1>& vector() const { return _vector; }
//     std::size_t dimension() const { return _dim; }
//     Eigen::Matrix<double, N, 1>* mutable_vector() { return &_vector; }

// protected:
//     Eigen::Matrix<double, N, 1> _vector;
//     const std::size_t _dim;
// };

template <std::size_t N_STATE, std::size_t N_CONTROL>
class Node {
public:
    Node() = default;
    const Variable<N_STATE>& state() const { return _state; }
    const Variable<N_CONTROL>& control() const { return _control; }
    Variable<N_STATE>* mutable_state() { return &_state; }
    Variable<N_CONTROL>* mutable_control() { return &_control; }
    std::size_t step() const { return _step; }
    void set_step(std::size_t step) { _step = step; }
    double sample() const { return _sample; }
    void set_sample(double sample) { _sample = sample; }

private:
    Variable<N_STATE> _state;
    Variable<N_CONTROL> _control;
    std::size_t _step = 0;
    double _sample = 0.0;
};

template <std::size_t N_STATE, std::size_t N_CONTROL>
using Trajectory = std::vector<Node<N_STATE, N_CONTROL>>;

template <std::size_t N_STATE, std::size_t N_CONTROL>
struct DerivativesInfo {
    Eigen::Matrix<double, N_STATE, 1> lx = Eigen::MatrixXd::Zero(N_STATE, 1);
    Eigen::Matrix<double, N_CONTROL, 1> lu =  Eigen::MatrixXd::Zero(N_CONTROL, 1);
    Eigen::Matrix<double, N_STATE, N_STATE> lxx = Eigen::MatrixXd::Zero(N_STATE, N_STATE);
    Eigen::Matrix<double, N_CONTROL, N_CONTROL> luu = Eigen::MatrixXd::Zero(N_CONTROL, N_CONTROL);
    Eigen::Matrix<double, N_CONTROL, N_STATE> lux = Eigen::MatrixXd::Zero(N_CONTROL, N_STATE);
    Eigen::Matrix<double, N_STATE, N_STATE> fx = Eigen::MatrixXd::Zero(N_STATE, N_STATE);
    Eigen::Matrix<double, N_STATE, N_CONTROL> fu = Eigen::MatrixXd::Zero(N_STATE, N_CONTROL);
};

}