/**
 * @file piecewise_linear_kernel.cpp
 * @brief: The implementation of PiecewiseLinearKernel class.
 **/

//#include "ros/ros.h"
#include "piecewise_linear_kernel.h"
#include "common/base/log/include/log.h"

namespace acu {
namespace planning {
namespace math {

PiecewiseLinearKernel::PiecewiseLinearKernel(const uint32_t dimension,
                                             const double unit_segment)
    : dimension_(dimension),
      unit_segment_(unit_segment),
      kernel_matrix_(Eigen::MatrixXd::Zero(dimension_, dimension_)),
      offset_matrix_(Eigen::MatrixXd::Zero(dimension_, 1)) {}

void PiecewiseLinearKernel::AddRegularization(const double param) {
  Eigen::MatrixXd identity_matrix = kernel_matrix_ +=
      Eigen::MatrixXd::Identity(kernel_matrix_.rows(), kernel_matrix_.cols()) *
      param;
}

const Eigen::MatrixXd& PiecewiseLinearKernel::kernel_matrix() const {
  return kernel_matrix_;
}

const Eigen::MatrixXd& PiecewiseLinearKernel::offset_matrix() const {
  return offset_matrix_;
}

void PiecewiseLinearKernel::AddSecondOrderDerivativeMatrix(
    const double init_derivative, const double weight) {
  Eigen::MatrixXd second_derivative_matrix =
      Eigen::MatrixXd::Zero(dimension_, dimension_);
  for (size_t i = 0; i < dimension_; ++i) {
    if (i == 0) {
      second_derivative_matrix(0, 0) += 1.0;
    } else if (i == 1) {
      second_derivative_matrix(1, 1) += 1.0;
      second_derivative_matrix(0, 0) += 4.0;
      second_derivative_matrix(0, 1) += -2.0;
      second_derivative_matrix(1, 0) += -2.0;
    } else {
      second_derivative_matrix(i, i) += 1.0;
      second_derivative_matrix(i - 1, i - 1) += 4.0;
      second_derivative_matrix(i - 2, i - 2) += 1.0;
      second_derivative_matrix(i - 1, i) += -2.0;
      second_derivative_matrix(i, i - 1) += -2.0;
      second_derivative_matrix(i - 2, i - 1) += -2.0;
      second_derivative_matrix(i - 1, i - 2) += -2.0;
      second_derivative_matrix(i, i - 2) += 1.0;
      second_derivative_matrix(i - 2, i) += 1.0;
    }
  }
  second_derivative_matrix *= 2.0 * weight / std::pow(unit_segment_, 4);
  kernel_matrix_ += second_derivative_matrix;

  offset_matrix_(0, 0) +=
      -2.0 * weight * init_derivative / std::pow(unit_segment_, 3);
}

void PiecewiseLinearKernel::AddThirdOrderDerivativeMatrix(
    const double init_derivative, const double init_second_derivative,
    const double weight) {
  Eigen::MatrixXd jerk_matrix = Eigen::MatrixXd::Zero(dimension_, dimension_);
  for (size_t i = 0; i < dimension_; ++i) {
    if (i == 0) {
      jerk_matrix(0, 0) += 1.0;
    } else if (i == 1) {
      jerk_matrix(0, 0) += 9.0;
      jerk_matrix(1, 1) += 1.0;

      jerk_matrix(0, 1) += -3.0;
      jerk_matrix(1, 0) += -3.0;
    } else if (i == 2) {
      jerk_matrix(0, 0) += 9.0;
      jerk_matrix(1, 1) += 9.0;
      jerk_matrix(2, 2) += 1.0;

      jerk_matrix(1, 2) += -3.0;
      jerk_matrix(2, 1) += -3.0;
      jerk_matrix(0, 2) += 3.0;
      jerk_matrix(2, 0) += 3.0;

      jerk_matrix(0, 1) += -9.0;
      jerk_matrix(1, 0) += -9.0;
    } else {
      jerk_matrix(i - 3, i - 3) += 1.0;
      jerk_matrix(i - 2, i - 2) += 9.0;
      jerk_matrix(i - 1, i - 1) += 9.0;
      jerk_matrix(i, i) += 1.0;

      jerk_matrix(i - 1, i) += -3.0;
      jerk_matrix(i, i - 1) += -3.0;
      jerk_matrix(i - 2, i) += 3.0;
      jerk_matrix(i, i - 2) += 3.0;
      jerk_matrix(i - 3, i) += -1.0;
      jerk_matrix(i, i - 3) += -1.0;

      jerk_matrix(i - 2, i - 1) += -9.0;
      jerk_matrix(i - 1, i - 2) += -9.0;
      jerk_matrix(i - 3, i - 1) += 3.0;
      jerk_matrix(i - 1, i - 3) += 3.0;

      jerk_matrix(i - 3, i - 2) += -3.0;
      jerk_matrix(i - 2, i - 3) += -3.0;
    }
  }

  jerk_matrix *= 2.0 * weight / std::pow(unit_segment_, 6);
  kernel_matrix_ += jerk_matrix;

  const double quintic = std::pow(unit_segment_, 5);
  offset_matrix_(0, 0) +=
      -2.0 * weight *
      (init_derivative + init_second_derivative * unit_segment_) / quintic;
  offset_matrix_(0, 0) += -6.0 * weight * init_derivative / quintic;
  offset_matrix_(1, 0) += 2.0 * weight * init_derivative / quintic;
}

// reference line kernel
bool PiecewiseLinearKernel::AddReferenceLineKernelMatrix(
    const std::vector<uint32_t>& index_list,
    const std::vector<double>& pos_list, const double weight) {
  if (index_list.size() != pos_list.size()) {
    AINFO<<"index_list and pos_list must have equal size. index_list.size() = "
         << index_list.size() << ", pos_list.size() = " << pos_list.size();
    return false;
  }
  Eigen::MatrixXd ref_kernel = Eigen::MatrixXd::Zero(dimension_, dimension_);
  uint32_t index_list_size = index_list.size();
  for (uint32_t i = 0; i < index_list_size; ++i) {
    uint32_t index = index_list[i];
    ref_kernel(index, index) += 1.0;
    offset_matrix_(index, 0) += -2.0 * weight * pos_list[i];
  }
  kernel_matrix_ += ref_kernel * weight;
  return true;
}

}
}  // namespace planning
}  // namespace acu
