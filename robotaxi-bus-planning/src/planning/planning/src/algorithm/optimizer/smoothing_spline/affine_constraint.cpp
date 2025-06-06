/**
 * @file : affine_constraint.cpp
 **/

//#include "ros/ros.h"

#include "affine_constraint.h"
#include "common/base/log/include/log.h"
#include "src/execution/motionplan/common/datatype.h"

namespace acu {
namespace planning {
namespace math {
  
AffineConstraint::AffineConstraint(const bool is_equality)
    : is_equality_(is_equality) {}

AffineConstraint::AffineConstraint(const Eigen::MatrixXd& constraint_matrix,
                                   const Eigen::MatrixXd& constraint_boundary,
                                   const bool is_equality)
    : constraint_matrix_(constraint_matrix),
      constraint_boundary_(constraint_boundary),
      is_equality_(is_equality) {
  CHECK_EQ(constraint_boundary.rows(), constraint_matrix.rows());
}

void AffineConstraint::SetIsEquality(const double is_equality) {
  is_equality_ = is_equality;
}

const Eigen::MatrixXd& AffineConstraint::constraint_matrix() const {
  return constraint_matrix_;
}

const Eigen::MatrixXd& AffineConstraint::constraint_boundary() const {
  return constraint_boundary_;
}

bool AffineConstraint::AddConstraint(
    const Eigen::MatrixXd& constraint_matrix,
    const Eigen::MatrixXd& constraint_boundary) {
  if (constraint_matrix.rows() != constraint_boundary.rows()) {
    AWARN_IF(FLAGS_enable_debug_motion) << "Fail to add constraint because constraint matrix rows != "
              "constraint boundary rows.";
    AINFO_IF(FLAGS_enable_debug_motion) << "constraint matrix rows = " << constraint_matrix.rows();
    AINFO_IF(FLAGS_enable_debug_motion) << "constraint boundary rows = " << constraint_boundary.rows();
    return false;
  }

  if (constraint_matrix_.rows() == 0) {
    constraint_matrix_ = constraint_matrix;
    constraint_boundary_ = constraint_boundary;
    return true;
  }
  if (constraint_matrix_.cols() != constraint_matrix.cols()) {
    AWARN_IF(FLAGS_enable_debug_motion) << "constraint_matrix_ cols and constraint_matrix cols do not match.";
    AINFO_IF(FLAGS_enable_debug_motion) << "constraint_matrix_.cols() = " << constraint_matrix_.cols();
    AINFO_IF(FLAGS_enable_debug_motion) << "constraint_matrix.cols() = " << constraint_matrix.cols();
    return false;
  }
  if (constraint_boundary.cols() != 1) {
    AWARN_IF(FLAGS_enable_debug_motion) << "constraint_boundary.cols() should be 1.";
    return false;
  }

  Eigen::MatrixXd n_matrix(constraint_matrix_.rows() + constraint_matrix.rows(),
                           constraint_matrix_.cols());
  Eigen::MatrixXd n_boundary(
      constraint_boundary_.rows() + constraint_boundary.rows(), 1);

  n_matrix << constraint_matrix_, constraint_matrix;
  n_boundary << constraint_boundary_, constraint_boundary;
  constraint_matrix_ = n_matrix;
  constraint_boundary_ = n_boundary;
  return true;
}

}
}  // namespace planning
}  // namespace acu
