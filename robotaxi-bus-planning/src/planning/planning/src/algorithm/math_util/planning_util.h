/**
 * @file planning_util.h
 * @brief Some util functions.
 */

#pragma once

#include <algorithm>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>
#include "Eigen/Dense"
#include "Eigen/SVD"
#include "common/math/vec2d.h"
#include "global_define.h"
namespace acu {
namespace planning {
namespace math {
/**
 * calculate the distance beteween Point u and Point v, which are all have
 * member function x() and y() in XY dimension.
 * @param u one point that has member function x() and y().
 * @param b one point that has member function x() and y().
 * @return sqrt((u.x-v.x)^2 + (u.y-v.y)^2), i.e., the Euclid distance on XY
 * dimension.
 */
template <typename U, typename V>
double DistanceXY(const U& u, const V& v) {
  return std::hypot(u.x() - v.x(), u.y() - v.y());
}

/**
 * Check if two points u and v are the same point on XY dimension.
 * @param u one point that has member function x() and y().
 * @param v one point that has member function x() and y().
 * @return sqrt((u.x-v.x)^2 + (u.y-v.y)^2) < epsilon, i.e., the Euclid distance
 * on XY dimension.
 */
template <typename U, typename V>
bool SamePointXY(const U& u, const V& v) {
  constexpr double kMathEpsilonSqr = 1e-8 * 1e-8;
  return (u.x() - v.x()) * (u.x() - v.x()) < kMathEpsilonSqr &&
         (u.y() - v.y()) * (u.y() - v.y()) < kMathEpsilonSqr;
}


template <typename T, int M, int N, typename D>
void DenseToCSCMatrix(const Eigen::Matrix<T, M, N> &dense_matrix,
                      std::vector<T> *data, std::vector<D> *indices,
                      std::vector<D> *indptr) {
  constexpr double epsilon = 1e-9;
  int data_count = 0;
  for (int c = 0; c < dense_matrix.cols(); ++c) {
    indptr->emplace_back(data_count);
    for (int r = 0; r < dense_matrix.rows(); ++r) {
      if (std::fabs(dense_matrix(r, c)) < epsilon) {
        continue;
      }
      data->emplace_back(dense_matrix(r, c));
      ++data_count;
      indices->emplace_back(r);
    }
  }
  indptr->emplace_back(data_count);
}

}
}
}