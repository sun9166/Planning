/**
 * @file path_info.cpp
 **/

#include <vector>
//#include "ros/ros.h"
#include <algorithm>

#include "path_info.h"
#include "common/util/util.h"
#include "common/util/string_util.h"
#include "common/base/log/include/log.h"
#include "src/execution/motionplan/common/planning_gflags.h"
#include "src/algorithm/math_util/coordinate_conversion/cartesian_frenet_conversion.h"

namespace acu {
namespace planning {  

using acu::common::SLPoint;

bool PathInfo::SetDiscretizedPath(const DiscretizedPath &path, const bool update_history) {
  if (reference_line_ == nullptr) {
    AWARN_IF(FLAGS_enable_debug_motion) << "Should NOT set discretized path when reference line is nullptr. "
              "Please set reference line first.";
    discretized_path_ = path; 
    return false;
  }
  discretized_path_ = path;
  if (!XYToSL(discretized_path_, &frenet_path_)) {
    AWARN_IF(FLAGS_enable_debug_motion) << "Fail to transfer discretized path to frenet path.";
    return false;
  }
  DCHECK_EQ(discretized_path_.size(), frenet_path_.size());
  if (update_history || path_data_history_.empty()) {
    path_data_history_.push_back(std::make_pair(discretized_path_, frenet_path_));
  } else {
    path_data_history_.back() = std::make_pair(discretized_path_, frenet_path_);
  }
  return true;
}

bool PathInfo::SetFrenetPath(const FrenetFramePath &frenet_path) {
  if (reference_line_ == nullptr) {
    AWARN_IF(FLAGS_enable_debug_motion) << "Should NOT set frenet path when reference line is nullptr. "
              "Please set reference line first.";
    return false;
  }
  frenet_path_ = frenet_path;
  if (!SLToXY(frenet_path_, &discretized_path_)) {
    AWARN_IF(FLAGS_enable_debug_motion) << "Fail to transfer frenet path to discretized path.";
    return false;
  }
  DCHECK_EQ(discretized_path_.size(), frenet_path_.size());
  path_data_history_.push_back(std::make_pair(discretized_path_, frenet_path_));
  return true;
}

const DiscretizedPath &PathInfo::discretized_path() const {
  return discretized_path_;
}
DiscretizedPath* PathInfo::mutable_discretized_path() {
  return &discretized_path_;
}

bool PathInfo::Empty() const {
  return discretized_path_.empty() && frenet_path_.empty();
}

std::list<std::pair<DiscretizedPath, FrenetFramePath>>
    &PathInfo::path_data_history() {
  return path_data_history_;
}

const FrenetFramePath &PathInfo::frenet_frame_path() const {
  return frenet_path_;
}

void PathInfo::SetReferenceLine(const ReferenceLine *reference_line) {
  Clear();
  reference_line_ = reference_line;
}

bool PathInfo::GetPathPointWithPathS(
    const double s, common::PathPoint *const path_point) const {
  *path_point = discretized_path_.Evaluate(s);
  return true;
}

bool PathInfo::GetPathPointWithRefS(const double ref_s,
                                    common::PathPoint *const path_point) const {
  DCHECK_NOTNULL(reference_line_);
  DCHECK_NOTNULL(path_point);
  DCHECK_EQ(discretized_path_.size(), frenet_path_.size());
  if (ref_s < 0) {
    AWARN_IF(FLAGS_enable_debug_motion) << "ref_s[" << ref_s << "] should be > 0";
    return false;
  }
  if (ref_s > frenet_path_.back().s()) {
    AWARN_IF(FLAGS_enable_debug_motion) << "[PathInfo/GetPathPointWithRefS]ref_s is larger than the length of frenet_path_ length ["
           << frenet_path_.back().s() << "].";
    return false;
  }

  uint32_t index = 0;
  const double kDistanceEpsilon = 1e-3;
  uint32_t frenet_path_size = (uint32_t)frenet_path_.size();
  for (uint32_t i = 0; i + 1 < frenet_path_size; ++i) {
    if (fabs(ref_s - frenet_path_.at(i).s()) < kDistanceEpsilon) {
      path_point->CopyFrom(discretized_path_.at(i));
      return true;
    }
    if (frenet_path_.at(i).s() < ref_s && ref_s <= frenet_path_.at(i + 1).s()) {
      index = i;
      break;
    }
  }
  double r = (ref_s - frenet_path_.at(index).s()) /
             (frenet_path_.at(index + 1).s() - frenet_path_.at(index).s());

  const double discretized_path_s = discretized_path_.at(index).s() +
                                    r * (discretized_path_.at(index + 1).s() -
                                         discretized_path_.at(index).s());
  path_point->CopyFrom(discretized_path_.Evaluate(discretized_path_s));

  return true;
}

void PathInfo::Clear() {
  discretized_path_.clear();
  frenet_path_.clear();
  reference_line_ = nullptr;
}

bool PathInfo::SLToXY(const FrenetFramePath &frenet_path,
                      DiscretizedPath *const discretized_path) {
  DCHECK_NOTNULL(discretized_path);
  std::vector<common::PathPoint> path_points;
  for (const common::FrenetFramePoint &frenet_point : frenet_path) {
    common::SLPoint sl_point;
    common::math::Vec2d cartesian_point;
    sl_point.set_s(frenet_point.s());
    sl_point.set_l(frenet_point.l());
    //1.调用reference_line_->SLToXY进行坐标转化,得到xy坐标，存储在cartesian_point
    if (!reference_line_->SLToXY(sl_point, &cartesian_point)) {
      AWARN_IF(FLAGS_enable_debug_motion) << "Fail to convert sl point to xy point";
      return false;
    }
    //2.由s匹配到参考线上的点，该点的信息包含heading，kappa,l,dl,再根据匹配的点的信息进行计算得到theta,kappa
    ReferencePoint ref_point =
        reference_line_->GetReferencePoint(frenet_point.s());
    double theta = math::CalculateCartesianTheta(
        ref_point.heading(), ref_point.kappa(), frenet_point.l(),
        frenet_point.dl());    
    double kappa = math::CalculateCartesianKappa(//计算曲率
        ref_point.kappa(), ref_point.dkappa(), frenet_point.l(),
        frenet_point.dl(), frenet_point.ddl());


    common::PathPoint path_point = common::util::MakePathPoint(
        cartesian_point.x(), cartesian_point.y(), 0.0, theta, kappa, 0.0, 0.0);
    //1.第0个点s设置为零，第一个点s等于0+该点到前一个点的距离，以此类推，
    if (path_points.empty()) {
      path_point.set_s(0.0);
      path_point.set_dkappa(0.0);
    } else {
      common::math::Vec2d last(path_points.back().x(), path_points.back().y());
      common::math::Vec2d current(path_point.x(), path_point.y());
      double distance = (last - current).Length();
      path_point.set_s(path_points.back().s() + distance);
      path_point.set_dkappa((path_point.kappa() - path_points.back().kappa()) /
                            distance);
    }
    
    // AWARN_IF(FLAGS_enable_debug_speedplan && fabs(path_point.kappa()) > 0.01)<<"s = "<<path_point.s()<<", kappa = "
    //              <<path_point.kappa()<<", ref_kappa = "<<ref_point.kappa()<<", l = "<<frenet_point.l();
    
    path_points.push_back(std::move(path_point));
  }
  *discretized_path = DiscretizedPath(std::move(path_points));

  return true;
}

bool PathInfo::XYToSL(const DiscretizedPath &discretized_path,
                      FrenetFramePath *const frenet_path) {
  CHECK_NOTNULL(frenet_path);
  CHECK_NOTNULL(reference_line_);
  std::vector<common::FrenetFramePoint> frenet_frame_points;
  const double max_len = reference_line_->Length();
  for (const auto &path_point : discretized_path) {
    common::FrenetFramePoint frenet_point =
        reference_line_->GetFrenetPoint(path_point);
    if (!frenet_point.has_s()) {
      SLPoint sl_point;
      if (!reference_line_->XYToSL({path_point.x(), path_point.y()},
                                   &sl_point)) {
        AWARN_IF(FLAGS_enable_debug_motion) << "Fail to transfer cartesian point to frenet point.";
        return false;
      }
      common::FrenetFramePoint frenet_point;
      // NOTICE: does not set dl and ddl here. Add if needed.
      frenet_point.set_s(std::max(0.0, std::min(sl_point.s(), max_len)));
      frenet_point.set_l(sl_point.l());
      frenet_frame_points.push_back(std::move(frenet_point));
      continue;
    }
    frenet_point.set_s(std::max(0.0, std::min(frenet_point.s(), max_len)));
    frenet_frame_points.push_back(std::move(frenet_point));
  }
  *frenet_path = FrenetFramePath(std::move(frenet_frame_points));
  return true;
}

bool PathInfo::LeftTrimWithRefS(const common::FrenetFramePoint &frenet_point) {
  CHECK_NOTNULL(reference_line_);
  std::vector<common::FrenetFramePoint> frenet_frame_points;
  frenet_frame_points.emplace_back(frenet_point);

  for (const common::FrenetFramePoint fp : frenet_path_) {
    if (std::fabs(fp.s() - frenet_point.s()) < 1e-6) {
      continue;
    }
    if (fp.s() > frenet_point.s()) {
      frenet_frame_points.push_back(std::move(fp));
    }
  }
  const FrenetFramePath frenet_path =
      FrenetFramePath(std::move(frenet_frame_points));
  SetFrenetPath(frenet_path);
  return true;
}

bool PathInfo::UpdateFrenetFramePath(const ReferenceLine *reference_line) {
  reference_line_ = reference_line;
  return SetDiscretizedPath(discretized_path_);
}

bool PathInfo::UpdateDiscretizedPath(const std::vector<common::TrajectoryPoint>& path) {
  if (path.empty() || path.size() > discretized_path_.size()) return false;

  for (size_t i = 0; i < path.size();++i) {
    discretized_path_.at(i).set_x(path.at(i).path_point().x());
    discretized_path_.at(i).set_y(path.at(i).path_point().y());
  }
  return true;
}

void PathInfo::set_path_label(const std::string &label) { path_label_ = label; }

const std::string &PathInfo::path_label() const { return path_label_; }

}  // namespace planning
}  // namespace acu
