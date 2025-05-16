/**
 * @file reference_line_provider.cpp
 * @brief Implementation of the class ReferenceLineProvider.
 */

//#include "ros/ros.h"
#include <algorithm>
#include <limits>
#include <utility>

#include "common/util/file.h"
#include "common/math/math_utils.h"
#include "reference_line_provider.h"
#include "src/execution/motionplan/common/path/path.h"
#include "src/execution/motionplan/common/datatype.h"
#include "src/execution/motionplan/common/planning_gflags.h"
#include "src/execution/motionplan/behavior_parser/behavior_parser.h"
#include "src/execution/motionplan/common/new_path_manager.h"
#include "src/execution/motionplan/common/ego_info.h"
#include "src/algorithm/math_util/planning_util.h"

/**
 * @namespace acu::planning
 * @brief acu::planning
 */
namespace acu {
namespace planning {

using acu::common::math::AngleDiff;
using acu::common::math::Vec2d;

ReferenceLineProvider::ReferenceLineProvider() {
  //std::string planning_pkg_path;
  //acu::common::util::getPackagePath("planning", planning_pkg_path);
  std::string smoother_config_filename = "/map/work/config/planning_node_config/motionplan_conf/discrete_points_smoother_config.config";
  //smoother_config_filename.assign(common::util::GetAbsolutePath(planning_pkg_path, smoother_config_filename));
  CHECK(acu::common::util::GetProtoFromFile(smoother_config_filename,
        &smoother_config_));
  smoother_.reset(new DiscretePointsReferenceLineSmoother(smoother_config_));
  is_initialized_ = true;
}  

ReferenceLineProvider::~ReferenceLineProvider() {
  Stop();
}


bool ReferenceLineProvider::Start() {
  return true;
}

void ReferenceLineProvider::Stop() {
  reference_lines_.clear();
  std::queue<std::list<ReferenceLine>> empty_queue;
  reference_line_history_ = empty_queue;
  last_reference_line_type_ = 0;
}

void ReferenceLineProvider::UpdateVehicleState(
    const VehicleState &vehicle_state) {
  vehicle_state_ = vehicle_state;
}

void ReferenceLineProvider::UpdateReferenceLine(
  const std::list<ReferenceLine> &reference_lines) {
  if (reference_lines_.size() != reference_lines.size()) {
    reference_lines_ = reference_lines;
  } else {
    auto internal_iter = reference_lines_.begin();
    for (auto iter = reference_lines.begin();
         iter != reference_lines.end() &&
         internal_iter != reference_lines_.end();
         ++iter, ++internal_iter) {
      if (iter->reference_points().empty()) {
        *internal_iter = *iter;
        AINFO_IF(FLAGS_enable_debug_motion)<<"update reference_line .";
        continue;
      }
      if (acu::planning::math::SamePointXY(
            iter->reference_points().front(),
            internal_iter->reference_points().front()) &&
          acu::planning::math::SamePointXY(iter->reference_points().back(),
                      internal_iter->reference_points().back()) &&
          std::fabs(iter->Length() - internal_iter->Length()) <
          common::math::kMathEpsilon) {
        continue;
      }
      *internal_iter = *iter;
      AINFO_IF(FLAGS_enable_debug_motion)<<"update reference_line .";
    }
  }

  // update history
  reference_line_history_.push(reference_lines_);
  constexpr int kMaxHistoryNum = 3;
  if (reference_line_history_.size() > kMaxHistoryNum) {
    reference_line_history_.pop();
  }
}

bool ReferenceLineProvider::GetReferenceLines(std::list<ReferenceLine> *reference_lines,
          const ReferenceLineFrame* ref_line,const bool pathplan_status_last_frame) {
  bool is_local_path = false;
  bool is_change_path = false;
  smoothed_status_ = 0;
  if (BehaviorParser::instance()->target_reference_line_id() 
      != BehaviorParser::instance()->current_reference_line_id()) {
    is_change_path = true;
  }
  if (FLAGS_enable_smooth_reference_line) {
    if (BehaviorParser::instance()->target_reference_line_change()) {
      reference_lines->emplace_back();
      AINFO_IF(FLAGS_enable_debug_motion)<<"target_reference_line_change, need smooth. target id = "<<ref_line->reference_lane_id; 
      if (!SmoothReferenceLine(ReferenceLine(hdmap::Path(ref_line),is_change_path,
                           ref_line->mapinfo.expected_speeds), &reference_lines->back())) {
        smoothed_status_ = 3; 
        AINFO_IF(FLAGS_enable_debug_motion)<<"smooth reference_line failed, use raw reference_line.";
      }
      smoothed_status_ = 1; 
      last_reference_line_type_ = 0;
    } else if (
      EgoInfo::instance()->vehicle_state().driving_mode != 0) {
      reference_lines->emplace_back();
      AINFO_IF(FLAGS_enable_debug_motion)<<"last pathplan failed, need smooth.";
      if (!SmoothReferenceLine(ReferenceLine(hdmap::Path(ref_line),is_change_path,
                           ref_line->mapinfo.expected_speeds), &reference_lines->back())) {
        smoothed_status_ = 3; 
        AINFO_IF(FLAGS_enable_debug_motion)<<"smooth reference_line failed, use raw reference_line.";
      } 
      smoothed_status_ = 1; 
      last_reference_line_type_ = 0;
    } else {
      bool need_accuracy_control = false;
      need_accuracy_control = !ref_line->mapinfo.lane_types.empty()
                && ref_line->mapinfo.lane_types.front().second == eLaneType::PARKING_LANE
                  || ref_line->mapinfo.dis_to_end < 30.0
                     &&(ref_line->mapinfo.dis2missionpoint - ref_line->mapinfo.dis_to_end) > 2.5;
      if (need_accuracy_control) {
        reference_lines->emplace_back(ReferenceLine(hdmap::Path(ref_line),is_change_path,
                             ref_line->mapinfo.expected_speeds));
        AINFO_IF(FLAGS_enable_debug_motion)<<"closed to parking road ,not need smooth~~";
        last_reference_line_type_ = 0;
      } else if (NewPathManager::instance()->is_new_stithing_path()) {//new local path do not be smoothed
        last_reference_line_type_ = 1;
        reference_lines->emplace_back(ReferenceLine(hdmap::Path(ref_line),is_change_path,
                           ref_line->mapinfo.expected_speeds));
        AINFO_IF(FLAGS_enable_debug_motion)<<"is new path .do not smooth it.";
      } else {
        reference_lines->emplace_back();
        if ((BehaviorParser::instance()->has_decision_lateral_behavior() 
          && ref_line->reference_lane_id == BehaviorParser::instance()->target_reference_line_id())
          || last_reference_line_type_ > 0 ) {
          AINFO_IF(FLAGS_enable_debug_motion)<<"reference line change . need smooth it.";
          if (!SmoothReferenceLine(ReferenceLine(hdmap::Path(ref_line),is_change_path,
                               ref_line->mapinfo.expected_speeds), &reference_lines->back())) {
            smoothed_status_ = 3; 
            AINFO_IF(FLAGS_enable_debug_motion)<<"smooth reference_line failed, use raw reference_line.";
          }
          smoothed_status_ = 1;  
          last_reference_line_type_ = 0;
        } else {
          AINFO_IF(FLAGS_enable_debug_motion)<<"reference line keeping . extend it.";
          if (!ExtendReferenceLine(vehicle_state_, ReferenceLine(hdmap::Path(ref_line),is_change_path,
                                 ref_line->mapinfo.expected_speeds), &reference_lines->back(),
                                 ref_line->mapinfo.expected_speeds)) {
            smoothed_status_ = 4; 
            AERROR_IF(FLAGS_enable_debug_motion) << "Failed to extend reference line, use raw reference_line";
          }
          smoothed_status_ = 2; 
          last_reference_line_type_ = 0;
        }
      }
    }
  } else {
    reference_lines->emplace_back(ReferenceLine(hdmap::Path(ref_line),is_change_path,
                           ref_line->mapinfo.expected_speeds));
  }
  
  UpdateReferenceLine(*reference_lines);
  return true;
}

bool ReferenceLineProvider::SmoothReferenceLine(
    const ReferenceLine &raw_reference_line, ReferenceLine *reference_line) {
  if (!FLAGS_enable_smooth_reference_line) {
    *reference_line = raw_reference_line;
    return true;
  }
  // generate anchor points:
  std::vector<AnchorPoint> anchor_points;
  GetAnchorPoints(raw_reference_line, &anchor_points);
  smoother_->SetAnchorPoints(anchor_points);
  if (!smoother_->Smooth(raw_reference_line, reference_line)) {
    *reference_line = raw_reference_line;
    AERROR_IF(FLAGS_enable_debug_motion) << "Failed to smooth reference line with anchor points";
    return false;
  }
  if (!IsReferenceLineSmoothValid(raw_reference_line, *reference_line)) {
    *reference_line = raw_reference_line;
    AERROR_IF(FLAGS_enable_debug_motion) << "The smoothed reference line error is too large";
    return false;
  }
  return true;
}

AnchorPoint ReferenceLineProvider::GetAnchorPoint(
    const ReferenceLine &reference_line, double s) const {
  AnchorPoint anchor;
  anchor.longitudinal_bound = smoother_config_.longitudinal_boundary_bound();
  auto ref_point = reference_line.GetReferencePoint(s);
  if (ref_point.lane_waypoints().empty()) {
    anchor.path_point = ref_point.ToPathPoint(s);
    anchor.lateral_bound = smoother_config_.max_lateral_boundary_bound();//与参考点的横向偏移量约束边界
    return anchor;
  }
}

void ReferenceLineProvider::GetAnchorPoints(
    const ReferenceLine &reference_line,
    std::vector<AnchorPoint> *anchor_points) const {
  //以配置文件确定的间隔max_constraint_interval作为每个AnchorPoints之间的间隔,
  CHECK_NOTNULL(anchor_points);
  const double interval = smoother_config_.max_constraint_interval();
  int num_of_anchors =
      std::max(2, static_cast<int>(reference_line.Length() / interval + 0.5));
  std::vector<double> anchor_s;
  common::util::uniform_slice(0.0, reference_line.Length(), num_of_anchors - 1,
                              &anchor_s);
  for (const double s : anchor_s) {
    AnchorPoint anchor = GetAnchorPoint(reference_line, s);
    anchor_points->emplace_back(anchor);
  }
  //约束首尾点与参考点一致
  anchor_points->front().longitudinal_bound = 1e-6;
  anchor_points->front().lateral_bound = 1e-6;
  anchor_points->front().enforced = true;
  anchor_points->back().longitudinal_bound = 1e-6;
  anchor_points->back().lateral_bound = 1e-6;
  anchor_points->back().enforced = true;
}

bool ReferenceLineProvider::SmoothPrefixedReferenceLine(
    const ReferenceLine &prefix_ref, const ReferenceLine &raw_ref,
    ReferenceLine *reference_line) {
  if (!FLAGS_enable_smooth_reference_line) {
    *reference_line = raw_ref;
    return true;
  }
  // generate anchor points:
  std::vector<AnchorPoint> anchor_points;
  GetAnchorPoints(raw_ref, &anchor_points);//从新的参考线找到锚点
  // modify anchor points based on prefix_ref
  // 约束后续平滑的点不能偏离原来历史平滑过的参考线上的点.set lateral_bound = 1e-6
  double length = 50.0;
  int count = 0;
  std::vector<ReferencePoint> prefix_points;

  auto it = anchor_points.begin();
  while(it != anchor_points.end()) {
    common::SLPoint sl_point;
    if (!prefix_ref.XYToSL(it->path_point, &sl_point)) {
      AINFO_IF(FLAGS_enable_debug_motion)<<"continue, count = "<<count;
      it = anchor_points.erase(it);
      continue;
    }
    if (sl_point.s() < -1.0 || sl_point.s() > prefix_ref.Length()) {
      it = anchor_points.erase(it);
      continue;
    }
    if (sl_point.s() > length + 10) {
      break;
    } else {
      auto prefix_ref_point = prefix_ref.GetNearestReferencePoint(sl_point.s());
      it->path_point.set_x(prefix_ref_point.x());
      it->path_point.set_y(prefix_ref_point.y());
      it->path_point.set_z(0.0);
      it->path_point.set_theta(prefix_ref_point.heading());
      it->longitudinal_bound = 1e-6;
      it->lateral_bound = 1e-6;
      it->enforced = true;

      if (sl_point.s() > length) {
        it->lateral_bound = 1.5;
      }
      prefix_points.push_back(prefix_ref_point);
    }
    it++;
    count++; 
  }
  for (size_t i = count; i < anchor_points.size();++i) {
    if (i - count > 40) {
      break;
    }
    common::SLPoint sl_point;
    if (!prefix_ref.XYToSL(anchor_points.at(i).path_point, &sl_point)) {
      continue;
    }
    if (sl_point.s() < 0 || sl_point.s() > prefix_ref.Length()) {
      continue;
    }
    auto prefix_ref_point = prefix_ref.GetNearestReferencePoint(sl_point.s());
    prefix_points.push_back(prefix_ref_point);
  }
  if (anchor_points.empty()) {
    AERROR_IF(FLAGS_enable_debug_motion) << "anchor_points is empty";
    *reference_line = raw_ref;
    return false;
  }

  smoother_->SetAnchorPoints(anchor_points);
  if (!smoother_->Smooth(raw_ref, reference_line)) {
    AERROR_IF(FLAGS_enable_debug_motion) << "Failed to smooth prefixed reference line with anchor points";
    return false;
  }

  if (!IsReferenceLineSmoothValid(raw_ref, *reference_line)) {
    AERROR_IF(FLAGS_enable_debug_motion) << "The smoothed reference line error is too large";
    *reference_line = raw_ref;
    return false;
  }
  return true;
}

bool ReferenceLineProvider::ExtendReferenceLine(const VehicleState& state,
                           const ReferenceLine& raw_reference_line,
                           ReferenceLine* reference_line,
                           const vector<pair<double, double> >& speed_limits) {
  if (reference_lines_.empty()) {
    AINFO_IF(FLAGS_enable_debug_motion)<<"no prev_reference_line . ";
    return SmoothReferenceLine(raw_reference_line,reference_line);
  }

  if (FLAGS_use_multi_ref_line) {//TODO 多条参考线时先不做平滑了
    *reference_line = raw_reference_line;
  }

  common::SLPoint sl_point;
  Vec2d car_pos(state.x, state.y);
  reference_lines_.front().XYToSL(car_pos,&sl_point);
  AINFO_IF(FLAGS_enable_debug_motion)<<"vehicle s = "<<sl_point.s();
  const double prev_reference_line_length = reference_lines_.front().Length();
  const double remain_s = prev_reference_line_length - sl_point.s();
  const double length_diff = raw_reference_line.Length() - remain_s;
  if (length_diff < FLAGS_look_forward_extend_distance) {
      *reference_line = reference_lines_.front();
      AINFO_IF(FLAGS_enable_debug_motion) << "Reference line length diff " << length_diff
             << ", which is less than required : "<<FLAGS_look_forward_extend_distance 
             << " and no need to extend";
      return Shrink(sl_point, reference_line,speed_limits,raw_reference_line);
  }
 
  auto prev_ref = reference_lines_.begin();
  if (!SmoothPrefixedReferenceLine(*prev_ref, raw_reference_line, reference_line)) {//用旧的和新的参考线进行拼接平滑生成新的参考线reference_line
    *reference_line = raw_reference_line;
    AWARN_IF(FLAGS_enable_debug_motion) << "Failed to smooth forward shifted reference line";
    return false;
  }

  common::SLPoint sl;
  if (!reference_line->XYToSL(car_pos, &sl)) {
    AWARN_IF(FLAGS_enable_debug_motion) << "Failed to project point: " << car_pos.DebugString()
          << " to stitched reference line";
  }
  return Shrink(sl, reference_line,speed_limits,raw_reference_line);
}

bool ReferenceLineProvider::Shrink(const common::SLPoint &sl,
                                   ReferenceLine *reference_line,
                                   const vector<pair<double, double> >& speed_limits,
                                   const ReferenceLine& raw_reference_line) {
  static constexpr double kMaxHeadingDiff = M_PI * 5.0 / 6.0;
  // shrink reference line
  double new_backward_distance = 1.0;//保留后向1m的参考线，以防计算sl出问题
  double new_forward_distance = reference_line->Length() - sl.s();//裁剪掉当前位置后面部分 之后的参考线长度
  bool need_shrink = false;
  const double look_backward_distance = 0.5;

  if (sl.s() > look_backward_distance) {
    AINFO_IF(FLAGS_enable_debug_motion) << "reference line back side is " << sl.s()
           << ", shrink reference line: origin length: "
           << reference_line->Length();
    need_shrink = true;
  }
  if (need_shrink) {
    if (!reference_line->Segment(sl.s(), 
                        raw_reference_line.GetMapPath().raw_refrence_line(),new_backward_distance,
                        new_forward_distance)) {
      AWARN_IF(FLAGS_enable_debug_motion) << "Failed to shrink reference line";
    }
  }
  reference_line->UpdateSpeedLimit(speed_limits);
  return true;
}

bool ReferenceLineProvider::IsReferenceLineSmoothValid(
  const ReferenceLine &raw, const ReferenceLine &smoothed) const {
  constexpr double kReferenceLineDiffCheckStep = 10.0;
  for (double s = 0.0; s < smoothed.Length();
       s += kReferenceLineDiffCheckStep) {
    auto xy_new = smoothed.GetReferencePoint(s);
    auto xy_raw = raw.GetReferencePoint(s);

    common::SLPoint sl_new;
    if (!raw.XYToSL(xy_new, &sl_new)) {
      AWARN_IF(FLAGS_enable_debug_motion)<< "Fail to change xy point on smoothed reference line to sl "
                      << "point respect to raw reference line.";
      return false;
    }

    const double diff = std::fabs(sl_new.l());
    if (diff > FLAGS_smoothed_reference_line_max_diff) {
      AINFO_IF(FLAGS_enable_debug_motion)<< "Fail to provide reference line because too large diff "
                      << "between smoothed and raw reference lines. diff: "
                      << diff;
      return false;
    }
    double kappa_diff = fabs(xy_new.kappa()) - fabs(xy_raw.kappa());
    double check_length_min = raw.GetMapPath().property_length();
    if (s > check_length_min && kappa_diff > FLAGS_smoothed_reference_line_max_kappa_diff) {
      AINFO_IF(FLAGS_enable_debug_motion)<< "Fail to provide reference line because kappa of smoothed reference point >"
                      << "that of raw reference point. diff: "
                      << kappa_diff <<", ("<<xy_new.kappa()<<", "<<xy_raw.kappa()<<"), s = "
                      <<s<<", check_length_min = "<<check_length_min;
      return false;
    }
  }
  return true;
}

}  // namespace planning
}  // namespace acu
