/**
 * @file new_path_manager.cpp
 **/

#include "new_path_manager.h"
#include "common/base/log/include/log.h"
#include "src/execution/motionplan/common/ego_info.h"
#include "src/execution/motionplan/common/scenario_context.h"
#include "datapool/include/data_pool.h"

#define FLAGS_lane_width 3.50
namespace acu {
namespace planning {

NewPathManager::NewPathManager() {
  
}

void NewPathManager::Clear() {
  new_local_path_.clear();
  is_new_stithing_path_ = false;
  new_path_counter_ = 0;
  stitch_state_ = true;
}

void NewPathManager::ResetIsNewStithingPath() {
  is_new_stithing_path_ = false;
  new_path_counter_ = 0;
  stitch_state_ = true;
}

bool NewPathManager::UpdateNewPath(const ReferenceLineFrame& raw_reference_line,
                                   const int& target_reference_line_id) {
  target_reference_line_id_ = target_reference_line_id;
  is_new_stithing_path_ = false;
  stitch_state_ = true;
  if (new_local_path_.empty()) {
    new_path_counter_ = 0;
    if (raw_reference_line.mapinfo.path_points.empty()) { 
      return false;
    } else {
      new_local_path_.clear();
      new_local_path_ = raw_reference_line.mapinfo.path_points;
      DataPool* DP = DataPool::Instance();
      DP->GetMainDataRef().stitching_point.set_x(0.0); 
      DP->GetMainDataRef().stitching_point.set_y(0.0);
      return true;
    }
  }

  bool angle_limit = false;//TODO for car back
  geometry::Site ego_point;
  ego_point.xg = EgoInfo::instance()->vehicle_state().x;
  ego_point.yg = EgoInfo::instance()->vehicle_state().y;
  ego_point.globalangle = EgoInfo::instance()->vehicle_state().heading / 3.1615926 * 180;
  int start_index = GetNearestIndex(new_local_path_, ego_point, angle_limit);
  if (start_index < 0 || start_index > new_local_path_.size()) {
    new_path_counter_ = 0;
    AWARN_IF(FLAGS_enable_debug_motion)<<"Can't find nearest point of location!!";
    return false;
  }
  for (int i = 0; i < start_index; i++) {
    new_local_path_.erase(new_local_path_.begin());
  }
  
  // update length;
  if (new_local_path_.empty()) {
    new_path_counter_ = 0;
    return false ;
  }
  double length = new_local_path_.front().length;
  for (auto& single_point : new_local_path_) { // 将motion_path中的点放入local_front_points // TODO 坐标转换
    single_point.length = single_point.length - length;
  }

  // stitch local & target path
  
  if (IsNewPath(new_local_path_)) {
    new_path_counter_ ++;
    is_new_stithing_path_ = true;
    if (!is_pull_over_path()) {
      auto status = Stitch(raw_reference_line);
      if (!status) {
        AWARN_IF(FLAGS_enable_debug_motion)<< "local path stithing failed.";
        is_new_stithing_path_ = false;
      }
    }
  } else if (is_pull_over_path() && !new_local_path_.empty()) {//pull over path may not satisfy IsNewPath
    return true;
  } else {
    AINFO_IF(FLAGS_enable_debug_motion)<<"is not new path ,use input raw reference_line .";
    new_path_counter_ = 0;
    is_new_stithing_path_ = false;
    new_local_path_.clear();
    new_local_path_ = raw_reference_line.mapinfo.path_points; 
    DataPool* DP = DataPool::Instance();
    DP->GetMainDataRef().stitching_point.set_x(0.0); 
    DP->GetMainDataRef().stitching_point.set_y(0.0); 
  }
  if (new_path_counter_ > 3) {
    new_path_counter_ = 3;
  }

  return true;
}



void NewPathManager::SetNewPath(const geometry::SiteVec& points, const NewPathType& path_type) {
  new_local_path_.clear();
  new_local_path_ = points;
  type_ = path_type;
  AINFO_IF(FLAGS_enable_debug_motion)<<"new path type = "<<(int)type_;
  new_path_counter_ = 0;
}

void NewPathManager::SetNewPath(const geometry::SiteVec& points) {
  new_local_path_.clear();
  new_local_path_ = points;
  new_path_counter_ = 0;
}

bool NewPathManager::is_pull_over_path() const {
  return ScenarioContext::instance()->scenario_info().current_latscenario 
                          == ScenarioContext::eLatScenarioEnum::PULL_OVER ? true : false;
}

bool NewPathManager::is_offset_path() const {
  return ScenarioContext::instance()->scenario_info().current_latscenario 
                          == ScenarioContext::eLatScenarioEnum::NUDGE_OFFSET ? true : false;
}

int NewPathManager::GetNearestIndex(const geometry::SiteVec &path, 
  const geometry::Site point, bool angle_limit) {
  const double kAngleError = 75;
  int index_nearest = -1;
  double min_l = std::numeric_limits<double>::max();
  double dis_l = 0;
  for (int i = 0; i < path.size(); i++) {
    dis_l = std::hypot(point.xg - path.at(i).xg, point.yg - path.at(i).yg);
    double delta_angle = 0;
    if (angle_limit) {
      delta_angle = IncludeAngle(path.at(i).globalangle, point.globalangle);
    }
    if (dis_l < min_l && fabs(delta_angle) < kAngleError) {
      min_l = dis_l;
      index_nearest = i;
    }
    if (dis_l > min_l + 5) {
      break;
    }
  }
  return index_nearest;
}

bool NewPathManager::IsNewPath(const geometry::SiteVec& path_points) const{
  if (path_points.empty()) {
    AERROR_IF(FLAGS_enable_debug_motion) << "points size is zero!";
    return false;
  }

  for (size_t i = 0 ;i < path_points.size(); ++i) {
    if (path_points.at(i).property == 4 || path_points.at(i).offset_property == 2) {
      AERROR_IF(FLAGS_enable_debug_motion) <<"No["<< i<<"] point property is 4 / 2!";
      return true;
    } else {
      continue;
    }
  }
  return false;
}

bool NewPathManager::Stitch(const ReferenceLineFrame& reference_line) {
  if (new_local_path_.empty() || reference_line.mapinfo.path_points.empty()) {
    return false;
  }

  if (!reference_line.mapinfo.path_points.empty()) {
    if (new_local_path_.front().reverse != reference_line.mapinfo.path_points.front().reverse) {
      new_local_path_.clear();
      new_local_path_.assign(reference_line.mapinfo.path_points.begin(), reference_line.mapinfo.path_points.end());
      return true;
    }
  }
  
  auto last_point = new_local_path_.back();
  double s = 0;
  double l = 0;
  if (!XYToSL(reference_line.mapinfo, last_point, s , l)) {
    AWARN_IF(FLAGS_enable_debug_motion)<< "failed to project the last point to the target reference line";
    return false;
  }
  AWARN_IF(FLAGS_enable_debug_motion)<<"nearest_l = "<< l <<"s = "<< s <<", join. ";

  const double kStitchingError = 0.05;//0.2;
  bool last_join = s > 0 && s <= reference_line.Length() &&
                   std::fabs(l) < kStitchingError;
  bool paral_stitch = s > 0 && s <= reference_line.Length() &&
                   std::fabs(l) >= kStitchingError;
  if ((last_join && paral_stitch) ||
       (!last_join && !paral_stitch)) {
    AWARN_IF(FLAGS_enable_debug_motion)<<"last_join = "<<last_join<<", paral_stitch = "<<paral_stitch;
    return true;
  }     

  if (paral_stitch && !is_offset_path()
    && (int)type_ < 2 
    && target_reference_line_id_ < 20) {//avoid path plan when lane change
    new_local_path_.clear();
    stitch_state_ = false;
    AERROR<<"stitch_state_ is false!!";
    return false;
  }   

  AINFO_IF(FLAGS_enable_debug_motion && last_join)<<"last_join====";
  AINFO_IF(FLAGS_enable_debug_motion && paral_stitch)<<"paral_stitch====";


  double accumulate_length = new_local_path_.back().length;
  common::math::Vec2d end_point(new_local_path_.back().xg,new_local_path_.back().yg);
  for (size_t i = 0; i < reference_line.mapinfo.path_points.size(); ++i) {
    if (reference_line.mapinfo.path_points.at(i).length > s) {
      auto point = reference_line.mapinfo.path_points.at(i);
      if (last_join) {
        common::math::Vec2d current_point(point.xg,point.yg);
        accumulate_length = accumulate_length + (current_point - end_point).Length();
        auto restruct_point = point;
        restruct_point.length = accumulate_length;
        end_point = current_point;
        new_local_path_.push_back(restruct_point);
      } else if (paral_stitch) {
        const auto angle = common::math::Angle16::from_rad(point.globalangle*kDEG_TO_RAD);
        common::math::Vec2d current_point(point.xg,point.yg);
        current_point.set_x(point.xg - common::math::sin(angle) * l);
        current_point.set_y(point.yg + common::math::cos(angle) * l);
        accumulate_length = accumulate_length + (current_point - end_point).Length();
        auto restruct_point = point;
        restruct_point.xg = current_point.x();
        restruct_point.yg = current_point.y();
        restruct_point.length = accumulate_length;
        restruct_point.property = 4;
        restruct_point.offset_property = 2;
        end_point = current_point;
        new_local_path_.push_back(restruct_point);
      }
    }
  }
  return true;
}

bool NewPathManager::GetNearestPoint(const geometry::SiteVec& points, const common::math::Vec2d& point, 
                     double* accumulate_s,double* lateral,int& index) {
  if (accumulate_s == nullptr || lateral == nullptr || points.empty()) {
    return false;
  }
  std::vector<common::math::LineSegment2d> segments;
  for (int i = 0; i + 1 < points.size(); ++i) {
    segments.emplace_back(common::math::Vec2d(points[i].xg,points[i].yg), 
                          common::math::Vec2d(points[i + 1].xg,points[i + 1].yg));
  }
  if (segments.empty()) {
    return false;
  }

  double num_segments = (int)points.size() - 1;
  double min_distance = std::numeric_limits<double>::infinity();
  int min_index = 0;
  for (int i = 0; i < num_segments; ++i) {
    const double distance = segments[i].DistanceSquareTo(point);
    if (distance < min_distance) {
      min_index = i;
      min_distance = distance;
    }
  }
  min_distance = std::sqrt(min_distance);
  const auto& nearest_seg = segments[min_index];
  const auto prod = nearest_seg.ProductOntoUnit(point);
  const auto proj = nearest_seg.ProjectOntoUnit(point);
  if (min_index == 0) {
    *accumulate_s = std::min(proj, nearest_seg.length());
    if (proj < 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * min_distance;
    }
  } else if (min_index == num_segments - 1) {
    *accumulate_s = points[min_index].length + std::max(0.0, proj);
    if (proj > 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * min_distance;
    }
  } else {
    *accumulate_s = points[min_index].length +
                    std::max(0.0, std::min(proj, nearest_seg.length()));
    *lateral = (prod > 0.0 ? 1 : -1) * min_distance;
  }
  index = min_index;
  return true;
}

double NewPathManager::IncludeAngle(double angle1, double angle2) {
  if (angle1 > angle2 + 180) {
    angle2 = angle2 + 360;
  } else if (angle2 > angle1 + 180) {
    angle1 = angle1 + 360;
  }
  double anglerr = angle1 - angle2;
  return anglerr;
}

}  // namespace planning
}  // namespace acu
