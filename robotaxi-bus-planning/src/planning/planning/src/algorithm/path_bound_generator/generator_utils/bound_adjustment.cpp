/**
 * @file bound_process.cpp
 BoundAdjustment类，用于对边界长度、类型、扫描中心线选取等属性进行处理
 **/

#include "bound_adjustment.h"


namespace acu {
namespace planning {

BoundAdjustment::BoundAdjustment(){}

bool BoundAdjustment::UpdateBoundaryType(
      const size_t& idx, const int& left_bound_type, const int& right_bound_type,
      std::vector<std::pair<int, int>>* const boundary_types) {
  double new_right_bound_type = std::max(right_bound_type,(*boundary_types)[idx].first);
  double new_left_bound_type = std::max(left_bound_type,(*boundary_types)[idx].second);
  (*boundary_types)[idx] = make_pair(new_right_bound_type,new_left_bound_type);
  return true;
}

bool BoundAdjustment::UpdatePathBoundaryAndCenterLine(
    const size_t& idx, const double& left_bound, const double& right_bound,
    LaneBound* const path_boundaries, double* const center_line) {
  // Update the right bound (l_min):
  double adc_half_width =
          EgoInfo::instance()->vehicle_param().half_wheel;
  double new_l_min =
      std::fmax(std::get<1>((*path_boundaries)[idx]),
                right_bound + adc_half_width);
  // Update the left bound (l_max):
  double new_l_max = std::fmin(std::get<2>((*path_boundaries)[idx]),
                               left_bound - adc_half_width);

  // Check if ADC is blocked.
  // If blocked, don't update anything, return false.
  if (new_l_min > new_l_max) {
    AINFO_IF(FLAGS_enable_debug_motion) << "Path is blocked at idx = " << idx
       <<", new_l_min: "<<new_l_min<<" > right_bound: "<< new_l_max;
    return false;
  }
  // Otherwise, update path_boundaries and center_line; then return true.
  std::get<1>((*path_boundaries)[idx]) = new_l_min;
  std::get<2>((*path_boundaries)[idx]) = new_l_max;
  *center_line = (std::get<1>((*path_boundaries)[idx]) +
                  std::get<2>((*path_boundaries)[idx])) /
                 2.0;
  return true;
}


void BoundAdjustment::TrimPathBounds(const int& path_blocked_idx,
                                       LaneBound* const path_boundaries) {
  if (path_blocked_idx != -1) {
    if (path_blocked_idx == 0) {
      AINFO_IF(FLAGS_enable_debug_motion) << "Completely blocked. Cannot move at all.";
    }
    int range = static_cast<int>(path_boundaries->size()) - path_blocked_idx;
    for (int i = 0; i < range; ++i) {
      path_boundaries->pop_back();
    }
  }
}

void BoundAdjustment::TrimPathSoftBounds(const int& path_blocked_idx,
      std::vector<std::tuple<double, double, double, double, std::string>>* const path_boundaries) {
  if (path_blocked_idx != -1) {
    if (path_blocked_idx == 0) {
      AINFO_IF(FLAGS_enable_debug_motion) << "Completely blocked. Cannot move at all.";
    }
    int range = static_cast<int>(path_boundaries->size()) - path_blocked_idx;
    for (int i = 0; i < range; ++i) {
      path_boundaries->pop_back();
    }
  }
}

void BoundAdjustment::TrimPathBoundTypes(const int& path_blocked_idx,
                            std::vector<std::pair<int, int>>* const path_boundary_types) {
  if (path_blocked_idx != -1) {
    if (path_blocked_idx == 0) {
      AINFO_IF(FLAGS_enable_debug_motion) << "Completely blocked. Cannot move at all.";
    }
    int range = static_cast<int>(path_boundary_types->size()) - path_blocked_idx;
    for (int i = 0; i < range; ++i) {
      path_boundary_types->pop_back();
    }
  }
}

//center_vec的size应不大于2，front为优先选择的centerline
//is_obstacle: 是否是障碍物类型改变了centerline
//is_worse_option: 是否centerline_vec中的优先解无法使用
void BoundAdjustment::UpdateCenterLineVec(std::vector<std::pair<double, bool>>& center_vec, 
    const double& input_center, const bool& is_obstacle, const bool& is_worse_option){
  if(center_vec.size() != 1 && center_vec.size() != 2){
    AERROR_IF(FLAGS_enable_debug_motion)<<"ERROR center_line_vec size: "<<center_vec.size()<<", re-init";
    center_vec.clear();
    center_vec.emplace_back(std::make_pair(input_center, is_obstacle));
    return;
  }
  //debug
  // for(const auto& option : center_vec){
  //   AINFO_IF(FLAGS_enable_debug_motion)<<"[input]current options: "<<option.first<<", "<<option.second;
  // }
  // AINFO_IF(FLAGS_enable_debug_motion)<<"current size: "<<center_vec.size();
  // AINFO_IF(FLAGS_enable_debug_motion)<<"input_center: "<<input_center<<", is_obstacle: "<<is_obstacle<<", is_worse_option: "<<is_worse_option;

  if(is_worse_option){
    // AWARN_IF(FLAGS_enable_debug_motion)<<"priority-solution not works";
    center_vec.clear();
    center_vec.emplace_back(std::make_pair(input_center, is_obstacle));
    return;
  }
  //障碍物修改centerline时
  if(is_obstacle){
    if(!center_vec.front().second){
    //当前最优不是障碍物修改的，直接清空并替换
      // AWARN_IF(FLAGS_enable_debug_motion)<<"center_vec push_front: "<<input_center;
      center_vec.clear();
      center_vec.emplace_back(std::make_pair(input_center, is_obstacle));
    }else{//当前最优是障碍物修改的
      if(center_vec.size() == 1){
        if(fabs(input_center - center_vec.front().first) < 0.2){
        //相比于最优centerline变化不大，清空并将最优更新
          // AINFO_IF(FLAGS_enable_debug_motion)<<"set input_center front: "<<input_center;
          center_vec.clear();
          center_vec.emplace_back(std::make_pair(input_center, is_obstacle));
        }else{//相比于最优centerline变化较大，仍保持之前的最优，将当前作为次优
          // AINFO_IF(FLAGS_enable_debug_motion)<<"set center_vec back: "<<input_center;
          center_vec.emplace_back(std::make_pair(input_center, is_obstacle));
        }
      }else if(center_vec.size() == 2){//size为2
        if(center_vec.back().second){//当前最优、次优都是障碍物引起的
          // AINFO_IF(FLAGS_enable_debug_motion)<<"both obstacle";
          if(fabs(input_center - center_vec.front().first) < 
              fabs(input_center - center_vec.back().first)){
            //input_center距离当前的最优比次优更近，保留最优，把次优更新
            // AINFO_IF(FLAGS_enable_debug_motion)<<"update non-optimal: "<<input_center;
            center_vec.back() = std::make_pair(input_center, is_obstacle);
          }else{
            //input_center距离当前次优更近，清空把最优更新
            // AERROR_IF(FLAGS_enable_debug_motion)<<"update optimal: "<<input_center;
            center_vec.clear();
            center_vec.emplace_back(std::make_pair(input_center, is_obstacle));
          }
        }else{
          //当前次优不是障碍物给出的
          // AINFO_IF(FLAGS_enable_debug_motion)<<"non-optimal is not obstacle";
          if(fabs(input_center - center_vec.front().first) < 0.2){
            //相比于最优centerline变化不大，清空并将最优更新
            AINFO_IF(FLAGS_enable_debug_motion)<<"set input_center front: "<<input_center;
            center_vec.clear();
            center_vec.emplace_back(std::make_pair(input_center, is_obstacle));
          }else{//相比于最优centerline变化较大，仍保持之前的最优，将当前作为次优
            // AINFO_IF(FLAGS_enable_debug_motion)<<"set center_vec back: "<<input_center;
            center_vec.back() = std::make_pair(input_center, is_obstacle);
          }
        }
      }
    }
  }else{//不是障碍物修改的centerline，都认为是次优
    if(center_vec.size() == 1){
      if(center_vec.front().second){
        // AINFO_IF(FLAGS_enable_debug_motion)<<"optimal is obstacle, emplace_back";
        center_vec.emplace_back(std::make_pair(input_center, is_obstacle));
      }else{
        // AINFO_IF(FLAGS_enable_debug_motion)<<"optimal not obstacle, change back";
        center_vec.back() = std::make_pair(input_center, is_obstacle);
      }
    }else{
      // AINFO_IF(FLAGS_enable_debug_motion)<<"center_vec.size() > 1, change back";
      center_vec.back() = std::make_pair(input_center, is_obstacle);
    }
  }
   //debug
  // for(const auto& option : center_vec){
  //   AINFO_IF(FLAGS_enable_debug_motion)<<"[output]current options: "<<option.first<<", "<<option.second;
  // }
  return;
}

void BoundAdjustment::PathBoundsDebugString(const LaneBound& path_boundaries, const int& line) {
  for (size_t i = 0; i < path_boundaries.size(); ++i) {
    AWARN_IF(FLAGS_enable_debug_motion && 
      (fabs(std::get<1>(path_boundaries[i]))<1e5 ||
       fabs(std::get<2>(path_boundaries[i]))<1e5)) << line <<" idx " << i << "; s = " << std::get<0>(path_boundaries[i])
          << "; l_min = " << std::get<1>(path_boundaries[i])
          << "; l_max = " << std::get<2>(path_boundaries[i]);
  }
  return;
}

}  // namespace planning
}  // namespace acu
