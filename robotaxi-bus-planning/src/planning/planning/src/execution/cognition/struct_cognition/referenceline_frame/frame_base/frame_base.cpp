#include "frame_base.h"
 
namespace acu{ 
namespace planning {

ReferenceLineFrame* GetTargetData(StructReferenceLineInfo &reference_info, int &target_line_id) {
  ReferenceLineFrame* target_line = nullptr;
  int a = target_line_id / 10;
  int b = target_line_id % 10;
  if (a == 1 && reference_info.current_reference_line.size() > b) {
    target_line = &reference_info.current_reference_line.at(b); 
    return target_line;
  }
  if (a == 2 && reference_info.left_reference_line.size() > b) {
    target_line = &reference_info.left_reference_line.at(b); 
    return target_line;
  }
  if (a == 3 && reference_info.right_reference_line.size() > b) {
    target_line = &reference_info.right_reference_line.at(b); 
    return target_line;
  }
  return nullptr;
}

bool PathBoxInLine(StructReferenceLineInfo &reference_info, 
                    const list<geometry::Site> &motionpath, 
                    const CarModel& car_model, const int& drive_state, int &current_id) {
  ReferenceLineFrame *current_line_ptr = GetTargetData(reference_info, current_id);
  if (current_line_ptr == nullptr || drive_state != 1) {
    return true;
  }
  double length = -2.1, left_w = 0.0, right_w = 0.0;
  for (auto &point : motionpath) {
    if (point.length - length < 2.0) continue;
    length = point.length;
    Vec2d path_center(point.xg, point.yg);
    Box2d path_box(path_center, point.globalangle * M_PI/180.0, car_model.length, car_model.car_width);
    int out_size = 0;
    vector<Vec2d> corners;
    path_box.GetAllCorners(&corners);
    for (auto &corner : corners) {
      Site corner_site;
      corner_site.xg = corner.x();
      corner_site.yg = corner.y();
      if (!current_line_ptr->GetWidthToLaneBoundary(left_w, right_w, corner_site)) continue;
      if (right_w < -FLAGS_collision_buff || left_w < -FLAGS_collision_buff) {
        out_size++;
      }
      if (out_size > 1) return false;
    }
  }
  return true;
}


bool LocalPathInLine(StructReferenceLineInfo &reference_info, list<geometry::Site> &motionpath, 
                     int &origin_id, int &check_id, int drive_state) {
  ReferenceLineFrame *orgin_line_ptr = GetTargetData(reference_info, origin_id);
  ReferenceLineFrame *check_line_ptr = GetTargetData(reference_info, check_id);
  if (orgin_line_ptr == nullptr || check_line_ptr == nullptr || drive_state != 1) {
    return true;
  }
  double length = -2.1;
  double left_w = 0.0, right_w = 0.0;
  for (auto &point : motionpath) {
    if (point.length - length < 2.0) continue;
    length = point.length;
    if (orgin_line_ptr->GetWidthToLaneBoundary(left_w, right_w, point) &&
        (right_w < -FLAGS_boundary_buff || left_w < -FLAGS_boundary_buff)) {
      AWARN_IF(FLAGS_log_enable)<<"origin_id "<<origin_id<<" left_w "<<left_w<<" right_w "<<right_w<<" s "<<length;
      if (orgin_line_ptr->mapinfo.path_points.empty() || 
          orgin_line_ptr->mapinfo.path_points.back().length < 15.0 &&
          point.length < 1.0) continue;// 目标车道很短，只剩刚开始几个点没到
      return false;
    }
  }
  bool is_divide_flag = false;
  if (orgin_line_ptr->mapinfo.front_lane_ids.size() &&
      check_line_ptr->mapinfo.front_lane_ids.size() &&
      orgin_line_ptr->mapinfo.front_lane_ids.front() == 
      check_line_ptr->mapinfo.front_lane_ids.front()) {
    is_divide_flag = true;
  }
  if (motionpath.size() && !is_divide_flag &&
      check_line_ptr->GetWidthToLaneBoundary(left_w, right_w, motionpath.front()) &&
      right_w > FLAGS_boundary_buff && left_w > FLAGS_boundary_buff) {
    AWARN_IF(FLAGS_log_enable)<<"still has point in check "<<check_id;
    return false;
  }
  return true;
}

void SetSpeedLimit(ReferenceLineFrame &input_line, SpeedplanConfig &speedplan_config,
                   pair<double, double> remove_point) {
  auto &map_points = input_line.mapinfo.path_points;
  auto &expected_speeds = input_line.mapinfo.expected_speeds;
  if (map_points.empty() || expected_speeds.empty()) return;
  int near_index;
  Site remove_site, near_site;
  double remove_s, min_dis;
  remove_site.xg = remove_point.first;
  remove_site.yg = remove_point.second;
  if (!input_line.GetGlobalNearestPoint(remove_site, near_site, remove_s, min_dis, near_index)) {
    return;
  }
  if (near_index <= 3 || remove_point.first < 1.0 || remove_point.second < 1.0) {
    remove_point.first = 0.0;
    remove_point.second = 0.0;
    near_index = 10000;
    for (auto &speed : expected_speeds) {
      speed.second = fmin(speed.second, speedplan_config.maximum_cruising_speed/3.6);
    }
    return;
  }
  AERROR_IF(FLAGS_log_enable)<<"remove point ("<<fixed<<remove_point.first
                             <<","<<remove_point.second
                             <<"), remove_s is "<<remove_s
                             <<" near_index"<<near_index;
  if (expected_speeds.front().first > remove_s &&
      expected_speeds.front().second > speedplan_config.maximum_bump_speed/3.6 + 0.1) {
    double over_speed = fmin(FLAGS_max_pd_velocity, expected_speeds.front().second * 1.1);
    pair<double, double> temp_expected_speed(remove_s, over_speed);
    expected_speeds.insert(expected_speeds.begin(), temp_expected_speed);
    for (int i = 0; i < expected_speeds.size(); i++) {
      AINFO_IF(FLAGS_log_enable)<<"s "<<expected_speeds.at(i).first<<" speed "<<expected_speeds.at(i).second;
    }
    return;
  }
  int remove_speed_index = -1;
  for (int i = 1; i < expected_speeds.size(); i++) {
    if (expected_speeds.at(i-1).first < remove_s &&
        expected_speeds.at(i).first > remove_s) {
      remove_speed_index = i - 1;
      break;
    }
    if (expected_speeds.back().first < remove_s) {
      remove_speed_index = i;
    }
  }
  for (int i = 0; i < expected_speeds.size(); i++) {
    if (i <= remove_speed_index) {
      if (expected_speeds.at(i).second > speedplan_config.maximum_bump_speed/3.6 + 0.1) {
        expected_speeds.at(i).second = fmin(FLAGS_max_pd_velocity,
                              expected_speeds.at(i).second * 1.1);
      }
    } else if (i == remove_speed_index + 1) {
      double over_speed = fmin(FLAGS_max_pd_velocity,
                               expected_speeds.at(i).second * 1.1);
      pair<double, double> temp_expected_speed(remove_s, over_speed);
      expected_speeds.insert(expected_speeds.begin()+i, temp_expected_speed);
    } 
  }
  for (int i = 0; i < expected_speeds.size(); i++) {
    AINFO_IF(FLAGS_log_enable)<<"s "<<expected_speeds.at(i).first<<" speed "<<expected_speeds.at(i).second;
  }
}

void ObjectSpeedTransform(ReferenceLineFrame& reference_line, LineObject& object) {
  Site reference_point;
  int index;
  if (object.is_static) {
    object.vlabs = 0.0;
    object.vsabs = 0.0;
    return;
  }
  if (!reference_line.GetNearestPoint(object.sl_boundary.min_s, reference_point, index)) {
    return;
  }
  double delta = reference_point.globalangle / 180.0 * M_PI;
  object.vlabs = object.vyabs*cos(delta) - object.vxabs*sin(delta);
  object.vsabs = object.vxabs*cos(delta) + object.vyabs*sin(delta);
}

void IsParallelObject(LineObject& object, const CarModel *car_model_ptr) {
  double nearest_l = fmin(fabs(object.sl_boundary.min_l), fabs(object.sl_boundary.max_l));
  // 静态、类型、有无预测线、s分布、l分部、速度分解
  if (object.is_static || object.type > 3 || object.prediction.trajectories.empty() ||
      object.sl_boundary.max_s < car_model_ptr->front_over_hang - 1.0 ||
      object.sl_boundary.min_s > 2.0 * FLAGS_frenet_perception_range ||
      object.sl_boundary.min_l * object.sl_boundary.max_l < 0 ||
      nearest_l < car_model_ptr->car_width / 2.0 + 0.2 || 
      nearest_l > car_model_ptr->car_width / 2.0 + 1.0 ||
      fabs(object.speed - object.vsabs) > 2.0) {
    object.parallel_counter = 0;
    return;
  }
  if (object.type == 2) {
    if (object.prediction.trajectories.front().lane_ids.empty() || 
        object.prediction.trajectories.front().lane_ids.size() &&
        object.prediction.trajectories.front().lane_ids.front() == "PEDESTRIAN") {
      object.parallel_counter = 0;
      return;
    }
  } else if (object.type == 3) {
    if (object.prediction.trajectories.front().lane_ids.empty() || 
        object.prediction.trajectories.front().lane_ids.size() &&
        object.prediction.trajectories.front().lane_ids.front() == "CYCLIST") {
      object.parallel_counter = 0;
      return;
    }
  }
  object.parallel_counter++;
  if (object.parallel_counter > FLAGS_history_size) {
    object.parallel_counter = FLAGS_history_size + 1;
    object.need_focus = true;
    object.key_focus = 1;
  }
}



} // namespace planning
} // namespace acu
