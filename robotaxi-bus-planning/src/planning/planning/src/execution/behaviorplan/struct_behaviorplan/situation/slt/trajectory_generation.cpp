#include "trajectory_generation.h"

namespace acu {
namespace planning {

TrajectoryGeneration::TrajectoryGeneration() {}

TrajectoryGeneration::~TrajectoryGeneration() {}

bool TrajectoryGeneration::GenerateSLTTrajectory() {
  GridGeneration generator;
  generator.InitProbGrid();
  reference_line_ = &context_->cognition_info_->reference_line_info;
  current_line_ = context_->reference_line_map_[reference_line_->current_line_id];
  length_ = context_->planning_config_.car_model.length;
  width_ = context_->planning_config_.car_model.car_width;
  front_over_hang_ = context_->planning_config_.car_model.front_over_hang;
  back_over_hang_ = context_->planning_config_.car_model.back_over_hang;
  line_index_[reference_line_->reverse_reference_line.reference_lane_id] = 0;
  for (auto& line : context_->reference_line_map_) {
    if (line.first < 20 || line.first >= 30) {
      line_index_[line.first] = line_index_.size();
    }
  }
  AddSpeedLimit();
  context_->point_info_.clear();
  GridInfoStruct point = context_->grid_info_.front().front().front();
  for (auto& grid : context_->grid_info_.front().front()) {
    if (fabs(grid.l - current_line_->mapinfo.dis2line) < 
        fabs(point.l - current_line_->mapinfo.dis2line)) {
      point = grid;
    }
  }
  point.v = context_->ego_speed_;
  point.a = context_->cognition_info_->vehicle_info.chassis.vehicle_accel;
  for (auto& pathpoint : context_->trajectory_info_.path) {
    if (pathpoint.t > 0.2) {
      point.v = pathpoint.velocity;
      point.a = pathpoint.a;
      break;
    }
  }
  auto DP = DataPool::Instance()->GetMainDataPtr();
  point.delta = (DP->loc_perception.localization_data.yaw - 
      current_line_->mapinfo.path_points.front().globalangle) / 180.0 * M_PI;
  point.total_cost = 0.0;
  std::vector<GridInfoStruct> points;
  std::vector<std::vector<GridInfoStruct>> sl_points;
  points.push_back(point);
  sl_points.push_back(points);
  context_->point_info_.push_back(sl_points);
  return SearchTrajectory();
}

void TrajectoryGeneration::AddSpeedLimit() {
  if (current_line_->mapinfo.expected_speeds.empty()) {
    return;
  }
  std::map<double, double> speed_limit;
  for (auto& speed : current_line_->mapinfo.expected_speeds) {
    speed_limit[speed.first] = speed.second;
  }
  double sample_s = 0.0;
  for (auto& point : current_line_->mapinfo.path_points) {
    if (point.length > current_line_->mapinfo.dis_to_end || 
        speed_limit.lower_bound(point.length) == speed_limit.end()) {
      break;
    } else if (point.length < sample_s) {
      continue;
    }
    sample_s += 2.0;
    double lane_limit = speed_limit.lower_bound(point.length)->second;
    double curv_limit = sqrt(1.2 / std::min(fabs(point.curvature), 1.0));
    speed_limit_[point.length] = (std::min(lane_limit, curv_limit));
  }
}

bool TrajectoryGeneration::SearchTrajectory() {
  GridInfoStruct point;
  for (int t_id = 1; t_id < context_->grid_info_.size(); t_id ++) {
    std::vector<std::vector<GridInfoStruct>> sl_points;
    auto& pre_sl_points = context_->point_info_.back();
    float min_s = pre_sl_points.front().front().s + 
        floor(pre_sl_points.front().front().v + FLAGS_max_dec);
    float max_s = pre_sl_points.back().front().s + 
        ceil(pre_sl_points.back().front().v + FLAGS_max_acc);
    // AWARN << "t = " << t_id << " min_s = " << min_s << " max_s = " << 
    //     max_s << " size = " << pre_sl_points.size();
    for (int s_id = 0; s_id < context_->grid_info_[t_id].size(); s_id++) {
      if (context_->grid_info_[t_id][s_id].front().s > max_s) {
        break;
      } else if (context_->grid_info_[t_id][s_id].front().s < min_s) {
        continue;
      }
      std::vector<GridInfoStruct> points;
      for (auto& grid : context_->grid_info_[t_id][s_id]) {
        point = grid;
        for (auto& pre_points : pre_sl_points) {
          if (pre_points.front().s > point.s) {
            continue;
          }
          for (auto& pre_point : pre_points) {
            CalculateCost(pre_point, point);
          }
        }
        if (point.total_cost < VALID_COST) {
          points.push_back(point);
        }
      }
      if (points.size()) {
        sl_points.push_back(points);
      }
    } 
    if (sl_points.empty()) {
      AERROR << "failed_t = " << t_id;
      return false;
    }
    context_->point_info_.push_back(sl_points);
  }
  GetFinalTrajectory();
  return true;
}

void TrajectoryGeneration::CalculateCost(const GridInfoStruct& pre_point, GridInfoStruct& point) {
  if (point.line_ids.front() > pre_point.line_ids.back() && 
      (point.l_solid || pre_point.r_solid) ||
      pre_point.line_ids.front() > point.line_ids.back() && 
      (pre_point.l_solid || point.r_solid)) {
    return;
  }
  double comfort_cost, safety_cost;
  if (GetComfortCost(pre_point, point, comfort_cost) == false || 
      GetSafetyCost(pre_point, point, safety_cost) == false) {
    return;
  }
  double cost = comfort_cost + safety_cost + pre_point.total_cost;
  if (cost < point.total_cost) {
    point.total_cost = cost;
    point.safety_cost = safety_cost;
    point.comfort_cost = comfort_cost;
    point.pre_s = pre_point.s;
    point.pre_l = pre_point.l;
    point.pre_s_id = pre_point.s_id;
    point.pre_l_id = pre_point.l_id;
    point.v = (point.s - pre_point.s) / (point.t - pre_point.t);
    point.a = (point.v - pre_point.v) / (point.t - pre_point.t);
    point.delta = atan2(point.l - pre_point.l, point.s - pre_point.s);
  }
}

bool TrajectoryGeneration::GetComfortCost(const GridInfoStruct& pre_point, 
                                          GridInfoStruct& point, double& cost) {
  float v = (point.s - pre_point.s) / (point.t - pre_point.t);
  float a = (v - pre_point.v) / (point.t - pre_point.t);
  if (speed_limit_.lower_bound(point.s) != speed_limit_.end() && 
      speed_limit_.lower_bound(point.s)->second < v ||
      a < FLAGS_max_dec || a > FLAGS_max_acc) {
    return false;
  } 
  double centripetal_a = 0.0;
  if (point.s == pre_point.s && point.l != pre_point.l) {
    return false;
  } else if (point.s != pre_point.s) {
    double angle = atan2(point.l - pre_point.l, point.s - pre_point.s);
    double dis = std::hypot(point.s - pre_point.s, point.l - pre_point.l);
    double radius = 0.5 * dis / sin(fabs(pre_point.delta - angle));
    if (fabs(point.l - pre_point.l) > 0.2 && radius < std::max(v * v / 1.2, 5.0)) {
      return false;
    }
    centripetal_a = v * v / radius;
  }
  double lat_cost = 1.0 * fabs(point.l) + 10.0 * fabs(point.l - pre_point.l);
  double lon_cost = 10.0 * fabs(a) + 100.0 * fabs(a - pre_point.a);
  if (pre_point.s_id > 0) {
    lat_cost += 100.0 * fabs(point.l - pre_point.pre_l);
  }
  cost = lat_cost + lon_cost + 100.0 * centripetal_a - 1.0 * point.s;
  return true;
}

bool TrajectoryGeneration::GetSafetyCost(const GridInfoStruct& pre_point,
                                         GridInfoStruct& point, double& cost) {
  cost = 0.0;
  double dis = std::hypot(point.xg - pre_point.xg, point.yg - pre_point.yg);
  double angle = atan2(point.yg - pre_point.yg, point.xg - pre_point.xg);
  double center_s_to_hang = cos(angle) * (front_over_hang_ - 0.5 * length_);
  double center_l_to_hang = sin(angle) * (front_over_hang_ - 0.5 * length_);
  Vec2d center(0.5 * (point.xg + pre_point.xg) + center_s_to_hang, 
      0.5 * (point.yg + pre_point.yg) + center_l_to_hang);
  Box2d ego_box(center, angle, 0.5 * (dis + length_), width_);
  for (auto& static_object : context_->static_objects_) {
    double distance = static_object.second.DistanceTo(ego_box);
    cost += 50.0 / std::max(distance, 1e-5);
    if (distance < FLAGS_collision_buff) {
      return false;
    }
  }
  for (auto& dynamic_object : context_->dynamic_objects_) {
    auto it = dynamic_object.second.lower_bound(pre_point.t);
    if (it == dynamic_object.second.end() || 
        point.t < dynamic_object.second.begin()->first) {
      continue;
    }
    for (; it != dynamic_object.second.end(); ++it) {
      if (it->first > point.t) {
        break;
      }
      double s = pre_point.xg + (point.xg - pre_point.xg) * (it->first - pre_point.t);
      double l = pre_point.yg + (point.yg - pre_point.yg) * (it->first - pre_point.t);
      Vec2d center(s + center_s_to_hang, l + center_l_to_hang);
      Box2d box(center, angle, length_, width_);
      double distance = it->second.first.DistanceTo(box);
      cost += 25.0 / std::max(distance, 1e-5);
      if (distance < FLAGS_collision_buff) {
        return false;
      }
    }
  }
  return true;
}

void TrajectoryGeneration::GetFinalTrajectory() {
  GridInfoStruct final_point = context_->point_info_.back().back().back();
  for (auto& points : context_->point_info_.back()) {
    for (auto& point : points) {
      if (final_point.line_ids.front() > current_line_->reference_lane_id || 
          final_point.line_ids.back() < current_line_->reference_lane_id) {
        if (point.line_ids.front() <= current_line_->reference_lane_id &&  
            point.line_ids.back() >= current_line_->reference_lane_id ||
            point.total_cost < final_point.total_cost) {
          final_point = point;
        }
      } else {
        if (point.line_ids.front() <= current_line_->reference_lane_id && 
            point.line_ids.back() >= current_line_->reference_lane_id && 
            point.total_cost < final_point.total_cost) {
          final_point = point;
        }
      }
    }
  }
  AERROR_IF(FLAGS_log_enable) << "s = " << final_point.s << " l = " << 
      final_point.l << " cost = " << final_point.total_cost;
  context_->final_path_.push_back(final_point);
  for (int t_id = context_->point_info_.size() - 2; t_id >= 0; t_id--) {
    for (auto& points : context_->point_info_[t_id]) {
      if (points.front().s == context_->final_path_.back().pre_s) {
        for (auto& point : points) {
          if (point.l_id == context_->final_path_.back().pre_l_id) {
            context_->final_path_.push_back(point);
            AERROR_IF(FLAGS_log_enable) << "s = " << point.s << " l = " << 
                point.l << " v = " << point.v << " type = " << point.p;
            break;
          }
        }
        break;
      }
    }
  }
}

}  //  namespace planning
}  //  namespace acu
