#include "line_gap.h" 

namespace acu{
namespace planning {

void LineGap::AddGapInfo(ReferenceLineFrame& reference_line,
                         const LocalizationData &locpose,
                         const SpeedplanConfig &speed_config) { 
  input_line_ptr_ = &reference_line;
  objects_ptr_ = &reference_line.objects_;
  car_speed_ = locpose.velocity;
  car_model_ptr_ = &reference_line.car_model_;
  speed_config_ptr_ = &speed_config;
  if (reference_line.reference_lane_id < 20 ||
      reference_line.reference_lane_id >= 40 ||
      reference_line.mapinfo.path_points.empty()) {
    return;
  }
  AERROR<<"-------------line gap--------------";
  FindGapObstacles();
  SimCarS_t();
  GapEvaluation();
}

void LineGap::FindGapObstacles() {
  sorted_objects_.clear();
  SortLineObstacles();
  SetBoundGapobjects();
}

void LineGap::SortLineObstacles() {
  double dis_to_end = fmax(input_line_ptr_->mapinfo.path_points.back().length, 
                           input_line_ptr_->mapinfo.dis2missionpoint);
  for (auto &it : input_line_ptr_->objects_) {
    auto &object = it.second;
    if (object.type >= 4 ||
        object.type == 2 && object.is_static ||
        object.type == 3 && object.is_static ||
        object.sl_boundary.min_s > dis_to_end) {
      continue;
    }

    if (object.sl_boundary.min_l > FLAGS_lane_width ||
    	object.sl_boundary.max_l < -FLAGS_lane_width) {
      continue;
    }
    double left_w = 0.0, right_w = 0.0, start_t = 0.0;
    input_line_ptr_->GetWidthToLaneBoundary(left_w, right_w, object.sl_boundary.min_s);
    double lane_width = left_w + right_w;
    Site obj_center;
    obj_center.xg = object.xabs;
    obj_center.yg = object.yabs;
    input_line_ptr_->GetWidthToLaneBoundary(left_w, right_w, obj_center);
    double delta_l = left_w - 0.5 * object.box.width() - FLAGS_boundary_width;
    double delta_r = right_w - 0.5 * object.box.width() - FLAGS_boundary_width;
    double passable_l = fmax(delta_l, delta_r);
    AINFO_IF(FLAGS_log_enable)<<"obj "<<object.id<<" passable_l "<<passable_l<<" delta_l "<<delta_l<<" delta_r "<<delta_r
        <<" left_w "<<left_w<<" right_w "<<right_w;
    if (passable_l < lane_width / 2.0 + car_model_ptr_->car_width / 2.0) {
      AERROR_IF(FLAGS_gaplog_enable)<<"block_object_id is "<<object.id;
      GapObject temp_gap_obj(object, passable_l);
      sorted_objects_.emplace_back(temp_gap_obj);
    } 
     else if (IsCutInObject(object, start_t)) { // 判定切入
      AERROR_IF(FLAGS_gaplog_enable)<<"cutin_object_id is "<<object.id;
      GapObject temp_gap_obj(object, passable_l, start_t);
      sorted_objects_.emplace_back(temp_gap_obj);
    } 
  }
  std::sort(sorted_objects_.begin(), sorted_objects_.end(),
    [](const GapObject& a, const GapObject& b) {
      return a.min_s < b.min_s;
    });
}

bool LineGap::IsCutInObject(LineObject &object, double &t) {
  t = 0.0;
  if (input_line_ptr_->mapinfo.front_lane_ids.empty() ||
      object.prediction.trajectories.empty()) {
    return false;
  }
  for (auto &pd_line : object.prediction.trajectories) {
    if (pd_line.lane_ids.size() <= 1) continue; // 无换道，肯定不是cutin
    if (pd_line.intentbylane == "Keep Current") continue;
    for (auto &lane : input_line_ptr_->mapinfo.front_lane_ids) {
      if (pd_line.lane_ids.front() == lane) {
        AERROR_IF(FLAGS_gaplog_enable)<<object.id<<" same first lane, not cut in.";
        return false;
      }
      if (pd_line.lane_ids.back() == lane && pd_line.st_boundary.size()) {
        t = pd_line.st_boundary.front().first.x() / car_speed_;
        if (t < 2.0) t = 0.0;
        return true;
      }
    }
  }
  return false;
}

void LineGap::SetBoundGapobjects() {
  gap_param_.max_speed = speed_config_ptr_->maximum_cruising_speed/3.6 + 0.1;
  if (input_line_ptr_->mapinfo.expected_speeds.size()) {
    gap_param_.max_speed= fmin(input_line_ptr_->mapinfo.expected_speeds.front().second * 1.1,
                                gap_param_.max_speed);
  }
  gap_param_.max_speed = fmax(gap_param_.max_speed, car_speed_ + 0.1);// 防止因为平行车道限速不同导致t不对
  AINFO_IF(FLAGS_gaplog_enable)<<"max_speed "<<gap_param_.max_speed;

  GapObject temp_obj;
  if (sorted_objects_.empty()) {
    temp_obj.Set(-1, FLAGS_front_perception_range - 0.1, 
                 FLAGS_front_perception_range + 0.1, 
                 FLAGS_front_perception_range, false, 
                 gap_param_.max_speed, 0.0, FLAGS_lane_width);
    sorted_objects_.push_back(temp_obj);
    temp_obj.Set(-2, -FLAGS_front_perception_range - 0.1, 
                 -FLAGS_front_perception_range + 0.1, 
                 -FLAGS_front_perception_range, false, car_speed_, 0.0, FLAGS_lane_width);
    sorted_objects_.push_back(temp_obj);
  } else {
    if (sorted_objects_.front().mid_s > 20.0 - FLAGS_front_perception_range) {
      temp_obj.Set(-2, -FLAGS_front_perception_range - 0.1, 
                   -FLAGS_front_perception_range + 0.1, 
                   -FLAGS_front_perception_range, false, car_speed_, 0.0, FLAGS_lane_width);
      sorted_objects_.insert(sorted_objects_.begin(), temp_obj);
    }
    if (sorted_objects_.back().mid_s < FLAGS_front_perception_range - 20.0) {
      temp_obj.Set(-1, FLAGS_front_perception_range - 0.1, 
                   FLAGS_front_perception_range + 0.1, 
                   FLAGS_front_perception_range, false, 
                   gap_param_.max_speed, 0.0, FLAGS_lane_width);
      sorted_objects_.push_back(temp_obj);
    }
  }

  for (auto &obj : sorted_objects_) {
    AERROR_IF(FLAGS_gaplog_enable)<<"sorted gap obj "<<obj.id
                                  <<" speed "<<obj.speed<<" acc "<<obj.acc;
  }
}

void LineGap::SimCarS_t() {
  if (sorted_objects_.size() < 2) {
    AERROR<<"sorted_allobjects size is wrong.";
    return;
  }
  double front_obj_speed, front_obj_s, station_speed;
  if (FrontObstacleLimit(front_obj_speed, front_obj_s)) {
    gap_param_.max_speed = fmin(gap_param_.max_speed, front_obj_s / gap_param_.ttc + front_obj_speed);
  }
  if (StationLimit(station_speed)) {
    gap_param_.max_speed = fmin(gap_param_.max_speed, station_speed);
  }
  car_ranges_.clear();
  for (double t = 0.0; t < gap_param_.total_time; t += 0.2) {
    CarRange_t temp_range;
    temp_range.t = t;
    temp_range.min_v = car_speed_ - fmax(FLAGS_self_dec * t, 0.01);
    temp_range.max_v = VT(car_speed_, t, FLAGS_self_acc); 
    temp_range.max_v = fmin(temp_range.max_v, gap_param_.max_speed);
    temp_range.min_v = fmin(temp_range.min_v, temp_range.max_v);
    temp_range.min_s = (temp_range.min_v + car_speed_) * t / 2.0;
    temp_range.max_s = (temp_range.max_v + car_speed_) * t / 2.0;
    SimFutureSortedObstacles(t, temp_range.future_objs);
    car_ranges_.push_back(temp_range);
  }         
}

bool LineGap::FrontObstacleLimit(double &front_obj_speed, double &front_obj_s) {
  if (objects_ptr_->count(input_line_ptr_->block_id)) {
    auto &front_obj = objects_ptr_->at(input_line_ptr_->block_id);
    front_obj_speed = front_obj.speed;
    front_obj_s = front_obj.sl_boundary.min_s - car_model_ptr_->front_over_hang - FLAGS_back_buff;
    AERROR_IF(FLAGS_gaplog_enable)<<"front obj "<<front_obj.id
            <<" front_obj_speed "<<front_obj_speed
            <<" front_obj_s "<<front_obj_s;
    return true;
  }
  return false;
}

bool LineGap::StationLimit(double &max_v, double start_s) {
  if (input_line_ptr_ == nullptr) return false;
  double max_s = input_line_ptr_->mapinfo.dis2missionpoint - start_s;
  max_s = fmax(0.0, max_s);
  if (max_s < 0.5 * car_speed_ * car_speed_ / FLAGS_self_dec) {
    max_v = sqrt(2.0 * FLAGS_self_dec * max_s);
  } else {
    max_v = sqrt((2.0 * FLAGS_self_acc * FLAGS_self_dec * max_s + 
                  FLAGS_self_dec * car_speed_ * car_speed_)/
                  (FLAGS_self_acc + FLAGS_self_dec));
  }
  return true;
}

void LineGap::SimFutureSortedObstacles(double t, vector<GapObject> &future_objs) {
  future_objs.clear();
  if (sorted_objects_.empty()) return;
  for (auto &sort_obj : sorted_objects_) {
    if (t < sort_obj.start_t) continue;
    double delta = sort_obj.speed * t + 0.5 * sort_obj.acc * t * t;
    if (sort_obj.is_static) {
      delta = 0.0;
      sort_obj.speed = 0.0;
      sort_obj.acc = 0.0;
    }
    GapObject future_obj;
    future_obj.id = sort_obj.id;
    future_obj.min_s = sort_obj.min_s + delta;
    future_obj.max_s = sort_obj.max_s + delta;
    future_obj.mid_s = sort_obj.mid_s + delta;
    future_obj.is_static = sort_obj.is_static;
    future_obj.speed = sort_obj.speed + t * sort_obj.acc;
    future_obj.acc = sort_obj.acc;
    future_obj.passable_l = sort_obj.passable_l;
    future_obj.start_t = sort_obj.start_t;
    future_objs.push_back(future_obj);
  }
  for (int i = future_objs.size() - 1; i >= 1; i--) {
    if (future_objs.at(i).id == -1) continue;// 顺序改动不考虑最前方虚拟障碍物
    if (future_objs.at(i-1).start_t > 1e-3) continue;
    if (future_objs.at(i-1).mid_s > future_objs.at(i).mid_s &&
        future_objs.at(i).passable_l < 1.0) {// 偏移导致顺序可能更改
      double predict_mid_s = future_objs.at(i).mid_s;
      double length_i = fabs(future_objs.at(i-1).max_s - future_objs.at(i-1).min_s) +
                        fabs(future_objs.at(i).max_s - future_objs.at(i).min_s);
      // 为防止障碍物加速度估计不准，可以尝试用初始速度作为最小距离评估
      double delta_s = fmax(0.5 * length_i + gap_param_.back_buff, 
                            FLAGS_min_thw * future_objs.at(i-1).speed);
      delta_s = fmin(30.0, 2.0 * future_objs.at(i-1).speed);
      future_objs.at(i-1).mid_s = future_objs.at(i).mid_s - delta_s;                    
      future_objs.at(i-1).min_s = future_objs.at(i).min_s - delta_s;
      future_objs.at(i-1).max_s = future_objs.at(i).max_s - delta_s;
      future_objs.at(i-1).speed -= 2.0 * delta_s / t;
      future_objs.at(i-1).speed = fmax(0.0, future_objs.at(i-1).speed);
    }
  }
  std::sort(future_objs.begin(), future_objs.end(),
    [](const GapObject& a, const GapObject& b) {
      return a.min_s < b.min_s;
    });
}

void LineGap::GapEvaluation() {
  LineGapInit();
  if (car_ranges_.empty()) return;
  for (auto &car_range : car_ranges_) {
    auto &t_future_objs = car_range.future_objs;
    AERROR_IF(FLAGS_gaplog_enable)<<"t "<<car_range.t;
    for (int i = 0; i + 1 < t_future_objs.size(); i++) {
      GapStruct gap_range;
      gap_range.start_id = t_future_objs.at(i).id;
      gap_range.end_id = t_future_objs.at(i+1).id;
      gap_range.allow_min_t = car_range.t;
      if (!IfArriveGap(car_range, gap_range, t_future_objs, i)) continue;
      if (!IfLCIntoGap(car_range, gap_range, t_future_objs, i)) continue;
      LCOverSafeLevel(car_range, gap_range, t_future_objs, i);
      bool find_init_gap = false;
      for (auto &line_gap : input_line_ptr_->line_gap) {
        if (line_gap.start_id == gap_range.start_id && line_gap.end_id == gap_range.end_id) {
          if (line_gap.safety_level == 0 || gap_range.safety_level > line_gap.safety_level &&
              gap_range.allow_min_t < line_gap.allow_min_t + 1.0) {
            line_gap = gap_range;
          }
          find_init_gap = true;
          break;
        }
      }
      if (!find_init_gap) {
        input_line_ptr_->line_gap.push_back(gap_range);     
      } 
      if (gap_range.safety_level == 2) break;
    }
    bool all_gap_great_flag = true;
    for (auto &line_gap : input_line_ptr_->line_gap) {
      if (line_gap.safety_level != 2) {
        all_gap_great_flag = false;
        break;
      }
    }
    if (all_gap_great_flag || car_range.t > gap_param_.max_prelc_time) break;
  }
  for (auto &line_gap : input_line_ptr_->line_gap) {
    AERROR_IF(FLAGS_log_enable)<<"**Gap "<<line_gap.start_id<<" to "<<line_gap.end_id
            <<" safety_level "<<line_gap.safety_level
            <<" min t "<<line_gap.allow_min_t<<" **";
  }
}

void LineGap::LineGapInit() {
  min_safety_dis_ = 0.0;
  input_line_ptr_->line_gap.clear();
  if (sorted_objects_.size() < 2) return;
  input_line_ptr_->line_gap.reserve((int)sorted_objects_.size() - 1);
  if (car_ranges_.empty()) return;
  auto &t_future_objs = car_ranges_.front().future_objs;
  for (int i = 0; i + 1 < t_future_objs.size(); i++) {
    GapStruct optional_gap;
    optional_gap.start_id = t_future_objs.at(i).id;
    optional_gap.end_id = t_future_objs.at(i+1).id;
    optional_gap.allow_min_t = car_ranges_.front().t;
    input_line_ptr_->line_gap.push_back(optional_gap);
  }
}

bool LineGap::IfArriveGap(CarRange_t &car_range, GapStruct &gap_range, 
                          vector<GapObject> &t_future_objs, int i) {
  if (t_future_objs.at(i).id == -2 && t_future_objs.at(i+1).id == -1) return true;
  gap_range.aim_min_s = t_future_objs.at(i).max_s + car_model_ptr_->back_over_hang;
  gap_range.aim_max_s = t_future_objs.at(i+1).min_s - car_model_ptr_->front_over_hang;
  if (t_future_objs.at(i).is_static) {//静态障碍物提前触发一点换道
    gap_range.aim_min_s -= car_model_ptr_->length;
  } else {
    min_safety_dis_ = fmax(gap_param_.back_buff, 
                           gap_param_.thw * t_future_objs.at(i).speed);
    min_safety_dis_ = fmin(min_safety_dis_, 10.0);
    gap_range.aim_min_s += min_safety_dis_;
  }
  if (t_future_objs.at(i+1).is_static) {
    gap_range.aim_max_s -= FLAGS_collision_buff;
  } else {
    gap_range.aim_max_s -= gap_param_.back_buff;
  }
  double gap_space = gap_range.aim_max_s - gap_range.aim_min_s;
  if (car_range.t < 0.1) {
    gap_range.aim_min_v = car_speed_ - 0.01;
    gap_range.aim_max_v = car_speed_ + 0.01;
  } else {
    gap_range.aim_min_v = 2.0 * gap_range.aim_min_s / car_range.t - car_speed_;
    gap_range.aim_max_v = 2.0 * gap_range.aim_max_s / car_range.t - car_speed_;
    gap_range.aim_min_v = fmax(0.0, gap_range.aim_min_v);
  }
  AINFO_IF(FLAGS_gaplog_enable)<<"future obj "<<t_future_objs.at(i).id
          <<" v "<<t_future_objs.at(i).speed
          <<" to "<<t_future_objs.at(i+1).id<<" v "<<t_future_objs.at(i+1).speed
          <<" gap range s ("<<gap_range.aim_min_s
          <<", "<<gap_range.aim_max_s<<") gap_space "<<gap_space
          <<", arrive gap need v ("<<gap_range.aim_min_v
          <<", "<<gap_range.aim_max_v<<"). car acc/dec v ("
          <<car_range.min_v<<", "<<car_range.max_v<<").";
  if (gap_range.aim_max_v < car_range.min_v - 0.01 ||
      gap_range.aim_min_v > car_range.max_v + 0.01) return false;
  gap_range.aim_min_v = fmax(gap_range.aim_min_v, car_range.min_v);
  gap_range.aim_min_v = fmax(gap_range.aim_min_v, 0.01);
  gap_range.aim_max_v = fmin(gap_range.aim_max_v, car_range.max_v);
  double delta_v = gap_range.aim_max_v - gap_range.aim_min_v;
  if (delta_v < -0.01 || gap_range.aim_max_s < 0.0 ||
      car_range.t < 0.1 && gap_range.aim_min_s > 0.0) {
    return false;
  }
  return (gap_space > 0.0);
}

bool LineGap::IfLCIntoGap(CarRange_t &car_range, GapStruct &gap_range, 
                          vector<GapObject> &t_future_objs, int i) {
  if (t_future_objs.at(i).id == -2 && t_future_objs.at(i+1).id == -1) return true;
  double lc_max_speed = 2.0 * (gap_range.aim_max_s - gap_range.aim_min_v * car_range.t)
                        / gap_param_.lc_time + 2.0 * t_future_objs.at(i+1).speed + 
                        t_future_objs.at(i+1).acc * gap_param_.lc_time - 
                        gap_range.aim_min_v;
  double lc_min_speed = 2.0 * (gap_range.aim_min_s - gap_range.aim_max_v * car_range.t - min_safety_dis_)
                        / gap_param_.lc_time + 2.0 * t_future_objs.at(i).speed + 
                        t_future_objs.at(i).acc * gap_param_.lc_time -
                        gap_range.aim_max_v;
  if (t_future_objs.at(i).id == -2 || lc_min_speed < 1e-3) {
    lc_min_speed = 0.0;
  } 
  // else {
  //   lc_min_speed = fmax(FLAGS_min_lc_speedtime * t_future_objs.at(i).speed, lc_min_speed);
  // }
  double lc_car_max_v = fmin(VT(gap_range.aim_max_v, gap_param_.lc_time), gap_param_.max_speed);
  double lc_car_min_v = fmax(VT(gap_range.aim_min_v, gap_param_.lc_time, -gap_param_.self_acc), 0.0);
  double station_speed = gap_param_.max_speed;
  StationLimit(station_speed, gap_range.aim_min_v * car_range.t);
  lc_car_max_v = fmin(lc_car_max_v, station_speed);
  AINFO_IF(FLAGS_gaplog_enable)<<"back speed "
          <<t_future_objs.at(i).speed<<" front speed "<<t_future_objs.at(i+1).speed
          <<" lc need speed ("<<lc_min_speed<<", "<<lc_max_speed
          <<"), car acc/dec v ("<<lc_car_min_v <<", "<<lc_car_max_v<<").";
  if (lc_min_speed > lc_max_speed || lc_max_speed < lc_car_min_v ||
      lc_min_speed > lc_car_max_v) return false;
  gap_range.allow_min_v = fmax(gap_range.aim_min_v, lc_car_min_v);
  gap_range.allow_min_v = fmax(gap_range.aim_min_v, lc_min_speed);
  gap_range.allow_max_v = fmin(gap_range.aim_max_v, lc_car_max_v);
  gap_range.allow_max_v = fmin(gap_range.aim_max_v, lc_max_speed);
  AINFO_IF(FLAGS_gaplog_enable)<<"gap_range.allow_max_v "
        <<gap_range.allow_max_v<<" min "<<gap_range.allow_min_v;
  return (gap_range.allow_max_v > gap_range.allow_min_v - 0.1);
}

void LineGap::LCOverSafeLevel(CarRange_t &car_range, GapStruct &gap_range, 
                              vector<GapObject> &t_future_objs, int i) {
  // 如果能和前车保证ttc+thw，和后车保证ttc+thw，认为是2
  // 如果能和前车保证thw，和后车保证thw，认为1
  // 和后车的thw在第一步筛选就考虑了
  if (t_future_objs.at(i).id == -2 && t_future_objs.at(i+1).id == -1) {
    gap_range.safety_level = 2;
    return;
  }
  double before_lc_s = gap_range.aim_max_v * car_range.t - gap_range.aim_min_s + min_safety_dis_;
  // 假定自车换道开始如果对后车有影响，后车会减速
  double car_lc_s = 0.5*(gap_range.aim_max_v + gap_range.allow_max_v) * gap_param_.lc_time;
  double obj_lc_s = t_future_objs.at(i).speed * gap_param_.lc_time;
  double after_lc_s = before_lc_s + car_lc_s - obj_lc_s;
  AINFO_IF(FLAGS_gaplog_enable)<<"Gap length "<<before_lc_s
        <<", after_ttc_gap length "<<after_lc_s;
  if (after_lc_s < 0.0) {
    gap_range.safety_level = 0;
    return;
  }
  // 计算ttc时候把扣除掉的thw补回来
  double back_ttc = after_lc_s / (t_future_objs.at(i).speed - gap_range.aim_max_v);
  double back_thw = after_lc_s / t_future_objs.at(i).speed;
  if (back_ttc >= gap_param_.ttc || back_ttc < -0.01) {
    gap_range.safety_level = 2;
  } else if (back_thw > 1.5 * gap_param_.thw) {
    gap_range.safety_level = 2;
  } else if (back_thw > gap_param_.thw) {
    gap_range.safety_level = 1;
  }
}

double LineGap::VT(double& init_v, double &t, double a) {
  double limit_a = 0.5, max_v;
  // 自车起步时候加速度给小一点
  if (init_v < 2.0) {
    if (t < (2.0 - init_v) / limit_a) {
      max_v = init_v + limit_a * t;
    } else {
      max_v = 2.0 + (t - (2.0 - init_v) / limit_a) * a;
    }
  } else {
    max_v = init_v + t * a;
  }
  
  return max_v;
}

void LineGap::NeedUpperSpeedLimit(double &allow_max_v, double &allow_max_s) {
  auto &expected_speeds = input_line_ptr_->mapinfo.expected_speeds;
  if (expected_speeds.empty()) return;
  int index = -1;
  bool need_upper_flag = false;
  for (int i = 0; i < expected_speeds.size(); i++) {
    index = i;
    if (expected_speeds.at(i).second < allow_max_v &&
        expected_speeds.at(i).second > 10.0) {
      expected_speeds.at(i).second = allow_max_v;
    }
    if (expected_speeds.at(i).first > allow_max_s + FLAGS_front_perception_range) {
      break;
    }
  }
}

}
}
