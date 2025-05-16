#ifndef DATAPOOL_INCLUDE_COMMON_CONFIG_H_
#define DATAPOOL_INCLUDE_COMMON_CONFIG_H_

#include <string>
#include <vector>
#include <map>
#include <list>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <limits>
#include "common/base/config/include/param_config_manager.h"


namespace acu {
namespace planning {

enum class SyncEnum {
  NOSYNC = 0,
  IMU = 1,
  VISION = 2,
  MAX_TYPE
};

struct CarModel {
  double length;
  double car_width;
  double length_wheelbase;
  double front_axle_tofront;
  double wheel_center;
  double min_turning_radius;
  double max_speed; //暂时默认120
  double max_accelaration;
  double max_braking_value;
  double min_drive_speed; //暂时不加
  double slide_accelaration;//测出来的
  double eps_transmission_ratio;
  double right_max_steerangle;
  double front_over_hang;
  double back_over_hang;
  double half_wheel;
  SyncEnum sync_type;
  CarModel() {
    Reset();
  }
  void Reset() {
    length = 0.0;
    car_width = 0.0;
    length_wheelbase = 0.0;
    front_axle_tofront = 0.0;
    wheel_center = 0.0;
    min_turning_radius = 0.0;
    max_speed = 0.0;
    max_accelaration = 0.0;
    max_braking_value = 0.0;
    min_drive_speed = 0.0;
    slide_accelaration = 0.0;
    eps_transmission_ratio = 0.0;
    right_max_steerangle = 0.0;
    front_over_hang = 0.0;
    back_over_hang = 0.0;
    half_wheel = 0.0;
    sync_type = SyncEnum::NOSYNC;
  }
  void Set(double frontoverhang, double backoverhang, double halfwheel) {
    front_over_hang = frontoverhang;
    back_over_hang = backoverhang;
    half_wheel = halfwheel;
  }
  bool LoadVeicleParamsFromModel(ModelConfig *pmodel_config) {
    if (pmodel_config->GetValue("car_length", &length) == false) {
      std::cout << "cannot get car_length" << std::endl;
      return false;
    }
    if (pmodel_config->GetValue("car_width", &car_width) == false) {
      std::cout << "cannot get car_width" << std::endl;
      return false;
    }
    if (pmodel_config->GetValue("wheel_base_length", &length_wheelbase) == false) {
      std::cout << "cannot get length_wheel_base" << std::endl;
      return false;
    }
    if (pmodel_config->GetValue("front_axle_tofront", &front_axle_tofront) == false) {
      std::cout << "cannot get front_axle_tofront" << std::endl;
      return false;
    }
    if (pmodel_config->GetValue("wheel_center", &wheel_center) == false)  {std:
      std::cout << "cannot get wheel_center" << std::endl;
      return false;
    }
    if (pmodel_config->GetValue("min_turn_radius", &min_turning_radius) == false) {
      std::cout << "cannot get min_turn_radius" << std::endl;
      return false;
    }
    if (pmodel_config->GetValue("acel_max", &max_accelaration) == false) {
      std::cout << "cannot get acel_max" << std::endl;
      return false;
    }
    if (pmodel_config->GetValue("plan_emergency_brake_accel", &max_braking_value) == false) {
      std::cout << "cannot get emergency_brake_accel" << std::endl;
      return false;
    }
    if (pmodel_config->GetValue("right_reduce_ratio", &eps_transmission_ratio) == false) {
      std::cout << "cannot get right_reduce_ratio" << std::endl;
      return false;
    }
    if (pmodel_config->GetValue("right_max_steerangle", &right_max_steerangle) == false) {
      std::cout << "cannot get right_max_steerangle" << std::endl;
      return false;
    }
    int synchronization = 0;
    if (pmodel_config->GetValue("synchronization", &synchronization) == false) {
      std::cout << "cannot get synchronization" << std::endl;
      return false;
    }
    sync_type = (SyncEnum)synchronization;
    front_over_hang = length_wheelbase + front_axle_tofront;
    back_over_hang = length - front_over_hang;
    half_wheel = car_width / 2.0;
    return true;
  }
};

struct TaskConfig {
  bool enable_unstruct;
  bool enable_struct;
  TaskConfig() {
    enable_unstruct = true;
    enable_struct = true;
  }
};

struct CognitionConfig {
  bool enable_acurracy_control;
  double set_scale_x;
  double set_scale_xb;
  double set_scale_y;
  double speed_threshold;
  double high_speed_threshold;
  double result_path_dis;
  double front_path_dis;
  CognitionConfig() {
    enable_acurracy_control = false;
    set_scale_x = 10.0;
    set_scale_xb = -5.0;
    set_scale_y = 8.0;
    speed_threshold = 0.5; // m/s
    high_speed_threshold = 10.0; // m/s
    result_path_dis = 6.5;
    front_path_dis = 10.0;
  }
  bool IsInArea(double x, double y) {
    if (x > set_scale_xb && x < set_scale_x && fabs(y) < set_scale_y)
      return true;
    return false;
  }
};

struct BehaviorConfig {
  bool enable_unstruct_avoid;
  bool enable_unstruct_back_follow;
  bool enable_unstruct_yield;
  bool enable_unstruct_adjust;
  bool enable_border;
  int unstruct_avoid_level;

  bool enable_struct_following;
  bool enable_struct_free_lanechange;
  bool enable_struct_force_lanechange;
  bool enable_struct_obs_avoid;
  bool enable_struct_cross_yield;
  bool enable_struct_narrow_yield;
  bool enable_struct_pull_over;
  int following_radicalness;
  int avoid_radicalness;
  int lanechange_radicalness; 

  int forward_stay_still_times_setting;
  int back_stay_still_times_setting;
  int back_stay_still_failed_times_setting;
  int back_following_times_setting;
  int time_for_dwa_setting;
  int time_for_ha_setting;
  double curvature_threshold;
  double slow_dis_threshold;
  double deviation_dis_err;
  double deviation_angle_err;
  double detect_height_threshold;
  double back_follow_dis_threshold;


  BehaviorConfig() {
    enable_unstruct_avoid = true;
    enable_unstruct_back_follow = true;
    enable_unstruct_yield = false;
    enable_unstruct_adjust = true;
    enable_border = false;
    unstruct_avoid_level = 1;
    enable_struct_following = true;
    enable_struct_free_lanechange = true;
    enable_struct_force_lanechange = true;
    enable_struct_obs_avoid = true;
    enable_struct_pull_over = true;
    enable_struct_cross_yield = true;
    enable_struct_narrow_yield = true;
    following_radicalness = 0;
    avoid_radicalness = 0;
    lanechange_radicalness = 0;

    forward_stay_still_times_setting = 30;
    back_stay_still_times_setting = 1200000;
    back_stay_still_failed_times_setting = 1200;
    back_following_times_setting = 150;
    time_for_dwa_setting = 5;
    time_for_ha_setting = 50;
    curvature_threshold = 1.0 / 1.2;
    slow_dis_threshold = 1.7;
    deviation_dis_err = 0.4;
    deviation_angle_err = 90.0;
    detect_height_threshold = 0.2;
    back_follow_dis_threshold = 3.0;
  }
  bool LoadBehviorConfigParamsFromModel(ModelConfig *pmodel_config) {
    if (pmodel_config->GetValue("enable_struct_following", &enable_struct_following) == false) {
      std::cout << "cannot get enable_struct_following" << std::endl;
      return false;
    }
    if (pmodel_config->GetValue("enable_unstruct_back_follow", &enable_unstruct_back_follow) == false) {
      std::cout << "cannot get enable_unstruct_back_follow" << std::endl;
      return false;
    }
    if (pmodel_config->GetValue("enable_unstruct_avoid", &enable_unstruct_avoid) == false) {
      std::cout << "cannot get enable_unstruct_avoid" << std::endl;
      return false;
    }
    if (pmodel_config->GetValue("unstruct_avoid_level", &unstruct_avoid_level) == false) {
      std::cout << "cannot get enable_unstruct_avoid" << std::endl;
      return false;
    }
    return true;
  }
};

struct PathplanConfig {
  bool enable_unstruct_dwa;
  bool enable_ss_plan;
  bool enable_dynamic_plan;
  bool enable_hybrid_astar;

  bool enable_alc;
  bool enable_struct_dwa;
  bool enable_em;
  PathplanConfig() {
    enable_unstruct_dwa = true;
    enable_ss_plan = true;
    enable_dynamic_plan = true;
    enable_hybrid_astar = true;
    enable_alc = true;
    enable_struct_dwa = true;
    enable_em = true;
  }
};

struct SpeedplanConfig {
  double braking_dis;
  double maximum_cruising_speed;
  double maximum_junction_speed;
  double maximum_bump_speed;
  SpeedplanConfig() {
    Reset();
  }
  void Reset() {
    braking_dis = 0.0;
    maximum_cruising_speed = 60.0;
    maximum_junction_speed = 60.0;
    maximum_bump_speed = 60.0;
  }
  bool LoadSpeedplanConfigParamsFromModel(ModelConfig *pmodel_config) {
    if (pmodel_config->GetValue("maximum_cruising_speed", &maximum_cruising_speed) == false) {
      std::cout << "cannot get maximum_cruising_speed" << std::endl;
      return false;
    }
    if (pmodel_config->GetValue("maximum_junction_speed", &maximum_junction_speed) == false) {
      std::cout << "cannot get maximum_junction_speed" << std::endl;
      return false;
    }
    if (pmodel_config->GetValue("maximum_bump_speed", &maximum_bump_speed) == false) {
      std::cout << "cannot get maximum_bump_speed" << std::endl;
      return false;
    }
    return true;
  }
};
struct BusinessConfig {
  bool  enable_use_decision_point_calate_dis2end;
  double minimum_dist_of_stop_mission;
  double maximum_dist_of_stop_mission;
  double park_dist_of_stop_mission;
  BusinessConfig() {
    Reset();
  }
  void Reset() {
    enable_use_decision_point_calate_dis2end = false;
    minimum_dist_of_stop_mission = 2;
    maximum_dist_of_stop_mission = 20;
    park_dist_of_stop_mission = 2;
  }
  bool LoadBusinessConfigParamsFromModel(ModelConfig *pmodel_config) {
    if (pmodel_config->GetValue("enable_use_decision_point_calate_dis2end", &enable_use_decision_point_calate_dis2end) == false) {
      std::cout << "cannot get enable_use_decision_point_calate_dis2end" << std::endl;
      return false;
    }
    if (pmodel_config->GetValue("minimum_dist_of_stop_mission", &minimum_dist_of_stop_mission) == false) {
      std::cout << "cannot get minimum_dist_of_stop_mission" << std::endl;
      return false;
    }
    if (pmodel_config->GetValue("maximum_dist_of_stop_mission", &maximum_dist_of_stop_mission) == false) {
      std::cout << "cannot get maximum_dist_of_stop_mission" << std::endl;
      return false;
    }
    if (pmodel_config->GetValue("park_dist_of_stop_mission", &park_dist_of_stop_mission) == false) {
      std::cout << "cannot get park_dist_of_stop_mission" << std::endl;
      return false;
    }
    return true;
  }
};


typedef struct ConfigInfo {
  CarModel car_model;
  TaskConfig task_config;
  CognitionConfig cognition_config;
  BehaviorConfig behavior_config;
  PathplanConfig pathplan_config;
  SpeedplanConfig speedplan_config;
  BusinessConfig business_config;
  std::string car_type; //add by ly
} ConfigInfo;

}
}

#endif
