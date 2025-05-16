#ifndef DATAPOOL_INCLUDE_COGNITION_TYPEDEF_H_
#define DATAPOOL_INCLUDE_COGNITION_TYPEDEF_H_

#include "public_typedef.h"
#include "mapengine_input.h"
#include "prediction_input.h"
#include "locperception_input.h"
#include "task_input.h"
#include "common/toolbox/geometry/include/dwa_header.h"
#include "common/toolbox/geometry/include/geoheader.h"
#include "common/math/vec2d.h"
#include "common/math/box2d.h"
#include "referenceline_frame/referenceline_frame.h"

namespace acu {
namespace planning {

enum class eCarRiskStatus {
  FREE = 0,
  LIGHTLY_RISKY = 1,
  EXTREMELY_RISKY = 2
};

struct CollisionCell {
  int id;
  int type;
  double x;
  double y;
  double z;
  double height;
  CollisionCell() {
    Reset();
  }
  void Reset() {
    id = -1;
    type = -1;
    x = 0.0;
    y = 0.0;
    z = 0.0;
    height = 2.0;
  }
};

struct CollisionInfo {
  CollisionCell cell_info;
  geometry::Site path_pt_info;
  StatisticsDetailTimes collid_times;
  CollisionInfo() {
    Reset();
  }
  void Reset() {
    cell_info.Reset();
    path_pt_info.Reset();
    path_pt_info.index = 9999;
    path_pt_info.length = 999.0;
    collid_times.SetStatisticsTimes(10, 0.8);
  }
};

struct UnstructStaticInfo {
  double dis_fs_left;
  double dis_fs_right;
  PathData front_local_path;
  PathData front_pursuit_path;
  PathData back_local_path;
  PathData front_raw_path;
  PathData result_path;
  PathData result_pursuit_path;
  geometry::Site start_point;
  geometry::Site end_point;
  UnstructStaticInfo() {
    Reset();
  }
  void Reset() {
    dis_fs_left = 0.0;
    dis_fs_right = 0.0;
    front_local_path.Reset();
    front_pursuit_path.Reset();
    back_local_path.Reset();
    front_raw_path.Reset();
    result_path.Reset();
    result_pursuit_path.Reset();
    start_point.Reset();
    end_point.Reset();
  }
};

struct UnstructDynamicInfo {
  acu::dwa::SortedTree dynamic_obj;
  acu::dwa::SortedTree static_obj;
  acu::dwa::SortedTree static_lidar_cell;
  acu::dwa::SortedTree static_ultrasonic_cell;
  UnstructDynamicInfo() {
    Reset();
  }
  void Reset() {
    dynamic_obj.clear();
    static_obj.clear();
    static_lidar_cell.clear();
    static_ultrasonic_cell.clear();
  }
};

struct UnstructPredictInfo {
  
  std::vector<std::vector<MapEnginePoint>> predict_lanes;
  UnstructPredictInfo() {
    Reset();
  }
  void Reset() {
    predict_lanes.clear();
  }
  bool GetpointsByID() {
    return true;  
  }
};

struct UnstructSemanticInfo {
  eCarRiskStatus curent_car_risk;
  double slow_down_dis;
  CollisionInfo result_fs_collision_info;
  CollisionInfo result_dynamic_obj_collision_info;
  CollisionInfo result_static_obj_collision_info;
  CollisionInfo result_static_lidar_cell_collision_info;
  CollisionInfo result_static_ultra_cell_collision_info;
  CollisionInfo result_info;
  CollisionInfo front_fs_collision_info;
  CollisionInfo front_dynamic_obj_collision_info;
  CollisionInfo front_static_obj_collision_info;
  CollisionInfo front_static_lidar_cell_collision_info;
  CollisionInfo front_static_ultra_cell_collision_info;
  CollisionInfo front_info;
  StatisticsDetailTimes path_resultcollision_times;
  StatisticsDetailTimes path_frontcollision_times;
  StatisticsDetailTimes path_emergency_times;
  StatisticsTimes time_for_abnormal_endding;
  UnstructSemanticInfo() {
    Reset();  
  }
  void Reset() {
    curent_car_risk = eCarRiskStatus::FREE;
    slow_down_dis = 9999.9;
    result_fs_collision_info.Reset();
    result_dynamic_obj_collision_info.Reset();
    result_static_obj_collision_info.Reset();
    result_static_lidar_cell_collision_info.Reset();
    result_static_ultra_cell_collision_info.Reset();
    result_info.Reset();
    front_fs_collision_info.Reset();
    front_dynamic_obj_collision_info.Reset();
    front_static_obj_collision_info.Reset();
    front_static_lidar_cell_collision_info.Reset();
    front_static_ultra_cell_collision_info.Reset();
    front_info.Reset();
    path_resultcollision_times.SetStatisticsTimes(10, 0.8);
    path_frontcollision_times.SetStatisticsTimes(10, 0.8);
    path_emergency_times.SetStatisticsTimes(10, 0.8);
    time_for_abnormal_endding.SetStatisticsTimes(200000);
  }
  void UpdateNewResultCollision() {
    path_resultcollision_times.NewPositiveData();
  }
  void UpdateNonResultCollision() {
    path_resultcollision_times.NewNegativeData();
  }
  bool IsPathCollisionTimeUp() {
    return path_resultcollision_times.IsTimesUp();
  }
  void ResetResultCollision() {
    path_resultcollision_times.Reset();
  }
  void UpdateNewFrontCollision() {
    path_frontcollision_times.NewPositiveData();
  }
  void UpdateNonFrontCollision() {
    path_frontcollision_times.NewNegativeData();
  } 
  bool IsPathNoCollisionTimeUp() {
    return path_frontcollision_times.IsNoTimesUp();
  }
  void ResetFrontCollision() {
    path_frontcollision_times.Reset();
  }
  void UpdateNewEmergency() {
    path_emergency_times.NewPositiveData();
  }
  void UpdateNonEmergency() {
    path_emergency_times.NewNegativeData();
  }
  bool IsEmergencyTimeUp() {
    return path_emergency_times.IsTimesUp();
  }
  void ResetEmergency() {
    path_emergency_times.Reset();
  }

  bool GetYieldPoint() {
    return true;
  }
  bool GetCenterLane() {
    return true;
  }
};

struct UnstructEnv {
  UnstructStaticInfo static_info;
  UnstructDynamicInfo dynamic_info;
  UnstructPredictInfo predict_info;
  UnstructSemanticInfo semantic_info;
  UnstructEnv() {
    Reset();
  }
  void Reset() {
    static_info.Reset();
    dynamic_info.Reset();
    predict_info.Reset();
    semantic_info.Reset();
  }
};



struct StructLocalizationInfo {
  double time_stamp;
  double xg;
  double yg;
  double global_angle;
  double dr_x;
  double dr_y;
  double dr_angle;
  double loc_velocity;
  double angular_velocity;
  StructLocalizationInfo() {
    Reset();
  }
  void Reset() {
    time_stamp = 0.0;
    xg = 0.0;
    yg = 0.0;
    global_angle = 0.0;
    dr_x = 0.0;
    dr_y = 0.0;
    dr_angle = 0.0;
    loc_velocity = 0.0;
    angular_velocity = 0.0;
  }
  void Set(const LocalizationData &input) {
    time_stamp = input.time;
    xg = input.xg;
    yg = input.yg;
    global_angle = input.yaw;
    dr_x = input.loc_xg_dr;
    dr_y = input.loc_yg_dr;
    dr_angle = input.loc_yaw_dr;
    loc_velocity = input.velocity;
    angular_velocity = input.yawrate;
  }
};

struct StructChassisInfo {
  double time_stamp;
  int drive_mode;
  int drive_state;  // -1 : manual 0 : waiting 1 : auto
  double chassis_velocity;
  double steering_angle;
  int drive_gear;
  double velocity;
  double vehicle_accel;
  double kappa_curvature;
  int brake_state;
  int on_accpedal;
  StructChassisInfo() {
    Reset();
  }
  void Reset() {
    time_stamp = 0.0;
    drive_mode = 0;
    drive_state = 0;
    chassis_velocity = 0.0;
    steering_angle = 0.0;
    drive_gear = 0;
    velocity = 0;
    vehicle_accel = 0;
    kappa_curvature = 0.0;
    brake_state = 0;
    on_accpedal = 0;
  }
  void Set(const VehicleDriveStatus &input) {
    time_stamp = input.timestamp;
    drive_mode = input.control_mode;
    chassis_velocity = input.velocity;
    steering_angle = input.steer_angle;
    drive_gear = input.shiftlvlposition;
    velocity = input.velocity;
    vehicle_accel = input.vehicle_accel;
    kappa_curvature = 0.0;
    brake_state = input.brake_state;
    on_accpedal = input.on_accpedal;
  }
};

struct StructVehicleInfo {
  StructLocalizationInfo localization;
  StructChassisInfo chassis;
  StructVehicleInfo() {
    Reset();
  }
  void Reset() {
    localization.Reset();
    chassis.Reset();
  }
};



struct StructReferenceLineInfo {
  double time_stamp;
  int current_line_id;
  int target_line_id;
  int rightest_line_id;
  std::vector<ReferenceLineFrame> current_reference_line;
  std::vector<ReferenceLineFrame> left_reference_line;
  std::vector<ReferenceLineFrame> right_reference_line;
  ReferenceLineFrame reverse_reference_line;
  ReferenceLineFrame local_reference_line;
  double limit_speed;
  bool road_blocked;
  bool pathbox_in_current;
  bool path_in_current;
  bool path_in_target;
  bool task_change;
  StructReferenceLineInfo() {
    Reset();
  }
  void ClearData() {
    for (int i = 0; i < current_reference_line.size(); i++) {
      current_reference_line.at(i).ClearData();
    }
    for (int i = 0; i < left_reference_line.size(); i++) {
      left_reference_line.at(i).ClearData();
    }
    for (int i = 0; i < right_reference_line.size(); i++) {
      right_reference_line.at(i).ClearData();
    }
    local_reference_line.ClearData();
    reverse_reference_line.ClearData();
    road_blocked = false;
    rightest_line_id = -1;
    current_line_id = 0;
    limit_speed = 0.0;
    task_change = false;
  }
  void Reset() {
    current_reference_line.clear();
    left_reference_line.clear();
    right_reference_line.clear();
    reverse_reference_line.Reset();
    local_reference_line.Reset();
    road_blocked = false;
    current_line_id = 0;
    rightest_line_id = -1;
    limit_speed = 0.0;
    target_line_id = 0;//单独更新
    pathbox_in_current = true;
    path_in_current = true;
    path_in_target = true;
    task_change = false;
  }
};

struct StructScenarioInfo {
  int waiting_status;
  int congestion_level;
  double aver_num;
  double aver_speed;
  bool enable_offset_;
  std::map<int, double> invader_;
  std::set<int> waiting_objects_;
  StructScenarioInfo() {
    Reset();
  }
  void Reset() {
    waiting_status = 0;
    congestion_level = 0;
    aver_num = 0;
    aver_speed = FLAGS_free_speed;
    enable_offset_ = false;
    // invader_.clear();
  }
};

struct StructEnv {
  StructVehicleInfo vehicle_info;
  StructReferenceLineInfo reference_line_info;
  StructScenarioInfo scenario_info;
  StructEnv() {
    Reset();
  }
  void Reset() {
    vehicle_info.Reset();
    reference_line_info.Reset();
    scenario_info.Reset();
  }
};

typedef struct CognitionInfo {
  eMapType maptype;
  UnstructEnv unstruct_env_info;
  StructEnv struct_env_info;
  void SetMapType(eMapType &type) {
    maptype = type;
  }
} CognitionInfo;

} // namespace planning
} // namespace acu

#endif // DATAPOOL_INCLUDE_COGNITION_TYPEDEF_H_
