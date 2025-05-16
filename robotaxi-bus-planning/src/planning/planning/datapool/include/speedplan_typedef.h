#ifndef DATAPOOL_INCLUDE_SPEEDPLAN_TYPEDEF_H_
#define DATAPOOL_INCLUDE_SPEEDPLAN_TYPEDEF_H_

#include "public_typedef.h"

namespace acu {
namespace planning {

enum class eSpeedPlanMode {
  DEFAULT = 0,
  LOW_SPEED,
  MIDDLE_SPEED,
  HIGH_SPEED
};

typedef struct SpeedPlanMode {
  eSpeedPlanMode speedplan_mode;
  std::string speedplan_mode_str;
  SpeedPlanMode() {
    speedplan_mode = eSpeedPlanMode::DEFAULT;
    speedplan_mode_str = "default";
  }
  void SetLowSpeedMode() {
    speedplan_mode = eSpeedPlanMode::LOW_SPEED;
    speedplan_mode_str = "low_speed";
  }
  void SetMiddleSpeedMode() {
    speedplan_mode = eSpeedPlanMode::MIDDLE_SPEED;
    speedplan_mode_str = "middle_speed";
  }
  void SetHighSpeedMode() {
    speedplan_mode = eSpeedPlanMode::HIGH_SPEED;
    speedplan_mode_str = "high_speed";
  }
} SpeedPlanMode;

struct SpeedplanInfo {
  bool is_passable;
  bool pathReplanFlag;
  int pathDirection;
  int pathplannerGo;
  double pathplannerSpeedLimit;
  double velocity_limit;
  bool business_speed_limit;
  double stop_dis;
  double stop_dis_AC;
  double stop_dis_fs;//added by DZ
  bool is_final_adjust;
  SpeedPlanMode speedplan_mode;
  PathData path;
  CollisionInfoVec collision_info_vec;
  SpeedplanInfo() {
    is_passable = false;
    pathDirection = 0;
    pathplannerSpeedLimit = 0.5;
    pathplannerGo = 0;
    pathReplanFlag = false;
    velocity_limit = 2.0;
    business_speed_limit = false;
    stop_dis = std::numeric_limits<double>::max();
    stop_dis_AC = std::numeric_limits<double>::max();
    stop_dis_fs = std::numeric_limits<double>::max();
    collision_info_vec.clear();
    is_final_adjust = false;
  }
};

}
}

#endif