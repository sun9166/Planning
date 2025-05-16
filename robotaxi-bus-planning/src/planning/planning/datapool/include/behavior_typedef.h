#ifndef DATAPOOL_INCLUDE_BEHAVIOR_TYPEDEF_H_
#define DATAPOOL_INCLUDE_BEHAVIOR_TYPEDEF_H_

#include "public_typedef.h"
#include "cognition_typedef.h"
#include "locperception_input.h"
#include "common_config.h"
#include "speedplan_typedef.h"
#include "src/algorithm/interface/interface_base/include/algorithm_base.h"


namespace acu {
namespace planning {

enum class eBehaviorStatus {
  DEFAULT = 0,
  IDLE,
  FORWARDFOLLOWING,
  SLOW_DOWN,
  BACKFOLLOWING,
  FORCE_AVOID,
  FORWARD_AVOID,
  BACK_AVOID,
  NARROW_TURN,
  FORWARD_STAY_STILL,
  BACK_STAY_STILL,
  YIELD,
  BEHAVIOR_ERROR
};

typedef struct BehaviorStatus {
  eBehaviorStatus status;
  std::string status_str;
  BehaviorStatus() {
    status = eBehaviorStatus::DEFAULT;
    status_str = "default";
  }

  void Reset() {
    status = eBehaviorStatus::DEFAULT;
    status_str = "default";
  }
  void SetIdle() {
    status = eBehaviorStatus::IDLE;
    status_str = "behavior idle";
  }

  void SetForwardFollowing() {
    status = eBehaviorStatus::FORWARDFOLLOWING;
    status_str = "ForwardFollowing";
  }

  void SetSlowDown() {
    status = eBehaviorStatus::SLOW_DOWN;
    status_str = "slow down";
  }

  void SetBackFollowing() {
    status = eBehaviorStatus::BACKFOLLOWING;
    status_str = "BackFollowing";
  }

  void SetForwardAvoid() {
    status = eBehaviorStatus::FORWARD_AVOID;
    status_str = "ForwardAvoid";
  }

  void SetBackAvoid() {
    status = eBehaviorStatus::BACK_AVOID;
    status_str = "BackAvoid";
  }

  void SetForwardStayStill() {
    status = eBehaviorStatus::FORWARD_STAY_STILL;
    status_str = "forward stay still";
  }
  void SetBackStayStill() {
    status = eBehaviorStatus::BACK_STAY_STILL;
    status_str = "back stay still";
  }

  void SetYield() {
    status = eBehaviorStatus::YIELD;
    status_str = "Yield";
  }

  void SetForceAvoid() {
    status = eBehaviorStatus::FORCE_AVOID;
    status_str = "force avoid";
  }

  void SetBehaviorError() {
    status = eBehaviorStatus::BEHAVIOR_ERROR;
    status_str = "behavior error";
  }

  void SetNarrowTurn() {
    status = eBehaviorStatus::NARROW_TURN;
    status_str = "narrow turn";
  }
} BehaviorStatus;


enum class eBehaviorBusyStatus {
  DEFAULT = 0,
  BUSY_STATUS,
  NORMAL_STATUS
};
typedef struct BehaviorBusyStatus
{
  eBehaviorBusyStatus status;
  std::string status_str;
  BehaviorBusyStatus() {
    status = eBehaviorBusyStatus::DEFAULT;
    status_str = "default";
  }

  void SetBusyStatus() {
    status = eBehaviorBusyStatus::BUSY_STATUS;
    status_str = "busy_status";
  }

  void SetNormalStatus() {
    status = eBehaviorBusyStatus::NORMAL_STATUS;
    status_str = "normal_status";
  }
  void Reset() {
    status = eBehaviorBusyStatus::DEFAULT;
    status_str = "default";
  }
} BehaviorBusyStatus;

enum class eBehaviorExecuteStatus {
  DEFAULT = 0,
  EXECUTING ,
  EXE_FAILED,
  EXE_SUCCESS
};
typedef struct BehaviorExecuteStatus {
  eBehaviorExecuteStatus status;
  std::string status_str;
  BehaviorExecuteStatus() {
    status = eBehaviorExecuteStatus::DEFAULT;
    status_str = "Default";
  }
  void Reset() {
    status = eBehaviorExecuteStatus::DEFAULT;
    status_str = "Default";
  }
  void SetExecuting() {
    status = eBehaviorExecuteStatus::EXECUTING;
    status_str = "executing";
  }

  void SetExecutFailed() {
    status = eBehaviorExecuteStatus::EXE_FAILED;
    status_str = "ExecutFailed";
  }

  void SetExecutSuccess() {
    status = eBehaviorExecuteStatus::EXE_SUCCESS;
    status_str = "ExecutSuccess";
  }
} BehaviorExecuteStatus;

typedef struct BehaviorFSMInfo {
  BehaviorStatus behavior_status;

  BehaviorBusyStatus behavior_busy_status;

  BehaviorExecuteStatus forward_following_status;
  BehaviorExecuteStatus back_following_status;
  BehaviorExecuteStatus forward_avoid_status;
  BehaviorExecuteStatus back_avoid_status;
  BehaviorExecuteStatus yield_status;
  BehaviorExecuteStatus force_avoid_status;
  BehaviorExecuteStatus narrow_turn_status;
  BehaviorExecuteStatus forward_stay_still_status;
  BehaviorExecuteStatus back_stay_still_status;
  BehaviorExecuteStatus slow_down_status;

  StatisticsTimes forward_stay_still_times;
  StatisticsTimes back_stay_still_times;
  StatisticsTimes back_following_times;

  BehaviorExecuteStatus dwa_result_status;
  BehaviorExecuteStatus astar_result_status;

  StatisticsTimes time_for_dwa;
  StatisticsTimes time_for_hybrid_atar;

  double back_distance;
  bool is_calculating_back_dis;
  double last_xg;
  double last_yg; 

  BehaviorFSMInfo() {
    Reset();
    // path_collision_times.SetStatisticsTimes(10, 0.8);
    // path_collision_times_AC.SetStatisticsTimes(10, 0.8);
    // path_nocollision_times.SetStatisticsTimes(10, 0.8);
    // path_nocollision_times_AC.SetStatisticsTimes(10, 0.8);
    // path_emergency_times.SetStatisticsTimes(10, 0.8);
    // slow_down_dis = 1000;
    // is_calculating_back_dis = false;
    // last_xg = 0.0;
    // last_yg = 0.0;
  }

  void Reset() {
    behavior_status.Reset();
    behavior_busy_status.Reset();
    time_for_dwa.SetStatisticsTimes(5);
    time_for_hybrid_atar.SetStatisticsTimes(50);
    back_following_times.SetStatisticsTimes(150);
    back_distance = 0.0;
    is_calculating_back_dis = false;
    last_xg = 0.0;
    last_yg = 0.0;
  }
  void ResetCalculateBackDis() {
    back_distance = 0.0;
    is_calculating_back_dis = false;
    last_xg = 0.0;
    last_yg = 0.0;
  }
  bool IsNeedCallDwa() {
    if (time_for_dwa.IsTimesUp()) return true;
    return false;
  }

  bool IsNeedCallHybridAstar() {
    if (time_for_hybrid_atar.IsTimesUp()) return true;
    return false;
  }
  bool IsDWAResultSuccess() {
    // todo
    return (dwa_result_status.status == eBehaviorExecuteStatus::EXE_SUCCESS);
  }
  bool IsHybridAstarSuccess() {
    // todo
    return (astar_result_status.status == eBehaviorExecuteStatus::EXE_SUCCESS);
  }
  void ResetDWA() {
    dwa_result_status.Reset();
  }
  void ResetHybridAstar() {
    astar_result_status.Reset();
  }
  bool IsForceAvoidFinish() {
    // todo
    return (force_avoid_status.status == eBehaviorExecuteStatus::EXE_SUCCESS);
  }
  bool IsForceAvoidFault() {
    // todo
    return (force_avoid_status.status == eBehaviorExecuteStatus::EXE_FAILED);
  }
  bool IsForwardStayTimeOut() {
    // todo
    return forward_stay_still_times.IsTimesUp();
  }
  bool IsBackStayTimeOut() {
    // todo
    return back_stay_still_times.IsTimesUp();
  }
  bool IsBackFollowingFinish() {
    return back_following_times.IsTimesUp();
  }
  bool IsForwardAvoidFinish() {
    return (forward_avoid_status.status == eBehaviorExecuteStatus::EXE_SUCCESS);
  }
  bool IsBackAvoidFinish() {
    return (back_avoid_status.status == eBehaviorExecuteStatus::EXE_SUCCESS);
  }
} BehaviorFSMInfo;

typedef struct BehaviorFSMParam {
  BehaviorFSMInfo behavior_fsm_info;
  VehicleDriveStatus drive_status;
  UnstructDynamicInfo dynamic_info;
  UnstructSemanticInfo semantic_info;
  LocalizationData localization_data;
  BehaviorConfig behavior_config;
  SpeedplanInfo speedplan_info;
  ExceptionAnalyseResult exception_analyse_result;

  std::shared_ptr<AlgorithmBase> algorithm_ptr;

  PathData last_result_path;
  PathData result_path;
  PathData calculate_path;
  Paths paths;

  DebugPlanningMsg debug_planning_msg;

} BehaviorFSMParam;

} // namespace planning
} // namespace acu

#endif // DATAPOOL_INCLUDE_BEHAVIOR_TYPEDEF_H_