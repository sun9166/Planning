#ifndef SRC_EXECUTION_BEHAVIORPLAN_NORMAL_BEHAVIOR_CONTEXT_H_
#define SRC_EXECUTION_BEHAVIORPLAN_NORMAL_BEHAVIOR_CONTEXT_H_

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <stddef.h>
#include <functional>
#include "src/execution/behaviorplan/normal_behavior/include/NormalBehaviorFSM.h"
#include "src/execution/behaviorplan/behavior_base/behavior_context_base.h"
#include "datapool/include/data_pool.h"

namespace acu {
namespace planning {

typedef std::function<bool (void)> pBehaviorEventFunction;

class NormalBehaviorContext : public BehaviorContextBase {
 public:
  NormalBehaviorContext() {
    behavior_context_type_.SetNormalBehavior();
    pbehavior_fsm_ = std::make_shared<NormalBehaviorFSM>((void*)0);
  };
  
  ~NormalBehaviorContext() {};
  
  void PullData() {
    auto DP = DataPool::Instance();
    behavior_param_.behavior_fsm_info = DP->GetMainDataRef().behavior_fsm_info;
    behavior_param_.debug_planning_msg = DP->GetMainDataRef().debug_planning_msg;
    behavior_param_.drive_status = DP->GetMainDataRef().drive_status;
    behavior_param_.dynamic_info = DP->GetMainDataRef().cognition_info.unstruct_env_info.dynamic_info;
    behavior_param_.semantic_info = DP->GetMainDataRef().cognition_info.unstruct_env_info.semantic_info;
    behavior_param_.localization_data = DP->GetMainDataRef().loc_perception.localization_data;
    behavior_param_.behavior_config = DP->GetMainDataRef().config_info.behavior_config;
    behavior_param_.speedplan_info = DP->GetMainDataRef().speedplan_info;
    behavior_param_.exception_analyse_result = DP->GetMainDataRef().task_fsm_info.exception_analyse_result_;
  
    behavior_param_.last_result_path = DP->GetMainDataRef().result_path;
    behavior_param_.paths = DP->GetMainDataRef().paths;
  }

  void PushData() {
    auto DP = DataPool::Instance();
    DP->GetMainDataRef().result_path = behavior_param_.result_path;
    DP->GetMainDataRef().behavior_fsm_info = behavior_param_.behavior_fsm_info;
    DP->GetMainDataRef().debug_planning_msg = behavior_param_.debug_planning_msg;
    DP->GetMainDataRef().cognition_info.unstruct_env_info.semantic_info = behavior_param_.semantic_info;
    DP->GetMainDataRef().speedplan_info = behavior_param_.speedplan_info;
    if (behavior_param_.calculate_path.is_new) {
      DP->GetMainDataRef().calculate_path = behavior_param_.calculate_path;
      behavior_param_.calculate_path.is_new = false;
    }
    DP->GetMainDataRef().task_fsm_info.exception_analyse_result_ = behavior_param_.exception_analyse_result;
  }
  void Process() {
    ScanBehaviorEvent();
  }
  void ScanBehaviorEvent() {
    AINFO << "ENTRY  " << behavior_param_.behavior_fsm_info.behavior_status.status_str;
    switch (behavior_param_.behavior_fsm_info.behavior_status.status) {
    case eBehaviorStatus::IDLE: ScanVectorEvents(idle_scan_events); break;
    case eBehaviorStatus::FORWARDFOLLOWING: ScanVectorEvents(forward_following_scan_events); break;
    case eBehaviorStatus::SLOW_DOWN: ScanVectorEvents(slow_down_scan_events); break;
    case eBehaviorStatus::BACKFOLLOWING: ScanVectorEvents(back_following_scan_events); break;
    case eBehaviorStatus::FORWARD_AVOID: ScanVectorEvents(forward_avoid_scan_events); break;
    case eBehaviorStatus::FORCE_AVOID: ScanVectorEvents(force_avoid_scan_events); break;
    case eBehaviorStatus::BACK_AVOID: ScanVectorEvents(back_avoid_scan_events); break;
    case eBehaviorStatus::FORWARD_STAY_STILL: ScanVectorEvents(forward_stay_still_scan_events); break;
    case eBehaviorStatus::BACK_STAY_STILL: ScanVectorEvents(back_stay_still_scan_events); break;
    case eBehaviorStatus::NARROW_TURN: ScanVectorEvents(narrow_turn_scan_events); break;
    case eBehaviorStatus::YIELD: ScanVectorEvents(yield_scan_events); break;

    case eBehaviorStatus::BEHAVIOR_ERROR: ScanVectorEvents(error_scan_events); break;
    default: ScanVectorEvents(idle_scan_events); break;
    }
  }

 private:
  bool PathNoCollisionEvent() {
    AINFO << "[Behavior Scan]: PathNoCollision";
    behavior_param_.debug_planning_msg.SetPathNoCollisionEvent(
      behavior_param_.semantic_info.path_frontcollision_times.ToString());
    if (behavior_param_.semantic_info.IsPathNoCollisionTimeUp() &&
        !behavior_param_.drive_status.control_emergency &&
        !behavior_param_.drive_status.aeb_emergency) {
      AWARN << "Set PathNoCollision";
      behavior_param_.debug_planning_msg.SetPathNoCollisionEvent("SETTING");
      pbehavior_fsm_->PathNoCollision((void*)&behavior_param_);
      behavior_param_.semantic_info.ResetFrontCollision();
      return true;
    }
    return false;
  }

  bool CurvatureFaultEvent() {
    AINFO << "[Behavior Scan]: CurvatureFault";
    behavior_param_.debug_planning_msg.SetCurvatureFaultEvent();
    double check_dis = 5.0;
    if (behavior_param_.last_result_path.path.empty()) return false;
    for (auto it = behavior_param_.last_result_path.path.begin();
         it != behavior_param_.last_result_path.path.end(); it = std::next(it)) {
      if (it->length - behavior_param_.last_result_path.path.front().length > check_dis) break;
      if (it->curvature > behavior_param_.behavior_config.curvature_threshold) {
        AWARN << "Set CurvatureFault";
        behavior_param_.debug_planning_msg.SetCurvatureFaultEvent("SETTING: " + std::to_string(it->curvature));
        pbehavior_fsm_->CurvatureFault((void*)&behavior_param_);
        return true;
      }
    }
    return false;
  }

  bool DeviationPathEvent() {
    AINFO << "[Behavior Scan]: DeviationPath";
    behavior_param_.debug_planning_msg.SetDeviationPathEvent();
    if (behavior_param_.last_result_path.path.empty()) {
      behavior_param_.debug_planning_msg.SetDeviationPathEvent("PATH EMPTY");
      return false;
    }
    double dis = std::hypot(behavior_param_.localization_data.xg - behavior_param_.last_result_path.path.front().xg,
                            behavior_param_.localization_data.yg - behavior_param_.last_result_path.path.front().yg);
    double angle = std::fabs(behavior_param_.localization_data.yaw - behavior_param_.last_result_path.path.front().globalangle);
    AINFO << "cur_loc:" << behavior_param_.localization_data.xg << ","
          << behavior_param_.localization_data.yg << ","
          << behavior_param_.localization_data.yaw;
    AINFO << "first:" << behavior_param_.last_result_path.path.front().xg << ","
          << behavior_param_.last_result_path.path.front().yg << ","
          << behavior_param_.last_result_path.path.front().globalangle;
    if (angle > 180.0) angle = 360.0 - angle;
    if (dis > behavior_param_.behavior_config.deviation_dis_err ||
        angle > behavior_param_.behavior_config.deviation_angle_err) {
      AWARN << "Set DeviationPath";
    behavior_param_.debug_planning_msg.SetDeviationPathEvent("SETTING: " + std::to_string(dis) + "," + std::to_string(angle));
      pbehavior_fsm_->DeviationPath((void*)&behavior_param_);
      return true;
    }
    return false;
  }

  bool PathCollisionEvent() {
    AINFO << "[Behavior Scan]: PathCollision";
    behavior_param_.debug_planning_msg.SetPathCollisionEvent(
      behavior_param_.semantic_info.path_resultcollision_times.ToString());
    if (behavior_param_.semantic_info.IsPathCollisionTimeUp()) {
      AWARN << "Set PathCollision";
      behavior_param_.debug_planning_msg.SetPathCollisionEvent("SETTING");
      pbehavior_fsm_->PathCollision((void*)&behavior_param_);
      return true;
    }
    return false;
  }

  bool AebEmergencyEvent() {
    AINFO << "[Behavior Scan]: AebEmergency";
    behavior_param_.debug_planning_msg.SetAebEmergencyEvent(
      behavior_param_.semantic_info.path_emergency_times.ToString());
    if (behavior_param_.semantic_info.IsEmergencyTimeUp()) {
      AWARN << "Set AebEmergency";
      behavior_param_.debug_planning_msg.SetAebEmergencyEvent("SETTING");
      pbehavior_fsm_->AebEmergency((void*)&behavior_param_);
      return true;
    }
    return false;
  }

  bool SlowDownFinishEvent() {
    AINFO << "[Behavior Scan]: SlowDownFinish";
    behavior_param_.debug_planning_msg.SetSlowDownFinishEvent();
    if (std::fabs(behavior_param_.drive_status.velocity) < 0.1 &&
        behavior_param_.speedplan_info.stop_dis_fs < behavior_param_.behavior_config.slow_dis_threshold) {
      AWARN << "Set SlowDownFinish";
      behavior_param_.debug_planning_msg.SetSlowDownFinishEvent("SETTING");
      pbehavior_fsm_->SlowDownFinish((void*)&behavior_param_);
      return true;
    }
    return false;
  }

  bool NarrowTurnFinishEvent() {
    // AINFO << "[Behavior Scan]: NarrowTurnFinish";
    // behavior_param_.debug_planning_msg.SetNarrowTurnFinishEvent();
    // if (true) {
    //   AWARN << "Set NarrowTurnFinish";
    //   pbehavior_fsm_->NarrowTurnFinish((void*)&behavior_param_);
    //   return true;
    // }
    return false;
  }

  bool NarrowTurnFaultEvent() {
    // AINFO << "[Behavior Scan]: NarrowTurnFault";
    // behavior_param_.debug_planning_msg.SetNarrowTurnFaultEvent();
    // if (true) {
    //   AWARN << "Set NarrowTurnFault";
    //   pbehavior_fsm_->NarrowTurnFault((void*)&behavior_param_);
    //   return true;
    // }
    return false;
  }

  bool ForceAvoidFinishEvent() {
    AINFO << "[Behavior Scan]: ForceAvoidFinish";
    if (behavior_param_.behavior_fsm_info.IsForceAvoidFinish()) {
      AWARN << "Set ForceAvoidFinish";
      behavior_param_.debug_planning_msg.SetForceAvoidFinishEvent("SETTING");
      pbehavior_fsm_->ForceAvoidFinish((void*)&behavior_param_);
      return true;
    }
    return false;
  }

  bool ForceAvoidFaultEvent() {
    AINFO << "[Behavior Scan]: ForceAvoidFault";
    if (behavior_param_.behavior_fsm_info.IsForceAvoidFault()) {
      AWARN << "Set ForceAvoidFault";
      behavior_param_.debug_planning_msg.SetForceAvoidFaultEvent("SETTING");
      pbehavior_fsm_->ForceAvoidFault((void*)&behavior_param_);
      return true;
    }
    return false;
  }

  bool DWAResultSuccessEvent() {
    AINFO << "[Behavior Scan]: DWAResultSuccess";
    if (behavior_param_.behavior_fsm_info.IsDWAResultSuccess()) {
      AWARN << "Set DWAResultSuccess";
      behavior_param_.behavior_fsm_info.ResetDWA();
      behavior_param_.debug_planning_msg.SetDWAResultSuccessEvent("SETTING");
      pbehavior_fsm_->DWAResultSuccess((void*)&behavior_param_);
      return true;
    }
    return false;
  }

  bool HybridAstarSuccessEvent() {
    AINFO << "[Behavior Scan]: HybridAstarSuccess";
    if (behavior_param_.behavior_fsm_info.IsHybridAstarSuccess()) {
      AWARN << "Set HybridAstarSuccess";
      behavior_param_.behavior_fsm_info.ResetHybridAstar();
      behavior_param_.debug_planning_msg.SetHybridAstarSuccessEvent("SETTING");
      pbehavior_fsm_->HybridAstarSuccess((void*)&behavior_param_);
      return true;
    }
    return false;
  }

  bool ForwardStayTimeOutEvent() {
    AINFO << "[Behavior Scan]: ForwardStayTimeOut";
    behavior_param_.debug_planning_msg.SetForwardStayTimeOutEvent(behavior_param_.behavior_fsm_info.forward_stay_still_times.ToString());
    if (behavior_param_.behavior_fsm_info.IsForwardStayTimeOut()) {
      AWARN << "Set ForwardStayTimeOut";
      behavior_param_.debug_planning_msg.SetForwardStayTimeOutEvent("SETTING");
      pbehavior_fsm_->ForwardStayTimeOut((void*)&behavior_param_);
      return true;
    }
    return false;
  }

  bool BackStayTimeOutEvent() {
    AINFO << "[Behavior Scan]: BackStayTimeOut";
    behavior_param_.debug_planning_msg.SetBackStayTimeOutEvent(behavior_param_.behavior_fsm_info.back_stay_still_times.ToString());
    if (behavior_param_.behavior_fsm_info.IsBackStayTimeOut()) {
      AWARN << "Set BackStayTimeOut";
      behavior_param_.debug_planning_msg.SetBackStayTimeOutEvent("SETTING");
      pbehavior_fsm_->BackStayTimeOut((void*)&behavior_param_);
      return true;
    }
    return false;
  }

  bool BackFollowingFinishEvent() {
    AINFO << "[Behavior Scan]: BackFollowingFinish";
    behavior_param_.debug_planning_msg.SetBackFollowingFinishEvent();
    if (behavior_param_.semantic_info.front_info.cell_info.height <
        behavior_param_.behavior_config.detect_height_threshold) {
      AWARN << "Set BackFollowingFinish, too low";
      behavior_param_.debug_planning_msg.SetBackFollowingFinishEvent("SETTING, too low");
      pbehavior_fsm_->BackFollowingFinish((void*)&behavior_param_);
      return true;
    }
    if (behavior_param_.behavior_fsm_info.IsBackFollowingFinish() ||
        behavior_param_.behavior_fsm_info.back_distance > behavior_param_.behavior_config.back_follow_dis_threshold) {
      AWARN << "Set BackFollowingFinish";
      behavior_param_.debug_planning_msg.SetBackFollowingFinishEvent("SETTING");
      pbehavior_fsm_->BackFollowingFinish((void*)&behavior_param_);
      return true;
    }
    return false;
  }

  bool ForwardAvoidFinishEvent() {
    AINFO << "[Behavior Scan]: ForwardAvoidFinish";
    if (behavior_param_.behavior_fsm_info.IsForwardAvoidFinish()) {
      AWARN << "Set ForwardAvoidFinish";
      behavior_param_.debug_planning_msg.SetForwardAvoidFinishEvent("SETTING");
      pbehavior_fsm_->ForwardAvoidFinish((void*)&behavior_param_);
      return true;
    }
    return false;
  }

  bool BackAvoidFinishEvent() {
    AINFO << "[Behavior Scan]: BackAvoidFinish";
    if (behavior_param_.behavior_fsm_info.IsBackAvoidFinish()) {
      AWARN << "Set BackAvoidFinish";
      behavior_param_.debug_planning_msg.SetBackAvoidFinishEvent("SETTING");
      pbehavior_fsm_->BackAvoidFinish((void*)&behavior_param_);
      return true;
    }
    return false;
  }

  bool YieldFinishEvent() {
    // AINFO << "[Behavior Scan]: YieldFinish";
    // behavior_param_.debug_planning_msg.SetYieldFinishEvent();
    // if (true) {
    //   AWARN << "Set YieldFinish";
    //   pbehavior_fsm_->YieldFinish((void*)&behavior_param_);
    //   return true;
    // }
    return false;
  }

  bool BehaviorErrorFinishEvent() {
    // AINFO << "[Behavior Scan]: BehaviorErrorFinish";
    // behavior_param_.debug_planning_msg.SetBehaviorErrorFinishEvent();
    // if (true) {
    //   AWARN << "Set BehaviorErrorFinish";
    //   pbehavior_fsm_->BehaviorErrorFinish((void*)&behavior_param_);
    //   return true;
    // }
    return false;
  }

  bool BehaviorResetEvent() {
    pbehavior_fsm_->BehaviorReset((void*)&behavior_param_);
    return true;
    // AINFO << "[Behavior Scan]: BehaviorReset";
    // behavior_param_.debug_planning_msg.SetBehaviorResetEvent();
    // if (true) {
    //   AWARN << "Set BehaviorReset";
    //   pbehavior_fsm_->BehaviorReset((void*)&behavior_param_);
    //   return true;
    // }
    // return false;
  }

  bool NoThingToDoEvent() {
    AWARN << "Set NoThingToDo";
    behavior_param_.debug_planning_msg.SetNoThingToDoEvent("SETTING");
    pbehavior_fsm_->NoThingToDo((void*)&behavior_param_);
    return true;
    // AINFO << "[Behavior Scan]: NoThingToDo";
    // behavior_param_.debug_planning_msg.SetNoThingToDoEvent();
    // if (true) {
    //   AWARN << "Set NoThingToDo";
    //   behavior_param_.debug_planning_msg.SetNoThingToDoEvent("SETTING");
    //   pbehavior_fsm_->NoThingToDo((void*)&behavior_param_);
    //   return true;
    // }
    // return false;
  }

  void ScanVectorEvents(std::vector<pBehaviorEventFunction> &scan_events) {
    for (int i = 0; i < scan_events.size(); i++) {
      if (scan_events[i]()) return;
    }
  }

 private:
  BehaviorFSMParam behavior_param_;
  std::shared_ptr<NormalBehaviorFSM> pbehavior_fsm_;

  std::vector<pBehaviorEventFunction> idle_scan_events = {
    // std::bind(&NormalBehaviorContext::PathNoCollisionEvent, this),
    // std::bind(&NormalBehaviorContext::PathCollisionEvent, this),
    std::bind(&NormalBehaviorContext::NoThingToDoEvent, this)
  };
  std::vector<pBehaviorEventFunction> forward_following_scan_events = {
    // std::bind(&NormalBehaviorContext::CurvatureFaultEvent, this),
    std::bind(&NormalBehaviorContext::DeviationPathEvent, this),
    std::bind(&NormalBehaviorContext::PathCollisionEvent, this),
    std::bind(&NormalBehaviorContext::AebEmergencyEvent, this),
    std::bind(&NormalBehaviorContext::NoThingToDoEvent, this)
  };

  std::vector<pBehaviorEventFunction> slow_down_scan_events = {
    std::bind(&NormalBehaviorContext::AebEmergencyEvent, this),
    std::bind(&NormalBehaviorContext::PathNoCollisionEvent, this),
    std::bind(&NormalBehaviorContext::SlowDownFinishEvent, this),
    std::bind(&NormalBehaviorContext::DWAResultSuccessEvent, this),
    std::bind(&NormalBehaviorContext::DeviationPathEvent, this),
    std::bind(&NormalBehaviorContext::NoThingToDoEvent, this)
  };
  std::vector<pBehaviorEventFunction> back_following_scan_events = {
    std::bind(&NormalBehaviorContext::PathNoCollisionEvent, this),
    std::bind(&NormalBehaviorContext::DeviationPathEvent, this),
    std::bind(&NormalBehaviorContext::DWAResultSuccessEvent, this),
    std::bind(&NormalBehaviorContext::BackFollowingFinishEvent, this),
    std::bind(&NormalBehaviorContext::NoThingToDoEvent, this)
  };

  std::vector<pBehaviorEventFunction> forward_avoid_scan_events = {
    std::bind(&NormalBehaviorContext::DeviationPathEvent, this),
    std::bind(&NormalBehaviorContext::ForwardAvoidFinishEvent, this),
    std::bind(&NormalBehaviorContext::PathCollisionEvent, this),
    std::bind(&NormalBehaviorContext::NoThingToDoEvent, this)
  };

  std::vector<pBehaviorEventFunction> back_avoid_scan_events = {
    std::bind(&NormalBehaviorContext::DeviationPathEvent, this),
    std::bind(&NormalBehaviorContext::PathCollisionEvent, this),
    std::bind(&NormalBehaviorContext::BackAvoidFinishEvent, this),
    std::bind(&NormalBehaviorContext::NoThingToDoEvent, this)
  };


  std::vector<pBehaviorEventFunction> force_avoid_scan_events = {
    std::bind(&NormalBehaviorContext::ForceAvoidFinishEvent, this),
    std::bind(&NormalBehaviorContext::ForceAvoidFaultEvent, this),
    std::bind(&NormalBehaviorContext::PathCollisionEvent, this),
    std::bind(&NormalBehaviorContext::NoThingToDoEvent, this)
  };


  std::vector<pBehaviorEventFunction> narrow_turn_scan_events = {
    std::bind(&NormalBehaviorContext::NarrowTurnFinishEvent, this),
    std::bind(&NormalBehaviorContext::NarrowTurnFaultEvent, this),
    std::bind(&NormalBehaviorContext::PathCollisionEvent, this),
    std::bind(&NormalBehaviorContext::NoThingToDoEvent, this)
  };

  std::vector<pBehaviorEventFunction> forward_stay_still_scan_events = {
    std::bind(&NormalBehaviorContext::PathNoCollisionEvent, this),
    std::bind(&NormalBehaviorContext::DeviationPathEvent, this),
    std::bind(&NormalBehaviorContext::DWAResultSuccessEvent, this),
    std::bind(&NormalBehaviorContext::ForwardStayTimeOutEvent, this),
    std::bind(&NormalBehaviorContext::NoThingToDoEvent, this)
  };

  std::vector<pBehaviorEventFunction> back_stay_still_scan_events = {
    std::bind(&NormalBehaviorContext::DeviationPathEvent, this),
    std::bind(&NormalBehaviorContext::PathNoCollisionEvent, this),
    std::bind(&NormalBehaviorContext::DWAResultSuccessEvent, this),
    std::bind(&NormalBehaviorContext::HybridAstarSuccessEvent, this),
    std::bind(&NormalBehaviorContext::BackStayTimeOutEvent, this),
    std::bind(&NormalBehaviorContext::NoThingToDoEvent, this)
  };

  std::vector<pBehaviorEventFunction> yield_scan_events = {
    std::bind(&NormalBehaviorContext::NoThingToDoEvent, this)
  };

  std::vector<pBehaviorEventFunction> error_scan_events = {
    std::bind(&NormalBehaviorContext::NoThingToDoEvent, this)
  };

};


} // namespace planning
} // namespace acu


#endif // SRC_EXECUTION_BEHAVIORPLAN_NORMAL_BEHAVIOR_CONTEXT_H_