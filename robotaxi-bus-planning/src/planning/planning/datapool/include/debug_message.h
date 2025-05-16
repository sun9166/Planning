#ifndef DATAPOOL_INCLUDE_DEBUG_MESSAGE_H_
#define DATAPOOL_INCLUDE_DEBUG_MESSAGE_H_

#include <iostream>
#include <string>

#include "planning_msgs.pb.h"

#include "planning_debug_msgs.pb.h"

namespace acu {
namespace planning {

typedef struct DebugStateMsg {
  std::string str;
  int value;
  void SetDebugStateMsg(std::string _str, int _value = 0) {
    str = str + _str + " ";
    value = _value;
  }
  void Reset() {
    str = "";
    value = 0;
  }
} DebugStateMsg;

typedef struct DebugSemanticInfo {
  int pursuit_index;
  int obs_id;
  int obs_type;
  double obs_x;
  double obs_y;
  double obs_z;
  double obs_height;
  double path_x;
  double path_y;
  double path_angle;
  double path_index;
  double path_length;
  std::string collid_count;
  void Reset() {
    pursuit_index = -1;
    obs_id = -1;
    obs_type = -1;
    obs_x = 0.0;
    obs_y = 0.0;
    obs_z = 0.0;
    obs_height = 2.0;
    path_x = 0.0;
    path_y = 0.0;
    path_angle = 0.0;
    path_index = 999;
    path_length = 999.0;
    collid_count = " ";
  }
} DebugSemanticInfo;

typedef struct DebugUnsCollidMsg {
  DebugSemanticInfo front_collision_info; 
  DebugSemanticInfo result_collision_info;
  double slow_down_dis;
} DebugUnsCollidMsg;

typedef struct DebugRefpathMsg
{
  int local_path_index;
  int cut_path_index;
  bool is_avoid;
  bool cur_reverse;
  int cur_nearest_index;
  int cur_nearest_origin_index;
  DebugRefpathMsg() {
    Reset();
  }
  void Reset() {
    local_path_index = 0;
    cut_path_index = 0;
    is_avoid = false;
    cur_reverse = false;
    cur_nearest_index = 0;
    cur_nearest_origin_index = 0;
  }
} DebugRefpathMsg;

typedef struct DebugDWAMsg
{
  std::string DWA_info;
  std::string lat_goals_info;
  std::string lon_goals_info;
  float search_path_size;
  float link_path_size;
  float search_model_halfwheel_left;
  float search_model_halfwheel_right;
  bool if_force_avoid;
  bool if_dwa_success;
  bool if_lateral_goal_reached;
  bool if_longitudinal_goal_reached;
  bool if_link_success;
  int lateral_goals_num;
  int longitudinal_goals_num;
  int refpath_size;
  int dwapath_size;
  int linkpath_size;
  int allpath_size;

  void Reset()
  {
    DWA_info = "No news...";
    lat_goals_info = "Not assigned...";
    lon_goals_info = "Not assigned...";
    search_path_size = -1.0;
    link_path_size = -1.0;
    search_model_halfwheel_left = -1.0;
    search_model_halfwheel_right = -1.0;
    if_force_avoid = false;
    if_dwa_success = false;
    if_lateral_goal_reached = false;
    if_longitudinal_goal_reached = false;
    if_link_success = false;
    lateral_goals_num = -1;
    longitudinal_goals_num = -1;
    refpath_size = -1;
    dwapath_size = -1;
    linkpath_size = -1;
    allpath_size = -1;
  }

} DebugDWAMsg;

typedef struct DebugASTARMsg {
  std::string goalinfo;
  std::string startinfo;
  std::string type;
  std::string MTR_info;
  std::string allowance_info;
  std::string stayinfo;
  bool goalvalid;
  bool startvalid;
  std::vector<int> loop;
  int RS_size;
  int SEHS_size;
  int allpath_size;

  void Reset()
  {
    goalinfo = "No news...";
    startinfo = "No news...";
    type = "No news...";
    MTR_info = "No news...";
    allowance_info = "No news...";
    stayinfo = "";
    goalvalid = false;
    startvalid = false;
    loop.clear();
    RS_size = -1;
    SEHS_size = -1;
    allpath_size = -1;
  }

} DebugASTARMsg;

typedef struct DebugPlanningMsg {
  DebugStateMsg main_stream_msg;
  DebugStateMsg task_fsm_condition;
  DebugStateMsg task_fsm_state;
  DebugStateMsg cognition_msg;
  DebugStateMsg business_msg;
  DebugStateMsg pathplan_msg;
  DebugStateMsg behavior_fsm_condition;
  DebugStateMsg behavior_fsm_state;
  DebugStateMsg thread_state;

  DebugUnsCollidMsg uns_collid_msg;
  DebugRefpathMsg refpath_msg;
  DebugDWAMsg DWA_message;
  DebugASTARMsg ASTAR_message;

  common_msgs::Header  header;
  planning_debug_msgs::DebugBusiness business_debug;
  planning_debug_msgs::DebugMotionPlan motionplan_debug;
  planning_msgs::Trajectory target_reference_line;
  planning_debug_msgs::DebugCognition cognition;
  planning_debug_msgs::DebugDecision decision;
  planning_debug_msgs::Debugstmap stmap;
  planning_debug_msgs::DebugObjectByLine object_debug_by_line;
  std::vector<planning_debug_msgs::DebugSLBoundary> sl_boundaries;
  planning_debug_msgs::motion_obstacle_vec motion_obstacle_vec_msg;
  std::vector<planning_debug_msgs::XTBounds> xt_bounds;
  planning_debug_msgs::DebugExtraDecision extra_decision;
  double time_cost_ms;

  DebugPlanningMsg() {
    Reset();
  }
  void Reset() {
    main_stream_msg.Reset();
    task_fsm_condition.Reset();
    task_fsm_state.Reset();
    cognition_msg.Reset();
    business_msg.Reset();
    pathplan_msg.Reset();
    behavior_fsm_condition.Reset();
    behavior_fsm_state.Reset();
    thread_state.Reset();
    business_debug.Clear();
    motionplan_debug.Clear();
    target_reference_line.Clear();
    cognition.Clear();
    decision.Clear();
    stmap.Clear();
    object_debug_by_line.Clear();
    sl_boundaries.clear();
    motion_obstacle_vec_msg.Clear();
    xt_bounds.clear();
    extra_decision.Clear();
  }

  // task fsm condition
  void SetTaskIsNewEvent(const std::string info = "") {
    task_fsm_condition.str = "SetTaskIsNewEvent " + info + "\n";
    task_fsm_condition.value = 1;
  }
  void SetTaskExecutableEvent(const std::string info = "") {
    task_fsm_condition.str = "SetTaskExecutableEvent " + info + "\n";
    task_fsm_condition.value = 1;
  }
  void SetTaskUnexecutableEvent(const std::string info = "") {
    task_fsm_condition.str = "SetTaskUnexecutableEvent " + info + "\n";
    task_fsm_condition.value = 2;
  }
  void SetTaskExecuteOverEvent(const std::string info = "") {
    task_fsm_condition.str = "SetTaskExecuteOverEvent " + info + "\n";
    task_fsm_condition.value = 3;
  }
  void SetTaskExecuteFaultEvent(const std::string info = "") {
    task_fsm_condition.str = "SetTaskExecuteFaultEvent " + info + "\n";
    task_fsm_condition.value = 4;
  }
  void SetTaskOverEvent(const std::string info = "") {
    task_fsm_condition.str = "SetTaskOverEvent " + info + "\n";
    task_fsm_condition.value = 5;
  }
  void SetTaskAbortedEvent(const std::string info = "") {
    task_fsm_condition.str = "SetTaskAbortedEvent " + info + "\n";
    task_fsm_condition.value = 6;
  }
  void SetTaskSuspendedEvent(const std::string info = "") {
    task_fsm_condition.str = "SetTaskSuspendedEvent " + info + "\n";
    task_fsm_condition.value = 7;
  }
  void SetTaskContinuedEvent(const std::string info = "") {
    task_fsm_condition.str = "SetTaskContinuedEvent " + info + "\n";
    task_fsm_condition.value = 8;
  }
  void SetSyetemExceptionEvent(const std::string info = "") {
    task_fsm_condition.str = "SetSyetemExceptionEvent " + info + "\n";
    task_fsm_condition.value = 9;
  }
  void SetSyetemNormalEvent(const std::string info = "") {
    task_fsm_condition.str = "SetSyetemNormalEvent " + info + "\n";
    task_fsm_condition.value = 10;
  }
  void SetBtnEmergencyPressedEvent(const std::string info = "") {
    task_fsm_condition.str = "SetBtnEmergencyPressedEvent " + info + "\n";
    task_fsm_condition.value = 11;
  }
  void SetBtnEmergencyUnpressedEvent(const std::string info = "") {
    task_fsm_condition.str = "SetBtnEmergencyUnpressedEvent " + info + "\n";
    task_fsm_condition.value = 12;
  }
  void SetRemoteCtrlConnectedEvent(const std::string info = "") {
    task_fsm_condition.str = "SetRemoteCtrlConnectedEvent " + info + "\n";
    task_fsm_condition.value = 13;
  }
  void SetRemoteCtrlUnconnectedEvent(const std::string info = "") {
    task_fsm_condition.str = "SetRemoteCtrlUnconnectedEvent " + info + "\n";
    task_fsm_condition.value = 14;
  }
  void SetEndTaskHandleOverEvent(const std::string info = "") {
    task_fsm_condition.str = "SetEndTaskHandleOverEvent " + info + "\n";
    task_fsm_condition.value = 15;
  }

  // task state func
  void SetIdle2AnalyseTaskFunc() {
    task_fsm_state.str = "Idle2AnalyseTaskFunc";
    task_fsm_state.value = 1;
  }
  void SetIdleTaskFunc() {
    task_fsm_state.str = "IdleTaskFunc";
    task_fsm_state.value = 2;
  }
  void SetAnalyseTaskFunc() {
    task_fsm_state.str = "AnalyseTaskFunc";
    task_fsm_state.value = 3;
  }
  void SetExecuteTaskFunc() {
    task_fsm_state.str = "ExecuteTaskFunc";
    task_fsm_state.value = 4;
  }
  void SetEndTaskFunc() {
    task_fsm_state.str = "EndTaskFunc";
    task_fsm_state.value = 5;
  }
  void SetExceptionalTaskFunc() {
    task_fsm_state.str = "ExceptionalTaskFunc";
    task_fsm_state.value = 6;
  }
  void SetSuspendTaskFunc() {
    task_fsm_state.str = "SuspendTaskFunc";
    task_fsm_state.value = 7;
  }
  void SetEmergencyBtnHandleFunc() {
    task_fsm_state.str = "EmergencyBtnHandleFunc";
    task_fsm_state.value = 8;
  }
  void SetRemoteControlFunc() {
    task_fsm_state.str = "RemoteControlFunc";
    task_fsm_state.value = 9;
  }
  void SetSuspend2AnalyseTaskFunc() {
    task_fsm_state.str = "Suspend2AnalyseTaskFunc";
    task_fsm_state.value = 10;
  }
  void SetRemote2AnalyseTaskFunc() {
    task_fsm_state.str = "Remote2AnalyseTaskFunc";
    task_fsm_state.value = 11;
  }

  // behavior condition
  void SetPathNoCollisionEvent(const std::string info = "") {
    behavior_fsm_condition.str = behavior_fsm_condition.str + "PathNoCollision " + info + "\n";
    behavior_fsm_condition.value = 1;
  }
  void SetCurvatureFaultEvent(const std::string info = "") {
    behavior_fsm_condition.str = behavior_fsm_condition.str + "CurvatureFault " + info + "\n";
    behavior_fsm_condition.value = 2;
  }
  void SetDeviationPathEvent(const std::string info = "") {
    behavior_fsm_condition.str = behavior_fsm_condition.str + "DeviationPath " + info + "\n";
    behavior_fsm_condition.value = 3;
  }
  void SetPathCollisionEvent(const std::string info = "") {
    behavior_fsm_condition.str = behavior_fsm_condition.str + "PathCollision " + info + "\n";
    behavior_fsm_condition.value = 4;
  }
  void SetAebEmergencyEvent(const std::string info = "") {
    behavior_fsm_condition.str = behavior_fsm_condition.str + "AebEmergency " + info + "\n";
    behavior_fsm_condition.value = 5;
  }
  void SetSlowDownFinishEvent(const std::string info = "") {
    behavior_fsm_condition.str = behavior_fsm_condition.str + "SlowDownFinish " + info + "\n";
    behavior_fsm_condition.value = 6;
  }
  void SetNarrowTurnFinishEvent(const std::string info = "") {
    behavior_fsm_condition.str = behavior_fsm_condition.str + "NarrowTurnFinish " + info + "\n";
    behavior_fsm_condition.value = 7;
  }
  void SetNarrowTurnFaultEvent(const std::string info = "") {
    behavior_fsm_condition.str = behavior_fsm_condition.str + "NarrowTurnFault " + info + "\n";
    behavior_fsm_condition.value = 8;
  }
  void SetForceAvoidFinishEvent(const std::string info = "") {
    behavior_fsm_condition.str = behavior_fsm_condition.str + "ForceAvoidFinish " + info + "\n";
    behavior_fsm_condition.value = 9;
  }
  void SetForceAvoidFaultEvent(const std::string info = "") {
    behavior_fsm_condition.str = behavior_fsm_condition.str + "ForceAvoidFault " + info + "\n";
    behavior_fsm_condition.value = 10;
  }
  void SetDWAResultSuccessEvent(const std::string info = "") {
    behavior_fsm_condition.str = behavior_fsm_condition.str + "DWAResultSuccess " + info + "\n";
    behavior_fsm_condition.value = 11;
  }
  void SetHybridAstarSuccessEvent(const std::string info = "") {
    behavior_fsm_condition.str = behavior_fsm_condition.str + "HybridAstarSuccess " + info + "\n";
    behavior_fsm_condition.value = 12;
  }
  void SetForwardStayTimeOutEvent(const std::string info = "") {
    behavior_fsm_condition.str = behavior_fsm_condition.str + "ForwardStayTimeOut " + info + "\n";
    behavior_fsm_condition.value = 13;
  }
  void SetBackStayTimeOutEvent(const std::string info = "") {
    behavior_fsm_condition.str = behavior_fsm_condition.str + "BackStayTimeOut " + info + "\n";
    behavior_fsm_condition.value = 14;
  }
  void SetBackFollowingFinishEvent(const std::string info = "") {
    behavior_fsm_condition.str = behavior_fsm_condition.str + "BackFollowingFinish " + info + "\n";
    behavior_fsm_condition.value = 15;
  }
  void SetForwardAvoidFinishEvent(const std::string info = "") {
    behavior_fsm_condition.str = behavior_fsm_condition.str + "ForwardAvoidFinish " + info + "\n";
    behavior_fsm_condition.value = 16;
  }
  void SetBackAvoidFinishEvent(const std::string info = "") {
    behavior_fsm_condition.str = behavior_fsm_condition.str + "BackAvoidFinish " + info + "\n";
    behavior_fsm_condition.value = 17;
  }
  void SetYieldFinishEvent(const std::string info = "") {
    behavior_fsm_condition.str = behavior_fsm_condition.str + "YieldFinish " + info + "\n";
    behavior_fsm_condition.value = 18;
  }
  void SetBehaviorErrorFinishEvent(const std::string info = "") {
    behavior_fsm_condition.str = behavior_fsm_condition.str + "BehaviorErrorFinish " + info + "\n";
    behavior_fsm_condition.value = 19;
  }
  void SetBehaviorResetEvent(const std::string info = "") {
    behavior_fsm_condition.str = behavior_fsm_condition.str + "BehaviorReset " + info + "\n";
    behavior_fsm_condition.value = 20;
  }
  void SetNoThingToDoEvent(const std::string info = "") {
    behavior_fsm_condition.str = behavior_fsm_condition.str + "NoThingToDo " + info + "\n";
    behavior_fsm_condition.value = 21;
  }

  // behavior state function
  void SetIdleFunc(const std::string &info = "") {
    behavior_fsm_state.str = "IdleFunc:" + info;
    behavior_fsm_state.value = 1;
  }
  void SetForwardFollowingFunc(const std::string &info = "") {
    behavior_fsm_state.str = "ForwardFollowingFunc:" + info;
    behavior_fsm_state.value = 2;
  }
  void SetNarrowTurnFunc(const std::string &info = "") {
    behavior_fsm_state.str = "NarrowTurnFunc:" + info;
    behavior_fsm_state.value = 3;
  }
  void SetForceAvoidFunc(const std::string &info = "") {
    behavior_fsm_state.str = "ForceAvoidFunc:" + info;
    behavior_fsm_state.value = 4;
  }
  void SetSlowDownFunc(const std::string &info = "") {
    behavior_fsm_state.str = "SlowDownFunc:" + info;
    behavior_fsm_state.value = 5;
  }
  void SetBackStayStillFunc(const std::string &info = "") {
    behavior_fsm_state.str = "BackStayStillFunc:" + info;
    behavior_fsm_state.value = 6;
  }
  void SetForwardStayStillFunc(const std::string &info = "") {
    behavior_fsm_state.str = "ForwardStayStillFunc:" + info;
    behavior_fsm_state.value = 7;
  }
  void SetForwardAvoidFunc(const std::string &info = "") {
    behavior_fsm_state.str = "ForwardAvoidFunc:" + info;
    behavior_fsm_state.value = 8;
  }
  void SetBackFollowingFunc(const std::string &info = "") {
    behavior_fsm_state.str = "BackFollowingFunc:" + info;
    behavior_fsm_state.value = 9;
  }
  void SetBackAvoidFunc(const std::string &info = "") {
    behavior_fsm_state.str = "BackAvoidFunc:" + info;
    behavior_fsm_state.value = 10;
  }
  void SetBehaviorErrorFunc(const std::string &info = "") {
    behavior_fsm_state.str = "BehaviorErrorFunc:" + info;
    behavior_fsm_state.value = 11;
  }

} DebugPlanningMsg;

} // namespace planning
} // namespace acu

#endif