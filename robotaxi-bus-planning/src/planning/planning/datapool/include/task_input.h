#ifndef DATAPOOL_INCLUDE_TASK_INPUT_H_
#define DATAPOOL_INCLUDE_TASK_INPUT_H_

#include <iostream>
#include "public_typedef.h"
#include "common/base/time/include/node_time.h"
namespace acu {
namespace planning {

enum class eMissionType {
  STOP = 0,
  FREE_DRIVE = 1,
  PATH_FOLLOW = 2,
  PATH_TRACTION = 3,
  PARK = 4
};

enum class eCommand {
  COMMAND_STOP = 0,
  COMMAND_START = 1,
  COMMAND_SUSPEND = 2,
  COMMAND_RESUME = 3
};

enum class eTaskInputStatus {
  DEFAULT = 0,
  NEW_TASK = 1,
  ABORTED_TASK = 2,
  PARALLED_TASK = 3,
  SUSPENDED_TASK = 4,
  RESUMED_TASK
};

enum class eEmergencyStatus {
  DEFAULT = 0,
  EMERGENCY_PRESSED,
  EMERGENCY_UNPRESSED
};

enum class eRemoteCtrlStatus {
  DEFAULT = 0,
  REMOTE_CTRL_PRESSED,
  REMOTE_CTRL_UNPRESSED
};

enum class eAutoDriveStatus {
  DEFAULT = 0,
  AUTO_DRIVE_ENTRY,
  AUTO_DRIVE_EXIT
};

struct VehicleDriveStatus {
  /**origin input**/
  double timestamp;
  double targetsangle;
  double targetaccel;
  double targetshiftposition;
  double steer_angle; // 车辆实际转角
  double realstrtorque;
  double shiftlvlposition; // 车辆实际档位
  double velocity; // 车辆车速（控制从底层接收），可能没有数据
  int fault_level; // 故障等级（从业务端获取）
  int epb_status; // 车辆实际epb状态
  int control_switch_state;
  double vehicle_accel; // 车辆实际加速度 
  double brake_pressure;
  int system_state; // 车辆系统状态（5:自动驾驶，7：拍停，4：遥控）
  int control_mode;
  bool control_emergency; // 障碍物急停，未赋值，需要和夏老师确认
  bool aeb_emergency; // 不用
  int brake_state;//驾驶员制动踏板踩下状态，1为踩下。@pqg
  int on_accpedal;//驾驶员油门踏板踩下状态，1为踩下。@pqg
  bool steer_left_down;
  /**origin input**/
  FaultStatus canbus_status;
  eEmergencyStatus emergency_status;
  eRemoteCtrlStatus remote_ctrl_status;
  eAutoDriveStatus auto_drive_status;
  
  VehicleDriveStatus() {
    Reset();
  }
  void Reset() {
    timestamp = 0.0;
    targetsangle = 0.0;
    targetaccel = 0.0;
    targetshiftposition = 0.0;
    steer_angle = 0.0;
    realstrtorque = 0.0;
    shiftlvlposition = 0.0;
    velocity = 0.0;
    fault_level = 0;
    epb_status = 0;
    control_switch_state = 0;
    vehicle_accel = 0.0;
    brake_pressure = 0.0;
    system_state = 0;
    control_mode = 0;
    control_emergency = false;
    aeb_emergency = false;
    brake_state = 0;
    on_accpedal = 0;
    steer_left_down = false;
    emergency_status = eEmergencyStatus::DEFAULT;
    remote_ctrl_status = eRemoteCtrlStatus::DEFAULT;
    auto_drive_status = eAutoDriveStatus::DEFAULT;
  }

  bool IsSyetemException() {
    return (fault_level == 2);
  }
  bool IsSyetemNormal() {
    return (fault_level == 0);
  }
  bool IsBtnEmergencyPressed() {
    return (emergency_status == eEmergencyStatus::EMERGENCY_PRESSED);
  }
  bool IsBtnEmergencyUnpressed() {
    return (emergency_status == eEmergencyStatus::EMERGENCY_UNPRESSED);
  }
  bool IsRemoteCtrlConnected() {
    return (remote_ctrl_status == eRemoteCtrlStatus::REMOTE_CTRL_PRESSED);
  }
  bool IsRemoteCtrlUnconnected() {
    return (remote_ctrl_status == eRemoteCtrlStatus::REMOTE_CTRL_UNPRESSED);
  }
  void SetDriveStatus() {
    emergency_status = eEmergencyStatus::DEFAULT;
    remote_ctrl_status = eRemoteCtrlStatus::DEFAULT;
    auto_drive_status = eAutoDriveStatus::DEFAULT;
    if (system_state == 7) {
      emergency_status = eEmergencyStatus::EMERGENCY_PRESSED;
      remote_ctrl_status = eRemoteCtrlStatus::REMOTE_CTRL_UNPRESSED;
      auto_drive_status = eAutoDriveStatus::AUTO_DRIVE_EXIT;
    } else if (system_state == 4) {
      emergency_status = eEmergencyStatus::EMERGENCY_UNPRESSED;
      remote_ctrl_status = eRemoteCtrlStatus::REMOTE_CTRL_PRESSED;
      auto_drive_status = eAutoDriveStatus::AUTO_DRIVE_EXIT;
    } else if (system_state == 5) {
      emergency_status = eEmergencyStatus::EMERGENCY_UNPRESSED;
      remote_ctrl_status = eRemoteCtrlStatus::REMOTE_CTRL_UNPRESSED;
      auto_drive_status = eAutoDriveStatus::AUTO_DRIVE_ENTRY;
    } else {
      emergency_status = eEmergencyStatus::EMERGENCY_UNPRESSED;
      remote_ctrl_status = eRemoteCtrlStatus::REMOTE_CTRL_UNPRESSED;
      auto_drive_status = eAutoDriveStatus::AUTO_DRIVE_EXIT;
    }
  }
};

struct PathConstraint {
  std::string  id;
  double speed_limit;
  std::vector<int> behaviorlimit;
  std::vector<int> behavioractivethreshold;
  std::vector<int> behaviorexecutethreshold;
  
  PathConstraint() {
    Reset();
  }
  void Reset() {
    id = "";
    speed_limit = 0.0;
    behaviorlimit.clear();
    behavioractivethreshold.clear();
    behaviorexecutethreshold.clear();
  }
};

typedef struct TaskContent {
  /***origin input***/
  double time;
  int valid_count;
  bool is_valid;
  bool new_input;  /*!< judge the LocPerception is the new data from callback, default is false*/
  int mission_id;
  eMissionType mission_type;
  int mission_type_nav;
  eCommand command;
  int brake_command;  // 0 default  1 brake slightly  2 brake hard
  int direction_command;  // 0 default  1 left  2 right
  double speed_limit; // m/s
  std::vector<PathConstraint> path_constraints;
  /***origin input***/

  eCommand command_info;
  eCommand command_info_bak;
  eTaskInputStatus task_state;  /*!< for judge new task or abandon task   0 default  1 new_task  2 abandon task 3 Parallel driver*/
  eSystemStatus system_state;
  double new_task_time;

  TaskContent() {
    time = 0;
    valid_count = 0;
    is_valid = false;
    new_input = false;
    Reset();
  }
  void Reset() {
    mission_id = 0;
    mission_type = eMissionType::STOP;
    mission_type_nav=1;
    command = eCommand::COMMAND_STOP;
    path_constraints.clear();
    command_info = eCommand::COMMAND_STOP;
    command_info_bak = eCommand::COMMAND_STOP;
    task_state = eTaskInputStatus::DEFAULT;
    system_state = eSystemStatus::DEFAULT;
    new_task_time = acu::common::NodeTime::Now().ToSecond();
    brake_command = 0;
    direction_command = 0;
    speed_limit = 100.0;
  }

  void SetNewTask() {
    task_state = eTaskInputStatus::NEW_TASK;
    new_task_time = acu::common::NodeTime::Now().ToSecond();
  }
  void SetSuspendTask() {
    task_state = eTaskInputStatus::SUSPENDED_TASK;
  }
  void SetContinuedTask() {
    task_state = eTaskInputStatus::RESUMED_TASK;
  }
  void SetAbortedTask() {
    task_state = eTaskInputStatus::ABORTED_TASK;
  }
  void ResetNewTask() {
    task_state = eTaskInputStatus::DEFAULT;
    new_task_time = acu::common::NodeTime::Now().ToSecond();
  }
  bool IsNewTask() {
    return (task_state == eTaskInputStatus::NEW_TASK);
  }
  bool IsAbordTask() {
    return (task_state == eTaskInputStatus::ABORTED_TASK);
  }
  bool IsSuspendedTask() {
    return (task_state == eTaskInputStatus::SUSPENDED_TASK);
  }
  bool IsContinuedTask() {
    return (task_state == eTaskInputStatus::RESUMED_TASK);
  }
  void SetTaskStatus() {
    if (command == eCommand::COMMAND_START &&
        command_info != eCommand::COMMAND_START) {
      SetNewTask();
    }
    if (command == eCommand::COMMAND_SUSPEND) {
      SetSuspendTask();
    }
    if (command == eCommand::COMMAND_RESUME &&
        command_info == eCommand::COMMAND_SUSPEND) {
      SetContinuedTask();
    }
    if (command == eCommand::COMMAND_STOP &&
        command_info != eCommand::COMMAND_STOP) {
      SetAbortedTask();
    }
  }
  void SetCommandInfo() {
    command_info = command;
  }
} TaskContent;


} // namespace planning
} // namespace acu

#endif // DATAPOOL_INCLUDE_TASK_INPUT_H_
