#ifndef DATAPOOL_INCLUDE_TASK_TYPEDEF_H_
#define DATAPOOL_INCLUDE_TASK_TYPEDEF_H_

#include "public_typedef.h"
#include "task_input.h"
#include "locperception_input.h"
#include "debug_message.h"
#include "src/execution/business/include/business_base.h"
#include "src/execution/behaviorplan/behavior_base/behavior_context_base.h"
#include "src/algorithm/interface/interface_base/include/algorithm_base.h"

namespace acu {
namespace planning {


enum class eTaskExeResult {
  DEFAULT = 0,
  SUCCESS,
  FAILURE,
  FUNCTION_ERROR
};

/**
* @brief define the task status enum for task fsm
*/
enum class eTaskStatus {
  DEFAULT = 0,    /*!< default is zero */
  IDLE_TASK,
  ANALYSE_TASK,
  EXECUTE_TASK,
  SUSPEND_TASK,
  EXECEPTIONAL_TASK,
  EMEREGENCY_BTN_HANDLE,
  REMOTE_CONTROL,
  END_TASK
};

typedef struct TaskStatus {
  eTaskStatus status;
  std::string str_status;
  TaskStatus() {
    Reset();
  }
  void Reset() {
    status = eTaskStatus::DEFAULT;
    str_status = "DEFAULT";
  }
  void SetIdle2AnalyseTask() {
    status = eTaskStatus::ANALYSE_TASK;
    str_status = "Idle2AnalyseTask";
  }
  void SetIdleTask() {
    status = eTaskStatus::IDLE_TASK;
    str_status = "IDLE_TASK";
  }
  void SetAnalyseTask() {
    status = eTaskStatus::ANALYSE_TASK;
    str_status = "ANALYSE_TASK";
  }
  void SetExecuteTask() {
    status = eTaskStatus::EXECUTE_TASK;
    str_status = "EXECUTE_TASK";
  }
  void SetEndTask() {
    status = eTaskStatus::END_TASK;
    str_status = "END_TASK";
  }
  void SetExceptionalTask() {
    status = eTaskStatus::EXECEPTIONAL_TASK;
    str_status = "EXECEPTIONAL_TASK";
  }
  void SetSuspendTask() {
    status = eTaskStatus::SUSPEND_TASK;
    str_status = "SUSPEND_TASK";
  }
  void SetEmergencyBtnHandle() {
    status = eTaskStatus::EMEREGENCY_BTN_HANDLE;
    str_status = "EMEREGENCY_BTN_HANDLE";
  }
  void SetRemoteControl() {
    status = eTaskStatus::REMOTE_CONTROL;
    str_status = "REMOTE_CONTROL";
  }
  void SetSuspend2AnalyseTask() {
    status = eTaskStatus::ANALYSE_TASK;
    str_status = "Suspend2AnalyseTask";
  }
  void SetRemote2AnalyseTask() {
    status = eTaskStatus::ANALYSE_TASK;
    str_status = "Remote2AnalyseTask";
  }
} TaskStatus;

enum class eTaskAnalyse {
  DEFAULT = 0,
  EXECUTABLE,
  UNEXECUTABLE,
  END_TASK
};
typedef struct TaskAnalyseResult {
  eTaskAnalyse status;
  std::string str_status;
  TaskAnalyseResult() {
    status = eTaskAnalyse::DEFAULT;
    str_status = " ";
  }
} TaskAnalyseResult;

enum class eSubTaskStatus {
  DEFAULT = 0,    /*!< default is zero */
  OK,       /*!< sub task is exe ok */
  EXCPETION,  /*!< sub task has exception */
  REASON1,  /*!< sub task has REASON1 */
  REASON2,    /*!< sub task has REASON2 */
  REASON3,     /*!< sub task has REASON3 */
};
typedef struct ExeSubTaskResult {
  eSubTaskStatus status;
  ExeSubTaskResult() {
    status = eSubTaskStatus::DEFAULT;
  }
} ExeSubTaskResult;

enum class eEndTaskHandleRes {
  DEFAULT = 0,
  HANDING_OVER,
  HANDLING
};
typedef struct EndTaskHandleResult {
  eEndTaskHandleRes status;
  EndTaskHandleResult() {
    status = eEndTaskHandleRes::DEFAULT;
  }
} EndTaskHandleResult;


typedef struct TaskFSMInfo {
  TaskStatus task_status;
  bool is_final_adjust;
  TaskAnalyseResult task_analyse_result_;
  ExeSubTaskResult  sub_task_result_;
  ExceptionAnalyseResult exception_analyse_result_;
  EndTaskHandleResult endtask_handle_result_;
  TaskFSMInfo() {
    task_status.Reset();
    Reset();
  }
 
  // clear flags
  void ResetAnalyseStatus() {
    task_analyse_result_.status = eTaskAnalyse::DEFAULT;
  }
  void ResetExeSubTaskStatus() {
    sub_task_result_.status = eSubTaskStatus::DEFAULT;
  }
  void ResetExceptionAnalyseStatus() {
    exception_analyse_result_.status = eExceptionHandleRes::DEFAULT;
  }
  void ResetEndTaskHandleStatus() {
    endtask_handle_result_.status = eEndTaskHandleRes::DEFAULT;
  }
  // set method
  void setTaskOver() {
    task_analyse_result_.status = eTaskAnalyse::END_TASK;
  }
  void setTaskExecutable() {
    task_analyse_result_.status = eTaskAnalyse::EXECUTABLE;
  }
  void setTaskUnexecutable() {
    task_analyse_result_.status = eTaskAnalyse::UNEXECUTABLE;
  }
  void setTaskExeOK() {
    sub_task_result_.status = eSubTaskStatus::OK;
  }
  void setTaskExeFault() {
    sub_task_result_.status = eSubTaskStatus::EXCPETION ;
  }
  void setTaskUnhandledExcption() {
    exception_analyse_result_.status = eExceptionHandleRes::UNEXECUTABLE  ;
  }
  void setTaskHandlingExcption() {
    exception_analyse_result_.status = eExceptionHandleRes::EXECUTABLE ;
  }
  void setEndTaskHandleOver() {
    endtask_handle_result_.status = eEndTaskHandleRes::HANDING_OVER ;
  }

  // get method
  bool IsTaskOver() {
    return (task_analyse_result_.status == eTaskAnalyse::END_TASK );
  }

  bool IsTaskExecutable() {
    return (task_analyse_result_.status == eTaskAnalyse::EXECUTABLE );
  }

  bool IsTaskUnexecutable() {
    return (task_analyse_result_.status == eTaskAnalyse::UNEXECUTABLE );
  }

  bool IsTaskExeOK() {
    return (sub_task_result_.status == eSubTaskStatus::OK );
  }

  bool IsTaskExeFault() {
    return (sub_task_result_.status == eSubTaskStatus::EXCPETION );
  }

  bool IsTaskUnhandledExcption() {
    return (exception_analyse_result_.status == eExceptionHandleRes::UNEXECUTABLE );
  }

  bool IsTaskHandlingExcption() {
    return (exception_analyse_result_.status == eExceptionHandleRes::EXECUTABLE );
  }

  bool IsEndTaskHandleOver() {
    return (endtask_handle_result_.status == eEndTaskHandleRes::HANDING_OVER );
  }


  void Reset() {
    is_final_adjust = false;
    ResetAnalyseStatus();
    ResetExeSubTaskStatus();
    ResetExceptionAnalyseStatus();
    ResetEndTaskHandleStatus();
  }
} TaskFSMInfo;

class BusinessBase;
class AlgorithmBase;
class BehaviorContextBase;
typedef struct TaskFSMParam {
  TaskContent task_content;
  TaskFSMInfo task_fsm_info;
  VehicleDriveStatus drive_status;
  LocalizationData localization_data;
  eMapType map_type;
  eTaskExeResult task_exe_status;

  Paths paths;
  
  std::shared_ptr<BusinessBase> business_ptr;
  std::shared_ptr<BusinessBase> last_business_ptr;
  std::shared_ptr<AlgorithmBase> algorithm_ptr;
  std::shared_ptr<AlgorithmBase> last_algorithm_ptr;
  std::shared_ptr<BehaviorContextBase> behavior_context_ptr;
  std::shared_ptr<BehaviorContextBase> last_behavior_context_ptr;
  DebugPlanningMsg debug_planning_msg;
  TaskFSMParam() {
    Reset();
  }
  void Reset() {
    task_content.Reset();
    task_fsm_info.Reset();
    drive_status.Reset();
    task_exe_status = eTaskExeResult::DEFAULT;
    paths.Reset();
    debug_planning_msg.Reset();
    business_ptr.reset();
    last_business_ptr.reset();
    algorithm_ptr.reset();
    last_algorithm_ptr.reset();
    behavior_context_ptr.reset();
    last_behavior_context_ptr.reset();
  }
  void SaveScene() {
    last_business_ptr = business_ptr;
    last_algorithm_ptr = algorithm_ptr;
    last_behavior_context_ptr = behavior_context_ptr;
  }
  void RestoreScene() {
    business_ptr = last_business_ptr;
    algorithm_ptr = last_algorithm_ptr;
    behavior_context_ptr = last_behavior_context_ptr;
  }
} TaskFSMParam;


} // namespace planning
} // namespace acu

#endif // DATAPOOL_INCLUDE_TASK_INPUT_H_