#ifndef SRC_EXECUTION_TASK_INCLUDE_TASK_CONTEXT_H_
#define SRC_EXECUTION_TASK_INCLUDE_TASK_CONTEXT_H_

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <stddef.h>
#include <functional>
#include "src/execution/task/include/TaskFSM.h"
#include "datapool/include/data_pool.h"
#include "datapool/include/task_typedef.h"

namespace acu {
namespace planning {

typedef std::function<bool (void)> pEventFunction;

class TaskContext {
 public:
  TaskContext() {
    ptask_fsm_ = std::make_shared<TaskFSM>((void *)0);
  }
  ~TaskContext() {};
  void PullData() {
    auto DP = DataPool::Instance();
    task_param_.task_content = DP->GetMainDataRef().task_content;
    task_param_.task_fsm_info = DP->GetMainDataRef().task_fsm_info;
    task_param_.task_exe_status = DP->GetMainDataRef().task_exe_result;
    task_param_.drive_status = DP->GetMainDataRef().drive_status;
    task_param_.localization_data = DP->GetMainDataRef().loc_perception.localization_data;
    task_param_.map_type = DP->GetMainDataRef().cognition_info.maptype;
    task_param_.business_ptr = DP->GetMainDataRef().business_ptr;
    task_param_.last_business_ptr = DP->GetMainDataRef().last_business_ptr;
    task_param_.algorithm_ptr = DP->GetMainDataRef().algorithm_ptr;
    task_param_.last_algorithm_ptr = DP->GetMainDataRef().last_algorithm_ptr;
    task_param_.paths = DP->GetMainDataRef().paths;
    task_param_.debug_planning_msg = DP->GetMainDataRef().debug_planning_msg;
  }

  void PushData() {
    auto DP = DataPool::Instance();
    DP->GetMainDataRef().task_fsm_info = task_param_.task_fsm_info;
    DP->GetMainDataRef().task_content = task_param_.task_content;
    DP->GetMainDataRef().task_exe_result = task_param_.task_exe_status;
    DP->GetMainDataRef().business_ptr = task_param_.business_ptr;
    DP->GetMainDataRef().last_business_ptr = task_param_.last_business_ptr;
    DP->GetMainDataRef().algorithm_ptr = task_param_.algorithm_ptr;
    DP->GetMainDataRef().last_algorithm_ptr = task_param_.last_algorithm_ptr;
    DP->GetMainDataRef().paths = task_param_.paths;
    DP->GetMainDataRef().debug_planning_msg = task_param_.debug_planning_msg;
    if (task_param_.business_ptr != nullptr) {
      AINFO << task_param_.business_ptr->GetBusinessType().type_str;
    } else {
      AINFO << "Business empty";
    }
  }

  void ScanTaskEvent() {
    AINFO << "[Task Entry:]"<< task_param_.task_fsm_info.task_status.str_status;
    AINFO << "[Task Entry:]"<< (int)task_param_.task_fsm_info.task_status.status;
    AINFO << "[Task_Status:]" << (int)task_param_.task_content.task_state;
    switch (task_param_.task_fsm_info.task_status.status) {
    case eTaskStatus::IDLE_TASK: ScanVectorEvents(idletask_state_scan_event); break;
    case eTaskStatus::ANALYSE_TASK: ScanVectorEvents(analysetask_state_scan_event); break;
    case eTaskStatus::EXECUTE_TASK: ScanVectorEvents(executetask_state_scan_event); break;
    case eTaskStatus::SUSPEND_TASK: ScanVectorEvents(suspendtask_state_scan_event); break;
    case eTaskStatus::EXECEPTIONAL_TASK: ScanVectorEvents(exceptionaltask_state_scan_event); break;
    case eTaskStatus::EMEREGENCY_BTN_HANDLE: ScanVectorEvents(emergencybtnhandle_state_scan_event); break;
    case eTaskStatus::REMOTE_CONTROL: ScanVectorEvents(remotecontrol_state_scan_event); break;
    case eTaskStatus::END_TASK: ScanVectorEvents(endtask_state_scan_event); break;
    default: ScanVectorEvents(idletask_state_scan_event); break;
    }
  }

 private:
  bool TaskIsNewEvent() {
    if (task_param_.task_content.IsNewTask()) {
      AINFO << "[task condition set]: IsNewTask";
      ptask_fsm_->TaskIsNew((void*)&task_param_);
      task_param_.debug_planning_msg.SetTaskIsNewEvent();
      return true;
    }
    return false;
  }
  bool TaskExecutableEvent() {
    if (task_param_.task_fsm_info.IsTaskExecutable()) {
      AINFO << "[task condition set]: IsTaskExecutable";
      task_param_.debug_planning_msg.SetTaskExecutableEvent();
      ptask_fsm_->TaskExecutable((void*)&task_param_);
      return true;
    }
    return false;
  }
  bool TaskUnexecutableEvent() {
    if (task_param_.task_fsm_info.IsTaskUnexecutable()) {
      AINFO << "[task condition set]: IsTaskUnexecutable";
      task_param_.debug_planning_msg.SetTaskUnexecutableEvent();
      ptask_fsm_->TaskUnexecutable((void*)&task_param_);
      return true;
    }
    return false;
  }
  bool TaskExecuteOverEvent() {
    if (task_param_.task_fsm_info.IsTaskExeOK()) {
      AINFO << "[task condition set]: IsTaskExeOver";
      task_param_.debug_planning_msg.SetTaskExecuteOverEvent();
      ptask_fsm_->TaskExecuteOver((void*)&task_param_);
      return true;
    }
    return false;
  }
  bool TaskExecuteFaultEvent() {
    if (task_param_.task_fsm_info.IsTaskExeFault()) {
      AINFO << "[task condition set]: IsTaskExeFault";
      task_param_.debug_planning_msg.SetTaskExecuteFaultEvent();
      ptask_fsm_->TaskExecuteFault((void*)&task_param_);
      return true;
    }
    return false;
  }
  bool TaskOverEvent() {
    if (task_param_.task_fsm_info.IsTaskOver()) {
      AINFO << "[task condition set]: IsTaskOver";
      task_param_.debug_planning_msg.SetTaskOverEvent();
      ptask_fsm_->TaskOver((void*)&task_param_);
      return true;
    }
    return false;
  }
  bool TaskAbortedEvent() {
    if (task_param_.map_type != eMapType::STRUCTURE_ROAD) {
      if (task_param_.task_content.IsAbordTask()) {
        AINFO << "[task condition set]: IsAbordTask";
        task_param_.debug_planning_msg.SetTaskAbortedEvent();
        ptask_fsm_->TaskAborted((void*)&task_param_);
        return true;
      }
      return false;
    } else {
      if (task_param_.task_content.IsAbordTask() &&
          std::fabs(task_param_.localization_data.velocity < 0.1)) {
        AINFO << "[task condition set]: IsAbordTask";
        task_param_.debug_planning_msg.SetTaskAbortedEvent();
        ptask_fsm_->TaskAborted((void*)&task_param_);
        return true;
      }
      return false;
    }
  }
  bool TaskSuspendedEvent() {
    if (task_param_.map_type != eMapType::STRUCTURE_ROAD) {
      if (task_param_.task_content.IsSuspendedTask()) {
        AINFO << "[task condition set]: IsSuspendedTask";
        task_param_.debug_planning_msg.SetTaskSuspendedEvent();
        ptask_fsm_->TaskSuspended((void*)&task_param_);
        return true;
      }
      return false;
    } else {
      return false;
    }
  }
  bool TaskContinuedEvent() {
    if (task_param_.task_content.IsContinuedTask()) {
      AINFO << "[task condition set]: TaskContinued";
      task_param_.debug_planning_msg.SetTaskContinuedEvent();
      ptask_fsm_->TaskContinued((void*)&task_param_);
      return true;
    }
    return false;
  }
  bool SyetemExceptionEvent() {
    if (task_param_.map_type != eMapType::STRUCTURE_ROAD) {
      if (task_param_.drive_status.IsSyetemException()) {
        AINFO << "[task condition set]: SyetemException";
        task_param_.debug_planning_msg.SetSyetemExceptionEvent();
        ptask_fsm_->SyetemException((void*)&task_param_);
        return true;
      }
      return false;
    } else {
      return false;
    }
  }
  bool SyetemNormalEvent() {
    if (task_param_.drive_status.IsSyetemNormal()) {
      AINFO << "[task condition set]: SyetemNormal";
      task_param_.debug_planning_msg.SetSyetemNormalEvent();
      ptask_fsm_->SyetemNormal((void*)&task_param_);
      return true;
    }
    return false;
  }
  bool BtnEmergencyPressedEvent() {
    if (task_param_.drive_status.IsBtnEmergencyPressed()) {
      AINFO << "[task condition set]: BtnEmergencyPressed";
      task_param_.debug_planning_msg.SetBtnEmergencyPressedEvent();
      ptask_fsm_->BtnEmergencyPressed((void*)&task_param_);
      return true;
    }
    return false;
  }
  bool BtnEmergencyUnpressedEvent() {
    if (task_param_.drive_status.IsBtnEmergencyUnpressed()) {
      AINFO << "[task condition set]: BtnEmergencyUnpressed";
      task_param_.debug_planning_msg.SetBtnEmergencyUnpressedEvent();
      ptask_fsm_->BtnEmergencyUnpressed((void*)&task_param_);
      return true;
    }
    return false;
  }
  bool RemoteCtrlConnectedEvent() {
    if (task_param_.drive_status.IsRemoteCtrlConnected()) {
      AINFO << "[task condition set]: RemoteCtrlConnected";
      task_param_.debug_planning_msg.SetRemoteCtrlConnectedEvent();
      ptask_fsm_->RemoteCtrlConnected((void*)&task_param_);
      return true;
    }
    return false;
  }
  bool RemoteCtrlUnconnectedEvent() {
    if (task_param_.drive_status.IsRemoteCtrlUnconnected()) {
      AINFO << "[task condition set]: RemoteCtrlUnconnected";
      task_param_.debug_planning_msg.SetRemoteCtrlUnconnectedEvent();
      ptask_fsm_->RemoteCtrlUnconnected((void*)&task_param_);
      return true;
    }
    return false;
  }
  bool EndTaskHandleOverEvent() {
    if (task_param_.task_fsm_info.IsEndTaskHandleOver()) {
      AINFO << "[task condition set]: EndTaskHandleOver";
      task_param_.debug_planning_msg.SetEndTaskHandleOverEvent();
      ptask_fsm_->EndTaskHandleOver((void*)&task_param_);
      return true;
    }
    return false;
  }

  void ScanVectorEvents(std::vector<pEventFunction> &scan_events) {
    for (int i = 0; i < scan_events.size(); i++) {
      if (scan_events[i]()) return;
    }
  }

 private:
  TaskFSMParam task_param_;
  std::shared_ptr<TaskFSM> ptask_fsm_;

  std::vector<pEventFunction> idletask_state_scan_event = {
    std::bind(&TaskContext::TaskIsNewEvent, this),
    std::bind(&TaskContext::SyetemNormalEvent, this)
  };
  std::vector<pEventFunction> analysetask_state_scan_event = {
    std::bind(&TaskContext::TaskIsNewEvent, this),
    std::bind(&TaskContext::TaskExecutableEvent, this),
    std::bind(&TaskContext::TaskUnexecutableEvent, this),
    std::bind(&TaskContext::TaskOverEvent, this),
    std::bind(&TaskContext::TaskAbortedEvent, this),
    std::bind(&TaskContext::SyetemNormalEvent, this)
  };
  std::vector<pEventFunction> executetask_state_scan_event = {
    std::bind(&TaskContext::TaskExecuteOverEvent, this),
    std::bind(&TaskContext::TaskExecuteFaultEvent, this),
    std::bind(&TaskContext::TaskAbortedEvent, this),
    std::bind(&TaskContext::TaskSuspendedEvent, this),
    std::bind(&TaskContext::SyetemExceptionEvent, this),
    std::bind(&TaskContext::BtnEmergencyPressedEvent, this),
    std::bind(&TaskContext::RemoteCtrlConnectedEvent, this),
    std::bind(&TaskContext::SyetemNormalEvent, this)
  };
  std::vector<pEventFunction> suspendtask_state_scan_event = {
    std::bind(&TaskContext::TaskAbortedEvent, this),
    std::bind(&TaskContext::TaskContinuedEvent, this),
    std::bind(&TaskContext::SyetemNormalEvent, this)
  };
  std::vector<pEventFunction> exceptionaltask_state_scan_event = {
    std::bind(&TaskContext::TaskAbortedEvent, this),
    std::bind(&TaskContext::SyetemNormalEvent, this),
    std::bind(&TaskContext::SyetemNormalEvent, this)
  };
  std::vector<pEventFunction> emergencybtnhandle_state_scan_event = {
    std::bind(&TaskContext::TaskAbortedEvent, this),
    std::bind(&TaskContext::BtnEmergencyUnpressedEvent, this),
    std::bind(&TaskContext::SyetemNormalEvent, this)
  };
  std::vector<pEventFunction> remotecontrol_state_scan_event = {
    std::bind(&TaskContext::TaskAbortedEvent, this),
    std::bind(&TaskContext::RemoteCtrlUnconnectedEvent, this),
    std::bind(&TaskContext::SyetemNormalEvent, this)
  };
  std::vector<pEventFunction> endtask_state_scan_event = {
    std::bind(&TaskContext::TaskIsNewEvent, this),
    std::bind(&TaskContext::EndTaskHandleOverEvent, this),
    std::bind(&TaskContext::SyetemNormalEvent, this)
  };
};


} // namespace planning
} // namespace acu

#endif // SRC_EXECUTION_TASK_INCLUDE_TASK_CONTEXT_H_ 