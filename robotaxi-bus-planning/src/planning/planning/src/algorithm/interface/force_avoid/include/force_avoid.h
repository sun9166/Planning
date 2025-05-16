#ifndef SRC_ALGORITHM_INTERFACE_FORCE_AVOID_INCLUDE_FORCE_AVOID_H_
#define SRC_ALGORITHM_INTERFACE_FORCE_AVOID_INCLUDE_FORCE_AVOID_H_


#include "src/algorithm/interface/interface_base/include/algorithm_base.h"
#include "src/algorithm/methods/dwa/interface/include/force_avoid.h"
#include "common/base/thread_pool/include/thread_pool.h"
// #include "runnable/thread/include/compute_thread.h"

using namespace acu::dwa;
namespace acu {
namespace planning {

class ForceAvoid : public AlgorithmBase {
public:
  ForceAvoid() {
    run_in_compute_thread_ = false;
    algorithm_type_.SetForceAvoid();
    result_path_.is_new = false;
    target_index_ = -888;

  }
  ~ForceAvoid() {}

  eCheckAlgthmResult CheckForceAvoidResult() {
    auto thread_pool = ThreadPool::Instance();
    eThreadStatus thread_state = thread_pool->CheckThreadState(ePlanningThread::COMPUTE_THREAD_ID);
    DataPool* DP = DataPool::Instance();
    if (DP->GetMainDataPtr()->compute_thread_type.type != eComputeThreadType::PLANNER_DWA_FORCE_AVOID &&
        thread_state == eThreadStatus::BUSY) {
      return eCheckAlgthmResult::ERROR;
    }
    if (DP->GetMainDataPtr()->compute_thread_type.type == eComputeThreadType::PLANNER_DWA_FORCE_AVOID &&
        thread_state == eThreadStatus::BUSY) {
      return eCheckAlgthmResult::BUSY;
    }
    if (DP->GetMainDataPtr()->compute_thread_type.type != eComputeThreadType::PLANNER_DWA_FORCE_AVOID &&
        thread_state == eThreadStatus::WAITING) {
      DP->GetMainDataPtr()->compute_thread_type.Reset();
      // thread_pool->ResetThreadStatus(ePlanningThread::COMPUTE_THREAD_ID);
      return eCheckAlgthmResult::IDLE;
    }
    if (DP->GetMainDataPtr()->compute_thread_type.type == eComputeThreadType::PLANNER_DWA_FORCE_AVOID &&
        thread_state == eThreadStatus::WAITING) {
      DP->GetMainDataPtr()->compute_thread_type.Reset();
      // thread_pool->ResetThreadStatus(ePlanningThread::COMPUTE_THREAD_ID);
      return eCheckAlgthmResult::SUCCESS;
    }
    if (thread_state == eThreadStatus::BUSY)
      return eCheckAlgthmResult::ERROR;
    return eCheckAlgthmResult::IDLE;
  }

  bool RunInThread() {
    auto thread_pool = ThreadPool::Instance();
    if (thread_pool == nullptr) return false;
    DataPool* DP = DataPool::Instance();
    DP->GetMainDataPtr()->compute_thread_type.SetPlannerDwaForceAvoid();
    DP->BakComputeThreadData();
    const auto work = [&]() {
      DataPool* DP = DataPool::Instance();
      DwaConfig dwa_config_;
      std::shared_ptr<DwaForceAvoid> dwa_force_avoid_ptr = std::make_shared<DwaForceAvoid>();
      dwa_config_.car_model = DP->GetSwapComputeDateRef().car_model;
      dwa_config_.risk_mode_enable = false;
      dwa_force_avoid_ptr->SetDwaConfig(dwa_config_);
      int result = dwa_force_avoid_ptr->GenerateDwaForceAvoidPath(
                       DP->GetSwapComputeDateRef().localization_data,
                       DP->GetSwapComputeDateRef().paths,
                       DP->GetSwapComputeDateRef().cognition_info.unstruct_env_info,
                       DP->GetSwapComputeDateRef().result_path);
      return result;

    };
    if (thread_pool->AddThreadWork(ePlanningThread::COMPUTE_THREAD_ID, work ) == -1) {
      AERROR << "add work failed ";
    }
    return true;
  }

  int InitAlgorithm() { //error -1,  success 0 1 busy
    auto thread_pool = ThreadPool::Instance();
    eCheckAlgthmResult result = CheckForceAvoidResult();
    int ret = -1;
    if (result == eCheckAlgthmResult::ERROR) {
      is_init_over_ = false;
      thread_pool->StopWork(ePlanningThread::COMPUTE_THREAD_ID);
      return -1;
    }
    if (result == eCheckAlgthmResult::SUCCESS) {
      ret = thread_pool->GetWorkReturnValue(ePlanningThread::COMPUTE_THREAD_ID);
      thread_pool->ResetThreadStatus(ePlanningThread::COMPUTE_THREAD_ID);
      if (ret == -1) {  //fail
        AERROR << "GenerateForce avoid error";
        is_init_over_ = false;
        return -1;
      } else {
        DataPool* DP = DataPool::Instance();
        result_path_ = DP->GetSwapComputeDateRef().result_path;
        result_path_.is_new = true;
        last_result_path_point_ = result_path_.path.back();
        target_index_ =  result_path_.index;
        is_init_over_ = true;
        return 0;
      }
    }
    if (result == eCheckAlgthmResult::BUSY) {
      return 1;
    }
    if (result == eCheckAlgthmResult::IDLE) {
      RunInThread();
      is_init_over_ = false;
      return 1;
    }
    return 1;
  }

  int CheckAlgorithmState() { //error -1,  success 0 1 busy
    return 0;
    // if (!is_init_over_) {
    //   AERROR << "is_init_over_ is failed, shoudn't in here";
    //   return -1;
    // }
    // if (target_index_ == -888 || target_index_ == -999 ) {
    //   AERROR << "forceavoid  index is unvalid " << target_index_;
    //   return 0;
    // }

    // double tolerance_dis = 0.5;
    // auto it = paths_.front_local_path.path.begin();

    // double sumdis = 0.0;
    // for (; it != paths_.front_local_path.path.end() &&
    //         std::next(it, 1) != paths_.front_local_path.path.end(); ++it) {
    //   double temp_dis = std::hypot(it->x - std::next(it, 1)->x, it->y - std::next(it, 1)->y);
    //   sumdis += temp_dis;
    //   if ( std::next(it, 1)->index > target_index_) {
    //     break;
    //   }
    //   if (sumdis > tolerance_dis) {
    //     break;
    //   }
    // }
    // if (std::next(it, 1) == paths_.front_local_path.path.end()) {
    //   AERROR << "forceavoid get the endof path";
    //   return 0;
    // }
    // if (sumdis < tolerance_dis) {
    //   AINFO << "forceavoid over";

    //   return 0;
    // }
    // return 1;
  }



private:

  bool run_in_compute_thread_;
  Site last_result_path_point_;
  int target_index_;

};
}
}








#endif