#ifndef SRC_ALGORITHM_INTERFACE_BACK_AVOID_INCLUDE_BACK_AVOID_H_
#define SRC_ALGORITHM_INTERFACE_BACK_AVOID_INCLUDE_BACK_AVOID_H_

#include "src/algorithm/interface/interface_base/include/algorithm_base.h"
#include "src/algorithm/methods/astar/interface/include/astar_avoid.h"
#include "common/base/thread_pool/include/thread_pool.h"

using namespace acu::astar_interface;

namespace acu {
namespace planning {

class BackAvoid : public AlgorithmBase {
 public:
  BackAvoid() {
    run_in_compute_thread_ = false;
    algorithm_type_.SetBackAvoid();
    result_path_.is_new = false;
    target_index_ = -888;
  }
  ~BackAvoid() {}



  // int RunInNative() {
  //  astar_avoid_config_.car_model = car_status_.car_model;
  //  astar_avoid_.SetAstarAvoidConfig(astar_avoid_config_);
  //  int result = astar_avoid_.GenerateAstarPath(loc_perception_.localization,
  //               paths_, loc_perception_.perception_data, result_path_);
  //  if (result == -1) {
  //    AERROR << "GenerateForceAvoid failed";
  //    return -1;
  //  }
  //  last_result_path_point_ = result_path_.path.back();
  //  result_path_.is_new = true;
  //  is_init_over_ = true;
  //  return 0;
  // }

//0 success  1 busy  -1 error 2 idle
  eCheckAlgthmResult CheckBackAvoidResult() {
    auto thread_pool = ThreadPool::Instance();
    eThreadStatus thread_state = thread_pool->CheckThreadState(ePlanningThread::COMPUTE_THREAD_ID);
    DataPool* DP = DataPool::Instance();
    AINFO << "CheckForwardAvoidResult  " << DP->GetMainDataPtr()->compute_thread_type.str_type;
    if (DP->GetMainDataPtr()->compute_thread_type.type != eComputeThreadType::PLANNER_ASTAR_AVOID &&
        thread_state == eThreadStatus::BUSY) {
      return eCheckAlgthmResult::ERROR;
    }
    if (DP->GetMainDataPtr()->compute_thread_type.type == eComputeThreadType::PLANNER_ASTAR_AVOID &&
        thread_state == eThreadStatus::BUSY) {
      return eCheckAlgthmResult::BUSY;
    }
    if (DP->GetMainDataPtr()->compute_thread_type.type != eComputeThreadType::PLANNER_ASTAR_AVOID &&
        thread_state == eThreadStatus::WAITING) {
      DP->GetMainDataPtr()->compute_thread_type.Reset();
      // thread_pool->ResetThreadStatus(ePlanningThread::COMPUTE_THREAD_ID);
      return eCheckAlgthmResult::IDLE;
    }
    if (DP->GetMainDataPtr()->compute_thread_type.type == eComputeThreadType::PLANNER_ASTAR_AVOID &&
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
    //prepare env
    DataPool* DP = DataPool::Instance();
    DP->GetMainDataPtr()->compute_thread_type.SetPlannerAstarAvoid();
    DP->BakComputeThreadData();
    const auto work = [&]() {
        DataPool* DP = DataPool::Instance();
        // set basic config
        AstarAvoidConfig astar_avoid_config;
        astar_avoid_config.car_model = DP->GetSwapComputeDateRef().car_model;
        bool emergency_flag = DP->GetSwapComputeDateRef().cognition_info.unstruct_env_info.semantic_info.IsEmergencyTimeUp();
        if (emergency_flag) {
            astar_avoid_config.use_ultra = true;
        } else {
            astar_avoid_config.use_ultra = true;
        }
        // set alogorithm
        DP->GetSwapComputeDateRef().astar_avoid_ptr = std::make_shared<acu::astar_interface::AstarAvoid>();
        DP->GetSwapComputeDateRef().astar_avoid_ptr->SetAstarAvoidConfig(astar_avoid_config);
        int result = DP->GetSwapComputeDateRef().astar_avoid_ptr->GenerateAstarPath(
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


  int InitAlgorithm() { //error -1 or other,  success 0   , busy 1

    auto thread_pool = ThreadPool::Instance();
    eCheckAlgthmResult result = CheckBackAvoidResult();
    int ret = -1;
    if (result == eCheckAlgthmResult::ERROR) {
      AINFO << "CheckBackAvoidResult result ERROR";
      // return RunInNative();
      is_init_over_ = false;
      thread_pool->StopWork(ePlanningThread::COMPUTE_THREAD_ID);
    }
    if (result == eCheckAlgthmResult::IDLE) {
      AINFO << "CheckBackAvoidResult result IDLE";
      RunInThread();
      is_init_over_ = false;
      return 1;
    }
    if (result == eCheckAlgthmResult::SUCCESS) {   //thread over
      ret = thread_pool->GetWorkReturnValue(ePlanningThread::COMPUTE_THREAD_ID);
      thread_pool->ResetThreadStatus(ePlanningThread::COMPUTE_THREAD_ID);
      if (ret == -1) {
        AERROR << "Generate back avoid error";
        is_init_over_ = false;
        return -1;
      } else {
        AINFO << "Generate back avoid is success";
        DataPool* DP = DataPool::Instance();
        result_path_ = DP->GetSwapComputeDateRef().result_path;
        result_path_.is_new = true;
        last_result_path_point_ = result_path_.path.back();
        target_index_ = result_path_.index;
        is_init_over_ = true;
        return 0;
      }
    }
    AINFO << "Generate back avoid is busy";
    return 1;

  }

  int CheckAlgorithmState() { //error -1 or other,  success 0   , busy 1
    return 0;
    if (!is_init_over_) {
      AERROR << "is_init_over_ is failed, shoudn't in here";
      return -1;
    }

    if (target_index_ == -888 || target_index_ == -999 ) {
      AERROR << "back avoid  index is unvalid " << target_index_;
      return 0;
    }
    return 0;

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
    //   AERROR << "back avoid get the endof path";
    //   return 0;
    // }
    // if (sumdis < tolerance_dis) {
    //   AINFO << "back avoid over";

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