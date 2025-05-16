#ifndef SRC_ALGORITHM_INTERFACE_FORWARD_AVOID_INCLUDE_FORWARD_AVOID_H_
#define SRC_ALGORITHM_INTERFACE_FORWARD_AVOID_INCLUDE_FORWARD_AVOID_H_


#include "src/algorithm/interface/interface_base/include/algorithm_base.h"
#include "src/algorithm/methods/dwa/interface/include/dwa_avoid.h"
#include "common/base/thread_pool/include/thread_pool.h"


using namespace acu::dwa;
namespace acu {
namespace planning {


class ForwardAvoid : public AlgorithmBase {
 public:
  ForwardAvoid() {
    run_in_compute_thread_ = false;
    algorithm_type_.SetForwardAvoid();
    result_path_.is_new = false;
    target_index_ = -888;

  }
  ~ForwardAvoid() {}



  // int RunInNative() {
  //  dwa_config_.car_model = car_status_.car_model;
  //  dwa_avoid_.SetDwaConfig(dwa_config_);
  //  int result = dwa_avoid_.GenerateDwaPath(loc_perception_.localization,
  //                                          paths_, loc_perception_.perception_detail_data, result_path_);
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
  eCheckAlgthmResult CheckForwardAvoidResult() {
    auto thread_pool = ThreadPool::Instance();
    eThreadStatus thread_state = thread_pool->CheckThreadState(ePlanningThread::COMPUTE_THREAD_ID);
    DataPool* DP = DataPool::Instance();
    AINFO << "CheckForwardAvoidResult  " << DP->GetMainDataPtr()->compute_thread_type.str_type;
    if (DP->GetMainDataPtr()->compute_thread_type.type != eComputeThreadType::PLANNER_DWA_AVOID &&
        thread_state == eThreadStatus::BUSY) {
      return eCheckAlgthmResult::ERROR;
    }
    if (DP->GetMainDataPtr()->compute_thread_type.type == eComputeThreadType::PLANNER_DWA_AVOID &&
        thread_state == eThreadStatus::BUSY) {
      return eCheckAlgthmResult::BUSY;
    }
    if (DP->GetMainDataPtr()->compute_thread_type.type != eComputeThreadType::PLANNER_DWA_AVOID &&
        thread_state == eThreadStatus::WAITING) {
      DP->GetMainDataPtr()->compute_thread_type.Reset();
      // thread_pool->ResetThreadStatus(ePlanningThread::COMPUTE_THREAD_ID);
      return eCheckAlgthmResult::IDLE;
    }
    if (DP->GetMainDataPtr()->compute_thread_type.type == eComputeThreadType::PLANNER_DWA_AVOID &&
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
    DP->GetMainDataPtr()->compute_thread_type.SetPlannerDwaAvoid();
    DP->BakComputeThreadData();
    const auto work = [&]() {
      DataPool* DP = DataPool::Instance();
      DwaConfig dwa_config_;
      std::shared_ptr<DwaPlanner> dwa_avoid_ptr = std::make_shared<DwaPlanner>();
      dwa_config_.car_model = DP->GetSwapComputeDateRef().car_model;
      dwa_config_.risk_mode_enable = false;
      dwa_avoid_ptr->SetDwaConfig(dwa_config_);
      int result = dwa_avoid_ptr->GenerateDwaPath(
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
    eCheckAlgthmResult result = CheckForwardAvoidResult();
    int ret = -1;
    if (result == eCheckAlgthmResult::ERROR) {
      AINFO << "CheckForwardAvoidResult result error";
      is_init_over_ = false;
      thread_pool->StopWork(ePlanningThread::COMPUTE_THREAD_ID);
    }
    if (result == eCheckAlgthmResult::IDLE) {
      AINFO << "CheckForwardAvoidResult result idle";
      RunInThread();
      is_init_over_ = false;
      return 1;
    }
    if (result == eCheckAlgthmResult::SUCCESS) {   //thread over
      ret = thread_pool->GetWorkReturnValue(ePlanningThread::COMPUTE_THREAD_ID);
      thread_pool->ResetThreadStatus(ePlanningThread::COMPUTE_THREAD_ID);
      if (ret == -1) {  //fail
        AERROR << "GenerateForward avoid error";
        is_init_over_ = false;
        return -1;
      } else {
        AINFO << "GenerateForward avoid success";
        DataPool* DP = DataPool::Instance();
        result_path_ = DP->GetSwapComputeDateRef().result_path;
        result_path_.is_new = true;
        last_result_path_point_ = result_path_.path.back();
        target_index_ = result_path_.index;
        is_init_over_ = true;
        return 0;
      }
    }
    AINFO << "GenerateForward avoid is busy";
    return 1;
  }

  double GetAngle(double angle1, double angle2) {
    if (angle1 > angle2 + 180) {
      angle2 = angle2 + 360;
    } else if (angle2 > angle1 + 180) {
      angle1 = angle1 + 360;
    }
    double anglerr = angle1 - angle2;
    return anglerr;
  }
  int CheckAlgorithmState() { //error -1,  success 0 , busy 1
    if (!is_init_over_) {
      AERROR << "is_init_over_ is failed, shoudn't in here";
      return -1;
    }

    if (target_index_ == -888 || target_index_ == -999 ) {
      AERROR << "ForwardAvoid  index is unvalid " << target_index_;
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
    //   AERROR << "ForwardAvoid get the endof path";
    //   return 0;
    // }
    // if (sumdis < tolerance_dis) {
    //   AINFO << "ForwardAvoid over";

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