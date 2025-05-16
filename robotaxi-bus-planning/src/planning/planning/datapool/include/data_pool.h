
#ifndef DATAPOOL_INCLUDE_DATA_POOL_H_
#define DATAPOOL_INCLUDE_DATA_POOL_H_

#include <condition_variable>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <vector>
#include "datapool/include/common_typedef.h"
#include "common/base/log/include/log.h"
#include "common/base/macros.h"

namespace acu {
namespace planning {

class DataPool {
 public:
  PlanningMainData* GetMainDataPtr() { return &main_data_; }
  PlanningMainData& GetMainDataRef() { return main_data_; }

  PlanningSwapComputeData &GetSwapComputeDateRef() { return compute_data_; }
  // PlanningSwapAdjustData &GetSwapAdjustDateRef() { return adjust_data_; }
  PlanningSwapCognitionData &GetSwapCognitionDateRef() { return cognition_data_; }
  PlanningSwapBehaviorData &GetSwapBehaviorDateRef() { return behavior_data_; }
  PlanningSwapPathplanData &GetSwapPathplanDateRef() { return pathplan_data_; }
  void BakComputeThreadData() {
    compute_data_.localization_data = main_data_.loc_perception.localization_data;
    compute_data_.paths = main_data_.paths;
    compute_data_.cognition_info = main_data_.cognition_info;
    compute_data_.car_model = main_data_.config_info.car_model;
    compute_data_.is_final_adjust = main_data_.task_fsm_info.is_final_adjust;
  }
  
  std::mutex mutex_;

 private:
  PlanningMainData main_data_;
  PlanningSwapComputeData compute_data_;
  PlanningSwapCognitionData cognition_data_; 
  PlanningSwapBehaviorData behavior_data_;
  PlanningSwapPathplanData pathplan_data_;
  DataPool() {}

  BASE_DECLARE_SINGLETON(DataPool)
};

}  // namespace planning
}  // namespace acu

#endif  // DATAPOOL_INCLUDE_DATA_POOL_H_
