#ifndef SRC_EXECUTION_BUSINESS_INCLUDE_DUMMY_BUSINESS_H_
#define SRC_EXECUTION_BUSINESS_INCLUDE_DUMMY_BUSINESS_H_

#include "business_base.h"
#include "common/base/log/include/log.h"
#include "datapool/include/data_pool.h"
#include "src/execution/behaviorplan/dummy_behavior/include/dummy_behavior.h"
// #include "src/application/execution/dummy/include/dummy_pathplan.h"
namespace acu {
namespace planning {

class DummyBusiness: public BusinessBase
{
public:
  DummyBusiness() {
    business_type_.type = eBusinessType::DUMMY;
  };
  ~DummyBusiness() {};

  bool Init() {
    is_init_ = true;
    // business_param_.pathplan_ptr = std::make_shared<DummyPathplan>();
    behavior_context_ptr_ = std::make_shared<DummyBehaviorContext>();
    AINFO << "DummyBusiness Init";
    return true;
  }
  bool Process()  {
    AINFO << "DummyBusiness Process";
    return true;
  }

  /**
  * @brief : get/set data from datapoll
  * @param : none
  * @return: none
  **/
  void PullData() {
    DataPool* DP = DataPool::Instance();
  }
  void PushData() {
    DataPool* DP = DataPool::Instance();
    // DP->GetMainDataRef().pathplan_ptr = business_param_.pathplan_ptr;
    DP->GetMainDataRef().behavior_context_ptr = behavior_context_ptr_;
    DP->GetMainDataRef().result_path.clear();
    DP->GetMainDataRef().result_path.is_new = false;
    // DP->GetMainDataRef().car_status.sound_light_input.local_path.path.clear();
    DP->GetMainDataRef().debug_planning_msg.refpath_msg.Reset();
  }

private:
  // BusinessParam business_param_;
  std::shared_ptr<BehaviorContextBase> behavior_context_ptr_;


};

}
}

#endif