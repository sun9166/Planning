#ifndef SRC_EXCUTION_BEHAVIORPLAN_DEUMMY_BEHAVIOR_INCLUDE_DUMMY_BEHAVIOR_H_
#define SRC_EXCUTION_BEHAVIORPLAN_DEUMMY_BEHAVIOR_INCLUDE_DUMMY_BEHAVIOR_H_

#include "src/execution/behaviorplan/behavior_base/behavior_context_base.h"
#include "common/base/log/include/log.h"
#include "datapool/include/data_pool.h"

namespace acu {
namespace planning {
class DummyBehaviorContext: public BehaviorContextBase
{
public:
  DummyBehaviorContext() {
    behavior_context_type_.SetDummyBehaviorContext();
  }
  ~DummyBehaviorContext() {}

  void PullData() {
    AINFO << "DummyBehaviorContext PullData";
  }
  void PushData() {
    DataPool *DP = DataPool::Instance();
    AINFO << "DummyBehaviorContext PushData";
    DP->GetMainDataRef().result_path = DP->GetMainDataRef().paths.front_local_path;
    DP->GetMainDataRef().speedplan_info.pathplannerGo = 0; //dummy
  }
  void ResetFSM() {
    AINFO << "DummyBehaviorContext ResetFSM";
  }
  void Process() {
    AINFO << "DummyBehaviorContext Process";
  }
  void ScanBehaviorEvent() {
    AINFO << "DummyBehaviorContext ScanBehaviorEvent";
  }

};
}
}












#endif