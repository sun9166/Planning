#ifndef SRC_EXECUTION_BEHAVIORPLAN_BEHAVIORBASE_BEHAVIOR_CONTEXT_BASE_H_
#define SRC_EXECUTION_BEHAVIORPLAN_BEHAVIORBASE_BEHAVIOR_CONTEXT_BASE_H_

namespace acu {
namespace planning {

enum class eBehaviorContextType {
  DEFAULT = 0,
  DUMMY_BEHAVIOR_CONTEXT,
  NORMAL_BEHAVIOR,
  BEHAVIOR_TREE
};

typedef struct BehaviorContextType
{
  eBehaviorContextType type;
  std::string type_str;
  BehaviorContextType() {
    type = eBehaviorContextType::DEFAULT;
    type_str = "defualt";
  }
  void SetDummyBehaviorContext() {
    type = eBehaviorContextType::DUMMY_BEHAVIOR_CONTEXT;
    type_str = "dummy Behavior Context";
  }

  void SetNormalBehavior() {
    type = eBehaviorContextType::NORMAL_BEHAVIOR;
    type_str = "Normal Behavior Context";
  }

  void SetBehaviorTree() {
    type = eBehaviorContextType::BEHAVIOR_TREE;
    type_str = "Behavior Tree Context";
  }

} BehaviorContextType;


class BehaviorContextBase
{
public:
  BehaviorContextBase() {}
  virtual ~BehaviorContextBase() {};

  virtual void PullData() = 0;
  virtual void PushData() = 0;
  virtual void Process() = 0;



  BehaviorContextType behavior_context_type_;

};

}
}


#endif // SRC_EXECUTION_BEHAVIORPLAN_BEHAVIORBASE_BEHAVIOR_CONTEXT_BASE_H_
