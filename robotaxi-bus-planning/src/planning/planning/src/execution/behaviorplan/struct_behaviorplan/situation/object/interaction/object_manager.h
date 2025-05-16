#ifndef OBJECT_MANAGER_H
#define OBJECT_MANAGER_H

#include "src/execution/behaviorplan/struct_behaviorplan/situation/object/interaction/acc_analysis.h"
#include "src/execution/behaviorplan/struct_behaviorplan/situation/object/interaction/junction_analysis.h"

namespace acu {
namespace planning {

class ObjectManager {
 public:
  ObjectManager();
  ~ObjectManager();

  void AddObjectInfo();
  void UpdateObjectInfo();
  void FindInvaders();
  void AddKeyObject(const ReferenceLineFrame* line);
  void AddOriginalMapDebug();
  void AddInteractionMapDebug();

 private:
 	DecisionContext* context_ = DecisionContext::Instance();
 	StructReferenceLineInfo* reference_line_;
  ReferenceLineFrame* current_line_;
  ReferenceLineFrame* target_line_;
  ReferenceLineFrame* search_line_;
};

}  //  namespace planning
}  //  namespace acu

#endif
