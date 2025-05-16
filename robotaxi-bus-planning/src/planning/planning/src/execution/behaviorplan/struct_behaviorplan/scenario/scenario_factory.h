#ifndef SCENARIO_FACTORY_H_
#define SCENARIO_FACTORY_H_

#include "cruise/cruise_decider.h"
#include "highway/highway_decider.h"
#include "junction/junction_decider.h"
#include "pullover/pullover_decider.h"
#include "valet/valet_decider.h"

namespace acu {
namespace planning {

class ScenarioFactory {
 public:
 	ScenarioFactory();
 	~ScenarioFactory();

 	ScenarioBase* GetScenarioPtr(const eScenarioEnum type);
 	
 private:
};

}  // namespace planning
}  // namespace acu

#endif  // VALET_DECIDER_H_