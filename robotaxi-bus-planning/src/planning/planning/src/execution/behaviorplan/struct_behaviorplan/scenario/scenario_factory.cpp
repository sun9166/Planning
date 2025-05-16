#include "scenario_factory.h"

namespace acu {
namespace planning {

ScenarioFactory::ScenarioFactory() {}

ScenarioBase* ScenarioFactory::GetScenarioPtr(const eScenarioEnum type) {
	ScenarioBase* ptr = nullptr;
	switch (type) {
    case eScenarioEnum::CRUISE:
      ptr = new CruiseDecider();
      break;
    case eScenarioEnum::JUNCTION:
      ptr = new JunctionDecider();
      break;
    case eScenarioEnum::PULLOVER:
      ptr = new PulloverDecider();
      break;
    case eScenarioEnum::VALET:
      ptr = new ValetDecider();
      break;
    case eScenarioEnum::HIGHWAY:
      ptr = new HighwayDecider();
      break;
    default:
      break;
  }
  return ptr;
}

ScenarioFactory::~ScenarioFactory() {}

}  // namespace planning
}  // namespace acu