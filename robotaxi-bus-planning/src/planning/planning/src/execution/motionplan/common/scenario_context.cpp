/**
 * @file scenario_context.cpp
 **/

#include "scenario_context.h"

namespace acu {
namespace planning {

ScenarioContext::ScenarioContext() {
  
}

void ScenarioContext::Clear() {
  scenario_info_.reset();
}

bool ScenarioContext::UpdateLonScenario(const eLonScenarioEnum scenario) {
   scenario_info_.history_lonscenario = scenario_info_.current_lonscenario;	
   scenario_info_.current_lonscenario = scenario;	
   return true;
}

bool ScenarioContext::UpdateLatScenario(const eLatScenarioEnum scenario) {
   scenario_info_.history_latscenario = scenario_info_.current_latscenario;	
   scenario_info_.current_latscenario = scenario;	
   return true;
}

}  // namespace planning
}  // namespace acu
