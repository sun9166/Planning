/**
 * @file scenario_context.h
 **/

#pragma once

#include "macro.h"

namespace acu {
namespace planning {

class ScenarioContext {
 public:

  enum class eLonScenarioEnum {
    DEFAULT_VALUE = 0,
    LANE_FOLLOWING = 1,
    YIELD = 2,
    OVERTAKE = 3,
    STOP = 4
  };

  enum class eLatScenarioEnum {
    DEFAULT_VALUE = 0,
    NUDGE_OFFSET = 1,
    LANE_CHANGE = 2,
    OBSTACLE_AVOID = 3,
    PULL_OVER = 4
  };

  struct ScenarioInfo {
    eLonScenarioEnum current_lonscenario;
    eLonScenarioEnum history_lonscenario;
    eLatScenarioEnum current_latscenario;
    eLatScenarioEnum history_latscenario;
    ScenarioInfo() {
      reset();
    }
    void reset() {
      current_lonscenario = eLonScenarioEnum::DEFAULT_VALUE;
      history_lonscenario = eLonScenarioEnum::DEFAULT_VALUE;
      current_latscenario = eLatScenarioEnum::DEFAULT_VALUE;
      history_latscenario = eLatScenarioEnum::DEFAULT_VALUE;
    }
  };

  ~ScenarioContext() = default;

  void Clear();

  bool UpdateLonScenario(const eLonScenarioEnum scenario);

  bool UpdateLatScenario(const eLatScenarioEnum scenario);

  ScenarioInfo scenario_info() {
    return scenario_info_;
  }
  
 private:
  ScenarioInfo scenario_info_;   
  DECLARE_SINGLETON(ScenarioContext)
};

}  // namespace planning
}  // namespace acu
