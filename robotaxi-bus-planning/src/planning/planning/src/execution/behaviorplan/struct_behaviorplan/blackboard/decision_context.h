#ifndef DECISION_CONTEXT_H_
#define DECISION_CONTEXT_H_

#include "datapool/include/data_pool.h"
#include "common/base/log/include/log.h"
#include "common/util/file.h"
#include "decision_config.pb.h"
#include "planning_config.pb.h"
#include "dp_st_speed_config.pb.h"
#include "src/execution/cognition/struct_cognition/conf/cognition_gflags.h"
#include "src/execution/behaviorplan/struct_behaviorplan/conf/decision_gflags.h"
#include "referenceline_frame/sl_boundary/sl_boundary.h"

namespace acu {
namespace planning {
class DecisionContext {
 public:
 	// cognition input
	StructEnv* cognition_info_;
	PathData trajectory_info_;
	double ego_speed_;
	// config parameter
	ConfigInfo planning_config_;
	std::map<eScenarioEnum, DeciderConfig> config_;
	DpStSpeedConfig speedplan_config_;
	DeciderConfig* decider_config_;
	// frame data
	std::map<int, ReferenceLineFrame*> reference_line_map_;
	eScenarioEnum scenario_type;
	double dis_to_mission_;
	double dis_to_junction_;
	double dis_to_solid_;
	int lc_emergency_level_;
	// decison result
	eReplanReasonEnum replan_reason_ = eReplanReasonEnum::DEFAULT_VALUE;
	DecisionInfo decision_result_;
	OptionStruct best_option_;
	// decision state
	eReplanStateEnum replan_state_ = eReplanStateEnum::CRUISE;
	int target_direction_ = 0;
	int target_line_id_ = 0;
	int replan_counter_ = 0;
	int stable_counter_ = 0;
	// for stop behavior
	bool trafficlight_stop_ = false;
	bool crosswalk_stop_ = false;
	bool crosswalk_slowdown_ = false;
	bool conflict_stop_ = false;
	int conflict_counter_ = 0;
	int crosswalk_stop_counter_ = 0;
	int crosswalk_slowdown_counter_ = 0;
	int stop_type_ = 0;
	double stop_s_;
	Site crosswalk_point_;
	Site conflict_point_;
	// for pull over
	bool pull_over_ = false;
	bool park_road_side_ = false;
	bool find_destination_ = false;
	// other information
	int departure_counter_ = 0;
	int lc_command_counter = 0;
	int lc_command = 0;
	std::map<int, InteractionInfo> st_data_;
	bool pedal_command_;
	int fit_gap_counter_ = 0;
	double fit_gap_speed_ = 100.0;
	std::pair<int, int> pullover_position;

	// for slt
	std::vector<std::vector<std::vector<GridInfoStruct>>> grid_info_;
	std::vector<std::vector<std::vector<GridInfoStruct>>> point_info_;
	std::vector<GridInfoStruct> final_path_;
	std::map<int, Box2d> static_objects_;
	std::map<int, std::map<double, std::pair<Box2d, StructSLBoundary>>> dynamic_objects_;

	// for impassable and replan
	bool is_passable_ = true;
	int mission_lc_blocked_counter_;
	int cruise_blocked_counter_;
	std::vector<string> passable_lane_ids_;
	std::vector<string> black_road_ids_;
 private:
	DecisionContext() {};
		
	BASE_DECLARE_SINGLETON(DecisionContext)
};

}  // namespace planning
}  // namespace acu

#endif  // DECISION_CONTEXT_H_
