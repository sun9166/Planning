#ifndef DATAPOOL_INCLUDE_COMMON_TYPEDEF_H_
#define DATAPOOL_INCLUDE_COMMON_TYPEDEF_H_

#include <string>
#include <vector>
#include <map>
#include <list>
#include <memory>
#include <cmath>
#include <limits>
#include "public_typedef.h"
#include "locperception_input.h"
#include "task_input.h"
#include "can_input.h"
#include "mapengine_input.h"
#include "prediction_input.h"
#include "task_typedef.h"
#include "cognition_typedef.h"
#include "behavior_typedef.h"
#include "decision_typedef.h"
#include "speedplan_typedef.h"
#include "common_config.h"
#include "v2x_alerts_typedeh.h"
#include "common/base/time/include/node_time.h"
#include "src/execution/business/include/business_base.h"
//#include "src/algorithm/methods/astar/interface/include/astar_avoid.h"
//#include "src/algorithm/methods/astar/interface/include/astar_adjust.h"
//#include "planning_msgs/TrajectoryPoint.h"
#include "planning_msgs.pb.h"


namespace acu {
namespace planning {


typedef struct PlanningMainData {
  /***********INPUT***************/
  LocPerception loc_perception;  // input from  perception
  TaskContent task_content;  // input from  task topic
  VehicleDriveStatus drive_status; // input from  task topic
  VehicleCanStatus can_status;  // input from  can topic

  MapEngineData mapengine_data;  // input from  mapengine
  
  std::vector<MapEngineData> mapengine_data_list;  // input from  mapengine
  
  PredictionData prediction_data;  // input from  prediction
  
  /***********INPUT***************/
  
  /***********OUTPUT***************/
  eTaskExeResult task_exe_result;
  PathData motion_path; // 交给motion来定义数据结构
  DebugPlanningMsg debug_planning_msg;
  PredictionData prediction_bk_data;
  v2xAlertsData v2x_alerts_data;
  // path
  /***********OUTPUT***************/

  ConfigInfo config_info;
  TaskFSMInfo task_fsm_info;
  CognitionInfo cognition_info;
  BehaviorFSMInfo behavior_fsm_info;
  SpeedplanInfo speedplan_info;
  DecisionInfo decision_info;

  std::shared_ptr<BusinessBase> business_ptr;
  std::shared_ptr<BusinessBase> last_business_ptr;
  std::shared_ptr<AlgorithmBase> algorithm_ptr;
  std::shared_ptr<AlgorithmBase> last_algorithm_ptr;
  std::shared_ptr<BehaviorContextBase> behavior_context_ptr;
  std::shared_ptr<BehaviorContextBase> last_behavior_context_ptr;

  Paths paths;
  PathData result_path;
  PathData result_pursuit_path;
  PathData calculate_path;

  ComputeThreadType compute_thread_type;

  int turning;
  int impassable_flag;
  int ready_state;
  planning_msgs::TrajectoryPoint stitching_point; //@pqg

  PlanningMainData() {
    // is_loc_valid = false;
    stitching_point.set_x(0.0);
    stitching_point.set_y(0.0);
  }

  void ResetAll() {

  }
} PlanningMainData;

typedef struct PlanningSwapComputeData {
  LocalizationData localization_data;
  Paths paths;
  CognitionInfo cognition_info;
  PathData result_path;
  CarModel car_model;
  bool is_final_adjust;
  //DebugDWAMsg DWA_message;
  //DebugASTARMsg ASTAR_message;
  //std::shared_ptr<acu::astar_interface::AstarAvoid> astar_avoid_ptr;
  //std::shared_ptr<acu::astar_interface::AstarAdjust> astar_adjust_ptr;
  PlanningSwapComputeData() {
    //astar_avoid_ptr = std::make_shared<acu::astar_interface::AstarAvoid>();
   // astar_adjust_ptr = std::make_shared<acu::astar_interface::AstarAdjust>();
    is_final_adjust = false;
  }
} PlanningSwapComputeData;

typedef struct PlanningSwapCognitionData {
  int bak_type;
} PlanningSwapCognitionData;
typedef struct PlanningSwapBehaviorData {
  int bak_type;
} PlanningSwapBehaviorData;
typedef struct PlanningSwapPathplanData {
  int bak_type;
} PlanningSwapPathplanData;

} // namespace planning 
} // namespace acu

#endif // DATAPOOL_INCLUDE_COMMON_TYPEDEF_H_
