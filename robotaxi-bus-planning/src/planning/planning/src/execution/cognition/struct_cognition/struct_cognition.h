#ifndef SRC_EXECUTION_COGNITION_STRUCT_COGNITION_H_
#define SRC_EXECUTION_COGNITION_STRUCT_COGNITION_H_
#include "datapool/include/data_pool.h"
#include "base_func/base_func.h"
#include "input_process/time_matching/time_matching.h"
#include "input_process/light_process/trafficlight_matching.h" 
#include "input_process/line_process/line_process.h"
#include "input_process/object_process/object_process.h"
#include "input_process/object_process/prediction_process.h"
#include "referenceline_frame/frame_base/frame_base.h"
#include "referenceline_frame/frame_process/frame_process.h"
#include "referenceline_frame/new_path_info/new_path_info.h"
#include "scenario/scenario_evaluation.h" 
//#include "planning_debug_msgs/DebugGap.h"
//#include "planning_debug_msgs/DebugObject.h"
//#include "planning_debug_msgs/DebugPdObject.h"
#include "planning_debug_msgs.pb.h"
using geometry::Site;
using DebugObjMsg = planning_debug_msgs::DebugObject;

namespace acu{
namespace planning {

class StructCognition {
public:
  StructEnv* GetMainDataPtr() { return main_data_ptr_; }
  StructEnv& GetMainDataRef() { return *main_data_ptr_; }

  StructCognition();
  ~StructCognition();
  void Init();
  void PullData();
  void ProcessData();
  void PushData();
  void Reset();

private:
  void PushVehicleData();
  bool MotionPathToFrame(ReferenceLineFrame &local_line);
  void DebugMsg();
  void LineDebug(const ReferenceLineFrame &line, planning_debug_msgs::DebugLine& line_msg);
  std::vector<DebugObjMsg> ObjectDebug(const ReferenceLineFrame &line,
    DebugObjMsg& debug_obj, int debug_id = -1);
  bool CheckCurrentPoseEnablePark();


private:
  StructEnv* main_data_ptr_;
	/***********config***************/
  CognitionConfig cognition_config_;
  SpeedplanConfig speedplan_config_;
  CarModel car_model_;
	/************input***************/
  LocalizationData localization_;
  PerceptionData perception_;
	// TaskContent task_content_;
	VehicleDriveStatus drivestatus_;
  MapEngineData mapengine_;
  PredictionData prediction_;
  //add by ly
  PredictionData v2v_prediction_;
  PerceptionData v2v_perception_;
	/*********behaviorplan***********/
  DecisionInfo behavior_;
  /***********pathplan*************/
  PathData motionpath_;

  acu::vectormap::GeoTool geotool_;
  ObjectProcess object_process_;
  TrafficLightMatching light_matching_;
  PredictionProcess prediction_process_;

  vector<string> last_current_lanes_;
  vector<string> last_target_lanes_;
  ReferenceLineFrame *target_line_ptr_;
  ReferenceLineFrame *current_line_ptr_;

  std::map<int, LineObject> allline_objects_;

  int correction_index_;//根据上一帧参考点匹配到的current车道的索引
  int target_index_;
  int start_counter_ = 0;
  double start_time_;
  double last_prediction_time_;
  double local_path_length_ = 0.0;
  vector<StructTrafficlightInfo> matched_lights_;
  float compensate_s_in_unit_m_ = 0.0, compensate_t_in_unit_s_ = 0.0;
};

} // namespace planning
} // namespace acu


#endif
