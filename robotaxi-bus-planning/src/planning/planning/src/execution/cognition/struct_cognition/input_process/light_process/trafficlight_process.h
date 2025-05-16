#include "locperception_input.h"
#include "mapengine_input.h"
#include "decision_typedef.h"
//#include "planning_debug_msgs/DebugLight.h"
#include "planning_debug_msgs.pb.h"
namespace acu{
namespace planning {


class TrafficLightProcess {
 public:
	void ChooseFocusLight(const vector<StructTrafficlightInfo> &history_lights,
						  const DecisionInfo &behavior,
						  const PathData &motion_path, 
						  const double car_speed, 
						  MapEngineLineList &input_line);
	void SpeedLimit(MapEngineLineList &input_line, double &junction_speed);

 public:	
	
 private:
 	bool MapAndPerceptionLight(const vector<StructTrafficlightInfo> &history_lights, MapEngineLineList &input_line);
 	bool MapAndCalmcarPerceptionLight(const vector<StructTrafficlightInfo> &history_lights, MapEngineLineList &input_line);
	void GetLightLaneIds(MapEngineLineList &input_line);

 private:
 	double timestamp_ = 0.0;
 	bool response_area_light_ = true;
 	int jc_light_state_ = 0; // 0-nowaiting_area 1-beforejunction 2-inarea 
 	vector<StructTrafficlightInfo> history_lights_;
 	vector<string> light_lane_ids_; 
 	StructTrafficlightInfo last_light;//add by ly 0209
};

}
}