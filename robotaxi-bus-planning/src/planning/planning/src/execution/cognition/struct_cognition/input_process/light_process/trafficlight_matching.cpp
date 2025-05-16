#include "trafficlight_matching.h"

namespace acu{
namespace planning {

void TrafficLightMatching::MatchTrafficLights(const PerceptionData& perception_data,
											  											const LocalizationData &localization,
											  											vector<StructTrafficlightInfo> &matched_lights) {	
	auto &perception_lights = perception_data.traffic_lights;
	if (perception_lights.empty()) {
		matched_lights.clear();
		return;
	}
	//matched_lights.clear();
	if (matched_lights.empty()) {
		for (auto &per_light : perception_lights) {
			StructTrafficlightInfo new_light;
			new_light.SetNewLight(per_light, perception_data.timestamp);
    	matched_lights.push_back(new_light);
		}
		return;
	}

	for (int i = 0; i < matched_lights.size();i++) {// 去除感知没给的灯
		bool disappear_flag = true;
		for (auto &light : perception_lights) {
			if (light.id == matched_lights.at(i).light_id) {
				disappear_flag = false;
				break;
			}
		}
		if (disappear_flag) {
			matched_lights.erase(matched_lights.begin()+i);
			i--;
		}
	}
	for (auto &per_light : perception_lights) {
		bool have_history = false;
		int index = -1;
		for (int i = 0; i < matched_lights.size(); i++) {
			if (matched_lights.at(i).light_id == per_light.id) {
				have_history = true;
				index = i;
				break;
			}
		}
		if (have_history && index >= 0) {
			// AERROR_IF(FLAGS_log_enable)<<"old light";
			Convert(per_light, matched_lights.at(index), perception_data.timestamp);
		} else {
			// AERROR_IF(FLAGS_log_enable)<<"new light";
			StructTrafficlightInfo new_light;
			new_light.SetNewLight(per_light, perception_data.timestamp);
    	matched_lights.push_back(new_light);
		}
	}	
}

void TrafficLightMatching::Convert(const TrafficLight& perception_light, 
								  								 StructTrafficlightInfo &output_light,
								  								 double timestamp) {
	output_light.time_stamp = timestamp;
	output_light.light_id = perception_light.id;
	//add by ly for v2x traffic light 
	output_light.is_v2x_traffic_light = perception_light.is_v2x_traffic_light; 
   output_light.red_period = perception_light.red_period;
   output_light.green_period = perception_light.green_period;
   output_light.yellow_period = perception_light.yellow_period;
	//
	if (output_light.history_light.empty() || 
		output_light.history_light.back().color != (int)perception_light.color) {
		StructLightInfo new_light_info;
    	new_light_info.color = (int)perception_light.color;
    	new_light_info.state = (int)perception_light.state;
    	new_light_info.type = (int)perception_light.type;
    	new_light_info.time = 0.1;
    	if (perception_light.number >= 1e-3) {
    	  new_light_info.left_time = perception_light.number;
    	}
    	output_light.history_light.push_back(new_light_info);
    	return;
	}
	output_light.history_light.back().time += 0.1;
	if (perception_light.number > 1e-3) {
		output_light.history_light.back().left_time_delay_cnt = 0;
		if (output_light.history_light.back().left_time < 1e-3) {
			output_light.history_light.back().left_time = perception_light.number;
		} else if (ceil(output_light.history_light.back().left_time) == perception_light.number) {
			output_light.history_light.back().left_time -= 0.1;
			output_light.history_light.back().left_time = fmax(output_light.history_light.back().left_time, 0.1);
		} else {
			output_light.history_light.back().left_time = perception_light.number;
		}
   }else {
  	   if ( output_light.history_light.back().left_time_delay_cnt < 20 ) {
            output_light.history_light.back().left_time_delay_cnt += 1;
      } else {
        	  output_light.history_light.back().left_time_delay_cnt = 0;
           output_light.history_light.back().left_time = -1.0;
      }
   }
}



}
}