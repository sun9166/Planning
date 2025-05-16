#include "trafficlight_process.h"

namespace acu{
namespace planning {
	
void TrafficLightProcess::ChooseFocusLight(const vector<StructTrafficlightInfo> &history_lights,
																					 const DecisionInfo &behavior,
																					 const PathData &motion_path,
																					 const double car_speed, 
																					 MapEngineLineList &input_line) {
	if (FLAGS_use_calmcar_tfl) {
		MapAndCalmcarPerceptionLight(history_lights, input_line);
		return;
	}

	if (!MapAndPerceptionLight(history_lights, input_line)) return;
	GetLightLaneIds(input_line);
	if (input_line.all_trafficlights.size() == 1) {
		if (jc_light_state_ == 0 || response_area_light_ || FLAGS_waiting_in_junction) {
			input_line.trafficlight = input_line.all_trafficlights.front();
		}
		return;
	}
	auto &unkey_light = input_line.all_trafficlights.front();// 路口灯
	auto &area_light = input_line.all_trafficlights.back();// 待转区灯
	if (area_light.history_light.empty()) {
		// AERROR<<"Can't detection area light color.";
		input_line.trafficlight = unkey_light;// 感知没给待转区，看非待转区灯
		return;
	} 
	if (unkey_light.history_light.empty()) {
		// AERROR<<"Can't detection unkey light color.";
		input_line.trafficlight = area_light;// 感知没给非待转区，看待转区灯
		return;
	}
	int unkey_color = unkey_light.history_light.back().color;
	int area_color = area_light.history_light.back().color;
	AERROR_IF(FLAGS_log_enable)<<"unkey_color "<<unkey_color<<" area_color "<<area_color;
	double area_start_s = 0.0, area_end_s = 0.0;
	for (int i = 0; i < input_line.lane_types.size(); i++) {
		if (input_line.lane_types.at(i).second == eLaneType::WAITINGLEFT &&
				input_line.distance_to_ends.size() > i) {
			area_start_s = input_line.distance_to_ends.at(i).first;
			area_end_s = input_line.distance_to_ends.at(i).second;
		}
	}
	if (unkey_color == 3 && area_color == 1) {
		input_line.trafficlight = area_light;
		response_area_light_ = true;
	} else if (unkey_color == 2 && area_color == 1) {
		response_area_light_ = true;
		if (car_speed * car_speed < 2.0 * FLAGS_recommended_planning_dec * area_start_s) {
			input_line.trafficlight = unkey_light;
		} else if (unkey_light.history_light.size() > 0) {
			double yellow_time_value = unkey_light.history_light.back().left_time;
			bool getin_junction = (area_start_s < 0.5);
			for (auto &motion_point : motion_path.path) {
				if (motion_point.t > yellow_time_value - unkey_light.history_light.back().time) {
					getin_junction = (motion_point.length > area_start_s - 0.1 * car_speed);
					break;
				}
			}
			double start_v = 100.0, sim_a = 0.0;
			for (auto &motion_point : motion_path.path) {
				if (motion_point.length - area_start_s > 1e-3) {
					start_v = motion_point.velocity;
					break;
				}
			}
			sim_a = 0.5 * start_v * start_v / (area_end_s - area_start_s);
			unkey_color = (!getin_junction || sim_a > FLAGS_self_dec)? 1 : unkey_color;
			input_line.trafficlight = (!getin_junction || sim_a > FLAGS_self_dec)? unkey_light : area_light;
			AINFO_IF(FLAGS_log_enable)<<"start_v "<<start_v<<" sim_a "<<sim_a<<" getin_junction "<<getin_junction;
		}
	} else if (unkey_color == 1 && area_color == 1) {
		response_area_light_ = true;
		input_line.trafficlight = unkey_light;
	} else if (unkey_color == 1 && area_color == 3) {
		response_area_light_ = false;
		input_line.trafficlight = area_light;
	} else if (unkey_color == 1 && area_color == 2) {
		if (area_light.history_light.size() > 0 &&
			area_light.history_light.back().time < 0.3) {// 先尝试响应area，看下规划结果
			input_line.trafficlight = area_light;
		} else {// 已经响应arealight,判定是否能经过unkeylight
			if (car_speed * car_speed < 2.0 * FLAGS_recommended_planning_dec * area_start_s) {
				input_line.trafficlight = unkey_light;
			} else {
				double yellow_time_value = area_light.history_light.back().left_time;
				bool getin_junction = (area_start_s < 0.5);
				for (auto &motion_point : motion_path.path) {
					if (motion_point.t > yellow_time_value - area_light.history_light.back().time) {
						getin_junction = (motion_point.length > area_start_s - 0.1 * car_speed);
						break;
					}
				}
				input_line.trafficlight = (!getin_junction)? unkey_light : area_light;
			}
		}
	} else if (unkey_color == 1 && area_color == 1) {
		response_area_light_ = true;
		input_line.trafficlight = unkey_light;
	} else if( unkey_color == 0 || area_color == 0 || unkey_color == 4 || area_color == 4){ 
		input_line.trafficlight = last_light;
	}
    if( fabs(input_line.trafficlight.light_s - unkey_light.light_s) < 2.0 ){
  	  last_light = unkey_light;
    } else if(fabs(input_line.trafficlight.light_s - area_light.light_s) < 2.0){
  	  last_light = area_light;
    }
}

bool TrafficLightProcess::MapAndPerceptionLight(const vector<StructTrafficlightInfo> &history_lights, 
																								MapEngineLineList &input_line) {
	if (history_lights.empty() || input_line.all_trafficlights.empty()) return false;
	if (input_line.all_trafficlights.size() > 2) return false;
	for (auto &map_light : input_line.all_trafficlights) {
		for (const auto &light : history_lights) {
			if (light.light_id == map_light.light_id) {
				map_light.history_light = light.history_light;
				//add by ly for v2x traffic light 
				map_light.is_v2x_traffic_light = light.is_v2x_traffic_light ;
    			map_light.red_period = light.red_period ;
    			map_light.green_period = light.green_period;
    			map_light.yellow_period = light.yellow_period;
    			//
			}
		}
		for (const auto &light : history_lights) {
			for (auto &same_id : map_light.same_light_ids) {
				if (same_id != light.light_id || light.history_light.empty()) continue;
				if (light.history_light.back().color != 4 && light.history_light.back().color != 0) {
					if (map_light.history_light.empty()) {
						map_light.history_light.push_back(light.history_light.back());
					} else if (light.history_light.back().color < map_light.history_light.back().color 
						|| (0 == map_light.history_light.back().color && light.history_light.back().color > 0)) {
						map_light.history_light.back().color = light.history_light.back().color;
					}
					AINFO_IF(FLAGS_log_enable)<<"Matched same light "<<map_light.light_id<<" and "<<same_id
							 <<" color "<<map_light.history_light.back().color
							 <<" time "<<map_light.history_light.back().time;
				}
			}
		}
	}
	return true;
}

void TrafficLightProcess::GetLightLaneIds(MapEngineLineList &input_line) {
	if (light_lane_ids_.empty()) {
		for (int i = 1; i < input_line.lane_types.size(); i++) {
			if (input_line.lane_types.at(i).second == eLaneType::WAITINGLEFT &&
				input_line.lane_types.at(i-1).second == eLaneType::CITY_DRIVING_LANE &&
				input_line.all_trafficlights.size() > 1) {
				double s = input_line.lane_types.at(i-1).first;
				for (int j = 1; j < input_line.front_lane_ids.size(); j++) {
					if (input_line.distance_to_ends.size() > j &&
						fabs(input_line.distance_to_ends.at(j-1).second -s) < 0.1) {
						light_lane_ids_.push_back(input_line.front_lane_ids.at(j-1));
						light_lane_ids_.push_back(input_line.front_lane_ids.at(j));
						AINFO_IF(FLAGS_log_enable)<<"light_lane_ids_ "<<light_lane_ids_.front();
						AINFO_IF(FLAGS_log_enable)<<"light_lane_ids_ "<<light_lane_ids_.back();
						break;
					}
				}
			}
		}
		jc_light_state_ = (light_lane_ids_.size() == 2)? 1 : 0;
		response_area_light_ = (light_lane_ids_.size() == 2)? response_area_light_ : false;
		return;
	} else {
		bool waiting_flag = false;
		for (int i = 1; i < input_line.front_lane_ids.size(); i++) {
			if (input_line.front_lane_ids.front() == light_lane_ids_.back() &&
					input_line.all_trafficlights.size() < 2) {
				jc_light_state_ = 2;
				waiting_flag = true;
				break;
			}
			if (input_line.front_lane_ids.at(i-1) == light_lane_ids_.front() &&
				input_line.front_lane_ids.at(i) == light_lane_ids_.back() ||
				input_line.all_trafficlights.size() == 2) {
				jc_light_state_ = 1;
				waiting_flag = true;
				break;
			}
		}
		if (!waiting_flag) {
			jc_light_state_ = 0;
			light_lane_ids_.clear();
			response_area_light_ = false;
		}
	}
	AERROR_IF(FLAGS_log_enable)<<"light_lane_ids_ size "<<light_lane_ids_.size() 
			 											<<" jc_light_state_ "<<jc_light_state_ 
			 											<<" response_area_light_ "<<response_area_light_;
}

void TrafficLightProcess::SpeedLimit(MapEngineLineList &input_line, double &junction_speed) {
	if (input_line.trafficlight.light_id.empty() ||
		input_line.trafficlight.history_light.empty()) return;
	if (input_line.trafficlight.history_light.back().state != (int)eLightState::LENGTH_YELLOW) return;
	string junction_pre_lane = input_line.trafficlight.lane_id;
	double start_s = 1000.0, end_s = 1000.0;
	for (int i = 1; i < input_line.front_lane_ids.size(); i++) {
		if (input_line.front_lane_ids.at(i-1) == junction_pre_lane &&
			i < input_line.distance_to_ends.size()) {
			start_s = input_line.distance_to_ends.at(i).first;
			end_s = input_line.distance_to_ends.at(i).second;
		}
	}
	if (start_s > 200.0) return;
	for (int i = 0; i < input_line.expected_speeds.size(); i++) {
		if (input_line.expected_speeds.at(i).first < start_s + 0.1) continue;
		if (input_line.expected_speeds.at(i).second > junction_speed) {
			if (input_line.expected_speeds.at(i).first > end_s + 0.1) {
				std::pair<double, double> junction_pair(end_s, junction_speed);
				input_line.expected_speeds.insert(input_line.expected_speeds.begin()+i, junction_pair);
			} else {
				input_line.expected_speeds.at(i).second = junction_speed;
			}
		}
		break;
	}
}
//用感知灯信息更新参考线上灯信息，同时计算出停止线距离
bool TrafficLightProcess::MapAndCalmcarPerceptionLight(const vector<StructTrafficlightInfo> &history_lights, 
												       MapEngineLineList &input_line) {
	if (history_lights.empty() /*|| input_line.all_trafficlights.empty()*/) return false;
	if (input_line.all_trafficlights.size() > 2) return false;

    double dis_to_junc_start = 1000.0;
    double dis_to_junc_end   = 1000.0;
    if (input_line.distance_to_junctions.size()) {
    	dis_to_junc_start = input_line.distance_to_junctions.front().first;
    	dis_to_junc_end = input_line.distance_to_junctions.front().second;
    } 
    //接近路口时，判断路口内车道的弯道属性 1: no turn 2:left 3:right 4:U turn 
    int input_line_direction = 1;                            
    if (0.0 < dis_to_junc_start && dis_to_junc_start < 100.0) {
    	for (auto &lane_turn : input_line.lane_turns) {
    		AERROR_IF(FLAGS_log_enable) << "lane turn : " << lane_turn.second;
    		if (lane_turn.first > dis_to_junc_start - 1.0 && 
    			lane_turn.first < dis_to_junc_end + 1.0) {
			  input_line_direction  = max(lane_turn.second,input_line_direction);
    		} else if (lane_turn.first > dis_to_junc_end) {
    			break;
    		}
    	}
    }
    AINFO_IF(FLAGS_log_enable) <<"junction turn :" << input_line_direction;
    //直行：直行箭头灯 > 圆灯,左转:左转箭头灯 > 圆灯,右转：右转箭头灯,掉头：掉头灯 > 左转灯 > 圆灯
    //light type 0:UNKNOWN, 1:ARROW_STRAIGHT, 2:ARROW_LEFT,
    //           3:ARROW_RIGHT, 4:ARROW_UTURN, 5:CIRCLE,6:NUM 
    for (auto &light : history_lights) { 
	  if (light.history_light.size() && input_line_direction == (int)light.history_light.back().type) { //找到匹配的灯就终止
	  	input_line.trafficlight.history_light = light.history_light;
		break;
	  } 
	  if (light.history_light.size() && 1 == input_line_direction 
	                                 && 5 == (int)light.history_light.back().type) { //没到直行灯，默认看圆灯
	  	input_line.trafficlight.history_light = light.history_light;
	  } 
	}
    // if (0 == input_line_direction) {
	//     for (auto &light : history_lights) {
	// 	    if (light.history_light.size() &&  1 == (int)light.history_light.back().type) {
	// 	       input_line.trafficlight.history_light = light.history_light;
	// 		   break;
	// 	    } else if (5 == (int)light.history_light.back().type) {
	// 	       input_line.trafficlight.history_light = light.history_light;
	// 	    }
	//     }
    // } else if (1 == input_line_direction) {
	//     for (auto &light : history_lights) {
	// 	    if (2 == (int)light.history_light.back().type) {
	// 	       input_line.trafficlight.history_light = light.history_light;
	// 		   break;
	// 	    }  else if (5 == (int)light.history_light.back().type) {	    
	// 	       input_line.trafficlight.history_light = light.history_light;
	// 	    }
	// 	    input_line.trafficlight.history_light = light.history_light;
	// 	}

    // } else  if (2 == input_line_direction) {
	//     for (auto &light : history_lights) {
	// 	    if (3 == (int)light.history_light.back().type) {
	// 	       input_line.trafficlight.history_light = light.history_light;
	// 		   break;
	// 	    } 
	//     }
    // } else if (3 == input_line_direction) { 
	//     StructTrafficlightInfo tmp_light;
	//     for (auto &light : history_lights) {
	// 	    if (4 == (int)light.history_light.back().type) {
	// 	       tmp_light = light;
	// 		   break;
	// 	    } else if (3 == (int)light.history_light.back().type){
	// 	    	tmp_light = light;
	// 	    } else if ( 3 > (int)tmp_light.history_light.back().type && 2 == (int)light.history_light.back().type) {
	// 	    	tmp_light = light;
	// 	    } 

	//     }
	//     input_line.trafficlight.history_light = tmp_light.history_light;
    // }
    //
    input_line.trafficlight.light_s = dis_to_junc_start;
    if (input_line.trafficlight.history_light.size()) {
    	AINFO_IF(FLAGS_log_enable) << "trafficlight  color: " << input_line.trafficlight.history_light.back().color 
    							   << " ,type : "           <<  input_line.trafficlight.history_light.back().type
    	                           << " ,light s: "          << input_line.trafficlight.light_s;
    }							
	return true;
}


}
}