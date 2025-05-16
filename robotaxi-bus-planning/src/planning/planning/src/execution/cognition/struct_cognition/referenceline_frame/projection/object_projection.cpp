#include "object_projection.h"
#include "map/vectormap/src/hdmap/hdmap.h"

using namespace acu::vectormap;
using namespace acu::hdmap;

namespace acu{
namespace planning {

ObjectProjection::ObjectProjection(ReferenceLineFrame& reference_line, 
																	 const std::map<int, LineObject> &all_objects,
																	 const double &local_path_length,
																	 const LocalizationData &locpose, 
																	 const CarModel &car_input) {
	vectormap_ = map::MapLoader::GetVectorMapPtr();
	objects_ptr_ = &all_objects;
	reference_line_ptr_ = &reference_line;
	car_model_ptr_ = &car_input;
	loc_ptr_ = &locpose;
	local_path_length_ = local_path_length;
}

ObjectProjection::~ObjectProjection() {}

void ObjectProjection::AddObjectToFrame() {
	if (reference_line_ptr_->mapinfo.path_points.size() < 3) return;
	for (auto it = objects_ptr_->begin(); it != objects_ptr_->end(); it++) {
		const auto &object = it->second;
		StructSLBoundary sl_boundary;
		if (!GetSLBoundary(reference_line_ptr_->mapinfo, object, sl_boundary)) {
			reference_line_ptr_->objects_.erase(object.id);
			AINFO_IF(FLAGS_log_enable)<<object.id<<" GetSLBoundary failed.";
			continue;
		}
		if (object.is_static && 
				(sl_boundary.max_l < -1.0 * FLAGS_frenet_perception_range || 
				 sl_boundary.min_l > FLAGS_frenet_perception_range || 
				 sl_boundary.max_s < -1.0 * FLAGS_s_range || 
				 sl_boundary.min_s > FLAGS_s_range)) {
			if (reference_line_ptr_->objects_.count(object.id)) {
				reference_line_ptr_->objects_.erase(object.id);
			}
			continue;
		}
		double max_length = fmax(local_path_length_, reference_line_ptr_->mapinfo.dis2missionpoint);
		if (object.is_static && sl_boundary.min_s > max_length + FLAGS_front_buff) {
			if (reference_line_ptr_->objects_.count(object.id)) {
				reference_line_ptr_->objects_.erase(object.id);
			}
			continue;
		}
		if (reference_line_ptr_->objects_.count(object.id)) {
			reference_line_ptr_->objects_.at(object.id).SetData(object, sl_boundary);
		} else {
			reference_line_ptr_->objects_[object.id] = object;
			reference_line_ptr_->objects_.at(object.id).sl_boundary = sl_boundary;
		}
		if (reference_line_ptr_->mapinfo.distance_to_junctions.empty() || 
				reference_line_ptr_->mapinfo.distance_to_junctions.front().first > 70.0 ||
				reference_line_ptr_->mapinfo.distance_to_junctions.front().first > 0.0 &&
				reference_line_ptr_->mapinfo.global_cost >= 10) { 
			if (reference_line_ptr_->reference_lane_id < 40 && 
					loc_ptr_->velocity * 0.7 + 10.0 < sl_boundary.min_s &&
					reference_line_ptr_->objects_.count(object.id)) {
				ExpandPedestrian(reference_line_ptr_->objects_.at(object.id));
			}
		}
		if (reference_line_ptr_->objects_.count(object.id)) {
			ObjectSpeedTransform(*reference_line_ptr_, reference_line_ptr_->objects_.at(object.id));
		}
		PredictionToSL(reference_line_ptr_->mapinfo, reference_line_ptr_->objects_.at(object.id));
		PredictionTrajectorysCutOff(reference_line_ptr_->mapinfo, 
			                           reference_line_ptr_->objects_.at(object.id));
	}
	// 相信感知能够做障碍物保持，所以障碍物id没了就删除了，不做衰减
	for (auto it = reference_line_ptr_->objects_.begin(); 
						it != reference_line_ptr_->objects_.end(); ) {
		if (!objects_ptr_->count(it->first)) {
			reference_line_ptr_->objects_.erase(it++);
			continue;
		}
		++it;
	}
	if (reference_line_ptr_->reference_lane_id < 50) {
		FrameFocus frame_focus;
		frame_focus.FindFocusObstacles(*reference_line_ptr_, car_model_ptr_);// 根据障碍物投影id筛选关注障碍物
	}
	for (auto it = reference_line_ptr_->objects_.begin(); 
							it != reference_line_ptr_->objects_.end(); it++) {
		if (reference_line_ptr_->reference_lane_id < 50) {
			IsParallelObject(it->second, car_model_ptr_);
		}
		if (FLAGS_log_enable) {
			AERROR<<"line "<<reference_line_ptr_->reference_lane_id
							<<", obj "<<it->second.id<<setprecision(4)
							<<" s ("<<it->second.sl_boundary.min_s
							<<","<<it->second.sl_boundary.max_s
							<<"), l ("<<it->second.sl_boundary.min_l
							<<","<<it->second.sl_boundary.max_l
							<<"), focus "<<it->second.need_focus
							<<", key focus "<<it->second.key_focus
							<<", vsabs "<<it->second.vsabs
							<<", is_waiting "<<it->second.is_waiting;
		} 
	}
}

void ObjectProjection::ExpandPedestrian(LineObject& object) {
	auto& sl = object.sl_boundary;
	if (object.is_static && object.type == 2 && sl.max_s > 0.0) {
		double l_width, r_width, car_width = 
				DataPool::Instance()->GetMainDataRef().config_info.car_model.car_width;
		reference_line_ptr_->GetWidthToLaneBoundary(l_width, r_width, sl.min_s);
		if (l_width - sl.max_l > car_width + 0.45 && 
				sl.max_l > -0.5 * car_width - 0.8) {
			double history = sl.max_l;
			sl.max_l = std::min(sl.max_l + 0.8, l_width - car_width - 0.45);
			sl.max_l = l_width - car_width - 0.45;
			reference_line_ptr_->GetWidthToLaneBoundary(l_width, r_width, sl.max_s);
			sl.max_l = std::min(sl.max_l, l_width - car_width - 0.45);
			AERROR_IF(FLAGS_log_enable) << "expand left = " << 
					sl.max_l - history << " max_l = " << sl.max_l;
		}
		if (r_width + sl.min_l > car_width + 0.45 && 
				sl.min_l < 0.5 * car_width + 0.8) {
			double history = sl.min_l;
			sl.min_l = std::max(sl.min_l - 0.8, car_width + 0.45 - r_width);
			reference_line_ptr_->GetWidthToLaneBoundary(l_width, r_width, sl.max_s);
			sl.min_l = std::max(sl.min_l, car_width + 0.45 - r_width);
			AERROR_IF(FLAGS_log_enable) << "expand right = " << 
					history - sl.min_l << " min_l = " << sl.min_l;
		}
	}
}

void ObjectProjection::AddFrameObjectInfo() {
	FindOnLinePedestrian(); 
	FindOnLineCar();
	FindOnLineInvader();
	auto& line_objects = reference_line_ptr_->objects_;
	for (auto it = line_objects.begin(); it != line_objects.end(); it++) {
		AddSemanticInfo(it->second);
		if (it->second.conflict_type > 0) {
			AERROR_IF(FLAGS_log_enable)<<"obj "<<it->second.id
					<<" conflict_type "<<it->second.conflict_type
					<<" right_of_way "<<it->second.right_of_way;
		}
		AddFirstConflictBlockInfo(it->second);
	}
}

void ObjectProjection::FindOnLinePedestrian() {
	auto& object_on_line = reference_line_ptr_->object_on_line_;
	std::set<int> target_id;
	for (auto &it : reference_line_ptr_->objects_) {
		auto &object = it.second;
		if (object.type < 2 || object.type > 3 || object.is_static) {
			continue;
		}
		target_id.insert(object.id);
		if (object_on_line.count(object.id) == 0) {
			object_on_line[object.id] = 0;
		}
		if (object.sl_boundary.min_l < 0.5 * FLAGS_lane_width && 
				object.sl_boundary.max_l > -0.5 * FLAGS_lane_width) {
			object_on_line.at(object.id) += 1;
		} else {
			object_on_line.at(object.id) = 0;
		}		
	}
	for (auto it = object_on_line.begin(); it != object_on_line.end(); ) {
		if (target_id.count(it->first)) {
			++it;
		} else {
			object_on_line.erase(it++);
		}
	}
}

void ObjectProjection::FindOnLineInvader() {
	auto& invader_on_line = reference_line_ptr_->invader_on_line_;
	std::set<int> invader_id;
	for (auto &it : reference_line_ptr_->objects_) {
		auto &object = it.second;
		if (object.is_static || object.speed > 5.0 || 
				object.sl_boundary.max_s < car_model_ptr_->front_over_hang || 
				object.sl_boundary.min_l > 0.5 * car_model_ptr_->car_width + 0.6 || 
				object.sl_boundary.max_l < -0.5 * car_model_ptr_->car_width - 0.6) {
			continue;
		}
		double left_w, right_w;
    reference_line_ptr_->GetWidthToLaneBoundary(left_w, right_w, object.sl_boundary.min_s);
    // AERROR << "pass_l = " << left_w - object.sl_boundary.max_l - car_model_ptr_->car_width;
    // AERROR << "pass_l = " << car_model_ptr_->car_width - object.sl_boundary.min_l - right_w;
    if (left_w - object.sl_boundary.max_l > car_model_ptr_->car_width || 
    		object.sl_boundary.min_l + right_w > car_model_ptr_->car_width) {
    	double pass_l = left_w - object.sl_boundary.max_l > car_model_ptr_->car_width ? 
    			left_w - object.sl_boundary.max_l - car_model_ptr_->car_width : 
    			car_model_ptr_->car_width - object.sl_boundary.min_l - right_w;
    	invader_id.insert(object.id);
    	if (invader_on_line.count(object.id)) {
    		// AERROR << "pass_l = " << object.pass_l;
    		invader_on_line.at(object.id) += 1;
    		object.pass_l = (object.pass_l + pass_l) / 2.0;
    		object.pass_l = pass_l;
    	} else {
    		invader_on_line[object.id] = 1;
    		object.pass_l = pass_l;
    	}
    }
	}
	for (auto it = invader_on_line.begin(); it != invader_on_line.end();) {
		if (invader_id.count(it->first)) {
			++it;
		} else {
			invader_on_line.erase(it++);
		}
	}
}

void ObjectProjection::FindOnLineCar() {
	auto& car_on_line = reference_line_ptr_->car_on_line_;
	std::set<int> target_id;
	for (auto &it : reference_line_ptr_->objects_) {
		auto &object = it.second;
		if (object.type > 1) {
			continue;
		}
		target_id.insert(object.id);
		if (car_on_line.count(object.id) == 0) {
			car_on_line[object.id] = 0;
		}
		if (object.sl_boundary.min_l < 0.5 * FLAGS_lane_width && 
				object.sl_boundary.max_l > -0.5 * FLAGS_lane_width) {
			car_on_line.at(object.id) += 1;
		} else {
			car_on_line.at(object.id) = 0;
		}		
	}
	for (auto it = car_on_line.begin(); it != car_on_line.end(); ) {
		if (target_id.count(it->first)) {
			++it;
		} else {
			car_on_line.erase(it++);
		}
	}
}

void ObjectProjection::AddSemanticInfo(LineObject& object) {
	if (reference_line_ptr_->reference_lane_id == 40) {
		return;
	}
	object.conflict_type = 0; // 重置
	for (auto it = object.st_area.pd_objs.begin(); it != object.st_area.pd_objs.end(); it++) {
		auto &pd_obj = *it;
		if (object.type > 1 && object.type < 4 && object.st_area.pd_objs.size() == 1 && 
			pd_obj.st.size() > 0 && !object.is_static) {
			pd_obj.right_of_way = 1;
		}
		if (object.is_static) {
			pd_obj.conflict_type = pd_obj.st.size() ? 1 : 0;
			continue;
		}
		if (pd_obj.prediction_index >= object.prediction.trajectories.size() || 
				pd_obj.prediction_index < 0) {
			AERROR_IF(FLAGS_log_enable) <<object.id<<" Failed to get valid prediction trajectory!";
			pd_obj.conflict_type = 0;
			continue;
		}
		auto& prediction = object.prediction.trajectories.at(pd_obj.prediction_index);
		bool match_flag = false;
		for (int i = 0; i < prediction.lane_ids.size(); i++) {
			//if (object.type == 2 || object.type == 3) {
			if (object.type == 2) {
				break;
			}
			match_flag = false;
			for (auto lane : reference_line_ptr_->mapinfo.front_lane_ids) {
				if (prediction.lane_ids.at(i) == lane) {
					pd_obj.conflict_type = GetConflictType(prediction);
					pd_obj.right_of_way = GetRightOfWay(object, prediction, pd_obj);
					match_flag = true;
					break;
				}
			}
			if (match_flag) break;
			match_flag = false;
			for (auto lanes : reference_line_ptr_->mapinfo.back_lane_idss) {
				for (auto lane : lanes) {
					if (prediction.lane_ids.at(i) == lane) {
						pd_obj.conflict_type = GetConflictType(prediction);
						pd_obj.right_of_way = GetRightOfWay(object, prediction, pd_obj);
						match_flag = true;
						break;
					}
				}
			}
			if (match_flag) break;
		}
		if (!match_flag && pd_obj.st.size() > 0) {
			//if (object.type == 2 || object.type == 3) {
			if (object.type == 2) {
				pd_obj.right_of_way = 1;
				pd_obj.conflict_type = 5;
				int min_t = 100;
				int max_t = 0;
				for (auto &st_point : pd_obj.st) {
					if (st_point.t < min_t && st_point.p > FLAGS_p_threshold) {
						min_t = st_point.t;
					}
					if (st_point.t > max_t && st_point.p > FLAGS_p_threshold) {
						max_t = st_point.t;
					}
				}
				if (min_t < (int)(1.0 / FLAGS_scale_t) && max_t > (int)(7.0 / FLAGS_scale_t) || 
						reference_line_ptr_->object_on_line_.count(object.id) && 
						reference_line_ptr_->object_on_line_.at(object.id) > 50) {
					pd_obj.conflict_type = 1;
				}
			} else {// 预测线id和自车车道id都不相同，判断是否平行
				int parallel_index = 0;
				if ( (object.type < 2 || 3 == object.type) && IsParallelConflict(prediction, parallel_index)) {
					if (object.sl_boundary.max_s < car_model_ptr_->front_over_hang) {
						AINFO_IF(FLAGS_log_enable)<<object.id<<"Back car parallel conflict.";
						pd_obj.conflict_type = 1;
					} else if (parallel_index == 0) {
						AINFO_IF(FLAGS_log_enable)<<object.id<<"Front car parallel conflict.";
						pd_obj.conflict_type = 4;
					}
				} else {
						pd_obj.conflict_type = 5;
						//add by ly 如果前方目标车的前继车道和当前参考线的车道序列重合，冲突类型判断为1 
						double heading_diff = fabs(loc_ptr_->yaw - object.global_angle);
						if (heading_diff > 180.0){
							heading_diff = 360.0 - heading_diff; 
						}
						if (prediction.lane_ids.size() > 0 && 
							object.sl_boundary.min_s > car_model_ptr_->front_over_hang && 
							object.type < 2 &&  heading_diff < 45.0 ) {
							Id Id_lane;
							Id_lane.set_id(prediction.lane_ids.front());
							std::vector<Id> ids_prede;
							int ids_prede_size = vectormap_->GetLanePredecessorIDs(Id_lane, ids_prede);
							if (ids_prede_size > 0) {
								bool find_overlap_lane = false;
								for (auto pre_id : ids_prede) {
									for (auto lane : reference_line_ptr_->mapinfo.front_lane_ids) {
										if (lane == pre_id.id()) {
											AINFO_IF(FLAGS_log_enable)<<object.id<<"Front car in divided lane conflict.";
											pd_obj.conflict_type = 1;
											find_overlap_lane = true;
											break;
										}
									}
									if (find_overlap_lane) break;
								}
							} 
						}
						//end
				}
				std::vector<std::string> lane_ids;
				for (int i = 0; i < reference_line_ptr_->mapinfo.front_lane_ids.size(); i++) {
					if (reference_line_ptr_->mapinfo.distance_to_ends.size() > i && 
							reference_line_ptr_->mapinfo.distance_to_ends[i].first < 60.0) {
						lane_ids.push_back(reference_line_ptr_->mapinfo.front_lane_ids[i]);
					} else {
						break;
					}
				}
				int ego_type = GetJunctionType(lane_ids);
				int object_type = GetJunctionType(prediction.lane_ids);
				pd_obj.right_of_way = ego_type < object_type ? 0 : 1;
			}
		}
		else if (!match_flag) {
			AERROR<<"pd st points size 0.";
			pd_obj.conflict_type = 0;
		}
		if (pd_obj.conflict_type > 3) {
			pd_obj.right_of_way = JunctionRightOfWay(object, prediction, pd_obj);
		}
	}

	for (auto& pd : object.st_area.pd_objs) {
    if (pd.conflict_type > 0 && 
    		(object.conflict_type == 0 || pd.conflict_type < object.conflict_type)) {
      object.conflict_type = pd.conflict_type;
      object.right_of_way = pd.right_of_way;
    }
  }
}

int ObjectProjection::GetConflictType(PredictionTrajectory &prediction) {
	for (auto lane : reference_line_ptr_->mapinfo.front_lane_ids) {
		if (lane == prediction.lane_ids.front()) {
			return 1;
		}
	}
	for (auto lanes : reference_line_ptr_->mapinfo.back_lane_idss) {
		for (auto lane : lanes) {
			if (lane == prediction.lane_ids.front()) {
				return 1;
			} 
		}
	}
	for (auto lanes : reference_line_ptr_->mapinfo.back_lane_idss) {
		for (auto lane : lanes) {
			for (auto prediction_lane : prediction.lane_ids) {
				if (prediction_lane == lane) {
					return prediction.intentbylane == "Keep Current" ? 1 : 3;
				}
			}
		}
	}
	return prediction.intentbylane == "Keep Current" ? 2 : 3;
}

int ObjectProjection::GetRightOfWay(const LineObject& object, 
																		const PredictionTrajectory &prediction, 
																		PdObjectStruct &pd_obj) {
	if (pd_obj.conflict_type == 1) {
		return reference_line_ptr_->reference_lane_id < 20 && object.sl_boundary.min_s < 0.0 ? 0 : 1;
	} else if (pd_obj.conflict_type == 2) {
		std::vector<std::string> lane_ids;
		for (int i = 0; i < reference_line_ptr_->mapinfo.front_lane_ids.size(); i++) {
			if (reference_line_ptr_->mapinfo.distance_to_ends.size() > i && 
					reference_line_ptr_->mapinfo.distance_to_ends[i].first < 60.0) {
				lane_ids.push_back(reference_line_ptr_->mapinfo.front_lane_ids[i]);
			} else {
				break;
			}
		}
		int type_1 = GetJunctionType(lane_ids);
		int type_2 = GetJunctionType(prediction.lane_ids);
		AERROR_IF(FLAGS_log_enable) << "ego = " << type_1 << " pd_obj = " << type_2;
		return type_1 < type_2 ? 0 : 1;
	} else {
		return 0;
	}
	return 0;
}

int ObjectProjection::JunctionRightOfWay(const LineObject& object, 
																				 const PredictionTrajectory &prediction, 
																					PdObjectStruct &pd_obj) {
	vector<Id> pd_lanes, reference_lanes;
	for (auto &lane : prediction.lane_ids) {
		Id Id_lane;
		Id_lane.set_id(lane);
		pd_lanes.push_back(Id_lane);
	}
	int index = 0;
	for (auto &lane : reference_line_ptr_->mapinfo.front_lane_ids) {
		Id Id_lane;
		Id_lane.set_id(lane);
		reference_lanes.push_back(Id_lane);
		index++;
		if (index < reference_line_ptr_->mapinfo.distance_to_ends.size() &&
				reference_line_ptr_->mapinfo.distance_to_ends.at(index).second > 
				FLAGS_front_perception_range) break;
	}
	int junction_relation = -1, relation = -1, right_of_way = 1;
	CollisionIndex pd_index, reference_index;
	if (vectormap_->GetLanesRelvant(reference_lanes, pd_lanes, junction_relation, relation, 
									pd_index, reference_index, right_of_way) < 0 || right_of_way < 0) {
		return 1;
	}
	AERROR_IF(FLAGS_log_enable)<<"obj "<<object.id<<" relation "<<relation<<" reference ("
				<<reference_index.start_s<<", "<<reference_index.end_s<<"), right_of_way "<<right_of_way;
	return right_of_way;
}

bool ObjectProjection::IsParallelConflict(PredictionTrajectory &prediction, int &index) {
	index = 0;
	for (auto &pd_lane : prediction.lane_ids) {
		for (auto lane : reference_line_ptr_->mapinfo.front_lane_ids) {
			if (LaneToRoad(lane) == LaneToRoad(pd_lane)) {
				return true;
			} else {
				for (auto &relation_lane : reference_line_ptr_->mapinfo.front_relation_lanes) {
					if (relation_lane.lane_id != pd_lane) continue;
					if (relation_lane.type == RelationType::CROSS ||
							relation_lane.type == RelationType::CROSSPRE) {
						return false;
					}
				}
			}
		}
		index++;
	}
	return false;
}

int ObjectProjection::GetRightOfWay(LineObject& object, const PredictionTrajectory &prediction) {
	if (object.conflict_type == 1) {
		return reference_line_ptr_->reference_lane_id < 20 && object.sl_boundary.min_s < 0.0 ? 0 : 1;
	} else if (object.conflict_type == 2) {
		std::vector<std::string> lane_ids;
		for (int i = 0; i < reference_line_ptr_->mapinfo.front_lane_ids.size(); i++) {
			if (reference_line_ptr_->mapinfo.distance_to_ends.size() > i && 
					reference_line_ptr_->mapinfo.distance_to_ends[i].first < 60.0) {
				lane_ids.push_back(reference_line_ptr_->mapinfo.front_lane_ids[i]);
			} else {
				break;
			}
		}
		int type_1 = GetJunctionType(lane_ids);
		int type_2 = GetJunctionType(prediction.lane_ids);
		AERROR_IF(FLAGS_log_enable) << "ego = " << type_1 << " pd_obj = " << type_2;
		return type_1 < type_2 ? 0 : 1;
	} else {
		return 0;
	}
}

int ObjectProjection::GetJunctionType(const std::vector<string>& lane_ids) {
	int junction_type = 0;
	for (auto& lane : lane_ids) {
		if (lane == "PEDESTRIAN" || lane == "CYCLIST"|| lane == "CAR" ||
				lane == "TRUCK" || lane == "UNKNOWN") continue;
		Id lane_id;
		lane_id.set_id(lane);
		Lane::LaneTurn turn_type;
		//auto monitor_api = acu::common::MonitorApi::Instance();
		if (vectormap_ == nullptr) {
			//monitor_api->SetFaultInfo(acu::common::NodeTime::Now().ToSecond(), 
						//DFPLANNING_PLANNING_SOFTWARE_10_ERROR,
    	              	//"MAP API WRONG: ObjectProjection::map ptr null");
		} 
		if(vectormap_->GetLaneTurn(lane_id, turn_type) != 0) {
			//monitor_api->SetFaultInfo(acu::common::NodeTime::Now().ToSecond(), 
						//DFPLANNING_PLANNING_SOFTWARE_10_ERROR,
    	              	//"MAP API WRONG:" + lane + " no turn type");
		}
		if ((int)turn_type > junction_type) {
			junction_type = (int)turn_type;
		}
	}
	return junction_type;
}

void ObjectProjection::AddFirstConflictBlockInfo(LineObject& object) {
	if (reference_line_ptr_->reference_lane_id == 40) {
		return;
	}
	object.block_first_conflict = false;
	for (auto &pd_obj : object.st_area.pd_objs) {
		if (pd_obj.st.empty() || pd_obj.prediction_index < 0 ||
			pd_obj.prediction_index >= object.prediction.trajectories.size()) {
			pd_obj.st.clear();
			continue;
		}

		auto &pd_line = object.prediction.trajectories.at(pd_obj.prediction_index);
		if (pd_line.st_boundary.size() >= 2) {
			double first_t = pd_line.st_boundary.front().first.y();
			double min_s = pd_line.st_boundary.front().first.x();
			// 与st map 过滤的时间保持一致
			if (first_t > 1.0) continue;

			// AINFO <<"[0]AddFirstConflictBlockInfo,ref_id:"<< reference_line_ptr_->reference_lane_id<<", obj:"<<object.id
			// 	  <<" first_t:"<< first_t<<" first_s:"<< min_s;
			for(auto& pd_point : pd_line.points) {
				if(pd_point.t >= first_t) {
					Site temp_site;
					double left_w, right_w;

					temp_site.set_g(pd_point.xg, pd_point.yg);
					Vec2d center(pd_point.xg, pd_point.yg);
			        double angle = pd_point.globalangle * M_PI / 180.0;
			        Box2d box(center, angle, object.box.length(), object.box.width());
			        StructSLBoundary sl_boundary;
			        BoxToSL(reference_line_ptr_->mapinfo, box, sl_boundary);

			        reference_line_ptr_->GetWidthToLaneBoundary(left_w, right_w, min_s);
			        double left_l = left_w - sl_boundary.max_l;
    				double right_l = sl_boundary.min_l + right_w;
    				// AINFO <<"left_l:" << left_l <<", right_l:"<< right_l;
    				if (left_l < car_model_ptr_->car_width + FLAGS_collision_buff + 0.2 && 
        				right_l < car_model_ptr_->car_width + FLAGS_collision_buff + 0.2) {
    					object.block_first_conflict = true;
    					return;
    				}
				} else {
					continue;
				}
			}
		}
	}

}

void ObjectProjection::PredictionTrajectorysCutOff(const MapEngineLineList &map_info, 
	                                                   LineObject &temp_obj) 
{
	if (map_info.isolationbelts.empty()) return;
	if (temp_obj.prediction.trajectories.empty()) return; 
	vector<Vec2d> isolationbelt_points;
	for (auto isolat : map_info.isolationbelts) {
		  isolationbelt_points.clear();
      for (auto point : isolat) {
      	Vec2d temp_point(point.first, point.second);
        isolationbelt_points.push_back(temp_point);
      }
      //构建单个隔离带的凸包
      Polygon2d isolationbelt_polygon;
      if (Polygon2d::ComputeConvexHull(isolationbelt_points, &isolationbelt_polygon)){
        //构建前后两个预测轨迹点的线段
      	for (auto &pd_trajectory : temp_obj.prediction.trajectories) {
      		for (size_t i = 0 ; (i + 1) < pd_trajectory.points.size() ; i++) {
      			LineSegment2d pd_line_seg(Vec2d(pd_trajectory.points[i].xg,pd_trajectory.points[i].yg),
      				                        Vec2d(pd_trajectory.points[i+1].xg,pd_trajectory.points[i+1].yg));
      			//判断凸包和线段是否重叠
      			if (isolationbelt_polygon.HasOverlap(pd_line_seg)) {
      				size_t j = i;
      				for (j = i; j > 0; j--) { //多截断出来车身长度
      					if (pd_trajectory.points[i].x - pd_trajectory.points[j].x > temp_obj.box.length() + 2.0) {
                   break;
      					}
      				}
      				pd_trajectory.points.erase(pd_trajectory.points.begin() + j, pd_trajectory.points.end());
      				AINFO_IF(FLAGS_log_enable)<< "cutoff pd_trajectory obj id "<< temp_obj.id;
      				break;
      			}
      			
      		}
      	}
	    }
	}


}


}
}
