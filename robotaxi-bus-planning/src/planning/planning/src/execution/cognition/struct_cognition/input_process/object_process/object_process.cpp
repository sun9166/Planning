#include "object_process.h"
//#include "common/base/monitor_api/include/monitor_api.h"

using namespace acu::common::math;

namespace acu{
namespace planning {

ObjectProcess::ObjectProcess() {
	vector_map_ = map::MapLoader::GetVectorMapPtr();
	hdmap_ = map::MapLoader::GetHDMapImplPtr();
	if (vector_map_ == nullptr || hdmap_ == nullptr) {
		return;
	}
} 

void ObjectProcess::CellsToObjects(PerceptionData &perception_data) {
	auto DP = DataPool::Instance()->GetMainDataPtr();
	//if (DP->drive_status.shiftlvlposition != 1) return;
	//自车车速低于Flag_low_spd_thr时输出补盲cells点云目标
	if ( DP->cognition_info.struct_env_info.vehicle_info.localization.loc_velocity 
		  > FLAGS_low_spd_thr / 3.6 ) return;
	CallbackObject cell_object;
	int index = -1;
	double min_x = 100.0, min_y = 100.0, max_x = -100.0, max_y = -100.0;
	for (int i = 0; i < perception_data.cells.size(); i++) {
		auto &cell = perception_data.cells.at(i);
		bool same_obj = false;
		if (cell_object.cells.empty() ||
				cell.x > min_x - 0.5 && cell.x < max_x + 0.5 && 
				cell.y > min_y - 0.5 && cell.y < max_y + 0.5) {
			cell_object.cells.push_back(cell);
			min_x = fmin(cell.x, min_x);
			max_x = fmax(cell.x, max_x);
			min_y = fmin(cell.y, min_y);
			max_y = fmax(cell.y, max_y);
			same_obj = true;
		} 
		if (same_obj && i != perception_data.cells.size() - 1) continue;
		Box2d obj_box = CellsToBox(cell_object);
		cell_object.id = index;
		cell_object.xabs = obj_box.center_x();
  	cell_object.yabs = obj_box.center_y();
  	cell_object.width = obj_box.width();
  	cell_object.length = obj_box.length();
  	cell_object.speed = 0.0;
  	cell_object.type = (int)eObjectType::BLINDLIDAR;
  	// AINFO<<"Cell obj "<<cell_object.id<<" size "<<cell_object.cells.size();
  	perception_data.objects.push_back(cell_object);
		index--;
		min_x = 100.0, min_y = 100.0, max_x = -100.0, max_y = -100.0;
		cell_object.Reset();
		if (i != perception_data.cells.size() - 1) i--;
	}
}

void ObjectProcess::CompleteObjectInfo(vector<CallbackObject>& objects, 
																			 PredictionData &prediction, 
																			 std::map<int, LineObject> &lines_objects) {	
	prediction_ptr_ = &prediction.prediction_objects;
	AERROR_IF(FLAGS_log_enable)<<" input objets size "<< objects.size();
	for (auto it = lines_objects.begin(); it != lines_objects.end(); it++) {// 初始重置
		it->second.disapear_counter++;
	}
	for (auto &per_obj : objects) {// 更新或新建
		if (per_obj.x < -1.0 * FLAGS_front_perception_range || 
			per_obj.x > FLAGS_front_perception_range + 10.0) {
			continue;// 过滤范围以外的
		}
		if (per_obj.speed < 0.1 && per_obj.age < 7) {//过滤感知误检几帧的目标
			continue;
		}
		if (2 == per_obj.type && per_obj.speed > (15 / 3.6) && per_obj.age < 10) {// 过滤速度过快且周期短的行人
			continue;
		}
		if (lines_objects.count(per_obj.id) && per_obj.id >= 0) {
			UpdateObject(per_obj, lines_objects.at(per_obj.id));
		} else {
			LineObject new_object;
			UpdateObject(per_obj, new_object);
			lines_objects[per_obj.id] = new_object;
		}
	}
	for (auto it = lines_objects.begin(); it != lines_objects.end(); ) {// 清理没更新的
		if (it->second.disapear_counter > 1) {// 更改阈值可以控制消失时间
			lines_objects.erase(it++);
			continue;
		}
		++it;
	}
}

void ObjectProcess::UpdateObject(CallbackObject &per_obj, LineObject &line_obj) {
	line_obj.disapear_counter = 0;
	UpdatePerceptionInfo(per_obj, line_obj);
	if (per_obj.id < 0) return;
	UpdateMapAPIInfo(per_obj, line_obj);
	UpdateHistoryInfo(per_obj, line_obj);
}

void ObjectProcess::UpdatePerceptionInfo(CallbackObject &per_obj, LineObject &line_obj) {
	line_obj.id 					= per_obj.id;
	line_obj.x 						= per_obj.x;
	line_obj.y 						= per_obj.y;
	line_obj.xabs 				= per_obj.xabs;
	line_obj.yabs 				= per_obj.yabs;
	line_obj.global_angle = per_obj.global_angle;
	line_obj.vxabs 				= per_obj.vxabs;
	line_obj.vyabs 				= per_obj.vyabs;
	line_obj.speed 				= per_obj.speed;
	line_obj.height 			= per_obj.height;
	line_obj.confidence 	= per_obj.confidence;
	line_obj.age 					= per_obj.age;
	line_obj.box 					= GetBox(per_obj);
	line_obj.cells 				= per_obj.cells;
	if (per_obj.cells.size() > 2) {
		line_obj.cell_box = CellsToBox(per_obj);
	}
	if(per_obj.type == 7){
		line_obj.type = per_obj.type;
	}
}

void ObjectProcess::UpdateMapAPIInfo(CallbackObject &per_obj, LineObject &line_obj) {
	string pd_obj_lane = "";
	for (int i = 0; i < prediction_ptr_->size(); i++) {
		if (per_obj.id == prediction_ptr_->at(i).id) {
			if (!prediction_ptr_->at(i).obj_lane_id.empty()) {
				pd_obj_lane = prediction_ptr_->at(i).obj_lane_id;
			}
			break;
		}
	}
	double s, l;
	bool projection_flag = false;
	LaneInfoConstPtr pd_lane_ptr = nullptr;
	if (pd_obj_lane != "") {
		pd_lane_ptr = vector_map_->GetLaneById(pd_obj_lane);
		if (pd_lane_ptr != nullptr) {
			Vec2d obj_center;
			obj_center.set_x(per_obj.xabs);
			obj_center.set_y(per_obj.yabs);
			projection_flag = pd_lane_ptr->GetProjection(obj_center, &s, &l);
		}
	}
	if (!projection_flag) {
		if (!GetNearestLaneFromLocpos(per_obj.xabs, per_obj.yabs, line_obj.lane_ptr, s, l)) {
			return;
		}
	} else {
		line_obj.lane_ptr = pd_lane_ptr;
	}

	line_obj.obj_lane_s = s;
	line_obj.obj_lane_l = l;
	line_obj.obj_lane_id = line_obj.lane_ptr->id().id();
	line_obj.dis_to_junction = Dis2Junction(per_obj, line_obj.lane_ptr, s);
	line_obj.is_in_junction = (line_obj.dis_to_junction < 1.0);
	// AERROR_IF(FLAGS_log_enable)<<"MapAPI obj "<<line_obj.id<<" dis_to_junction "<<line_obj.dis_to_junction;
}

void ObjectProcess::UpdateHistoryInfo(CallbackObject &per_obj, LineObject &line_obj) {
	ObjectHistory current_obj;
	current_obj.speed = per_obj.speed;
	current_obj.type = per_obj.type;
	current_obj.obj_lane_id = line_obj.obj_lane_id;
	line_obj.position.push(make_pair(per_obj.xabs, per_obj.yabs));
	line_obj.history.push_back(current_obj);
	if (line_obj.history.size() > FLAGS_history_size) {
		line_obj.history.erase(line_obj.history.begin());
	}
	if (line_obj.position.size() > POSITION_LENGTH) {
		line_obj.position.pop();
	}

	line_obj.type = GetType(per_obj, line_obj);
	line_obj.is_static = GetMovementType(line_obj);
	line_obj.is_reverse_traveling = GetIsReverseTraveling(line_obj);
	if (FLAGS_vstatus_enable && line_obj.type < 2) {
		if (line_obj.is_static != (int)per_obj.moving_status) {
			AERROR<<"obj "<<line_obj.id<<" is static "<<line_obj.is_static
						<<" moving_status "<<(int)per_obj.moving_status;
		}
		//line_obj.is_static = (per_obj.moving_status >= 1);
	}
	line_obj.is_moveble = per_obj.moving_status > 1 ? true : false;
	line_obj.acc_type = GetAccType(line_obj);
	line_obj.was_dynamic = GetDynamicType(line_obj);
	line_obj.is_waiting = GetJunctionType(line_obj);
	if (line_obj.is_waiting) {
		line_obj.was_waiting = true;
	} else if (line_obj.was_waiting && line_obj.is_static == false) {
		if (line_obj.speed > 10.0 || line_obj.dis_to_junction < 1e-3) {
			line_obj.was_waiting = false;
		}
	}
	line_obj.disapear_counter = 0;
	// AERROR_IF(FLAGS_log_enable)<<"UpdateHistoryInfo id "<<line_obj.id<<" type "<<line_obj.type
					// <<" acc_type "<<line_obj.acc_type<<" acc "<<line_obj.acc
					// <<" static "<<line_obj.is_static<<" was_dynamic "<<line_obj.was_dynamic
					// <<" is waiting "<<line_obj.is_waiting<<" dis_to_junction "<<line_obj.dis_to_junction
					// <<" speed "<<line_obj.speed;
}

// bool ObjectProcess::GetJunctionType(const LineObject& object) {	
//   if (FLAGS_avoid_junction_object || object.obj_lane_id == "" || 
//    		object.dis_to_junction < 1e-3) {
// 		return false;
// 	}
// 	vectormap::VectorMap* vectormap = map::MapLoader::GetVectorMapPtr();
// 	LaneInfoConstPtr lane = vectormap->GetLaneById(object.obj_lane_id);
// 	bool is_in_junction = false;
//   for (auto junction_ptr : lane->junctions()) {
//     for (const auto& obj : junction_ptr->overlap().object()) {
//       if (obj.id().id() == lane->id().id() &&
//           obj.lane_overlap_info().end_s() - obj.lane_overlap_info().start_s() > 0.1) {
//       	is_in_junction = object.obj_lane_s > 1.0;
//       }
//     } 
//   }
// 	if (object.dis_to_junction < 120.0 && !object.is_static && object.type < 2) {
// 		return true;
// 	} else if (object.type > 1 || is_in_junction || object.dis_to_junction > 100.0) {
// 		return false;
// 	} else {
// 		Id left_id, right_id;
// 		acu::hdmap::Lane::LaneType type;
// 		if (vectormap->GetLeftRightLaneIDs(lane->id(), left_id, right_id) > 1 || 
// 				(0 == vectormap->GetLaneType(lane->id(), type)  && 
// 				   type == acu::hdmap::Lane_LaneType::Lane_LaneType_CITY_DRIVING) ) {
// 			return true;
// 		}
// 	}
// 	return false;
// }

bool ObjectProcess::GetJunctionType(const LineObject& object) {	
  if (FLAGS_avoid_junction_object || object.obj_lane_id == "" || 
   		object.dis_to_junction < 1e-3) {
		return false;
	}
	vectormap::VectorMap* vectormap = map::MapLoader::GetVectorMapPtr();
	LaneInfoConstPtr lane = vectormap->GetLaneById(object.obj_lane_id);
	bool is_in_junction = false;
	bool is_in_waiting_left = false;
	//判断是否位于左转待转区
	acu::hdmap::Lane::LaneType lanetype;
	if(vectormap->GetLaneType(lane->id(), lanetype) == 0){
		if(lanetype == acu::hdmap::Lane_LaneType::Lane_LaneType_WAITINGLEFT){
			is_in_waiting_left = true;
		}
	}
	//障碍物位于路口内时，可能同时位于掉头车道以及左转待转车道，
	//目前障碍物直接使用预测的结果进行车道id赋值（单一值），因此可能预测没有给出左转待转属性
	//在此增加障碍物的附近所有车道遍历，检查是否存在左转待转的可能，见20645
	if(!is_in_waiting_left){
		common::PointENU point;
		point.set_x(object.xabs);
		point.set_y(object.yabs);
	 	const double distance = 2.0;
	 	const double central_heading = object.global_angle * M_PI / 180.0;
	 	const double max_heading_difference = 20.0 * M_PI / 180.0;
	 	std::vector<std::pair<LaneInfoConstPtr, double>> output_lanes;
	 	double nearest_s = -1, nearest_l = -1;
	 	if(-1 != vectormap->GetLanesWithHeading(point, distance, central_heading, 
 									max_heading_difference, output_lanes)){
	 		for(const auto& l : output_lanes){
	 			// AINFO<<"l: "<<l.first->id().id()<<", "<<l.second;
	 			if(vectormap->GetLaneType(l.first->id(), lanetype) == 0){
					if(lanetype == acu::hdmap::Lane_LaneType::Lane_LaneType_WAITINGLEFT){
						AWARN_IF(FLAGS_log_enable)<<"set is_in_waiting_left: "<<object.id;
						is_in_waiting_left = true;
						break;//注意，即便判断车道与预测结果不同，也并没有修改object.obj_lane_id
					}
				}
	 		}
	 	}
	}
	for (auto junction_ptr : lane->junctions()) {
		for (const auto& obj : junction_ptr->overlap().object()) {
		  if (obj.id().id() == lane->id().id() &&
		      obj.lane_overlap_info().end_s() - obj.lane_overlap_info().start_s() > 0.1) {
		  	is_in_junction = object.obj_lane_s > 1.0;
		  }
		} 
	}
	if (object.dis_to_junction < 120.0 && !object.is_static && object.type < 2) {
		return true;
	} else if(is_in_waiting_left && object.type < 2 && is_in_junction && object.is_static){
		AERROR<<"set is_waiting by static in WAITINGLEFT: "<<object.id;
		return true;
	} else if (object.type > 1 || is_in_junction || object.dis_to_junction > 150.0) {
		return false;
	} else {
		Id left_id, right_id;
		acu::hdmap::Lane::LaneType type;
		if (vectormap->GetLeftRightLaneIDs(lane->id(), left_id, right_id) > 1 || 
				(0 == vectormap->GetLaneType(lane->id(), type)  && 
				   type == acu::hdmap::Lane_LaneType::Lane_LaneType_CITY_DRIVING) ) {
			return true;
		}
	}
	return false;
}


bool ObjectProcess::IsInJunction(CallbackObject &per_obj) {
	acu::common::PointENU obj_center;
	obj_center.set_x(per_obj.xabs);
	obj_center.set_y(per_obj.yabs);
	std::vector<JunctionInfoConstPtr> junctions;
    if (hdmap_->GetJunctions(obj_center, 1.0, &junctions) == 0 && !junctions.empty()) {
		return true;
	}
	return false;
}

double ObjectProcess::Dis2Junction(CallbackObject &per_obj, LaneInfoConstPtr &nearest_lane, double &s) {
	double dis_to_junction = 1000.0;
	acu::common::PointENU obj_center;
	obj_center.set_x(per_obj.xabs);
	obj_center.set_y(per_obj.yabs);
	std::vector<JunctionInfoConstPtr> junctions;
	if (hdmap_->GetJunctions(obj_center, 1.0, &junctions) == 0 && !junctions.empty()) {
		dis_to_junction = 0.0;
	} else {
		dis_to_junction = nearest_lane->total_length() - s;
		bool junction_lane = false;
		if (nearest_lane->lane().successor_id().empty()) {
			return dis_to_junction;
		}
		auto temp_lane = nearest_lane->lane().successor_id().begin();
		LaneInfoConstPtr temp_lane_ptr = hdmap_->GetLaneById(*temp_lane);
		while (dis_to_junction < 3 * FLAGS_front_perception_range) {
			if (temp_lane_ptr == nullptr) break;
			const std::vector<OverlapInfoConstPtr> junctions_ptr = temp_lane_ptr->junctions();
  			for (auto junction_ptr : junctions_ptr) {
  				for (const auto& object : junction_ptr->overlap().object()) {
  					if (object.id().id() == temp_lane_ptr->id().id()) {
  						junction_lane = true;
  					}
  				} 
  			}
  			if (junction_lane) {
  				break;
  			}
  			dis_to_junction += temp_lane_ptr->total_length();
  			if (temp_lane_ptr->lane().successor_id().empty()) break;
  			temp_lane = temp_lane_ptr->lane().successor_id().begin();
  			temp_lane_ptr = hdmap_->GetLaneById(*temp_lane);
		}
		if (!junction_lane) {
			dis_to_junction = 1000.0;
		}
	}
	return dis_to_junction;
}

int ObjectProcess::GetType(CallbackObject &per_obj, LineObject& line_obj) {
	int type = per_obj.type;
	if (line_obj.history.empty()) {
		return type;
	}
	if (line_obj.history.size() < FLAGS_history_size) {
		line_obj.last_type = type;
		line_obj.type_counter = 0;
	}	
	else if (type == line_obj.last_type) {
		line_obj.type_counter = 0;
	} 
	else if (line_obj.type_counter > FLAGS_static_times) {
		line_obj.last_type = type;
		line_obj.type_counter = 0;
	} 
	else {
		type = line_obj.last_type;
		line_obj.type_counter++;
	}
	return type;
}

int ObjectProcess::GetAccType(LineObject& line_obj) {
	int acc_type = 0;
	if (line_obj.history.size() == FLAGS_history_size) {
		vector<pair<double, double> > speeds;
		for (int i = 0; i < line_obj.history.size(); i++) {
			pair<double, double> speed((double)i * 0.1, line_obj.history.at(i).speed);
			speeds.push_back(speed);
		}
		line_obj.acc = PointsSimLine(speeds);
		line_obj.acc = fmin(line_obj.acc, FLAGS_max_pd_acc);
		line_obj.acc = fmax(line_obj.acc, -FLAGS_max_pd_acc);
		if (line_obj.acc > FLAGS_acc_level) {
			acc_type = 1;
		} 
		else if (line_obj.acc < -FLAGS_acc_level) {
			acc_type = -1;
		}
	}
	return acc_type;
}

bool ObjectProcess::GetMovementType(LineObject& line_obj) {
	bool is_static = false;
	if (line_obj.history.empty()) {
		return is_static;
	}
	if (line_obj.history.back().type == 2 || line_obj.history.back().type == 3) {
		is_static = (line_obj.history.back().speed < FLAGS_people_static_speed);
	} 
	else {
		is_static = (line_obj.history.back().speed < FLAGS_car_static_speed);
	}
	if (line_obj.history.size() < FLAGS_history_size) {
		line_obj.last_is_static = is_static;
	}	
	else if (is_static == line_obj.last_is_static) {
		line_obj.movement_counter = 0;
	} 
	else if (line_obj.movement_counter > FLAGS_static_times) {
		line_obj.last_is_static = is_static;
		line_obj.movement_counter = 0;
	} 
	else {
		is_static = line_obj.last_is_static;
		line_obj.movement_counter++;
		if (line_obj.last_is_static) {
			line_obj.movement_counter++;	// sensitive to dynamic objects
		}
	}
	return is_static;
}

bool ObjectProcess::GetIsReverseTraveling(LineObject& line_obj) {
	bool is_reverse_traveling = false;
	if (line_obj.history.empty() || !line_obj.lane_ptr) {// first time, set false
		return is_reverse_traveling;
	}
    
    //calculate speed angle
    double speed_heading = std::atan2(line_obj.vyabs, line_obj.vxabs);  
    double angle_diff = speed_heading - line_obj.lane_ptr->Heading(line_obj.obj_lane_s);
    angle_diff = acu::common::math::NormalizeAngle(angle_diff); 
    // AWARN_IF(FLAGS_log_enable)<<"id : "<<line_obj.id<<", speed_heading = "<<speed_heading
    //                           <<", angle_diff = "<<angle_diff
    //                           <<", lane heading = "<<line_obj.lane_ptr->Heading(line_obj.obj_lane_s)
    //                           <<", line_obj.obj_lane_l = "<<line_obj.obj_lane_l
    //                           <<", line_obj.obj_lane_s = "<<line_obj.obj_lane_s;                     
	
     if (!line_obj.is_static && fabs(angle_diff) > 0.75 * M_PI  && line_obj.dis_to_junction > 0) {
       is_reverse_traveling = true;
     // AWARN_IF(FLAGS_log_enable)<<"id : "<<line_obj.id<<", is_reverse_traveling = "<<is_reverse_traveling;
     }   

	if (line_obj.history.size() < FLAGS_history_size) {
		line_obj.last_is_reverse_traveling = is_reverse_traveling;
	} else if (is_reverse_traveling == line_obj.last_is_reverse_traveling) {
		line_obj.reverse_traveling_counter = 0;
	} else if (line_obj.reverse_traveling_counter > FLAGS_static_times) {
		line_obj.last_is_reverse_traveling = is_reverse_traveling;
		line_obj.reverse_traveling_counter = 0;
	} else {
		is_reverse_traveling = line_obj.last_is_reverse_traveling;
		line_obj.reverse_traveling_counter++;
		if (line_obj.last_is_reverse_traveling) {
			line_obj.reverse_traveling_counter++;	
		}
	}
	// std::cout<<line_obj.id<<" is_reverse_traveling = "<<is_reverse_traveling
	//          <<", history size = "<<line_obj.history.size()
	//          <<", reverse_traveling_counter = "<<line_obj.reverse_traveling_counter<<std::endl;
	return is_reverse_traveling;
}

bool ObjectProcess::GetDynamicType(LineObject& object) {
	if (object.is_static == false) {
		object.static_counter = 0;
	} else if (object.static_counter < 2 * FLAGS_junction_static_times) {
		object.static_counter++;
		if (object.type == 4 && object.speed < 1e-3) {
			object.static_counter++;
		}
	} else {
		object.static_counter = 2 * FLAGS_junction_static_times;
	}
	if (object.dynamic_counter > FLAGS_static_times) {
		return true;
	} else if (object.history.back().speed > 2.0) {
		object.dynamic_counter++;
	} else {
		object.dynamic_counter = 0;
	}
	return false;
}


bool ObjectProcess::GetNearestLaneFromLocpos(const double xg, const double yg,
  						LaneInfoConstPtr &nearest_lane, double &s, double &l) 
{
	acu::common::PointENU temp_point;
	temp_point.set_x(xg);
	temp_point.set_y(yg);
	return (vector_map_->GetNearestLane(temp_point, nearest_lane, s, l) == 0);
}


double ObjectProcess::GetNearestMapPointHeading(Site obj)
{
	double obj_heading = 0.0;
	double s = 0.0;
	double min_dis = 0.0;
	int min_index = 0;
	Site output_p;
	auto ref_line_info = 
		DataPool::Instance()->GetMainDataRef().cognition_info.struct_env_info.reference_line_info;
	for (int i = 0; i < ref_line_info.current_reference_line.size(); i++) {
  	if (ref_line_info.current_line_id == ref_line_info.current_reference_line.at(i).reference_lane_id) {
  		ref_line_info.current_reference_line.at(i).GetGlobalNearestPoint(obj, output_p, s, min_dis, min_index);
  		obj_heading = output_p.globalangle;
  	}
	}
	return obj_heading;
}

}
}