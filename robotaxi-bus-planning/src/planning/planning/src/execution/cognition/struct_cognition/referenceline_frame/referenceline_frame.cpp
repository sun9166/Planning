#include "referenceline_frame.h"

namespace acu {
namespace planning {

ReferenceLineFrame::ReferenceLineFrame() {
	vector_map_ = map::MapLoader::GetVectorMapPtr();
	basemap_ = map::MapLoader::GetBasemapPtr();
	if (vector_map_ == nullptr || basemap_ == nullptr) {
		return;
	}
}

ReferenceLineFrame::~ReferenceLineFrame() {}

void ReferenceLineFrame::ClearData() {
	reference_lane_id = 0;	
	temp_blocked = false;
	is_congestion = false;
	meeting_state = 0;
	meeting_id = -1;
	conflict_s = 100.0;
	average_v = 0.0;
	side_cost = 0.0;
	plan_length = 100.0;
	plan_follow_id = -1;
	jam_level = 0;
	dis_to_last_lc = 0.0;
	blind_dis = std::make_pair(100.0, 100.0);
	block_l_.clear();
	line_gap.clear();
	car_info_.clear();
	car_model_.Reset();
	loc_site_.Reset();
	follow_objects_.clear();
	park_objects_.clear();
}

void ReferenceLineFrame::Reset() {
	ClearData();
	mapinfo.Reset();
	last_path_points.clear();
	objects_.clear();
	car_on_line_.clear();
	matched_history = false;
	safety_cost = 0.0;
	block_id = -1;
	line_queue = false;
	queue_counter = 0;
	line_blocked = false;
	block_counter = 0;
	block_dis = 1000.0;
	line_slow = false;
	slow_counter = 0;
	line_speed = 1000.0;
	meeting_car = false;
	line_safety = true;
	safety_counter = 0;
	meeting_counter = 0;
	short_term_speed = FLAGS_free_speed;
	long_term_speed = FLAGS_free_speed;
	speed_cost = 0.0;
	object_on_line_.clear();
	invader_on_line_.clear();
	follow_objects_.clear();
	park_objects_.clear();
}

bool ReferenceLineFrame::CalculationFrame(const MapEngineLineList &linelist, 
					const LocalizationData &locpose, const CarModel &car_input) 
{
	Reset();
	loc_site_ = locpose.ToSite();
	car_speed_ = locpose.velocity;
	car_model_ = car_input;
	mapinfo = linelist;
	matched_history = true;
	return true;
}

bool ReferenceLineFrame::CalculationFrame(const PathData &motion_result, 
						  const LocalizationData &locpose, const CarModel &car_input) 
{
	ClearData();
	motion_path_.clear();
	bool is_new_path = false;
	for (auto &point : motion_result.path) {
		if (point.property == 4) is_new_path = true;
		motion_path_.push_back(point);
	}
	if (motion_result.is_new) {
		Reset();
	}
	else if (!is_new_path) {
		Reset();
		return false;
	}
	
	if (motion_result.path.empty()) {
		Reset();
		return false;
	}
	loc_site_ = locpose.ToSite();
	car_speed_ = locpose.velocity;
	car_model_ = car_input;
	mapinfo.path_points.reserve(3000);
	mapinfo.path_points.assign(motion_path_.begin(),motion_path_.end());
	DisToLine(mapinfo.dis2line, loc_site_);
	AERROR_IF(FLAGS_log_enable)<<"is new path "<< motion_result.is_new
								<<", size "<<motion_path_.size();
	return true;
}

void ReferenceLineFrame::SetData(const MapEngineLineList &linelist, 
																 const LocalizationData &locpose, 
																 const CarModel &car_input) 
{
	ClearData(); 
	int max_size = min((int)mapinfo.path_points.size(), 50);
	last_path_points.assign(mapinfo.path_points.begin(), mapinfo.path_points.begin() + max_size);
	mapinfo = linelist;
	loc_site_   = locpose.ToSite();
	car_model_	= car_input;
	car_speed_	= locpose.velocity;
	matched_history = true;
}

void ReferenceLineFrame::SetData(const ReferenceLineFrame &input_data) {
	ClearData(); 
	int max_size = min((int)mapinfo.path_points.size(), 50);
	last_path_points.assign(mapinfo.path_points.begin(), mapinfo.path_points.begin() + max_size);
	mapinfo = input_data.mapinfo;
	loc_site_   = input_data.loc_site_;
	car_model_	= input_data.car_model_;
	car_speed_	= input_data.car_speed_;
}


/* ***********************************api ********************************************** */
bool ReferenceLineFrame::GetNearestPoint(const double &s, Site &output_p, int &index) const {
	output_p.Reset();
	if (mapinfo.path_points.empty()) {
		AERROR<<"GetNearestPoint no reference path_points.";
		return false;
	} 
	double min_s = std::numeric_limits<double>::max();
	for (auto & p: mapinfo.path_points) {
		double distance = fabs(p.length - s);
		if (distance < min_s) {
			min_s = distance;
			output_p = p;
		}
		if (distance - min_s > FLAGS_search_range) break;
	}
	return true;//未判断min_s大小，如果需要，就给个阈值
}

bool ReferenceLineFrame::GetNearestPoint(const Site &input_p, Site &output_p, double &min_dis, int &min_index)
{
	output_p.Reset();
	if (mapinfo.path_points.empty()) {
		AERROR<<"GetNearestPoint no reference path_points.";
		return false;
	}
	min_dis = std::numeric_limits<double>::max();
	min_index = mapinfo.path_points.size();
	for (int i = 0; i < mapinfo.path_points.size(); i++) {
		double distance = hypot(input_p.x - mapinfo.path_points[i].x, 
								input_p.y - mapinfo.path_points[i].y);
		if (distance < min_dis) {
			min_dis = distance;
			min_index = i;
		}
		if (distance - min_dis > FLAGS_search_range) break;
	}
	if (min_index < 0 || min_index >= mapinfo.path_points.size()) {
		return false;
	}
	return true;//未判断min_s大小，如果需要，就给个阈值
}

bool ReferenceLineFrame::GetGlobalNearestPoint(const Site &input_p, Site &output_p, double &s,
									 double &min_dis, int &min_index) const
{
	output_p.Reset();
	if (mapinfo.path_points.empty()) {
		AERROR<<"GetGlobalNearestPoint no reference path_points.";
		return false;
	}
	min_dis = std::numeric_limits<double>::max();
	min_index = mapinfo.path_points.size();
	for (int i = 0; i < mapinfo.path_points.size(); i += 10) {
		double distance = mapinfo.path_points[i].DistanceTo(input_p);
		if (distance < min_dis) {
			min_dis = distance;
			min_index = i;
		}
	}
	if (min_index < 0 || min_index >= mapinfo.path_points.size()) {
		return false;
	}
	int start_range = 0, end_range = mapinfo.path_points.size();
	if (min_index - 5 >= 0) start_range = min_index - 5;
	if (min_index + 6 <= end_range) end_range = min_index + 6;
	for (int i = start_range; i < end_range; i++) {
		double distance = mapinfo.path_points[i].DistanceTo(input_p);
		if (distance <= min_dis) {
			min_dis = distance;
			min_index = i;
			s = mapinfo.path_points[i].length;
			output_p = mapinfo.path_points.at(i);
		}
	}
	return true;//未判断min_s大小，如果需要，就给个阈值
}

bool ReferenceLineFrame::GetGlobalNearestPointwithHeading(const Site &input_p, Site &output_p, 
									 double &s, double &min_dis, int &min_index) const 
{
	output_p.Reset();
	if (mapinfo.path_points.empty()) {
		AERROR<<"GetGlobalNearestPointwithHeading no reference path_points.";
		return false;
	}
	SiteVec anglelimit_sitevec;
	for (auto &point : mapinfo.path_points) {
		if (fabs(IncludeAngle(input_p.globalangle, point.globalangle)) < 120.0) {
			anglelimit_sitevec.push_back(point);
		}
	}
	if (anglelimit_sitevec.empty()) {
		return GetGlobalNearestPoint(input_p, output_p, s, min_dis, min_index);
	}
	else {
		min_dis = std::numeric_limits<double>::max();
		min_index = anglelimit_sitevec.size();
		for (int i = 0; i < anglelimit_sitevec.size(); i++) {
			double distance = anglelimit_sitevec[i].DistanceTo(input_p);
			if (distance < min_dis) {
				min_dis = distance;
				min_index = i;
				s = anglelimit_sitevec[i].length;
				output_p = anglelimit_sitevec.at(i);
			}
		}
		if (min_index < 0 || min_index >= mapinfo.path_points.size()) {
			return false;
		}
	}
	return true;//未判断min_s大小，如果需要，就给个阈值
}

bool ReferenceLineFrame::GetReferencePoints(const double &start_s, const double &end_s, SiteVec &output) {
	output.clear();
	if (mapinfo.path_points.empty() || end_s <= start_s) {
		AERROR<<"GetReferencePoints no reference path_points or end_s <= start_s.";
		return false;
	}
	for (auto &p : mapinfo.path_points) {
		if (p.length >= start_s) {
			output.push_back(p);
		}
		if (p.length > end_s) break;
	}
	return (output.size() > 0);
}

bool ReferenceLineFrame::GetReferencePoints(const Site &start_p, const Site &end_p, SiteVec &output) {
	Site output_p;
	double start_s, end_s, min_dis;
	int min_index;
	if (!GetGlobalNearestPoint(start_p, output_p, start_s, min_dis, min_index) ||
		!GetGlobalNearestPoint(end_p, output_p, end_s, min_dis, min_index)) {
		return false;
	}
	return GetReferencePoints(start_s, end_s, output);
}

bool ReferenceLineFrame::GetLaneType(int &type, double s) {
	type = -1;
	if (mapinfo.lane_types.empty()) {
		AERROR<<"GetLaneType no reference lane_types.";
		return false;
	}
	for (auto &lane_t : mapinfo.lane_types) {
		if (s < lane_t.first) {
			type = (int)lane_t.second;
		}
	}
	return (type >= 0);
}

bool ReferenceLineFrame::GetLaneType(int &type, const Site &input_p) {
	type = -1;
	if (mapinfo.lane_types.empty()) {
		return false;
	}
	Site output_p;
	double s, min_dis;
	int min_index;
	if (!GetGlobalNearestPoint(input_p, output_p, s, min_dis, min_index)) {
		return false;
	}
	return GetLaneType(type, s);
}

bool ReferenceLineFrame::GetLaneBoundaryType(int &l_bd_type, int &r_bd_type, double s) const {
	if (mapinfo.left_bd_types.empty() || 
		mapinfo.right_bd_types.empty()) {
		AERROR<<"GetLaneBoundaryType no reference bd_types.";
		return false;
	}
	l_bd_type = -1;
	r_bd_type = -1;
	for (auto &l_type : mapinfo.left_bd_types) {
		if (s < l_type.first) {
			l_bd_type = l_type.second;
			break;
		}
	}
	for (auto &r_type : mapinfo.right_bd_types) {
		if (s < r_type.first) {
			r_bd_type = r_type.second;
			break;
		}
	}
	return (l_bd_type >= 0)&&(r_bd_type >= 0);
}

bool ReferenceLineFrame::GetLaneBoundaryType(int &l_bd_type, int &r_bd_type, const Site &input_p) const {
	if (mapinfo.left_bd_types.empty() || 
		mapinfo.right_bd_types.empty()) {
		AERROR<<"GetLaneBoundaryType no reference bd_types.";
		return false;
	}
	Site output_p;
	double s, min_dis;
	int min_index;
	if (!GetGlobalNearestPoint(input_p, output_p, s, min_dis, min_index)) {
		return false;
	}
	return GetLaneBoundaryType(l_bd_type, r_bd_type, s);
}

bool ReferenceLineFrame::GetWidthToLaneBoundary(double &left_w, double &right_w, double s) const {
	if (mapinfo.front_lane_ids.empty()) {
		AERROR<<"GetWidthToLaneBoundary no reference front_lane_ids.";
		return false;
	}
	double real_s = s + mapinfo.first_lane_start_s;
	for (int i = 0; i < mapinfo.front_lane_ids.size(); i++) {
		LaneInfoConstPtr lane_ptr = vector_map_->GetLaneById(mapinfo.front_lane_ids.at(i));
		if (lane_ptr == nullptr) {
			//auto monitor_api = acu::common::MonitorApi::Instance();
			//monitor_api->SetFaultInfo(acu::common::NodeTime::Now().ToSecond(), 
			//				DFPLANNING_PLANNING_SOFTWARE_10_ERROR,
      //                      "MAP API WRONG:" + mapinfo.front_lane_ids.at(i) +" ptr null");
			// std::cout<<"lane_ptr is nullptr. "<<mapinfo.front_lane_ids.at(i)<<std::endl;
			continue;
		}
		double length = lane_ptr->total_length();
		if (real_s > length && i != mapinfo.front_lane_ids.size() - 1) {
			real_s -= length;
			continue;
		}
		lane_ptr->GetWidth(real_s, &left_w, &right_w);
		left_w -= FLAGS_boundary_width; 
		right_w -= FLAGS_boundary_width;
    	return true;
	}
	// AERROR_IF(FLAGS_log_enable)<<"GetWidthToLaneBoundary no find s "<<s;
	return false;
}

bool ReferenceLineFrame::GetWidthToLaneBoundary(double &left_w, double &right_w, const Site &input_p) const {
	left_w = 0.0;
	right_w = 0.0;
	double s = 1000.0;
	double l = 1000.0;
	int min_index;
	if (!XYToSL(mapinfo, input_p, s, l)) {
		return false;
	}
	s = fmax(0.001, s);
	if (!GetWidthToLaneBoundary(left_w, right_w, s)) {
		AERROR<<"GetWidthToLaneBoundary s failed.";
		return false;
	}
	left_w -= l;
	right_w += l;
	return true;
}

bool ReferenceLineFrame::GetWidthToFreeSpace(double &left_w, double &right_w, const double& s) const {
  Site nearest_point;
  int nearest_index;
  if (!GetNearestPoint(s, nearest_point, nearest_index)) {
  	AERROR_IF(FLAGS_log_enable)<<"can not Get NearestPoint";
  	return false;
  }
  acu::vectormap::PointGCCS point;
  point.xg = nearest_point.xg;
  point.yg = nearest_point.yg;
  point.angle = nearest_point.globalangle / 180 * M_PI;
  if(!GetWidthToRoadBoundary(left_w,right_w,s)){
  	return false;
  } 
  //if (basemap_->GetLeftRightDistanceToFreespace(point,point.angle,left_w,right_w) < 0) {
  //  return false; 
  //}
  return true;
}

bool ReferenceLineFrame::IsInFreeSpace(const PointGCCS& target_point) const {
  PointVCS pvcs{0.0, 0.0, 0.0};
  // CHECK_NOTNULL(basemap_);
  //if (basemap_->DistanceToFreeSpace(target_point, pvcs) < 0 ) {
  //  return false;
  //}
  return true;
}

bool ReferenceLineFrame::GetWidthToRoadBoundary(double &left_w, double &right_w, double s, bool road) const {
	left_w = 0.0;
	right_w = 0.0;
	if (mapinfo.front_lane_ids.empty()) {
		AERROR<<"GetWidthToRoadBoundary no reference front_lane_ids.";
		return false;
	}
	LaneInfoConstPtr lane_ptr;
	double real_s = s + mapinfo.first_lane_start_s;
	for (int i = 0; i < mapinfo.front_lane_ids.size(); i++) {
		lane_ptr = vector_map_->GetLaneById(mapinfo.front_lane_ids.at(i));
		if (lane_ptr == nullptr) {
			//auto monitor_api = acu::common::MonitorApi::Instance();
			//monitor_api->SetFaultInfo(acu::common::NodeTime::Now().ToSecond(), 
																//DFPLANNING_PLANNING_SOFTWARE_10_ERROR,
                        				//"MAP API WRONG:" + mapinfo.front_lane_ids.at(i) +" ptr null");
			continue;
		}
		double length = lane_ptr->total_length();
		// if (i == 0 && mapinfo.distance_to_ends.size() > 0) {
		// 	length = mapinfo.distance_to_ends.front().second;
		// }
		if (real_s > length && i != mapinfo.front_lane_ids.size() - 1) {
			real_s -= length;
			continue;
		}
		break;
	}
	// AERROR<<"real_s "<<real_s<<" s "<<s;
	if (road) {
		Vec2d near_point = lane_ptr->Point(real_s);
		vector<string> lanes_in_road;
		if (vector_map_->GetRoadLanes(lane_ptr->road_id().id(), lanes_in_road) < 0) {
			AERROR<<"GetWidthToRoadBoundary no GetRoadLanes.";
			return false;
		}
		if (lanes_in_road.size() == 1) {
			lane_ptr->GetWidth(real_s, &left_w, &right_w);
			return true;
		}
		LaneInfoConstPtr left_ptr = vector_map_->GetLaneById(lanes_in_road.front());
		LaneInfoConstPtr right_ptr = vector_map_->GetLaneById(lanes_in_road.back());
		double temp_s, temp_l, l_w, r_w;
  	if (!left_ptr->GetProjection(near_point, &temp_s, &temp_l)) {
  		return false;
  	}
  	// AERROR<<"Left lane l "<<temp_l;
  	left_ptr->GetWidth(real_s, &l_w, &r_w);
  	left_w = l_w - temp_l;

  	if (!right_ptr->GetProjection(near_point, &temp_s, &temp_l)) {
  		return false;
  	}
  	// AERROR<<"Right lane l "<<temp_l;
  	right_ptr->GetWidth(real_s, &l_w, &r_w);
  	right_w = r_w + temp_l;
  	// AERROR_IF(FLAGS_log_enable)<<"GetWidthToLaneRoadBoundary "<<lane_ptr->id().id()
			// 			<<" left_w "<<left_w<<" right_w "<<right_w;
	} else {
		lane_ptr->GetRoadWidth(real_s, &left_w, &right_w);
		// AERROR_IF(FLAGS_log_enable)<<"GetWidthToRoadBoundary "<<lane_ptr->id().id()
		// 				<<" left_w "<<left_w<<" right_w "<<right_w;
	}
	return true;
}

bool ReferenceLineFrame::GetWidthToRoadBoundary(double &left_w, double &right_w, const Site &input_p, bool road) const {
	left_w = 0.0;
	right_w = 0.0;
	double s = 1000.0;
	double l = 1000.0;
	if (!XYToSL(mapinfo, input_p, s, l)) {
		return false;
	}
	s = fmax(0.001, s);
	if (!GetWidthToRoadBoundary(left_w, right_w, s, road)) {
		return false;
	}
	AERROR_IF(FLAGS_log_enable)<<"GetWidthToRoadBoundary s "<<s<<" l "<<l;
	left_w -= l;
	right_w += l;
	return true;
}

bool ReferenceLineFrame::IsInLine(const string &lane_id) {
	if (mapinfo.front_lane_ids.empty() || lane_id.empty()) {
		AERROR<<"IsInLine no reference front_lane_ids.";
		return false;
	}
	for (auto &lane : mapinfo.front_lane_ids) {
		if (lane_id == lane) {
			return true;
		} 
	}
	return false;
}

bool ReferenceLineFrame::IsInLine(const Site &input_p) {
	if (mapinfo.front_lane_ids.empty()) {
		AERROR<<"IsInLine no reference front_lane_ids.";
		return false;
	}
	double dis2lane = 100.0;
	for (int i = 0; i < mapinfo.path_points.size(); i++) {
		if (mapinfo.path_points.at(i).length < input_p.length - 10.0 ||
				mapinfo.path_points.at(i).length > input_p.length + 10.0) {
			continue;
		}
		double dis = hypot(mapinfo.path_points.at(i).xg - input_p.xg,
											 mapinfo.path_points.at(i).yg - input_p.yg);
		if (dis < dis2lane) {
			dis2lane = dis;
		}
	}

	double left_w = 0.0;
	double right_w = 0.0;
	int lane_index = -1;
	double delta_length = 0.0;
	for (int i = 0; i < mapinfo.distance_to_ends.size(); i++) {
		if (mapinfo.distance_to_ends.at(i).second > input_p.length &&
				mapinfo.distance_to_ends.at(i).first <= input_p.length) {
			lane_index = i;
			delta_length = input_p.length - mapinfo.distance_to_ends.at(i).first;
			break;
		}
	}
  // AERROR<<"dis2lane "<<dis2lane<<" lane_index "<<lane_index<<" delta_length "<<delta_length;
  if (lane_index >= mapinfo.front_lane_ids.size() || lane_index < 0) {
  	return false;
  }
  LaneInfoConstPtr lane_ptr = vector_map_->GetLaneById(mapinfo.front_lane_ids.at(lane_index));
  lane_ptr->GetWidth(delta_length, &left_w, &right_w);

  if (dis2lane <= left_w && dis2lane <= right_w && dis2lane > 0.0) {
  	return true;
  }
	return false;
}

bool ReferenceLineFrame::DisToLine(double &distance, const Site &input_p) {
	distance = -1.0;
	if (mapinfo.front_lane_ids.empty()) {
		AERROR<<"DisToLine no reference front_lane_ids.";
		return false;
	}
	acu::common::math::Vec2d point;
  	point.set_x(input_p.xg);
  	point.set_y(input_p.yg);

	for (auto &lane : mapinfo.front_lane_ids) {
    	double s = 0.0;
    	double l = 0.0;
    	LaneInfoConstPtr lane_ptr = vector_map_->GetLaneById(lane);
    	if (lane_ptr == nullptr) {
    		//auto monitor_api = acu::common::MonitorApi::Instance();
    		//monitor_api->SetFaultInfo(acu::common::NodeTime::Now().ToSecond(), 
							//DFPLANNING_PLANNING_SOFTWARE_10_ERROR,
                            //"MAP API WRONG:" + lane +" ptr null");
    	}
    	if (lane_ptr->GetProjection(point, &s, &l) && s <= lane_ptr->total_length()) {
    		distance = l;
    		return true;
    	}
	}
	return false;
}

bool ReferenceLineFrame::DisToEnd(double &distance, double s) {
	distance = -1.0;
	if (mapinfo.path_points.empty()) {
		AERROR<<"DisToEnd no reference point.";
		return false;
	}
	if (s > mapinfo.path_points.back().length) {
		AERROR<<"DisToEnd input s is out of range.";
		return false;
	}
	distance = mapinfo.path_points.back().length - s;
	return true;
}

bool ReferenceLineFrame::DisToEnd(double &distance, const Site &input_p) {
	distance = -1.0;
	Site output_p;
	double s, min_dis;
	int index;
	if (!GetGlobalNearestPoint(input_p, output_p, s, min_dis, index)) {
		return false;
	}
	return DisToEnd(distance, s);
}

bool ReferenceLineFrame::DisToSpecialArea(double &distance, const Area &type, double s) {
	distance = 1000.0;
	vector<pair<double, int> > special_areas;
	switch (type) {
		case Area::SPEEDBUMP :
			special_areas.assign(mapinfo.distance_to_speed_bumps.begin(), 
								 mapinfo.distance_to_speed_bumps.end());
			break;
		case Area::FORBIDAREA :
			special_areas.assign(mapinfo.distance_to_forbid_areas.begin(), 
								 mapinfo.distance_to_forbid_areas.end());
			break;
		case Area::JUNCTION :
			special_areas.assign(mapinfo.distance_to_junctions.begin(), 
								 mapinfo.distance_to_junctions.end());
			break;
		case Area::CROSSWALK :
			special_areas.assign(mapinfo.distance_to_crosswalks.begin(), 
								 mapinfo.distance_to_crosswalks.end());
			break;
		default :
			special_areas.clear();
			break;
	}
	if (s > special_areas.back().second || s < 0.0) {
		return false;
	}
	if (special_areas.empty()) {
		return true;
	}
	if (s < special_areas.front().first) {// 在第一段
		distance = special_areas.front().first - s;
		return true;
	}
	for (int i = 0; i < special_areas.size(); i++) {// 在bump上
		if (s >= special_areas.at(i).first && s <= special_areas.at(i).second) {
			distance = 0.0;
			return true;
		}
	}
	for (int i = 1; i < special_areas.size(); i++) {// 在bump间
		if (s > special_areas.at(i-1).second && s < special_areas.at(i).first) {
			distance = special_areas.at(i).first - s;
			return true;
		}
	}
	return false;
}

bool ReferenceLineFrame::DisToSpecialArea(double &distance, const Area &type, const Site &input_p) {
	distance = 1000.0;
	Site output_p;
	double s, min_dis;
	int index;
	if (!GetGlobalNearestPoint(input_p, output_p, s, min_dis, index)) {
		return false;
	}
	return DisToSpecialArea(distance, type, s);
}

bool ReferenceLineFrame::GetSFromIndex(const int &index, double &s) {
	s = -1.0;
	if (index < 0 || index > mapinfo.path_points.size()) {
		return false;
	}
	s = mapinfo.path_points.at(index).length;
	return true;
}

bool ReferenceLineFrame::GetIndexFromS(const double &s, int &index) {
	index = -1;
	if (s < 0 || s > mapinfo.path_points.back().length) {
		return false;
	}
	double min_dis = std::numeric_limits<double>::max();
	for (int i = 0; i < mapinfo.path_points.size(); i++) {
		double delta_s = fabs(s - mapinfo.path_points.at(i).length);
		if (delta_s < min_dis) {
			min_dis = delta_s;
			index = i;
		}
		if (delta_s > min_dis + 5) break;
	}
	return (index >= 0);
}

bool ReferenceLineFrame::IsUturn(std::pair<double, double>& u_turn_s_interval) const {
    double start_s = 0.0;
    double end_s = 0.0;
    bool find_start_s = false;
    bool find_end_s = false;
    u_turn_s_interval = std::make_pair(-1.0, 0.0);

    if (mapinfo.path_points.empty()) {
		return false;
	}
	for (size_t i = 0; i < mapinfo.lane_turns.size(); ++i) {
		//std::cout<<"turn type = "<<mapinfo.lane_turns.at(i).second<<std::endl;
		if (mapinfo.lane_turns.at(i).second == 4 
			&& !find_end_s && !find_start_s) {
			find_start_s = true;
			find_end_s = true;
			end_s = mapinfo.lane_turns.at(i).first;
			//std::cout<<"IsUturn find_end_s "<<end_s<<", i ="<<i<<std::endl;
			if (i > 0) {
              start_s = mapinfo.lane_turns.at(i-1).first;
			} else {
              start_s = 0.0;	
			}
			//std::cout<<"IsUturn find_start_s "<<start_s<<std::endl;
		}
		if (find_start_s && find_end_s) {
           u_turn_s_interval = std::make_pair(start_s, end_s);
           //std::cout<<"IsUturn debug "<<std::endl;
           return true;
		}
	}

	return false;
}

bool ReferenceLineFrame::IsRoadSide() {
	Site corner;
	GeoTool geotool;
	PointVCS rel_point;
	PointGCCS origin_point, target_point;
	double left_w, right_w = 0.0; 
	corner.set_g(loc_site_.xg, loc_site_.yg);
	GetWidthToRoadBoundary(left_w, right_w, corner);
	AWARN_IF(FLAGS_log_enable) << "left_w = " << left_w << " right_w = " << right_w;
	return right_w < 0.0;
}

double ReferenceLineFrame::Length() const {
		if (!mapinfo.path_points.empty()) {
           return mapinfo.path_points.back().length;
		} 
		return 0;
	}

bool ReferenceLineFrame::GetSucJunctionRoadId(vector<string> &suc_junction_roads) const {
	suc_junction_roads.clear();
	if (mapinfo.front_lane_ids.empty()) return false;
	vector<Id> suc_lanes;
	if (vector_map_->GetLaneSuccessorIDs(mapinfo.front_lane_ids.back(), suc_lanes) <= 0) {
		return false;
	}
	for (auto &temp_Id : suc_lanes) {
		acu::hdmap::Lane::LaneType lanetype;
		if (vector_map_->GetLaneType(temp_Id, lanetype) != 0) continue;
    	if (lanetype == acu::hdmap::Lane::LaneType::Lane_LaneType_CITY_DRIVING ||
				lanetype == acu::hdmap::Lane::LaneType::Lane_LaneType_ISLAND ||
				lanetype == acu::hdmap::Lane::LaneType::Lane_LaneType_WAITINGLEFT ||
				lanetype == acu::hdmap::Lane::LaneType::Lane_LaneType_WAITINGSTRAIGHT) {
    		string raod_id;
    		if (vector_map_->GetRoad(temp_Id.id(), raod_id) == 0) {
    			suc_junction_roads.push_back(temp_Id.id());
    		}
    	}
	}
	return !suc_junction_roads.empty();
}

bool ReferenceLineFrame::GetBoundaryXY(const double s,bool left, double& xg,double& yg){
	double l_w,r_w;
	int index;
	Site point;
	bool w_re = GetWidthToLaneBoundary(l_w,r_w,s);
	bool n_re = GetNearestPoint(s, point, index);
	if(w_re && n_re){
		double l = left? l_w:-r_w;
		double angle = point.globalangle * 3.1415/180;
		xg = point.xg - std::sin(angle)*l;
		yg = point.yg + std::cos(angle)*l;
		return true;
	}
	return false;
}



}
}
