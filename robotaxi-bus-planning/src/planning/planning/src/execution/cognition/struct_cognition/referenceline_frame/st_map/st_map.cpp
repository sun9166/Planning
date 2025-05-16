#include "st_map.h"

namespace acu {
namespace planning {

STmap::STmap():
 compensate_t_(0), compensate_s_(0)	
 {
	range_s_ = (int)((FLAGS_s_range)/FLAGS_scale_s) + 1;
	range_t_ = (int)(FLAGS_t_range/FLAGS_scale_t) + 1;
	t_map_.clear();
	t_map_.reserve(range_t_ + 1);

	// current_st_map_ = (STMapPointStruct**) malloc(range_t_ * sizeof(STMapPointStruct*));
	// for(int i = 0; i < range_t_ ; i++) {
	// 	current_st_map_[i] = (STMapPointStruct*)malloc( range_s_ * sizeof(STMapPointStruct));
	// }

	current_st_map_ = new STMapPointStruct *[range_t_];
	for(int i = 0; i < range_t_; i++) {
		current_st_map_[i] = new STMapPointStruct[range_s_]();
	}

	for (int t = 0; t < range_t_; t++) {
		for (int s = 0; s < range_s_; s++) {
			STMapPointStruct init_st_point;
			init_st_point.index.s = s;
			init_st_point.index.t = t;
			current_st_map_[t][s] = init_st_point;
		}
	}
}

STmap::STmap(const STmap& st_map){
	this->vector_map_ = map::MapLoader::GetVectorMapPtr();
	this->line_data_ptr_ = & (*st_map.line_data_ptr_);
	this->last_points_ptr_ = &(*st_map.last_points_ptr_);
	this->car_model_ptr_ = &(*st_map.car_model_ptr_);
	this->loc_ptr_ = &(*st_map.loc_ptr_);
	this->reference_lane_id_ = st_map.reference_lane_id_;
	this->compensate_t_ = st_map.compensate_t_;
	this->compensate_s_ = st_map.compensate_s_;
	this->range_s_ = st_map.range_s_;
	this->range_t_ = st_map.range_t_;
	this->update_s_ = st_map.update_s_;
	this->update_t_ = st_map.update_t_;

	current_st_map_ = new STMapPointStruct *[range_t_];
	for(int i = 0; i < range_t_; i++) {
		current_st_map_[i] = new STMapPointStruct[range_s_];
	}
	if(st_map.current_st_map_ != nullptr) {
		for(int i = 0; i < range_t_; i++) {
			std::copy(st_map.current_st_map_[i] , st_map.current_st_map_[i] + range_s_,current_st_map_[i]);
		}
	}

	GetTMap();
}

void STmap::operator=(const STmap& st_map){
	this->vector_map_ = map::MapLoader::GetVectorMapPtr();
	this->line_data_ptr_ = & (*st_map.line_data_ptr_);
	this->last_points_ptr_ = &(*st_map.last_points_ptr_);
	this->car_model_ptr_ = &(*st_map.car_model_ptr_);
	this->loc_ptr_ = &(*st_map.loc_ptr_);
	this->reference_lane_id_ = st_map.reference_lane_id_;
	this->compensate_t_ = st_map.compensate_t_;
	this->compensate_s_ = st_map.compensate_s_;
	this->range_s_ = st_map.range_s_;
	this->range_t_ = st_map.range_t_;
	this->update_s_ = st_map.update_s_;
	this->update_t_ = st_map.update_t_;

	if(st_map.current_st_map_ != nullptr) {
		for(int i = 0; i < range_t_; i++) {
			std::copy(st_map.current_st_map_[i] , st_map.current_st_map_[i] + range_s_,current_st_map_[i]);
		}
	}

	GetTMap();
}

STmap::~STmap() {
	if(current_st_map_ != nullptr) {
		for(int i=0; i < range_t_;i++){
			delete[] current_st_map_[i];
		}
		delete[] current_st_map_;
	}

	current_st_map_ = nullptr;
	t_map_.clear();
}

void STmap::Reset() { 
	STmapReset();
	line_data_ptr_ = nullptr;
	car_model_ptr_ = nullptr;
	loc_ptr_ = nullptr;
	car_speed_ = 0.0;
 	compensate_t_ = 0.0;
 	compensate_s_ = 0.0;	
}

void STmap::STmapReset() {
	for(int i=0; i < range_t_; i++) {
		for(int j=0; j < range_s_; j++) {
			current_st_map_[i][j].Reset();
		}
	}
} 

bool STmap::InitSTmap(MapEngineLineList *line_data_ptr, 
					  					const SiteVec *last_points_ptr, 
					  					const int &reference_lane_id,
											const Site *locpose, const CarModel *car_input) {	
	vector_map_ = map::MapLoader::GetVectorMapPtr();
	car_model_ptr_ = car_input;
	line_data_ptr_ = line_data_ptr;
	reference_lane_id_ = reference_lane_id;
	last_points_ptr_ = last_points_ptr;
	loc_ptr_ = locpose;
	if (line_data_ptr_ == nullptr || loc_ptr_ == nullptr) {
		AERROR_IF(FLAGS_log_enable)<<"STmap init failed.";
		return false;
	}
	car_speed_ = loc_ptr_->velocity;
	return true;
}
 
bool STmap::CalculateObjectsST(std::map<int, LineObject> &line_objs) {
	STmapReset();
	if (line_objs.empty()) return true;
	if (line_data_ptr_->path_points.size() < 3) {
		AERROR_IF(FLAGS_log_enable)<<"path points are too less to calculate s t.";
		return false;
	}
	if (!CalculateDeltaST()) {//自车的移动补偿
		AERROR_IF(FLAGS_log_enable)<<"Update s t failed.";
		return false;
	}
	vector<BoxStruct> line_boxes;
	st_common_.GetLineBoxes(car_model_ptr_, line_data_ptr_->path_points, line_boxes);
	for (auto it = line_objs.begin(); it != line_objs.end(); it++) {
		auto &obj = it->second;
		obj.st_boundary.s_t.clear();
		//先生成pd_trajectory.st_boundary，再根据pd_trajectory.st_boundary生成pd_trajectory.st_points
		if (!GetPredictionSTBoundary(it->second, line_boxes)) {//插值拟合生成pd_trajectory.st_points
			continue;
		}
		//根据pd_trajectory.st_points生成st_area.pd_objs.at(i).st
		//st_area.pd_objs的数量表示单障碍物预测线的个数
		//st_area.pd_objs.at(i).st表示某条预测线的st
		UpdateObjectSTArea(it->second);//更新obj.st_area.pd_objs(.st)
	}
	return true;
}

void STmap::BuildSTMap(std::map<int, LineObject> &line_objs) {
	if (!FLAGS_stmap_enable) return;
	LimitFollowingST(line_objs);
	CalculateMapProbability(line_objs);
}

bool STmap::CalculateDeltaST() {
	update_s_ = (int)(car_speed_ * 0.1 / FLAGS_scale_s);
	double min_dis = std::numeric_limits<double>::max();
	if (last_points_ptr_->empty()) {
		AERROR_IF(FLAGS_log_enable)<<"first time no last_line_points.";
		return true;
	}
	int min_index = -1;
	for(int i = 0; i < last_points_ptr_->size(); i++) {
		double distance = hypot(loc_ptr_->xg - last_points_ptr_->at(i).xg, 
														loc_ptr_->yg - last_points_ptr_->at(i).yg);
		if (distance < min_dis) {
			min_dis = distance;
			min_index = i;
		}
	}
	if (min_index < 0 || min_dis > 10.0) {
		return false;
	}
	update_s_ = RoundN(last_points_ptr_->at(min_index).length, FLAGS_scale_s);
	update_t_ = ceil(0.1 / FLAGS_scale_t);// 相当于往前多推了一点点
	return true;
}

bool STmap::GetPredictionSTBoundary(LineObject &line_obj, vector<BoxStruct> &line_boxes) {
	if (!line_obj.need_focus || line_obj.is_static || line_obj.prediction.is_ultra_static) {
		return true;
	}
	double obj_width, obj_length;
	// if (fmin(fabs(line_obj.sl_boundary.min_l), fabs(line_obj.sl_boundary.max_l)) < 2.0 &&
	// 		hypot(line_obj.sl_boundary.min_l, line_obj.sl_boundary.min_s) < 40.0) {
	// 	obj_width = line_obj.cell_box.width() + 2.0 * FLAGS_collision_buff;
	// 	obj_length = line_obj.cell_box.length();
	// }
	// else 
	{
		obj_width = line_obj.box.width() + 2.0 * FLAGS_collision_buff;
		obj_length = line_obj.box.length();
	}

	for (auto &pd_trajectory : line_obj.prediction.trajectories) {
		if (pd_trajectory.points.empty()) continue;
		vector<BoxStruct> pd_boxes;
		pd_trajectory.st_boundary.clear();
		AINFO_IF(FLAGS_log_enable)<<"-----obj "<<line_obj.id<<"------------"<<line_obj.prediction.trajectories.size();
		st_common_.GetPdBoxes(pd_trajectory, obj_length, obj_width, line_obj.is_reverse_traveling, pd_boxes);
		st_common_.BoxST(pd_boxes, line_boxes, pd_trajectory.range_pd_s, pd_trajectory.st_boundary);//生成pd_trajectory.st_boundary
		// local后向有碰撞障碍物都离散
		if (reference_lane_id_ == 40 && !pd_trajectory.st_boundary.empty() &&
				line_obj.sl_boundary.max_s < 0.0) {
			line_obj.key_focus = 2;
		}
		if (!FLAGS_stmap_enable) return true;
		if (line_obj.key_focus <= 1 || (reference_lane_id_ >= 20 && reference_lane_id_ < 40)) {
			st_common_.RoundST(pd_trajectory);//根据离散的pd_trajectory.st_boundary插值拟合生成pd_trajectory.st_points
			//pd_trajectory.st_points所有点的p与pd_trajectory.probability相同
		} 
		// else if (line_obj.key_focus == 1) {
		// 	double min_l = fmin(fabs(line_obj.sl_boundary.min_l), fabs(line_obj.sl_boundary.max_l));
		// 	min_l -= car_model_ptr_->car_width / 2.0;
		// 	st_discret_.Latdiscret(line_data_ptr_, min_l, pd_trajectory, line_obj.key_focus);
		// } 
		//挤压障碍物，修改其pd_line.st_points，pd_line.londiscrete_areas，pd_line.range_a
		else if (line_obj.key_focus == 2) {
			st_discret_.Londiscret(pd_trajectory);
		}
	}
	return true;
}

void STmap::UpdateObjectSTArea(LineObject &obj) {
	if (!FLAGS_stmap_enable) return;
	obj.st_area.id = obj.id;
	for (auto &pd_obj : obj.st_area.pd_objs) {// 重置障碍物更新标志位
		pd_obj.update_flag = false;
	}
	int index = -1;
	for (auto &pd_line : obj.prediction.trajectories) {// 预测线匹配用lane_ids做
		index++;
		if (pd_line.st_points.empty()) continue;
		int pd_history_index = -1;
		for (int i = 0; i < obj.st_area.pd_objs.size(); i++) {
			if (obj.st_area.pd_objs.at(i).equal(pd_line.lane_ids, FLAGS_lane_id_match_num) && 
					!obj.st_area.pd_objs.at(i).update_flag) {
				pd_history_index = i;
				break;
			}
		}
		// 障碍物每帧只有一条预测线，并且闪烁
		if (pd_history_index < 0 && obj.st_area.pd_objs.size() == 1 &&
				obj.prediction.trajectories.size() == 1) {
			pd_history_index = 0;
		}
		if (pd_history_index < 0) {// 第一种 没匹配上历史预测线 
			AINFO_IF(FLAGS_log_enable)<<"No history st, new it.";
			PdObjectStruct new_pd_obj;
			for (auto &point : pd_line.st_points) {
				point.p = point.p * FLAGS_p_o;
				new_pd_obj.st.push_back(point);//单条预测线的st
			}
			new_pd_obj.history_st.clear();
			new_pd_obj.pd_id = pd_line.lane_ids;
			new_pd_obj.update_flag = true;
			new_pd_obj.prediction_index = index;
			new_pd_obj.accumulate_time = 1;
			obj.st_area.pd_objs.push_back(new_pd_obj);
		} else {// 第二种 匹配上了历史预测线
			AINFO_IF(FLAGS_log_enable)<<"Match history st.";
			obj.st_area.pd_objs.at(pd_history_index).history_st = obj.st_area.pd_objs.at(pd_history_index).st;
			OverlayObjProbability(obj.st_area.pd_objs.at(pd_history_index).st, pd_line.st_points, update_s_, update_t_);
			obj.st_area.pd_objs.at(pd_history_index).pd_id = pd_line.lane_ids;
			obj.st_area.pd_objs.at(pd_history_index).update_flag = true;
			obj.st_area.pd_objs.at(pd_history_index).prediction_index = index;
			obj.st_area.pd_objs.at(pd_history_index).accumulate_time = 1;
		}
	}
	ReduceDisappearedPdObj(obj);
}

void STmap::ReduceDisappearedPdObj(LineObject &obj) {
	for (int i = 0; i < obj.st_area.pd_objs.size(); i++) {
		auto &pd_obj = obj.st_area.pd_objs.at(i);
		if (pd_obj.update_flag) {
			continue;
		} else if (!FLAGS_stmap_enable) {
			obj.st_area.pd_objs.erase(obj.st_area.pd_objs.begin() + i);
			i--;
			AERROR_IF(FLAGS_log_enable)<<"obj "<<obj.id<<" pd obj empty, erase.";
			continue;
		}
		for (int j = 0; j < pd_obj.st.size(); j++) {// 衰减
			double L = (1.0 - FLAGS_p_e)/FLAGS_p_e;
			if (pd_obj.st.at(j).p > FLAGS_p_thd_delete) {
				L *= pd_obj.st.at(j).p/(1.01 - pd_obj.st.at(j).p);
				pd_obj.st.at(j).p = L/(1.0 + L);
			}
			if (pd_obj.st.at(j).p < FLAGS_p_thd_delete) {
				pd_obj.st.erase(pd_obj.st.begin() + j);
				j--;
			}
		}
		pd_obj.accumulate_time++;
		if (obj.st_area.pd_objs.at(i).st.empty()) {
			obj.st_area.pd_objs.erase(obj.st_area.pd_objs.begin() + i);
			i--;
		}
	}
}

bool STmap::CalculateMapProbability(std::map<int, LineObject> &line_objs) {
	if (current_st_map_ == nullptr) {
		return false;
	}
	for (auto &it : line_objs) {
		auto &object = it.second;
		SetObjectToSTMap(object);//利用object.st_area.pd_objs.st更新current_st_map_
	}
	MapProbabilityDamping();
	GetTMap();
	return true;
}

void STmap::MapProbabilityDamping() {
	if (current_st_map_ == nullptr) {
		return;
	}
	for (int t = 0; t < range_t_; t++) {
		if (t < 20) continue;
		for (int s = 0; s < range_s_; s++) {
			auto &point_p = current_st_map_[t][s].p;
			if (point_p < FLAGS_p_thd_delete) continue;
			double damping_p = point_p - fmax(FLAGS_p_thd_delete, point_p * 0.8);
			current_st_map_[t][s].p -= (double)(t - 20)/20.0 * damping_p;
		}
	}
}

/*------------------------api-------------------------*/
void STmap::GetTMap() {
	t_map_.clear();
	if (current_st_map_ == nullptr) {
		return;
	}
	t_map_.reserve(range_t_);
	TpointsStruct temp_t_points;
	for (int t = 0; t < range_t_; t++) {
		temp_t_points.t = t;
		if (current_st_map_[t] == nullptr) {
			return;
		}
		temp_t_points.t_points.clear();
		for (int s = 0; s < range_s_; s ++) {
			if (current_st_map_[t][s].p >= FLAGS_p_thd_delete) {
				if (temp_t_points.occupied_range.first < 0) {
					temp_t_points.occupied_range.first = s;
				}
				temp_t_points.occupied_range.second = s;
				temp_t_points.t_points.push_back(&(current_st_map_[t][s]));
			}
			if (current_st_map_[t][s].p >= FLAGS_p_threshold) {
				if (temp_t_points.solid_occupied_range.first < 0) {
					temp_t_points.solid_occupied_range.first = s;
				}
				temp_t_points.solid_occupied_range.second = s;
			}			
		}
		t_map_.push_back(temp_t_points);
	}
}

bool STmap::GetTPoints(const int t, vector<STMapPointStruct> &t_points) const {
	t_points.clear();
	if (t < 0 || t >= range_t_ || t >= t_map_.size()) {
		// AERROR_IF(FLAGS_log_enable)<<"input t "<<t<<" out of map.";
		return false;
	}
	// AERROR_IF(FLAGS_log_enable)<<"tmap size "<<t_map_.size()<<" t "<<t;
	t_points.reserve(range_s_);
	if (current_st_map_ == nullptr) {
		return false;
	}
	for (int s = 0; s < range_s_; s ++) {
		if (current_st_map_[t][s].p >= FLAGS_p_thd_delete) {
			t_points.push_back(current_st_map_[t][s]);
		}
	}
	return true;
}

//补偿S与T的值，使得STmap整体平移（向原点方向平移补偿应为正值）
bool STmap::GetCompensateTPoints(const int t, vector<STMapPointStruct> &t_points) const {
	bool rtvalue = false;
	if(fabs(compensate_t_) < 1e-5 && fabs(compensate_s_) < 1e-5){
		return GetTPoints(t, t_points);
	}
	if(GetTPoints(t + compensate_t_, t_points)){
		auto iter = t_points.begin();
		while(iter != t_points.end()){
			//index的p a_p未处理
			iter->index.s -= compensate_s_;
			iter->index.t -= compensate_t_;
			++iter;
		}
		rtvalue = true;
	}
	return rtvalue;
}

//重载函数，允许将补偿量作为形参传入
bool STmap::GetCompensateTPoints(const int t, vector<STMapPointStruct> &t_points,
								 int input_compensate_t, int input_compensate_s) const {
	bool rtvalue = false;
	if(fabs(input_compensate_t) < 1e-5 && fabs(input_compensate_s) < 1e-5){
		return GetTPoints(t, t_points);
	}
	if(GetTPoints(t + input_compensate_t, t_points)){
		auto iter = t_points.begin();
		while(iter != t_points.end()){
			//index的p a_p未处理
			iter->index.s -= input_compensate_s;
			iter->index.t -= input_compensate_t;
			++iter;
		}
		rtvalue = true;
	}
	return rtvalue;
}

//重载函数，允许将补偿量作为形参传入，累加使用内置平移参数
bool STmap::GetAccumulateCompensateTPoints(const int t, vector<STMapPointStruct> &t_points,
								 int input_compensate_t, int input_compensate_s) const {
	bool rtvalue = false;
	if(fabs(input_compensate_t) < 1e-5 && fabs(input_compensate_s) < 1e-5){
		return GetCompensateTPoints(t, t_points);
	}
	if(GetCompensateTPoints(t + input_compensate_t, t_points)){
		auto iter = t_points.begin();
		while(iter != t_points.end()){
			//index的p a_p未处理
			iter->index.s -= input_compensate_s;
			iter->index.t -= input_compensate_t;
			++iter;
		}
		rtvalue = true;
	}
	return rtvalue;
}

bool STmap::GetCost(const ST &point, double &v, double &cost) {
	cost = 0.0;
	if (point.t >= range_t_ || point.t < 0 || point.s >= range_s_ || point.s < 0 ||
			point.t >= t_map_.size()) {
		// AERROR_IF(FLAGS_log_enable)<<"input ST ("<<point.s<<", "<<point.t<<") out of map.";
		return false;
	}
	auto &t_info = t_map_.at(point.t);
	if (t_info.occupied_range.second < 0) return true;

	v = fmax(v, 2.0);
	int range = max(4, (int)(v / FLAGS_scale_s));
	int min_s = max((int)(point.s - range), 0);
	int max_s = min((int)(point.s + range), range_s_);
	int up_bound = min(max_s, t_info.occupied_range.second);
	int down_bound = max(min_s, t_info.occupied_range.first);
	// AERROR_IF(FLAGS_log_enable)<<"min_s "<<min_s<<" max_s "<<max_s
	// 			<<" occupied_range first "<<t_info.occupied_range.first
	// 			<<" occupied_range second "<<t_info.occupied_range.second
	// 			<<" down_bound "<<down_bound<<" up_bound "<<up_bound;
	// AERROR_IF(FLAGS_log_enable)<<"solid_occupied_range first"<<t_info.solid_occupied_range.first
	// 			<<" solid_occupied_range second "<<t_info.solid_occupied_range.second;
	// if (point.s >= t_info.solid_occupied_range.first &&
	// 		point.s <= t_info.solid_occupied_range.second) {
	// 	cost = 100.0;
	// }
	if (point.p > FLAGS_p_threshold) {
		cost = 100.0;
	}
	else {
		for (auto* st_point:t_info.t_points) {
			if (st_point->index.s <= down_bound || st_point->index.s >= up_bound) continue;
			if (st_point->index.s == point.s) continue;
			double delta_s = fabs((double)(point.s - st_point->index.s));
			cost += v * st_point->p / delta_s;
		}
	}
	// AERROR_IF(FLAGS_log_enable)<<"cost is "<<cost;
	return true;
}

void STmap::ExtrusionObjSTArea(LineObject &line_obj, double &input_a, double min_a, double max_a) {
	for (auto &pd_obj : line_obj.st_area.pd_objs) {
		pd_obj.st.clear();
		if (pd_obj.prediction_index < 0 || 
				pd_obj.prediction_index >= line_obj.prediction.trajectories.size() ||
				!pd_obj.update_flag) continue;
		auto &pd_line = line_obj.prediction.trajectories.at(pd_obj.prediction_index);
		if (pd_line.points.empty() || pd_line.range_pd_s.empty()) continue;
		double delta_a = fabs(input_a - pd_line.points.front().a);
		if (delta_a > FLAGS_max_pd_acc / 2.0 || fabs(input_a) > FLAGS_max_pd_acc / 2.0) {
			AWARN_IF(FLAGS_log_enable)<<"Input a isn't possible.";
			return;
		}
		int expand_state = 1; // 0不离散 1 正常挤压衰减历史 2 狠挤压不要历史
		if (input_a > pd_line.range_a.second + 0.01 || input_a < pd_line.range_a.first - 0.01) {
			expand_state = 2;
			double max_front_t;
			if (fabs(input_a) < 0.1) {
				max_front_t = pd_line.range_pd_s.front() / pd_line.points.front().v;
			} else {
				max_front_t = sqrt(2.0 * input_a * pd_line.range_pd_s.front() + 
													 pd_line.points.front().v * pd_line.points.front().v) / input_a - 
													 pd_line.points.front().v/input_a;
			}
			if (fabs(max_front_t - pd_line.st_boundary.front().first.y()) < 0.2) {
				AWARN_IF(FLAGS_log_enable)<<"extrusion delta t is small. old "<<pd_line.st_boundary.front().first.y()
							<<" new "<<max_front_t;
				continue;
			}
		}
		STDdiscret st_discret;
		st_discret.Londiscret(pd_line, input_a, min_a, max_a);
		OverlayObjProbability(pd_obj.history_st, pd_line.st_points, update_s_, update_t_, expand_state);
		pd_obj.st = pd_obj.history_st;
		for (auto &st_point : pd_obj.st) {
			if (st_point.t < 20) continue;
			double damping_p = st_point.p - fmax(FLAGS_p_thd_delete, st_point.p * 0.8);
			st_point.p -= (double)(st_point.t - 20)/20.0 * damping_p;
		}
	}
	ClearObjInSTMap(line_obj.id);
	SetObjectToSTMap(line_obj);
	GetTMap();
}

void STmap::DecisionObjProbability(LineObject &line_obj, double &p) {
	if (p > 1.0 || p < 0.0) {
		AWARN_IF(FLAGS_log_enable)<<"Input p "<<p<<" is invalid.";
	}
	if (current_st_map_ == nullptr) {
		return;
	}
	AWARN_IF(FLAGS_log_enable)<<"Input p "<<p<<" obj "<<line_obj.id;
	for (int t = 0; t < range_t_; t++) {
		for (int s = 0; s < range_s_; s++) {
			if (!current_st_map_[t][s].objs.count(line_obj.id)) continue;
			current_st_map_[t][s].objs.at(line_obj.id).p = p;
			current_st_map_[t][s].p = 0.0;
			for (auto &it : current_st_map_[t][s].objs) {
				if (current_st_map_[t][s].p < it.second.p && it.second.p < 1.0) {
					current_st_map_[t][s].p = it.second.p;
				}
			}
			// AINFO_IF(FLAGS_log_enable)<<"(s,t): ("<<s<<", "<<t<<" p "<<current_st_map_.at(t).at(s).p;
		}
	}
	GetTMap();
}

void STmap::ClearObjInSTMap(int &object_id) {
	if (current_st_map_ == nullptr) {
		return;
	}
	for (int t = 0; t < range_t_; t++) {
		for (int s = 0; s < range_s_; s++) {
			if (!current_st_map_[t][s].objs.count(object_id)) continue;
			current_st_map_[t][s].objs.erase(object_id);
			current_st_map_[t][s].p = 0.0;
			for (auto &it : current_st_map_[t][s].objs) {
				if (current_st_map_[t][s].p < it.second.p && it.second.p < 1.0) {
					current_st_map_[t][s].p = it.second.p;
				}
			}
		}
	}
}

void STmap::SetObjectToSTMap(LineObject &object) {
	if (current_st_map_ == nullptr) {
		return;
	}
	if (object.sl_boundary.max_s < car_model_ptr_->front_over_hang && object.type >= 2) {
		if (object.sl_boundary.max_l > -0.5*FLAGS_lane_width &&
				object.sl_boundary.min_l < 0.5*FLAGS_lane_width) {
			AERROR_IF(FLAGS_log_enable)<<"Back pedestrain or bike "<<object.id;
			return;
		}
	}
	if (!object.need_focus || object.age < 5 && object.sl_boundary.min_s > FLAGS_front_perception_range) {
		return;
	}
	// if (!object.need_focus ||  object.sl_boundary.min_s > FLAGS_front_perception_range) {
	// 	return;
	// }
	for (auto &pd_obj : object.st_area.pd_objs) {
		if (pd_obj.st.empty() || pd_obj.prediction_index < 0 ||
			pd_obj.prediction_index >= object.prediction.trajectories.size()) {
			pd_obj.st.clear();
			continue;
		}
		auto &pd_line = object.prediction.trajectories.at(pd_obj.prediction_index);
        //add by ly 1125
        int turn = 0;
        for (auto& lane : pd_line.lane_ids) {
            Id lane_id;
            lane_id.set_id(lane);
            Lane::LaneTurn turn_type;//NO_TURN = 1;LEFT_TURN = 2;RIGHT_TURN = 3;U_TURN = 4;
            if (vector_map_->GetLaneTurn(lane_id, turn_type) == 0) {
              turn = std::max((int)turn_type, turn);
            }
        }
        //预测车道序列中有掉头且车速超20/3.6=5.56 过滤掉
        if (4 == turn && object.speed > 5.56 && 2 == pd_obj.conflict_type) {
           AINFO_IF(FLAGS_log_enable) << "filter object prediction : "<<  object.id;
           pd_obj.st.clear();
           continue;
        }
        //预测车道序列中有左右转弯且车速超30/3.6=8.34且冲突类型是交叉冲突 过滤掉
        if ( (2 == turn || 3 == turn)  && object.speed > 8.34 && 5 == pd_obj.conflict_type){
           AINFO_IF(FLAGS_log_enable) << "filter object prediction : "<<  object.id;
           pd_obj.st.clear();
           continue;
        }
		bool is_side_start = NearToRoadBoundary();
		if (object.sl_boundary.max_s < car_model_ptr_->front_over_hang) {// 后方障碍物
			if (reference_lane_id_ < 40) {
				if (pd_obj.conflict_type == 1 || pd_obj.conflict_type == 3 ||
					pd_obj.conflict_type == 0 || pd_obj.conflict_type == 4) {
					if (!is_side_start) {
						AERROR_IF(FLAGS_log_enable && pd_obj.conflict_type == 1)<<"Back followingobj "<<object.id;
						AERROR_IF(FLAGS_log_enable && pd_obj.conflict_type == 3)<<"Back cutting_in obj "<<object.id;
						AERROR_IF(FLAGS_log_enable && pd_obj.conflict_type == 4)<<"Back parallel obj "<<object.id;
						pd_obj.st.clear();
						continue;
					}
				}
				if (pd_obj.conflict_type == 2 && CarInPdLanes(pd_obj.pd_id)) { // 如果2关系，自车三个点在对方车道内，去除
					AERROR_IF(FLAGS_log_enable)<<"Car is on obj "<<object.id<<" pd lane.";
					pd_obj.st.clear();
					continue;
				}
				if (pd_obj.conflict_type == 2 && pd_line.st_boundary.size() && pd_line.range_pd_s.size() &&
					pd_line.st_boundary.front().first.x() < 8.0 &&
					pd_line.range_pd_s.front() - pd_line.st_boundary.front().first.x() > 30.0) {
					AERROR_IF(FLAGS_log_enable)<<"merge obj "<<object.id<<" pd too near.";
					pd_obj.st.clear();
					continue;
				}
			} else if (pd_line.intentbylane == "Keep Current" && !is_side_start) {
				bool similar_lane_flag = false;
				for (auto &lane : line_data_ptr_->front_lane_ids) {// 和current相同
					for (auto &pd_lane : pd_line.lane_ids) {
						similar_lane_flag = (lane == pd_lane);
						if (similar_lane_flag) break;	
					}
					if (similar_lane_flag) break;
				}
				if (similar_lane_flag) {
					AERROR_IF(FLAGS_log_enable)<<"Local line obj "<<object.id<<" is keeping in current, ingore it.";
					pd_obj.st.clear();
					continue;
				}
			}
		}
		if (pd_line.st_boundary.size() >= 2) {
			double min_s = fmin(pd_line.st_boundary.front().first.x(), pd_line.st_boundary.back().first.x());
			if (min_s < FLAGS_collision_buff && reference_lane_id_ < 40) {
				/** 当障碍物第一次和当前路径干涉时，时间在1s 内，按照舒适的减速度刹车已经没有办法保证和其有4m 
				的距离，并且和路干涉时block住，此时为了保证安全不将其过滤
				*/
				if(!(object.block_first_conflict && 
					pow(car_speed_, 2) / 1.0 + FLAGS_collision_buff + 4.0 >  pd_line.st_boundary.front().first.x()
					&& pd_line.st_boundary.front().first.y() < 1.0)){
					AERROR_IF(FLAGS_log_enable)<<"Pd line has collision with car "<<object.id 
											  <<" brake dis:"<< pow(car_speed_, 2) / 1.0 + FLAGS_collision_buff 
											  <<", first_collision_min_s:"<< pd_line.st_boundary.front().first.x()
											  << " first_collision_time: "<< pd_line.st_boundary.front().first.y();
					pd_obj.st.clear();
					continue;
				}
			}
		}	

		for (auto &point : pd_obj.st) {//单条预测线st
			STPointObjectStruct point_obj;
			point_obj.p = point.p;
			point_obj.id = object.id; 
			point_obj.speed = object.speed;
			// STPointPdLineStruct point_pd_obj;
			// point_pd_obj.pd_id = pd_obj.pd_id;
			// point_pd_obj.a_p = point.a_p;
			// point_obj.pd_objs.push_back(point_pd_obj);
			if (point.t < range_t_ && point.t >= 0 &&
					point.s < range_s_ && point.s >= 0) {
				//current_st_map_由STMapPointStruct组成，包含一个或多个障碍物
				//Add操作会根据point_obj.id寻找相同障碍物，相同障碍物不同预测线则概率叠加
				//STMapPointStruct的概率为所有障碍物最大的概率
				current_st_map_[point.t][point.s].Add(point_obj, point);
			}
		}
	}
	// for (int i = 0; i < object.st_area.pd_objs.size(); i++) {
	// 	if (object.st_area.pd_objs.at(i).st.empty()) {
	// 		object.st_area.pd_objs.erase(object.st_area.pd_objs.begin() + i);
	// 		i--;
	// 	}
	// }
	auto iter = object.st_area.pd_objs.begin();
	while(iter != object.st_area.pd_objs.end()){
		if (iter->st.empty()) {
			iter = object.st_area.pd_objs.erase(iter);
			continue;
		}
		++iter;
	}
}

void STmap::LimitFollowingST(std::map<int, LineObject> &line_objs) {
	if (reference_lane_id_ >= 20 && reference_lane_id_ < 40) return;
	STDdiscret st_discret;
	for (auto &it : line_objs) {
		auto &line_obj = it.second;
		if (line_obj.is_static || line_obj.prediction.trajectories.empty() ||
			line_obj.type == 2 || line_obj.type >= 4) continue;
		if (line_obj.type == 3 && line_obj.prediction.trajectories.front().lane_ids.size() &&
				line_obj.prediction.trajectories.front().lane_ids.front() == "CYCLIST") continue;
		double value = car_model_ptr_->car_width/2.0+FLAGS_collision_buff;
		if (line_obj.sl_boundary.min_s < car_model_ptr_->front_over_hang || 
				line_obj.sl_boundary.min_l > value ||
				line_obj.sl_boundary.max_l < -value) continue;
		// 主要是挤压预测线的st_points; 如果是历史预测线生成的pd_obj，没有被匹配更新，那么在当时帧计算时应该已经被挤压过，这里不再进行计算
		for (auto &pd_obj : line_obj.st_area.pd_objs) {
			if (pd_obj.prediction_index < 0 || 
					pd_obj.prediction_index >= line_obj.prediction.trajectories.size() ||
					!pd_obj.update_flag) continue;
			auto &pd_line = line_obj.prediction.trajectories.at(pd_obj.prediction_index);
			if (pd_line.points.empty()) continue;
			if (pd_obj.conflict_type == 1) {
				double input_a = 0.0;
				AERROR_IF(FLAGS_log_enable)<<"Following obj id "<<line_obj.id;
				st_discret.Londiscret(pd_line, input_a, -0.1, 0.1, true);
				OverlayObjProbability(pd_obj.history_st, pd_line.st_points, update_s_, update_t_);
				pd_obj.st = pd_obj.history_st;
				for (auto &st_point : pd_obj.st) {
					if (st_point.t < 20) continue;
					double damping_p = st_point.p - fmax(FLAGS_p_thd_delete, st_point.p * 0.8);
					st_point.p -= (double)(st_point.t - 20)/20.0 * damping_p;
				}
			}
		}
	}
}

bool STmap::CarInPdLanes(vector<string> &pd_lanes) {
	bool car_in_pd_lane = false;
	auto &front_l = car_model_ptr_->front_over_hang;
	auto &back_l = car_model_ptr_->back_over_hang;
	double pi_angle = loc_ptr_->globalangle * M_PI / 180.0;
	Vec2d center = Vec2d((front_l - back_l)/2.0, 0.0).rotate(pi_angle) +
						 			Vec2d(loc_ptr_->xg, loc_ptr_->yg);
	Box2d car_box(center, pi_angle, car_model_ptr_->length, car_model_ptr_->car_width);
	vector<Vec2d> car_corners;
	car_box.GetAllCorners(&car_corners);
	for (auto &lane : pd_lanes) {
		int num = 0;
		LaneInfoConstPtr lane_ptr = vector_map_->GetLaneById(lane);
		if (lane_ptr == nullptr) continue;
		for (auto &corner : car_corners) {
			if (lane_ptr->IsOnLane(corner)) {
				num++;
			}
			if (num >= 2) car_in_pd_lane = true;
		}
		if (car_in_pd_lane) break;
	}
	return car_in_pd_lane;
}

bool STmap::DevideRelation(vector<string> &pd_lanes) {
	if (line_data_ptr_->front_lane_ids.empty() || pd_lanes.empty()) {
		return false;
	} 
	auto &current_lane = line_data_ptr_->front_lane_ids.front();
	LaneInfoConstPtr lane_ptr = vector_map_->GetLaneById(current_lane);
	if (lane_ptr == nullptr) return false;
	vector<string> devide_lanes;
	const vector<OverlapInfoConstPtr> crosses_ptr = lane_ptr->cross_lanes();
	for (auto &cross_ptr : crosses_ptr) {
		if (cross_ptr->overlap().object().size() != 2) continue;
		for (auto &object : cross_ptr->overlap().object()) {
			if (object.id().id() != lane_ptr->id().id() &&
      			!object.lane_overlap_info().is_merge()) {
				devide_lanes.push_back(object.id().id());
			}
		}
	}
	for (auto &devide_lane :devide_lanes) {
		for (auto &pd_lane : pd_lanes) {
			if (pd_lane == devide_lane) return true;
		}
	}
	return false;
}

bool STmap::NearToRoadBoundary() {
	if (reference_lane_id_ >= 20 || line_data_ptr_->front_lane_ids.empty()) return false;
	LaneInfoConstPtr current_ptr = vector_map_->GetLaneById(line_data_ptr_->front_lane_ids.front());
	if (current_ptr == nullptr) return false;
	PointENU loc_enu_point;
	loc_enu_point.set_x(loc_ptr_->xg);
	loc_enu_point.set_y(loc_ptr_->yg);
	double left_w, right_w;
	if (vector_map_->GetWidthToBoundary(loc_enu_point, left_w, right_w) < 0) return false;
	if (right_w > 0.0) return false;
	string current_road;
	if (vector_map_->GetRoad(line_data_ptr_->front_lane_ids.front(), current_road) == 0) {
		vector<string> road_lanes;
		if (vector_map_->GetRoadLanes(current_road, road_lanes) == 0 && road_lanes.size() > 0 && 
				road_lanes.back() == line_data_ptr_->front_lane_ids.front()) {
			return true;
		}
	}
	return false;
}



}
}
