#include "dynamic_sl_generation.h"

namespace acu {
namespace planning {

DynamicSLGeneration::DynamicSLGeneration() {
	Reset();
}

DynamicSLGeneration::~DynamicSLGeneration() {}

void DynamicSLGeneration::Reset() {
  	dynamic_obstacle_boxes_.clear();
  	dynamic_obstacle_boxes_in_time_.clear();//存储所有box
  	result_dynamic_obstacle_boxes_.clear();//存储所有发生干涉的box
  	enable_debug_ = FLAGS_enable_dynamic_sl_debug_plot;
	//时间维度参数
	num_of_time_stamps_ = 0;
	eval_time_interval_ = 0.1;
	total_time_ = 7.0;
	context_->decision_result_.dynamic_sl.clear();
	st_sampling_ptr_ = std::make_shared<STSampling>();
}

//API
int DynamicSLGeneration::CalculateDynamicBoxesWithCollision(){
	int rtvalue = -1;
	//获取目标参考线
	auto iter = context_->reference_line_map_.find(
		context_->cognition_info_->reference_line_info.current_line_id);
	if(iter != context_->reference_line_map_.end()){
		current_line_ = iter->second;
	}else{
		AERROR_IF(enable_debug_)<<"cannot find current_line_";
		return rtvalue;
	}
	if(-1 == SortObstacles()) {
		AERROR_IF(enable_debug_)<<"SortObstacles fail";
		return rtvalue;
	}
	if(-1 == OrganizeDynamicObsInTime()) {
		AERROR_IF(enable_debug_)<<"OrganizeDynamicObsInTime fail";
		return rtvalue;
	}
	//检查上一帧路径
	if(-1 == CheckCollisionAndTrimBoxSet(context_->trajectory_info_.path)) {
		// //加速度采样得到其他采样路径
		// std::vector<std::list<Site>> path_vec;
		// if(-1 != st_sampling_ptr_->SampleDECCToGeneratePaths(context_->trajectory_info_.path, path_vec)) {
		// 	//TODO
		// 	for(const auto& path : path_vec){
		// 		//会修改result_dynamic_obstacle_boxes_，如需保留多个，需要及时存储
		// 		if(-1 == CheckCollisionAndTrimBoxSet(path)) {continue;}
		// 	}
		// 	//TODO
		// }
		AERROR_IF(enable_debug_)<<"CheckCollisionAndTrimBoxSet fail";
		return rtvalue;
	}
	AINFO_IF(enable_debug_)<<"GetResultDynamicObstacleBoxes: "<<GetResultDynamicObstacleBoxes().size();
	if(GetResultDynamicObstacleBoxes().size() > 0) {rtvalue = 0;}
	context_->decision_result_.dynamic_sl = GetResultDynamicObstacleBoxes();
	return rtvalue;
}



int DynamicSLGeneration::SortObstacles(){
	int rtvalue;
	AINFO_IF(enable_debug_)<<"decision get: "<<context_->decision_result_.meeting_ids.size()<<" meeting_ids";
	//也许会需要考虑其他frame?预留接口
	if(-1 == SortObstaclesOfCertainFrame(current_line_)) {return rtvalue;}
	rtvalue = 0;
	return rtvalue;
}

//筛选line_frame_ptr上的障碍物，并存储至dynamic_obstacle_boxes_中
int DynamicSLGeneration::SortObstaclesOfCertainFrame(const ReferenceLineFrame* line_frame_ptr){
	int rtvalue = -1;
	if(!line_frame_ptr) {return rtvalue;}
	//自车尺寸
  	const auto& vehicle_param = acu::planning::DataPool::Instance()->GetMainDataPtr()->config_info.car_model;
	const auto back_edge_to_center = vehicle_param.back_over_hang;
	const auto front_edge_to_center = vehicle_param.length - back_edge_to_center;
	//根据自车尺寸更新可以忽略障碍物的s距离
	const auto ignore_s_min = front_edge_to_center;
	const auto ignore_s_max = 2 * context_->ego_speed_;//忽略两倍车速以外的障碍物
	//巡航速度
	auto cruise_speed = 8.33;
	if(!line_frame_ptr->mapinfo.expected_speeds.empty()){
		cruise_speed = line_frame_ptr->mapinfo.expected_speeds.front().second;
	}
	const double speed_buffer = 0.05;
	const double ignore_speed = (1.0 + speed_buffer) * cruise_speed;//忽略巡航速度以上的障碍物
	const bool use_self_filter = false;
	//时间维度参数在头文件中给了初值，与规划环节一致
	num_of_time_stamps_ = static_cast<uint32_t>(
	                      std::floor(total_time_ / eval_time_interval_));
	//遍历障碍物
	auto ptr_obstacle_pair = line_frame_ptr->objects_.begin();
	for (;ptr_obstacle_pair != line_frame_ptr->objects_.end(); ++ptr_obstacle_pair) {
		const int obs_id = ptr_obstacle_pair->first;
		const auto* ptr_obstacle = &ptr_obstacle_pair->second;
		//只使用会车目标障碍物
		if(!context_->decision_result_.meeting_ids.count(obs_id)) {continue;}
		if(use_self_filter){
			//1.要求障碍物存在stmap，否则认为动态障碍物与目标参考线没有交集，无需考虑
			if(ptr_obstacle->st_area.pd_objs.empty()){
				AINFO_IF(enable_debug_)<<"Ignore obs for no-st_area "<<obs_id;
				continue;
			}
			//2.要求障碍物S方向在一定区域内
			const auto &sl_boundary = ptr_obstacle->sl_boundary;
			if (sl_boundary.max_s <= ignore_s_min 
			  || sl_boundary.min_s >= ignore_s_max 
			  || sl_boundary.min_l*sl_boundary.max_l < 0//为增加规划成功率，不考虑横跨路线的障碍物
			  || ptr_obstacle->speed >= ignore_speed) {
			  AINFO_IF(enable_debug_)<<"Ignore obs for invalid s or speed: "<<obs_id
			    <<" | "<<(sl_boundary.max_s <= ignore_s_min)
			    <<" | "<<(sl_boundary.min_s >= ignore_s_max)
			    <<" | "<<(sl_boundary.min_l*sl_boundary.max_l < 0)
			    <<" | "<<(ptr_obstacle->speed >= ignore_speed);
			  continue;
			}
			//3.要求障碍物的l在一定范围以内
			const auto min_l = std::fmin(fabs(sl_boundary.min_l), fabs(sl_boundary.max_l));
			const auto consideration_min_l = 5.0;//忽略l过大的障碍物
			if (consideration_min_l < min_l) {
			  AINFO_IF(enable_debug_)<<"Ignore obs for invalid l: "<<obs_id
			    <<"min_l: "<<min_l<<" > "<<consideration_min_l;
			  continue;
			}
		}
		//剩余为通过筛选的障碍物
		//在预测的结果中，选择一个概率最高的预测轨迹
    	double probability = -1;
	    int path_count = 0;
    	int trajectory_index = -1;
	    for(const auto &predict_path : ptr_obstacle->prediction.trajectories) {
	      if (predict_path.probability > probability) {
	        probability = predict_path.probability;
	        trajectory_index = path_count;
	      }
	      path_count++;
	    }
	    //得到概率最高的轨迹索引
	    PredictionTrajectory obj_predict_trajectory;
        if(trajectory_index >= 0 && trajectory_index < ptr_obstacle->prediction.trajectories.size()) {
        	obj_predict_trajectory = ptr_obstacle->prediction.trajectories.at(trajectory_index);
        }
		if (obj_predict_trajectory.points.empty()) {
		  AINFO_IF(enable_debug_)<<"Ignore obs for invalid obj_predict_trajectory: "<<obs_id;
		  continue;
		}
		//根据预测轨迹，生成一系列box
		double predition_time_max = obj_predict_trajectory.points.back().t;
		std::vector<Box2d> box_by_time;//同一个障碍物，按照预测时间的一系列box
		for (uint32_t t = 0; t <= num_of_time_stamps_; ++t) {
			if (t * eval_time_interval_ > predition_time_max) {
			  break;
			}
			//获取t * eval_time_interval_的预测位置
			PredictionPoint predict_point = obj_predict_trajectory.GetPredictionPointAtTime(t * eval_time_interval_);
			//根据预测位置生成box
			Box2d predict_box = Box2d({predict_point.xg, predict_point.yg},
	                             predict_point.globalangle * 3.14 / 180.0,
	                             ptr_obstacle->box.length(),
	                             ptr_obstacle->box.width());
			//将预测轨迹上的box进行一些膨胀
			const double kBuff = 0.5;
			Box2d expanded_predict_box = Box2d(predict_box.center(), predict_box.heading(),
			        							predict_box.length() + kBuff, predict_box.width() + kBuff);
			box_by_time.emplace_back(expanded_predict_box);
		}
		rtvalue = 0;
		dynamic_obstacle_boxes_.emplace_back(std::move(box_by_time));
	}
	AINFO_IF(enable_debug_)<<"dynamic_obstacle_boxes_.size = "<<dynamic_obstacle_boxes_.size();
	return rtvalue;
}

//根据筛选障碍物的结果，按照相同时间内所有障碍物的形式组织
int DynamicSLGeneration::OrganizeDynamicObsInTime(){
	int rtvalue = -1;
	if(dynamic_obstacle_boxes_.empty()) {return rtvalue;}
	//时间维度，对应dynamic_obstacle_boxes_的内部存储顺序，
	//个数不超过num_of_time_stamps_，乘eval_time_interval_后表示真实时间
	for(int time_index = 0; time_index <= num_of_time_stamps_; ++time_index){
	std::vector<common::math::Box2d> all_obs_in_same_time;
	for(int i = 0; i < dynamic_obstacle_boxes_.size(); ++i){
	  const auto& single_obs_all_time = dynamic_obstacle_boxes_.at(i);
	  if(time_index < single_obs_all_time.size()){
	    //当前障碍物的预测时间超过time_index*eval_time_interval_
	    all_obs_in_same_time.emplace_back(single_obs_all_time.at(time_index));
	  }else{
	    continue;
	  }
	}
		dynamic_obstacle_boxes_in_time_.emplace_back(all_obs_in_same_time);
	}
	DCHECK_EQ(dynamic_obstacle_boxes_in_time_.size(), num_of_time_stamps_);
	rtvalue = 0;
	return rtvalue;
}

//遍历动态障碍物-时间集合，寻找自车某个给定轨迹行驶时，是否会与动态障碍物在某个时间T、某个S附近干涉
//根据干涉结果删减动态障碍物-时间集合，得到所有发生干涉的box结果
int DynamicSLGeneration::CheckCollisionAndTrimBoxSet(const std::list<Site>& given_path){
	int rtvalue = -1;
	//合理性检查
	if(dynamic_obstacle_boxes_in_time_.empty()){
		AWARN_IF(enable_debug_)<<"empty dynamic_obstacle_boxes_in_time_, no need to keep on";
		return rtvalue;    
	}
	AINFO_IF(enable_debug_)<<"there are: "<<dynamic_obstacle_boxes_in_time_.size()<<" box in time";
	if(given_path.size() < 1){
		AERROR_IF(enable_debug_)<<"invalid given_path size: "<<given_path.size();
		return rtvalue;
	}
	AINFO_IF(enable_debug_)<<"there are: "<<given_path.size()<<" points of given_path";
	//自车尺寸
  	const auto& vehicle_param = acu::planning::DataPool::Instance()->GetMainDataPtr()->config_info.car_model;
	const auto back_edge_to_center = vehicle_param.back_over_hang;
	const auto front_edge_to_center = vehicle_param.length - back_edge_to_center;
	//记录寻找到的路点索引，由于路径时间单调性，每循环匹配路径最近时间路点时，可以基于上一帧匹配结果继续进行
	int last_path_index = 0;
	int last_active_time_index = dynamic_obstacle_boxes_in_time_.size();//记录车辆沿路径，有速度时的最后的索引time_index
	double active_assumtion_ego_s = 0.0;//记录车辆静止前，行驶的最远s距离
	double active_assumtion_ego_l = 0.0;//记录车辆静止前，行驶的最远l距离
	//遍历dynamic_obstacle_boxes_in_time_,在给定路径上找到time_index*eval_time_interval_对应的自车s值
	for(int time_index = 0; time_index < dynamic_obstacle_boxes_in_time_.size(); ++time_index){
		auto time = time_index * eval_time_interval_;
		if(time_index < last_active_time_index){
			auto min_time_bias = MatchTimeIndexBeginWithIndex<Site, std::list<Site>>(last_path_index, time, given_path);
			if(min_time_bias < 1.0){
				// AINFO_IF(enable_debug_)<<"find match time for: "<<time<<", min bias is: "
				// 	<<min_time_bias<<", last_path_index: "<<last_path_index;
			}else{
				AINFO_IF(enable_debug_)<<"cannot match time for: "<<time<<", min bias is: "<<min_time_bias;
				// auto iter_time_index = std::next(dynamic_obstacle_boxes_in_time_.begin(), time_index);
				// dynamic_obstacle_boxes_in_time_.earse(iter_time_index);
				continue;
			}
		}else{//自车停止后，last_path_index保持最后的位置
		}
		auto iter = std::next(given_path.begin(), last_path_index);
		if(iter == given_path.end()) {continue;}
		//没有找到速度为0的索引时，检查速度过低的位置
		if(last_active_time_index == dynamic_obstacle_boxes_in_time_.size()){
			if(fabs(iter->velocity) < 0.05){//路径在该时间后静止，不可能出现在更远的S位置
				AWARN_IF(enable_debug_)<<"given_path is still after time: "<<time;
				last_active_time_index = time_index;//路径静止时的时间索引
			}else{
				//按照给定路径行驶，假定车辆到达的位置
				double assumtion_ego_s, assumtion_ego_l;
				if(!XYToSL(current_line_->mapinfo, *iter, assumtion_ego_s, assumtion_ego_l)){
					AERROR_IF(enable_debug_)<<"XYToSL fail";
					continue;
				}
				//只在存在速度时，更新自车可能出现的位置
				active_assumtion_ego_s = assumtion_ego_s;
				active_assumtion_ego_l = assumtion_ego_l;
			}
		}else{//速度为0的索引之后，不再累积自车行驶距离
		}

		auto assumtion_ego_end_s = active_assumtion_ego_s + front_edge_to_center + 10.0;
		auto assumtion_ego_start_s = active_assumtion_ego_s - back_edge_to_center - 10.0;
		for(const auto& single_obs_at_time_index : dynamic_obstacle_boxes_in_time_.at(time_index)){
			StructSLBoundary sl_boundary;
			if(!BoxToSL(current_line_->mapinfo, single_obs_at_time_index, sl_boundary)){
				AERROR_IF(enable_debug_)<<"BoxToSL fail";
				continue;
			}
			//自车按照上一帧规划的速度行驶至assumtion_ego_s时，s方向上是否会与障碍物相交
			if(sl_boundary.max_s < assumtion_ego_start_s || sl_boundary.min_s > assumtion_ego_end_s) {
			// if(sl_boundary.max_s < assumtion_ego_start_s) {
				AINFO_IF(enable_debug_)
					<<"ignore obs for no-overlap, sl: "<<sl_boundary.max_s<<", "<<sl_boundary.min_s
					<<", ego_s: "<<assumtion_ego_start_s<<", "<<assumtion_ego_end_s;
				continue;
			}else if(fabs(active_assumtion_ego_l - sl_boundary.min_l)>5.0 && 
					 fabs(active_assumtion_ego_l - sl_boundary.max_l)>5.0){
				//自车按照上一帧规划的速度行驶至assumtion_ego_s时，l方向上是否会与障碍物相交，不是太离谱的都留下了
				AINFO_IF(enable_debug_)
					<<"ignore obs for no-overlap, sl: "<<sl_boundary.max_s<<", "<<sl_boundary.min_s
					<<", active_assumtion_ego_l: "<<active_assumtion_ego_l;
				continue;
			}else{
				rtvalue = 0;
				// AINFO_IF(enable_debug_)<<"push valid obs";
				result_dynamic_obstacle_boxes_.emplace_back(single_obs_at_time_index);
			}
		}
	}
	return rtvalue;
}






}  //  namespace planning
}  //  namespace acu
