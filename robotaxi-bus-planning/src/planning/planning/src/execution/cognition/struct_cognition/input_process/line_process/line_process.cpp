#include "line_process.h"
 
namespace acu{ 
namespace planning {

LineProcess::LineProcess(const MapInfoData &mapengine_data, 
							   const vector<string> &last_current_lanes,
							   const vector<string> &last_target_lanes,
							   const PathData &motionpath,
							   const int drive_state, 
							   LocalizationData &localization, CarModel &car_model) 
{
	task_change_ = mapengine_data.task_change;
	mapengine_index_ = mapengine_data.index;
	all_lines_ptr_ = &mapengine_data.alllinelists;
	rev_line_ptr_ = &mapengine_data.revlinelist;
	motionpath_ptr_ = &motionpath;
	localization_ = localization;
	car_model_ = car_model;
	drive_state_ = drive_state;
	reference_lane_ids_.clear();
	reference_target_ids_.clear();
	reference_lane_ids_.assign(last_current_lanes.begin(), last_current_lanes.end());
	reference_target_ids_.assign(last_target_lanes.begin(), last_target_lanes.end());
	if (!last_current_lanes.empty() && !last_target_lanes.empty() &&
		last_current_lanes.front() == last_target_lanes.front()) {
		reference_lane_ids_.assign(last_target_lanes.begin(), last_target_lanes.end());
	}
	if (reference_lane_ids_.empty()) {
		reference_target_ids_.clear();
	}
	if (FLAGS_log_enable) {
		for (auto &map_line : *all_lines_ptr_) {
			std::cout<<"mapengine input ";
			for (auto &lane : map_line.front_lane_ids) {
				std::cout<<lane<<"\t";
			}
			std::cout<<std::endl;
		}
		std::cout<<"reference_lane_ids_ are ";
		for(auto &lane : reference_lane_ids_) {
			std::cout<<lane<<"\t";
		}
		std::cout<<std::endl;
		std::cout<<"reference_target_ids_ are ";
		for(auto &lane : reference_target_ids_) {
			std::cout<<lane<<"\t";
		}
		std::cout<<std::endl;
	}
}

bool LineProcess::CorrectionProcess(int &current_index, int &target_index) {
	correct_index_ = -1;
	target_index_ = -1;
	current_index = -1;
	target_index = -1;
	if (all_lines_ptr_->empty()) {
		return false;
	}
	if (drive_state_ != 1) {
		AWARN_IF(FLAGS_log_enable)<<"cognition manul mode or start light.";
  		correct_index_ = mapengine_index_;
  		target_index_ = mapengine_index_;
  		current_index = mapengine_index_;
  		target_index = mapengine_index_;
  		return true;
  	}

	if (reference_lane_ids_.empty() || task_change_) {// 第一帧，没有参考
		AERROR_IF(FLAGS_log_enable) <<"init first time, no reference lane ids "<<mapengine_index_;
		correct_index_ = mapengine_index_;
		target_index_ = mapengine_index_;
		current_index = mapengine_index_;
  	target_index = mapengine_index_;
  	return true;
	}
	
	int current_min_cost = std::numeric_limits<int>::max();
	for (int i = 0 ; i < all_lines_ptr_->size(); i++) {
		if (!all_lines_ptr_->at(i).able_driving) continue;
		if (LaneIdsMatching(all_lines_ptr_->at(i).front_lane_ids, reference_lane_ids_) &&
			all_lines_ptr_->at(i).global_cost < current_min_cost) {
			correct_index_ = i;
			current_min_cost = all_lines_ptr_->at(i).global_cost;
		}
	}
	current_min_cost = std::numeric_limits<int>::max();
	for (int i = 0; i < all_lines_ptr_->size(); i++) {
		if (!all_lines_ptr_->at(i).able_driving) continue;
		if (LaneIdsMatching(all_lines_ptr_->at(i).front_lane_ids, reference_target_ids_) &&
			all_lines_ptr_->at(i).global_cost < current_min_cost) {
			target_index_ = i;
			current_min_cost = all_lines_ptr_->at(i).global_cost;
		}
	}
	CheckArriveTargetLine();
	if (correct_index_ < 0 && mapengine_index_ >= 0) {
		correct_index_ = mapengine_index_;
	}
	if (target_index_ < 0 && mapengine_index_ >= 0) {
		target_index_ = mapengine_index_;
	}
	current_index = correct_index_;
	target_index = target_index_;
	if (current_index < 0 || target_index < 0) {
		return false;
	}
	return true;
}

 bool LineProcess::LaneIdsMatching(vector<string> current_lanes, vector<string> last_lanes) {
 	if (last_lanes.empty() || current_lanes.empty()) {
 		return false;
 	}
 	if (last_lanes.size() == 1) {
 		return (current_lanes.front() == last_lanes.front());
 	}

 	int index = -1;
 	for (int i = 0; i < last_lanes.size(); i++) {
         if (LaneToRoad(last_lanes.at(i)) == LaneToRoad(current_lanes.front()) && 
             current_lanes.front() != "") {
             index = i;
             break;
         }
     }
     if (index > 0 && index < last_lanes.size()) {
         last_lanes.erase(last_lanes.begin(), last_lanes.begin() + index);
     }
     if (index < 0 || current_lanes.size() < last_lanes.size()) return false;
     index = -1;
     for (int i = 0; i < last_lanes.size(); i++) {
         if (last_lanes.at(i) != current_lanes.at(i)) {
             return false;
         }
     }
     return true;
}

string LineProcess::LaneToRoad(string &lane) {
    string road = "";
    for (auto &p : lane) {
        if (p == '_') break;
        road.push_back(p);
    }
    return road;
}

void LineProcess::CheckArriveTargetLine() {
	if (target_index_ == correct_index_ || all_lines_ptr_->empty() ||
			correct_index_ < 0 || correct_index_ >= all_lines_ptr_->size() ||
			target_index_ < 0 || target_index_ >= all_lines_ptr_->size()) {
		AERROR_IF(correct_index_ < 0)<<"Can't find last current line, change into target.";
		if (target_index_ >= 0) {
			correct_index_ = target_index_;
		}
		return;
	}
	ReferenceLineFrame correct_line, target_line;
	correct_line.mapinfo = all_lines_ptr_->at(correct_index_);
	target_line.mapinfo = all_lines_ptr_->at(target_index_);
	if (correct_line.mapinfo.front_lane_ids.size() && target_line.mapinfo.front_lane_ids.size() &&
			correct_line.mapinfo.front_lane_ids.front() == target_line.mapinfo.front_lane_ids.front()) {
		correct_index_ = target_index_;
		AERROR_IF(FLAGS_log_enable)<<"Divide lane, arrive at target line.";
		return;
	}
	if ( correct_line.mapinfo.path_points.size() < 5) {
		correct_index_ = target_index_;
		AERROR_IF(FLAGS_log_enable)<<"Correction is too short, arrive at target line.";
		return;
	}
	bool path_in_target = true;
  double length = -1.1;
  double left_w = 0.0, right_w = 0.0;
  for (auto &point : motionpath_ptr_->path) {
    if (point.length - length < 1.0) continue;
    length = point.length;
    if (target_line.GetWidthToLaneBoundary(left_w, right_w, point) &&
    		(right_w < -FLAGS_boundary_buff || left_w < -FLAGS_boundary_buff)) {
      AWARN_IF(FLAGS_log_enable)<<"length "<<length<<" left_w "<<left_w<<" right_w "<<right_w;
    	if (target_line.mapinfo.path_points.empty() || 
          target_line.mapinfo.path_points.back().length < 15.0 &&
          point.length < 1.0) continue;// 目标车道很短，只剩刚开始几个点没到
      path_in_target = false;
      break;
    }
  }
  if (path_in_target && motionpath_ptr_->path.size() > 0 &&
  		correct_line.GetWidthToLaneBoundary(left_w, right_w, motionpath_ptr_->path.front()) &&
  		right_w > FLAGS_boundary_buff && left_w > FLAGS_boundary_buff) {// 仍有点在原始车道上
  	AWARN_IF(FLAGS_log_enable)<<"still has point in origin "<<correct_index_;
  	path_in_target = false;
  }
  if (path_in_target) {
  	correct_index_ = target_index_;
  }
}

void LineProcess::LineClassification(StructReferenceLineInfo &reference_lines) {
	auto &current_reference_line = reference_lines.current_reference_line;
	auto &left_reference_line = reference_lines.left_reference_line;
	auto &right_reference_line = reference_lines.right_reference_line;
	auto &reverse_reference_line = reference_lines.reverse_reference_line;
	// auto &rightest_reference_line = reference_lines.rightest_reference_line;
	auto &current_line_id = reference_lines.current_line_id;
	auto &target_line_id = reference_lines.target_line_id;
	target_line_id = -1;
	current_line_id = -1;
	RevLineFrame(reverse_reference_line);
	if (all_lines_ptr_->empty() || correct_index_ < 0 || target_index_ < 0) {
		AERROR_IF(FLAGS_log_enable)<<"all_frames_ size "<<all_lines_ptr_->size()
				<<" correct_index_ "<<correct_index_<<" target_index_ "<<target_index_;
		return;
	}
	vector<pair<int, int> > line_devides;// <区间起始index，终止index>
	pair<int, int> temp_devide(0, 0);
	line_devides.push_back(temp_devide);
	for (int i = 1; i  < all_lines_ptr_->size(); i++) {
		if (all_lines_ptr_->at(i).front_lane_ids.front() == 
			all_lines_ptr_->at(i-1).front_lane_ids.front()) {
			line_devides.back().second = i;
		}
		else {
			temp_devide.first = i;
			temp_devide.second = i;
			line_devides.push_back(temp_devide);
		}
	}
	pair<int, int> left_line_index(-1, -1);
	pair<int, int> current_line_index(-1, -1);
	pair<int, int> right_line_index(-1, -1);
	for (int i = 0; i < line_devides.size(); i++) {
		if (correct_index_ > line_devides.at(i).second) {
			continue;
		}
		if (i > 0 ) {
			left_line_index = line_devides.at(i-1);
		}
		current_line_index = line_devides.at(i);
		if (i + 1 < line_devides.size()) {
			right_line_index = line_devides.at(i+1);
		}
		break;
	}
	if ((target_index_ < left_line_index.first || 
			 target_index_ > right_line_index.second &&
			 right_line_index.second > -1) && target_index_ >= 0) {// 强制到达目标车道，重新划分
		AERROR_IF(FLAGS_log_enable)<<"target line is not in current/left/right.";
		correct_index_ = target_index_;
		for (int i = 0; i < line_devides.size(); i++) {
			if (correct_index_ > line_devides.at(i).second) {
				continue;
			}
			if (i > 0 ) {
				left_line_index = line_devides.at(i-1);
			}
			current_line_index = line_devides.at(i);
			if (i + 1 < line_devides.size()) {
				right_line_index = line_devides.at(i+1);
			}
			break;
		}
	}
	AERROR_IF(FLAGS_log_enable) <<"left index "<<left_line_index.first
		<<", "<<left_line_index.second<<", current index "<<current_line_index.first
		<<", "<<current_line_index.second<<", right index "<<right_line_index.first
		<<", "<<right_line_index.second;
	
	if (left_line_index.first >= 0 && left_line_index.second < all_lines_ptr_->size()) {// 车道组合计算
		MatchingFrame(left_reference_line, left_line_index);
		for (int i = left_line_index.first; i <= left_line_index.second; i++) {
			int delta = i - left_line_index.first;
			left_reference_line.at(delta).reference_lane_id = 20 + delta;
			target_line_id = (i == target_index_)? (20 + delta) : target_line_id;
		}
	} else {
		left_reference_line.clear();
	}
	reference_lines.rightest_line_id = -1;
	if (current_line_index.first >= 0 && current_line_index.second < all_lines_ptr_->size()) {// 车道组合计算
		MatchingFrame(current_reference_line, current_line_index);
		for (int i = current_line_index.first; i <= current_line_index.second; i++) {
			int delta = i - current_line_index.first;
			current_reference_line.at(delta).reference_lane_id = 10 + delta;
			current_line_id = (i == correct_index_)? (10 + delta) : current_line_id;
			target_line_id = (i == target_index_)? (10 + delta) : target_line_id;
			reference_lines.rightest_line_id = 10 + delta;
		}
	} else {
		current_reference_line.clear();
	}

	if (right_line_index.first >= 0 && right_line_index.second < all_lines_ptr_->size()) {// 车道组合计算
		MatchingFrame(right_reference_line, right_line_index);
		for (int i = right_line_index.first; i <= right_line_index.second; i++) {
			int delta = i - right_line_index.first;
			right_reference_line.at(delta).reference_lane_id = 30 + delta;
			target_line_id = (i == target_index_)? (30 + delta) : target_line_id;
			reference_lines.rightest_line_id = 30 + delta;
		}
	} else {
		right_reference_line.clear();
	}
}

void LineProcess::MatchingFrame(vector<ReferenceLineFrame>& last_frames, const pair<int, int> &index_range) {
	if (all_lines_ptr_->empty()) {
		last_frames.clear();
		return;
	}
	for (auto &last_frame : last_frames) {
		last_frame.matched_history = false;
	}
	
	vector<bool> matched_flags;
	for (int i = max(index_range.first, 0); i <= index_range.second; i++) {
		if (i >= all_lines_ptr_->size()) continue;
		matched_flags.push_back(false);
		for (int j = 0; j < last_frames.size(); j++) {
			if (!last_frames.at(j).matched_history &&
					LaneIdsMatching(all_lines_ptr_->at(i).front_lane_ids, 
											last_frames.at(j).mapinfo.front_lane_ids)) {
				last_frames.at(j).SetData(all_lines_ptr_->at(i), localization_, car_model_);
				matched_flags.back() = true;
				break;
			}
		}
	}

	for (int i = 0; i < last_frames.size(); i++) {// 去掉未更新历史frame
		if (!last_frames.at(i).matched_history) {
			last_frames.erase(last_frames.begin() + i);
			i--;
		}
	}

	for (int i = max(index_range.first, 0); i <= index_range.second; i++) {// 插入新增分流车道
		if (i >= all_lines_ptr_->size()) continue;
		if (matched_flags.at(i-max(index_range.first, 0))) continue;
		int devide_lon_index = -1;
		int delta_lat_index = 0;
		int reference_index = -1;
		for (int j = 0; j < last_frames.size(); j++) {
			int temp_devide_index = 0, temp_delta_index = 0;
			if (IsDiversionLine(all_lines_ptr_->at(i).front_lane_ids, 
													last_frames.at(j).mapinfo.front_lane_ids, 
													temp_delta_index, temp_devide_index) &&
					temp_devide_index > devide_lon_index) {
				devide_lon_index = temp_devide_index;
				delta_lat_index = temp_delta_index;
				reference_index = j;
			}
		}
		if (devide_lon_index < 0 || reference_index < 0) continue;
		if (fabs(delta_lat_index) != 1) continue;
		int insert_index = (delta_lat_index < 0)? reference_index : reference_index + 1;
		if (insert_index < last_frames.size()) {
			last_frames.insert(last_frames.begin() + insert_index, last_frames.at(reference_index));
		} else {
			last_frames.push_back(last_frames.at(reference_index));
		}
		last_frames.at(insert_index).SetData(all_lines_ptr_->at(i), localization_, car_model_);
		matched_flags.at(i-max(index_range.first, 0)) = true;
	}

	for (int i = max(index_range.first, 0); i <= index_range.second; i++) {// 插入新增边缘车道
		if (i >= all_lines_ptr_->size()) continue;
		int insert_index = i - max(index_range.first, 0);
		if (matched_flags.at(insert_index)) continue;
		ReferenceLineFrame temp_frame;
		if (temp_frame.CalculationFrame(all_lines_ptr_->at(i), localization_, car_model_)) {
			if (insert_index < last_frames.size()) {
				last_frames.insert(last_frames.begin() + insert_index, temp_frame);
			} else {
				last_frames.push_back(temp_frame);
			}
		}		
	}

}

bool LineProcess::IsDiversionLine(vector<string> current_lanes, vector<string> last_lanes, 
																		 int &delta_index, int &devide_index) {
	if (last_lanes.empty() || current_lanes.empty()) {
		return false;
	}
	if (last_lanes.size() == 1 && current_lanes.size() == 1) {
		return false;
	}
	if (current_lanes.front() != last_lanes.front()) {
		last_lanes.erase(last_lanes.begin());
		if (last_lanes.empty() || current_lanes.empty() ||
				last_lanes.front() != current_lanes.front()) {
			return false;
		}
	}
	for (int i = 0; i < min((int)last_lanes.size(), (int)current_lanes.size()); i++) {
		if (last_lanes.at(i) == current_lanes.at(i)) continue;
		devide_index = i;
		int current_index = current_lanes.at(i).back() - '0';
		int last_index = last_lanes.at(i).back() - '0';
		delta_index = current_index - last_index;
		return true;
	}
	return false;
}

void LineProcess::RevLineFrame(ReferenceLineFrame &reverse_reference_line) {
	reverse_reference_line.Reset();
	if (rev_line_ptr_ == nullptr || rev_line_ptr_->front_lane_ids.empty()) {
		return;
	}
	reverse_reference_line.SetData(*rev_line_ptr_, localization_, car_model_);
	reverse_reference_line.reference_lane_id = 50;
}




} // namespace planning
} // namespace acu
