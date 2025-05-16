#include "filter_focus.h"

namespace acu{
namespace planning {

void FrameFocus::FindFocusObstacles(ReferenceLineFrame &reference_line, const CarModel *car_model_ptr) {
	reference_line_ptr_ = &reference_line;
  car_model_ptr_ = car_model_ptr;
  int max_focus_index = 100;
	InitFocusLevel();
	SortObstacles();
	FilterFrontObstacles(max_focus_index);
	FilterBackObstacles();
	GetFocusLevel(max_focus_index);
}

void FrameFocus::InitFocusLevel() {
	for (auto it = reference_line_ptr_->objects_.begin(); it != reference_line_ptr_->objects_.end(); it++) {
    auto &object = it->second;
    object.need_focus = false;
    if (object.type > 1 && object.type < 4 && 
    		object.sl_boundary.max_s > -FLAGS_frenet_perception_range) {
      object.need_focus = true;
    }
    if (object.type <= 1 && object.dis_to_junction < 10.0 || object.type > 5) {
      object.need_focus = true;
    }
    //
    if (object.x > 0.0 && object.x <  60.0 && fabs(object.y) < 10.0){
      object.need_focus = true;
    } 
    
    object.key_focus = 0; // 初始赋值
  }
}

void FrameFocus::SortObstacles() {
	auto &start_s = reference_line_ptr_->mapinfo.first_lane_start_s;
	for (auto it = reference_line_ptr_->objects_.begin(); it != reference_line_ptr_->objects_.end(); it++) {
    auto &object = it->second;
    for (auto &front_relation_lane : reference_line_ptr_->mapinfo.front_relation_lanes) {
    	if (front_relation_lane.lane_id != object.obj_lane_id) continue;
    	if (object.obj_lane_id == reference_line_ptr_->mapinfo.front_relation_lanes.front().lane_id &&
    			object.sl_boundary.max_s < 0.0) continue;
    	SortedObstacle sorted_obj(object.id, front_relation_lane.lon_index,
                                object.obj_lane_s, object.x);
    	front_relation_lane.sorted_objs.push_back(sorted_obj);
    }
    for (auto &back_relation_lane : reference_line_ptr_->mapinfo.back_relation_lanes) {
    	if (back_relation_lane.lane_id != object.obj_lane_id) continue;
    	if (reference_line_ptr_->mapinfo.front_relation_lanes.size() &&
    			object.obj_lane_id == reference_line_ptr_->mapinfo.front_relation_lanes.front().lane_id &&
    			object.sl_boundary.max_s > 0.0) continue;
    	SortedObstacle sorted_obj(object.id, back_relation_lane.lon_index,
                                object.obj_lane_s, object.x);
    	back_relation_lane.sorted_objs.push_back(sorted_obj);
    }
  }

  for (auto &relation_lane : reference_line_ptr_->mapinfo.front_relation_lanes) {
    std::sort(relation_lane.sorted_objs.begin(), relation_lane.sorted_objs.end(),
    [](const SortedObstacle& lhs, const SortedObstacle& rhs) {
      if (lhs.lane_s != rhs.lane_s) {
        return lhs.lane_s < rhs.lane_s;
      } else {
        return lhs.id < rhs.id;
      }
    });
  }
  for (auto &relation_lane : reference_line_ptr_->mapinfo.back_relation_lanes) {
    std::sort(relation_lane.sorted_objs.begin(), relation_lane.sorted_objs.end(),
    [](const SortedObstacle& lhs, const SortedObstacle& rhs) {
      if (lhs.lane_s != rhs.lane_s) {
        return lhs.lane_s > rhs.lane_s;
      } else {
        return lhs.id > rhs.id;
      }
    });
  }
}

void FrameFocus::FilterFrontObstacles(int &max_focus_index) {
	auto &start_s = reference_line_ptr_->mapinfo.first_lane_start_s;
	auto &front_relation_lanes = reference_line_ptr_->mapinfo.front_relation_lanes;
	// 将相关车道按照纵向顺序排列
	SortRelationLanes(front_relation_lanes, true);
	vector<int> self_focus_ids, other_focus_ids;
  max_focus_index = 100;
	int front_focus_size = 0;
  double max_focus_s = 0.0;
  for (auto &front_relation_lane : front_relation_lanes) {
  	if (front_relation_lane.type != RelationType::SELF) continue;
  	for (auto &sorted_obj : front_relation_lane.sorted_objs) {
  		if (front_focus_size >= FLAGS_focus_num && sorted_obj.lane_s > max_focus_s + 20) break;
      self_focus_ids.push_back(sorted_obj.id);
  		max_focus_s = sorted_obj.lane_s;
      if (reference_line_ptr_->objects_.count(sorted_obj.id) &&
          reference_line_ptr_->objects_.at(sorted_obj.id).sl_boundary.max_l > -0.5 * car_model_ptr_->car_width &&
          reference_line_ptr_->objects_.at(sorted_obj.id).sl_boundary.min_l < 0.5 * car_model_ptr_->car_width) {
        max_focus_index = front_relation_lane.lon_index;
        if (!reference_line_ptr_->objects_.at(sorted_obj.id).is_static) {
          front_focus_size++;
        }
      }
  		//AINFO_IF(FLAGS_log_enable)<<"add "<<sorted_obj.id<<" self lane."<<front_focus_size;
  	}
  }
  
  for (auto &front_relation_lane : front_relation_lanes) {
  	if (front_relation_lane.type != RelationType::REVERSE) continue;
  	for (auto &sorted_obj : front_relation_lane.sorted_objs) {
  		other_focus_ids.push_back(sorted_obj.id);
  		//AINFO_IF(FLAGS_log_enable)<<"add "<<sorted_obj.id<<" reverse lane.";
  	}
  }

  // AINFO<<"max_focus_index is "<<max_focus_index<<" max_focus_s "<<max_focus_s;
  for (auto &front_relation_lane : front_relation_lanes) {
  	if (front_relation_lane.type != RelationType::LEFT &&
  			front_relation_lane.type != RelationType::RIGHT) continue;	
  	if (front_relation_lane.lon_index > max_focus_index) continue;	
  	for (auto &sorted_obj : front_relation_lane.sorted_objs) {
  		if (self_focus_ids.empty() || front_relation_lane.lon_index < max_focus_index ||
  				front_relation_lane.lon_index == max_focus_index &&
  				sorted_obj.lane_s <= max_focus_s + FLAGS_front_buff) {
  			other_focus_ids.push_back(sorted_obj.id);
  		}
  	}
  }
  for (auto &front_relation_lane : front_relation_lanes) {
  	if (front_relation_lane.type == RelationType::SELF || 
				front_relation_lane.type == RelationType::LEFT ||
				front_relation_lane.type == RelationType::RIGHT) continue;
		if (front_relation_lane.lon_index > max_focus_index) continue;
  	if (front_relation_lane.type == RelationType::MERGE ||
				front_relation_lane.type == RelationType::CROSS) {
  		if (front_relation_lane.lon_index == 0 && 
  			front_relation_lane.end_s < start_s - FLAGS_lane_width) continue;
  	}
  	for (auto &sorted_obj : front_relation_lane.sorted_objs) {
  		if ((front_relation_lane.type == RelationType::MERGE ||
					 front_relation_lane.type == RelationType::CROSS) &&
					front_relation_lane.lon_index == max_focus_index &&
  				sorted_obj.lane_s > front_relation_lane.relation_end_s + 
  					car_model_ptr_->car_width + FLAGS_front_buff) continue;
  		other_focus_ids.push_back(sorted_obj.id);
  		if (reference_line_ptr_->mapinfo.front_lane_ids.size() > sorted_obj.lon_index &&
  				sorted_obj.lon_index >= 0 && reference_line_ptr_->objects_.count(sorted_obj.id)) {
  			reference_line_ptr_->objects_.at(sorted_obj.id).relation_lane = 
  						reference_line_ptr_->mapinfo.front_lane_ids.at(sorted_obj.lon_index);
  		}
  	}
  }

  for (auto &focus_obj : self_focus_ids) {
  	if (reference_line_ptr_->objects_.count(focus_obj)) {
  		reference_line_ptr_->objects_.at(focus_obj).need_focus = true;
  		// AWARN<<"add front self focus_obj "<<focus_obj;
  	}
  }
  for (auto &focus_obj : other_focus_ids) {
  	if (reference_line_ptr_->objects_.count(focus_obj)) {
  		reference_line_ptr_->objects_.at(focus_obj).need_focus = true;
  		// AWARN<<"add front other focus_obj "<<focus_obj;
  	}
  }
}

void FrameFocus::FilterBackObstacles() {
	auto &back_relation_lanes = reference_line_ptr_->mapinfo.back_relation_lanes;
	SortRelationLanes(back_relation_lanes, false);
  int temp_size = 0;
  for (auto &back_relation_lane : back_relation_lanes) {
  	if (back_relation_lane.type == RelationType::SELF) {
  		for (auto &sorted_obj : back_relation_lane.sorted_objs) {
  			if (temp_size >= FLAGS_focus_num) break;
  			if (!reference_line_ptr_->objects_.count(sorted_obj.id)) continue;
  			reference_line_ptr_->objects_.at(sorted_obj.id).need_focus = true;
  			temp_size++;
  		}
  	}
  	temp_size = 0;
  	if (back_relation_lane.type == RelationType::LEFT) {
  		for (auto &sorted_obj : back_relation_lane.sorted_objs) {
  			if (temp_size >= FLAGS_focus_num) break;
  			if (!reference_line_ptr_->objects_.count(sorted_obj.id)) continue;
  			reference_line_ptr_->objects_.at(sorted_obj.id).need_focus = true;
  			temp_size++;
  		}
  	}
  	temp_size = 0;
  	if (back_relation_lane.type == RelationType::RIGHT) {
  		for (auto &sorted_obj : back_relation_lane.sorted_objs) {
  			if (temp_size >= FLAGS_focus_num) break;
  			if (!reference_line_ptr_->objects_.count(sorted_obj.id)) continue;
  			reference_line_ptr_->objects_.at(sorted_obj.id).need_focus = true;
  			temp_size++;
  		}
  	}
  	temp_size = 0;
  	if (back_relation_lane.type == RelationType::MERGE) {
  		for (auto &sorted_obj : back_relation_lane.sorted_objs) {
  			if (temp_size >= FLAGS_focus_num) break;
  			if (!reference_line_ptr_->objects_.count(sorted_obj.id)) continue;
  			reference_line_ptr_->objects_.at(sorted_obj.id).need_focus = true;
  			temp_size++;
  		}
  	}
  	temp_size = 0;
  	if (back_relation_lane.type == RelationType::MERGEPRE) {
  		for (auto &sorted_obj : back_relation_lane.sorted_objs) {
  			if (temp_size >= FLAGS_focus_num) break;
  			if (!reference_line_ptr_->objects_.count(sorted_obj.id)) continue;
  			reference_line_ptr_->objects_.at(sorted_obj.id).need_focus = true;
  			temp_size++;
  		}
  	}
  	temp_size = 0;
  	if (back_relation_lane.type == RelationType::MERGESIM) {
  		for (auto &sorted_obj : back_relation_lane.sorted_objs) {
  			if (temp_size >= FLAGS_focus_num) break;
  			if (!reference_line_ptr_->objects_.count(sorted_obj.id)) continue;
  			reference_line_ptr_->objects_.at(sorted_obj.id).need_focus = true;
  			temp_size++;
  		}
  	}
  	temp_size = 0;
  	if (back_relation_lane.type == RelationType::MERGESIMPRE) {
  		for (auto &sorted_obj : back_relation_lane.sorted_objs) {
  			if (temp_size >= FLAGS_focus_num) break;
  			if (!reference_line_ptr_->objects_.count(sorted_obj.id)) continue;
  			reference_line_ptr_->objects_.at(sorted_obj.id).need_focus = true;
  			temp_size++;
  		}
  	}
  }
}

void FrameFocus::SortRelationLanes(vector<RelationLane> &relation_lanes, bool is_front) {
	std::sort(relation_lanes.begin(), relation_lanes.end(),
   				[](const RelationLane& lhs, const RelationLane& rhs) {
   				  if (lhs.lon_index != rhs.lon_index) {
   				    return lhs.lon_index < rhs.lon_index;
   				  } else {
   				    return lhs.lane_id < rhs.lane_id;
   				  }
   				}); 
  if (!is_front) {
    reverse(relation_lanes.begin(), relation_lanes.end());
  }
}

void FrameFocus::GetFocusLevel(int &max_focus_index) {
  vector<SortedObstacle> lon_focus_objs, lat_focus_objs;
  auto &start_s = reference_line_ptr_->mapinfo.first_lane_start_s;
  auto &front_relation_lanes = reference_line_ptr_->mapinfo.front_relation_lanes;
	for (auto &front_relation_lane : front_relation_lanes) {
  	if (front_relation_lane.type == RelationType::SELF || 
				front_relation_lane.type == RelationType::LEFT ||
				front_relation_lane.type == RelationType::RIGHT) continue;
		if (front_relation_lane.lon_index > max_focus_index) continue;
  	if (front_relation_lane.type == RelationType::MERGE ||
				front_relation_lane.type == RelationType::CROSS) {
  		if (front_relation_lane.lon_index == 0 && 
  			front_relation_lane.end_s < start_s - FLAGS_lane_width) continue;
  	}
  	for (auto &sorted_obj : front_relation_lane.sorted_objs) {
  		if ((front_relation_lane.type == RelationType::MERGE ||
					 front_relation_lane.type == RelationType::CROSS) &&
					front_relation_lane.lon_index == max_focus_index &&
  				sorted_obj.lane_s > front_relation_lane.relation_end_s + 
  					car_model_ptr_->car_width + FLAGS_front_buff) continue;
  		if (front_relation_lane.type == RelationType::MERGE ||
					front_relation_lane.type == RelationType::CROSS ||
					front_relation_lane.type == RelationType::MERGEPRE ||
					front_relation_lane.type == RelationType::CROSSPRE) {
  			lon_focus_objs.push_back(sorted_obj);
  		}
  		if (front_relation_lane.type == RelationType::MERGESIM ||
					front_relation_lane.type == RelationType::MERGEPRE) {
  			lat_focus_objs.push_back(sorted_obj);
  		}
  	}
  }
  // for (auto &lat_focus : lat_focus_objs) {
  // 	if (reference_line_ptr_->objects_.count(lat_focus.id)) {
  // 		if (reference_line_ptr_->objects_.at(lat_focus.id).type == 2 ||
  // 				reference_line_ptr_->objects_.at(lat_focus.id).type == 4) {
  // 			continue;
  // 		}
  // 		reference_line_ptr_->objects_.at(lat_focus.id).key_focus = 1;
  // 	}
  // }
  for (auto &lon_focus : lon_focus_objs) {
  	if (reference_line_ptr_->objects_.count(lon_focus.id)) {
  		if (reference_line_ptr_->objects_.at(lon_focus.id).type == 2 ||
  				reference_line_ptr_->objects_.at(lon_focus.id).type == 4) {
  			continue;
  		}
  		reference_line_ptr_->objects_.at(lon_focus.id).key_focus = 2;
  	}
  }
}




}
}
