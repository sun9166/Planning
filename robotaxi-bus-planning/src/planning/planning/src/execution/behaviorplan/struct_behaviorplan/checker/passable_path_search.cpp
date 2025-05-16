#include "passable_path_search.h"

namespace acu {
namespace planning {
 
PassablePathSearch::PassablePathSearch() {}

PassablePathSearch::~PassablePathSearch() {}

bool PassablePathSearch::GeneratePath(std::vector<int>& passable_ref_lane_ids) {
  reference_line_ = &context_->cognition_info_->reference_line_info;
  current_line_ = context_->reference_line_map_[reference_line_->current_line_id];
  if(current_line_->mapinfo.path_points.size() < 3){
    AINFO_IF(FLAGS_log_enable) <<" mapinfo path points is too less";
    return false;
  }
  passable_ref_lane_ids.clear();
  Init();
  AddLineInfo();
  GetInitState();
  BuildStaticMap();
  AddSampleLevel();
  AddSamplePoints();
  if (sample_points_.empty()) {
    return false;
  }
  SearchPath();
  if(passable_refs_.empty()){
    return false;
  }else{
    for(auto& line_id: passable_refs_){
      auto line = lines_[line_id];
      passable_ref_lane_ids.push_back(line->reference_lane_id);
      AINFO_IF(FLAGS_log_enable) <<"GeneratePath,id: "<<line->reference_lane_id;
    }
  }

  return true;
}

void PassablePathSearch::Init(){
  half_width_ = 0.5 * context_->planning_config_.car_model.car_width;
  buffer_ = FLAGS_collision_buff;
  level_s_ = std::max(min_s_, 
      context_->planning_config_.speedplan_config.maximum_cruising_speed / 3.6);

  sample_s_.clear();
  points_.clear();
  sample_points_.clear();
  passable_refs_.clear();
  farest_points_.clear();
}
// Get target line and reference lines
void PassablePathSearch::AddLineInfo() {
  // reference_line_ = &context_->cognition_info_->reference_line_info;
  // current_line_ = context_->reference_line_map_[reference_line_->current_line_id];
  lines_.clear();
  // 车道从左到右依次排序
  for (auto& line : context_->reference_line_map_) {
    if (line.first / 10 == 2) {
      lines_[line.first - 20] = line.second;
    } else {
      lines_[line.first] = line.second;
    }
  }

  left_to_right_line_idx_.clear();
  int idx = 0;
  for(auto& line:lines_){
    left_to_right_line_idx_[line.first] = idx;
    idx++;
  }
}

void PassablePathSearch::GetInitState() {
  XYToSL(current_line_->mapinfo, current_line_->loc_site_, start_s_, start_l_);
  start_yaw_ = DP_->loc_perception.localization_data.yaw - 
      current_line_->mapinfo.path_points.front().globalangle;
}

// Add static objects in path bound
void PassablePathSearch::BuildStaticMap() {
  static_objects_.clear();
  for (auto& object : current_line_->objects_) {
    auto& obj = object.second;
    if (obj.sl_boundary.min_s > FLAGS_front_perception_range || 
        obj.sl_boundary.min_s > current_line_->mapinfo.dis_to_end || 
        obj.sl_boundary.min_s < 1e-3 || obj.is_static == false || 
        current_line_->follow_objects_.count(obj.id)) {
      continue;
    }
    double left_bd, right_bd;
    GetPathBound(obj.sl_boundary.min_s, left_bd, right_bd);
    if (obj.sl_boundary.min_l > left_bd || obj.sl_boundary.max_l < -right_bd) {
      continue;
    }
    static_objects_[obj.sl_boundary.min_s] = obj.id;
    AINFO_IF(FLAGS_log_enable) << "BuildStaticMap,add object " << obj.id;
  }
}

// Get distance from s of target line to path bound
void PassablePathSearch::GetPathBound(const double s, double& left_bd, double& right_bd) {
  left_bd = 0.0;
  right_bd = 0.0;
  int index;
  Site point;
  current_line_->GetNearestPoint(s, point, index);
  double left_w, right_w;

  for (auto it = lines_.begin(); it != lines_.end(); ++it) {
    if (s < it->second->mapinfo.dis_to_end) {
      it->second->GetWidthToLaneBoundary(left_bd, right_w, point);
      break;
    }
  }

  for (auto it = lines_.rbegin(); it != lines_.rend(); ++it) {
    if (s < it->second->mapinfo.dis_to_end) {
      it->second->GetWidthToLaneBoundary(left_w, right_bd, point);
      break;
    }
  }
}

// Get sample s according to speed, boundary type and static objects
void PassablePathSearch::AddSampleLevel() {
  double max_s = 0.0,s = 0.0;
  // 最远的
  for(auto& line: lines_){
    max_s = std::max(max_s,std::min(line.second->mapinfo.dis_to_end, 
                           FLAGS_front_perception_range + 10.0));
    AINFO_IF(FLAGS_log_enable) <<"line.id:"<<line.first<<",line.dis_to_end:"<<line.second->mapinfo.dis_to_end;
  }
  AINFO_IF(FLAGS_log_enable) <<"level_s:"<<level_s_<<",max_s:"<<max_s;
  while (s <= max_s) {
    sample_s_.push_back(std::make_pair(s, 0));
    s += level_s_;
  }

  // Add s from boundary type
  for (auto& bd : current_line_->mapinfo.left_bd_types) {
    for (auto it = sample_s_.begin(); it != sample_s_.end(); it++) {
      if (fabs(it->first - bd.first) < min_s_ && it->first > 0.0 && !it->second) {
        it->first = bd.first;
        it->second = 1;
        break;
      } else if (it->first > bd.first) {
        sample_s_.insert(it, std::make_pair(it->first, 1));
        break;
      }
    }
  }
  for (auto& bd : current_line_->mapinfo.right_bd_types) {
    for (auto it = sample_s_.begin(); it != sample_s_.end(); it++) {
      if (fabs(it->first - bd.first) < min_s_ && it->first > 0.0 && !it->second) {
        it->first = bd.first;
        it->second = 1;
        break;
      } else if (it->first > bd.first) {
        sample_s_.insert(it, std::make_pair(it->first, 1));
        break;
      }
    }
  }
  for (auto& object : static_objects_) {
    double temp_s = 0.0;
    for (auto it = sample_s_.begin(); it != sample_s_.end(); it++) {
      if (it->first >= object.first) {
        if (it->first - object.first >= min_s_ && object.first - temp_s >= min_s_) {
          sample_s_.insert(it, std::make_pair(object.first, 2));
        }
        break;
      }
      temp_s = it->first;
    }
  }
}

// Add two points in single line
void PassablePathSearch::AddSamplePoints() {
  double left_w, right_w, s, l;
  Site temp_site;
  MapPointStruct point;
  int index;
  for (auto it = sample_s_.begin(); it != sample_s_.end(); it++) {
    // AWARN_IF(FLAGS_log_enable) << "s = " << it->first << " type = " << it->second;
    point.s = it->first;
    point.last_on_ref = false;
    double radius = (pow(context_->ego_speed_, 2) - 2.0 * point.s) / 2.0;
    point.min_radius = std::max(radius, 10.0);
    std::vector<MapPointStruct> points;
    for (auto line : lines_) {
      // 更远的不采样
      if(point.s > line.second->mapinfo.dis_to_end){
        continue;
      }
      // 最后一层标识
      if(std::next(it) == sample_s_.end() || 
        (std::next(it) != sample_s_.end() && std::next(it)->first > line.second->mapinfo.dis_to_end)){
        // AINFO_IF(FLAGS_log_enable) <<"sample_points: last slice on line "<< line.first;
        point.last_on_ref = true;
      }

      auto it = line.second;
      it->GetWidthToLaneBoundary(left_w, right_w, point.s);
      it->GetNearestPoint(point.s, temp_site, index);
      XYToSL(current_line_->mapinfo, temp_site, s, l);
      // point.l_solid = it->left_bds_.lower_bound(point.s)->second;
      // point.r_solid = it->right_bds_.lower_bound(point.s)->second;
      // if (pull_over_) {
      //   point.l_solid = false;
      //   point.r_solid = false;
      // }
      // 
      point.l_solid = true;
      point.r_solid = true;
      if(it->left_bds_.lower_bound(point.s) != it->left_bds_.end()){
        auto iter = it->left_bds_.lower_bound(point.s);
        if(!iter->second && (iter->first - point.s) > 3.0){
          point.l_solid = false;
        }
      }

      if(it->right_bds_.lower_bound(point.s) != it->right_bds_.end()){
        auto iter = it->right_bds_.lower_bound(point.s);
        if(!iter->second && (iter->first - point.s) > 3.0){
          point.r_solid = false;
        }
      }
      // Add sample points on line for curvature constraint on highway
      const double on_line_width = 0.3;
      point.l = l + left_w - on_line_width;
      
      if (!point.l_solid && (points.empty() || point.l < points.back().l)) {
        AddPointLineIds(line.first, point);
        points.push_back(point);
      }
      point.l = l + left_w - half_width_;
      if (points.empty() || point.l < points.back().l) {
        AddPointLineIds(line.first, point);
        points.push_back(point);
      }
      point.l = l;
      if (point.l < points.back().l) {
        AddPointLineIds(line.first, point);
        points.push_back(point);
      }
      
      point.l = l + half_width_ - right_w;
      if (point.l < points.back().l) {
        AddPointLineIds(line.first, point);
        points.push_back(point);
      }
      point.l = l + on_line_width - right_w;
      if (!point.r_solid && point.l < points.back().l) {
        AddPointLineIds(line.first, point);
        points.push_back(point);
      }
    }
    sample_points_.push_back(points);
  }


  AINFO_IF(FLAGS_log_enable) << "sample_points result";
  if(FLAGS_log_enable){
    std::cout<<"sample_points_s=[";
    for(auto slice:sample_points_){
      std::cout<<"[";
      for(auto&p: slice){
        std::cout<<p.s<<",";
      }
      std::cout<<"],";
    }
    std::cout<<"]\n"<<std::endl;

    std::cout<<"sample_points_l=[";
    for(auto slice:sample_points_){
      std::cout<<"[";
      for(auto&p: slice){
        std::cout<<p.l<<",";
      }
      std::cout<<"],";
    }
    std::cout<<"]"<<std::endl;

    std::cout<<"sample_points_ids=[";
    for(auto slice:sample_points_){
      std::cout<<"[";
      for(auto&p: slice){
        std::cout<<"[";
        for(auto id: p.line_ids){
          std::cout<<id<<",";
        }
        std::cout<<"],";
      }
      std::cout<<"],";
    }
    std::cout<<"]"<<std::endl;
  }
}

void PassablePathSearch::AddPointLineIds(const int id, MapPointStruct& point) {
  point.line_ids.clear();
  point.line_ids.push_back(id);
  double left_w, right_w, s, l;
  Site temp_site;
  int index;
  for (auto it = lines_.upper_bound(id); it != lines_.end(); ++it) {
    auto line = it->second;
    line->GetWidthToLaneBoundary(left_w, right_w, point.s);
    line->GetNearestPoint(point.s, temp_site, index);
    XYToSL(current_line_->mapinfo, temp_site, s, l);
    if (point.s < line->mapinfo.dis_to_end && point.l < left_w + l) {
      point.line_ids.push_back(it->first);
    } else {
      break;
    }
  }
}

bool PassablePathSearch::SearchPath() {
  std::vector<MapPointStruct> points;
  // 起始点和delta
  if(sample_points_.empty() || sample_points_.front().empty()) {
    AERROR_IF(FLAGS_log_enable) <<"Error with passable_path_search sample_points";
    return false;
  }
  MapPointStruct point = sample_points_.front().front();
  for (auto& pt : sample_points_.front()) {
    if (fabs(pt.l - start_l_) < std::fabs(point.l - start_l_)) {
      point = pt;
    }
  }
  point.delta = start_yaw_;
  if (fabs(point.delta) > 180.0) {
    point.delta += point.delta < 0.0 ? 360.0 : -360.0;
  } 
  point.delta = point.delta * M_PI / 180.0;
  point.lc_num = 0;
  for (auto index : point.line_ids) {
    search_index_[index] = 0;
  }
  points.push_back(point);
  points_.push_back(points);

  for (int i = 1; i < sample_points_.size(); i++) {
    if (points_.back().empty()) {
      AERROR_IF(FLAGS_log_enable) << "temp_block_s = " << sample_s_[i - 1].first;
      break;
    } 
    points.clear();
    int lc_num;
    double last_s = points_.back().front().s;
    // 第i层采样点
    for (auto& pt : sample_points_[i]) {
      // 第i-1层
      for (auto& pre_point : points_.back()) {
        if (SatisfyConstrain(pre_point, pt, lc_num)) {
          if (lc_num < pt.lc_num) {
            pt.lc_num = lc_num;
            pt.last_l = pre_point.l;
          }
        }
      }
      if (pt.lc_num < MAX_COST) {
        // 某条参考线的最后一层
        if(pt.last_on_ref && !pt.line_ids.empty()){
          passable_refs_.insert(pt.line_ids[0]);
          farest_points_.push_back(pt);
        }  
        pt.delta = atan((pt.l - pt.last_l) / (pt.s - last_s));
        points.push_back(pt);
        // Get max s index for line
        for (auto index : pt.line_ids) {
          search_index_[index] = i;
        }
      }
    }
    points_.push_back(points);
  }
  farest_points_.insert(farest_points_.end(),
                        points_.back().begin(),
                        points_.back().end());
  BackTrack(farest_points_);
  return passable_refs_.size();
}

bool PassablePathSearch::BackTrack(std::vector<MapPointStruct> farest_points){
  std::map<double, MapPointStruct> s_l;
  int back_track_num = 0;
  for(auto last_node:farest_points){
    double last_l = last_node.last_l;
    s_l.clear();
    s_l[last_node.s] = last_node;
    int start_slice = sample_points_.size()-1;
    for(int i = sample_points_.size() - 2; i >=0; i--){
      if(sample_points_.at(i+1).front().s >= last_node.s){
        start_slice = i;
        break;
      }
    }
    for(int i = start_slice; i >=0; i--){
      for(auto node : sample_points_.at(i)) {
        if (node.l == last_l){
          s_l[node.s] = node;
          last_l = node.last_l;
        }
      }
    }
    AINFO_IF(FLAGS_log_enable) << "BackTrack result";
    if(FLAGS_log_enable){
      std::cout<<"final_s=[";
      for(auto& node:s_l){
        std::cout<<node.second.s<<",";
      }
      std::cout<<"]"<<std::endl;
      std::cout<<"final_l=[";
      for(auto& node:s_l){
        std::cout<<node.second.l<<",";
      }
      std::cout<<"]"<<std::endl;
      std::cout<<"final_lane_ids=[";
      for(auto& node:s_l){
        std::cout<<"[";
        for(auto id: node.second.line_ids){
          std::cout<<id<<",";
        }
        std::cout<<"],";
      }
      std::cout<<"]\n"<<std::endl;
    }
   
  }
  return true;
}

bool PassablePathSearch::SatisfyConstrain(const MapPointStruct& pre_point, 
                                  const MapPointStruct& point, int& lc_num) {
  lc_num = MAX_COST;
  if(pre_point.last_on_ref){
    return false;
  }
  // point 左侧实线，上层采样点在point左侧
  // point 右侧实线，下层采样点在point右侧
  if (point.l_solid && (point.line_ids.front() > pre_point.line_ids.back()) || 
      point.r_solid && (point.line_ids.back() < pre_point.line_ids.front())) {
    return false;
  }
  if(point.line_ids.front() > pre_point.line_ids.back()){
    int point_idx = left_to_right_line_idx_[point.line_ids.front()];
    int pre_point_idx = left_to_right_line_idx_[pre_point.line_ids.back()];
    if(point_idx - pre_point_idx > 1 || point.l_solid && (point_idx - pre_point_idx == 1)) {
      return false;
    }
  }
  if(point.line_ids.back() < pre_point.line_ids.front()){
    int point_idx = left_to_right_line_idx_[point.line_ids.back()];
    int pre_point_idx = left_to_right_line_idx_[pre_point.line_ids.front()];
    if(pre_point_idx - point_idx > 1 || point.r_solid && (pre_point_idx - point_idx == 1)) {
      return false;
    }
  }

  double s_1 = pre_point.s, s_2 = point.s;
  double l_1 = pre_point.l, l_2 = point.l;
  double k = (l_2 - l_1) / (s_2 - s_1);
  double delta = fabs(pre_point.delta - atan(k));
  // 两个采样点间距太大
  if (0.5 * std::hypot(s_2 - s_1, l_2 - l_1) < point.min_radius * sin(delta)) {
    return false;
  }
  // 从近到远比遍历障碍物
  for (auto it = static_objects_.begin(); it != static_objects_.end(); ++it) {
    auto& object = current_line_->objects_[it->second];
    // 后续障碍物不干涉本层采样点
    if (object.sl_boundary.min_s > point.s) {
      break;
    } else if (object.sl_boundary.max_s < pre_point.s) {
      continue;
    }
    double min_l = l_1 + k * std::max(object.sl_boundary.min_s - s_1, 0.0);
    double max_l = l_1 + k * std::min(object.sl_boundary.max_s - s_1, s_2 - s_1);
    // 会碰撞
    if (min_l + (half_width_ + buffer_) > object.sl_boundary.min_l && 
        min_l - (half_width_ + buffer_) <  object.sl_boundary.max_l || 
        max_l + (half_width_ + buffer_) > object.sl_boundary.min_l && 
        max_l - (half_width_ + buffer_) <  object.sl_boundary.max_l) {
      return false;
    }
  }
  lc_num = pre_point.lc_num;
  if (point.line_ids.front() > pre_point.line_ids.back()) {
    lc_num += point.line_ids.front() - pre_point.line_ids.back();
  } else if (pre_point.line_ids.front() > point.line_ids.back()) {
    lc_num += pre_point.line_ids.front() - point.line_ids.back();
  }
  return true;
}

bool PassablePathSearch::CheckDestPassable(const int line_id, const double s) {
  int index = (int)(s / level_s_);
  if (points_.size() > index) {
    for (auto& point : points_[index]) {
      if (line_id >= point.line_ids.front() && line_id <= point.line_ids.back()) {
        AWARN_IF(FLAGS_log_enable) << "s = " << point.s << " l = " << point.l;
        return true;
      }
    }
  }
  if (points_.size() > index + 1) {
    for (auto& point : points_[index + 1]) {
      if (line_id >= point.line_ids.front() && line_id <= point.line_ids.back()) {
        AWARN_IF(FLAGS_log_enable) << "s = " << point.s << " l = " << point.l;
        return true;
      }
    }
  }
  return false;
}

void PassablePathSearch::GetSearchS(std::map<int,double>& search_s){
  search_s.clear();
  for(auto& idx: search_index_){
    int ref_lane_id = lines_[idx.first]->reference_lane_id;
    search_s[ref_lane_id] = sample_s_[idx.second].first;
  }
}
}  //  namespace planning
}  //  namespace acu
