#include "acc_analysis.h"

namespace acu {
namespace planning {

AccAnalysis::AccAnalysis() {}

AccAnalysis::~AccAnalysis() {}

void AccAnalysis::UpdateProbGrid(std::set<int>& special_objects) {
  reference_line_ = &context_->cognition_info_->reference_line_info;
  current_line_ = context_->reference_line_map_[reference_line_->current_line_id];
  target_line_ = context_->reference_line_map_[context_->decision_result_.reference_line_id];
  search_line_ = reference_line_->path_in_current ? 
      current_line_ : &reference_line_->local_reference_line;
  UpdateExpectedAcc(special_objects);
  UpdatePushedPoints();
}

void AccAnalysis::UpdateExpectedAcc(std::set<int>& special_objects) {
  for (auto& object : context_->st_data_) {
    object.second.is_pushed = false;
    if (FLAGS_gap_enable && context_->decision_result_.reference_line_id >= 20) {
      if (target_line_->objects_.count(object.first) && 
          target_line_->objects_.at(object.first).sl_boundary.max_s < 0.0 && 
          target_line_->objects_.at(object.first).conflict_type < 4) {
        if (AllowChangeAcc(object.first)) {
          AINFO_IF(FLAGS_log_enable) << "Push " << object.first << " for rear car";
        }
        continue;
      }
    }
    if (object.second.controllability == 0 || object.second.key_points.empty() || 
        search_line_->objects_.count(object.first) == 0) {
      continue;
    }
    float min_t = object.second.key_points.begin()->first * FLAGS_scale_t;
    float max_t = object.second.key_points.rbegin()->first * FLAGS_scale_t;
    float min_s_1 = *object.second.key_points.begin()->second.begin() * FLAGS_scale_s;
    float min_s_2 = *object.second.key_points.rbegin()->second.begin() * FLAGS_scale_s;
    min_t += min_t < FLAGS_scale_t ? 0.01 : 0.0;
    max_t += max_t < FLAGS_scale_t ? 0.01 : 0.0;
    float a_1 = 2.0 * (min_s_1 - context_->ego_speed_ * min_t) / pow(min_t, 2);
    float a_2 = 2.0 * (min_s_2 - context_->ego_speed_ * max_t) / pow(max_t, 2);
    for (auto &pd :search_line_->objects_[object.first].prediction.trajectories) {
      if (pd.range_pd_s.empty()) {
        continue;
      }
      if (min_s_1 < 1.0 && pd.range_pd_s.front() - min_s_1 > 6.0 && 
          min_s_1 / context_->ego_speed_ < 1.0) {
        for (int i = 0; i < pd.st_boundary.size(); i++) {
          double t = std::max(pd.st_boundary[i].second.x() / context_->ego_speed_, 0.1);
          if (i < pd.range_pd_s.size() && pd.range_pd_s[i] < 
              search_line_->objects_[object.first].speed * t - 0.25 * FLAGS_max_pd_acc * t * t) {
            special_objects.insert(object.first);
            context_->decision_result_.object_decision[object.first] = eObjectDecisionEnum::CONFLICT;
            break;
          }
        }
      }
    }
    if (a_1 > -0.6 && a_2 > -0.6 || object.second.right_of_way && 
        a_1 > -1.5 && a_2 > -1.5) {
      AINFO_IF(FLAGS_log_enable)<< "Don't push " << object.first << " for acc " << std::min(a_1 , a_2);
      continue;
    } else if (min_t > 5.0) {
      if (AllowChangeAcc(object.first, -1)) {
        AINFO_IF(FLAGS_log_enable)<< "Push " << object.first << " for acc " << std::min(a_1 , a_2);
      }
      continue;
    }
    object.second.push_cost += fabs(std::min(a_1 , a_2));
    double final_v;
    if (AllowChangeAcc(object.first, final_v)) {
      search_line_->st_map.ExtrusionObjSTArea(search_line_->objects_[object.first], final_v);
      AINFO_IF(FLAGS_log_enable) << "Push " << object.first << " final_a = " << final_v;
      object.second.is_pushed = true;
    }
  }
}

bool AccAnalysis::AllowChangeAcc(const int& id) {
  if (search_line_->objects_.count(id) == 0) {
    return false;
  }
  auto& object = search_line_->objects_.at(id);
  for (auto &pd :object.prediction.trajectories) {
    if (pd.londiscrete_areas.empty() || pd.points.empty() || pd.range_pd_s.empty()) {
      continue;
    }
    double v = context_->planning_config_.speedplan_config.maximum_cruising_speed;
    double ego_s = pd.range_s.first + context_->planning_config_.car_model.length;
    double a = (pow(v, 2) - pow(context_->ego_speed_, 2)) / 2.0 / ego_s;
    double max_a = std::max(std::min(a, FLAGS_max_acc), 0.01);
    double yield_t =  sqrt(context_->ego_speed_ * context_->ego_speed_ + 
        2.0 * max_a * ego_s) / max_a;
    double yield_a = std::min(pd.londiscrete_areas.back().a, 
        2.0 * (pd.range_pd_s.front() - object.speed * yield_t) / pow(yield_t, 2));
    if (pd.range_s.first < pd.range_pd_s.front()) {
      double meet_a = max_a - pow(context_->ego_speed_ - object.speed, 2) / 
          2.0 / (pd.range_pd_s.front() - pd.range_s.first);
      yield_a = std::min(yield_a, meet_a);
    }
    AINFO_IF(FLAGS_log_enable)<< "pd_s = " << pd.range_pd_s.front() << " s = " << pd.range_s.first;
    AINFO_IF(FLAGS_log_enable)<< "ego_a " << max_a << " yield_t = " << yield_t;
    AINFO_IF(FLAGS_log_enable) << "Push " << id << " current_a = " << 
        pd.points.front().a << " final_a = " << yield_a;
    search_line_->st_map.ExtrusionObjSTArea(search_line_->objects_[id], yield_a);
    context_->st_data_.at(id).is_pushed = true;
    return true;
  }
  return false;
}

bool AccAnalysis::AllowChangeAcc(const int& id, const int& direction) {
  if (search_line_->objects_.count(id) == 0) {
    return false;
  }
  for (auto &pd :search_line_->objects_.at(id).prediction.trajectories) {
    if (pd.londiscrete_areas.empty() || pd.points.empty()) {
      continue;
    }
    double target_a = direction > 0 ? pd.londiscrete_areas.front().a : 
        pd.londiscrete_areas.back().a;
    AINFO_IF(FLAGS_log_enable) << "Push " << id << " current_a = " << 
        pd.points.front().a << " final_a = " << target_a;
    search_line_->st_map.ExtrusionObjSTArea(search_line_->objects_[id], target_a);
    context_->st_data_.at(id).is_pushed = true;
    return true;
  }
  return false;
}

bool AccAnalysis::AllowChangeAcc(const int& id, double& final_v) {
  auto& object = search_line_->objects_.at(id);
  final_v = object.speed;
  for (auto &pd :object.prediction.trajectories) {
    if (pd.londiscrete_areas.empty() || pd.points.empty() || pd.range_pd_s.empty()) {
      return false;
    }
    if (object.right_of_way && (pd.range_s.first / context_->ego_speed_ > 1.0 || 
        pd.range_pd_s.front() - pd.range_s.first < 10.0)) {
      return false;
    }
    double current_a = pd.points.front().a;
    double yield_a = pd.points.front().a - 1e-3;
    double overtake_a = pd.points.front().a + 1e-3;
    double ego_v = std::max(context_->ego_speed_, 1e-3);
    for (int i = 0; i < pd.st_boundary.size(); i++) {
      double t = std::max(pd.st_boundary[i].second.x() / ego_v, 0.1);
      if (i < pd.range_pd_s.size()) {
        yield_a = std::min(yield_a, 2.0 * (pd.range_pd_s[i] - object.speed * t) / pow(t, 2));
      }
    }
    for (int i = 0; i < pd.st_boundary.size(); i++) {
      double t = pd.st_boundary[i].first.x() / ego_v;
      if (i < pd.range_pd_s.size()) {
        overtake_a = std::max(overtake_a, 2.0 * (pd.range_pd_s[i] - object.speed * t) / pow(t, 2));
      }
    }
    double yield_e = fabs(yield_a - current_a);
    double overtake_e = fabs(overtake_a - current_a);
    double rate = yield_e / overtake_e;
    AERROR << "current_a = " << current_a << " yield_a = " << yield_a << " overtake_a = " << 
        overtake_a << " rate = " << rate << " p = " << pd.probability;
    context_->st_data_.at(id).push_cost -= rate * pd.probability;
    for (auto& st : pd.londiscrete_areas) {
      final_v = st.a;
      // AERROR << "v = " << st.vi << " a = " << st.a;
      if (st.a <= yield_a && st.a <= current_a) {        
        break;
      }
    }
    if (pd.range_pd_s.front() - pd.range_s.first > 10.0 && 
        pd.range_s.first / context_->ego_speed_ < 1.0) {
      yield_a = std::max(yield_a, current_a - 0.5 * FLAGS_max_pd_acc);
      final_v = std::min(final_v, std::max(yield_a, -0.5 * FLAGS_max_pd_acc));
    }
  }
  return context_->st_data_.at(id).push_cost > 0.0;
}

void AccAnalysis::UpdatePushedPoints() {
  for (auto& object : context_->st_data_) {
    if (object.second.is_pushed) {
      AINFO_IF(FLAGS_log_enable) << "push id = " << object.first << 
          " max_p = " << object.second.max_p;
      for (auto& point : object.second.key_points) {
        // AINFO << "t = " << point.first * FLAGS_scale_t << " min_s = " << 
        //     *point.second.begin() * FLAGS_scale_s << " max_s = " << 
        //     *point.second.rbegin() * FLAGS_scale_s;
      }
      object.second.max_p = 0.0;
      object.second.key_points.clear();
    } else {
      object.second.push_cost = 0.0;
    }
  }
  const int range_t = search_line_->GetSTMapRangeT();
  const int range_s = search_line_->GetSTMapRangeS();
  auto st_map_ptr = search_line_->GetSTMapPtr();
  for (int t = 0; t < range_t; t++) {
    for(int s = 0; s < range_s; s++) {
      auto& point = st_map_ptr[t][s];
      for (auto& obj : point.objs) {
        if (context_->st_data_.count(obj.second.id) && 
            context_->st_data_.at(obj.second.id).is_pushed) {
          if (obj.second.p > context_->st_data_[obj.second.id].max_p) {
            context_->st_data_[obj.first].max_p = obj.second.p;
            context_->st_data_[obj.first].center_t = point.index.t * FLAGS_scale_t;
            context_->st_data_[obj.first].center_s = point.index.s * FLAGS_scale_s;
          }
          if (obj.second.p > FLAGS_p_threshold) {
            context_->st_data_[obj.first].key_points[point.index.t].insert(point.index.s);
          }
        }
      }
    }
  }
}

}  //  namespace planning
}  //  namespace acu
