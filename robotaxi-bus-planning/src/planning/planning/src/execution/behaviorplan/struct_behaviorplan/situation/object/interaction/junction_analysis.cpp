#include "junction_analysis.h"

namespace acu {
namespace planning {

JunctionAnalysis::JunctionAnalysis() {}

JunctionAnalysis::~JunctionAnalysis() {}

void JunctionAnalysis::AddInteractionInfo() {
  reference_line_ = &context_->cognition_info_->reference_line_info;
  current_line_ = context_->reference_line_map_[reference_line_->current_line_id];
  junction_type_ = 1;
  for (auto& lane_turn : current_line_->mapinfo.lane_turns) {
    if (lane_turn.first > context_->dis_to_junction_) {
      junction_type_ = lane_turn.second;
    }
  }
  dec_ = -1.0;
  dec_s_ = pow(context_->ego_speed_, 2) / (2.0 * fabs(dec_));
  for (auto& st_data : context_->st_data_) {
    st_data.second.priority = 0;
    if (current_line_->objects_.count(st_data.first) && 
        st_data.second.key_points.size()) {
      auto& object = current_line_->objects_[st_data.first];
      if (object.type > 1) {
        GetCrossWalkPriority(object);
      }
      if (object.type < 2 && junction_type_ == 2 && object.sl_boundary.min_l < 0.0) {
        GetStraightPriority(object);
      }
    }
  }
}

void JunctionAnalysis::GetCrossWalkPriority(const LineObject& object) {
  if (NearCrosswalk(object) == false) {
    return;
  }
  AERROR << object.id << " on crosswalk";
  auto& st_data = context_->st_data_[object.id];
  st_data.special_type = 1;
  double min_t = st_data.key_points.begin()->first;
  double min_s = *st_data.key_points.begin()->second.begin();
  if (context_->ego_speed_ > min_t) {
    //dec_s_ = context_->ego_speed_ * min_t + 0.5 * dec_ * pow(min_t, 2);
    dec_s_ = context_->ego_speed_ * min_t + 0.5 * (-2) * pow(min_t, 2);
  }
  // if (context_->ego_speed_ * min_t > min_s) {
  //   return;
  // }
  if (object.sl_boundary.min_s > 10.0 && 
      st_data.interaction_time < 1.0 && min_s > dec_s_) {
    st_data.priority = 1;
    AERROR << "yield " << object.id << " at " << st_data.interaction_time;
  }
}

void JunctionAnalysis::GetStraightPriority(const LineObject& object) {
  if (IsStraight(object) == false) {
    return;
  }
  AERROR << object.id << " go through";
  auto& st_data = context_->st_data_[object.id];
  st_data.special_type = 2;
  const double dec = -1.0;
  double min_t = st_data.key_points.begin()->first;
  double min_s = *st_data.key_points.begin()->second.begin();
  if (context_->ego_speed_ > min_t) {
    dec_s_ = context_->ego_speed_ * min_t + 0.5 * dec_ * pow(min_t, 2);
  }
  // if (context_->ego_speed_ * min_t > min_s) {
  //   return;
  // }
  AERROR << "dec_s_ = " << dec_s_ << " interaction_time " << st_data.interaction_time;
  if (st_data.interaction_time < 1.0 && min_s > dec_s_) {
    st_data.priority = 1;
    AERROR << "yield " << object.id << " at " << st_data.interaction_time;
  }
}

bool JunctionAnalysis::NearCrosswalk(const LineObject& object) {
  for (auto& crosswalk : current_line_->mapinfo.distance_to_crosswalks) {
    if (crosswalk.first > FLAGS_front_perception_range || crosswalk.first < 0.0) {
      continue;
    }
    if (object.sl_boundary.min_s < crosswalk.second + 2.5 && 
        object.sl_boundary.max_s > crosswalk.first - 2.5) {
      return true;
    }
  }
  return false;
}

bool JunctionAnalysis::IsStraight(const LineObject& object) {
  vectormap::VectorMap* vectormap = map::MapLoader::GetVectorMapPtr();
  for (auto& prediction : object.prediction.trajectories) {
    int turn = 0;
    for (auto& lane : prediction.lane_ids) {
      Id lane_id;
      lane_id.set_id(lane);
      Lane::LaneTurn turn_type;
      if (vectormap->GetLaneTurn(lane_id, turn_type) == 0) {
        turn = std::max((int)turn_type, turn);
      }
    }
    if (turn == 1) {
      return true;
    }
  }
  return false;
}

}  //  namespace planning
}  //  namespace acu
