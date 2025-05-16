
#include "src/execution/business/include/struct_business.h"
// #include "src/application/execution/dummy/include/dummy_pathplan.h"
namespace acu {
namespace planning {

StructBusiness::StructBusiness() {
  business_type_.SetBusinessTypeStruct();
  AINFO << "Contruct StructBusiness";
  business_error_ = false;
  business_exucute_over_ = false;
  short_lane_flag_ = false;
  no_map_counter_ = 0;
  frontrefpath_.clear();
}
bool StructBusiness::Init() {
  if (frontrefpath_.empty()) {
    is_init_ = false;
  } else {
    is_init_ = true;
    if (dis2missionpoint_ < 5) {
      short_lane_flag_ = true;
    }
  }
  AINFO << "[StructBusiness]:Init....... frontrefpath_ size = "<<frontrefpath_.size()<<";dis2missionpoint_ = "<<dis2missionpoint_;
  return true;
}
bool StructBusiness::Process() {
  AINFO << "[StructBusiness]:Process.......";
  static int static_counter = 0;
  auto DP = DataPool::Instance()->GetMainDataPtr();
  DP->debug_planning_msg.business_debug.set_process_status(1);
  auto &current = DP->cognition_info.struct_env_info.reference_line_info.current_reference_line;
  if (current.empty()) {
    DP->debug_planning_msg.business_debug.set_business_exucute_over((int)business_exucute_over_);
    DP->debug_planning_msg.business_debug.set_is_current_line_null(1);
    return true;
  }
  double length = car_model_.length + 0.5;
  if ((LaneDirectionType)current.front().mapinfo.direction == LaneDirectionType::BACKWARD) {
    length = 0.5;
  }
  AERROR<<" park length is "<<length<<" dis2missionpoint_ "<<dis2missionpoint_<<" v "<<car_velocity_;
  if (dis2missionpoint_ < length && car_velocity_ < 0.1) {
    business_exucute_over_ = true;
    static_counter = 0;
  }
  else if (length > 0.6 && dis2missionpoint_ < 20 && !business_exucute_over_) {
    if (car_velocity_ < 0.1 && DP->drive_status.control_mode == 0) {
      static_counter++;
    }
    else {
      static_counter = 0;
    }
    if (static_counter > 50) {
      business_exucute_over_ = true;
      static_counter = 0;
    }
  } else {
    static_counter = 0;
  }
  DP->debug_planning_msg.business_debug.set_business_exucute_over((int)business_exucute_over_);
  return true;
}
void StructBusiness::PullData() {
  auto DP = DataPool::Instance()->GetMainDataPtr();
  car_velocity_ = DP->loc_perception.localization_data.velocity;
  dis2missionpoint_ = DP->mapengine_data.map_info_data.dis2missionpoint;
  DP->debug_planning_msg.business_debug.set_car_velocity(car_velocity_);
  DP->debug_planning_msg.business_debug.set_dis_to_mission_point(dis2missionpoint_);
  car_model_ = DP->config_info.car_model;
  int index = DP->mapengine_data.map_info_data.index;
  if (DP->mapengine_data.map_info_data.alllinelists.empty() || index < 0 ||
      index >= DP->mapengine_data.map_info_data.alllinelists.size()) {
    frontrefpath_.clear();
  } else if (!DP->mapengine_data.map_info_data.alllinelists.at(index).path_points.empty()) {
    frontrefpath_= DP->mapengine_data.map_info_data.alllinelists.at(index).path_points;
  } else {
    frontrefpath_.clear();
  }
}
void StructBusiness::PushData() {
  auto DP = DataPool::Instance()->GetMainDataPtr();
  if (DP->mapengine_data.map_info_data.alllinelists.empty() ||
      DP->drive_status.control_mode == 1 && DP->task_content.command == eCommand::COMMAND_STOP) {
    business_exucute_over_ = false;
    short_lane_flag_ = false;
    DP->task_fsm_info.Reset();
    DP->task_fsm_info.task_status.SetIdleTask();
    no_map_counter_ = 0;
    return;
  }
  if (business_exucute_over_) {
    business_exucute_over_ = false;
    short_lane_flag_ = false;
    DP->task_fsm_info.setTaskExeOK();
  }
}


}
}

