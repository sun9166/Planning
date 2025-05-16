/**
 * @file ego_info.h
 **/

#pragma once

#include <vector>
#include "macro.h"
#include "planning_gflags.h"
#include "datapool/include/common_config.h"
#include "src/execution/motionplan/common/datatype.h"
#include "src/execution/motionplan/common/obstacle/obstacle.h"
#include "src/execution/motionplan/common/reference_line_info/reference_line_info.h"

namespace acu {
namespace planning {

class EgoInfo {
 public:
  ~EgoInfo() = default;

  void SetVehicleParams(const CarModel& vehicle_params);
  bool Update(const StructVehicleInfo& veh_info, const StructLocalizationInfo& newest_loc);
  bool Update(const common::TrajectoryPoint& start_point);
  void UpdateLastPathLength(double length) {last_path_length_ = length;}
  void UpdateDisToEnd(double dis_to_end) {dis_to_end_ = dis_to_end;}
  void UpdateDisToVirtualWall(double dis_to_virtual_wall) {dis_to_virtual_wall_ = dis_to_virtual_wall;}
  void UpdateCurrentPathStatus(const bool path_access_safely) {last_path_access_safely_ = path_access_safely;}
  void Clear();

  common::TrajectoryPoint start_point() const { return start_point_; }

  VehicleState vehicle_state() const { return vehicle_state_; }
  VehicleState vehicle_state_new() const { return vehicle_state_new_; }

  double front_clear_distance() const { return front_clear_distance_; }
  double last_path_length() const {return last_path_length_;}
  double dis_to_end() const {return dis_to_end_;}
  double dis_to_virtual_wall() const {return dis_to_virtual_wall_;}
  common::math::Box2d ego_box() const { return ego_box_; }
  bool Last_path_accessible() const {return last_path_access_safely_;}

  void CalculateFrontObstacleClearDistance(
      const std::vector<const Obstacle*>& obstacles);
  void CalculateFrontObstacleClearDistance(
         ReferenceLineInfo* reference_line_info);
  CarModel vehicle_param() {  return vehicle_param_; }
  double vehicle_width() {  return vehicle_param_.car_width; }
  double vehicle_length() {  return vehicle_param_.length; }
  double vehicle_wheelbase() { return vehicle_param_.length_wheelbase; }
  double vehicle_front_axle_2_front() { return vehicle_param_.front_axle_tofront; }
  double vehicle_radius_min() { return vehicle_param_.min_turning_radius; }
  double vehicle_back_edge_to_center() { return back_edge_to_center_; }
  double vehicle_front_edge_to_center() { return front_edge_to_center_; }
  double vehicle_left_edge_to_center() { return left_edge_to_center_; }
  double vehicle_right_edge_to_center() { return right_edge_to_center_; } 

  bool is_in_solid_line_area() const;
  bool is_in_juction_area() const;    

  int is_on_acc_pedal() const {
    return on_accpedal_;
  } 

 private:

  void set_vehicle_state(const VehicleState& vehicle_state) {
    vehicle_state_ = vehicle_state;
  }

  void set_start_point(const common::TrajectoryPoint& start_point) {
    start_point_ = start_point;
  }

  void CalculateEgoBox();

  // stitched point (at stitching mode)
  // or real vehicle point (at non-stitching mode)
  common::TrajectoryPoint start_point_;

  int on_accpedal_ = 0;
  // 上帧规划的路径是否通行安全
  bool last_path_access_safely_ = true;
  // ego vehicle state
  VehicleState vehicle_state_;
  VehicleState vehicle_state_new_;
  double last_path_length_ = 0.0;
  double dis_to_end_ = 0.0;
  double dis_to_virtual_wall_= 9999;

  double front_clear_distance_ = FLAGS_default_front_clear_distance;

  // common::VehicleConfig ego_vehicle_config_;
  CarModel vehicle_param_;
  double back_edge_to_center_  ;
  double front_edge_to_center_ ;
  double left_edge_to_center_ ;
  double right_edge_to_center_;
  const double kEstimateDecel_;
  common::math::Box2d ego_box_;

  DECLARE_SINGLETON(EgoInfo)
};

}  // namespace planning
}  // namespace acu
