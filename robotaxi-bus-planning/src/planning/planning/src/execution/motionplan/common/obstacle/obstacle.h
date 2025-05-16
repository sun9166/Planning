/**
 * @file obstacle.h
 **/

#pragma once

#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "decision.pb.h"
#include "sl_boundary.pb.h"
#include "prediction_trajectory.pb.h"

#include "common/math/box2d.h"
#include "common/math/vec2d.h"
#include "datapool/include/common_config.h"
#include "src/execution/motionplan/common/datatype.h"
#include "datapool/include/cognition_typedef.h"
#include "datapool/include/decision_typedef.h"
#include "src/execution/motionplan/common/indexed_list.h"
#include "src/execution/motionplan/common/speed/st_point.h"
#include "src/execution/motionplan/common/speed/st_boundary.h"
#include "src/execution/motionplan/common/reference_line_info/reference_line.h"
//#include "planning_debug_msgs/motion_obstacle.h"
#include "planning_debug_msgs.pb.h"
namespace acu {
namespace planning {

/**
 * @class Obstacle
 * @brief This is the class that associates an Obstacle with its path
 * properties. An obstacle's path properties relative to a path.
 * The `s` and `l` values are examples of path properties.
 * The decision of an obstacle is also associated with a path.
 *
 * The decisions have two categories: lateral decision and longitudinal
 * decision.
 * Lateral decision includes: nudge, ignore.
 * Lateral decision saftey priority: nudge > ignore.
 * Longitudinal decision includes: stop, yield, follow, overtake, ignore.
 * Decision safety priorities order: stop > yield >= follow > overtake > ignore
 *
 * Ignore decision belongs to both lateral decision and longitudinal decision,
 * and it has the lowest priority.
 */
class Obstacle {
 public:
  struct OverlapPoints {
    std::vector<STPoint> lower_points;
    std::vector<STPoint> upper_points;
    OverlapPoints() {
      reset();
    }
    void reset() {
      lower_points.clear();
      upper_points.clear();
    }
    bool time_equal(const STPoint& p1, const STPoint& p2) {
      return fabs(p1.t() - p2.t())< 1e-6;
    }
    bool HasPoint() const {
      return !lower_points.empty() && !upper_points.empty();
    }
    void AddPoint(const STPoint& _point) {
      if(upper_points.empty()) {
        upper_points.emplace_back(_point);
      } else {
        if (time_equal(upper_points.back(), _point)) {
          // std::cout<<"up_in_t "<<upper_points.back().t()<<"|"<<_point.t()<<
          //   " up_in_s "<<upper_points.back().s()<<"|"<<_point.s()<<std::endl;
          if (upper_points.back().s() < _point.s()) {
            upper_points.back().set_s(_point.s());
          }
        } else {
          // std::cout<<"up_in_t "<<upper_points.back().t()<<"|"<<_point.t()<<
          //   " up_in_s "<<upper_points.back().s()<<"|"<<_point.s()<<std::endl;
          upper_points.emplace_back(_point);
        }
      }
      if(lower_points.empty()) {
        lower_points.emplace_back(_point);
      } else {
        if (time_equal(lower_points.back(), _point)) {
          if (lower_points.back().s() > _point.s()) {
            lower_points.back().set_s(_point.s());
          }
        } else {
          lower_points.emplace_back(_point);
        }
      }
    }
  }; 
  Obstacle() = default;
  explicit Obstacle(
      const std::string& id,
      const LineObject& perception_obstacle,
      const double& obstacle_priority,
      const bool is_static,
      const bool is_ultra_static,
      const int conflict_type,
      const int conflict_type_on_target,
      const bool is_waiting,
      const bool was_dynamic);
  explicit Obstacle(
      const std::string& id,
      const LineObject& perception_obstacle,
      const std::vector<PredictionPoint>& trajectory,//new
      const double& obstacle_priority,
      const bool is_static,
      const bool is_ultra_static,
      const double start_time,
	    const int conflict_type,
      const int conflict_type_on_target,
      const bool is_waiting,
      const bool was_dynamic);

  const std::string& Id() const { return id_; }
  void SetId(const std::string& id) { id_ = id; }

  double speed() const { return speed_; }
  double heading() const { return heading_; }
  double acc() const { return acceleration_; }
  double speed_along_reference_line() const {return speed_linear_;};

  int32_t PerceptionId() const { return perception_id_; }

  bool IsStatic() const { return is_static_; }
  bool IsMovable() const { return is_movable_; }
  bool IsUltraStatic() const { return is_ultra_static_;}
  bool IsVirtual() const { return is_virtual_; }
  bool IsWaiting() const {return is_waiting_;}
  bool WasDynamic() const {return was_dynamic_;}
  bool IsReverseTraveling() const {return is_reverse_traveling_;}

  int conflict_type() const { return conflict_type_;}
  int conflict_type_on_target() const {
    return conflict_type_on_target_;
  }

  common::TrajectoryPoint GetPointAtTime(const double time) const;

  common::math::Box2d GetBoundingBox(
      const common::TrajectoryPoint& point) const;

  const common::math::Box2d& PerceptionBoundingBox() const {
    return perception_bounding_box_;
  }

  // common::math::Box2d* mutable_perceptionboundingbox() { //@pqg add 更新perceptionboundingbox
  //   return &perception_bounding_box_; 
  // } 

  const common::math::Polygon2d& PerceptionPolygon() const {
    return perception_polygon_;
  }
  const ObjectPredictionTrajectory& Trajectory() const { return trajectory_; }
  common::TrajectoryPoint* AddTrajectoryPoint() {
    return trajectory_.add_trajectory_point();
  }
  bool HasTrajectory() const {
    return !(trajectory_.trajectory_point().empty());
  }

  const LineObject& Perception() const {
    return perception_obstacle_;
  }

  /**
   * @brief This is a helper function that can create obstacles from prediction
   * data.  The original prediction may have multiple trajectories for each
   * obstacle. But this function will create one obstacle for each trajectory.
   * @param predictions The prediction results
   * @return obstacles The output obstacles saved in a list of unique_ptr.
   */
  static std::list<std::unique_ptr<Obstacle>> CreateObstacles(
      const std::vector<LineObject>& perception_obstacles, const double start_time);

  static std::unique_ptr<Obstacle> CreateStaticVirtualObstacles(
      const std::string& id, const common::math::Box2d& obstacle_box);

  static bool IsValidPerceptionObstacle(const LineObject& obstacle);

  static bool IsValidTrajectoryPoint(const StructMissionInfo& point);

  /**
   * return the merged lateral decision
   * Lateral decision is one of {Nudge, Ignore}
   **/
  const ObjectDecisionType& LateralDecision() const;

  /**
   * @brief return the merged longitudinal decision
   * Longitudinal decision is one of {Stop, Yield, Follow, Overtake, Ignore}
   **/
  const ObjectDecisionType& LongitudinalDecision() const;

  const eObjectDecisionEnum& BehaviorLongitudinalDecision() const {
    return behavior_longitudinal_decision_;
  }

  const SLBoundary& PerceptionSLBoundary() const;

  const StBoundary& reference_line_st_boundary() const;

  const StBoundary& st_boundary() const;
  const StBoundary& cognition_st_boundary() const;

  const std::vector<std::string>& decider_tags() const;

  const std::vector<ObjectDecisionType>& decisions() const;

  void AddLongitudinalDecision(const std::string& decider_tag,
                               const ObjectDecisionType& decision);

  void AddLateralDecision(const std::string& decider_tag,
                          const ObjectDecisionType& decision);
  bool HasLateralDecision() const;

  void SetStBoundary(const StBoundary& boundary);

  void SetStBoundaryType(const StBoundary::BoundaryType type);

  void EraseStBoundary();

  void SetReferenceLineStBoundary(const StBoundary& boundary);

  void SetReferenceLineStBoundaryType(const StBoundary::BoundaryType type);

  void EraseReferenceLineStBoundary();

  bool HasLongitudinalDecision() const;

  bool HasNonIgnoreDecision() const;

  /**
   * @brief Calculate stop distance with the obstacle using the ADC's minimum
   * turning radius
   */
  double MinRadiusStopDistance(const CarModel& vehicle_param) const;

  double GetStopDistance(const CarModel& vehicle_param) const;

  void SetStopDistanceHistory(const double& dis) {
    stop_distance_history_ = dis;
  }

  /**
   * @brief Check if this object can be safely ignored.
   * The object will be ignored if the lateral decision is ignore and the
   * longitudinal decision is ignore
   *  return longitudinal_decision_ == ignore && lateral_decision == ignore.
   */
  bool IsIgnore() const;
  bool IsLongitudinalIgnore() const;
  bool IsLateralIgnore() const;

  void BuildReferenceLineStBoundary(const ReferenceLine& reference_line,
                                    const double adc_start_s, const double init_point_s, const double init_t);

  void SetPerceptionSlBoundary(const SLBoundary& sl_boundary);

  /**
   * @brief check if a ObjectDecisionType is a longitudinal decision.
   **/
  static bool IsLongitudinalDecision(const ObjectDecisionType& decision);

  /**
   * @brief check if a ObjectDecisionType is a lateral decision.
   **/
  static bool IsLateralDecision(const ObjectDecisionType& decision);

  void SetBlockingObstacle(bool blocking) { is_blocking_obstacle_ = blocking; }
  bool IsBlockingObstacle() const { return is_blocking_obstacle_; }

  /*
   * @brief IsLaneBlocking is only meaningful when IsStatic() == true.
   */
  bool IsLaneBlocking() const { return is_lane_blocking_; }
  void CheckLaneBlocking(const ReferenceLine& reference_line);

  void Set_Overlap_Points(const OverlapPoints& points) {
    overlap_points_ = points;
  }
  const OverlapPoints& overlap_points() const {
    return overlap_points_;
  }

  void AddSTUpperLowerPoints(const STPoint& _upper_point, const STPoint& _lower_point) {
    st_upper_lower_points_.AddPoint(_upper_point);
    st_upper_lower_points_.AddPoint(_lower_point);
  }
  void SetSTUpperLowerPoints(const OverlapPoints& points) {
    st_upper_lower_points_ = points;
  }
  const OverlapPoints& STUpperLowerPoints() const {
    return st_upper_lower_points_;
  }

  void SetLastFrameOverlapPoints(const OverlapPoints& points) {
    last_frame_overlap_points_ = points;
  }
  const OverlapPoints& LastFrameOverlapPoints() const {
    return last_frame_overlap_points_;
  }

  void Set_has_overlap(bool has_overlap) {
    has_overlap_ = has_overlap;
  }

  bool has_over_lap() const {
    return has_overlap_;
  }

  double nudge_l_buffer_min() const {
    return nudge_l_buffer_min_;
  }

  void SetNudgeLatBufferMin(const double& nudge_l_buffer) {
    nudge_l_buffer_min_ = nudge_l_buffer;
  }

  planning_debug_msgs::motion_obstacle TransformToMontionObsMsg() const{
    planning_debug_msgs::motion_obstacle msg;
   // msg.Id = Id();
   // msg.PerceptionId = PerceptionId();
   // msg.Perception_xg = Perception().xabs;
   // msg.Perception_yg = Perception().yabs;
   // msg.IsStatic = IsStatic() ? 1 : 0;
   // msg.IsVirtual = IsVirtual() ? 1 : 0;
    msg.set_id(id_);
    msg.set_perceptionid(perception_id_);
    msg.set_perception_xg(perception_obstacle_.xabs);
    msg.set_perception_yg(perception_obstacle_.yabs);
    msg.set_isstatic(IsStatic() ? 1 : 0);
    msg.set_isvirtual(IsVirtual() ? 1 : 0);
    return msg;
  }

 private:
  static ObjectDecisionType MergeLongitudinalDecision(
      const ObjectDecisionType& lhs, const ObjectDecisionType& rhs);
  static ObjectDecisionType MergeLateralDecision(const ObjectDecisionType& lhs,
                                                 const ObjectDecisionType& rhs);

  bool BuildTrajectoryStBoundary(const ReferenceLine& reference_line,
                                 const double adc_start_s,
                                 StBoundary* const st_boundary);
  bool IsValidObstacle(
      const LineObject& perception_obstacle);
  void UpdateObstacleDimensions(const common::math::Polygon2d& obstacle_convex_hull);    

  bool StBoundaryIsValid(
    const std::vector<std::pair<common::math::Vec2d, common::math::Vec2d>>& point_pairs) const;

 private:
  std::string id_;
  int32_t perception_id_ = 0;
  bool is_static_ = false;
  bool is_ultra_static_= false;
  bool is_movable_ = false;
  bool is_reverse_traveling_ = false;
  bool is_virtual_ = false;
  double speed_ = 0.0;
  double heading_ = 0.0;
  double speed_linear_ = 0.0;
  double acceleration_ = 0.0;
  double acceleration_max_ = 0.0;
  double acceleration_min_ = 0.0;
  bool is_waiting_ = true;
  bool was_dynamic_ = true;
  double stop_distance_history_ = -1;
  int conflict_type_ = 0;
  int conflict_type_on_target_ = 0;
  ObjectPredictionTrajectory trajectory_;
  LineObject perception_obstacle_;
  common::math::Box2d perception_bounding_box_;
  common::math::Polygon2d perception_polygon_;

  std::vector<ObjectDecisionType> decisions_;
  std::vector<std::string> decider_tags_;
  SLBoundary sl_boundary_;

  StBoundary reference_line_st_boundary_;
  StBoundary st_boundary_;

  ObjectDecisionType lateral_decision_;
  ObjectDecisionType longitudinal_decision_;
  eObjectDecisionEnum behavior_longitudinal_decision_;

  OverlapPoints overlap_points_;
  OverlapPoints last_frame_overlap_points_;
  OverlapPoints st_upper_lower_points_;
  bool has_overlap_ = false;

  // for keep_clear usage only
  bool is_blocking_obstacle_ = false;

  bool is_lane_blocking_ = false;

  double min_radius_stop_distance_ = -1.0;

  double nudge_l_buffer_min_ = 0.4;

  struct ObjectTagCaseHash {
    size_t operator()(
        const planning::ObjectDecisionType::ObjectTagCase tag) const {
      return static_cast<size_t>(tag);
    }
  };

  static const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                                  ObjectTagCaseHash>
      s_lateral_decision_safety_sorter_;
  static const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                                  ObjectTagCaseHash>
      s_longitudinal_decision_safety_sorter_;
};

typedef IndexedList<std::string, Obstacle> IndexedObstacles;
typedef ThreadSafeIndexedList<std::string, Obstacle> ThreadSafeIndexedObstacles;

}  // namespace planning
}  // namespace acu
