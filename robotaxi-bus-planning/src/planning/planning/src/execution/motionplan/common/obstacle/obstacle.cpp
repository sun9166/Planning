/**
 * @file obstacle.cpp
 **/

#include <algorithm>
#include <utility>
//#include "ros/ros.h"

#include "obstacle.h"
#include "common/util/util.h"
#include "common/util/map_util.h"
#include "common/util/string_util.h"
#include "common/base/log/include/log.h"
#include "common/math/linear_interpolation.h"
#include "src/execution/motionplan/common/ego_info.h"
#include "src/execution/motionplan/common/planning_gflags.h"
#include "src/execution/motionplan/common/speed/st_boundary.h"
#include "src/algorithm/math_util/interpolation/linear_interpolation.h"
#include "src/execution/motionplan/behavior_parser/behavior_parser.h"
#include "datapool/include/data_pool.h"
#include "src/algorithm/math_util/planning_util.h"
#include "src/execution/motionplan/common/new_path_manager.h"
namespace acu {
namespace planning {

using acu::common::util::FindOrDie;

namespace {
  const double kStBoundaryDeltaS = 0.2;        // meters
  const double kStBoundarySparseDeltaS = 1.0;  // meters
  const double kStBoundaryDeltaT = 0.05;       // seconds
}  // namespace

const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                         Obstacle::ObjectTagCaseHash>
    Obstacle::s_longitudinal_decision_safety_sorter_ = {
        {ObjectDecisionType::kIgnore, 0},
        {ObjectDecisionType::kOvertake, 100},
        {ObjectDecisionType::kFollow, 300},
        {ObjectDecisionType::kYield, 400},
        {ObjectDecisionType::kStop, 500}};

const std::unordered_map<ObjectDecisionType::ObjectTagCase, int,
                         Obstacle::ObjectTagCaseHash>
    Obstacle::s_lateral_decision_safety_sorter_ = {
        {ObjectDecisionType::kIgnore, 0}, {ObjectDecisionType::kNudge, 100}};

Obstacle::Obstacle(const std::string& id,
                   const LineObject& perception_obstacle,
                   const double& obstacle_priority,
                   const bool is_static,
                   const bool is_ultra_static,
                   const int conflict_type,
                   const int conflict_type_on_target,
                   const bool is_waiting,
                   const bool was_dynamic)
    : id_(id),
      perception_id_(perception_obstacle.id),
      perception_obstacle_(perception_obstacle),
      conflict_type_(conflict_type),
      conflict_type_on_target_(conflict_type_on_target),
      is_waiting_(is_waiting),
      is_ultra_static_(is_ultra_static),
      is_movable_(perception_obstacle.is_moveble),
      was_dynamic_(was_dynamic),
      acceleration_(perception_obstacle.acc),
      perception_bounding_box_({perception_obstacle_.xabs,
                                perception_obstacle_.yabs},
                               perception_obstacle_.global_angle * kDEG_TO_RAD,
                               perception_obstacle_.box.length(),
                               perception_obstacle_.box.width()) {
  nudge_l_buffer_min_ = FLAGS_static_decision_nudge_l_buffer;//判定是否可通行的横向距离宽度设置成默认值，后续若需要，通过Set_nudge_l_buffer_min进行更新      
  // if (is_moveable_) {
  //   // nudge_l_buffer_min_ = nudge_l_buffer_min_ + FLAGS_moveable_obstacle_l_buffer;
  //   nudge_l_buffer_min_ = FLAGS_moveable_obstacle_l_buffer;
  //   AINFO_IF(FLAGS_enable_debug_motion)<<"No["<<perception_obstacle.id 
  //      << "] is moveble set nudge min l = "<<nudge_l_buffer_min_;  
  // }
  std::vector<common::math::Vec2d> polygon_points;
  if (FLAGS_is_simulation_mode
      || perception_obstacle.cells.size() <= 2 ) {
    perception_bounding_box_.GetAllCorners(&polygon_points);
  } else {
    CHECK(perception_obstacle.cells.size() > 2)
        << "object " << id << "has less than 3 cells points";
    for (const auto& point : perception_obstacle.cells) {
      if (point.xg < 10000 || point.yg < 10000) {
        AWARN_IF(FLAGS_enable_debug_motion) << "id :" << id << " invalid cell point!!!";
      }
      polygon_points.emplace_back(point.xg, point.yg);
    }
  }
  if (polygon_points.size()) {
    common::math::Vec2d append_cell_point(
         polygon_points.back().x() + 0.01,polygon_points.back().y());
    polygon_points.emplace_back(append_cell_point);
    append_cell_point = common::math::Vec2d(
         polygon_points.back().x(),polygon_points.back().y() + 0.01);
    polygon_points.emplace_back(append_cell_point);
  }
  
  if (common::math::Polygon2d::ComputeConvexHull(polygon_points,
                                                   &perception_polygon_)) {
    if (perception_polygon_.num_points() > 0) {
      UpdateObstacleDimensions(perception_polygon_);//@pqg add 
    }
  }
  is_static_ = (is_static || 
          obstacle_priority < 0 || 
          !perception_obstacle_.sl_polygons.empty()
          || is_ultra_static_);//超低速障碍物也认为是静态的
  if (!perception_obstacle_.sl_polygons.empty()) {
    AINFO_IF(FLAGS_enable_debug_motion)<< "No["<<id << "]<< sl_polygon min s = "
                <<perception_obstacle_.sl_polygons.back().min_x()
                <<". max s = "<<perception_obstacle_.sl_polygons.back().max_x();
  }

  // is_virtual_ = (perception_obstacle.id < 0);//
  is_virtual_ = perception_obstacle.id < -1000;//
  speed_ = perception_obstacle.speed;
  speed_linear_ = perception_obstacle.vsabs;
  heading_ = perception_obstacle.global_angle;

  if (FLAGS_using_cognition_sl_boundary && !is_virtual_) {
    CHECK_LT(perception_obstacle.sl_boundary.min_s,perception_obstacle.sl_boundary.max_s);
    sl_boundary_.set_start_s(perception_obstacle.sl_boundary.min_s);
    sl_boundary_.set_end_s(perception_obstacle.sl_boundary.max_s);
    sl_boundary_.set_start_l(perception_obstacle.sl_boundary.min_l);
    sl_boundary_.set_end_l(perception_obstacle.sl_boundary.max_l);
    AINFO_IF(FLAGS_enable_debug_motion) << "cognition sl boundary: s = ("<<sl_boundary_.start_s()
                                   << ", "<<sl_boundary_.end_s()<<"). l = ("
                                   << sl_boundary_.start_l()
                                   << ", "<<sl_boundary_.end_l()<<") .";                     
  }
  //set behavior object decision 
  auto bp_ptr = BehaviorParser::instance();
  auto iterater = bp_ptr->object_decision().find(std::atoi(id_.c_str()));
  if (iterater == bp_ptr->object_decision().end()) {
    if (!is_virtual_) {
      AWARN_IF(FLAGS_enable_debug_motion)<< "struct decision has not make decision for No["<<id_<<"] obstacle, set DEFAULT";
    }
    behavior_longitudinal_decision_ = eObjectDecisionEnum::DEFAULT;
  } else {
    if (iterater->second == eObjectDecisionEnum::CONFLICT) {
      behavior_longitudinal_decision_ = eObjectDecisionEnum::IGNORE; //0520 TODO
    } else {
      behavior_longitudinal_decision_ = iterater->second;
    }
  }

  AINFO_IF(FLAGS_enable_debug_motion) << "No["<<id << "], speed :" << perception_obstacle_.speed 
    << "], acc :" << perception_obstacle_.acc << ", cells.size() :"<<perception_obstacle.cells.size()
    <<",length:" <<perception_obstacle_.box.length() <<",width:" <<perception_obstacle_.box.width() 
    << ", is_static :"<<is_static<<", type = "<<perception_obstacle_.type<<", conflict_type = "<<conflict_type_
    <<", conflict_type_on_target = "<<conflict_type_on_target_<<", behavior_longitudinal_decision_: "<<(int)behavior_longitudinal_decision_;
  if (!is_static_) {
    if (perception_obstacle.vsabs < -0.5 && perception_obstacle.is_reverse_traveling
      && sl_boundary_.start_s() > 0 
      && (perception_obstacle.lane_ptr 
          && fabs(perception_obstacle.obj_lane_l) <= 0.5*perception_obstacle.lane_ptr->GetWidth(fabs(perception_obstacle.obj_lane_s)))) {
      is_reverse_traveling_ = true;
    AWARN_IF(FLAGS_enable_debug_motion && id_ == "18882")<<"18882, is_reverse_traveling_ = "<<is_reverse_traveling_;
    }
  }  
}

void Obstacle::UpdateObstacleDimensions(const common::math::Polygon2d& obstacle_convex_hull){
  if(obstacle_convex_hull.points().empty()){
    return ;
  }
  if(!obstacle_convex_hull.IsPointIn(perception_bounding_box_.center())){
    AWARN_IF(FLAGS_enable_debug_motion) << "No:"<< id_ <<"obstacle'center is not in obstacle_convex_hull";
  }
  perception_bounding_box_ = 
     obstacle_convex_hull.BoundingBoxWithHeading(perception_bounding_box_.heading());
}

Obstacle::Obstacle(const std::string& id,
                   const LineObject& perception_obstacle,
                   const std::vector<PredictionPoint>& trajectory,
                   const double& obstacle_priority,
                   const bool is_static,
                   const bool is_ultra_static,
				           const double start_time,
                   const int conflict_type,
                   const int conflict_type_on_target,
                   const bool is_waiting,
                   const bool was_dynamic)
    : Obstacle(id, perception_obstacle, obstacle_priority, 
       is_static, is_ultra_static , conflict_type,conflict_type_on_target,is_waiting,was_dynamic) {
  trajectory_.set_probability(obstacle_priority);
  AINFO_IF(FLAGS_enable_debug_motion) << "predict trajectory points size:"<<trajectory.size();
  size_t trajectory_num = trajectory.size();
  double acc_sum = 0.0;
 
  for (size_t i = 0 ; i < trajectory_num;++i) {        //给 xy赋值
    if (trajectory[i].t < start_time) {//ignore which time behind planning start time
      continue;
    }
    auto trajectory_points = trajectory_.add_trajectory_point();
    trajectory_points->mutable_path_point()->set_x(trajectory[i].xg);
    trajectory_points->mutable_path_point()->set_y(trajectory[i].yg);
    trajectory_points->mutable_path_point()->set_theta(trajectory[i].globalangle*kDEG_TO_RAD);
    trajectory_points->set_relative_time(trajectory[i].t - start_time);
    acc_sum+=trajectory[i].a;
    if (trajectory[i].a > acceleration_max_) {
      acceleration_max_ = trajectory[i].a;
    } else if (trajectory[i].a < acceleration_min_) {
      acceleration_min_ = trajectory[i].a;
    }
  }

  double cumulative_s = 0.0;
  if (trajectory_.trajectory_point_size() > 0) {
    trajectory_.mutable_trajectory_point(0)->mutable_path_point()->set_s(0.0);
  }
  auto& trajectory_points = *trajectory_.mutable_trajectory_point();
  size_t trajectory_points_num = trajectory_points.size();
  for (size_t i = 1; i < trajectory_points_num; ++i) {
    const auto& prev = trajectory_points[i - 1];
    const auto& cur = trajectory_points[i];
    cumulative_s +=
        planning::math::DistanceXY(prev.path_point(), cur.path_point());
    trajectory_points[i].mutable_path_point()->set_s(cumulative_s);
  }
}

bool Obstacle::StBoundaryIsValid(
    const std::vector<std::pair<common::math::Vec2d, common::math::Vec2d>>& point_pairs) const {
  if (point_pairs.size() < 2) {
    AWARN_IF(FLAGS_enable_debug_motion) << "point_pairs.size() must > 2. current point_pairs.size() = "
           << point_pairs.size();
    return false;
  }

  constexpr double kStBoundaryEpsilon = 1e-9;
  constexpr double kMinDeltaT = 1e-6;
  size_t point_pairs_size = point_pairs.size();
  for (size_t i = 0; i < point_pairs_size; ++i) {
    const auto& curr_lower = point_pairs[i].first;
    const auto& curr_upper = point_pairs[i].second;
    if (curr_upper.x() < curr_lower.x()) {
      AWARN_IF(FLAGS_enable_debug_motion) << "s is not increasing";
      return false;
    }

    if (std::fabs(curr_lower.y() - curr_upper.y()) > kStBoundaryEpsilon) {
      AWARN_IF(FLAGS_enable_debug_motion) << "t diff is larger in each STPoint pair";
      return false;
    }

    if (i + 1 != point_pairs.size()) {
      const auto& next_lower = point_pairs[i + 1].first;
      const auto& next_upper = point_pairs[i + 1].second;
      if (std::fmax(curr_lower.y(), curr_upper.y()) + kMinDeltaT >=
          std::fmin(next_lower.y(), next_upper.y())) {
        AWARN_IF(FLAGS_enable_debug_motion) << "t is not increasing. curr.t = ("
             <<curr_lower.y()<<", "<<curr_upper.y()<<") , next t = ("<<next_lower.y()<<", "<<next_upper.y()<<").";
        return false;
      }
    }
  }
  return true;
}
common::TrajectoryPoint Obstacle::GetPointAtTime(
    const double relative_time) const {
  const auto& points = trajectory_.trajectory_point();
  if (points.size() < 2) {
    common::TrajectoryPoint point;
    point.mutable_path_point()->set_x(perception_obstacle_.xabs);
    point.mutable_path_point()->set_y(perception_obstacle_.yabs);
    point.mutable_path_point()->set_theta(perception_bounding_box_.heading());
    point.mutable_path_point()->set_s(0.0);
    point.mutable_path_point()->set_kappa(0.0);
    point.mutable_path_point()->set_dkappa(0.0);
    point.mutable_path_point()->set_ddkappa(0.0);
    point.set_v(0.0);
    point.set_a(0.0);
    point.set_relative_time(0.0);
    return point;
  } else {
    auto comp = [](const common::TrajectoryPoint p, const double time) {
      return p.relative_time() < time;
    };

    auto it_lower =
        std::lower_bound(points.begin(), points.end(), relative_time, comp);

    if (it_lower == points.begin()) {
      return *points.begin();
    } else if (it_lower == points.end()) {
      return *points.rbegin();
    }
    return math::InterpolateUsingLinearApproximation(
        *(it_lower - 1), *it_lower, relative_time);
  }
}

common::math::Box2d Obstacle::GetBoundingBox(
    const common::TrajectoryPoint& point) const {
  return common::math::Box2d({point.path_point().x(), point.path_point().y()},
                             point.path_point().theta(),
                             perception_obstacle_.box.length(),
                             perception_obstacle_.box.width());
}

bool Obstacle::IsValidPerceptionObstacle(const LineObject& obstacle) {
  if (obstacle.box.length() <= 0.0) {
    AWARN_IF(FLAGS_enable_debug_motion) << "invalid obstacle length:" << obstacle.box.length();
    return false;
  }
  if (obstacle.box.width() <= 0.0) {
    AWARN_IF(FLAGS_enable_debug_motion) << "invalid obstacle width:" << obstacle.box.width();
    return false;
  }

  if( obstacle.id >= 0 && obstacle.cells.empty()){
    AWARN_IF(FLAGS_enable_debug_motion) << "invalid obstacle cell size ("<< obstacle.cells.size()<<") smaller than 2";
    return true;
  }
  if (!FLAGS_is_simulation_mode && obstacle.id >= 0 && obstacle.cells.size() <= 2) {
    AWARN_IF(FLAGS_enable_debug_motion) << "invalid obstacle cell size ("<< obstacle.cells.size()<<") smaller than 2";
    return false;
  }

  for (auto pt : obstacle.cells) {
    if (std::isnan(pt.x) || std::isnan(pt.y)) {
      AWARN_IF(FLAGS_enable_debug_motion) << "invalid obstacle cell point";
      return false;
    }
    if (!FLAGS_is_simulation_mode) {
      if (std::isnan(pt.xg) || std::isnan(pt.yg) || pt.xg < 10000 || pt.yg < 10000) {
        AWARN_IF(FLAGS_enable_debug_motion) << "invalid obstacle cell point";
        return false;
      }
    }
  }
  return true;
}

std::list<std::unique_ptr<Obstacle>> Obstacle::CreateObstacles(
    const std::vector<LineObject>& perception_obstacles,const double start_time) {
  std::list<std::unique_ptr<Obstacle>> obstacles;
  const auto& DP = acu::planning::DataPool::Instance()->GetMainDataRef();
  auto current_id = DP.cognition_info.struct_env_info.reference_line_info.current_line_id;
  for (const auto& obstacle : perception_obstacles) {
    if (!IsValidPerceptionObstacle(obstacle)) {
      AWARN_IF(FLAGS_enable_debug_motion) << "Invalid perception obstacle: ";
      continue;
    }
    const auto perception_id = std::to_string(obstacle.id);
    //find currenpath conflict_type
    int conflict_type = 0;
    for (auto& ref_line : DP.cognition_info.struct_env_info.reference_line_info.current_reference_line) {
      if (current_id == ref_line.reference_lane_id) {
        for (const auto& it : ref_line.objects_) {
          auto &obj = it.second;
          if (obj.id == obstacle.id) {
            conflict_type = obj.conflict_type;
            break;
          }
        }
        break;
      }
    }

    int conflict_type_on_target = 0;
    if (!BehaviorParser::instance()->reference_lines_data().empty()) {
      for (const auto& it : BehaviorParser::instance()->target_reference_line().objects_) {
        auto &obj = it.second;
        if (obj.id == obstacle.id) {
          conflict_type_on_target = obj.conflict_type;
          AINFO_IF(FLAGS_enable_debug_motion && obj.id == 9)<<"conflict_type_on_target = "<<conflict_type_on_target;
          break;
        }
      }
    }

    double confidence = 1;//TODO
    if (obstacle.prediction.trajectories.empty() || !FLAGS_use_predict_info
         || obstacle.is_static
         || !obstacle.sl_polygons.empty()
         || obstacle.prediction.is_ultra_static) {
      obstacles.emplace_back(
          new Obstacle(perception_id, 
                       obstacle,
                       confidence,
                       obstacle.is_static,
                       obstacle.prediction.is_ultra_static,
                       conflict_type,
                       conflict_type_on_target,
                       obstacle.is_waiting,
                       obstacle.was_dynamic));
      continue;
    }
    
    std::vector<PredictionPoint> predict_trajectory;
    double probability = -1;//input maybe 0. 
    int trajectory_index = -1;
    AINFO_IF(FLAGS_enable_debug_motion) << "No[" << perception_id << "] obstacle has " 
                      << obstacle.prediction.trajectories.size()
                      << " predict trajectory .";
    if (!obstacle.is_static && obstacle.prediction.trajectories.size() <= 0) {
      AWARN_IF(FLAGS_enable_debug_motion) << "Intention !! Non-static obstacle[" << obstacle.id
             << "] has No prediction trajectory. ignore it !!";
      continue;       
    }                  
    int path_count = 0;
    for (auto &predict_path : obstacle.prediction.trajectories) {
      AINFO_IF(FLAGS_enable_debug_motion)<<"predict_path.probability = "<<predict_path.probability;
      if (predict_path.probability > probability) {
        probability = predict_path.probability;
        trajectory_index = path_count;
      }
      path_count++;
    }

    if (trajectory_index >= 0 
       && trajectory_index < obstacle.prediction.trajectories.size()) {
      bool is_static = false;
      obstacles.emplace_back(
          new Obstacle(perception_id, obstacle,
                       obstacle.prediction.trajectories.at(trajectory_index).points,
                       probability,
                       is_static,
                       obstacle.prediction.is_ultra_static,
                       start_time,
                       conflict_type,
                       conflict_type_on_target,
                       obstacle.is_waiting,
                       obstacle.was_dynamic));
    }
  }
  return obstacles;
}

std::unique_ptr<Obstacle> Obstacle::CreateStaticVirtualObstacles(
    const std::string& id, const common::math::Box2d& obstacle_box) {
  // create a "virtual" perception_obstacle
  LineObject perception_obstacle;
  // simulator needs a valid integer
  size_t negative_id = std::hash<std::string>{}(id);
  // set the first bit to 1 so negative_id became negative number
  negative_id |= (0x1 << 31);
  perception_obstacle.id = static_cast<int32_t>(negative_id);
  perception_obstacle.xabs = obstacle_box.center().x();
  perception_obstacle.yabs = obstacle_box.center().y();
  perception_obstacle.speed = 0;
  perception_obstacle.global_angle = obstacle_box.heading() * kRAD_TO_DEG;
  perception_obstacle.box.Set_length(obstacle_box.length());
  perception_obstacle.box.Set_width(obstacle_box.width());
  perception_obstacle.height = FLAGS_virtual_stop_wall_height;

  std::vector<common::math::Vec2d> corner_points;
  obstacle_box.GetAllCorners(&corner_points);
  for (const auto& corner_point : corner_points) {
    ObjectCell cell_point;  
    cell_point.xg = corner_point.x();
    cell_point.yg = corner_point.y();
    perception_obstacle.cells.push_back(cell_point);
  }
  auto* obstacle =
      new Obstacle(id, perception_obstacle, 1, true,false, 0,0,true,true);
  obstacle->is_virtual_ = true;
  return std::unique_ptr<Obstacle>(obstacle);
}

bool Obstacle::IsValidTrajectoryPoint(const StructMissionInfo& point) {
  return !(std::isnan(point.x) ||
           std::isnan(point.y) ||
           std::isnan(point.angle) );
}

void Obstacle::SetPerceptionSlBoundary(const SLBoundary& sl_boundary) {
  sl_boundary_ = sl_boundary;
}

double Obstacle::GetStopDistance(const CarModel& vehicle_param) const {
  double stop_distance = FLAGS_min_stop_distance_obstacle;
  if (min_radius_stop_distance_ > 0) {
    AERROR << "min_radius_stop_distance_ = " << min_radius_stop_distance_;
    return min_radius_stop_distance_;
  }
  bool is_in_solid_line_area = EgoInfo::instance()->is_in_solid_line_area();
  bool is_in_juction_area = EgoInfo::instance()->is_in_juction_area();
  double is_pull_over = NewPathManager::instance()->is_pull_over_path();
  double blindScope = vehicle_param.length_wheelbase + vehicle_param.front_axle_tofront;//TBD 需要添加倒车档位blindScope
  double real_dis = sl_boundary_.start_s() - blindScope;
  double ego_speed = EgoInfo::instance()->vehicle_state().linear_velocity;
  AINFO_IF(FLAGS_enable_debug_motion)<<"real_dis = "<< real_dis;
  AINFO_IF(FLAGS_enable_debug_motion)<<"ego_speed = "<< ego_speed;
  bool need_stop_close = (is_static_ && (is_waiting_ || was_dynamic_)) 
      || !is_static_ || is_in_solid_line_area || is_in_juction_area||(perception_id_<0&&perception_id_>-100);
  double stop_dis = 5;
  double max_stop_dis_to_obs = 8;
  double min_stop_dis_to_obj = 4;
  
  if (stop_distance_history_ < 0) {
    //还没有设置停车距离，第一次设置
    if (real_dis < min_stop_dis_to_obj) {
      //第一次生成stop决策时，就已经在最小停车距内了
      stop_distance = real_dis;
    } else if(real_dis < FLAGS_max_stop_distance_obstacle) {
      //在最小和最大停车距离之间
      if (need_stop_close) {
        stop_distance = min_stop_dis_to_obj;
      } else {
        stop_distance = real_dis;
      }
    } else {
      //在最大停车距离之外
      if (need_stop_close) {
        stop_distance = min_stop_dis_to_obj;
      } else {
        stop_distance = FLAGS_max_stop_distance_obstacle;
      }
    }
  } else {
    //已设置停车距离
    if (stop_distance_history_ >= FLAGS_min_stop_distance_obstacle) {
      if (need_stop_close) {
        double deceleration_max = -1;
        double deceleration_min = -1;
        double comfort_deceleration = -0.75;
        if (real_dis - max_stop_dis_to_obs > 0) {
          deceleration_max = -1 * ego_speed * ego_speed * 0.5 / (real_dis - max_stop_dis_to_obs); 
          deceleration_min = -1 * ego_speed * ego_speed * 0.5 / (real_dis - min_stop_dis_to_obj); 
          if (fabs(deceleration_min) > fabs(comfort_deceleration)) {
            stop_dis = min_stop_dis_to_obj;
          } else if(fabs(deceleration_max) > fabs(comfort_deceleration)) {
            double deceleration_dis = ego_speed * ego_speed * 0.5 / fabs(comfort_deceleration);
            if (deceleration_dis > real_dis - min_stop_dis_to_obj) {
              deceleration_dis = real_dis - min_stop_dis_to_obj;
            }
            stop_dis = real_dis - deceleration_dis;
          } else {
            stop_dis = max_stop_dis_to_obs;
          }
        } else if (real_dis - min_stop_dis_to_obj > 0) {
          deceleration_min = -1 * ego_speed * ego_speed * 0.5 / (real_dis - min_stop_dis_to_obj); 
          if (fabs(deceleration_min) > fabs(comfort_deceleration)) {
            stop_dis = min_stop_dis_to_obj;
          } else {
            double deceleration_dis = ego_speed * ego_speed * 0.5 / fabs(comfort_deceleration);
            if (deceleration_dis > real_dis - min_stop_dis_to_obj) {
              deceleration_dis = real_dis - min_stop_dis_to_obj;
            }
            stop_dis = real_dis - deceleration_dis;
          }
        } else {
          stop_dis = real_dis;
        }
        stop_distance = stop_dis;
      } else {
        if (real_dis > FLAGS_max_stop_distance_obstacle) {
          stop_distance = FLAGS_max_stop_distance_obstacle;
        } else {
          stop_distance = real_dis;
        }
      }
    } else {
      if (need_stop_close) {
        if (real_dis > min_stop_dis_to_obj) {
          stop_distance = stop_distance_history_;
        } else {
          stop_distance = real_dis;
        }
      } else {
        if (real_dis > FLAGS_max_stop_distance_obstacle) {
          stop_distance = FLAGS_max_stop_distance_obstacle;
        } else {
          stop_distance = real_dis;
        }
      }
    }
  }
  if(is_pull_over&&is_static_){
    stop_distance=std::min(stop_distance,2.5);
  }
  AINFO_IF(FLAGS_enable_debug_motion)<<"Stop obj id = "<< id_ <<", stop dis history = "<< stop_distance_history_;
  AINFO_IF(FLAGS_enable_debug_motion)<<"stop_distance = "<< stop_distance;
  return std::max(stop_distance, 2.0);
}

double Obstacle::MinRadiusStopDistance(
    const CarModel& vehicle_param) const {
  double stop_distance = FLAGS_min_stop_distance_obstacle;
  if (min_radius_stop_distance_ > 0) {
    return min_radius_stop_distance_;
  }
  
  constexpr double stop_distance_buffer = 0.5;
  const double min_turn_radius = vehicle_param.min_turning_radius;
  double lateral_diff =
      vehicle_param.half_wheel + std::max(std::fabs(sl_boundary_.start_l()),
                                             std::fabs(sl_boundary_.end_l()));
  const double kEpison = 1e-5;
  lateral_diff = std::min(lateral_diff, min_turn_radius - kEpison);
  stop_distance =
      std::sqrt(std::fabs(min_turn_radius * min_turn_radius -
                          (min_turn_radius - lateral_diff) *
                              (min_turn_radius - lateral_diff))) +
      stop_distance_buffer;
  double front_edge_to_center = vehicle_param.length_wheelbase + vehicle_param.front_axle_tofront;    
  stop_distance -= front_edge_to_center;
  AERROR_IF(FLAGS_enable_debug_motion)<<"stop_distance = "<<stop_distance;
  
  return stop_distance;
}

void Obstacle::BuildReferenceLineStBoundary(const ReferenceLine& reference_line,
                                            const double adc_start_s, const double init_point_s, const double init_t) { 
  if (FLAGS_using_cognition_st_boundary && !is_virtual_ && !is_static_) {
    std::vector<std::pair<STPoint, STPoint>> point_pairs;
    AINFO_IF(FLAGS_enable_debug_motion)<<"No["<< perception_id_<<"] st size = "<<perception_obstacle_.st_boundary.s_t.size();
    if (perception_obstacle_.st_boundary.s_t.size()) {
      std::vector<STPoint> upper_points;
      std::vector<STPoint> lower_points;
      if (StBoundaryIsValid(perception_obstacle_.st_boundary.s_t)) {
        for (const auto& pair_point:perception_obstacle_.st_boundary.s_t) {
          lower_points.push_back(STPoint(pair_point.first.x(),pair_point.first.y()));
          upper_points.push_back(STPoint(pair_point.second.x(),pair_point.second.y()));
        }
        reference_line_st_boundary_ = StBoundary::GenerateStBoundary(lower_points, upper_points);
        AINFO_IF(FLAGS_enable_debug_motion)<<"input st = ["<<reference_line_st_boundary_.lower_points().front().t()
                                   << ", "<<reference_line_st_boundary_.lower_points().back().s()<<"], ["
                                   <<reference_line_st_boundary_.lower_points().back().t() << ", "
                                   <<reference_line_st_boundary_.lower_points().back().s()<<"] .";
        if (init_point_s > 0) {
          reference_line_st_boundary_ = reference_line_st_boundary_.ModifyByST(init_t ,init_point_s);
        }
        reference_line_st_boundary_.SetObstacleId(id_, is_virtual_);
        reference_line_st_boundary_.SetSpeed(speed_);
        AINFO_IF(FLAGS_enable_debug_motion)<<"start point ts ("<<init_t<<", "<<init_point_s<< ", No["<< perception_id_<<"] cognition modify st_boundary ["
                                   <<reference_line_st_boundary_.min_t()
                                   << ", "<<reference_line_st_boundary_.min_s()<<"], ["
                                   <<reference_line_st_boundary_.max_t() << ", "
                                   <<reference_line_st_boundary_.max_s()<<"] .";
      } else {
        AERROR_IF(FLAGS_enable_debug_motion)<<"cognition st_boundary is invalid!!!!!";
      }
    }                                     
    return;                               
  }

  CarModel vehicle_param = EgoInfo::instance()->vehicle_param(); 

  const double adc_width = vehicle_param.car_width;
  if (is_static_ || trajectory_.trajectory_point().empty()) {
    std::vector<std::pair<STPoint, STPoint>> point_pairs;
    double start_s = sl_boundary_.start_s();
    double end_s = sl_boundary_.end_s();
    if (end_s - start_s < kStBoundaryDeltaS) {
      end_s = start_s + kStBoundaryDeltaS;
    }
    if (!reference_line.IsBlockRoad(perception_bounding_box_, adc_width)) {
      return;
    }
    point_pairs.emplace_back(STPoint(start_s - adc_start_s, 0.0),
                             STPoint(end_s - adc_start_s, 0.0));
    point_pairs.emplace_back(STPoint(start_s - adc_start_s, FLAGS_st_max_t),
                             STPoint(end_s - adc_start_s, FLAGS_st_max_t));
    reference_line_st_boundary_ = StBoundary(point_pairs);
  } else {
    if (BuildTrajectoryStBoundary(reference_line, adc_start_s,
                                  &reference_line_st_boundary_)) {
    } else {
      ADEBUG << "No st_boundary for obstacle " << id_;
    }
  }
}

bool Obstacle::BuildTrajectoryStBoundary(const ReferenceLine& reference_line,
                                         const double adc_start_s,
                                         StBoundary* const st_boundary) {
  CarModel vehicle_param = EgoInfo::instance()->vehicle_param();;

  if (!IsValidObstacle(perception_obstacle_)) {
    AWARN_IF(FLAGS_enable_debug_motion) << "Fail to build trajectory st boundary because object is not "
              "valid. ";
    return false;
  }
  const double object_width = perception_obstacle_.box.width();
  const double object_length = perception_obstacle_.box.length();
  const auto& trajectory_points = trajectory_.trajectory_point();
  if (trajectory_points.empty()) {
    return false;
  }
  const double adc_length = vehicle_param.length;
  const double adc_half_length = adc_length / 2.0;
  const double adc_width = vehicle_param.half_wheel * 2.0;
  common::math::Box2d min_box({0, 0}, 1.0, 1.0, 1.0);
  common::math::Box2d max_box({0, 0}, 1.0, 1.0, 1.0);
  std::vector<std::pair<STPoint, STPoint>> polygon_points;

  SLBoundary last_sl_boundary;
  int last_index = 0;
  size_t trajectory_points_num = trajectory_points.size();
  for (size_t i = 1; i < trajectory_points_num; ++i) {
    const auto& first_traj_point = trajectory_points[i - 1];
    const auto& second_traj_point = trajectory_points[i];
    const auto& first_point = first_traj_point.path_point();
    const auto& second_point = second_traj_point.path_point();

    double total_length =
        object_length + planning::math::DistanceXY(first_point, second_point);

    common::math::Vec2d center((first_point.x() + second_point.x()) / 2.0,
                               (first_point.y() + second_point.y()) / 2.0);
    common::math::Box2d object_moving_box(center, first_point.theta(),
                                          total_length, object_width);
    SLBoundary object_boundary;
    // NOTICE: this method will have errors when the reference line is not
    // straight. Need double loop to cover all corner cases.
    const double distance_xy =
        planning::math::DistanceXY(trajectory_points[last_index].path_point(),
                                 trajectory_points[i].path_point());
    if (last_sl_boundary.start_l() > distance_xy ||
        last_sl_boundary.end_l() < -distance_xy) {
      continue;
    }

    const double mid_s =
        (last_sl_boundary.start_s() + last_sl_boundary.end_s()) / 2.0;
    const double start_s = std::fmax(0.0, mid_s - 2.0 * distance_xy);
    const double end_s = (i == 1) ? reference_line.Length()
                                  : std::fmin(reference_line.Length(),
                                              mid_s + 2.0 * distance_xy);

    if (!reference_line.GetApproximateSLBoundary(object_moving_box, start_s,
                                                 end_s, &object_boundary)) {
      AWARN_IF(FLAGS_enable_debug_motion) << "failed to calculate boundary";
      return false;
    }

    // update history record
    last_sl_boundary = object_boundary;
    last_index = i;

    // skip if object is entirely on one side of reference line.
    constexpr double kSkipLDistanceFactor = 0.4;
    const double skip_l_distance =
        (object_boundary.end_s() - object_boundary.start_s()) *
            kSkipLDistanceFactor +
        adc_width / 2.0;

    if (std::fmin(object_boundary.start_l(), object_boundary.end_l()) >
            skip_l_distance ||
        std::fmax(object_boundary.start_l(), object_boundary.end_l()) <
            -skip_l_distance) {
      continue;
    }

    if (object_boundary.end_s() < 0) {  // skip if behind reference line
      continue;
    }
    constexpr double kSparseMappingS = 20.0;
    const double st_boundary_delta_s =
        (std::fabs(object_boundary.start_s() - adc_start_s) > kSparseMappingS)
            ? kStBoundarySparseDeltaS
            : kStBoundaryDeltaS;
    const double object_s_diff =
        object_boundary.end_s() - object_boundary.start_s();
    if (object_s_diff < st_boundary_delta_s) {
      continue;
    }
    const double delta_t =
        second_traj_point.relative_time() - first_traj_point.relative_time();
    double low_s = std::max(object_boundary.start_s() - adc_half_length, 0.0);
    bool has_low = false;
    double high_s =
        std::min(object_boundary.end_s() + adc_half_length, FLAGS_st_max_s);
    bool has_high = false;
    while (low_s + st_boundary_delta_s < high_s && !(has_low && has_high)) {
      if (!has_low) {
        auto low_ref = reference_line.GetReferencePoint(low_s);
        has_low = object_moving_box.HasOverlap(
            {low_ref, low_ref.heading(), adc_length, adc_width});
        low_s += st_boundary_delta_s;
      }
      if (!has_high) {
        auto high_ref = reference_line.GetReferencePoint(high_s);
        has_high = object_moving_box.HasOverlap(
            {high_ref, high_ref.heading(), adc_length, adc_width});
        high_s -= st_boundary_delta_s;
      }
    }
    if (has_low && has_high) {
      low_s -= st_boundary_delta_s;
      high_s += st_boundary_delta_s;
      double low_t =
          (first_traj_point.relative_time() +
           std::fabs((low_s - object_boundary.start_s()) / object_s_diff) *
               delta_t);
      polygon_points.emplace_back(
          std::make_pair(STPoint{low_s - adc_start_s, low_t},
                         STPoint{high_s - adc_start_s, low_t}));
      double high_t =
          (first_traj_point.relative_time() +
           std::fabs((high_s - object_boundary.start_s()) / object_s_diff) *
               delta_t);
      if (high_t - low_t > 0.05) {
        polygon_points.emplace_back(
            std::make_pair(STPoint{low_s - adc_start_s, high_t},
                           STPoint{high_s - adc_start_s, high_t}));
      }
    }
  }
  if (!polygon_points.empty()) {
    std::sort(polygon_points.begin(), polygon_points.end(),
              [](const std::pair<STPoint, STPoint>& a,
                 const std::pair<STPoint, STPoint>& b) {
                return a.first.t() < b.first.t();
              });
    auto last = std::unique(polygon_points.begin(), polygon_points.end(),
                            [](const std::pair<STPoint, STPoint>& a,
                               const std::pair<STPoint, STPoint>& b) {
                              return std::fabs(a.first.t() - b.first.t()) <
                                     kStBoundaryDeltaT;
                            });
    polygon_points.erase(last, polygon_points.end());
    if (polygon_points.size() > 2) {
      *st_boundary = StBoundary(polygon_points);
    }
  } else {
    return false;
  }
  return true;
}

const StBoundary& Obstacle::reference_line_st_boundary() const {
  return reference_line_st_boundary_;
}

const StBoundary& Obstacle::st_boundary() const { return st_boundary_; }

const std::vector<std::string>& Obstacle::decider_tags() const {
  return decider_tags_;
}

const std::vector<ObjectDecisionType>& Obstacle::decisions() const {
  return decisions_;
}

bool Obstacle::IsLateralDecision(const ObjectDecisionType& decision) {
  return decision.has_ignore() || decision.has_nudge();
}

bool Obstacle::IsLongitudinalDecision(const ObjectDecisionType& decision) {
  return decision.has_ignore() || decision.has_stop() || decision.has_yield() ||
         decision.has_follow() || decision.has_overtake();
}

ObjectDecisionType Obstacle::MergeLongitudinalDecision(
    const ObjectDecisionType& lhs, const ObjectDecisionType& rhs) {
  if (lhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return rhs;
  }
  if (rhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return lhs;
  }
  const auto lhs_val =
      FindOrDie(s_longitudinal_decision_safety_sorter_, lhs.object_tag_case());
  const auto rhs_val =
      FindOrDie(s_longitudinal_decision_safety_sorter_, rhs.object_tag_case());
  if (lhs_val < rhs_val) {
    return rhs;
  } else if (lhs_val > rhs_val) {
    return lhs;
  } else {
    if (lhs.has_ignore()) {
      return rhs;
    } else if (lhs.has_stop()) {
      return lhs.stop().distance_s() < rhs.stop().distance_s() ? lhs : rhs;
    } else if (lhs.has_yield()) {
      return lhs.yield().distance_s() < rhs.yield().distance_s() ? lhs : rhs;
    } else if (lhs.has_follow()) {
      return lhs.follow().distance_s() < rhs.follow().distance_s() ? lhs : rhs;
    } else if (lhs.has_overtake()) {
      return lhs.overtake().distance_s() > rhs.overtake().distance_s() ? lhs
                                                                       : rhs;
    } else {
      DCHECK(false) << "Unknown decision";
    }
  }
  return lhs;  // stop compiler complaining
}

const ObjectDecisionType& Obstacle::LongitudinalDecision() const {
  return longitudinal_decision_;
}

const ObjectDecisionType& Obstacle::LateralDecision() const {
  return lateral_decision_;
}

bool Obstacle::IsIgnore() const {
  return IsLongitudinalIgnore() && IsLateralIgnore();
}

bool Obstacle::IsLongitudinalIgnore() const {
  return longitudinal_decision_.has_ignore();
}

bool Obstacle::IsLateralIgnore() const {
  return lateral_decision_.has_ignore();
}

ObjectDecisionType Obstacle::MergeLateralDecision(
    const ObjectDecisionType& lhs, const ObjectDecisionType& rhs) {
  if (lhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return rhs;
  }
  if (rhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return lhs;
  }
  const auto lhs_val =
      FindOrDie(s_lateral_decision_safety_sorter_, lhs.object_tag_case());
  const auto rhs_val =
      FindOrDie(s_lateral_decision_safety_sorter_, rhs.object_tag_case());
  if (lhs_val < rhs_val) {
    return rhs;
  } else if (lhs_val > rhs_val) {
    return lhs;
  } else {
    if (lhs.has_ignore()) {
      return rhs;
    } else if (lhs.has_nudge()) {
      DCHECK(lhs.nudge().type() == rhs.nudge().type())
          << "could not merge left nudge and right nudge";
      return std::fabs(lhs.nudge().distance_l()) >
                     std::fabs(rhs.nudge().distance_l())
                 ? lhs
                 : rhs;
    }
  }
  DCHECK(false) << "Does not have rule to merge decision: "
                << lhs.ShortDebugString()
                << " and decision: " << rhs.ShortDebugString();
  return lhs;
}

bool Obstacle::HasLateralDecision() const {
  return lateral_decision_.object_tag_case() !=
         ObjectDecisionType::OBJECT_TAG_NOT_SET;
}

bool Obstacle::HasLongitudinalDecision() const {
  return longitudinal_decision_.object_tag_case() !=
         ObjectDecisionType::OBJECT_TAG_NOT_SET;
}

bool Obstacle::HasNonIgnoreDecision() const {
  return (HasLateralDecision() && !IsLateralIgnore()) ||
         (HasLongitudinalDecision() && !IsLongitudinalIgnore());
}

void Obstacle::AddLongitudinalDecision(const std::string& decider_tag,
                                       const ObjectDecisionType& decision) {
  DCHECK(IsLongitudinalDecision(decision))
      << "Decision: " << decision.ShortDebugString()
      << " is not a longitudinal decision";
  longitudinal_decision_ =
      MergeLongitudinalDecision(longitudinal_decision_, decision);
  decisions_.push_back(decision);
  decider_tags_.push_back(decider_tag);
}

void Obstacle::AddLateralDecision(const std::string& decider_tag,
                                  const ObjectDecisionType& decision) {
  DCHECK(IsLateralDecision(decision))
      << "Decision: " << decision.ShortDebugString()
      << " is not a lateral decision";
  lateral_decision_ = MergeLateralDecision(lateral_decision_, decision);
  decisions_.push_back(decision);
  decider_tags_.push_back(decider_tag);
}

const SLBoundary& Obstacle::PerceptionSLBoundary() const {
  return sl_boundary_;
}

void Obstacle::SetStBoundary(const StBoundary& boundary) {
  st_boundary_ = boundary;
}

void Obstacle::SetStBoundaryType(const StBoundary::BoundaryType type) {
  st_boundary_.SetBoundaryType(type);
}

void Obstacle::EraseStBoundary() { st_boundary_ = StBoundary(); }

void Obstacle::SetReferenceLineStBoundary(const StBoundary& boundary) {
  reference_line_st_boundary_ = boundary;
}

void Obstacle::SetReferenceLineStBoundaryType(
    const StBoundary::BoundaryType type) {
  reference_line_st_boundary_.SetBoundaryType(type);
}

void Obstacle::EraseReferenceLineStBoundary() {
  reference_line_st_boundary_ = StBoundary();
}

bool Obstacle::IsValidObstacle(
    const LineObject& perception_obstacle) {
  const double object_width = perception_obstacle.box.width();
  const double object_length = perception_obstacle.box.length();

  const double kMinObjectDimension = 1.0e-6;
  return !std::isnan(object_width) && !std::isnan(object_length) &&
         object_width > kMinObjectDimension &&
         object_length > kMinObjectDimension;
}

void Obstacle::CheckLaneBlocking(const ReferenceLine& reference_line) {
  if (!IsStatic()) {
    is_lane_blocking_ = false;
    return;
  }

  CarModel vehicle_param = EgoInfo::instance()->vehicle_param(); 

  DCHECK(sl_boundary_.has_start_s());
  DCHECK(sl_boundary_.has_end_s());
  DCHECK(sl_boundary_.has_start_l());
  DCHECK(sl_boundary_.has_end_l());

  if (sl_boundary_.start_l() * sl_boundary_.end_l() < 0.0) {
    is_lane_blocking_ = true;
    return;
  }

  double boudary_l_min = std::fmin(fabs(sl_boundary_.start_l()),fabs(sl_boundary_.end_l()));
  if (reference_line.IsOnLane(sl_boundary_) 
      && boudary_l_min < 
          vehicle_param.half_wheel + FLAGS_static_decision_nudge_l_buffer) {
    AINFO_IF(FLAGS_enable_debug_motion) << "is_lane_blocking !!!";
    is_lane_blocking_ = true;
    return;
  } 
  is_lane_blocking_ = false;
}

}  // namespace planning
}  // namespace acu
