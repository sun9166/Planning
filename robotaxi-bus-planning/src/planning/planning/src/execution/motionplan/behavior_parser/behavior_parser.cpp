
/**
 * @file behaviorplan_info.cpp
 **/

#include "behavior_parser.h"
#include "src/execution/motionplan/common/new_path_manager.h"
#include "common/base/log/include/log.h"
#include "src/execution/motionplan/common/ego_info.h"
#include "src/execution/motionplan/common/scenario_context.h"
#include "datapool/include/data_pool.h"
#include "common/mapcheck/geotool/include/coordtransform.h"

#define FLAGS_lane_width 3.50
namespace acu {
namespace planning {

BehaviorParser::BehaviorParser() {
  
}

void BehaviorParser::Reset() {
  pull_over_sl_.clear(); 
  pull_over_box_.clear();
  pull_over_modified_l_ = 99;
  ResetDecisionHistory();
  is_start_auto_drive_ = false;
  decision_speed_limit_ = 100;
  history_line_block_dis_ = 1000; 
  history_block_obs_id_ = -1;
  last_local_path_in_current_ = false;
  characteristic_obstacle_id_ = -1;
  decision_meeting_ids_.clear();
  decision_dynamic_obstacle_boxes_.clear();
  decision_expand_l_ = 0.0;
  decision_expect_right_most_ = false;
  decision_recommended_a_ = std::numeric_limits<double>::max();
}

int BehaviorParser::GetTargetLineId(const DecisionInfo& decision) {
  if (decision.sentence.empty()) {
    return decision.reference_line_id;
  } else {
    for (auto& sentence : decision.sentence) {
      if (sentence.action == eActionEnum::PULL_OVER) {
        AINFO_IF(FLAGS_enable_debug_motion)<<"pullover_line_id: "<<sentence.pullover_line_id<<", reference_line_id: "<<decision.reference_line_id;
        return sentence.pullover_line_id > 0 ? sentence.pullover_line_id : decision.reference_line_id;
      }
    }
  }
  return decision.reference_line_id;
}

bool BehaviorParser::Update(const DecisionInfo& decision,const StructReferenceLineInfo* cognition_info) {
  auto DP_ptr = acu::planning::DataPool::Instance()->GetMainDataPtr();    
  ResetDecisionHistory();
  auto DP = DataPool::Instance();
  bool now_stop_or_suspend_task_=(DP->GetMainDataRef().task_content.command == eCommand::COMMAND_STOP || 
  DP->GetMainDataRef().task_content.command == eCommand::COMMAND_SUSPEND);
  if(!last_stop_or_suspend_task_&&now_stop_or_suspend_task_) {
    is_stop_or_suspend_task_= true;
  }else{
    is_stop_or_suspend_task_= false;
  }
  last_stop_or_suspend_task_= now_stop_or_suspend_task_;

  last_local_path_in_current_ = cognition_info->path_in_current;
  current_reference_line_id_ = 
               cognition_info->current_line_id;
  bool need_stop = false;
  target_reference_line_id_ = GetTargetLineId(decision);
  characteristic_obstacle_id_ = decision.follow_id;
  AINFO_IF(FLAGS_enable_debug_motion && characteristic_obstacle_id_ > 0)
       <<"characteristic_obstacle_id_ = "<<characteristic_obstacle_id_;
  is_decision_success_ = decision.object_decision_success;
  decision_path_bound_ = decision.path_bound;
  lane_borrow_info_ = (LaneBorrowInfo)decision.borrow_lane_type;
  lanechange_prepare_direction_ = decision.prepare_direction;
  AINFO_IF(FLAGS_enable_debug_motion)<<"lanechange_prepare_direction_ = "<<lanechange_prepare_direction_
    <<" borrow_lane_type "<<decision.borrow_lane_type;
  lane_change_target_gap_ = decision.gap_id;
  is_congestion_ = decision.is_congestion;
  //简化版动态障碍物决策结果
  decision_meeting_ids_ = decision.meeting_ids;
  decision_dynamic_obstacle_boxes_ = decision.dynamic_sl;
  AINFO_IF(FLAGS_enable_debug_motion)<<"parser decision_dynamic_obstacle_boxes_: "<<decision_dynamic_obstacle_boxes_.size();
  decision_expand_l_ = decision.expand_l;
  AINFO_IF(FLAGS_enable_debug_motion)<<"parser decision_expand_l_: "<<decision_expand_l_;
  decision_expect_right_most_ = (decision_expand_l_ > 10.0) ? true : false;
  decision_recommended_a_ = (decision.recommended_a < 1e3) ? decision.recommended_a : decision_recommended_a_;

  AINFO_IF(FLAGS_enable_debug_motion)<<"is_congestion_ = "<<is_congestion_;
  if (!FindTargetRefLine(cognition_info)) {
    AERROR_IF(FLAGS_enable_debug_motion) << "cannot find target reference line in "
                           <<"all ref_line_infos of congnition,  decision's target lane id error !!";
    behavior_parse_failed_msg_ = "cannot find target reference line.";                   
    return false;
  }
  double distance_to_junction = target_reference_line_.mapinfo.distance_to_junctions.size() ? 
                               target_reference_line_.mapinfo.distance_to_junctions.front().first : std::numeric_limits<double>::max();
  int stop_behavior_size = 0;          
  if (decision.sentence.empty()) {
    AINFO_IF(FLAGS_enable_debug_motion)<<"(decision.sentence is empty, set current scenario : LANE_FOLLOWING ."
         <<", target line id = "<<target_reference_line_id_;
  } else {
    for (auto& sentence : decision.sentence) {
      DP_ptr->debug_planning_msg.motionplan_debug.set_sentence_action((int)sentence.action);      
      if (sentence.action == eActionEnum::START) {
        is_start_auto_drive_ = true;
        lateral_action_ = std::make_pair(sentence.action,sentence.direction);
        AINFO_IF(FLAGS_enable_debug_motion)<<"decision :: START. "
           << ", target_reference_line_id = " << target_reference_line_id_;
      } else if (sentence.action == eActionEnum::LANE_FOLLOWING) {
        AINFO_IF(FLAGS_enable_debug_motion)<<"decision :: LANE_FOLLOWING. "
           << ", target_reference_line_id = " << target_reference_line_id_;
      } else if (sentence.action == eActionEnum::LANE_CHANGE) {
        has_decision_lateral_behavior_ = true;
        lateral_action_ = std::make_pair(sentence.action,sentence.direction);
        AINFO_IF(FLAGS_enable_debug_motion)<<"decision :: LANE_CHANGE. "
           << ", target_reference_line_id = " << target_reference_line_id_;
      } else if (sentence.action == eActionEnum::OBSTACLE_AVOID) {
        has_decision_lateral_behavior_ = true;
        lateral_action_ = std::make_pair(sentence.action,sentence.direction);
        AINFO_IF(FLAGS_enable_debug_motion)<<"decision :: OBSTACLE_AVOID. "
           << ", target_reference_line_id = " << target_reference_line_id_;
      } else if (sentence.action == eActionEnum::STOP) {
        stop_behavior_size++;
        AINFO_IF(FLAGS_enable_debug_motion)<<"decision :: STOP. "
           << ", target_reference_line_id = " << target_reference_line_id_;
        need_stop = true;
        VirtualObstacle stop_wall;
        stop_wall.obstacle_box = sentence.box;
        stop_wall.id = distance_to_junction < 10 ? //用于区分红灯和其他虚拟障碍物，但只在距离路口<10m时生效。
                       "red_light_" + std::to_string(stop_behavior_size)
                       : "stop_" + std::to_string(stop_behavior_size); 
        AddVirtualObstacle(stop_wall);   
      } else if (sentence.action == eActionEnum::PULL_OVER) {
        if (!FLAGS_enable_pull_over_plan) continue;
        pullover_line_id_ = sentence.pullover_line_id;
        AINFO_IF(FLAGS_enable_debug_motion)<<"pullover_line_id_: "<<pullover_line_id_<<", pullover_line_id: "<<sentence.pullover_line_id;
        has_decision_lateral_behavior_ = true;
        pull_over_sl_.clear(); 
        pull_over_box_.clear();  
        lateral_action_ = std::make_pair(sentence.action,sentence.direction);
        dis_to_curb_ = sentence.dis_to_boundary;
        if (!FLAGS_use_road_boundary_in_pull_over) {
          AINFO_IF(FLAGS_enable_debug_motion)<<"decision :: PULLOVER. "
           << ", s: " << sentence.dis_to_end << " delta l: "<< sentence.dis_to_boundary;
          common::SLPoint pull_over_sl;
          pull_over_sl.set_s(std::max(0.0, sentence.dis_to_end));
          pull_over_sl.set_l(std::min(-FLAGS_lane_width / 2.0 + 
               sentence.dis_to_boundary + EgoInfo::instance()->vehicle_param().half_wheel,0.0));
          pull_over_sl_.push_back(pull_over_sl);
        }
      } else if (sentence.action == eActionEnum::ABANDON_LANE_CHANGE) {
        has_decision_lateral_behavior_ = true;
        abandon_lc_level_ = (int)sentence.intention;
        AINFO_IF(FLAGS_enable_debug_motion)<<"decision :: ABANDON_LANE_CHANGE. intention = "<<(int)sentence.intention;
        lateral_action_ = std::make_pair(sentence.action,sentence.direction);
      } else if (sentence.action == eActionEnum::CHANGE_OFFSET) {
        has_decision_lateral_behavior_ = true;
        AINFO_IF(FLAGS_enable_debug_motion)<<"decision :: CHANGE_OFFSET. ";
        if (sentence.direction == eDirectionEnum::DEFAULT_VALUE
          && !(ScenarioContext::instance()->scenario_info().current_latscenario 
               == ScenarioContext::eLatScenarioEnum::NUDGE_OFFSET)) {
          AWARN_IF(FLAGS_enable_debug_motion)<<"but current lat senario is not NUDGE_OFFSET .";
          lateral_action_= std::make_pair(eActionEnum::DEFAULT_VALUE,eDirectionEnum::DEFAULT_VALUE);
        } else {
          lateral_action_ = std::make_pair(sentence.action,sentence.direction);
        }
      }
    }
  }

  if (lanechange_prepare_direction_ < 1 
    && lateral_action_.first == eActionEnum::LANE_CHANGE) {//决策没发准备换道状态，但当前时刻为换道行为
    //判断一下是否还需要短距离再换一条道，是，则把lanechange_prepare_direction_置成对应的换道方向，以实现这次规划的换道路径偏向于未来下一次换道的方向
    if (fabs(target_reference_line_.mapinfo.first_lc_time) >= 1 
        && target_reference_line_.mapinfo.first_lc_index <= 1) {
      lanechange_prepare_direction_ = (int)(lateral_action_.second);
      AINFO_IF(FLAGS_enable_debug_motion)<<"lanechange_prepare_direction_ = "<<lanechange_prepare_direction_; 
    }
  }

  if (decision.is_arrive_target_line_failed) {
    target_ref_line_change_ = true;
  } else if (target_reference_line_id_history_ != target_reference_line_id_ && has_decision_lateral_behavior_) {
    target_ref_line_change_ = true;
  } else {
    target_ref_line_change_ = false;
  }
  AINFO_IF(FLAGS_enable_debug_motion && cognition_info->task_change)<<"cognition_info->task_change !!!";
  if (lateral_action_.first == eActionEnum::DEFAULT_VALUE 
    && cognition_info->task_change) {//决策没有横向行为，如果这时候切换任务，为了避免任务切换了，轨迹没切，强制重规划路径一次
    lateral_action_.first = eActionEnum::OBSTACLE_AVOID;

  }
  if (cognition_info->task_change) target_ref_line_change_ = true;//任务切换，参考线也需重新平滑一次，避免问题

  target_reference_line_id_history_ = target_reference_line_id_;
  if (is_stop_history_ && !need_stop) {
    is_restart_ = true;
    int color = 1;
    if (!target_reference_line_.mapinfo.trafficlight.history_light.empty()) {
      color = target_reference_line_.mapinfo.trafficlight.history_light.back().color;
    }
    restart_by_green_light_ = color == 3;
  }
  is_stop_history_ = need_stop;

  obstacle_decisions_ = decision.object_decision;//@pqg obstacle decision
  AINFO_IF(FLAGS_enable_debug_motion)<<"obj decision size "<<obstacle_decisions_.size()
    <<" restart_by_green_light_ "<<restart_by_green_light_;
  for (const auto& obj:obstacle_decisions_) {
    AINFO_IF(FLAGS_enable_debug_motion)<<"obj["<<obj.first<<"] decision: "<<(int)obj.second;
  }

  if (!CreateReferenceLineData(cognition_info)) {
    return false;
  }

  if (!decision_path_bound_.empty()) { //in order to avoid vehicle has overlap with solid line when lane change 
    for (size_t i = 0 ; i < decision_path_bound_.size();++i) {
      if (get<1>(decision_path_bound_.at(i)) == get<2>(decision_path_bound_.at(i))) {//@pqg 0318new
        std::get<0>(decision_path_bound_.at(i)) = 
                std::get<0>(decision_path_bound_.at(i)) + EgoInfo::instance()->vehicle_front_edge_to_center();
        if (i > 0) {
          std::get<0>(decision_path_bound_.at(i - 1)) = 
                std::get<0>(decision_path_bound_.at(i - 1)) - EgoInfo::instance()->vehicle_front_edge_to_center();
        }
      } else {
        //keep it
        plan_in_lane_ = false;//左右边界不一致，说明不是车道内避障。
      }                
    }
    for (const auto& bound_tuple :decision_path_bound_) {
      AINFO_IF(FLAGS_enable_debug_motion)<<"decision path_bound s = "<<get<0>(bound_tuple)
          <<", l = ("<<get<1>(bound_tuple)<<", "<<get<2>(bound_tuple)<<"). ";
    }
  } else {
    //current lane
    AINFO_IF(FLAGS_enable_debug_motion)<<"no decision bound, set default bound.";
    decision_path_bound_.clear();
    if (target_reference_line_id_ == 20 ) {
      decision_path_bound_.push_back(make_tuple(200,
                                      target_reference_line_id_,
                                      current_reference_line_id_));
    } else if (target_reference_line_id_ == 30) {
      decision_path_bound_.push_back(make_tuple(200,current_reference_line_id_,
                                      target_reference_line_id_));
    } else {
      decision_path_bound_.push_back(make_tuple(200,
                                      target_reference_line_id_,
                                      target_reference_line_id_));
    }
  }

  if (std::get<0>(decision_path_bound_.back()) < target_reference_line_.Length()) {
    decision_path_bound_.push_back(make_tuple(target_reference_line_.Length() + 10.0,
                                      target_reference_line_id_,
                                      target_reference_line_id_));
  }
  
  if (FLAGS_use_road_boundary_in_pull_over && FLAGS_enable_pull_over_plan) {
    for (auto& sentence : decision.sentence) {
      if (sentence.action == eActionEnum::PULL_OVER) {
        AINFO_IF(FLAGS_enable_debug_motion)<<"decision :: PULLOVER. "
           << ", s: " << sentence.dis_to_end << " delta l: "<< sentence.dis_to_boundary;
        pull_over_sl_.clear(); 
        pull_over_box_.clear();  
        if (reference_lines_data_.empty() ||
            reference_lines_data_.front().mapinfo.path_points.empty()) {
          AINFO_IF(FLAGS_enable_debug_motion)<<"reference_lines_data_.empty!";
          continue;
        }
        double road_left_width = 0;
        double road_right_width = 0;
        if (!reference_lines_data_.front().GetWidthToRoadBoundary(road_left_width,
                          road_right_width,sentence.dis_to_end,false)) {
          AINFO_IF(FLAGS_enable_debug_motion)<<"get road boundary failed!";
          continue;
        }
        CHECK_GT(fabs(road_left_width),0.0);
        CHECK_GT(fabs(road_right_width),0.0);
        AINFO_IF(FLAGS_enable_debug_motion)<<"road_right_width = "<<road_right_width;
        lateral_action_ = std::make_pair(sentence.action,sentence.direction);
        common::SLPoint pull_over_sl; 
        pull_over_sl.set_s(std::max(0.0, sentence.dis_to_end));
        pull_over_sl.set_l(std::min(-fabs(road_right_width) + 
             sentence.dis_to_boundary + EgoInfo::instance()->vehicle_param().half_wheel,0.0));
        pull_over_sl_.push_back(pull_over_sl);
        AINFO_IF(FLAGS_enable_debug_motion)<<pull_over_sl.s()<<pull_over_sl.l();
      } 
    }
  }
  double dis_to_end = reference_lines_data_.front().mapinfo.dis2missionpoint;
  EgoInfo::instance()->UpdateDisToEnd(dis_to_end);
  AddMissionPointStopWall();
  GetNewPathType();
  return true;
}
void BehaviorParser::GetNewPathType() {
  if (!has_lateral_behavior()) {
    local_path_type_ = NewPathType::NONE;
    return;
  }
  if (target_reference_line_id_>=20) {
    local_path_type_ = NewPathType::LANE_CHANGE;
  } else if (lateral_action_.first == eActionEnum::ABANDON_LANE_CHANGE) {
    local_path_type_ = NewPathType::ABANDON_LANE_CHANGE;
  } else {
    bool has_comfirm_type = false;
    for (const auto& bound_tuple :decision_path_bound_) {
      if (get<1>(bound_tuple) != get<2>(bound_tuple)) {
        local_path_type_ = NewPathType::BORROW_LANE;
        has_comfirm_type = true;
        break;
      }
    }
    if (!has_comfirm_type) {
      local_path_type_ = NewPathType::IN_LANE;
    }
  }
}

void BehaviorParser::ResetDecisionHistory() {
  is_restart_ = false;
  restart_by_green_light_ = false;
  reference_lines_.clear();
  reference_lines_data_.clear();
  lateral_action_= std::make_pair(eActionEnum::DEFAULT_VALUE,eDirectionEnum::DEFAULT_VALUE);
  stop_walls_.clear();
  decision_path_bound_.clear();
  obstacle_decisions_.clear();
  behavior_parse_failed_msg_ = "";
  current_line_block_dis_ = 1000;
  block_obs_id_ = -1;
  has_decision_lateral_behavior_ = false;
  lanechange_prepare_direction_ = 0;
  lane_change_target_gap_ = std::make_pair(-1,-2);
  target_reference_line_.Reset();
  target_position_ = LanePosition::CURRENT;
  is_congestion_ = false;
  target_ref_line_change_ = false;
  lane_borrow_info_ = LaneBorrowInfo::NO_BORROW;
  plan_in_lane_ = true;
  abandon_lc_level_ = 0;
  pullover_line_id_ = -1;
}

bool BehaviorParser::has_lateral_behavior() const {
  if (lateral_action_.first == eActionEnum::DEFAULT_VALUE) {
    return false;
  } 
  return true;
}

bool BehaviorParser::has_decision_lateral_behavior() const { 
  return has_decision_lateral_behavior_;
}

bool BehaviorParser::CreateReferenceLineData(const StructReferenceLineInfo* ref_line_info) {
  bool has_latral_behavior = lateral_action_.first == eActionEnum::DEFAULT_VALUE? false : true; 
  current_line_block_dis_ = target_reference_line_.block_dis;
  block_obs_id_ = target_reference_line_.block_id;
  DataPool* DP = DataPool::Instance();
  auto action_reason = eActionEnum::DEFAULT_VALUE;
  if (has_latral_behavior || EgoInfo::instance()->vehicle_state().driving_mode != 0) { 
    NewPathManager::instance()->ResetIsNewStithingPath();
    reference_lines_data_.push_back(target_reference_line_);
    AINFO_IF(FLAGS_enable_debug_motion)<<"target_reference_line_.objects.size = "<<target_reference_line_.objects_.size();
    reference_lines_.insert(pair<int, LanePosition>(target_reference_line_id_, target_position_));
    if (FLAGS_use_multi_ref_line) {
      reference_lines_data_.push_back(ref_line_info->local_reference_line); 
      reference_lines_.insert(pair<int, LanePosition>(ref_line_info->local_reference_line.reference_lane_id, LanePosition::CURRENT));
    }
  } else {
    if (AvoidObstacleDecision(&ref_line_info->local_reference_line, action_reason)) {
      NewPathManager::instance()->ResetIsNewStithingPath();
      lateral_action_.first = action_reason;
      // lateral_action_.first = eActionEnum::OBSTACLE_AVOID;
      reference_lines_data_.push_back(target_reference_line_);
      AINFO_IF(FLAGS_enable_debug_motion)<<"Need Avoid Obstacle In lane ! ";
      reference_lines_.insert(pair<int, LanePosition>(target_reference_line_id_, target_position_));
      return true ;
    }
    if (NewPathManager::instance()->new_local_path().empty()) {
      reference_lines_data_.push_back(target_reference_line_);
      AINFO_IF(FLAGS_enable_debug_motion)<<"target_reference_line_.objects.size = "<<target_reference_line_.objects_.size();
      reference_lines_.insert(pair<int, LanePosition>(target_reference_line_id_, target_position_));
      return true;
    }
    NewPathManager::instance()->UpdateNewPath(target_reference_line_,target_reference_line_id_);//.mapinfo.path_points);
    if (!NewPathManager::instance()->stitch_state()) {
      reference_lines_data_.push_back(target_reference_line_);
      AINFO_IF(FLAGS_enable_debug_motion)<<"failed to stitch! ,need path plan, target_reference_line_.objects.size = "<<target_reference_line_.objects_.size();
      reference_lines_.insert(pair<int, LanePosition>(target_reference_line_id_, target_position_));
      return true;
    }
    AERROR_IF(FLAGS_enable_debug_motion && !target_reference_line_.mapinfo.path_points.empty())
      <<__FUNCTION__<<" reverse "<<target_reference_line_.mapinfo.path_points.front().reverse;
    ReferenceLineFrame stitching_local_ref_line;
    if (ref_line_info->local_reference_line.mapinfo.path_points.empty()) {
      stitching_local_ref_line = target_reference_line_;
    } else {
      stitching_local_ref_line = ref_line_info->local_reference_line;
    }
    AINFO_IF(FLAGS_enable_debug_motion)<<"local_reference_line.path_points.size = "
        <<ref_line_info->local_reference_line.mapinfo.path_points.size();
    AINFO_IF(FLAGS_enable_debug_motion)<<"local_reference_line.object size = "<<ref_line_info->local_reference_line.objects_.size();
    stitching_local_ref_line.mapinfo.path_points.clear();
    stitching_local_ref_line.mapinfo.path_points = NewPathManager::instance()->new_local_path();
    reference_lines_data_.push_back(stitching_local_ref_line);
    reference_lines_.insert(pair<int, LanePosition>(stitching_local_ref_line.reference_lane_id, LanePosition::CURRENT));
  }  
  return true;        
}

bool BehaviorParser::AvoidObstacleDecision(const ReferenceLineFrame* local_ref_line, eActionEnum &action_reason)  {
  if (!FLAGS_enable_realtime_avoid_obstacle_in_lane) {
    return false;
  }

  if ( target_reference_line_.Length() < 20.0 ||
    target_reference_line_.mapinfo.IsNeedExtend()) {
    return false;
  }

  // if (!last_local_path_in_current_ && target_reference_line_id_ < 20
  //     && EgoInfo::instance()->vehicle_state().linear_velocity > 1.0) {
  //   action_reason = eActionEnum::LANE_CHANGE;
  //   return true;
  // }

  if (EgoInfo::instance()->Last_path_accessible()) {
    AINFO_IF(FLAGS_enable_debug_motion)<<"!!!!!! No need realtime path plan !!!!!.";
    return false; // 上一帧的结果是安全可通行的，则不需要实时规划
  } else {
    if (ScenarioContext::instance()->scenario_info().current_latscenario 
                          == ScenarioContext::eLatScenarioEnum::PULL_OVER) {
      action_reason = eActionEnum::PULL_OVER;
      return true;
    }
    if (ScenarioContext::instance()->scenario_info().current_latscenario 
        == ScenarioContext::eLatScenarioEnum::LANE_CHANGE 
        && fabs(target_reference_line_.mapinfo.dis2line) > 0.2 && target_reference_line_id_ < 20) {
      action_reason = eActionEnum::LANE_CHANGE;
      return true;
    }
  }

  // if (target_reference_line_id_ >= 100 
  //       ||  ScenarioContext::instance()->scenario_info().current_latscenario 
  //                         == ScenarioContext::eLatScenarioEnum::PULL_OVER) {//靠边停车路径不实时规划
  //   AINFO_IF(FLAGS_enable_debug_motion)<<"not current lane or !! enable_avoid_obstacle_decision is false";
  //   action_reason = eActionEnum::PULL_OVER;
  //   return false;
  // }


  // if (!last_local_path_in_current_ && target_reference_line_id_ < 20&&EgoInfo::instance()->vehicle_state().linear_velocity>1) {//借道避障未完成，也不重新规划
  //   AINFO_IF(FLAGS_enable_debug_motion)<<"borrow lane not complete.";
  //   if(borrow_time_count_%10==9){
  //       return true;
  //   }
  //   borrow_time_count_++;
  //   return false;
  // }

  // if (ScenarioContext::instance()->scenario_info().current_latscenario 
  //       == ScenarioContext::eLatScenarioEnum::LANE_CHANGE 
  //       && fabs(target_reference_line_.mapinfo.dis2line) > 0.2 && target_reference_line_id_ < 20) {//换道进入目标车道了，但距离道路中心线还没有＜0.2m，也不重规划
  //   AINFO_IF(FLAGS_enable_debug_motion)<<"lane change on.";
  //   return false;
  // }
  // if (target_reference_line_id_ < 20 
  //    && target_reference_line_.mapinfo.distance_to_junctions.size()
  //    && target_reference_line_.mapinfo.distance_to_junctions.front().first < 0.0) {//进入路口，就不再实时规划路径
  //   AINFO_IF(FLAGS_enable_debug_motion)<<"near to junction s = "<<target_reference_line_.mapinfo.distance_to_junctions.front().first;
  //   return false;
  // }

  // if (target_reference_line_id_ >= 20) {//进入下述逻辑之前先判断目标参考线是不是当前车道，如果不是，则不能进入下述的车道内静止障碍物实时避障的逻辑
  //   AWARN_IF(FLAGS_enable_debug_motion)<<"not current reference_lines .";
  //   return false;
  // }
  // double stitch_trajectory_length = FLAGS_stitching_trajectory_time_buffer * 
  //          EgoInfo::instance()->vehicle_state_new().linear_velocity  
  //          + EgoInfo::instance()->vehicle_front_edge_to_center(); 
  // if (!target_reference_line_.limit_space.empty()) {
  //   auto fist_path_bound = target_reference_line_.limit_space.front();
  //   double nudge_l_buffer = 
  //     EgoInfo::instance()->vehicle_state_new().linear_velocity <= FLAGS_obstacle_extreme_nudge_speed + 0.1?
  //     FLAGS_static_decision_nudge_l_buffer_min : FLAGS_static_decision_nudge_l_buffer;
  //   AINFO_IF(FLAGS_enable_debug_motion)<<"fist_path_bound s = "
  //        <<std::get<0>(fist_path_bound)<<", bound = ("<<std::get<1>(fist_path_bound)
  //        <<", "<<std::get<2>(fist_path_bound)<<"). nudge_l_buffer = "<<nudge_l_buffer
  //        <<",  v = "<<EgoInfo::instance()->vehicle_state_new().linear_velocity;
  //   if (std::get<2>(fist_path_bound) - std::get<1>(fist_path_bound) 
  //      > EgoInfo::instance()->vehicle_param().car_width) {
  //     return true;
  //   } else {
  //     AINFO_IF(FLAGS_enable_debug_motion)<<"space is not enough!!";
  //     return false;
  //   }
  // } else {
  //   AINFO_IF(FLAGS_enable_debug_motion)<<"no obstacle!!";
  //   return true;
  // }

  action_reason = eActionEnum::OBSTACLE_AVOID;
  return true;
}

bool BehaviorParser::FindTargetRefLine(const StructReferenceLineInfo* ref_line_info) {
  auto it_current = ref_line_info->current_reference_line.begin();
  while (it_current != ref_line_info->current_reference_line.end()) {
    if (target_reference_line_id_ == it_current->reference_lane_id) {
      target_reference_line_ = *it_current;
      target_position_ = LanePosition::CURRENT;
      SetDirection();
      return true;
    }
    it_current++;
  }

  auto it_left = ref_line_info->left_reference_line.begin();
  while (it_left != ref_line_info->left_reference_line.end()) {
    if (target_reference_line_id_ == it_left->reference_lane_id) {
      target_reference_line_ = *it_left;
      target_position_ = LanePosition::LEFT;
      return true;
    }
    it_left++;
  }

  auto it_right = ref_line_info->right_reference_line.begin();
  while (it_right != ref_line_info->right_reference_line.end()) {
    if (target_reference_line_id_ == it_right->reference_lane_id) {
      target_reference_line_ = *it_right;
      target_position_ = LanePosition::RIGHT;
      return true;
    }
    it_right++;
  }

  return false;
}

void BehaviorParser::SetDirection() {
  if (target_reference_line_.mapinfo.lane_types.empty()) {
    return;
  }
  bool reverse_flag = target_reference_line_.mapinfo.direction == LaneDirectionType::BACKWARD;
  if (reverse_flag) {
    for (auto& point : target_reference_line_.mapinfo.path_points) {
      point.reverse = reverse_flag;
    }
  }
  if ( target_reference_line_.mapinfo.lane_types.front().second == eLaneType::PARKING_LANE
       || target_reference_line_.mapinfo.dis_to_end < 30.0
          &&(target_reference_line_.mapinfo.dis2missionpoint - target_reference_line_.mapinfo.dis_to_end) > 2.5) {
    target_reference_line_.mapinfo.SetNeedExtend(true);
    const double extend_length = !target_reference_line_.mapinfo.path_points.empty()
      && target_reference_line_.mapinfo.path_points.front().reverse
      ? EgoInfo::instance()->vehicle_back_edge_to_center() 
      : EgoInfo::instance()->vehicle_front_edge_to_center();
    ExtendPath(target_reference_line_.mapinfo.path_points, reverse_flag, extend_length);
  }
  AERROR_IF(FLAGS_enable_debug_motion)<<"reverse "<<reverse_flag
    <<" need_extend_parking_path "<<target_reference_line_.mapinfo.IsNeedExtend();
}

void BehaviorParser::ExtendPath( geometry::SiteVec& ref_path_points,
    const bool reverse_flag, const double& extend_length) {
  if (ref_path_points.empty()) return;
  geometry::Site ego, origin_back_pt(ref_path_points.back()), 
    pre_point(ref_path_points.back()), temp(ref_path_points.back());
  ego.xg = EgoInfo::instance()->vehicle_state().x;
  ego.yg = EgoInfo::instance()->vehicle_state().y;
  ego.globalangle = EgoInfo::instance()->vehicle_state().yaw;

  int count_extend = 1;
  double accumulate_s = 0, kStep = 0.05;
  const int kSign = reverse_flag ? -1 : 1;
  double cosheading_g = kSign * std::cos(origin_back_pt.globalangle * M_PI / 180.0);
  double sinheading_g = kSign * std::sin(origin_back_pt.globalangle * M_PI / 180.0);
  CoordTransform *coor_transform = CoordTransform::Instance();
  while( accumulate_s < extend_length) {
    temp.xg = origin_back_pt.xg + count_extend * kStep * cosheading_g;
    temp.yg = origin_back_pt.yg + count_extend * kStep * sinheading_g;
    temp.length += kStep;
    temp.globalangle = origin_back_pt.globalangle;
    temp.curvature = origin_back_pt.curvature;
    coor_transform->GCCS2VCS(ego, temp, temp);
    accumulate_s += kStep;
    count_extend++;
    pre_point = temp;
    ref_path_points.push_back(temp);
  }
  AERROR_IF(FLAGS_enable_debug_motion)
    <<"count_extend "<<count_extend
    <<" accumulate_s "<<accumulate_s
    <<" extend_length "<<extend_length
    <<" back_length "<<ref_path_points.back().length;
}

bool BehaviorParser::GetNearestPoint(const geometry::SiteVec& points, const common::math::Vec2d& point, 
                     double* accumulate_s,double* lateral) {
  if (accumulate_s == nullptr || lateral == nullptr || points.empty()) {
    return false;
  }
  std::vector<common::math::LineSegment2d> segments;
  for (int i = 0; i + 1 < points.size(); ++i) {
    segments.emplace_back(common::math::Vec2d(points[i].xg,points[i].yg), 
                          common::math::Vec2d(points[i + 1].xg,points[i + 1].yg));
  }
  if (segments.empty()) {
    return false;
  }

  double num_segments = (int)points.size() - 1;
  double min_distance = std::numeric_limits<double>::infinity();
  int min_index = 0;
  for (int i = 0; i < num_segments; ++i) {
    const double distance = segments[i].DistanceSquareTo(point);
    if (distance < min_distance) {
      min_index = i;
      min_distance = distance;
    }
  }
  min_distance = std::sqrt(min_distance);
  const auto& nearest_seg = segments[min_index];
  const auto prod = nearest_seg.ProductOntoUnit(point);
  const auto proj = nearest_seg.ProjectOntoUnit(point);
  if (min_index == 0) {
    *accumulate_s = std::min(proj, nearest_seg.length());
    if (proj < 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * min_distance;
    }
  } else if (min_index == num_segments - 1) {
    *accumulate_s = points[min_index].length + std::max(0.0, proj);
    if (proj > 0) {
      *lateral = prod;
    } else {
      *lateral = (prod > 0.0 ? 1 : -1) * min_distance;
    }
  } else {
    *accumulate_s = points[min_index].length +
                    std::max(0.0, std::min(proj, nearest_seg.length()));
    *lateral = (prod > 0.0 ? 1 : -1) * min_distance;
  }
  return true;
}

void BehaviorParser::AddMissionPointStopWall() {
  if (reference_lines_data_.empty()) {
    return;
  }
  if (reference_lines_data_.front().mapinfo.path_points.empty()
    || !reference_lines_data_.front().mapinfo.lane_types.empty() &&
      reference_lines_data_.front().mapinfo.lane_types.front().second == eLaneType::PARKING_LANE) {
    return;
  }
  if (lateral_action_.first == eActionEnum::PULL_OVER) {
    return;
  }
  bool is_pull_over_path = ScenarioContext::instance()->scenario_info().current_latscenario 
                          == ScenarioContext::eLatScenarioEnum::PULL_OVER ? true : false;
  if (is_pull_over_path) {
    return;
  }                        
  double dis_to_end = EgoInfo::instance()->dis_to_end();
  if (dis_to_end < reference_lines_data_.front().Length()) {
    AINFO_IF(FLAGS_enable_debug_motion)<<" mission stop . << "<<dis_to_end;
    int match_index = -1;
    geometry::Site mission_point;
    if (!reference_lines_data_.front().GetNearestPoint(dis_to_end,mission_point,match_index)) {
      return;
    }
    double length = 0.01;
    double width = 1.0;
    VirtualObstacle mission_obstacle;
    mission_obstacle.id = "M_STOP";//FLAGS_destination_obstacle_id;//与终点停车的虚拟障碍物id一致，这样在有路的终点虚拟障碍物时，该障碍物不再重新建立，避免出错。
    mission_obstacle.obstacle_box = common::math::Box2d(common::math::Vec2d(mission_point.xg,mission_point.yg),
                                    mission_point.globalangle * M_PI / 180.0,length,width);
    AddVirtualObstacle(mission_obstacle);
  }
}

bool BehaviorParser::AddVirtualObstacle(const std::string stop_wall_id,const bool enable_relative_distance,
                const common::math::Box2d& stop_wall_box,const double stop_distance,
                const StopReasonCode reason){
  VirtualObstacle stop_wall ;
  stop_wall.id = stop_wall_id;
  stop_wall.enable_relative_distance = enable_relative_distance;
  stop_wall.obstacle_box = stop_wall_box;
  stop_wall.stop_distance = stop_distance;
  stop_wall.stop_reason = reason;
  stop_walls_.push_back(stop_wall);
  return true;
}

bool BehaviorParser::AddVirtualObstacle(const VirtualObstacle stop_wall){
  stop_walls_.push_back(stop_wall);
  return true;
}

bool BehaviorParser::UpdateVirtualObstacleList(const vector<VirtualObstacle> stop_wall_infos){
  stop_walls_ = stop_wall_infos;
  return true;
}

bool BehaviorParser::DeleteVirtualObstacle(const StopReasonCode stop_reason){
  if (stop_walls_.empty()) return true;
  vector<VirtualObstacle> stop_walls_update;
  size_t stop_walls_size = stop_walls_.size();
  for (size_t i = 0;i < stop_walls_size;++i) {
    if (stop_walls_.at(i).stop_reason != stop_reason){
      VirtualObstacle virtual_stop_wall = stop_walls_.at(i);
      stop_walls_update.push_back(virtual_stop_wall);
    } 
  }
  UpdateVirtualObstacleList(stop_walls_update);
  return true;
} 

bool BehaviorParser::DeleteVirtualObstacle(const std::string stop_wall_id){
  if (stop_walls_.empty()) return true;
  vector<VirtualObstacle> stop_walls_update;
  size_t stop_walls_size = stop_walls_.size();
  for (size_t i = 0;i < stop_walls_size;++i){
    if (stop_walls_.at(i).id != stop_wall_id){
      VirtualObstacle virtual_stop_wall = stop_walls_.at(i);
      stop_walls_update.push_back(virtual_stop_wall);
    } 
  }
  UpdateVirtualObstacleList(stop_walls_update);
  return true;
} 
             
vector<VirtualObstacle> BehaviorParser::stop_wall_infos() const{
  return stop_walls_;
}

bool BehaviorParser::GetDecisionSpeedlimit(int giveway_id, const double speed_limit, const double init_speed) {
  AINFO_IF(FLAGS_enable_debug_motion)<<"giveway_id = "<<giveway_id<<", speed_limit = "<<speed_limit
    <<", decision_speed_limit_ "<<decision_speed_limit_;
  const double deceleration = lateral_action_.first == eActionEnum::PULL_OVER?
                              -0.5 : -0.8; 
  const double invalid_speed_limit = 60/3.6;
  meeting_id_ = giveway_id;
  if (speed_limit >= invalid_speed_limit) {
    decision_speed_limits_.clear();
    decision_speed_limit_ = speed_limit;
    AINFO_IF(FLAGS_enable_debug_motion)<<"has no speedlimits.";
    return true;
  }

  if (reference_lines_data_.empty() || 
      reference_lines_data_.front().mapinfo.path_points.empty()) {
    AWARN_IF(FLAGS_enable_debug_motion)<<"target line is not exist!!!!";
    decision_speed_limits_.clear();
    decision_speed_limit_ = 100.0;
    return true;
  }

  double distance = 0.0;
  
  if (fabs(speed_limit - decision_speed_limit_) > 0) {
    double v_car = EgoInfo::instance()->vehicle_state().linear_velocity;
    distance = std::fmax((speed_limit * speed_limit - v_car * v_car )/2/deceleration, 
                         (speed_limit * speed_limit - init_speed * init_speed )/2/deceleration) 
               + EgoInfo::instance()->vehicle_front_edge_to_center();
    
    distance = std::fmin(distance, 
        reference_lines_data_.front().mapinfo.path_points.back().length - 
        EgoInfo::instance()->vehicle_front_edge_to_center());
    distance = std::fmax(distance,0.0);

    AINFO_IF(FLAGS_enable_debug_motion)<<"1) speed limit distance = "<<distance<<", speed_limit = "<<speed_limit;
    decision_speed_limits_.clear();
    decision_speed_limits_.push_back(make_tuple(distance,150.0,speed_limit));
    int match_index = -1;
    geometry::Site match_point;
    if (!reference_lines_data_.front().GetNearestPoint(distance,match_point,match_index)) {
      AWARN_IF(FLAGS_enable_debug_motion)<<"cannot find nearest speedlimit point!!!";
      return false;
    }
    speed_limit_point_ = match_point;
    decision_speed_limit_ = speed_limit;
    for (auto& spd_limit:decision_speed_limits_) {
      AINFO_IF(FLAGS_enable_debug_motion)<<" s =( "<<get<0>(spd_limit)<<", "<<get<1>(spd_limit)
            <<") ,speed limit = "<<get<2>(spd_limit);
    }
    AINFO_IF(FLAGS_enable_debug_motion)<<fixed << setprecision(4)<<"1) limit point xgyg = ("
             <<speed_limit_point_.xg<<", "<<speed_limit_point_.yg;
    if (speed_limit <= 0) {
      VirtualObstacle stop_wall;
      stop_wall.obstacle_box = common::math::Box2d(common::math::Vec2d(speed_limit_point_.xg,speed_limit_point_.yg), 
                               speed_limit_point_.globalangle * M_PI/180.0, 
                               FLAGS_remote_stop_obstacle_length,
                               FLAGS_remote_stop_obstacle_width);
      stop_wall.id = "zero_speed_point"; 
      AddVirtualObstacle(stop_wall); 
    }         
    return true;
  }
  int match_index = -1;
  geometry::Site match_point;
  double min_s = 0.0;
  AINFO_IF(FLAGS_enable_debug_motion)<<fixed << setprecision(4)<<"2) limit point xgyg = ("
             <<speed_limit_point_.xg<<", "<<speed_limit_point_.yg;
  if (!reference_lines_data_.front().GetGlobalNearestPoint(speed_limit_point_,
                  match_point, distance, min_s,match_index)) {
    AWARN_IF(FLAGS_enable_debug_motion)<<"cannot find nearest speedlimit point!!!";
    return false;
  }
  AINFO_IF(FLAGS_enable_debug_motion)<<fixed << setprecision(4)<<"2) match point xgyg = ("
             <<match_point.xg<<", "<<match_point.yg;
  AINFO_IF(FLAGS_enable_debug_motion)<<"2) speed limit distance = "<<distance <<", speed_limit = "<<speed_limit;
  const double buffer = 0.5;
  distance = distance - buffer;
  decision_speed_limits_.clear();
  decision_speed_limits_.push_back(make_tuple(distance,150.0,speed_limit));
  
  for (auto& spd_limit:decision_speed_limits_) {
    AINFO_IF(FLAGS_enable_debug_motion)<<" s =( "<<get<0>(spd_limit)<<", "<<get<1>(spd_limit)
          <<") ,speed limit = "<<get<2>(spd_limit);
  }
  if (speed_limit <= 0) {
    VirtualObstacle stop_wall;
    stop_wall.obstacle_box = common::math::Box2d(common::math::Vec2d(speed_limit_point_.xg,speed_limit_point_.yg), 
                             speed_limit_point_.globalangle * M_PI/180.0, 
                             FLAGS_remote_stop_obstacle_length,
                             FLAGS_remote_stop_obstacle_width);
    stop_wall.id = "zero_speed_point"; 
    AddVirtualObstacle(stop_wall); 
  }
  return true;
}

void BehaviorParser::GetRemoteDriveInfo() {
  if (reference_lines_data_.empty() || 
      reference_lines_data_.front().mapinfo.path_points.empty()) {
    AWARN_IF(FLAGS_enable_debug_motion)<<"target refline is not exist ! "
                  <<"cancel all remote brake command !!";
    cancel_remote_stop();
    return ;
  }
  auto DP = DataPool::Instance();
  int remote_brake = DP->GetMainDataRef().task_content.brake_command;
  if (DP->GetMainDataRef().task_content.command == eCommand::COMMAND_STOP || 
      DP->GetMainDataRef().task_content.command == eCommand::COMMAND_SUSPEND) {
    remote_brake = 1;
  }
  if (remote_brake) {
    std::string stop_type = remote_brake == 1? "SlightlyBrake":"HardBrake";
    if (remote_stop_boxes_.find(remote_brake) != remote_stop_boxes_.end()) {
      AWARN_IF(FLAGS_enable_debug_motion) << "Warnning!! there had " 
           <<stop_type<<" stop obstacle before !"<< endl;
      return;
    } else {
      double v_car = EgoInfo::instance()->vehicle_state().linear_velocity;
      double deceleration = -0.4;
      if (remote_brake > 1) {
        deceleration = -3.0;
      }
      double distance =  (v_car * v_car) / 2.0 /fabs(deceleration) 
                          + EgoInfo::instance()->vehicle_front_edge_to_center();
      geometry::Site match_point;
      int match_index = -1;
      if (!reference_lines_data_.front().GetNearestPoint(distance,match_point,match_index)) {
        AWARN_IF(FLAGS_enable_debug_motion)<<"cannot find nearest point ! "
                       <<"can not create "<<stop_type<<" stop objects !!";
        return;
      }
      common::math::Box2d stop(common::math::Vec2d(match_point.xg, match_point.yg), 
              match_point.globalangle * M_PI/180.0, FLAGS_remote_stop_obstacle_length,FLAGS_remote_stop_obstacle_width);
      add_remote_stop(remote_brake,stop);
    }
  } else {
    if (!remote_stop_boxes_.empty()) {
      AWARN_IF(FLAGS_enable_debug_motion)<<"There has no remote_brake command, "
                  <<"cancel all remote brake command.";
    }
    cancel_remote_stop();
  }
}

void BehaviorParser::GetRemoteDriveInfo(const common::TrajectoryPoint& init_point) {
  if (reference_lines_data_.empty() || 
      reference_lines_data_.front().mapinfo.path_points.empty()) {
    AWARN_IF(FLAGS_enable_debug_motion)<<"target refline is not exist ! "
                  <<"cancel all remote brake command !!";
    cancel_remote_stop();
    return ;
  }

  double init_s = 0.0;
  double init_l = 0.0;
  Site init_path_point;
  init_path_point.xg = init_point.path_point().x();
  init_path_point.yg = init_point.path_point().y();
  if (!XYToSL(reference_lines_data_.front().mapinfo, init_path_point, init_s, init_l)) {
    AWARN_IF(FLAGS_enable_debug_motion)<<"cannot get init point sl!";
    return ;
  }
  auto DP = DataPool::Instance();
  int remote_brake = FLAGS_enable_remote_drive_brake? 
            DP->GetMainDataRef().task_content.brake_command : 0;
  if (DP->GetMainDataRef().task_content.command == eCommand::COMMAND_STOP || 
      DP->GetMainDataRef().task_content.command == eCommand::COMMAND_SUSPEND) {
    remote_brake = 1;
  }
  if (remote_brake) {
    std::string stop_type = remote_brake == 1? "SlightlyBrake":"HardBrake";
    if (remote_stop_boxes_.find(remote_brake) != remote_stop_boxes_.end()) {
      AWARN_IF(FLAGS_enable_debug_motion) << "Warnning!! there had " 
           <<stop_type<<" stop obstacle before !"<< endl;
      return;
    } else {
      double v_car = init_point.v();//EgoInfo::instance()->vehicle_state().linear_velocity;
      double deceleration = -0.4;
      if (remote_brake > 1) {
        deceleration = -3.0;
      }
      double distance =  (v_car * v_car) / 2.0 /fabs(deceleration) 
                          + EgoInfo::instance()->vehicle_front_edge_to_center() + std::fmax(0.0,init_s);
      geometry::Site match_point;
      int match_index = -1;
      if (!reference_lines_data_.front().GetNearestPoint(distance,match_point,match_index)) {
        AWARN_IF(FLAGS_enable_debug_motion)<<"cannot find nearest point ! "
                       <<"can not create "<<stop_type<<" stop objects !!";
        return;
      }
      common::math::Box2d stop(common::math::Vec2d(match_point.xg, match_point.yg), 
              match_point.globalangle * M_PI/180.0, FLAGS_remote_stop_obstacle_length,FLAGS_remote_stop_obstacle_width);
      add_remote_stop(remote_brake,stop);
    }
  } else {
    if (!remote_stop_boxes_.empty()) {
      AWARN_IF(FLAGS_enable_debug_motion)<<"There has no remote_brake command, "
                  <<"cancel all remote brake command.";
    }
    
    cancel_remote_stop();
  }
}

bool BehaviorParser::add_remote_stop(int label,const common::math::Box2d& stop) {
  auto insert_sucess = remote_stop_boxes_.insert(make_pair(label, stop));
  if(!insert_sucess.second) {
    std::string stop_type = label == 1? "SlightlyBrake":"HardBrake";
    AWARN_IF(FLAGS_enable_debug_motion) << "Warnning!! insert " <<stop_type<<" stop obstacle failed!!"<< endl;
    return false;
  }
  return true;
}

bool BehaviorParser::is_fisrt_time_slow_down() {
  AINFO_IF(FLAGS_enable_debug_motion)<<"id = "<<block_obs_id_<<", dis = "<<current_line_block_dis_;
  AINFO_IF(FLAGS_enable_debug_motion)<<"history id = "<<history_block_obs_id_<<", history dis = "<<history_line_block_dis_;
  if (current_line_block_dis_ > 200 
    || block_obs_id_ == history_block_obs_id_
    || has_lateral_behavior()) {
    history_line_block_dis_ = current_line_block_dis_; 
    history_block_obs_id_ = block_obs_id_; 
    return false;
  } 

  double car_v = EgoInfo::instance()->vehicle_state().linear_velocity;
  //assume deceleration -1m/s^2
  double stop_distance = car_v * car_v / 2 + 
                       FLAGS_min_stop_distance_obstacle +
                       EgoInfo::instance()->vehicle_front_edge_to_center(); 
  if (current_line_block_dis_ <= stop_distance) {
    AINFO_IF(FLAGS_enable_debug_motion)<<"first time to add shift Trajectory";
    history_line_block_dis_ = current_line_block_dis_; 
    history_block_obs_id_ = block_obs_id_; 
    return true;
  }
  history_line_block_dis_ = current_line_block_dis_; 
  history_block_obs_id_ = block_obs_id_; 
  return false;                  
}

bool BehaviorParser::has_yield_or_overtake_behavior() const {
  if (obstacle_decisions_.empty()) {
    return false;
  }

  for (const auto& decision : obstacle_decisions_) {
    if (decision.second == eObjectDecisionEnum::GIVEWAY ||
        decision.second == eObjectDecisionEnum::TAKEWAY) {
      AINFO_IF(FLAGS_enable_debug_motion)<<"has yiled or overtake behavior";
      return true;
    }
  }
  return false;
}

bool BehaviorParser::has_follow_behavior() const {
  if (obstacle_decisions_.empty()) {
    return false;
  }

  for (const auto& decision : obstacle_decisions_) {
    if (decision.second == eObjectDecisionEnum::FOLLOW) {
      AINFO_IF(FLAGS_enable_debug_motion)<<"has follow behavior";
      return true;
    }
  }
  return false;
}

std::pair<double,double> BehaviorParser::closed_dynamic_obstacle() const {
  double min_s = std::numeric_limits<double>::max();
  double min_t = std::numeric_limits<double>::max();
  std::pair<double,double> return_value = std::make_pair(min_t,min_s);

  if (reference_lines_data_.empty() || reference_lines_data_.front().objects_.empty()) {
    return return_value;//(min_t,min_s);
  }

  for (auto const& it : reference_lines_data_.front().objects_) {
    auto & obs = it.second;
    if (!obs.st_boundary.s_t.empty() && 
         obs.st_boundary.s_t.front().first.x() < min_s) {
      min_s = obs.st_boundary.s_t.front().first.x(); 
      return_value = std::make_pair(obs.st_boundary.s_t.front().first.y(),obs.st_boundary.s_t.front().first.x());
    }
  }
  AINFO_IF(FLAGS_enable_debug_motion)<<"closed_dynamic_obstacle ts = ("<<return_value.first<<", "<<return_value.second<<") . ";
  return return_value;
}

}  // namespace planning
}  // namespace acu
