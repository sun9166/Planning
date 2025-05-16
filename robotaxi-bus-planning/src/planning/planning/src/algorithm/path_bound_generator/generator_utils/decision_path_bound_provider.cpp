
/**
 * @file decision_path_bound_provider.cpp
 **/

#include <algorithm>
#include "src/execution/motionplan/common/planning_gflags.h"
#include "decision_path_bound_provider.h"
#include "common/base/log/include/log.h"

namespace acu {
namespace planning {

using common::SpeedPoint;
using common::SLPoint;
using common::TrajectoryPoint;
using common::math::Vec2d;

bool DecisionPathBoundProvider::GetDecisionPathBound(double &left_bound, double &right_bound,const double s,
                            const int left_lane_id, const int right_lane_id, const ReferenceLineInfo* target_reference_line_info) {
  double left_bound_left_lane = 0.0;
  double right_bound_left_lane = 0.0;
  double left_bound_right_lane = 0.0;
  double right_bound_right_lane = 0.0;
  LanePosition left_bound_position;
  auto left_reference_line_data = FindTargetRefLine(left_lane_id,left_bound_position);
  auto right_reference_line_data = left_reference_line_data;
  bool is_left_same_with_right = true;
  LanePosition right_bound_position = left_bound_position;
  if (left_lane_id != right_lane_id) {
     right_reference_line_data = FindTargetRefLine(right_lane_id,right_bound_position);
     is_left_same_with_right = false;
  }

  if (right_reference_line_data == nullptr || 
      left_reference_line_data == nullptr) {
    AWARN_IF(FLAGS_enable_debug_motion)<<"Cannot find left bound lane: "<<left_lane_id
                                       <<" || right bound lane :"<<right_lane_id<<". ";
    return false;
  }

  if (target_reference_line_info->Position() == LanePosition::LEFT) { // 1.目标参考线在左边
    if (left_bound_position == LanePosition::CURRENT) { //1.1目标参考线在左边 ,左边界为当前参考线的左边界 
      double left_boundry = 0.0;
      double right_boundry = 0.0;
      if (target_reference_line_info->reference_line().GetLaneWidth(s, &left_boundry,
                            &right_boundry)) {
        left_bound = -(right_boundry + FLAGS_boundary_width);
      } else {
        return false;
      }

      if (right_bound_position == LanePosition::LEFT) {//1.1.3目标参考线在左边 ,左边界为当前参考线的左边界 , 右边界为左边参考线的右边界
        AWARN_IF(FLAGS_enable_debug_motion)<<"right decision bound error !!"<< "( "<<left_lane_id
                                       <<" ,"<<right_lane_id<<") .";
        return false;
      } else {
        Site nearest_point;
        int index;
        if (right_reference_line_data->GetNearestPoint(s, nearest_point, index)) {
          common::SLPoint sl_point;
          target_reference_line_info->reference_line().XYToSL(
                   common::math::Vec2d(nearest_point.xg, nearest_point.yg), &sl_point);
          double left_width = 0.0;
          double right_width = 0.0;
          right_reference_line_data->GetWidthToLaneBoundary(left_width,right_width, s);
          right_bound = sl_point.l() - right_width;
          return true;
        } else {
          return false;
        }
      }
    } else if (left_bound_position == LanePosition::RIGHT) { //1.2目标参考线在左边 ,左边界为右侧参考线的左边界
      AWARN_IF(FLAGS_enable_debug_motion)<<"left decision bound error !!"<< "( "<<left_lane_id
                                       <<" ,"<<right_lane_id<<") .";
      return false;
    } else if (left_bound_position == LanePosition::LEFT ) { //1.3目标参考线在左边 ,左边界为左侧参考线的左边界
      double left_boundry = 0.0;
      double right_boundry = 0.0;
      if (target_reference_line_info->reference_line().GetLaneWidth(s, &left_boundry,
                            &right_boundry)) {
        left_bound = left_boundry;
      } else {
        return false;
      }

      if (is_left_same_with_right) {//1.3.1 目标参考线在左边 ,左边界为左侧参考线的左边界 , 右边界为左侧参考线的右边界
        right_bound = - right_boundry;
      } else {
        if (right_reference_line_data->GetWidthToLaneBoundary(left_bound_right_lane,right_bound_right_lane, s)) {
          if (right_bound_position == LanePosition::CURRENT) { //1.3.2 目标参考线在左边 ,左边界为左侧参考线的左边界 , 右边界为当前参考线的右边界
            Site nearest_point;
            int index;
            if (right_reference_line_data->GetNearestPoint(s, nearest_point, index)) {
              common::SLPoint sl_point;
              target_reference_line_info->reference_line().XYToSL(
                       common::math::Vec2d(nearest_point.xg, nearest_point.yg), &sl_point);
              right_bound = sl_point.l() - right_bound_right_lane;
              return true;
            } else {
              return false;
            }
          } else if (right_bound_position == LanePosition::LEFT) {//1.3.3目标参考线在左边 ,左边界为左侧参考线的左边界 , 右边界为左边参考线的右边界
            AINFO_IF(FLAGS_enable_debug_motion)<<"left and right bound lane is rurrent ,but id is not the same "<< "( "<<left_lane_id
                                       <<" ,"<<right_lane_id<<") .";
           //TODO
            right_bound = - right_boundry;
          } else if (right_bound_position == LanePosition::RIGHT) {//1.3.4目标参考线在左边 ,左边界为左侧参考线的左边界 , 右边界为右侧参考线的右边界
            AINFO_IF(FLAGS_enable_debug_motion)<<"maybe wrong decision bound ,target is left ,left is left, right is right "<< "( "<<left_lane_id
                                       <<" ,"<<right_lane_id<<") .";
            return false;
          }
        } else {
          return false;
        }
      }
    }
  } else if (target_reference_line_info->Position() == LanePosition::RIGHT) { //2.目标参考线在右边
    if (left_bound_position == LanePosition::CURRENT) {//2.1目标参考线在右边 ,左边界为当前参考线的左边界
      if (!left_reference_line_data->GetWidthToLaneBoundary(left_bound_left_lane,right_bound_left_lane, s)) {
        return false;
      }
      double left_boundry = 0.0;
      double right_boundry = 0.0;
      Site nearest_point;
      int index;
      if (left_reference_line_data->GetNearestPoint(s, nearest_point, index)) {
        common::SLPoint sl_point;
        target_reference_line_info->reference_line().XYToSL(
                 common::math::Vec2d(nearest_point.xg, nearest_point.yg), &sl_point);
        left_bound = sl_point.l() + left_bound_left_lane;
      } else {
        return false;
      }
      if (is_left_same_with_right) {//2.1.1目标参考线在右边 ,左边界为当前参考线的左边界, 右边界为当前参考线的右边界
        if (target_reference_line_info->reference_line().GetLaneWidth(s, &left_boundry,
                          &right_boundry)) {
          right_bound = left_boundry + FLAGS_boundary_width;
        } else {
          return false;
        }
        // AWARN_IF(FLAGS_enable_debug_motion && s < 6)<<"right_bound = "<<right_bound;
      } else { 
        if (right_bound_position == LanePosition::CURRENT) {//2.1.2目标参考线在右边 ,左边界为当前参考线的左边界 , 右边界为当前参考线的右边界
          AINFO_IF(FLAGS_enable_debug_motion)<<"left and right bound lane is rurrent ,but id is not the same : "<< "( "<<left_lane_id
                                       <<" ,"<<right_lane_id<<") .";
          if (target_reference_line_info->reference_line().GetLaneWidth(s, &left_boundry,
                          &right_boundry)) {
            right_bound = left_boundry + FLAGS_boundary_width;
          } else {
            return false;
          }
           //TODO
        } else if (right_bound_position == LanePosition::LEFT) {//2.1.3目标参考线在右边 ,左边界为当前参考线的左边界 , 右边界为左侧参考线的右边界
          AWARN_IF(FLAGS_enable_debug_motion)<<"right decision bound error !!"<< "( "<<left_lane_id
                                       <<" ,"<<right_lane_id<<") .";
          return false;
        } else if (right_bound_position == LanePosition::RIGHT) {//2.1.4目标参考线在右边 ,左边界为当前参考线的左边界 , 右边界为右侧参考线的右边界
          //是否需要判断与目标参考线id一致?
          if(target_reference_line_info->reference_line().GetLaneWidth(s, &left_boundry,
                            &right_boundry)) {
            right_bound = -right_boundry;
          } else {
            return false;
          }
        }
      }
    } else if (left_bound_position == LanePosition::RIGHT) {//2.2目标参考线在右边 ,左边界为右侧参考线的左边界
      double left_boundry = 0.0;
      double right_boundry = 0.0;
      if (target_reference_line_info->reference_line().GetLaneWidth(s, &left_boundry,
                            &right_boundry)) {
        left_bound = left_boundry;
      } else {
        return false;
      }
      if (is_left_same_with_right) { //2.2.1目标参考线为右侧车道, 左边界为右侧参考线的左边界,右边界为右侧参考线的右边界
        right_bound = -right_boundry;
      } else { 
        if (right_bound_position == LanePosition::CURRENT) {//2.2.2目标参考线为右侧车道, 左边界为右侧参考线的左边界,右边界为当前车道参考线的右边界
          AINFO_IF(FLAGS_enable_debug_motion)<<"left and right bound lane is the same "<< "( "<<left_lane_id
                                       <<" ,"<<right_lane_id<<") .";
          return false;   
        } else if (right_bound_position == LanePosition::LEFT) {//2.2.3目标参考线为右侧车道, 左边界为右侧参考线的左边界,右边界为左侧参考线的右边界
          AWARN_IF(FLAGS_enable_debug_motion)<<"right decision bound error !!"<< "( "<<left_lane_id
                                       <<" ,"<<right_lane_id<<") .";
          return false;
        } else if (right_bound_position == LanePosition::RIGHT) {//2.2.4目标参考线为右侧车道, 左边界为右侧参考线的左边界,右边界为右边参考线的右边界
          right_bound = -right_boundry;//TODO
          AINFO_IF(FLAGS_enable_debug_motion)<<"left and right bound lane is right ,but id is not the same : "<< "( "<<left_lane_id
                                       <<" ,"<<right_lane_id<<") .";
        }
      }
    } else if (left_bound_position == LanePosition::LEFT ) {//2.3目标参考线在右边 ,左边界为左侧参考线的左边界
      AINFO_IF(FLAGS_enable_debug_motion)<<"maybe wrong decision bound ,target is right ,left is left : " << "( "<<left_lane_id
                                       <<" ,"<<right_lane_id<<") .";
      return false;
    }
  } else if (target_reference_line_info->Position() == LanePosition::CURRENT) { //3.目标参考线为当前车道
    if (left_bound_position == LanePosition::CURRENT) { //3.1目标参考线为当前车道, 左边界为当前参考线的左边界
      double left_boundry = 0.0;
      double right_boundry = 0.0;
      if (target_reference_line_info->reference_line().GetLaneWidth(s, &left_boundry,
                            &right_boundry)) {
        left_bound = left_boundry;
      } else {
        return false;
      }
      if (is_left_same_with_right) { //3.1.1目标参考线为当前车道, 左边界为当前参考线的左边界,右边界为当前参考线的右边界
        right_bound = -right_boundry;
      } else { 
        if (right_bound_position == LanePosition::CURRENT) {//3.1.2目标参考线为当前车道, 左边界为当前参考线的左边界,右边界为当前参考线的右边界
          AINFO_IF(FLAGS_enable_debug_motion)<<"left and right bound lane is rurrent ,but id is not the same : "<< "( "<<left_lane_id
                                       <<" ,"<<right_lane_id<<") .";
          //TODO
          right_bound = -right_boundry;  
        } else if (right_bound_position == LanePosition::LEFT) {//3.1.3目标参考线为当前车道, 左边界为当前参考线的左边界,右边界为左边参考线的右边界
          AWARN_IF(FLAGS_enable_debug_motion)<<"right decision bound error !! "<< "( "<<left_lane_id
                                       <<" ,"<<right_lane_id<<") .";
          return false;
        } else if (right_bound_position == LanePosition::RIGHT) {//3.1.4目标参考线为当前车道, 左边界为当前参考线的左边界,右边界为右边参考线的右边界
          if (right_reference_line_data->GetWidthToLaneBoundary(left_bound_right_lane,right_bound_right_lane, s)) {
            Site nearest_point;
            int index;
            if (right_reference_line_data->GetNearestPoint(s, nearest_point, index)) {
              common::SLPoint sl_point;
              target_reference_line_info->reference_line().XYToSL(
                       common::math::Vec2d(nearest_point.xg, nearest_point.yg), &sl_point);
              right_bound = sl_point.l() - right_bound_right_lane;
            } else {
              return false;
            }
          } else {
            return false;
          }
        }
      }
    } else if (left_bound_position == LanePosition::RIGHT) {//3.2目标参考线为当前车道, 左边界为右边参考线的左边界
      // AWARN_IF(FLAGS_enable_debug_motion)<<"left decision bound maybe error !! "<< "( "<<left_lane_id
      //                                  <<" ,"<<right_lane_id<<") .";
      double left_boundry = 0.0;
      double right_boundry = 0.0;
      if (!target_reference_line_info->reference_line().GetLaneWidth(s, &left_boundry,
                            &right_boundry)) {
        return false;
      } else {
        left_bound = -(right_boundry + FLAGS_boundary_width);//因为认知API提供的边界值减去了boundary_width_buffer，这里要加回来，否则会导致左边界相对实际往左移了一些
      }
      if (is_left_same_with_right) {//3.2.1目标参考线为当前车道, 左边界为右边参考线的左边界,右边界为右边参考线的右边界
        // AWARN_IF(FLAGS_enable_debug_motion)<<"maybe wrong decision bound : "<< "( "<<left_lane_id
        //                                <<" ,"<<right_lane_id<<") .";
        if (right_reference_line_data->GetWidthToLaneBoundary(left_bound_right_lane,right_bound_right_lane, s)) {
          Site nearest_point;
          int index;
          if (right_reference_line_data->GetNearestPoint(s, nearest_point, index)) {
            common::SLPoint sl_point;
            target_reference_line_info->reference_line().XYToSL(
                     common::math::Vec2d(nearest_point.xg, nearest_point.yg), &sl_point);
            right_bound = sl_point.l() - right_bound_right_lane;
          } else {
            return false;
          }
        } else {
          return false;
        }
      } else {
        if (right_reference_line_data->GetWidthToLaneBoundary(left_bound_right_lane,right_bound_right_lane, s)) {
          if (right_bound_position == LanePosition::CURRENT) {//3.2.2目标参考线为当前车道, 左边界为右边参考线的左边界,右边界为当前参考线的右边界
            AWARN_IF(FLAGS_enable_debug_motion)<<"maybe wrong decision bound : "<< "( "<<left_lane_id
                                       <<" ,"<<right_lane_id<<") .";
            return false;
          } else if (right_bound_position == LanePosition::LEFT) {//3.2.3目标参考线为当前车道, 左边界为右边参考线的左边界,右边界为左侧参考线的右边界
            AINFO_IF(FLAGS_enable_debug_motion)<<"maybe wrong decision bound : "<< "( "<<left_lane_id
                                       <<" ,"<<right_lane_id<<") .";
            return false;
           //TODO
          } else if (right_bound_position == LanePosition::RIGHT) {//3.2.4目标参考线为当前车道, 左边界为右边参考线的左边界,右边界为右侧参考线的右边界
            AINFO_IF(FLAGS_enable_debug_motion)<<"left and right bound lane is right ,but id is not the same : "<< "( "<<left_lane_id
                                       <<" ,"<<right_lane_id<<") .";
            Site nearest_point;
            int index;
            if (right_reference_line_data->GetNearestPoint(s, nearest_point, index)) {
              common::SLPoint sl_point;
              target_reference_line_info->reference_line().XYToSL(
                       common::math::Vec2d(nearest_point.xg, nearest_point.yg), &sl_point);
              right_bound = sl_point.l() - right_bound_right_lane;
            } else {
              return false;
            }
          }
        } else {
          return false;
        }
      }
    } else if (left_bound_position == LanePosition::LEFT ) {//3.3目标参考线为当前车道, 左边界为左边参考线的左边界
      double left_boundry = 0.0;
      double right_boundry = 0.0;
      if (!target_reference_line_info->reference_line().GetLaneWidth(s, &left_boundry,
                            &right_boundry)) {
        return false;
      }
      if (left_reference_line_data->GetWidthToLaneBoundary(left_bound_left_lane,right_bound_left_lane, s)) {
        Site nearest_point;
        int index;
        if (left_reference_line_data->GetNearestPoint(s, nearest_point, index)) {
          common::SLPoint sl_point;
          target_reference_line_info->reference_line().XYToSL(
                   common::math::Vec2d(nearest_point.xg, nearest_point.yg), &sl_point);
          left_bound = sl_point.l() + left_bound_left_lane;
        } else {
          return false;
        }
      } else {
        return false;
      }
      if (is_left_same_with_right) {//3.3.1目标参考线为当前车道, 左边界为左边参考线的左边界,右边界为左边参考线的右边界
        AWARN_IF(FLAGS_enable_debug_motion)<<"maybe wrong decision bound ,target is current ,left is left, right is left : "<< "( "<<left_lane_id
                                       <<" ,"<<right_lane_id<<") .";
        right_bound = left_boundry + FLAGS_boundary_width;
      } else {
        if (right_reference_line_data->GetWidthToLaneBoundary(left_bound_right_lane,right_bound_right_lane, s)) {
          if (right_bound_position == LanePosition::CURRENT) {//3.3.2目标参考线为当前车道, 左边界为左边参考线的左边界,右边界为当前参考线的右边界
            right_bound = - right_bound_right_lane;
          } else if (right_bound_position == LanePosition::LEFT) {//3.3.2目标参考线为当前车道, 左边界为左边参考线的左边界,右边界为左侧参考线的右边界
            AINFO_IF(FLAGS_enable_debug_motion)<<"left and right bound lane is rurrent ,but id is not the same and may be wrong : "<< "( "<<left_lane_id
                                       <<" ,"<<right_lane_id<<") .";
           //TODO
            right_bound = left_boundry + FLAGS_boundary_width;
          } else if (right_bound_position == LanePosition::RIGHT) {//3.3.2目标参考线为当前车道, 左边界为左边参考线的左边界,右边界为右侧参考线的右边界
            AINFO_IF(FLAGS_enable_debug_motion)<<"maybe wrong decision bound ,target is current ,left is left, right is right : "<< "( "<<left_lane_id
                                       <<" ,"<<right_lane_id<<") .";
            Site nearest_point;
            int index;
            if (right_reference_line_data->GetNearestPoint(s, nearest_point, index)) {
              common::SLPoint sl_point;
              target_reference_line_info->reference_line().XYToSL(
                       common::math::Vec2d(nearest_point.xg, nearest_point.yg), &sl_point);
              left_bound = sl_point.l() - right_bound_right_lane;
            } else {
              return false;
            }                           
          }
        } else {
          return false;
        }
      }
    } else if (left_bound_position == LanePosition::REVERSE) {//3.4目标参考线为当前车道, 左边界为逆向参考线的左边界
      double left_boundry = 0.0;
      double right_boundry = 0.0;
      if (!target_reference_line_info->reference_line().GetLaneWidth(s, &left_boundry,
                            &right_boundry)) {
        return false;
      }
      if (left_reference_line_data->GetWidthToLaneBoundary(left_bound_left_lane,right_bound_left_lane, s)) {
        // AWARN_IF(s < 10)<<"s= "<<s<<", reverse line bound = ("<<left_bound_left_lane<<", "<<right_bound_left_lane<<").";
        auto matched_point = target_reference_line_info->reference_line().GetNearestReferencePoint(s);
        // AWARN_IF(s < 10)<<"s= "<<s<<", target lane GetNearestReferencePoint = ("<<fixed<<matched_point.x()<<", "<<matched_point.y()<<").";
        double nearest_s = 0;
        double nearest_l = 0;
        Site target_match_point;
        target_match_point.xg = matched_point.x();
        target_match_point.yg = matched_point.y();
        if (XYToSL(left_reference_line_data->mapinfo,target_match_point,nearest_s,nearest_l)) {
          left_bound = fabs(nearest_l) + left_bound_left_lane;
          // AWARN_IF(s < 10)<<"reverse line point l to target line = ("<<fabs(nearest_l);
        } else {
          return false;
        }
      } else {
        return false;
      }
      if (is_left_same_with_right) {//3.4.1目标参考线为当前车道, 左边界为左边参考线的左边界,右边界为左边参考线的右边界
        AWARN_IF(FLAGS_enable_debug_motion)<<"maybe wrong decision bound ,target is current ,left is left, right is left : "<< "( "<<left_lane_id
                                       <<" ,"<<right_lane_id<<") .";
        right_bound = left_boundry;
      } else {
        if (right_reference_line_data->GetWidthToLaneBoundary(left_bound_right_lane,right_bound_right_lane, s)) {
          if (right_bound_position == LanePosition::CURRENT) {//3.4.2目标参考线为当前车道, 左边界为左边参考线的左边界,右边界为当前参考线的右边界
            right_bound = - right_bound_right_lane;
          } else if (right_bound_position == LanePosition::LEFT) {//3.4.2目标参考线为当前车道, 左边界为左边参考线的左边界,右边界为左侧参考线的右边界
            AINFO_IF(FLAGS_enable_debug_motion)<<"left and right bound lane is rurrent ,but id is not the same and may be wrong : "<< "( "<<left_lane_id
                                       <<" ,"<<right_lane_id<<") .";
           //TODO
            right_bound = left_boundry;
          } else if (right_bound_position == LanePosition::RIGHT) {//3.4.2目标参考线为当前车道, 左边界为左边参考线的左边界,右边界为右侧参考线的右边界
            AINFO_IF(FLAGS_enable_debug_motion)<<"maybe wrong decision bound ,target is current ,left is left, right is right : "<< "( "<<left_lane_id
                                       <<" ,"<<right_lane_id<<") .";
            Site nearest_point;
            int index;
            if (right_reference_line_data->GetNearestPoint(s, nearest_point, index)) {
              common::SLPoint sl_point;
              target_reference_line_info->reference_line().XYToSL(
                       common::math::Vec2d(nearest_point.xg, nearest_point.yg), &sl_point);
              left_bound = sl_point.l() - right_bound_right_lane;
            } else {
              return false;
            }                           
          }
        } else {
          return false;
        }
      }
    

    }
  }
  return true;
}

const ReferenceLineFrame* DecisionPathBoundProvider::FindTargetRefLine(const int id, LanePosition& position) {
  for (const auto& ref_line : 
    DP_->GetMainDataRef().cognition_info.struct_env_info.reference_line_info.current_reference_line) {
    if (id == ref_line.reference_lane_id) {
      position = LanePosition::CURRENT;
      return &ref_line;
    }
  }

  for (auto& ref_line : DP_->GetMainDataRef().cognition_info.struct_env_info.reference_line_info.left_reference_line) {
    if (id == ref_line.reference_lane_id) {
      position = LanePosition::LEFT;
      return &ref_line;
    }
  }

  for (auto& ref_line : DP_->GetMainDataRef().cognition_info.struct_env_info.reference_line_info.right_reference_line) { 
    if (id == ref_line.reference_lane_id) {
      position = LanePosition::RIGHT;
      return &ref_line;
    }
  }

  auto& ref_line = DP_->GetMainDataRef().cognition_info.struct_env_info.reference_line_info.reverse_reference_line;
  if (id == ref_line.reference_lane_id) {
    position = LanePosition::REVERSE;
    return &ref_line;
  }
  return nullptr;
}

}  // namespace planning
}  // namespace acu
