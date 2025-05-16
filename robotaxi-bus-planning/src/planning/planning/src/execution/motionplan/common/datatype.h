#ifndef _DATAYPE_H
#define _DATAYPE_H
#pragma once

#include "global_define.h"
#include "common/math/vec2d.h"
#include "common/math/box2d.h"
#include "datapool/include/cognition_typedef.h"
#include "decision.pb.h"

using namespace std;

namespace acu {
namespace planning {

    const double kDEG_TO_RAD = 0.017453293;  
    const double kRAD_TO_DEG = 57.29578;

    namespace {
    // LaneBoundPoint contains: (s, l_min, l_max).
    using LaneBoundPoint = std::tuple<double, double, double>;
    // LaneBound contains a vector of PathBoundPoints.
    using LaneBound = std::vector<LaneBoundPoint>;
    // ObstacleEdge contains: (is_start_s, s, l_min, l_max, obstacle_id).
    using ObstacleEdge = std::tuple<int, double, double, double, std::string>;

    }

    struct BlockedInfo {
       bool is_blocked;
       size_t index;
       double s;
     
       BlockedInfo() {
         is_blocked = false;
         index = 0.0;
         s = 0.0;
       }
   
       void reset() {
         is_blocked = false;
         index = 0.0;
         s = 0.0;
       }
     };

    enum class LanePosition{
       CURRENT = 0,
       LEFT = 1,
       RIGHT = 2,
       REVERSE = 3
    };

    enum class LaneChangingType{
       NONE = 0,
       MISSION = 1,
       SPEED_PRIOR = 2
    };

    enum class NewPathType {
       NONE = 0,
       IN_LANE = 1,
       LANE_CHANGE = 2,
       BORROW_LANE = 3,
       ABANDON_LANE_CHANGE = 4
    };

    enum class LaneBorrowInfo {
        NO_BORROW = 0,
        LEFT_BORROW = 1,
        RIGHT_BORROW = 2
    };

    struct VehicleState{
        double x ;
        double y ;
        double z ;
        double timestamp ;
        double roll ;
        double pitch ;
        double yaw ;
        double heading ;
        double dr_x;
        double dr_y;
        double dr_yaw;
        double kappa ;
        double linear_velocity ;
        double angular_velocity ;
        double linear_acceleration ;
        unsigned int gear ;
        unsigned int driving_mode ;
        int brake_state;
        void Clear() {
            x = 0;
            y = 0;
            z = 0;
            timestamp = 0;
            roll = 0;
            pitch = 0;
            yaw = 0 ;
            heading = 0 ;
            dr_x = 0.0;
            dr_y = 0.0;
            dr_yaw = 0.0;
            kappa = 0 ;
            linear_velocity = 0;
            angular_velocity = 0;
            linear_acceleration = 0;
            gear = 0;
            driving_mode = 0;
            brake_state = 0;
        }
    };
    struct VirtualObstacle {
        std::string id;
        bool enable_relative_distance;
        common::math::Box2d obstacle_box;
        double stop_distance;
        StopReasonCode stop_reason;
        VirtualObstacle() {
            id = "";
            enable_relative_distance = false;
            stop_distance = 0.0;
            stop_reason = StopReasonCode::STOP_REASON_YIELD_SIGN;
        }
        void reset() {
            id = "";
            enable_relative_distance = false;
            stop_distance = 0.0;
            stop_reason = StopReasonCode::STOP_REASON_YIELD_SIGN;
        }
    };
  }//namespace planning 
} //namespace acu

#endif


