#ifndef DATAPOOL_INCLUDE_DECISION_TYPEDEF_H_
#define DATAPOOL_INCLUDE_DECISION_TYPEDEF_H_

#include "cognition_typedef.h"
#include "common/math/box2d.h"
#include "planning_debug_msgs.pb.h"

using PathBoundPoint = std::tuple<double, int, int>;
using PathBound = std::vector<PathBoundPoint>;

namespace acu {
namespace planning {

enum class eObjectDecisionEnum {
  DEFAULT = 0,
  TAKEWAY = 1,
  GIVEWAY = 2,
  FOLLOW = 3,
  IGNORE = 4, 
  CONFLICT = 5,
  SPEED_LIMIT = 6,
  SIDEPASS = 7
};

enum class eReplanReasonEnum {
  DEFAULT_VALUE = 0,
  MISSION_LC = 1,
  OBSTACLE = 2,
  SPEED_LC = 3,
  MEETING = 4,
  ABANDON_LC = 5, 
  PULL_OVER = 6, 
  OFFSET = 7,
  COMMAND_LC = 8, 
  DEPARTURE = 9, 
  PREPARE = 10
};

enum class eActionEnum {
  DEFAULT_VALUE = 0,
  START = 1,
  STOP = 2,
  YIELD = 3,
  PARKING = 4,
  OBSTACLE_AVOID = 5,
  LANE_CHANGE = 6,
  PULL_OVER = 7,
  CHANGE_OFFSET = 8,
  LANE_FOLLOWING = 9,
  RE_MISSION_PLAN = 10, 
  GIVE_WAY = 11,
  ABANDON_LANE_CHANGE = 12
};

enum class eDirectionEnum {
  DEFAULT_VALUE = 0,
  LEFT = 1,
  RIGHT = 2,
  FORWARD = 3,
  BACKWARD = 4
};

struct PathPointStruct {
  double x;
  double y;
  double heading;
};

struct SentenceStruct {
  eReplanReasonEnum intention;
	eActionEnum action;
	eDirectionEnum direction;
	PathPointStruct point;
  common::math::Box2d box;
  double dis_to_end;
  double dis_to_boundary;
  bool boundary_enable;
  int pullover_line_id;
  SentenceStruct() {
    intention = eReplanReasonEnum::DEFAULT_VALUE;
    direction = eDirectionEnum::DEFAULT_VALUE;
  }
};

struct DecisionInfo {
  double timestamp;
  int reference_line_id = 0;
  int giveway_id = -1;
  int follow_id = -1;
  int object_decision_success = 0;
  PathBound path_bound;
  double speed_limit;
  std::unordered_map<int, eObjectDecisionEnum> object_decision;
  std::unordered_map<int, eObjectDecisionEnum> last_object_decision;
  std::vector<SentenceStruct> sentence;
  planning_debug_msgs::DebugSTGraph st_graph;
  std::vector<std::string> target_lanes;
  std::pair<int, int> gap_id;
  std::pair<double, double> speed_limit_point;
  int prepare_direction = 0;
  bool is_congestion;
  bool is_arrive_target_line_failed = false;
  int borrow_lane_type = 0;
  double expand_l;
  double recommended_a;
  std::set<int> meeting_ids;
  std::vector<Box2d> dynamic_sl;
};

enum class eScenarioEnum {
  DEFAULT_VALUE = 0,
  CRUISE = 1,
  JUNCTION = 2,
  PULLOVER = 3,
  VALET = 4,
  HIGHWAY = 5
};

enum class eReplanStateEnum {
  CRUISE = 0,
  PREPARE = 1,
  EXCUTE = 2
};

struct OptionStruct {
  ReferenceLineFrame* line_info;
  int type;
  float global_cost;
  float lc_cost;
  float stable_cost;
  float speed_cost;
  float total_cost;
  bool is_safe;
  double speed;
  std::vector<bool> boundary;
  std::pair<double, double> lc_s;
};

struct MapNodeStruct {
  double s;
  double l;
  double last_l;
  int lc_cost;
  int lane_num;
  int last_lane_num;
  bool is_solid;
  bool is_opposite;
  double lc_dis;
};

struct SpeedNodeStruct {
  int last_s;
  int s;
  double v;
};

struct STNodeStruct {
  float last_s;
  float s;
  float t;
  float v;
  float a;
  float total_cost;
  float safety_cost;
  float object_cost;
  float comfort_cost;
  int s_id;
  int t_id;
  std::map<int, int> object_decision;
  STNodeStruct() {
    s = 0.0;
    t = 0.0;
    safety_cost = 0.0;
    object_cost = 0.0;
    comfort_cost = 0.0;
    s_id = 0;
    t_id = 0;
  }
};

struct InteractionInfo {
  float center_t;
  float center_s;
  float max_p;
  float modified_a;
  float expected_a;
  float acc;
  int pre_decision;
  int conflict_type;
  int right_of_way;
  int controllability;
  float push_cost;
  bool is_pushed;
  std::map<float, std::set<float>> key_points;
  float interaction_time;
  float decision_time;
  float yield_p;
  float overtake_p;
  int special_type;
  int priority;
  float keep_decision_time;//add by liuya

  InteractionInfo() {
    center_t = 0.0;
    center_s = 0.0;
    max_p = 0.0;
    acc = 0.0;
    pre_decision = 0;
    conflict_type = 0;
    right_of_way = 0;
    controllability = 0;
    key_points.clear();
    is_pushed = false;
    push_cost = 0.0;
    interaction_time = 0.0;
    decision_time = 0.0;
    special_type = 0;
    priority = 0;
    keep_decision_time = 0.0;//add by liuya
  }
};

struct GridInfoStruct {
  float s;
  float l;
  float t;
  float p;
  float pre_s;
  float pre_l;
  int s_id;
  int l_id;
  int pre_s_id;
  int pre_l_id;
  bool l_solid;
  bool r_solid;
  std::vector<int> line_ids;
  float v;
  float a;
  float delta;
  double xg;
  double yg;
  double safety_cost;
  double comfort_cost;
  double total_cost;
  GridInfoStruct() {
    p = 0.0;
    pre_s_id = -1;
    pre_l_id = -1;
    total_cost = std::numeric_limits<double>::max();
  }
};

struct MapPointStruct {
  double s;
  double l;
  double last_l;
  double delta;
  double min_radius;
  int lc_num;
  bool l_solid;
  bool r_solid;
  bool l_on_line;
  bool r_on_line;
  std::vector<int> line_ids;
  double cost;
  double xg;
  double yg;
  bool last_on_ref;

  MapPointStruct() {
    Reset();
  }

  void Reset() {
    lc_num  = std::numeric_limits<int>::max();
    l_solid = 0;
    r_solid = 0;
    line_ids.clear();
    cost = std::numeric_limits<double>::max();
  }
};

}
}

#endif
