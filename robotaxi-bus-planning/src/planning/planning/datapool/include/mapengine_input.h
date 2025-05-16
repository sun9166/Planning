#ifndef DATAPOOL_INCLUDE_MAPENGINE_INPUT_H_
#define DATAPOOL_INCLUDE_MAPENGINE_INPUT_H_

#include "public_typedef.h"
#include "locperception_input.h"
#include "common/math/vec2d.h"
#include "common/math/box2d.h"

#include "mapengine_msgs.pb.h"

#include "conf/cognition_gflags.h"

using geometry::Site;
using geometry::SiteVec;
using namespace std;
using namespace acu::common;
using namespace acu::common::math; 

namespace acu {
namespace planning {

struct FuncInfo {
  int key;
  double a;
  double b;
  double c;
  double start_x;
  double end_x;
  int life_time;
  FuncInfo() {
    Reset();
  }
  FuncInfo(const mapengine_msgs::FuncInfo &input) {
    key = input.key();
    a = input.a();
    b = input.b();
    c = input.c();
    start_x = input.start_x();
    end_x = input.end_x();
    life_time = input.life_time();
  }
  void Reset() {
    key = -10;
    a = 0.0;
    b = 0.0;
    c = 0.0;
    start_x = 0.0;
    end_x = 0.0;
    life_time = 0.0;
  }
};

enum class LaneDirectionType {
  DEFAULT = 0,
  FORWARD = 1,
  BACKWARD = 2,
  DOUBLE_DIRECTION = 3
};

enum class eMapType {
  DEFAULT = 0,
  STRUCTURE_ROAD = 1,
  HYBRID_ROAD = 2,
  UNSTRUCTURE_ROAD = 3
};

enum class eLineType {
  WHITE_DASHED_LINE = 0,
  WHITE_SOLID_LINE = 1,
  YELLOW_DASHED_LINE = 2,
  YELLOW_SOLID_LINE = 3,
  DOUBLE_SOLID_LINE = 4,
  DOUBLE_DASHED_LINE = 5,
  DEFAULT = 6
};

enum class eLaneType {
  DEFAULT = 0,
  NONE = 1,
  CITY_DRIVING_LANE = 2,
  BIKING_LANE = 3,
  SIDEWALK_LANE = 4,
  PARKING_LANE = 5,
  BUSING_LANE = 6,
  ISLAND_LANE = 7,
  WAITINGLEFT = 8,
  WAITINGSTRAIGHT = 9
};

enum class eLandEndType {
  CROSSING = 0,
  ENTRY_POINT = 1,
  SPLIT_POINT = 2,
  TRAFFIC_LIGHT = 3,
  ZEBRA_CROSSING = 4,
  DEFAULT = 5
};

struct StructMissionInfo {
  double x;
  double y;
  double angle;
  bool is_park;
  StructMissionInfo() {
    Reset();
  }
  void Reset() {
    x = 0.0; 
    y = 0.0;
    angle = 0.0;
    is_park= false;
  }
  void Set(const mapengine_msgs::Point &mission_point) {
    x = mission_point.x();
    y = mission_point.y();
    angle = mission_point.angle();
  }
};

struct StructLightInfo {
  int color; // eLightColor
  int state; // eLightState
  double time;
  double left_time;
  int left_time_delay_cnt;

  int type; //eLightType

  StructLightInfo() {
    Reset();
  }
  void Reset() {
    color = -1;
    state = -1;
    time = 0.0;
    left_time = 0.0;
    left_time_delay_cnt = 0;
    type = 0;
  }
};

struct StructTrafficlightInfo {
  double time_stamp;
  string light_id;
  int lon_index;
  bool key_light;
  double light_s;
  Box2d box;
  string lane_id;
  vector<string> same_light_ids;
  vector<StructLightInfo> history_light;
  //add by ly
  bool is_v2x_traffic_light;
  double red_period;
  double green_period;
  double yellow_period;
  //
  StructTrafficlightInfo() {
  Reset();
  }
  void ClearData() {
    time_stamp = 0.0;
    light_s = 1000.0;
    light_id = "";
    lane_id = "";
    lon_index = -1;
    key_light = true;
    same_light_ids.clear();
    is_v2x_traffic_light = false;
    red_period = 0.0;
    green_period = 0.0;
    yellow_period = 0.0;
  }
  void Reset() {
    ClearData();
    history_light.clear();//单独清理
  }
  void SetNewLight(const TrafficLight &callback_light, const double &time) {
    time_stamp = time;
    light_id = callback_light.id;
    //add by ly for v2x traffic light 
    is_v2x_traffic_light = callback_light.is_v2x_traffic_light ;
    red_period = callback_light.red_period ;
    green_period = callback_light.green_period;
    yellow_period = callback_light.yellow_period; 
    //
    StructLightInfo new_light_info;
    new_light_info.color = (int)callback_light.color;
    new_light_info.type = (int)callback_light.type;
    new_light_info.time = 0.1;
    if (callback_light.number >= 0) {
      new_light_info.left_time = callback_light.number;
    }
    history_light.push_back(new_light_info);
  }
  void Set(const mapengine_msgs::TrafficLight &input) {
    Vec2d input_center(input.xg(), input.yg());
    Box2d input_box(input_center, input.globalangle() * M_PI / 180.0, 0.1, 4.0);
    box = input_box;
    light_s = input.light_s();
    light_id = input.id();
    lane_id = input.lane_id();
    lon_index = input.lon_index();
    key_light = input.key_light();
    for(const auto& id : input.same_light_ids()){
      same_light_ids.emplace_back(id);
    }
  }
};

struct MapEnginePoint {
  double x;
  double y;
  double angle;
  MapEnginePoint() {
    Reset();
  }
  void Reset() {
    x = 0.0;
    y = 0.0;
    angle = 0.0;
  }
  void Set(double _x, double _y, double _angle) {
    x = _x;
    y = _y;
    angle = _angle;
  }
  void Set(const mapengine_msgs::Point &p) {
    x = p.x();
    y = p.y();
    angle = p.angle();
  }
};

enum class RelationType {
  DEFAULT = 0,
  SELF = 1,
  LEFT = 2,
  RIGHT = 3,
  MERGE = 4,
  CROSS = 5,
  MERGEPRE = 6,
  CROSSPRE = 7,
  MERGESIM = 8,
  MERGESIMPRE = 9,
  REVERSE = 10
}; 

struct SortedObstacle {
  int id;
  int lon_index;
  double lane_s;
  double x;
  SortedObstacle() {
    id = 0;
    lon_index = 0;
    lane_s = 0.0;
    x = 0.0;
  }
  SortedObstacle(const int &id_, const int &lon_index_, 
                 const double &lane_s_, const double &x_) {
    id = id_;
    lon_index = lon_index_;
    lane_s = lane_s_;
    x = x_;
  }
};

struct RelationLane {
  string lane_id;
  RelationType type; 
  int lon_index; // "纵向级数"
  double start_s;
  double end_s;
  double relation_start_s;
  double relation_end_s;
  vector<SortedObstacle> sorted_objs;
  RelationLane() {
    Reset();
  }
  void Set(const string &lane_id_,
           const int &type_,
           const int &lon_index_,
           const double &start_s_,
           const double &end_s_,
           const double &relation_start_s_,
           const double &relation_end_s_) {
    lane_id = lane_id_;
    type = (RelationType)type_;
    lon_index = lon_index_;
    start_s = start_s_;
    end_s = end_s_;
    relation_start_s = relation_start_s_;
    relation_end_s = relation_end_s_;
  }
  void Reset() {
    lane_id = "";
    type = RelationType::DEFAULT;
    lon_index = -1;
    start_s = 0.0;
    end_s = 0.0;
    relation_start_s = 0.0;
    relation_end_s = 0.0;
    sorted_objs.clear();
    sorted_objs.reserve(100);
  }
};

struct MapEngineLineList { 
  bool able_driving;
  bool need_extend_parking_path;
  int global_cost;
  int first_lc_time;
  int all_lc_time;
  int first_lc_index;
  double dis2lc;
  double dis2line;
  double dis_to_end;
  double dis_to_diversion;
  double dis_to_merge;
  double first_lane_start_s;
  double leftside_length;
  double rightside_length;
  double dis_to_driving;
  string junction_id;
  LaneDirectionType direction;
  vector<string> front_lane_ids;
  vector<vector<string> > back_lane_idss;
  vector<RelationLane> front_relation_lanes;
  vector<RelationLane> back_relation_lanes;
  vector<pair<double, eLaneType> > lane_types;
  vector<pair<double, int> > lane_turns;
  vector<pair<double, int> > left_bd_types;
  vector<pair<double, int> > right_bd_types;
  vector<pair<double, double> > left_passable_distances;
  vector<pair<double, double> > right_passable_distances;
  vector<pair<double, double> > expected_speeds;
  vector<pair<double, double> > distance_to_speed_bumps;
  vector<pair<double, double> > distance_to_forbid_areas;
  vector<pair<double, double> > distance_to_junctions;
  vector<pair<double, double> > distance_to_crosswalks;
  vector<pair<double, double> > distance_to_yields;
  vector<pair<double, double> > distance_to_stop_areas;
  vector<pair<double, double> > distance_to_ends;
  //add by ly 附近多个隔离带
  vector<vector<pair<double, double> > >isolationbelts;
  //
  StructMissionInfo mission;
  double dis2missionpoint;
  bool contain_light_junction;
  double contain_light_s;
  vector<StructTrafficlightInfo> all_trafficlights;// 需要维护历史信息
  StructTrafficlightInfo trafficlight;
  SiteVec path_points;
  
  MapEngineLineList() {
    Reset();
  }
  void ClearData() {
    able_driving = true;
    need_extend_parking_path = false;
    dis2lc = 1000.0;
    first_lc_time = 0;
    all_lc_time = 0;
    first_lc_index = -1;
    global_cost = 1000;
    dis2line = 0.0;
    dis_to_end = 1000.0;
    dis_to_diversion = 1000.0;
    dis_to_merge = 1000.0;
    first_lane_start_s = 0.0;
    leftside_length = 0.0;
    rightside_length = 0.0;
    dis_to_driving = 0.0;
    direction = LaneDirectionType::DEFAULT;
    junction_id = "";
    path_points.clear();
    path_points.reserve(3000);
    front_relation_lanes.clear();
    back_relation_lanes.clear();
    back_lane_idss.clear();
    lane_types.clear();
    lane_turns.clear();
    left_bd_types.clear();
    right_bd_types.clear();
    distance_to_speed_bumps.clear();
    distance_to_forbid_areas.clear();
    distance_to_junctions.clear();
    distance_to_crosswalks.clear();
    distance_to_yields.clear();
    distance_to_stop_areas.clear();
    distance_to_ends.clear();
    expected_speeds.clear();
    left_passable_distances.clear();
    right_passable_distances.clear();
    mission.Reset();
    dis2missionpoint = 0.0;
    contain_light_junction = false;
    contain_light_s = 1000.0;
    for (auto &light : all_trafficlights) {
      light.ClearData();
    }
    trafficlight.Reset();
    isolationbelts.clear();
  }
  void Reset() {
    ClearData();
    front_lane_ids.clear();
    all_trafficlights.clear();
    trafficlight.Reset(); 
  }
  void SetMsg(const mapengine_msgs::LineList &p) {
    Reset();
    able_driving = p.frontline().able_driving();
    dis2lc = p.dis_lc();
    first_lc_time = p.first_lc_time();
    all_lc_time = p.all_lc_time();
    first_lc_index = p.first_lc_index();
    global_cost = p.global_cost();
    dis2line = p.frontline().dis_to_line();
    if (!p.frontline().dis_to_ends().empty()) {
      dis_to_end = p.frontline().dis_to_ends().rbegin()->end_s();
    }
    dis_to_diversion = p.frontline().dis_to_diversion();
    dis_to_merge = p.frontline().dis_to_merge();
    first_lane_start_s = p.frontline().first_lane_start_s();
    leftside_length = p.frontline().laneside_left();
    rightside_length = p.frontline().laneside_right();
    dis_to_driving = p.frontline().dis_to_driving();
    direction = (LaneDirectionType)p.frontline().line_direction();
    for(const auto& id : p.frontline().lane_ids()){
      front_lane_ids.emplace_back(id);
    }
    for (auto &relation_lane : p.frontline().relation_lanes()) {
      RelationLane temp_relation_lane;
      temp_relation_lane.Set(relation_lane.id(), relation_lane.type(), 
                    relation_lane.lon_index(), relation_lane.start_s(), 
                    relation_lane.end_s(), relation_lane.relation_start_s(), 
                    relation_lane.relation_end_s());
      front_relation_lanes.push_back(temp_relation_lane);
    }

    for (auto &backline : p.backlines()) {
      vector<string> back_lane_ids;
      for(const auto& lane_ids : backline.lane_ids()){
        back_lane_ids.emplace_back(lane_ids);
      }
      back_lane_idss.push_back(back_lane_ids);
      for (auto &relation_lane : backline.relation_lanes()) {
        RelationLane temp_relation_lane;
        temp_relation_lane.Set(relation_lane.id(), relation_lane.type(), 
                      relation_lane.lon_index(), relation_lane.start_s(), 
                      relation_lane.end_s(), relation_lane.relation_start_s(), 
                      relation_lane.relation_end_s());
        back_relation_lanes.push_back(temp_relation_lane);
      }
    }

    pair<double, eLaneType> temp_lane_type;
    for (auto &lane_type : p.frontline().lane_types()) {
      temp_lane_type.first = lane_type.end_s();
      temp_lane_type.second = (eLaneType)lane_type.type();
      lane_types.push_back(temp_lane_type);
    }
    pair<double, int> temp_lane_turn;
    for (auto &lane_turn : p.frontline().lane_turns()) {
      temp_lane_turn.first = lane_turn.end_s();
      temp_lane_turn.second = lane_turn.type();
      lane_turns.push_back(temp_lane_turn);
    }
    pair<double, int> temp_bd_type;
    for (auto &left_bd_type : p.frontline().left_bd_types()) {
      temp_bd_type.first = left_bd_type.end_s();
      temp_bd_type.second = left_bd_type.type();
      left_bd_types.push_back(temp_bd_type);
    }
    for (auto &right_bd_type : p.frontline().right_bd_types()) {
      temp_bd_type.first = right_bd_type.end_s();
      temp_bd_type.second = right_bd_type.type();
      right_bd_types.push_back(temp_bd_type);
    }
    pair<double, double> temp_passable_distance; 
    for (auto &left_passable_distance : p.frontline().left_passable_distances()) {
      temp_passable_distance.first = left_passable_distance.start_s();
      temp_passable_distance.second = left_passable_distance.end_s();
      left_passable_distances.push_back(temp_passable_distance);
    }
    for (auto &right_passable_distance : p.frontline().right_passable_distances()) {
      temp_passable_distance.first = right_passable_distance.start_s();
      temp_passable_distance.second = right_passable_distance.end_s();
      right_passable_distances.push_back(temp_passable_distance);
    }
    pair<double, double> temp_speed_bump; 
    for (auto &dis_to_speed_bump : p.frontline().dis_to_speed_bumps()) {
      temp_speed_bump.first = dis_to_speed_bump.start_s();
      temp_speed_bump.second = dis_to_speed_bump.end_s();
      distance_to_speed_bumps.push_back(temp_speed_bump);
    }
    pair<double, double> temp_forbid_area; 
    for (auto &dis_to_forbid_area : p.frontline().dis_to_forbid_areas()) {
      temp_forbid_area.first = dis_to_forbid_area.start_s();
      temp_forbid_area.second = dis_to_forbid_area.end_s();
      distance_to_forbid_areas.push_back(temp_forbid_area);
    }
    pair<double, double> temp_yield_area; 
    for (auto &dis_to_yield_area : p.frontline().dis_to_yield_areas()) {
      temp_yield_area.first = dis_to_yield_area.start_s();
      temp_yield_area.second = dis_to_yield_area.end_s();
      distance_to_yields.push_back(temp_yield_area);
    }
    pair<double, double> temp_junction; 
    for (auto &dis_to_junction : p.frontline().dis_to_junctions()) {
      temp_junction.first = dis_to_junction.start_s();
      temp_junction.second = dis_to_junction.end_s();
      distance_to_junctions.push_back(temp_junction);
    }
    if (!p.frontline().dis_to_junctions().empty() && 
        !p.frontline().dis_to_junctions().begin()->id().empty()) {
      junction_id = p.frontline().dis_to_junctions().begin()->id();
    }
    pair<double, double> temp_crosswalk; 
    for (auto &dis_to_crosswalk : p.frontline().dis_to_crosswalks()) {
      temp_crosswalk.first = dis_to_crosswalk.start_s();
      temp_crosswalk.second = dis_to_crosswalk.end_s();
      distance_to_crosswalks.push_back(temp_crosswalk);
    }
    pair<double, double> temp_stop_area; 
    for (auto &dis_to_stop_area : p.frontline().dis_to_stop_areas()) {
      temp_stop_area.first = dis_to_stop_area.start_s();
      temp_stop_area.second = dis_to_stop_area.end_s();
      distance_to_stop_areas.push_back(temp_stop_area);
    }
    pair<double, double> temp_end; 
    for (auto &dis_to_end : p.frontline().dis_to_ends()) {
      temp_end.first = dis_to_end.start_s();
      temp_end.second = dis_to_end.end_s();
      distance_to_ends.push_back(temp_end);
    }
    pair<double, double> temp_speed; 
    for (auto &expected_speed : p.frontline().expected_speeds()) {
      temp_speed.first = expected_speed.end_s();
      temp_speed.second = expected_speed.value();
      expected_speeds.push_back(temp_speed);
    }
    contain_light_junction = p.frontline().contain_light_junction();
    contain_light_s = p.frontline().contain_light_s();
    for (auto& map_light : p.frontline().trafficlights()) {
      StructTrafficlightInfo trafficlight;
      trafficlight.Set(map_light);
      if (trafficlight.light_id == "") continue; //add by ly for filter no light_id light
      all_trafficlights.push_back(trafficlight);
    }
    
    path_points.clear();
    path_points.reserve(3000);
    for (int i = 0; i < p.frontline().map_points().size(); i++) {
      Site temp_point;
      temp_point.x = p.frontline().map_points()[i].x();
      temp_point.y = p.frontline().map_points()[i].y();
      temp_point.xg = p.frontline().map_points()[i].xg();
      temp_point.yg = p.frontline().map_points()[i].yg();
      if(direction == LaneDirectionType::BACKWARD) {
        temp_point.angle = acu::common::math::NormalizeAngle(
          ( p.frontline().map_points()[i].angle() + 180.0 ) * M_PI/180.0 ) * 180.0/M_PI;
        temp_point.globalangle = acu::common::math::NormalizeAngle(
          ( p.frontline().map_points()[i].angleglobal() + 180.0 ) * M_PI/180.0 ) * 180.0/M_PI;
      } else {
        temp_point.angle = p.frontline().map_points()[i].angle();
        temp_point.globalangle = p.frontline().map_points()[i].angleglobal();
      }
      temp_point.index = i;
      temp_point.length = p.frontline().map_points()[i].length();
      temp_point.curvature = p.frontline().map_points()[i].curvature();
      temp_point.direction = (int)direction;
      path_points.push_back(temp_point);
    }
    //add by ly 0310
    pair<double, double> temp_isolation_point;
    vector<pair<double, double> > temp_isolation;
    isolationbelts.clear();
    for (int i = 0; i < p.frontline().isolationbelts().size(); i++) {
      temp_isolation.clear();
      for (int j = 0 ; j < p.frontline().isolationbelts()[i].area_points().size(); j++) {
        temp_isolation_point.first =  p.frontline().isolationbelts()[i].area_points()[j].x(); 
        temp_isolation_point.second = p.frontline().isolationbelts()[i].area_points()[j].y();
        temp_isolation.push_back(temp_isolation_point);
      }
      isolationbelts.push_back(temp_isolation);
    }
    //
  }
  void SetNeedExtend(bool _need_extend) {
    need_extend_parking_path = _need_extend;
  }
  bool IsNeedExtend() const {
    return need_extend_parking_path;
  }
};

struct MapInfoData {
  eMapType maptype;
  std::vector<MapEngineLineList> alllinelists;
  MapEngineLineList revlinelist;
  MapEnginePoint startpoint;
  MapEnginePoint endpoint;
  double dis2missionpoint;
  int index;
  bool task_change;
  MapInfoData() {
    Reset();
  }
  void Reset() {
    maptype = eMapType::DEFAULT;
    alllinelists.clear();
    revlinelist.Reset();
    startpoint.Reset();
    endpoint.Reset();
    dis2missionpoint = 0.0;
    index = -1;
    task_change = false;
  }
};


typedef struct MapEngineData {
  double time;
  FaultStatus mapengine_status;
  MapInfoData map_info_data;
  LocalizationData loc_data;
  LocalizationData last_loc_data;
  LocalizationData correction_loc_data;
  LocalizationData last_correct_loc_data;
  vector<FuncInfo> last_vision_data;
  vector<FuncInfo> vision_data;

  MapEngineData() {
    Reset();
  }
  void Reset() {
    map_info_data.Reset();
    loc_data.Reset();
    last_loc_data.Reset();
    correction_loc_data.Reset();
    last_correct_loc_data.Reset();
    last_vision_data.clear();
    vision_data.clear();
  }
  void Set(MapEngineData &new_data) {
    time = new_data.time;
    map_info_data = new_data.map_info_data;
    loc_data = new_data.loc_data;
    last_loc_data = new_data.last_loc_data;
    correction_loc_data = new_data.correction_loc_data;
    last_correct_loc_data = new_data.last_correct_loc_data;
    last_vision_data = new_data.last_vision_data;
    vision_data = new_data.vision_data;
    if (map_info_data.revlinelist.front_lane_ids.size()) {
      for (auto &linelist : map_info_data.alllinelists) {
        if (linelist.front_lane_ids.size() &&
            linelist.front_lane_ids.front().back() - '0' == 1) {
          int lon_index = 0, type = (int)RelationType::REVERSE;
          double s = 0.0;
          RelationLane temp_relation_lane;
          for (auto &rev_lane : map_info_data.revlinelist.front_lane_ids) {
            temp_relation_lane.Set(rev_lane, type, lon_index, s, s, s, s);
            linelist.front_relation_lanes.push_back(temp_relation_lane);
            AINFO_IF(FLAGS_log_enable)<<"add relation rev lane "<<rev_lane;
          }
        }
      }
    }
  }
} MapEngineData;

} // namespace planning
} // namespace acu

#endif // DATAPOOL_INCLUDE_MAPENGINE_INPUT_H_