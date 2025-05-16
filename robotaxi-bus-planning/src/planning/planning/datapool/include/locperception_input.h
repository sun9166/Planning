#ifndef DATAPOOL_INCLUDE_LOCPERCEPTION_INPUT_H_
#define DATAPOOL_INCLUDE_LOCPERCEPTION_INPUT_H_

#include "public_typedef.h"
#include "prediction_input.h"
#include "vectormap.h"
#include "map/vectormap/src/hdmap/hdmap.h" 

//#include "mapengine_msgs/ImuInfo.h"
//#include "planning_msgs/ImuInfo.h"
#include "mapengine_msgs.pb.h"
#include "planning_msgs.pb.h"
#include "polygon2d.h"
#include "line_segment2d.h"

using namespace std;
using namespace acu::common;
using namespace acu::hdmap;
using namespace acu::common::math;

namespace acu {
namespace planning {

enum class eObjectType {
  CAR = 0,
  TRUCK = 1,
  PEDESTRIAN = 2,
  CYCLIST = 3,
  VIRTUAL = 4,
  UNKNOWN = 5,
  BLINDLIDAR = 6,
  OBUEVENTS = 7
};

enum class eLightColor {
  UNKNOWN_COLOR = 0,
  RED = 1,
  YELLOW = 2,
  GREEN = 3,
  BLACK = 4,
  FLICKER_RED = 5,
  FLICKER_YELLOW = 6,
  FLICKER_GREEN = 7,
  LENGTH_BLACK = 8,
  LENGTH_YELLOW = 9,
  UNNORMAL = 10
};

enum class eLightState {
  NORMAL = 0,
  FLICKER_RED = 1,
  FLICKER_YELLOW = 2,
  FLICKER_GREEN = 3,
  LENGTH_BLACK = 4,
  LENGTH_YELLOW = 5,
  UNNORMAL = 6
};
enum class eLightType {
  TYPE_UNKNOWN = 0,
  TYPE_ARROW_STRAIGHT = 1,
  TYPE_ARROW_LEFT = 2,
  TYPE_ARROW_RIGHT = 3,
  TYPE_ARROW_UTURN = 4,
  TYPE_CIRCLE = 5,
  TYPE_NUM = 6
};

//
struct LocalizationData {
  double time;
  double xg, yg, zg, roll, pitch, yaw;
  double velocity;
  double vx, vy;
  double yawrate;
  double loc_xg_dr;
  double loc_yg_dr;
  double loc_zg_dr;
  double loc_yaw_dr;
  bool correction_flag;
  FaultStatus imu_status;
  
  LocalizationData() {
    Reset();
  }

  void Reset() {
    xg = 0.0;
    yg = 0.0;
    zg = 0.0;
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
    velocity = 0.0;
    vx = 0.0;
    vy = 0.0;
    yawrate = 0.0;
    loc_xg_dr = 0.0;
    loc_yg_dr = 0.0;
    loc_zg_dr = 0.0;
    loc_yaw_dr = 0.0;
    correction_flag = false;
  }

  void Set(const mapengine_msgs::ImuInfo &input) {
    time = input.time_stamp();
    xg = input.xg(); 
    yg = input.yg();
    yaw = input.yaw();
    velocity = input.velocity();
    vx = input.vx();
    vy = input.vy();
    loc_xg_dr = input.loc_xg_dr();
    loc_yg_dr = input.loc_yg_dr();
    loc_yaw_dr = input.loc_yaw_dr();
    correction_flag = input.correction_flag();
  }

  void Set(const LocalizationData &input) {
    time = input.time;
    xg = input.xg; 
    yg = input.yg;
    yaw = input.yaw;
    velocity = input.velocity;
    vx = input.vx;
    vy = input.vy;
    loc_xg_dr = input.loc_xg_dr;
    loc_yg_dr = input.loc_yg_dr;
    loc_yaw_dr = input.loc_yaw_dr;
    correction_flag = input.correction_flag;
  }

  void SetMsg(planning_msgs::ImuInfo &output) {
    output.set_xg(xg);
    output.set_yg(yg);
    output.set_yaw(yaw);
    output.set_velocity(velocity);
    output.set_vx(vx);
    output.set_vy(vy);
    output.set_loc_xg_dr(loc_xg_dr);
    output.set_loc_yg_dr(loc_yg_dr);
    output.set_loc_yaw_dr(loc_yaw_dr);
    output.set_correction_flag(correction_flag);

  }

  geometry::Site ToSite() const{
    geometry::Site loc_p;
    loc_p.xg = xg;
    loc_p.yg = yg;
    loc_p.globalangle = yaw;
    loc_p.x = 0.0;
    loc_p.y = 0.0;
    loc_p.angle = 0.0;
    loc_p.length = 0.0;
    loc_p.velocity = velocity;
    return loc_p;
  }
};

struct LioData {
  double time;
  double x, y, z, roll, pitch, yaw;
  LioData() {
    Reset();
  }
  void Reset() {
    time = 0.0;
    x = 0.0;
    y = 0.0;
    z = 0.0;
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
  }
};

struct StructSLBoundary {
  double min_s;
  double max_s;
  double min_l;
  double max_l;
  StructSLBoundary() {
    Reset();
  }
  void Reset() {
    min_s =  1000.0;
    max_s = -1000.0;
    min_l =  1000.0;
    max_l = -1000.0;
  }
  void Set(double _s, double _l) {
    min_s = fmin(_s, min_s);
    max_s = fmax(_s, max_s);
    min_l = fmin(_l, min_l);
    max_l = fmax(_l, max_l);
  }
};

struct StructSTBoundary {
  std::vector<std::pair<acu::common::math::Vec2d, 
                        acu::common::math::Vec2d>> s_t;
  double collision_vsabs;
  double ttc;
  StructSTBoundary() {
    Reset();
  }
  void Reset() {
    s_t.clear();
    ttc = 1000.0;
    collision_vsabs = 1000.0;
  }
  void set(double min_s, double max_s, double t) {
    acu::common::math::Vec2d v2d_min, v2d_max;
    v2d_min.set_x(min_s);
    v2d_min.set_y(t);
    v2d_max.set_x(max_s);
    v2d_max.set_y(t);
    std::pair<acu::common::math::Vec2d, acu::common::math::Vec2d> p1;
    p1.first = v2d_min;
    p1.second = v2d_max;
    s_t.push_back(p1);
  }
};

typedef struct ObjectCell
{
  int idc;
  int type;
  double x;
  double y;
  double xg;
  double yg;
  ObjectCell() {
    idc = 0;
    type = 0;
    x = 0.0;
    y = 0.0;
    xg = 0.0;
    yg = 0.0;
  }
} ObjectCell;

struct LaneSL {
  double s;
  double l;
  LaneSL() {
    Reset();
  }
  void Reset() {
    s = 0;
    l = 0;
  }
};

typedef struct CallbackObject {// from perception msgs
  //add by ly
  bool is_v2v_obj = false;
  std::string vehicle_vin;
  int flash_light = 0;
  double acc_x;
  //
  int id;                                           
  double x;
  double y;
  double z;
  double xabs;
  double yabs;
  double global_angle;

  double vxrel;
  double vyrel;
  double vxabs;
  double vyabs;
  double speed;
  double vsabs;// frenet坐标系下
  double vlabs;// frenet坐标系下

  double width;
  double length;
  double height;
  int type;
  int source; 
  double confidence;
  double age; 
  int position_property;
  int moving_status;
  std::vector<ObjectCell> cells;
  
  CallbackObject() {
    cells.reserve(MAX_OBJ_CELL_SIZE);
    Reset();   
  }
  void Reset() {
    is_v2v_obj = false;
    flash_light = 0;
    acc_x = 0.0;
    id = 0;
    x = 0.0;
    y = 0.0;
    z = 0.0;
    xabs = 0.0;
    yabs = 0.0;
    global_angle = 0.0;

    vxrel = 0.0;
    vyrel = 0.0;
    vxabs = 0.0;
    vyabs = 0.0;
    speed = 0.0;
    vsabs = 0.0;
    vlabs = 0.0; 

    width = 0.0;
    length = 0.0;
    height = 0.0;
    type = (int)eObjectType::UNKNOWN;
    source = 0;
    confidence = 0.0;
    age = 0.1;
    position_property = 0;
    moving_status = 0;
    cells.clear();
    cells.reserve(MAX_OBJ_CELL_SIZE);

  }
  ~CallbackObject() {
    cells.clear();
  }
} CallbackObject;

typedef struct ObjectHistory {
  double speed;
  int type;
  // double dis_to_junction;
  string obj_lane_id;
  std::pair<double, double> position;
  // double length;
  // double width;
  ObjectHistory() {
    speed = 0.0;
    type = (int)eObjectType::UNKNOWN;
    // dis_to_junction = 1000.0;
    obj_lane_id = "";
    position.first = 1000.0;
    position.second = 1000.0;
    // length = 0.0;
    // width = 0.0;
  }
} ObjectHistory;

typedef struct PdObjectStruct {
  vector<string> pd_id;
  bool update_flag;
  int prediction_index;
  vector<ST> st;
  vector<ST> history_st;
  int conflict_type;
  int right_of_way;
  int accumulate_time;
  PdObjectStruct() {
    Reset();
  }
  void Reset() {
    pd_id.clear();
    update_flag = false;
    prediction_index = -1;
    st.clear();
    history_st.clear();
    conflict_type = 0;
    right_of_way = 1;
    accumulate_time = 0;
  }
  bool equal(vector<string> &input_lane_ids, int match_lane_num = 0) {
    if (input_lane_ids.empty()) {
      return true;
    }
    // 删除input_lane_ids 重复lane id
    for (int i = 1; i < input_lane_ids.size(); i++) {
      if (input_lane_ids.at(i-1) == input_lane_ids.at(i)) {
        input_lane_ids.erase(input_lane_ids.begin() + i);
        i--;
      }
    }
    if (input_lane_ids.empty()) {
      return true;
    }
    // 删除pd_id 重复 lane id
    for (int i = 1; i < pd_id.size(); i++) {
      if (pd_id.at(i-1) == pd_id.at(i)) {
        pd_id.erase(pd_id.begin() + i);
        i--;
      }
    }
    if (input_lane_ids.empty()) {
      return false;
    }

    // 要求预测线一毛一样，才算匹配
    if (match_lane_num == 0) {
      // 如果数目相同
      if (input_lane_ids.size() == pd_id.size()) {
        // 完全相同
        if (input_lane_ids == pd_id) {
          return true;
        }
        else {
          // input_lane_ids 为 pd_id 的后继
          for (int i = 1; i < input_lane_ids.size(); i++) {
            if (input_lane_ids.at(i-1) != pd_id.at(i)) {
              return false;
            }
          }
          return true;
        }
      }
      else if (input_lane_ids.size() < pd_id.size()) {
        for (int i = input_lane_ids.size() - 1; i >= 0; i--) {
          if (input_lane_ids.at(i) != pd_id.at(i)) {
            return false;
          }
        }
        return true;
      }
      else if (input_lane_ids.size() > pd_id.size()) {
        for (int i = pd_id.size() - 1; i >= 0; i--) {
          if (input_lane_ids.at(i) != pd_id.at(i)) {
            return false;
          }
        }
        return true;
      }
      return false;
    } else {
      if(input_lane_ids.size() < match_lane_num) return false;
      // 要求有match_lane_num个相同lane即可匹配
      int matched_num = 0;
      int start_match_idx = -1;
      for(int i = 0; i < pd_id.size(); i++) {
        if(pd_id[i] == input_lane_ids[0]) {
          start_match_idx = i;
          break;
        }
      }

      if(start_match_idx == -1) return false;
      if(pd_id.size() - start_match_idx < match_lane_num) return false;
      for(int i = 1 ; i < match_lane_num; i++){
        if(pd_id[i+start_match_idx] != input_lane_ids[i]) {
          return false;
        }
      }
      return true;
    }

  }
} PdObjectStruct;

typedef struct ObjectSTStruct {
  int id;
  vector<PdObjectStruct> pd_objs;
  ObjectSTStruct() {
    Reset();
  }
  void Reset() {
    id = 0;
    pd_objs.clear();
  }
} ObjectSTStruct;

typedef struct LineObject {// in referenceframe
  //add by ly
  bool is_v2v_obj = false;
  //
  int id;
  double x;
  double y;                                           
  double xabs;
  double yabs;
  double global_angle;
  double vxabs;
  double vyabs;
  double speed;
  std::vector<ObjectCell> cells;
  acu::common::math::Box2d cell_box;
  double height;
  double confidence;
  double age;
  
  LaneInfoConstPtr lane_ptr;
  double obj_lane_s;
  double obj_lane_l;
  string obj_lane_id;
  bool is_waiting;
  bool is_in_junction;
  double dis_to_junction;
  bool need_focus;
  int focus_counter;
  int key_focus;//0-no 1-lat 2-lon 3-lat+lon

  string relation_lane; // 与自车冲突或关联车道

  vector<ObjectHistory> history;
  std::queue<std::pair<double, double>> position;
  double acc;
  int acc_type;
  int static_counter;
  bool was_dynamic;
  bool was_waiting;
  bool is_static;
  bool last_is_static;
  bool is_reverse_traveling;
  bool last_is_reverse_traveling;
  bool is_moveble; //perception moving_status = MOVABLE;
  int reverse_traveling_counter;
  int dynamic_counter;
  int movement_counter;
  int driving_status;
  double pass_l;
  int type;
  int last_type;
  int type_counter;
  int disapear_counter;
  acu::common::math::Box2d box;

  StructSLBoundary sl_boundary;
  vector<Polygon2d> sl_polygons;
  double vsabs;// frenet坐标系下
  double vlabs;// frenet坐标系下
  bool is_in_line;
  double nearest_xg;
  double nearest_yg;

  PredictionObject prediction;
  ObjectSTStruct st_area;

  // 为兼容旧代码，对障碍物筛选
  StructSTBoundary st_boundary;// 不用stmap时，用这个
  int prediction_index;
  int conflict_type;
  int right_of_way;

  int parallel_counter;
  bool block_first_conflict;
  
  LineObject() {
    Reset();   
  }
  void Reset() {
    is_v2v_obj = false;
    id = 0;
    x = 0.0;
    y = 0.0;
    xabs = 0.0;
    yabs = 0.0;
    global_angle = 0.0;
    vxabs = 0.0;
    vyabs = 0.0;
    speed = 0.0;
    height = 0.0;
    cells.clear();
    cells.reserve(MAX_OBJ_CELL_SIZE);
    confidence = 0.0;
    age = 0.1;

    lane_ptr = nullptr;
    obj_lane_s = 1000.0;
    obj_lane_l = 1000.0;
    obj_lane_id = "";
    is_waiting = false;
    is_in_junction = false;
    dis_to_junction = 1000.0;

    need_focus = false;
    focus_counter = 0;
    key_focus = 0;
    relation_lane = "";
  
    history.clear();
    acc = 0.0;
    acc_type = 0;
    was_dynamic = false;
    is_static = true;
    was_waiting = false;
    is_reverse_traveling = false;
    is_moveble = false;
    last_is_reverse_traveling = false;
    static_counter = 0;
    type_counter = 0;
    dynamic_counter = 0;
    movement_counter = 0;
    driving_status = 0;
    reverse_traveling_counter = 0;
    disapear_counter = 0;
    parallel_counter = 0;
    type = (int)eObjectType::UNKNOWN;
    last_type = (int)eObjectType::UNKNOWN;
  
    sl_boundary.Reset();
    sl_polygons.clear();
    vsabs = 0.0;// frenet坐标系下
    vlabs = 0.0;// frenet坐标系下
    is_in_line = false;
    nearest_xg = 0.0;
    nearest_yg = 0.0;
    
    prediction.Reset();
    st_area.Reset();
    prediction_index = -1;
    conflict_type = 0;
    right_of_way = 0;
    block_first_conflict = false;
  }
  void SetData(const LineObject &input_obj, StructSLBoundary input_sl) {
    is_v2v_obj = input_obj.is_v2v_obj;
    id = input_obj.id;
    x = input_obj.x;
    y = input_obj.y;                                           
    xabs = input_obj.xabs;
    yabs = input_obj.yabs;
    global_angle = input_obj.global_angle;
    vxabs = input_obj.vxabs;
    vyabs = input_obj.vyabs;
    speed = input_obj.speed;
    cells = input_obj.cells;
    cell_box = input_obj.cell_box;
    height = input_obj.height;
    confidence = input_obj.confidence;
    age = input_obj.age; 
    
    lane_ptr = input_obj.lane_ptr;
    obj_lane_s = input_obj.obj_lane_s;
    obj_lane_l = input_obj.obj_lane_l;
    obj_lane_id = input_obj.obj_lane_id;
    is_waiting = input_obj.is_waiting;
    is_in_junction = input_obj.is_in_junction;
    dis_to_junction = input_obj.dis_to_junction;
    if (input_obj.need_focus) {
      need_focus = input_obj.need_focus;
      focus_counter = 0;
    }
    else if (!need_focus) {
      need_focus = input_obj.need_focus;
      focus_counter = 0;
    }
    else {
      focus_counter++;
    }
    if (focus_counter > 5) {
      need_focus = input_obj.need_focus;
      focus_counter = 0;
    }
    key_focus = input_obj.key_focus;
    relation_lane = input_obj.relation_lane;
    history = input_obj.history;
    acc = input_obj.acc;
    acc_type = input_obj.acc_type;
    is_static = input_obj.is_static;
    was_waiting = input_obj.was_waiting;
    is_reverse_traveling = input_obj.is_reverse_traveling;
    is_moveble = input_obj.is_moveble;
    static_counter = input_obj.static_counter;
    last_is_static = input_obj.last_is_static;
    last_is_reverse_traveling = input_obj.last_is_reverse_traveling;
    movement_counter = input_obj.movement_counter;
    driving_status = input_obj.driving_status;
    pass_l = input_obj.pass_l;
    reverse_traveling_counter = input_obj.reverse_traveling_counter;
    static_counter = input_obj.static_counter;
    was_dynamic = input_obj.was_dynamic;
    dynamic_counter = input_obj.dynamic_counter;
    type = input_obj.type;
    last_type = input_obj.last_type;
    type_counter = input_obj.type_counter;
    disapear_counter = input_obj.disapear_counter;
    box = input_obj.box;
  
    sl_boundary = input_sl;
    vsabs = input_obj.vsabs;// frenet坐标系下
    vlabs = input_obj.vlabs;// frenet坐标系下
    is_in_line = input_obj.is_in_line;
    nearest_xg = input_obj.nearest_xg;
    nearest_yg = input_obj.nearest_yg;

    prediction = input_obj.prediction;
    prediction_index = input_obj.prediction_index;
    conflict_type = input_obj.conflict_type;
    right_of_way = input_obj.right_of_way;
    block_first_conflict = input_obj.block_first_conflict;

    // parallel_counter = input_obj.parallel_counter;
  }
} LineObject;

typedef struct TrafficLight {
  eLightColor color;
  eLightState state;
  std::string id;
  double confidence;
  double tracking_time;
  int number;
  eLightType type; //
  //add by ly
  bool is_v2x_traffic_light;
  double red_period;
  double green_period;
  double yellow_period;
} TrafficLight;

struct PerceptionData {
  double timestamp;
  FaultStatus perception_status;
  std::vector<ObjectCell> cells;
  std::vector<CallbackObject> objects;
  std::vector<TrafficLight> traffic_lights;
  PerceptionData() {
    cells.reserve(MAX_STATIC_CELL_SIZE);
    objects.reserve(MAX_OBJ_SIZE);
    traffic_lights.reserve(MAX_LIGHTS_SIZE);
    cells.clear();
    objects.clear();
    traffic_lights.clear();
  }
  ~PerceptionData() {
    cells.clear();
    objects.clear();
    traffic_lights.clear();
  }
};

typedef struct LocPerception {
  double time;
  std::vector<LocalizationData> loc_data_list;
  std::vector<LioData> lio_data_list;
  LocalizationData localization_data;
  LocalizationData last_localization;
  LocalizationData last_correct_localization;
  LocalizationData correct_localization;
  PerceptionData perception_data;
  int new_imu_index;
  LocPerception() {
    localization_data.Reset();
    last_localization.Reset();
    last_correct_localization.Reset();
    correct_localization.Reset();
    loc_data_list.clear();
    new_imu_index = 0;
  }
} LocPerception;

} // namespace planning
} // namespace acu

#endif // DATAPOOL_INCLUDE_LOCPERCEPTION_INPUT_H_