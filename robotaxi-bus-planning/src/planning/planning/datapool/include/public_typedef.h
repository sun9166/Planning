#ifndef DATAPOOL_INCLUDE_PUBLIC_TYPEDEF_H_
#define DATAPOOL_INCLUDE_PUBLIC_TYPEDEF_H_
#include <map>
#include <list>
#include <queue>
#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include "node_time.h"
#include <chrono>
#include "common/toolbox/geometry/include/geoheader.h"

namespace acu {
namespace planning {

#define INIT_TIMES 3

#define MAX_STATIC_CELL_SIZE 2000
#define MAX_OBJ_CELL_SIZE 500
#define MAX_OBJ_SIZE 100
#define MAX_LIGHTS_SIZE 50

typedef unsigned char uint8;
typedef char int8;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned int uint32;
typedef int int32;

using CollisionInfoVec = std::vector<std::tuple<int, geometry_msgs::Point32>>;

enum ePlanningThread {
  COMPUTE_THREAD_ID = 0,
  ADJUST_THREAD_ID = 1,
  COGNITION_THREAD_ID = 2,
  BEHAVIORPLAN_THREAD_ID = 3,
  PATHPLAN_THREAD_ID = 4,
  SPEEDPLAN_THREAD_ID = 5
};

enum class eComputeThreadType {
  DEFAULT = 0,    /*!< default is zero */
  GLOBAL_PATH,
  PLANNER_DWA_AVOID,
  PLANNER_DWA_FORCE_AVOID,
  PLANNER_ASTAR_AVOID,
  PLANNER_DP_AVOID,
  PLANNER_ASTAR_AVOID_FINAL,
  PLANNER_ASTAR_ADJUST,
  BORDER_DWA_AVOID
};

enum class eComputeResult {
  OTHER = -2,
  ERROR = -1,
  SUCCESS = 0,
  BUSY = 1
};

typedef struct ComputeThreadType {
  eComputeThreadType type;
  std::string str_type;
  ComputeThreadType() {
    type = eComputeThreadType::DEFAULT;
    str_type = "DEFAULT";
  }
  void Reset() {
    type = eComputeThreadType::DEFAULT;
    str_type = "DEFAULT";
  }

  void SetGlobalPath() {
    type = eComputeThreadType::GLOBAL_PATH;
    str_type = "global path";
  }

  void SetPlannerDwaAvoid() {
    type = eComputeThreadType::PLANNER_DWA_AVOID;
    str_type = "dwa avoid";
  }

  void SetPlannerDwaForceAvoid() {
    type = eComputeThreadType::PLANNER_DWA_FORCE_AVOID;
    str_type = "dwa force avoid";
  }

  void SetPlannerAstarAvoid() {
    type = eComputeThreadType::PLANNER_ASTAR_AVOID;
    str_type = "astar avoid";
  }

  void SetPlannerDPAvoid() {
    type = eComputeThreadType::PLANNER_DP_AVOID;
    str_type = "DP avoid";
  }

  void SetPlannerAstarAvoidFinal() {
    type = eComputeThreadType::PLANNER_ASTAR_AVOID_FINAL;
    str_type = "astar avoid final";
  }

  void SetPlannerAstarAdjust() {
    type = eComputeThreadType::PLANNER_ASTAR_ADJUST;
    str_type = "astar adjust";
  }

  void SetPlannerBorderDwa() {
    type = eComputeThreadType::BORDER_DWA_AVOID;
    str_type = "BorderDwa";
  }
} ComputeThreadType;

enum class eSystemStatus {
  DEFAULT = 0,
  FAULT_LEVEL_ONE = 1,
  FAULT_LEVEL_TWO = 2
};

enum class eExceptionHandleRes {
  DEFAULT = 0,
  EXECUTABLE,
  UNEXECUTABLE,
  PERSERVE
};

typedef struct NodeFaultMsg
{
  std::string info;
  int level;
  int is_preserve;
  int fault_type;
} NodeFaultMsg;

typedef struct ExceptionAnalyseResult {
  eExceptionHandleRes status;
  std::vector<NodeFaultMsg>  node_fault_vec;
  std::string info;
  ExceptionAnalyseResult() {
    status = eExceptionHandleRes::DEFAULT;
    node_fault_vec.reserve(10);
  }

  void ClearAllNodeFaultVec() {
    node_fault_vec.clear();
  }

  void AddNodeFault(std::string _info, int _level, int _fault_type, int _is_preserve = 1) {
    for (int i = 0; i < node_fault_vec.size(); i++) {
      if (node_fault_vec[i].fault_type == _fault_type) {
        return ;
      }
    }
    NodeFaultMsg node_fault_msg;
    node_fault_msg.info = _info;
    node_fault_msg.level = _level;
    node_fault_msg.fault_type = _fault_type;
    node_fault_msg.is_preserve = _is_preserve;
    node_fault_vec.push_back(node_fault_msg);
  }

  bool IsFaultExist() {
    return node_fault_vec.size() > 0;
  }
  void print() {
    for (int i = 0; i < node_fault_vec.size(); i++) {
      // AINFO << "node_fault_vec: " << node_fault_vec[i].info << "," <<  node_fault_vec[i].level;
    }
  }

  void RemoveParticular(int fault_level) {
    auto it = node_fault_vec.begin();
    for (; it != node_fault_vec.end(); ) {
      if (it->fault_type == fault_level ) {
        it = node_fault_vec.erase(it);
      } else {
        ++it;
      }
    }
  }

  void ClearNonPreserve() {
    auto it = node_fault_vec.begin();
    for (; it != node_fault_vec.end(); ) {
      if (it->is_preserve == 0 ) {
        it = node_fault_vec.erase(it);
      } else {
        ++it;
      }

    }
  }
} ExceptionAnalyseResult;

/**
* @brief define unstruct path data struct , for globle path  , refrence path , local path
*/
struct PathData {
  double time_stamp; //@pqg add for struct map /*!< the time for get the path data, the unit is second*/
  bool is_new;  
  int path_property;  /*!<  1 local  2 DR  default : -1 */
  std::string path_type;  /*!<  1 for output   "welt"or"normal" */ // "straight"or "UTurn" for car 
  int index;
  //todo
  std::list<geometry::Site> path;    /*!< define the points in path data*/
  float left_restrict;
  float right_restrict;

  int current_id;  //@pqg add 当前参考线编号
  int target_id;   //目标参考线编号
  std::vector<std::string> lane_ids; 
  bool is_blocked ;        //规划轨迹是否堵塞
  double control_accuracy;
  int senario_type;                 // 0: normal, 1:uncertain
  double steeringangle_rate_max ;   // steeringangle_rate limit for latcontrol

  PathData() {
    time_stamp = 0.0;
    is_new = false;
    path_property = -1;
    index = -999;
    path_type = "normal";
    path.clear();
    left_restrict = std::numeric_limits<double>::max();
    right_restrict = std::numeric_limits<double>::max();
    is_blocked = false;
    control_accuracy = 1.0;
    senario_type = 0;             
    steeringangle_rate_max = 500;
  }

  ~PathData() {
    time_stamp = 0.0;
    is_new = false;
    path_property = -1;
    index = -999;
    path_type = "normal";
    path.clear();
    left_restrict = std::numeric_limits<double>::max();
    right_restrict = std::numeric_limits<double>::max();
    is_blocked = false;
    control_accuracy = 1.0;
    senario_type = 0;             
    steeringangle_rate_max = 500;
  }
  void Reset() {
    time_stamp = 0.0;
    is_new = false;
    path_property = -1;
    index = -999;
    path.clear();
    path_type = "normal";
    left_restrict = std::numeric_limits<double>::max();
    right_restrict = std::numeric_limits<double>::max();
    is_blocked = false;
    control_accuracy = 1.0;
    senario_type = 0;             
    steeringangle_rate_max = 500;
  }
  void clear() {
    path.clear();
    left_restrict = std::numeric_limits<double>::max();
    right_restrict = std::numeric_limits<double>::max();
  }
  // void operator=(const std::vector<geometry::Site> temp_path) {
  //   path.clear();
  //   path.insert(path.end(), temp_path.begin(), temp_path.end());
  // }

  bool operator==(PathData& a) {
    if (a.path.size() != path.size())
      return false;
    auto origin = path.begin();
    auto target = a.path.begin();
    for (int i = 0; i < path.size(); i++) {
      if ((*origin) == (*target)) {
        origin = std::next(origin, 1);
        target = std::next(target, 1);
      } else {
        return false;
      }
    }
    left_restrict = a.left_restrict;
    right_restrict = a.right_restrict;
    return true;
  }


  double GetRestLength() {
    double sum_dis = 0.0;
    for (auto it = path.begin(); std::next(it, 1) != path.end() &&
         it != path.end(); ++it) {
      sum_dis += std::hypot(std::next(it, 1)->x - it->x,
                            std::next(it, 1)->y - it->y);
    }
    return sum_dis;
  }

  geometry::Site GetIndex(int i) {
    if (i >= path.size()) {
      return *(path.begin());
    }
    int count  = 0;
    for (auto &p : path) {
      if (i == count) return p;
      count++;
    }
    return *(path.begin());
  }

};

typedef struct StatisticsDetailTimes
{
  int setting_times;
  double setting_rate;
  double setting_negated_rate;
  double now_rate;
  double negated_rate;
  std::list<uint8> data_detail;
  StatisticsDetailTimes() {
    setting_times = 0;
    setting_rate = 0.0;
  }
  void Reset() {
    data_detail.clear();
  }

  bool SetStatisticsTimes(int times, double rate, double _negated_rate = 0.9) {
    data_detail.clear();
    if (times <= 0) return false;
    if (rate > 1 || rate <= 0) return false;
    setting_times = times;
    setting_rate = rate;
    setting_negated_rate = _negated_rate;
    return true;
  }

  void NewPositiveData() {
    data_detail.push_back(1);
    if (data_detail.size() > setting_times) {
      data_detail.erase(data_detail.begin());
    }
  }
  void NewNegativeData() {
    data_detail.push_back(0);
    if (data_detail.size() > setting_times) {
      data_detail.erase(data_detail.begin());
    }
  }

  std::string ToString() {
    int count = 0, negtive_count = 0;
    for (auto &p : data_detail) {
      if (!p) negtive_count++;
      else count++;
    }
    negated_rate = (double)(negtive_count) / (double )setting_times;
    now_rate = (double)(count) / (double )setting_times;
    return (std::to_string(count) + "," +
            std::to_string(negtive_count) + "," +
            std::to_string(now_rate) + "," +
            std::to_string(negated_rate));
  }

  bool IsNoTimesUp() {
    if (data_detail.size() != setting_times) return false;
    int count = 0;
    for (auto &p : data_detail) {
      if (!p) count++;
    }
    negated_rate = (double)(count) / (double )setting_times;
    if (negated_rate > setting_negated_rate) {
      return true;
    }
    return false;
  }

  bool IsTimesUp() {
    if (data_detail.size() != setting_times) return false;
    int count = 0;
    for (auto &p : data_detail) {
      if (p) count++;
    }
    now_rate = (double)(count) / (double )setting_times;
    if (now_rate > setting_rate) {
      return true;
    }
    return false;
  }

  int PositiveCount()
  {
    int totalcount = 0;
    for (const auto& c : data_detail)
    {
      totalcount += c;
    }
    return totalcount;
  }
  int NegtiveCount()
  {
    int totalcount = 0;
    for (const auto& c : data_detail)
    {
      totalcount += c;
    }
    return (setting_times - totalcount);
  }
} StatisticsDetailTimes;

typedef struct StatisticsTimes {
  int setting_times;
  int current_times;
  StatisticsTimes() {
    setting_times = 0;
    current_times = 0;
  }
  void Reset() {
    current_times = 0;
  }

  void SetStatisticsTimes(int times) {
    Reset();
    setting_times = times;
  }

  bool NewTimes() {
    current_times++;
    if (current_times > setting_times) {
      return true;
    }
    return false;
  }

  bool IsTimesUp() {
    if (current_times > setting_times) {
      return true;
    }
    return false;
  }

  float StayedTime() {
    return current_times * 0.1 / 60.0; //min
  }

  std::string ToString() {

    return (std::to_string(current_times) + "," +
            std::to_string(setting_times));
  }
} StatisticsTimes;


struct Paths {
  PathData front_local_path;
  PathData back_local_path;
  PathData front_pursuit_path;
  PathData front_local_path_raw;  /*!< for the reference path data */

  Paths() {
    Reset();
  }
  void Reset() {
    front_local_path.Reset();
    back_local_path.Reset();
    front_pursuit_path.Reset();
    front_local_path_raw.Reset();
  }
};

struct FaultStatus {
  double time;
  int valid_count;
  bool is_init;
    bool is_valid;

  FaultStatus() {
    Reset();
  }
  void Reset() {
    time = 0;
    valid_count = 0;
    is_init = false; 
    is_valid = true;
  }
  bool Recover() {
    if (is_valid) {
      return true;
    }
    valid_count++; 
    if (valid_count > INIT_TIMES) {
      valid_count = 0;
      is_valid = true;
      return true;
    }
    return false;
  }
  void CheckInit(const double &msg_time) {
    bool sim_flag = false;
    auto now = std::chrono::system_clock::now();
    auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch());
    double  time_stamp = double(microseconds.count())/1000000;
    time = sim_flag?  time_stamp : msg_time;
    if (!is_init) {
      valid_count++;
      if (valid_count > INIT_TIMES) {
        is_init = true;
        valid_count = 0;
      }
    }
  }
};

} 
}

#endif // DATAPOOL_INCLUDE_PUBLIC_TYPEDEF_H_