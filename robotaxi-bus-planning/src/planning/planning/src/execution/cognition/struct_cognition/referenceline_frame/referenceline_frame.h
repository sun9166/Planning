#ifndef SRC_EXECUTION_COGNITION_STRUCT_COGNITION_REFERENCELINE_FRAME_H_
#define SRC_EXECUTION_COGNITION_STRUCT_COGNITION_REFERENCELINE_FRAME_H_

#include <algorithm>
#include "conf/cognition_gflags.h"
#include "public_typedef.h"
#include "mapengine_input.h"
#include "prediction_input.h"
#include "locperception_input.h"
#include "common_config.h"
#include "map_loader.h"
#include "st_map/st_map.h"
#include "base_func/base_func.h" 
#include "referenceline_frame/sl_boundary/sl_boundary.h"
#include "common/toolbox/geometry/include/geoheader.h"
//#include "common/base/monitor_api/include/monitor_api.h"
#include "common/common_define/error_code_define.h"
#include "planning_debug_msgs.pb.h"
	
using geometry::Site;
using geometry::SiteVec;
using namespace std;
using namespace acu::hdmap;
using namespace acu::vectormap;
using namespace acu::common;
using namespace acu::common::math;
using acu::map::BaseMap;

 // LaneBoundPoint contains: (s, l_min, l_max).
using LaneBoundPoint = std::tuple<double, double, double>;
// LaneBound contains a vector of PathBoundPoints.
using LaneBound = std::vector<LaneBoundPoint>;
// ObstacleEdge contains: (is_start_s, s, l_min, l_max, obstacle_id).
using ObstacleEdge = std::tuple<int, double, double, double, int>;

namespace acu{
namespace planning {

enum class Area {
	NONE = 0x00,
	SPEEDBUMP = 0x01,
	FORBIDAREA = 0x02,
	JUNCTION = 0x03,
	CROSSWALK = 0x04,
};

struct GapStruct {// 需要更新
	int start_id;// 超越的障碍物id 
	int end_id;// 追随的障碍物id
	double aim_min_s;// 超越此障碍物需要的最小s
	double aim_max_s;// 超越此障碍物但不越过前面障碍物的最大s
	double aim_min_v;// 达到上述区间需要的最小车速
	double aim_max_v;// 处于上述区间允许的最大车速
	double allow_min_v;// 换道过程允许的最小车速
	double allow_max_v;// 换道过程允许的最大车速
	double allow_min_t;// 预估到达gap的最小可行时间
	int safety_level;// 0-bad 1-good 2-great
	int feasibility_level;// 0-bad 1-good 2-great
	GapStruct() {
		start_id = -1;
		end_id = -1;
		aim_min_s = 1000.0;
		aim_max_s = 0.0;
		allow_min_v = 0.0;
		allow_max_v = 0.0;
		allow_min_t = 1000.0;
		safety_level = 0;
	}
};


class ReferenceLineFrame {
public:
	
	CarModel car_model_;
	Site loc_site_;
	double car_speed_;

	int reference_lane_id; // 后定义信息
	MapEngineLineList mapinfo; // 所有地图原始信息
	SiteVec last_path_points;

	LaneBound limit_space;
	bool temp_blocked;
	bool line_blocked;
	bool is_congestion;
	bool line_slow;
	bool line_safety;
	bool line_queue;
	bool meeting_car;
	int meeting_state;
	int meeting_id;
	int block_counter;
	int slow_counter;
	int safety_counter;
	int queue_counter;
	int meeting_counter;
	int block_id;
	int jam_level;
	double block_dis;	
	double line_speed;	
	double conflict_s;
	double safety_cost;
	double speed_cost;
	double side_cost;
	double average_v;
	double dis_to_last_lc;
	double short_term_speed;
	double long_term_speed;
	double plan_length;
	int plan_follow_id;

	std::pair<double, double> blind_dis;
	std::map<int, LineObject> objects_;
	std::map<int, double> block_l_;
	std::map<double, int> car_info_;
	std::map<int, int> object_on_line_;
	std::map<int, int> car_on_line_;// key-id, second-time
	std::map<int, int> invader_on_line_;
	std::map<double, bool> left_bds_;
	std::map<double, bool> right_bds_;
	std::vector<int> meeting_objects_;

	std::set<int> follow_objects_;
	std::set<int> park_objects_;
	vector<GapStruct> line_gap;
	STmap st_map;
	bool matched_history;
 
public:
	ReferenceLineFrame();
	~ReferenceLineFrame();
	void Reset();
	void ClearData();
	bool CalculationFrame(const MapEngineLineList &linelist, 
		 const LocalizationData &locpose, const CarModel &car_input); 
	bool CalculationFrame(const PathData &motion_result, 
		 const LocalizationData &locpose, const CarModel &car_input); 
	void SetData(const MapEngineLineList &linelist, 
				 const LocalizationData &locpose, 
				 const CarModel &car_input);
	void SetData(const ReferenceLineFrame &input_data);
	const vector<vector<STMapPointStruct> > GetSTMap() const {
		return st_map.map_data();
	}

	const STMapPointStruct* const* GetSTMapPtr() const {
		return st_map.map_data_ptr();
	}

	const int GetSTMapRangeT() const {
		return st_map.map_data_range_t();
	}

	const int GetSTMapRangeS() const {
		return st_map.map_data_range_s();
	}

	void CopyWithoutSTMap(const ReferenceLineFrame &input_data);
	// int &GetSTAge() { return st_map.age();}

	/******************************** api from line path *****************************/
	bool GetNearestPoint(const double &s, Site &output_p, int &index) const;
	bool GetNearestPoint(const Site &input_p, Site &output_p, double &min_dis, int &min_index);
	bool GetGlobalNearestPoint(const Site &input_p, Site &output_p, double &s, 
								double &min_dis, int &min_index) const;
	bool GetGlobalNearestPointwithHeading(const Site &input_p, Site &output_p, 
					double &s, double &min_dis, int &min_index) const;
	bool GetReferencePoints(const double &start_s, const double &end_s, SiteVec &output);
	bool GetReferencePoints(const Site &start_p, const Site &end_p, SiteVec &output);
	bool GetLaneType(int &type, double s = 0);
	bool GetLaneType(int &type, const Site &input_p);
	bool GetLaneBoundaryType(int &l_bd_type, int &r_bd_type, double s = 0) const;
	bool GetLaneBoundaryType(int &l_bd_type, int &r_bd_type, const Site &input_p) const;
	bool GetWidthToLaneBoundary(double &left_w, double &right_w, double s = 0) const;
	bool GetWidthToLaneBoundary(double &left_w, double &right_w, const Site &input_p) const;
	// road == false witdh to freespace // xn
	bool GetWidthToRoadBoundary(double &left_w, double &right_w, double s = 0, bool road = true) const;
	bool GetWidthToRoadBoundary(double &left_w, double &right_w, const Site &input_p, bool road = true) const;
	bool GetWidthToFreeSpace(double &left_w, double &right_w, const double& s) const;
	bool IsInFreeSpace(const PointGCCS& target_point) const;

	bool IsInLine(const string &lane_id);
	bool IsInLine(const Site &input_p);
	bool DisToLine(double &distance, const Site &input_p);
	bool DisToEnd(double &distance, double s = 0);
	bool DisToEnd(double &distance, const Site &input_p);
	bool DisToSpecialArea(double &distance, const Area &type, double s = 0);
	bool DisToSpecialArea(double &distance, const Area &type, const Site &input_p);
	bool GetSFromIndex(const int &index, double &s);
	bool GetIndexFromS(const double &s, int &index);
	bool IsRoadSide();
	double Length() const;
	bool IsUturn(std::pair<double, double>& u_turn_s_interval) const; 
	bool GetSucJunctionRoadId(vector<string> &suc_junction_roads) const;
	bool GetBoundaryXY(const double s,bool left, double& xg,double& yg);
	/******************************** api *****************************/

private:
	
	MapEngineLineList lane_lists_;
	SiteVec motion_path_;
	const acu::vectormap::VectorMap* vector_map_;
	const BaseMap* basemap_;


};


}
}

#endif
