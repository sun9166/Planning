#ifndef SRC_EXECUTION_COGNITION_STRUCT_COGNITION_STMAP_H_
#define SRC_EXECUTION_COGNITION_STRUCT_COGNITION_STMAP_H_

#include <stdio.h>
#include <omp.h>
#include <math.h>
#include "vectormap.h"
#include "locperception_input.h"
#include "mapengine_input.h"
#include "st_discretization/st_discretization.h"
//#include "planning_debug_msgs/DebugDecision.h"
//#include "planning_debug_msgs/STGraph.h"
#include "planning_debug_msgs.pb.h"
#include "st_common/st_common.h"

using namespace std;

namespace acu{
namespace planning {

struct STPointPdLineStruct {
	vector<string> pd_id;
	double p;// 每条障碍物预测线对应在每个stpoint的概率
	vector<pair<double, double>> a_p;
	double average_a() {
    double num = 0.0;
    if (a_p.empty()) return num;
    for (auto &ai : a_p) {
      num += ai.first;
    }
    return num / (double)(a_p.size());
  }
};

struct STPointObjectStruct {
	int id;
	double p;// 每个障碍物对应在每个stpoint的概率
	double speed;
	// vector<STPointPdLineStruct> pd_objs;
	STPointObjectStruct () {
		Reset();
		// pd_objs.reserve(4);
	}
	void Reset() {
		id = 0;
		p = 0.0;
		speed = 0.0;

		// pd_objs.clear();
	}
};

struct STMapPointStruct {
	ST index;
	double p;
	std::map<int, STPointObjectStruct> objs;
	STMapPointStruct() {
		Reset();
	}
	void Reset() {
		p = 0.0;
		objs.clear();
	}
	void Add(const STPointObjectStruct &point_obj, ST &pd_point) {
		if (objs.count(point_obj.id)) {// 同一障碍物不同预测线占据相同st时，概率叠加
			objs.at(point_obj.id).p += point_obj.p;
			objs.at(point_obj.id).p = fmin(objs.at(point_obj.id).p, 1.0);
			// objs.at(point_obj.id).pd_objs.reserve(4);
			// objs.at(point_obj.id).pd_objs.insert(objs.at(point_obj.id).pd_objs.end(),
			// 					  point_obj.pd_objs.begin(), point_obj.pd_objs.end());
			pd_point.p = fmax(pd_point.p, objs.at(point_obj.id).p);
		}
		else {
			objs[point_obj.id] = point_obj;
		}
		p = 0.0;
		for (auto &it : objs) {// 不同障碍物取最大概率作为栅格概率
			if (p < it.second.p) {
				p = it.second.p;
			}
		}
	}

};

struct TpointsStruct {
	int t;
	vector<STMapPointStruct*> t_points;
	pair<int, int> occupied_range;
	pair<int, int> solid_occupied_range;
	TpointsStruct() {
		t = 0;
		t_points.clear();
		occupied_range.first = -1;
		occupied_range.second = -1;
		solid_occupied_range.first = -1;
		solid_occupied_range.second = -1;
	}
};

class STmap {

public:
	STmap();
	~STmap();

	STmap(const STmap& st_map);
	void operator=(const STmap& st_map);

	void Reset();
	void STmapReset();
	bool InitSTmap(MapEngineLineList *line_data_ptr, 
				   const SiteVec *last_points_ptr, 
				   const int &reference_lane_id,
				   const Site *locpose, 
				   const CarModel *car_input);
	bool CalculateObjectsST(std::map<int, LineObject> &line_objs);
	void BuildSTMap(std::map<int, LineObject> &line_objs);
	bool GetPredictionSTBoundary(LineObject &line_obj, vector<BoxStruct> &line_boxes);
	/* ----- api ----- */
	const vector<vector<STMapPointStruct> > map_data() const {
		vector<vector<STMapPointStruct> > map_data;
		if(current_st_map_ == nullptr) {
			return map_data;
		}
		for(int i = 0; i < range_t_; i++) {
			vector<STMapPointStruct> t_slice;
			for(int j = 0; j < range_s_; j++) {
				t_slice.push_back(current_st_map_[i][j]);
			}
			map_data.push_back(t_slice);
		}
		return map_data;
	}

	const STMapPointStruct* const* map_data_ptr() const{
		return current_st_map_;
	}

	const int map_data_range_t() const {
		return range_t_;
	}

	const int map_data_range_s() const{
		return range_s_;
	}

	void GetTMap();
	bool GetTPoints(const int t, vector<STMapPointStruct> &t_points) const; 

	void SetCompensate_T(const int& in_t){
		// compensate_t_ = (in_t > 1e-5) ? in_t : 0;//要求为正值
		compensate_t_ = in_t;
	}
	void SetCompensate_S(const int& in_s){
		// compensate_s_ = (in_s > 1e-5) ? in_s : 0;//要求为正值
		compensate_s_ = in_s;
	}
	bool GetCompensateTPoints(const int t, vector<STMapPointStruct> &t_points) const; 
	bool GetCompensateTPoints(const int t, vector<STMapPointStruct> &t_points, 
								int input_compensate_t, int input_compensate_s) const; 
	bool GetAccumulateCompensateTPoints(const int t, vector<STMapPointStruct> &t_points, 
								int input_compensate_t, int input_compensate_s) const; 
	bool GetCost(const ST &point, double &v, double &cost);
	void ClearObjInSTMap(int &object_id);
	void SetObjectToSTMap(LineObject &object);
	void ExtrusionObjSTArea(LineObject &line_obj, double &input_a, 
													double min_a = -100.0, double max_a = -100.0);
	void DecisionObjProbability(LineObject &line_obj, double &p); 


private:
	bool CalculateDeltaST();
	void UpdateObjectSTArea(LineObject &obj);
	void ReduceDisappearedPdObj(LineObject &obj);
	bool CalculateMapProbability(std::map<int, LineObject> &line_objs);
	void LimitFollowingST(std::map<int, LineObject> &line_objs);
	void MapProbabilityDamping();
	bool CarInPdLanes(vector<string> &pd_lanes);
	bool DevideRelation(vector<string> &pd_lanes);
	bool NearToRoadBoundary();

private:
	const acu::vectormap::VectorMap* vector_map_;
	vector<TpointsStruct> t_map_;
	const MapEngineLineList *line_data_ptr_;
	const SiteVec *last_points_ptr_;
	const CarModel *car_model_ptr_;
	int reference_lane_id_;
	const Site *loc_ptr_;
	double car_speed_;
	STDdiscret st_discret_;
	STCommon st_common_;
	int compensate_t_;//分辨率0.2s
	int compensate_s_;//分辨率0.5m

	int range_s_;
	int range_t_;

	STMapPointStruct** current_st_map_ = nullptr;
	int update_s_;
	int update_t_;
};


}
}

#endif