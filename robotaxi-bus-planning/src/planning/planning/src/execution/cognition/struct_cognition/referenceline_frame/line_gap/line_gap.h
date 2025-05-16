
#ifndef SRC_EXECUTION_COGNITION_GAP_H_
#define SRC_EXECUTION_COGNITION_GAP_H_

#include "../referenceline_frame.h"

namespace acu{
namespace planning {


struct GapObject {
	int id;
	double min_s;
	double max_s;
	double mid_s;
	bool is_static;
	double speed;
	double acc;
	double passable_l;
	double start_t;
	GapObject() {
		Reset();
	}
	GapObject(LineObject &object, double pass_l, double t = 0.0) {
		id = object.id;
		min_s = object.sl_boundary.min_s;
		max_s = object.sl_boundary.max_s;
		mid_s = (min_s + max_s) / 2.0;
		is_static = object.is_static;
		speed = object.speed;
		acc = object.acc;
		passable_l = pass_l;
		start_t = t;
	}
	void Reset() {
		id = -1;
		min_s = 1000.0;
		max_s = 1000.0;
		mid_s = 1000.0;
		is_static = false;
		speed = 0.0;
		acc = 0.0;
		passable_l = 0.0;
		start_t = 0.0;
	}
	void Set(const int _id, const double _min_s,
			 const double _max_s, const double _mid_s,
			 const bool _is_static, const double _speed, 
			 const double _acc, const double _pass_l) {
		id = _id;
		min_s = _min_s;
		max_s = _max_s;
		mid_s = _mid_s;
		is_static = _is_static;
		speed = _speed;
		acc = _acc;
		passable_l = _pass_l;
	}
};

struct CarRange_t {// 时刻t自车可到达的位置区间
	double t;
	double min_v;
	double max_v;
	double min_s;
	double max_s;
	vector<GapObject> future_objs;
};

struct GapParam {
	double max_speed;
	double max_prelc_time;// 最长准备时间
	double lc_time;
	double total_time;
	double ttc;
	double better_ttc;
	double thw;
	double self_acc;
	// double obj_acc;
	double speed_percent;
	double front_buff;// 自车前向最小安全距离
	double back_buff;// 自车后向最小安全距离
	GapParam() {
		Reset();
	}
	void Reset() {
		max_speed = 20.0;
		max_prelc_time = FLAGS_pre_lc_t;
		lc_time = 4.0;
		total_time = max_prelc_time + lc_time + 1.0;
		ttc = FLAGS_min_ttc;
		thw = FLAGS_min_thw;
		self_acc = FLAGS_self_acc;
		// obj_acc = FLAGS_obj_acc;
		speed_percent = FLAGS_min_lc_speedtime;
		front_buff = FLAGS_front_buff;
		back_buff = FLAGS_back_buff;
	}
	void AdjustJam(int level) {
		ttc -= 1.0 * level;
		thw -= 0.1 * level;
		// obj_acc += 0.5 * level;
		speed_percent -= 0.1 * level;
		front_buff -= 1.0 * level;
		back_buff -= 0.5 * level;
	}
};

class LineGap {
 public:
	void AddGapInfo(ReferenceLineFrame& reference_line,
                    const LocalizationData &locpose,
                    const SpeedplanConfig &speed_config);
	

private:
	void FindGapObstacles();
 	void SortLineObstacles();
	void SetBoundGapobjects();
	bool IsCutInObject(LineObject &object, double &t);
    void SimCarS_t();
 	bool FrontObstacleLimit(double &front_obj_speed, double &front_obj_s);
 	bool StationLimit(double &max_v, double start_s = 0.0);
 	void SimFutureSortedObstacles(double t, vector<GapObject> &future_objs);
 	void GapEvaluation();
 	void LineGapInit();
	bool IfArriveGap(CarRange_t &car_range, GapStruct &gap_range, 
					 vector<GapObject> &t_future_objs, int i);
	bool IfLCIntoGap(CarRange_t &car_range, GapStruct &gap_range, 
	                 vector<GapObject> &t_future_objs, int i);
	void LCOverSafeLevel(CarRange_t &car_range, GapStruct &gap_range, 
                         vector<GapObject> &t_future_objs, int i);
 	double VT(double& init_v, double &t, double a = FLAGS_upper_acc);
 	void NeedUpperSpeedLimit(double &allow_max_v, double &allow_max_s);

 private:
 	ReferenceLineFrame* input_line_ptr_;
 	const CarModel *car_model_ptr_;
 	const SpeedplanConfig *speed_config_ptr_;
 	std::map<int, LineObject>* objects_ptr_;
	double min_safety_dis_ = 0.0;
 	double car_speed_;
 	double max_speed_;
	vector<GapObject> sorted_objects_;
 	vector<CarRange_t> car_ranges_;
 	GapParam gap_param_;
};


}
}

#endif