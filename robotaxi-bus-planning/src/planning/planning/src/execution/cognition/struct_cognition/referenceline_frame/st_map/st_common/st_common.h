#ifndef STRUCT_COGNITION_ST_COMMON_H_
#define STRUCT_COGNITION_ST_COMMON_H_

// #include "common/base/log/include/log.h"
#include "conf/cognition_gflags.h" 
#include "locperception_input.h"
#include "mapengine_input.h"
#include "common_config.h"
#include "map/map_loader/include/map_loader.h"
#include "base_func/base_func.h"

using namespace acu::common;
using namespace std;
using geometry::Site;

namespace acu{
namespace planning {


struct BoxStruct {
	Box2d box;
	double length;
	double t;
	BoxStruct() {
		length = 0.0;
		t = 0.0;
	}
	BoxStruct(const Box2d &input_box, 
			  const double &input_length, 
			  const double &input_t) {
		box = input_box;
		length = input_length;
		t = input_t;
	}
};


class STCommon {

public:
	STCommon();
	~STCommon() {}
	void GetLineBoxes(const CarModel *car_model_ptr, const SiteVec &path_points, 
					  				vector<BoxStruct> &line_boxes);
	void GetPdBoxes(const PredictionTrajectory &pd_trajectory, const double &obj_length, 
									const double &obj_width, const bool &is_reverse_traveling,
									vector<BoxStruct> &pd_boxes);
	void BoxST(const vector<BoxStruct> &pd_boxes, const vector<BoxStruct> &line_boxes,
				vector<double> &range_pd_s, vector<pair<Vec2d, Vec2d>> &st_boundary);
	void RoundST(PredictionTrajectory &pd_trajectory);
	void RoundST(vector<pair<Vec2d, Vec2d>> &st_boundary, vector<ST> &st_points, double p);

	void SegmentLineFit(vector<pair<Vec2d, Vec2d>> &st_boundary, vector<double> &pd_s);
	void FindBreakIndex(vector<double> &list_t, vector<double> &list_s, 
											int final_index, vector<int> &seg_indexes);
	void STBoundaryToLine(pair<Vec2d, Vec2d> &start_bd, pair<Vec2d, Vec2d> &end_pd, 
												double &up_bd_k, double &up_bd_b, double &down_bd_k, 
												double &down_bd_b, int &front_t, int &back_t);
};


}
}

#endif