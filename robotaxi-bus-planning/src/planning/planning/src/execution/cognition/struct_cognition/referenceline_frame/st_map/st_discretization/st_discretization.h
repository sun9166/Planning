#ifndef STRUCT_COGNITION_ST_DISCRET_H_
#define STRUCT_COGNITION_ST_DISCRET_H_

#include "common/base/log/include/log.h"
#include "conf/cognition_gflags.h" 
#include "locperception_input.h"
#include "mapengine_input.h"
#include "common_config.h"
#include "vectormap.h"
#include "referenceline_frame/st_map/st_common/st_common.h"
#include "base_func/base_func.h"

using namespace acu::common;
using namespace acu::hdmap;
using namespace acu::common::math;
using namespace std;
using geometry::Site;


namespace acu{
namespace planning {


class STDdiscret {

public:
	STDdiscret();
	~STDdiscret() {}

	void Londiscret(PredictionTrajectory &pd_line,
					double input_a = -100.0, double min_a = -100.0,
					double max_a = -100.0, bool is_following = false);
	void Latdiscret(const MapEngineLineList *line_data_ptr, double &min_l, 
					PredictionTrajectory &pd_line, int type);

private:
	bool AccStBoundary(PredictionTrajectory &pd_line, double a, 
					   vector<pair<Vec2d, Vec2d>> &sim_st_boundary, bool is_following);
	bool FuncSA_T(double s, double a, double &t);
	void OverlapAccStBoundary(LocalSTArea &input_area);
	void Paralleldiscret(const MapEngineLineList *line_data_ptr, 
						 PredictionTrajectory &pd_line, double &p);

private:
	STCommon st_common_;
	
private:
	double object_v_;
	double object_a_;
	vector<vector<ST> > obj_st_points_;
	int range_s_;
	int range_t_;
	int area_min_s_;
	int area_max_s_;

};


}
}

#endif