#include "vectormap.h"
#include "public_typedef.h"
#include "conf/cognition_gflags.h"
#include "datapool/include/data_pool.h"
//#include "common/base/monitor_api/include/monitor_api.h"
#include "common/common_define/error_code_define.h"
#include "map/vectormap/src/hdmap/hdmap.h"
#include "base_func/base_func.h"


using namespace acu::common;
using namespace acu::hdmap;
using namespace acu::common::math;

#define DATA_LENGTH 5
#define POSITION_LENGTH 20

namespace acu{
namespace planning {


class ObjectProcess {
 public: 
	ObjectProcess();
	void CellsToObjects(PerceptionData &perception_data);
	void CompleteObjectInfo(vector<CallbackObject>& objects, PredictionData &prediction, 
							std::map<int, LineObject> &lines_objects);

	double GetNearestMapPointHeading(Site obj);

 private:
 	void UpdateObject(CallbackObject &per_obj, LineObject &line_obj);
	void UpdatePerceptionInfo(CallbackObject &per_obj, LineObject &line_obj);
	void UpdateMapAPIInfo(CallbackObject &per_obj, LineObject &line_obj); 
	void UpdateHistoryInfo(CallbackObject &per_obj, LineObject &line_obj);
	bool IsInJunction(CallbackObject &per_obj);
	double Dis2Junction(CallbackObject &per_obj, LaneInfoConstPtr &nearest_lane, double &s);
	bool GetJunctionType(const LineObject& object);
	int GetType(CallbackObject &per_obj, LineObject& line_obj);
	int GetAccType(LineObject& line_obj);
	bool GetMovementType(LineObject& line_obj);
	bool GetIsReverseTraveling(LineObject& line_obj);
	bool GetDynamicType(LineObject& object);
	bool GetNearestLaneFromLocpos(const double xg, const double yg,
  						LaneInfoConstPtr &nearest_lane, double &s, double &l);
	

	const acu::vectormap::VectorMap* vector_map_;
 	const acu::hdmap::HDMapImpl* hdmap_;
 	acu::vectormap::GeoTool geotool_;
 	vector<PredictionObject>* prediction_ptr_;
 	bool prediction_valid_;
};

}
}