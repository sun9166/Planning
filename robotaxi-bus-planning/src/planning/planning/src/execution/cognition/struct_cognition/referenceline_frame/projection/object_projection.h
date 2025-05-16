#include "datapool/include/data_pool.h"
#include "referenceline_frame/frame_base/frame_base.h"
#include "filter_focus/filter_focus.h"

using namespace std;

namespace acu{
namespace planning {

class ObjectProjection {
 public:
	ObjectProjection(ReferenceLineFrame& reference_line,
					 const std::map<int, LineObject> &all_objects, 
					 const double &local_path_length,
					 const LocalizationData &locpose, 
					 const CarModel &car_input);
	~ObjectProjection();
	void AddObjectToFrame();
	void AddFrameObjectInfo();

 private:
 	const std::map<int, LineObject>* objects_ptr_;
 	ReferenceLineFrame* reference_line_ptr_;
 	const CarModel *car_model_ptr_;
	const LocalizationData *loc_ptr_;
 	const vectormap::VectorMap* vectormap_;
 	bool has_target_;
 	vector<SortedObstacle> lon_focus_objs_, lat_focus_objs_;
 	double local_path_length_;

 private:
 	void ExpandPedestrian(LineObject& object);
 	void FindOnLinePedestrian();
 	void FindOnLineCar();
 	void FindOnLineInvader();
 	void AddSemanticInfo(LineObject& object);
	int GetConflictType(PredictionTrajectory &prediction);
	int GetRightOfWay(const LineObject& object, const PredictionTrajectory &prediction, 
					  PdObjectStruct &pd_obj);
	int GetRightOfWay(LineObject& object, const PredictionTrajectory &prediction);
	int JunctionRightOfWay(const LineObject& object, const PredictionTrajectory &prediction, 
						   PdObjectStruct &pd_obj);
	int GetJunctionType(const std::vector<string>& lane_ids);
	bool IsParallelConflict(PredictionTrajectory &prediction, int &index);
	void AddFirstConflictBlockInfo(LineObject& object);

	//add by ly 
	void PredictionTrajectorysCutOff(const MapEngineLineList &map_info, LineObject &temp_obj);
};


}
}