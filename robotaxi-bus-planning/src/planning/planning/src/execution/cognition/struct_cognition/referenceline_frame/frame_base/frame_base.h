#ifndef SRC_EXECUTION_COGNITION_FRAME_BASE_H_
#define SRC_EXECUTION_COGNITION_FRAME_BASE_H_

#include "conf/cognition_gflags.h" 
#include "datapool/include/common_typedef.h"
#include "referenceline_frame/referenceline_frame.h" 


using namespace std;

namespace acu{
namespace planning {

ReferenceLineFrame* GetTargetData(StructReferenceLineInfo &reference_info, int &target_line_id);
bool PathBoxInLine(StructReferenceLineInfo &reference_info, const list<geometry::Site> &motionpath,
					const CarModel& car_model, const int& drive_state, int &current_id);
bool LocalPathInLine(StructReferenceLineInfo &reference_info, list<geometry::Site> &motionpath, 
                     int &origin_id, int &check_id, int drive_state = 1);
void SetSpeedLimit(ReferenceLineFrame &input_line, SpeedplanConfig &speedplan_config,
                   pair<double, double> remove_point);
void ObjectSpeedTransform(ReferenceLineFrame& reference_line, LineObject& object);
void IsParallelObject(LineObject& object, const CarModel *car_model_ptr);

} // namespace planning
} // namespace acu


#endif
