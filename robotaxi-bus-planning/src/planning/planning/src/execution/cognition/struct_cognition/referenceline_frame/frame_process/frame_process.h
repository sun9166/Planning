#ifndef SRC_EXECUTION_COGNITION_FRAME_PROCESS_H_
#define SRC_EXECUTION_COGNITION_FRAME_PROCESS_H_

#include "datapool/include/common_typedef.h"
#include "conf/cognition_gflags.h" 
#include "input_process/light_process/trafficlight_process.h" 
#include "referenceline_frame/frame_base/frame_base.h"
#include "referenceline_frame/projection/object_projection.h"
#include "referenceline_frame/projection/semantic_projection.h"
#include "referenceline_frame/st_map/st_map.h"
#include "referenceline_frame/line_gap/line_gap.h"

using namespace std;

namespace acu{
namespace planning {

void CalculateLineInfo(ReferenceLineFrame &input_line, 
                     vector<StructTrafficlightInfo> &matched_lights,
                     std::map<int, LineObject> &all_objects,
                     DecisionInfo &behavior_info,
                     PathData &motionpath,
                     LocalizationData &localization,
                     CarModel &car_model,
                     SpeedplanConfig &speedplan_config,
                     const float& compensate_s_in_unit_m = 0.0, 
                     const float& compensate_t_in_unit_s = 0.0);
void CalculateRevLineInfo(ReferenceLineFrame &reverse_line, std::map<int, LineObject> &all_objects,
                    double &local_path_length, LocalizationData &localization, CarModel &car_model);
void MergeCurrentToLocal(ReferenceLineFrame &current_line, ReferenceLineFrame &local_line, bool has_local_flag);

} // namespace planning
} // namespace acu


#endif
