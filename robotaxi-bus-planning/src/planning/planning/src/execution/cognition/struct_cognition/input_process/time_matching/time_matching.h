#ifndef SRC_EXECUTION_COGNITION_MAPPROCESS_TIME_MATCHING_H_
#define SRC_EXECUTION_COGNITION_MAPPROCESS_TIME_MATCHING_H_
#include "conf/cognition_gflags.h" 
#include "datapool/include/data_pool.h"

using namespace std;

namespace acu{
namespace planning {

bool ChooseSyncData(double sync_time, 
                    vector<MapEngineData> &mapengine_data_list,
                    vector<LocalizationData> &loc_data_list, 
                    double &map_min_time, double &imu_min_time,
                    int &map_index, int &imu_index,
                    float& compensate_s_in_unit_m, float& compensate_t_in_unit_s);

} // namespace planning
} // namespace acu


#endif
