#include "time_matching.h"
 
namespace acu{ 
namespace planning {


bool ChooseSyncData(double sync_time, 
                    vector<MapEngineData> &mapengine_data_list,
                    vector<LocalizationData> &loc_data_list, 
                    double &map_min_time, double &imu_min_time,
                    int &map_index, int &imu_index,
                    float& compensate_s_in_unit_m, float& compensate_t_in_unit_s) {
  map_index = -1;
  imu_index = -1;
	map_min_time = std::numeric_limits<double>::max();
	imu_min_time = std::numeric_limits<double>::max();
  for (int i = 0; i < mapengine_data_list.size(); i++) {
    double delta_time = fabs(mapengine_data_list.at(i).time - sync_time);
    if (delta_time < map_min_time) {
      map_min_time = delta_time;
      map_index = i;
    }
  }
  AERROR_IF(FLAGS_log_enable)<<setprecision(6)<<"map_min_time is "<<map_min_time;
  if (map_index < 0 || map_index >= mapengine_data_list.size()) {
    return false;
  }
  if (map_index < mapengine_data_list.size() - 1) {
    mapengine_data_list.erase(mapengine_data_list.begin() + map_index + 1, 
    													mapengine_data_list.end());
  }

  for (int i = 0; i < loc_data_list.size(); i++) {
    double delta_time = fabs(loc_data_list.at(i).time - sync_time);
    if (delta_time < imu_min_time) {
      imu_min_time = delta_time;
      imu_index = i;
    } 
  }
  AERROR_IF(FLAGS_log_enable)<<setprecision(6)<<"imu_min_time is "<<imu_min_time;
  if (imu_index < 0 || imu_index >= loc_data_list.size()) {
    return false;
  }
  if (imu_index < loc_data_list.size() - 1) {
    loc_data_list.erase(loc_data_list.begin() + imu_index + 1, 
                        loc_data_list.end());
  }

  const double delay_time = 0.5;//10hz
  auto localization_data = loc_data_list.at(imu_index);
  if(!loc_data_list.empty()){
    //最新定位位置 与 感知匹配到的定位位置 之间的差距作为S补偿
    auto latest_localization = loc_data_list.front();
    //单位为m，未进行分辨率处理
    compensate_s_in_unit_m = std::hypot(latest_localization.xg - localization_data.xg, 
                                latest_localization.yg - localization_data.yg);
    //匹配到的定位速度乘延时作为S补偿
    compensate_s_in_unit_m += localization_data.velocity * delay_time;
    //感知延时时间作为T补偿
    compensate_t_in_unit_s = imu_min_time + delay_time;//单位为s，未进行分辨率处理
  }else{
    compensate_s_in_unit_m = 0.0;
    compensate_t_in_unit_s = 0.0;
  }
  return true;
}




} // namespace planning
} // namespace acu
