#include "frame_process.h"
 
namespace acu{ 
namespace planning {

TrafficLightProcess trafficlight_process_;

void CalculateLineInfo(ReferenceLineFrame &input_line, 
                         vector<StructTrafficlightInfo> &matched_lights,
                         std::map<int, LineObject> &all_objects,
                         DecisionInfo &behavior_info,
                         PathData &motionpath,
                         LocalizationData &localization,
                         CarModel &car_model,
                         SpeedplanConfig &speedplan_config,
                         const float& compensate_s_in_unit_m, 
                         const float& compensate_t_in_unit_s) {
  if (input_line.mapinfo.path_points.empty()) return; 
  double local_path_length = motionpath.path.back().length;
  trafficlight_process_.ChooseFocusLight(matched_lights, behavior_info, motionpath, 
                                        localization.velocity, input_line.mapinfo);
  trafficlight_process_.SpeedLimit(input_line.mapinfo, speedplan_config.maximum_junction_speed);
  ObjectProjection object_projection(input_line, all_objects, local_path_length,
                                     localization, car_model);
  object_projection.AddObjectToFrame();
  AERROR_IF(FLAGS_log_enable)<<"<<<<<<<<<<< st >>>>>>>>>>>>>";
  Site loc_site = localization.ToSite();
  input_line.st_map.InitSTmap(&input_line.mapinfo, 
                              &input_line.last_path_points, 
                              input_line.reference_lane_id,
                              &loc_site, &car_model);
  input_line.st_map.CalculateObjectsST(input_line.objects_);
  object_projection.AddFrameObjectInfo();
  SemanticProjection semantic;
  semantic.AddSemanticInfo(input_line);
  input_line.st_map.BuildSTMap(input_line.objects_);
  //分辨率处理
  int compensate_t = ceil(compensate_t_in_unit_s / FLAGS_scale_t);
  int compensate_s = ceil(compensate_s_in_unit_m / FLAGS_scale_s);
  AINFO_IF((compensate_t_in_unit_s > 1e-3 || compensate_s_in_unit_m > 1e-3)&&FLAGS_log_enable)
        <<" compensate_t_in_unit_s: "<<compensate_t_in_unit_s<<", "
        <<" compensate_t: "<<compensate_t<<", " 
        <<" compensate_s_in_unit_m: "<<compensate_s_in_unit_m<<", "
        <<" compensate_s: "<<compensate_s;
  input_line.st_map.SetCompensate_T(compensate_t);
  input_line.st_map.SetCompensate_S(compensate_s);
  LineGap line_gap;
  line_gap.AddGapInfo(input_line, localization, speedplan_config); 
  SetSpeedLimit(input_line, speedplan_config, behavior_info.speed_limit_point);
  AERROR_IF(FLAGS_log_enable)<<"line "<<input_line.reference_lane_id
                             <<", cost "<<input_line.mapinfo.global_cost;;
}

void CalculateRevLineInfo(ReferenceLineFrame &reverse_line, std::map<int, LineObject> &all_objects,
                    double &local_path_length, LocalizationData &localization, CarModel &car_model) {
  if (reverse_line.mapinfo.front_lane_ids.size() &&
      reverse_line.mapinfo.path_points.size() > 3) {
    AWARN_IF(FLAGS_log_enable) <<"--------------------reverse--------------------";
    ObjectProjection object_projection(reverse_line, all_objects, local_path_length, 
                                       localization, car_model);
    object_projection.AddObjectToFrame();
  }
}

void MergeCurrentToLocal(ReferenceLineFrame &current_line, ReferenceLineFrame &local_line, bool has_local_flag) {
  if (!has_local_flag) {
    local_line.Reset();
    local_line = current_line;
    local_line.reference_lane_id = 40;
    AERROR_IF(FLAGS_log_enable)<<"local same as current, points size "<<current_line.mapinfo.path_points.size(); 
  }
}


} // namespace planning
} // namespace acu
