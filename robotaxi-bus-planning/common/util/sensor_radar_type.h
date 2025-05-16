#ifndef _SENSOR_RADAR_TYPE_H_
#define _SENSOR_RADAR_TYPE_H_

#include <vector>

#include <boost/algorithm/string.hpp>

#include "common/base/config/include/param_config_manager.h"
#include "common/util/sensor_vision_type.h"

namespace acu {
namespace perception {

#define ARS_0_SOURCE 0x0201
#define ARS_1_SOURCE 0x0202

enum class RadarObjectClass {
  CAR = 0,
  TRUCK = 1,
  PEDESTRIAN = 2,
  CYCLIST = 3,
  UNKNOWN = 4,
  UNKNOWN_MOVABLE = 5,
  UNKNOWN_UNMOVABLE = 6
};

enum class RadarMeasState { DELETED = 0, NEW = 1, MEASURED = 2, PREDICTED = 3, DELETED_FOR = 4, NEW_FROM_MERGE = 5 };

enum class RadarDynProp {
  MOVING = 0,
  STATIONARY = 1,
  ONCOMING = 2,
  STATIONARY_CANDICATE = 3,
  UNKNOWN = 4,
  CROSSING_STATIONARY = 5,
  CROSSING_MOVING = 6,
  STOPPED = 7
};

struct RadarObject {
  int id = 0;
  float x = 0.0;
  float y = 0.0;
  float range = 0.0;
  float rangerate = 0.0;
  float lateralrate = 0.0;
  float vxrel = 0.0;
  float vyrel = 0.0;
  RadarDynProp dyn_prop = RadarDynProp::MOVING;
  double rcs = 0.0;

  RadarMeasState meas_state = RadarMeasState::DELETED;
  double prob_of_exist = 0.0;
  int invalid_state = 0;
  int ambig_state = 0;

  double dist_long_rms = 0.0;
  double dist_lat_rms = 0.0;
  double vrel_long_rms = 0.0;
  double vrel_lat_rms = 0.0;
  double arel_long_rms = 0.0;
  double arel_lat_rms = 0.0;
  double orientation_rms = 0.0;

  double arel_long = 0.0;
  RadarObjectClass object_class = RadarObjectClass::UNKNOWN;
  double arel_lat = 0.0;
  double orientation_angle = 0.0;
  double length = 0.0;
  double width = 0.0;

  int status = 0;
  int rangemode = 0;
  int bridge = 0;

  double confidence = 0.0;
  bool is_static = true;

  // tracking information
  // @brief anchor point, required
  Eigen::Vector3d anchor_point = Eigen::Vector3d(0, 0, 0);
  // @brief center of the boundingbox (cx, cy, cz), required
  Eigen::Vector3d center = Eigen::Vector3d(0, 0, 0);
  // @brief covariance matrix of the center uncertainty, required
  Eigen::Matrix3f center_uncertainty{};
  // @brief track id, required
  int track_id = -1;
  // @brief velocity of the object, required
  Eigen::Vector3f velocity = Eigen::Vector3f(0, 0, 0);
  // @brief covariance matrix of the velocity uncertainty, required
  Eigen::Matrix3f velocity_uncertainty{};
  // @brief if the velocity estimation is converged, true by default
  bool velocity_converged = true;
  // @brief velocity confidence, required
  float velocity_confidence = 1.0f;
  // @brief acceleration of the object, required
  Eigen::Vector3f acceleration = Eigen::Vector3f(0, 0, 0);
  // @brief covariance matrix of the acceleration uncertainty, required
  Eigen::Matrix3f acceleration_uncertainty{};

  // @brief age of the tracked object, required
  double tracking_time = 0.0;
  // @brief timestamp of latest measurement, required
  double latest_tracked_time = 0.0;
  bool matched_vision = false;
  int visionobject_id = 0;
  int visionobject_age = 0;
  double time_triggered;  // timestamp  seconds
  double time_received;   // timestamp  seconds
  int track_age = 0;
  int vision_type = 4;
  int fusion_type = 255;
};

typedef struct BBox2d {
  unsigned int xmin, ymin, xmax, ymax;
  float x;
  float y;
  std::string object_class;
};

typedef struct ObjWithImg {
  cv::Point3f ObjInVCS;
  cv::Point3f ObjInImg;
} ObjWithImg;

typedef struct RadarCamObject {
  int camIdx;  // 0-front_h60, 1-left_h90, 2-right_h90
  int orientation_id;
  int matchedCnt = 0;
  bool is_matched = false;
  bool in_img = false;
  bool is_box = false;
  BBox2d bbox2d_;
  ObjWithImg vcsobj;
  ObjDetectBoxes imgobj;
} RadarCamObject;

struct RadarDataMsg {
  double time;
  int source;
  std::string radar_type = "all";
  std::vector<RadarObject> objs;
  void reset() {
    time = 0.0;
    radar_type = "all";
    objs.clear();
  }
};

using Object = RadarObject;
using ObjectPtr = std::shared_ptr<RadarObject>;
using ObjectConstPtr = std::shared_ptr<const RadarObject>;
using Frame = RadarDataMsg;
using FramePtr = std::shared_ptr<Frame>;
using FrameConstPtr = std::shared_ptr<const Frame>;

struct RadarProcessConfig {
  bool enable_debug_show = false;
  bool use_cluster = true;
  bool use_tracker = true;
  bool use_filter = false;
  bool use_mobileye = false;
  double tracking_time_win = 0.3;
  double tracked_times_threshold = 3;
  std::string chosen_filter = "AdaptiveKalmanFilter";
  double max_match_distance = 2.5;
  double bound_match_distance = 10;
  double cluster_espilon = 2.0;
  double mefilter_espilon = 2.0;
  bool output_csv = false;

  bool LoadFromModle(ModelConfig *pmodel_config) {
    if (pmodel_config->GetValue("enable_debug_show", &enable_debug_show) == false ||
        pmodel_config->GetValue("use_cluster", &use_cluster) == false ||
        pmodel_config->GetValue("use_tracker", &use_tracker) == false ||
        pmodel_config->GetValue("use_filter", &use_filter) == false ||
        pmodel_config->GetValue("use_mobileye", &use_mobileye) == false ||
        pmodel_config->GetValue("tracking_time_win", &tracking_time_win) == false ||
        pmodel_config->GetValue("tracked_times_threshold", &tracked_times_threshold) == false ||
        pmodel_config->GetValue("chosen_filter", &chosen_filter) == false ||
        pmodel_config->GetValue("max_match_distance", &max_match_distance) == false ||
        pmodel_config->GetValue("bound_match_distance", &bound_match_distance) == false ||
        pmodel_config->GetValue("cluster_espilon", &cluster_espilon) == false ||
        pmodel_config->GetValue("mefilter_espilon", &mefilter_espilon) == false ||
        pmodel_config->GetValue("output_csv", &output_csv) == false) {
      AERROR << "Get radarprocess param error!";
      return false;
    }
    return true;
  }
};

typedef struct SensorCalibrationConfig {
  int source;
  double x_offset;
  double y_offset;
  double z_offset;
  double roll;
  double pitch;
  double yaw;
  SensorCalibrationConfig() {
    x_offset = 0.0;
    y_offset = 0.0;
    z_offset = 0.0;
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
  }
  bool LoadFromModle(const std::string &prefix, ModelConfig *pmodel_config) {
    if (pmodel_config->GetValue(prefix + "_xoffset", &x_offset) == false) {
      AERROR << "error " << prefix + "_xoffset";
      return false;
    }
    if (pmodel_config->GetValue(prefix + "_yoffset", &y_offset) == false) {
      AERROR << "error " << prefix + "_yoffset";
      return false;
    }
    if (pmodel_config->GetValue(prefix + "_zoffset", &z_offset) == false) {
      AERROR << "error " << prefix + "_zoffset";
      return false;
    }
    if (pmodel_config->GetValue(prefix + "_roll", &roll) == false) {
      AERROR << "error " << prefix + "_roll";
      return false;
    }
    if (pmodel_config->GetValue(prefix + "_pitch", &pitch) == false) {
      AERROR << "error " << prefix + "_pitch";
      return false;
    }
    if (pmodel_config->GetValue(prefix + "_yaw", &yaw) == false) {
      AERROR << "error " << prefix + "_yaw";
      return false;
    }
    return true;
  }

} SensorCalibrationConfig;

typedef struct Radar2CamMoel {
  bool enable_debug_show;
  bool enable_fusion;
  std::string car_config_root;
  std::string camera_param_path;
  std::vector<std::string> camera_names;
  std::string camera_driver_node_name;
  bool enable_logging;
  Radar2CamMoel() {
    enable_debug_show = false;
    enable_fusion = true;
    camera_param_path = "";
    camera_names = {};
    camera_driver_node_name = {};
    enable_logging = false;
  }

  bool LoadFromModle(ModelConfig *pmodel_config, const std::string config_root) {
    if (pmodel_config->GetValue("enable_fusion", &enable_fusion) == false) {
      AERROR << "cannot get param radar2camera/enable_fusion";
      return false;
    }
    if (pmodel_config->GetValue("car_config_root", &car_config_root) == false) {
      AERROR << "cannot get param radar2camera/car_config_root";
      return false;
    }
    if (pmodel_config->GetValue("camera_param_path", &camera_param_path) == false) {
      AERROR << "cannot get param radar2camera/camera_param_path";
      return false;
    }
    if (pmodel_config->GetValue("camera_driver_node_name", &camera_driver_node_name) == false) {
      AERROR << "cannot get param radar2camera/camera_driver_node_name";
      return false;
    }
    std::string camera_names_str = "";
    if (pmodel_config->GetValue("camera_names", &camera_names_str) == false) {
      AERROR << "cannot get param radar2camera/camera_names";
      return false;
    } else {
      boost::algorithm::split(camera_names, camera_names_str, boost::algorithm::is_any_of(","));
    }
    if (pmodel_config->GetValue("enable_debug_show", &enable_debug_show) == false) {
      AERROR << "cannot get param radar2camera/enable_debug_show";
      return false;
    }
    if (pmodel_config->GetValue("enable_logging", &enable_logging) == false) {
      AERROR << "cannot get param radar2camera/enable_logging";
      return false;
    }
    camera_param_path = config_root + camera_param_path;
    return true;
  }
} Radar2CamMoel;

typedef struct Radar2CamConfigType {
  Radar2CamMoel cfg;
  double height;
  bool LoadConfig(ModelConfig *pmodel_config, const std::string config_root) {
    if (cfg.LoadFromModle(pmodel_config, config_root) == false) {
      return false;
    }
    return true;
  }
  bool LoadGround(ModelConfig *pmodel_config) {
    if (pmodel_config->GetValue("main_lidar_z_ground_offset", &height) == false) {
      AERROR << "cannot get param main_lidar_z_ground_offset";
      return false;
    }
    return true;
  }
} Radar2CamConfigType;

}  // namespace perception
}  // namespace acu

#endif
