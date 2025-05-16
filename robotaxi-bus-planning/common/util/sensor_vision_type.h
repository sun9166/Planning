#ifndef _SENSOR_VISION_TYPE_H
#define _SENSOR_VISION_TYPE_H

#include <boost/algorithm/string.hpp>
#include <opencv2/opencv.hpp>
#include "common/base/log/include/log.h"
#include "common/base/config/include/param_config_manager.h"

namespace acu {
namespace perception {

typedef struct ObjDetectBoxes {
  unsigned int x, y, w, h;  // (x,y) - top-left corner, (w, h) - width & height of bounded box
  float prob;               // confidence - probability that the object was found correctly
  unsigned int obj_id;      // class of object - from range [0, classes-1]
  unsigned int orientation_id;
  bool track_enable;
  unsigned int track_id;
  unsigned int track_age;
  unsigned int track_time_since_update;
  float track_score;
} ObjDetectBoxes;

typedef struct ObjDetectBoxesTime {
  double time_triggered;  // timestamp  seconds
  double time_received;   // timestamp  seconds
  bool image_valid[4];    // 0:front60  1: left90  2:right90  3:front30
  // std::array<cv::Mat, 4> current_image;
  std::array<std::vector<ObjDetectBoxes>, 4> boxes_data;
  void reset() {
    time_triggered = 0;
    time_received = 0;
    image_valid[0] = false;
    image_valid[1] = false;
    image_valid[2] = false;
    image_valid[3] = false;
    boxes_data[0].clear();
    boxes_data[1].clear();
    boxes_data[2].clear();
    boxes_data[3].clear();
  }
  ~ObjDetectBoxesTime() {
    boxes_data[0].clear();
    boxes_data[1].clear();
    boxes_data[2].clear();
    boxes_data[3].clear();
  }
} ObjDetectBoxesTime;

typedef struct ImageWithTime {
  double time_triggered;  // timestamp  seconds
  double time_received;   // timestamp  seconds
  cv::Mat image;
  void reset() {
    time_triggered = 0;
    time_received = 0;
    image.release();
  }
  ~ImageWithTime() { image.release(); }
} ImageWithTime;

// the following params are for lidar2camera fusion functionality
typedef struct LidarCamFusionManagerConfig {
  bool enable_fusion;
  bool enable_debug_show;
  std::string car_config_root;
  std::string camera_param_path;
  std::vector<std::string> camera_names;
  std::string camera_driver_node_name;
  bool enable_logging;
  LidarCamFusionManagerConfig() {
    enable_fusion = true;
    enable_debug_show = false;
    camera_param_path = "";
    camera_names = {};
    camera_driver_node_name = {};
    enable_logging = false;
  }

  bool LoadFromModle(ModelConfig *pmodel_config, const std::string config_root) {
    if (pmodel_config->GetValue("enable_fusion", &enable_fusion) == false) {
      AERROR << "cannot get param lidar2camera/enable_fusion";
      return false;
    }
    if (pmodel_config->GetValue("car_config_root", &car_config_root) == false) {
      AERROR << "cannot get param lidar2camera/car_config_root";
      return false;
    }
    if (pmodel_config->GetValue("camera_param_path", &camera_param_path) == false) {
      AERROR << "cannot get param lidar2camera/camera_param_path";
      return false;
    }
    if (pmodel_config->GetValue("camera_driver_node_name", &camera_driver_node_name) == false) {
      AERROR << "cannot get param lidar2camera/camera_driver_node_name";
      return false;
    }
    std::string camera_names_str = "";
    if (pmodel_config->GetValue("camera_names", &camera_names_str) == false) {
      AERROR << "cannot get param lidar2camera/camera_names";
      return false;
    } else {
      boost::algorithm::split(camera_names, camera_names_str, boost::algorithm::is_any_of(","));
    }
    if (pmodel_config->GetValue("enable_debug_show", &enable_debug_show) == false) {
      AERROR << "cannot get param lidar2camera/enable_debug_show";
      return false;
    }
    if (pmodel_config->GetValue("enable_logging", &enable_logging) == false) {
      AERROR << "cannot get param lidar2camera/enable_logging";
      return false;
    }
    camera_param_path = config_root + camera_param_path;
    return true;
  }
} LidarCamFusionManagerConfig;

typedef struct LidarCamFusionConfigType {
  LidarCamFusionManagerConfig configmanager;
  bool LoadLidarCamFusionConfigFromModle(ModelConfig *pmodel_config, const std::string config_root) {
    if (configmanager.LoadFromModle(pmodel_config, config_root) == false) {
      return false;
    }
    return true;
  }
} LidarCamFusionConfigType;
typedef struct SingleSegmentROI {
  int x;
  int y;
  int w;
  int h;
  SingleSegmentROI() {
    x = 0;
    y = 0;
    w = 0;
    h = 0;
  }
} SingleSegmentROI;
typedef struct VisionSegmentROIConfig {
  SingleSegmentROI front_roi_;
  SingleSegmentROI left_roi_;
  SingleSegmentROI right_roi_;
  VisionSegmentROIConfig() {
    front_roi_.x = 320;
    front_roi_.y = 240;
    front_roi_.w = 640;
    front_roi_.h = 320;
    left_roi_.x = 0;
    left_roi_.y = 80;
    left_roi_.w = 1280;
    left_roi_.h = 720 - 80;
    left_roi_.x = 0;
    left_roi_.y = 80;
    left_roi_.w = 1280;
    left_roi_.h = 720 - 80;
  }
  bool LoadSegmentROI(ModelConfig *pmodel_config) {
    if (pmodel_config->GetValue("segment_front_ROI_x", &front_roi_.x) == false) {
      AERROR << "cannot get param segment_front_ROI_x";
      return false;
    }
    if (pmodel_config->GetValue("segment_front_ROI_y", &front_roi_.y) == false) {
      AERROR << "cannot get param segment_front_ROI_y";
      return false;
    }
    if (pmodel_config->GetValue("segment_front_ROI_w", &front_roi_.w) == false) {
      AERROR << "cannot get param segment_front_ROI_w";
      return false;
    }
    if (pmodel_config->GetValue("segment_front_ROI_h", &front_roi_.h) == false) {
      AERROR << "cannot get param segment_front_ROI_h";
      return false;
    }
    if (pmodel_config->GetValue("segment_left_ROI_x", &left_roi_.x) == false) {
      AERROR << "cannot get param segment_left_ROI_x";
      return false;
    }
    if (pmodel_config->GetValue("segment_left_ROI_y", &left_roi_.y) == false) {
      AERROR << "cannot get param segment_left_ROI_y";
      return false;
    }
    if (pmodel_config->GetValue("segment_left_ROI_w", &left_roi_.w) == false) {
      AERROR << "cannot get param segment_left_ROI_w";
      return false;
    }
    if (pmodel_config->GetValue("segment_left_ROI_h", &left_roi_.h) == false) {
      AERROR << "cannot get param segment_left_ROI_h";
      return false;
    }
    if (pmodel_config->GetValue("segment_right_ROI_x", &right_roi_.x) == false) {
      AERROR << "cannot get param segment_right_ROI_x";
      return false;
    }
    if (pmodel_config->GetValue("segment_right_ROI_y", &right_roi_.y) == false) {
      AERROR << "cannot get param segment_right_ROI_y";
      return false;
    }

    if (pmodel_config->GetValue("segment_right_ROI_w", &right_roi_.w) == false) {
      AERROR << "cannot get param segment_right_ROI_w";
      return false;
    }
    if (pmodel_config->GetValue("segment_right_ROI_h", &right_roi_.h) == false) {
      AERROR << "cannot get param segment_right_ROI_h";
      return false;
    }
    return true;
  }
} VisionSegmentROIConfig;

}  // namespace perception
}  // namespace acu

#endif
