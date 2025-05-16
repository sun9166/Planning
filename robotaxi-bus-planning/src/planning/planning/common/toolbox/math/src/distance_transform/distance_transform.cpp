#include "common/toolbox/math/include/distance_transform/distance_transform.h"


distance_transform::distance_transform()
  : GRID_PRECISION(0.1),
    GRID_WIDTH_HALF(8),
    GRID_FRONT(8),
    GRID_BACK(2.0) {
}

distance_transform::~distance_transform() {}


bool distance_transform::LoadBaseMap(const double precision,
                                     const double f_length, const double b_length, const double half_width,
                                     cv::Mat& basemap) {
  if (precision <= 0 || half_width <= 0 || f_length <= 0 || b_length < 0) {
    AERROR <<  "[LoadBaseMap] Input Param Error";
    return false;
  }
  if (precision < BASEMAP_GRID_PRECISION) {
    AERROR <<  "[LoadBaseMap] Input Precision is too Small";
    return false;
  } else {
    int64_t dx = static_cast<int64_t>((f_length + b_length) / precision) + 1;
    int64_t dy = static_cast<int64_t>((half_width * 2) / precision) + 1;
    if ((dx * dy) > static_cast<int64_t>(std::numeric_limits<int32_t>::max())) {
      AERROR <<  "[LoadBaseMap] Load Basemap Failed";
      return false;
    }

    cols_bound_.first  = static_cast<int> (round(-b_length / precision));
    cols_bound_.second = static_cast<int> (round(f_length / precision));
    rows_bound_.first  = static_cast<int> (round(-half_width / precision));
    rows_bound_.second = static_cast<int> (round(half_width / precision));
    map_rows_ = rows_bound_.second - rows_bound_.first + 1;
    map_cols_ = cols_bound_.second - cols_bound_.first + 1;
    basemap = cv::Mat(map_rows_, map_cols_, CV_8UC1, cv::Scalar(255));
    // imshow("mat_test", basemap);
    // waitKey();
  }
  // add freespacec
  // 张富强修改，可能存在bug
//   geometry::Site vcs_temp;

// #if CHECK_FREE_SPACE
//   for (int i = cols_bound_.first; i < cols_bound_.second; ++i) {
//     vcs_temp.x = i * precision;
//     for (int j = rows_bound_.first; j < rows_bound_.second; ++j) {
//       vcs_temp.y = j * precision;
//       if (false == Adapter::VCSFSCheck(vcs_temp)) {// MAP2  //todo  xy freespace
//         basemap.at<uchar>(j - rows_bound_.first, i - cols_bound_.first) = 0;
//       }
//     }
//   }
// #endif
  // 张富强修改，可能存在bug

  // imshow("fs_test", basemap);
  // waitKey();
  return true;
}


bool distance_transform::ModifyObject(SortedTree& nearestobs,
                                      cv::Mat& obj_map) {
  geometry_msgs::Point32 tt, tp;
  // geometry_msgs::Polygon poly, poly_plus;
  // tp.x = 0.0;
  // tp.y = 0.0;
  // tp.z = 0.0;
  // carmodel(tp, poly, objmodify_vehiclemodel);

  Site p_obj_vcs;

  for (auto ha = nearestobs.begin(); ha != nearestobs.end(); ha++) {
    for (auto hi = nearestobs[ha->first].begin(); hi != nearestobs[ha->first].end(); hi++) {
      p_obj_vcs.x = ha->first;
      p_obj_vcs.y = (*hi).y;
// std::cout<<"obj : "<<p_obj_vcs.x<<" , "<<p_obj_vcs.y<<std::endl;
      auto obj_id = GridIDIndex(p_obj_vcs.x, p_obj_vcs.y, GRID_PRECISION);
      if (obj_id.gx < 0 || obj_id.gy < 0) {
        AERROR <<  "Obj is beyond the grid map !!";
        continue;
        AERROR <<  "Obj is beyond the grid map !!";
      }
      obj_map.at<uchar>(obj_id.gx, obj_id.gy) = 0;
    }
  }
  // imshow("obj_map", obj_map);
  // cv::waitKey();
  return true;
}

bool distance_transform::DistanceTransform(const cv::Mat& grid_image,
    cv::Mat& dis_image) {
  if (grid_image.empty()) {
    AERROR <<  "[DistanceTransform] Input Data Is Empty";
    return false;
  }
  distanceTransform(grid_image, dis_image, CV_DIST_L2, CV_DIST_MASK_PRECISE);

  return true;
}

void distance_transform::GenerateGridMap(SortedTree& nearestobs) {
  cv::Mat obj_map;
  bool is_load_basemap_ok = LoadBaseMap(GRID_PRECISION, GRID_FRONT, GRID_BACK,
                                        GRID_WIDTH_HALF, obj_map);
  if (!is_load_basemap_ok) {
    AERROR <<  "Load basemap failed!!";
    return;
  }
  bool is_modify_obj_ok = ModifyObject(nearestobs, obj_map);
  if (is_modify_obj_ok) {
    DistanceTransform(obj_map, dismap_);
  } else {
    return;
  }
}

double distance_transform::ReadDistanceFromMap(const Site& point) {
  GridID read_id;
  double dis = 0;
  read_id = GridIDIndex(point.x, point.y, GRID_PRECISION);
  if (dismap_.empty()) return __DBL_MAX__;
  if (-1 == read_id.gx && -1 == read_id.gy) {
    dis = __DBL_MAX__;
  } else {
    dis = dismap_.at<float>(read_id.gx, read_id.gy) * GRID_PRECISION;
  }
  return dis;
}

GridID distance_transform::GridIDIndex(const double x, const double y,
                                       const float precision) {

  GridID grid_id;
  if (x > GRID_FRONT || x < -GRID_BACK || y > GRID_WIDTH_HALF
      || y < -GRID_WIDTH_HALF) {
    grid_id.gx = -1;
    grid_id.gy = -1;
    // ROS_ERROR_STREAM("("<<x<<", "<<y<<") is beyond the grid map !!");
    return (grid_id);
  }
  grid_id.gx = rows_bound_.second - static_cast<int> (round(y / precision));
  grid_id.gy = static_cast<int> (round (x / precision)) - cols_bound_.first;

  return grid_id;
}

std::array<GridID, 8> distance_transform::GridIDIndex8(
  const GridID& center_grid) {

  std::array<GridID, 8> grid_id8;

  if (center_grid.gx > map_cols_ || center_grid.gy > map_rows_ ||
      center_grid.gx < 0 || center_grid.gy < 0) {
    AINFO << "[GridIDIndex8] Input Grid Beyond Map !!";
    grid_id8.fill(GridID(-1, -1));
    return grid_id8;
  }
  int gnum = 0;
  for (int i = -1; i < 2; ++i) {
    for (int j = -1; j < 2; ++j) {
      if (center_grid.gx + i > map_cols_ || center_grid.gx + i < 0) {
        grid_id8.at(gnum) = GridID(-1, -1);
        gnum++;
        continue;
      }
      if (0 == i && 0 == j) continue;
      if (center_grid.gy + j > map_rows_ || center_grid.gy + j < 0) {
        grid_id8.at(gnum) = GridID(-1, -1);
      } else {
        grid_id8.at(gnum) = GridID(center_grid.gx + i, center_grid.gy + j);
      }
      gnum++;
    }
  }
  return grid_id8;
}
