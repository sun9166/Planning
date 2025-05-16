/**
 * @file path_info.h
 **/

#pragma once

#include <list>
#include <string>
#include <utility>

#include "discretized_path.h"
#include "frenet_frame_path.h"
#include "src/execution/motionplan/common/reference_line_info/reference_line.h"

namespace acu {
namespace planning {

class PathInfo {
 public:
  PathInfo() = default;

  bool SetDiscretizedPath(const DiscretizedPath &path, const bool update_history = true );

  bool SetFrenetPath(const FrenetFramePath &frenet_path);

  void SetReferenceLine(const ReferenceLine *reference_line);

  const DiscretizedPath &discretized_path() const;
  DiscretizedPath* mutable_discretized_path();

  const FrenetFramePath &frenet_frame_path() const;

  bool GetPathPointWithPathS(const double s,
                             common::PathPoint *const path_point) const;

  std::list<std::pair<DiscretizedPath, FrenetFramePath>> &path_data_history();

  /*
   * brief: this function will find the path_point in discretized_path whose
   * projection to reference line has s value closest to ref_s.
   */
  bool GetPathPointWithRefS(const double ref_s,
                            common::PathPoint *const path_point) const;

  bool LeftTrimWithRefS(const common::FrenetFramePoint &frenet_point);

  bool UpdateFrenetFramePath(const ReferenceLine *reference_line);

  bool UpdateDiscretizedPath(const std::vector<common::TrajectoryPoint>& path);

  void Clear();

  bool Empty() const;

  void SetPropertyLength(double length) {
     length_ = length;
  } 
  double property_length() const {
     return length_;
  }

  void SetOffsetPropertyLength(double length) {//车道内偏移轨迹的特征长度
     offset_property_length_ = length;
  } 
  double offset_property_length() const {
     return offset_property_length_;
  }

  bool is_new() const {
    return is_new_;
  }

  void SetIsNew(bool is_new_path) {
    is_new_ = is_new_path;
  } 

  bool IsReverse() const{
    return reverse_;
  }

  void SetReverse(bool reverse_flag) {
    reverse_ = reverse_flag;
    return;
  }

  void set_path_label(const std::string &label);

  const std::string &path_label() const;

 private:
  /*
   * convert frenet path to cartesian path by reference line
   */
  bool SLToXY(const FrenetFramePath &frenet_path,
              DiscretizedPath *const discretized_path);
  bool XYToSL(const DiscretizedPath &discretized_path,
              FrenetFramePath *const frenet_path);
  const ReferenceLine *reference_line_ = nullptr;
  DiscretizedPath discretized_path_;
  FrenetFramePath frenet_path_;
  std::list<std::pair<DiscretizedPath, FrenetFramePath>> path_data_history_;

  double length_ = -1;
  double offset_property_length_ = -1;
  bool reverse_ = false;
  bool is_new_ = false;
  std::string path_label_ = "";
};

}  // namespace planning
}  // namespace acu
