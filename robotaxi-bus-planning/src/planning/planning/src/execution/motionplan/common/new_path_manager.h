/**
 * @file new_path_manager.h
 **/

#pragma once

#include <vector>
#include <map>
#include "vectormap.h"
#include "macro.h"
#include "src/execution/motionplan/common/datatype.h"
#include "datapool/include/decision_typedef.h"
#include "common/math/vec2d.h"
#include "common/math/angle.h"
#include "referenceline_frame/sl_boundary/sl_boundary.h"
using namespace acu::vectormap;
namespace acu {
namespace planning {


class NewPathManager {
 public:
  ~NewPathManager() = default;

  bool UpdateNewPath(const ReferenceLineFrame& raw_reference_line,
                     const int& target_reference_line_id);

  void SetNewPath(const geometry::SiteVec& points, const NewPathType& path_type);
  
  void SetNewPath(const geometry::SiteVec& points);
  const geometry::SiteVec& new_local_path() const {
    return new_local_path_;
  }

  void Clear();

  bool is_first_stithching() const {
    return new_path_counter_ > 1 ? false : true;
  }

  bool is_new_stithing_path() const {
    return is_new_stithing_path_;
  }
  void ResetIsNewStithingPath();

  bool is_pull_over_path() const;
  bool is_offset_path() const;
  
  bool stitch_state() const {
    return stitch_state_;
  }

 private:

  int GetNearestIndex(const geometry::SiteVec &path, const geometry::Site point, bool angle_limit);
  bool IsNewPath(const geometry::SiteVec& path_points) const;
  bool Stitch(const ReferenceLineFrame& reference_line);//@pqg 
  bool GetNearestPoint(const geometry::SiteVec& points, const common::math::Vec2d& point, 
                     double* accumulate_s,double* lateral,int& index);
  double IncludeAngle(double angle1, double angle2);

  geometry::SiteVec new_local_path_;
  bool is_new_stithing_path_ = false; 

  size_t new_path_counter_ = 0;    

  bool stitch_state_ = true;

  int target_reference_line_id_ = -1;

  NewPathType type_ = NewPathType::NONE;

  DECLARE_SINGLETON(NewPathManager)
};

}  // namespace planning
}  // namespace acu
