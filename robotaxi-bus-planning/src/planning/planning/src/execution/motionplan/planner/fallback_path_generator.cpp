
/**
 * @file fallback_path_profile_generator.cpp
 **/

#include <algorithm>
#include "../common/ego_info.h"
#include "../common/planning_gflags.h"
#include "fallback_path_generator.h"
#include "common/base/log/include/log.h"
#include "src/execution/motionplan/common/frame/frame.h"


namespace acu {
namespace planning {
namespace fallback_path_generator {  

using common::SpeedPoint;
using common::SLPoint;
using common::TrajectoryPoint;
using common::math::Vec2d;

void GenerateFallbackPathProfile(//参考线朝本车所在的位置平移得到路径
  const ReferenceLineInfo* reference_line_info, PathInfo* path_data) {
  auto adc_point = EgoInfo::instance()->start_point();
  auto init_point =
      reference_line_info->reference_line().GetFrenetPoint(adc_point.path_point());
  const double max_s = 150.0;
  const double unit_s = 1.0;

  // projection of adc point onto reference line
  const auto& adc_ref_point =
    reference_line_info->reference_line().GetReferencePoint(init_point.s());

  DCHECK(adc_point.has_path_point());
  const double dx = adc_point.path_point().x() - adc_ref_point.x();
  const double dy = adc_point.path_point().y() - adc_ref_point.y();

  std::vector<common::PathPoint> path_points;
  for (double s = 0.0; s < max_s; s += unit_s) {
    if (s > reference_line_info->reference_line().Length()) break; 
    const auto& ref_point =
      reference_line_info->reference_line().GetReferencePoint(s + init_point.s());
    common::PathPoint path_point = common::util::MakePathPoint(
                                     ref_point.x() + dx, ref_point.y() + dy, 0.0, ref_point.heading(),
                                     ref_point.kappa(), ref_point.dkappa(), 0.0);
    path_point.set_s(s);

    path_points.push_back(std::move(path_point));
    if (FLAGS_enable_generate_one_pathpoint_fallback_path) {
      break; //if path plan failed when start , then generate stop trajectory. 
    }
  }
  path_data->SetReferenceLine(&reference_line_info->reference_line());
  path_data->SetDiscretizedPath(DiscretizedPath(std::move(path_points)));
}

void GenerateFallbackPathProfileUsingPrevTrajectory(
  const ReferenceLineInfo* reference_line_info, PathInfo* path_data, const PublishableTrajectory &path) {
  CHECK_GT(path.NumOfPoints(),0); 
  auto adc_point = reference_line_info->speed_init_point();//should be speedplan start point //0525 @pqg
  common::math::Vec2d start_path_point(adc_point.path_point().x(), adc_point.path_point().y());
  auto match_index = path.QueryNearestPointWithBuffer(start_path_point,1.0e-6);
  if (match_index + 1 > path.size()) {
    AERROR<<"start point is over path end!!!!! , match_index = "<<match_index<<", last path size = "<<path.size();
    return;
  }
  std::vector<common::PathPoint> path_points;
  double adc_s = path.at(match_index).path_point().s();
  for (size_t i = match_index; i < path.size(); ++i) {
    common::PathPoint path_point = path.at(i).path_point();
    path_point.set_s(path.at(i).path_point().s() - adc_s);
    path_points.push_back(std::move(path_point));
  }
  path_data->SetReferenceLine(&reference_line_info->reference_line());
  path_data->SetDiscretizedPath(DiscretizedPath(std::move(path_points)));
  path_data->SetPropertyLength(path.property_length());
}

}
}  // namespace planning
}  // namespace acu
