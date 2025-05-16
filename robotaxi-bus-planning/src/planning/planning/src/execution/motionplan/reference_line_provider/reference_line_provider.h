/**
 * @file reference_line_provider.h
 *
 * @brief Declaration of the class ReferenceLineProvider.
 */

#pragma once

#include <list>
#include <memory>
#include <queue>
#include <string>
#include <unordered_set>
#include <vector>

#include "src/execution/motionplan/common/reference_line_info/reference_line.h"
#include "common/util/util.h"
#include "src/execution/motionplan/common/datatype.h"
#include "common/toolbox/geometry/include/geoheader.h"
#include "src/execution/motionplan/reference_line_provider/reference_line_smoother/discrete_points_reference_line_smoother.h"
#include "reference_line_smoother_config.pb.h"
/**
 * @namespace acu::planning
 * @brief acu::planning
 */
namespace acu {
namespace planning {

/**
 * @class ReferenceLineProvider
 * @brief The class of ReferenceLineProvider.
 *        It provides smoothed reference line to planning.
 */

class ReferenceLineProvider {
 public:
  ReferenceLineProvider() ;

  /**
   * @brief Default destructor.
   */
  ~ReferenceLineProvider();

  bool Start();

  void Stop();

  void UpdateVehicleState(const VehicleState &vehicle_state);

  bool GetReferenceLines(std::list<ReferenceLine> *reference_lines,
          const ReferenceLineFrame* ref_line,
          const bool pathplan_status_last_frame);

  int smoothed_status() {
    return smoothed_status_;
  }

 private:

  /**
   * @brief store the computed reference line. This function can avoid
   * unnecessary copy if the reference lines are the same.
   */

  void UpdateReferenceLine(
      const std::list<ReferenceLine>& reference_lines);

  void IsValidReferenceLine();

  bool IsReferenceLineSmoothValid(const ReferenceLine& raw,
                                  const ReferenceLine& smoothed) const;
  AnchorPoint GetAnchorPoint(const ReferenceLine& reference_line,
                             double s) const;
  void GetAnchorPoints(const ReferenceLine& reference_line,
                       std::vector<AnchorPoint>* anchor_points) const;
  bool SmoothReferenceLine(const ReferenceLine& raw_reference_line,
                           ReferenceLine* reference_line);
  bool ExtendReferenceLine(const VehicleState& state,
                           const ReferenceLine& raw_reference_line,
                           ReferenceLine* reference_line,
                           const vector<pair<double, double> >& speed_limits) ;
  bool SmoothPrefixedReferenceLine(const ReferenceLine& prefix_ref,
                                   const ReferenceLine& raw_ref,
                                   ReferenceLine* reference_line);
  bool Shrink(const common::SLPoint& sl, ReferenceLine* ref,const vector<pair<double, double> >& speed_limits,
              const ReferenceLine& raw_reference_line);
 private:
  
  bool is_initialized_ = false;

  VehicleState vehicle_state_;

  std::list<ReferenceLine> reference_lines_;

  std::unique_ptr<ReferenceLineSmoother> smoother_;
  ReferenceLineSmootherConfig smoother_config_;

  int smoothed_status_ = 0; //0: default value  1: SmoothReferenceLine 2:ExtendReferenceLine 3:smoothed failed. 4. Extend failed

  std::queue<std::list<ReferenceLine>> reference_line_history_;
  int last_reference_line_type_ = 0; //0: normal 1: local_stithing
};

}  // namespace planning
}  // namespace acu
