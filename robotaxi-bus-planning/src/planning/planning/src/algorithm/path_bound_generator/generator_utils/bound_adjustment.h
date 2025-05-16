/**
 * @file bound_process.h
 **/

#pragma once
#include "src/execution/motionplan/behavior_parser/behavior_parser.h"
#include "src/execution/motionplan/common/ego_info.h"

namespace acu {
namespace planning {
class BoundAdjustment{
public:
	BoundAdjustment();
	~BoundAdjustment() = default;

	bool UpdateBoundaryType(
	      const size_t&, const int&, const int&,
	      std::vector<std::pair<int, int>>* const boundary_types);
	bool UpdatePathBoundaryAndCenterLine(
	    const size_t&, const double&, const double&,
	    LaneBound* const path_boundaries, double* const center_line);
	void TrimPathBounds(const int&, LaneBound* const path_boundaries);
	void TrimPathSoftBounds(const int&, 
			std::vector<std::tuple<double, double, double, double, std::string>>* const path_boundaries);
	void TrimPathBoundTypes(const int&, 
			std::vector<std::pair<int, int>>* const path_boundary_types);
	void UpdateCenterLineVec(std::vector<std::pair<double, bool>>&, 
	    					 const double&, const bool&, const bool&);
	void PathBoundsDebugString(const LaneBound& path_boundaries, const int& line);
protected:


private:
  
}; 

}  // namespace planning
}  // namespace acu