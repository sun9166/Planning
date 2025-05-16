#ifndef SRC_EXECUTION_COGNITION_STRUCT_COGNITION_NEW_PATH_H_
#define SRC_EXECUTION_COGNITION_STRUCT_COGNITION_NEW_PATH_H_

#include "public_typedef.h"
#include "mapengine_input.h"
#include "prediction_input.h"
#include "locperception_input.h"
#include "common_config.h"
#include "conf/cognition_gflags.h"
#include "common/toolbox/geometry/include/geoheader.h"
#include "base_func/base_func.h"
#include "referenceline_frame/referenceline_frame.h"
#include <algorithm>

using geometry::Site;
using geometry::SiteVec;
using namespace std;
using namespace acu::common::math;

namespace acu{
namespace planning {

class NewPathInfo {
public:
	bool MotionPathToFrame(PathData &local_path, const ReferenceLineFrame &target_line_frame,
												 const ReferenceLineFrame &current_line_frame, LocalizationData &localization,
                         CarModel &car_model, ReferenceLineFrame &local_line);
	bool UpdateAllDistance(ReferenceLineFrame &update_frame);
	bool UpdateS(const double &old_s, double &new_s);
private:
	const ReferenceLineFrame* target_line_ptr_;
	SiteVec* new_path_ptr_;
};


}
}

#endif