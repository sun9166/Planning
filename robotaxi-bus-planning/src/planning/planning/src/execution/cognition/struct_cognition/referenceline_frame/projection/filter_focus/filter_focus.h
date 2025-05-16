#include "datapool/include/mapengine_input.h"
#include "datapool/include/locperception_input.h"
#include "referenceline_frame/frame_base/frame_base.h"

using namespace std;

namespace acu{
namespace planning {

class FrameFocus {
 public:
	void FindFocusObstacles(ReferenceLineFrame &reference_line, const CarModel *car_model_ptr);

 	ReferenceLineFrame* reference_line_ptr_;
 	const CarModel *car_model_ptr_;

 private:
	void InitFocusLevel();
	void SortObstacles();
	void FilterFrontObstacles(int &max_focus_index);
	void FilterBackObstacles();
	void SortRelationLanes(vector<RelationLane> &relation_lanes, bool is_front);
	void GetFocusLevel(int &max_focus_index);
};


}
}