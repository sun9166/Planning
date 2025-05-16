#ifndef SRC_EXECUTION_COGNITION_LINEPROCESS_H_
#define SRC_EXECUTION_COGNITION_LINEPROCESS_H_
#include "conf/cognition_gflags.h" 
#include "cognition_typedef.h"

using geometry::Site;
using geometry::SiteVec;
using namespace std;

namespace acu{
namespace planning {

class LineProcess {
public:
	LineProcess(const MapInfoData &mapengine_data, 
				const vector<string> &last_current_lanes,
				const vector<string> &last_target_lanes,
				const PathData &motionpath,
				const int drive_state, 
				LocalizationData &localization, CarModel &car_model);

	bool CorrectionProcess(int &calculate_index, int &target_index);
	void LineClassification(StructReferenceLineInfo &reference_lines);

private:
	const vector<MapEngineLineList>* all_lines_ptr_;
	const MapEngineLineList* rev_line_ptr_;
	LocalizationData localization_;
	const PathData *motionpath_ptr_;
	CarModel car_model_;
	int mapengine_index_;
	vector<string> reference_lane_ids_;
	vector<string> reference_target_ids_;
	int correct_index_;
	int target_index_;
	int drive_state_;
	bool task_change_;

private:
  	bool LaneIdsMatching(vector<string> current_lanes, vector<string> last_lanes);
  	bool IsDiversionLine(vector<string> current_lanes, vector<string> last_lanes, 
  						 int &delta_index, int &devide_index);
  	void CheckArriveTargetLine();
  	void MatchingFrame(vector<ReferenceLineFrame>& last_frames, 
					   const pair<int, int> &index_range); 
  	void RevLineFrame(ReferenceLineFrame &reverse_reference_line);
	string LaneToRoad(string &lane);

};

} // namespace planning
} // namespace acu


#endif
