#include "datapool/include/data_pool.h"

namespace acu{
namespace planning {

class SemanticProjection {
 public:
	SemanticProjection();
	~SemanticProjection();
	void AddSemanticInfo(ReferenceLineFrame& reference_line);
 private:
 	StructEnv* DP_ = &DataPool::Instance()->GetMainDataRef().cognition_info.struct_env_info;
 	vectormap::VectorMap* vectormap_ = map::MapLoader::GetVectorMapPtr();
 	ReferenceLineFrame* reference_line_;
 	ReferenceLineFrame* target_line_;
 	std::set<string> jam_junction_ids_;
 	std::set<int> follow_objects_;
 	std::set<int> park_objects_;
 	void AddBoundaryInfo();
 	void AddQueueInfo();
 	void AddBlockInfo();
	void AddSpeedInfo();
	void AddSafetyInfo();
	void AddReverseInfo();
	void AddConflictInfo();
	void AddPassableInfo();
	bool IsTempCollision();
	void AddBlindDis();
	void GetBlockL();
	void GetJamLevel();
	void GetQueueLength();
	void GetFrontSpeed();
	void AddSpeedCost(int direction = 0);
	double GetPolygonL(const LineObject& object);
	void LineLimitSpace();
	void SortObjects(vector<ObstacleEdge>& sorted_objects);
	void InitBoundary(const vector<ObstacleEdge>& sorted_objects, LaneBound& line_boundary);
	void CalculateBoundary(const vector<ObstacleEdge>& sorted_objects,
	                  	   LaneBound& line_boundary, int &block_obejct_id);
	bool UpdateBoundaryAndCenterLine(int idx, const double &left_w, const double &right_w,
                                	LaneBound& line_boundary, double &center_line);
	void FindFollowObjects();
};


}
}