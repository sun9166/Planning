#ifndef PASSABLE_PATH_SEARCH_H
#define PASSABLE_PATH_SEARCH_H

#define MAX_COST 100.0
#include "src/execution/behaviorplan/struct_behaviorplan/blackboard/decision_context.h"

namespace acu {
namespace planning {

class PassablePathSearch {
 public:
  PassablePathSearch();
  ~PassablePathSearch();

  void Init();
  bool GeneratePath(std::vector<int>& passable_ref_lane_ids);
  void AddLineInfo();
  void GetInitState();
  void BuildStaticMap();
  void GetPathBound(const double s, double& left_bd, double& right_bd);
  void AddSampleLevel();
  void AddSamplePoints();
  void AddPointLineIds(const int id, MapPointStruct& point);
  bool SearchPath();
  bool SatisfyConstrain(const MapPointStruct& pre_point, 
      const MapPointStruct& point, int& lc_num);
  int LineLcNumber(const int line_id);
  bool GetPathAndBoundary(std::vector<MapPointStruct>& points,PathBound& bound);
  void GeneratePath(std::map<int, ReferenceLineFrame*> lines);
  bool CheckDestPassable(const int line_id, const double s);
  bool BackTrack(std::vector<MapPointStruct> farest_points);
  void GetSearchS(std::map<int,double>& search_s);

 private:

  int left_bd_, right_bd_;
  double start_s_, start_l_, start_yaw_;
  double min_s_ = 5.0, level_s_, half_width_, buffer_ = 0.4;
  PathBound path_bound_;
  std::vector<std::vector<MapPointStruct>> points_;
  std::vector<std::vector<MapPointStruct>> sample_points_;
  std::vector<MapPointStruct> farest_points_;
  std::map<double, int> static_objects_;
  std::vector<std::pair<double, int>> sample_s_;
  std::map<int, int> search_index_;

  std::map<int,int> left_to_right_line_idx_; 

  DecisionContext* context_ = DecisionContext::Instance();
  PlanningMainData* DP_ = DataPool::Instance()->GetMainDataPtr();
  StructReferenceLineInfo* reference_line_;
  ReferenceLineFrame* current_line_;
  // ReferenceLineFrame* target_line_;
  std::map<int, ReferenceLineFrame*> lines_;
  bool road_side_;
  bool pull_over_ = false;
  std::set<int> passable_refs_;
};
}  //  namespace planning
}  //  namespace acu

#endif
