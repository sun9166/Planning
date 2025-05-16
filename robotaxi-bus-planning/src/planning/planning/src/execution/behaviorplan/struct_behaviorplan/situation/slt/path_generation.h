#ifndef PATH_SEARCH_H
#define PATH_SEARCH_H

#define MAX_COST 100.0
#include "src/execution/behaviorplan/struct_behaviorplan/blackboard/decision_context.h"

namespace acu {
namespace planning {

class PathGeneration {
 public:
  PathGeneration();
  ~PathGeneration();

  bool GeneratePath();
  void AddLineInfo();
  void GetInitState();
  void BuildStaticMap();
  void GetPathBound(const double s, double& left_bd, double& right_bd);
  void AddSampleLevel();
  void AddSamplePoints();
  void AddPointLineIds(const int id, MapPointStruct& point);
  bool SearchPath();
  bool GetTotalCost(const MapPointStruct& pre_point, const MapPointStruct& point, double& cost);
  void GetExpandL();

 private:

  int left_bd_, right_bd_;
  double start_s_, start_l_, start_yaw_;
  double min_s_ = 5.0, level_s_, half_width_, width_, length_, front_over_hang_;
  PathBound path_bound_;
  std::vector<std::vector<MapPointStruct>> points_;
  std::vector<std::vector<MapPointStruct>> sample_points_;
  std::map<double, int> static_objects_;
  std::vector<std::pair<double, int>> sample_s_;
  std::map<int, int> search_index_;
  std::vector<Box2d> static_boxes_;

  DecisionContext* context_ = DecisionContext::Instance();
  PlanningMainData* DP_ = DataPool::Instance()->GetMainDataPtr();
  StructReferenceLineInfo* reference_line_;
  ReferenceLineFrame* current_line_;
  ReferenceLineFrame* target_line_;
  std::map<int, ReferenceLineFrame*> lines_;
  bool road_side_;
};

}  //  namespace planning
}  //  namespace acu

#endif
