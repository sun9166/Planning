#ifndef STATIC_MAP_SEARCH_H
#define STATIC_MAP_SEARCH_H

#define LEVEL_S 10.0
#define MAX_COST 100.0
#include "src/execution/behaviorplan/struct_behaviorplan/blackboard/decision_context.h"

namespace acu {
namespace planning {

class StaticMapSearch {
 public:
  StaticMapSearch();
  ~StaticMapSearch();

  void GenerateGrid(const bool is_same, const int direction, OptionStruct& option);
  void GeneratePath(const bool is_same, const int direction, OptionStruct& option);
  void BoundaryDecision(const int direction, const MapNodeStruct& last_node,
                        OptionStruct& option);
  bool CheckCollision(const MapNodeStruct& last_node, 
                      const MapNodeStruct& cur_node, int& cost);
  bool CheckPolygon(const MapNodeStruct& last_node, 
                    const MapNodeStruct& cur_node, LineObject& object);
  void AddBoundary(const int direction, const double current_len, 
      						 const double left_len, const double right_len);
  void AddPoints(const int direction, MapNodeStruct& point, 
                 std::vector<MapNodeStruct>& points);
  bool LcEarly(const MapNodeStruct& last_node, const MapNodeStruct& cur_node);

 private:
 	DecisionContext* context_ = DecisionContext::Instance();
 	StructReferenceLineInfo* reference_line_;
 	ReferenceLineFrame* current_line_;
  ReferenceLineFrame* left_line_;
  ReferenceLineFrame* right_line_;
 	std::vector<std::vector<MapNodeStruct>> points_;
 	std::vector<std::vector<MapNodeStruct>> graph_nodes_;
 	double length_, half_width_, buffer_, front_over_hang_, back_over_hang_;
};

}  //  namespace planning
}  //  namespace acu

#endif