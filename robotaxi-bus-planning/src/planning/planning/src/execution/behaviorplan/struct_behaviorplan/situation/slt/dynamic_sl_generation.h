#ifndef DYNAMIC_SL_GENERATION_H
#define DYNAMIC_SL_GENERATION_H

#include "src/execution/behaviorplan/struct_behaviorplan/blackboard/decision_context.h"
#include "st_sampling/st_sampling.h"
#include <memory>

namespace acu {
namespace planning {

class DynamicSLGeneration {
 public:
  DynamicSLGeneration();
  ~DynamicSLGeneration();

  void Reset();

  // void GetSLBoundary();
  void GetExpandWidth();

  //API
  int CalculateDynamicBoxesWithCollision();

  std::vector<common::math::Box2d> GetResultDynamicObstacleBoxes() const {
  	return result_dynamic_obstacle_boxes_;
  }


 private:
 	DecisionContext* context_ = DecisionContext::Instance();
 	StructReferenceLineInfo* reference_line_;
 	ReferenceLineFrame* current_line_;
 	//障碍物-时间
  	std::vector<std::vector<common::math::Box2d>> dynamic_obstacle_boxes_;
 	//时间-障碍物 要求时间维度由小至大排列
  	std::vector<std::vector<common::math::Box2d>> dynamic_obstacle_boxes_in_time_;//存储所有box
  	std::vector<common::math::Box2d> result_dynamic_obstacle_boxes_;//存储所有发生干涉的box，时间维度不需要了

  	bool enable_debug_;
	//时间维度参数
	uint32_t num_of_time_stamps_;
	double eval_time_interval_;
	double total_time_;
	std::shared_ptr<STSampling> st_sampling_ptr_;


private:
	int SortObstacles();
	int SortObstaclesOfCertainFrame(const ReferenceLineFrame* line_frame_ptr);
	int OrganizeDynamicObsInTime();
	int CheckCollisionAndTrimBoxSet(const std::list<Site>& given_path);

	//以start_index为索引起点，找到given_path上距离time最近的时间点的索引
	template <class T, class ContainerT>
	double MatchTimeIndexBeginWithIndex(int& start_index, const double& time, 
										const ContainerT& given_path){
	double rtvalue = std::numeric_limits<double>::max();
	if(start_index >= given_path.size()) {return rtvalue;}
	const double kMatchBuff = 0.1;
	for(int path_index = start_index; path_index < given_path.size(); ++ path_index){
		auto iter = std::next(given_path.begin(), path_index);
		if(iter == given_path.end()) {break;}
		const T& point = *iter;
		auto time_bias = fabs(point.t - time);
		if(time_bias < 0.1){//勉强接受
			if(time_bias < 1e-2){//提前退出
				rtvalue = time_bias;
				start_index = path_index;
				break;
			}else{
				if(time_bias < rtvalue){//记录最小
					rtvalue = time_bias;
					start_index = path_index;
				}else if(time_bias > rtvalue + kMatchBuff){//退出
					break;
				}
			}
		}
	}
	return rtvalue;
}
};

}  //  namespace planning
}  //  namespace acu

#endif