#ifndef ST_SAMPLING_H
#define ST_SAMPLING_H

#include "src/execution/behaviorplan/struct_behaviorplan/blackboard/decision_context.h"

namespace acu {
namespace planning {

class STSampling {
 public:
	STSampling();
	~STSampling();

	void Reset();
	int SampleDECCToGeneratePaths(const std::list<Site>& given_path, 
							 	std::vector<std::list<Site>>& path_vec);
	int 
	GenerateSTPath(const float& t0, const float& s0, const float& v0,
					const float& t_max, const float& s_max,
					const float& acc, const float& time,
					float& out_s, float& out_t, float& out_v);
 private:
 	DecisionContext* context_ = DecisionContext::Instance();
  	bool enable_debug_;
	//时间维度参数
	uint32_t num_of_time_stamps_;
	double eval_time_interval_;
	double total_time_;

	float v_max_;//搜索st时的限制

private:
	inline float LimitVelocity(const float& v) const {
		auto tmp_v = (v > 0.0) ? v : 0.0;
		tmp_v = (v < v_max_) ? v : v_max_;
		return tmp_v;
	}

	inline bool IsValidTime(const float& time) const {
		return (time > 0 && time < 1e5);
	}
};

}  //  namespace planning
}  //  namespace acu

#endif