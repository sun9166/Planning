#pragma once

#include "st_sampling.h"

namespace acu {
namespace planning {


class STSamplingSim{
public:
	STSamplingSim() {
		st_sampling_ptr_ = std::make_shared<STSampling>();
		Reset();
	}

	~STSamplingSim() {}

	void 
	Reset() {}

	void Start(){
		float t0 = 20.0, s0 = 30.0, v0 = 0;
		float t_max = 35.0, s_max = 500.0; 
		float acc = 0.5;
		float out_s, out_t, out_v;
		for(float time = t0; time < 100.0; time += 0.1){
			auto rt = st_sampling_ptr_->GenerateSTPath(t0, s0, v0, t_max, s_max, acc, time,
											 out_s, out_t, out_v);
			cout<<time<<" "<<rt<<" "<<out_s<<" "<<out_t<<" "<<out_v<<endl;
		}
	}

private:
	std::shared_ptr<STSampling> st_sampling_ptr_;
};

}  //  namespace planning
}  //  namespace acu
