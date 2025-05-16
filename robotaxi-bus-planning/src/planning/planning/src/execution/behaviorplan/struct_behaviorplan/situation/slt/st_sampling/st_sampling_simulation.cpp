// #include "st_sampling_simulation.h"

// namespace acu {
// namespace planning {


// class STSamplingSim{
// public:
// 	STSamplingSim() {
// 		st_sampling_ptr_ = std::make_shared<STSampling>();
// 		Reset();
// 	}

// 	~STSamplingSim() {}

// 	void 
// 	Reset() {}

// 	void Start(){
// 		float t0 = 0.0, s0 = 0.0, v0 = 2.5;
// 		float t_max = 5.0, s_max = 5.0; 
// 		float acc = 1.0;
// 		float out_s, out_t, out_v;
// 		for(float time = 0.0; time < 10.0; time += 0.5){
// 			st_sampling_ptr_->GenerateSTPath(t0, s0, v0, t_max, s_max, acc, time,
// 											 out_s, out_t, out_v);
// 			AINFO<<"t: "<<time<<", out_s: "<<out_s<<", out_t: "<<out_t<<", out_v: "<<out_v;
// 		}
// 	}

// private:
// 	std::shared_ptr<STSampling> st_sampling_ptr_;
// };




// }  //  namespace planning
// }  //  namespace acu
