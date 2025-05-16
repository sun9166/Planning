#include "st_sampling.h"

namespace acu {
namespace planning {

STSampling::STSampling() {
	Reset();
}

STSampling::~STSampling() {}

void 
STSampling::Reset() {
  	enable_debug_ = FLAGS_enable_dynamic_sl_debug_plot;
	//时间维度参数
	num_of_time_stamps_ = 0;
	eval_time_interval_ = 0.1;
	total_time_ = 7.0;
	v_max_ = 50.0; 
}


//基于given_path做加速度采样，得到一系列的采样结果路径path_vec
int 
STSampling::SampleDECCToGeneratePaths(const std::list<Site>& given_path, 
									 std::vector<std::list<Site>>& path_vec){
	int rtvalue = -1;
	if(given_path.empty()) {return rtvalue;}
	const double stitching_time = 0.1;//s
	//在given_path上找到粘合线终点
	int i = 0;
	for(; i < given_path.size(); ++i){
		if(std::next(given_path.begin(), i)->velocity < 0.05) break;
		if(std::next(given_path.begin(), i)->t > stitching_time) break;
	}
	//采样起始值
	const auto start_point = *std::next(given_path.begin(), i);
	const auto start_acc = start_point.a;
	const auto start_vel = start_point.velocity;
	const auto start_t = start_point.t;
	const int start_index = i;
	AINFO_IF(enable_debug_)<<"sample start with t: "<<start_t
			<<", v: "<<start_vel<<", a: "<<start_acc<<", index: "<<start_index;
	path_vec.clear();
	//加速度采样
	const std::vector<double> initial_acc_vec = {-0.5, -1.0, -1.5};
	for(auto sample_acc : initial_acc_vec){
		auto sample_path = given_path;
		auto iter = next(sample_path.begin(), start_index);
		while(iter != sample_path.end() && next(iter) != sample_path.end()){
			auto delta_length = hypot(iter->x - next(iter)->x, iter->y - next(iter)->y);
			//s = v*t + 0.5*a*t^2
			//求根公式
			auto time1 = ((-1) * iter->velocity + pow(pow(iter->velocity, 2) + 
						2 * sample_acc * delta_length, 0.5)) / sample_acc;
			auto time2 = ((-1) * iter->velocity - pow(pow(iter->velocity, 2) + 
						2 * sample_acc * delta_length, 0.5)) / sample_acc;
			auto time = -1;
			if(time1 < 0 && time2 < 0){
				AERROR_IF(enable_debug_)<<"ERROR LOGIC for sample acc";
				break;
			}
			if(time1 < time2){
				time = (time1 < 0) ? time2 : time1;
			}else{
				time = (time2 < 0) ? time1 : time2;
			}
			DCHECK_GT(time, 0);
			next(iter)->t = time;//根据点间隔距离，更新时间
			next(iter)->velocity = max(iter->velocity + sample_acc * time, 0.0);//根据点间隔距离，更新速度
			++iter;
		}
		path_vec.emplace_back(sample_path);
	}
	return rtvalue;
}

/*
  smax	-------------------
		|                *  |
		|              *    |
		|             *     |
  out_s	|----------- *		|
		|		  *	 |		|
		|		*  	 |		|
		|	*	   	 |		|
		|*___________|______|____
	 (s0, t0)       time   tmax        

输入起始状态(t0, s0, v0)，以给定acc计算st曲线，曲线到达tmax或smax结束。`
该函数获取给定时间time位置的(s, t, v)

case1：曲线达到s上限结束，输出的t和给定的time有可能不同，该情况下out_t为曲线真正到达s上限的时间。
case2：曲线达到t上限结束，输出的t和给定的time相同。

所有输入、输出都是相对于原点的绝对值。
返回值：-1 失败；0 未到达任何上限；1 到达S上限；2 到达T上限
*/
int 
STSampling::GenerateSTPath(const float& t0, const float& s0, const float& v0,
						   const float& t_max, const float& s_max,
						   const float& acc, const float& time,
						   float& out_s, float& out_t, float& out_v){
	int rtvalue = -1;
	if(time > t_max){
		AERROR_IF(enable_debug_)<<"invalid input time & tmax: "<<time<<" > "<<t_max;
		return rtvalue;
	}
	if(time < t0){
		AERROR_IF(enable_debug_)<<"invalid input time & t0: "<<time<<" < "<<t0;
		return rtvalue;
	}
	if(s0 > s_max){
		AERROR_IF(enable_debug_)<<"invalid input s0 & smax: "<<s0<<" > "<<s_max;
		return rtvalue;
	}
	if(v0 > v_max_){
		AERROR_IF(enable_debug_)<<"invalid input v0 & vmax: "<<v0<<" > "<<v_max_;
		return rtvalue;
	}

	//计算匀加速度到达最大速度限制的时间time_to_max_v
	auto time_to_max_v = (v_max_ - v0) / acc;
	if(!IsValidTime(time_to_max_v)) {
		if(acc > 0){
			AERROR_IF(enable_debug_)<<"invalid time_to_max_v: "<<time_to_max_v<<", acc: "<<acc
									<<", v0: "<<v0<<", v_max_: "<<v_max_<<", time: "<<time;
			return rtvalue;
		}
		time_to_max_v = std::numeric_limits<float>::max();
	}

	//计算匀加速度到达最大距离限制的时间time_to_max_s
	auto v_end_square = 2 * acc * (s_max - s0) + pow(v0, 2);
	if(v_end_square < 0){
		if(acc > 0){
			AERROR_IF(enable_debug_)<<"invalid v_end_square: "<<v_end_square<<", acc: "<<acc
								<<", s_max: "<<s_max<<", s0: "<<s0<<", v0: "<<v0;
			return rtvalue;
		}
		v_end_square = std::numeric_limits<float>::max();
	}
	auto time_to_max_s = (pow(v_end_square, 0.5) - v0) / acc;

	//计算减速到速度为0的时间
	auto time_to_zero_velocity = std::numeric_limits<float>::max();
	// auto zero_velocity_s = std::numeric_limits<float>::max();
	if(acc < 0){
		time_to_zero_velocity = (0.0 - v0) / acc;
		// zero_velocity_s = v0 * time_to_zero_velocity + 0.5 * acc * pow(time_to_zero_velocity, 2);
	}

	//不超过速度降为零的时间
	auto tmp_time = (time < time_to_zero_velocity) ? time : time_to_zero_velocity;
	auto tmp_out_v = v0 + acc * tmp_time;
	auto tmp_out_s = s0 + v0 * tmp_time + 0.5 * acc * tmp_time * tmp_time;

	//达到速度上限，不可用匀加速度模型计算距离，修正
	if(tmp_time > time_to_max_v){
		tmp_out_s = (pow(v_max_, 2) - pow(v0, 2)) / (2 * acc) + s0;
		tmp_out_s += v_max_ * (tmp_time - time_to_max_v);
	}
	
	//输出时间约束
	//case1
	if(tmp_out_s > s_max){//求到达smax的时间
		if(time_to_max_s < time_to_max_v){//行驶至smax的时间小于行驶至vmax的时间，所以一定是以匀加速到达的smax
			if(!IsValidTime(time_to_max_s)){
				AERROR_IF(enable_debug_)<<"invalid time_to_max_s: "<<time_to_max_s;
				return rtvalue;
			}
			out_t = time_to_max_s;
		}else{//匀加速行驶time_to_max_v时长，后匀速行驶至smax
			//匀速行驶到smax的时间
			auto time_with_max_v = (s_max - (s0 + v0 * time_to_max_v + 0.5 * acc * pow(time_to_max_v, 2))) / v_max_;
			if(!IsValidTime(time_with_max_v) || !IsValidTime(time_to_max_v)){
				AERROR_IF(enable_debug_)<<"invalid time_with_max_v: "<<time_with_max_v<<", time_to_max_v: "<<time_to_max_v;
				return rtvalue;
			}
			out_t = time_with_max_v + time_to_max_v;
		}
		rtvalue = 1;
		tmp_out_s = s_max;
	}else{//case0 & case2
		rtvalue = (time > t_max - 1e-5) ? 2 : 0;
		out_t = time;
		if(2 == rtvalue){//case2限制返回时间为tmax，距离重新赋值
			out_t = t_max;
			// tmp_out_s = s0 + v0 * out_t + 0.5 * acc * out_t * out_t;
		}
	}

	//其他输出结果约束
	out_s = tmp_out_s;
	out_v = LimitVelocity(tmp_out_v);

	return rtvalue;
}


/*
给定起点，分区域调用GenerateSTPath，生成一系列ST采样路径
*/
// int 
// STSampling::GenerateSTSamplePaths(const float& t0, const float& s0, const float& v0,
// 						   const float& t_max, const float& s_max, vectro<list<Site>>& out_paths){
// 	int rtvalue = -1;

// 	return rtvalue;
// }



}  //  namespace planning
}  //  namespace acu
