#include "st_discretization.h"

namespace acu {
namespace planning {

STDdiscret::STDdiscret() {
	range_s_ = (int)((FLAGS_s_range)/FLAGS_scale_s);
	range_t_ = (int)(FLAGS_t_range/FLAGS_scale_t);
}

void STDdiscret::Londiscret(PredictionTrajectory &pd_line, double input_a, 
														double min_a, double max_a, bool is_following) {	
	auto &st_boundary = pd_line.st_boundary;
	object_v_ = pd_line.points.front().v;
	object_a_= pd_line.points.front().a;
	double center_a = object_a_;
	double center_p = pd_line.probability;
	pd_line.londiscrete_areas.clear();
	if (st_boundary.size() < 2) return;
	// 构建障碍物局部的map(正方形)，便于叠加
	area_min_s_ = 0, area_max_s_ = range_s_;
	area_min_s_ = RoundN(fmin(st_boundary.front().first.x(), st_boundary.back().first.x()),
											 FLAGS_scale_s);
	area_max_s_ = range_s_;
	area_min_s_ = fmax(area_min_s_, 0);
	area_max_s_ = fmax(area_min_s_ + 1, area_max_s_);
	pd_line.range_s.first = area_min_s_;// 初值
	pd_line.range_s.second = area_max_s_;// 初值

	obj_st_points_.clear();
	obj_st_points_.resize(range_t_ + 1);
	for (int i = 0; i < obj_st_points_.size(); i++) {
		obj_st_points_.at(i).resize(area_max_s_ - area_min_s_ + 1);
		for (int j = 0; j < obj_st_points_.at(i).size(); j++) {
			obj_st_points_.at(i).at(j).p = 0.0;
			obj_st_points_.at(i).at(j).t = i;
			obj_st_points_.at(i).at(j).s = j + area_min_s_;
		}
	}
	// 首先给定加速度离散范围（overlap_min_a,overlap_max_a）
	int input_state = 0;//0-不挤压，小范围离散 1-挤压 2-强挤压
	double overlap_min_a, overlap_max_a;
	if (input_a > -10.0) {
		center_a = input_a;
		center_a = ceil(center_a / 0.1) * 0.1;
		if (center_a > object_a_ + FLAGS_upper_acc + 0.01 || 
				center_a < object_a_ - FLAGS_upper_acc - 0.01) {// 强挤压，几乎不离散
			input_state = 2;
			overlap_max_a = floor(input_a / 0.1) * 0.1;
			overlap_min_a = overlap_max_a - 0.1;
		} else {
			input_state = 1;
			overlap_max_a = fmax(input_a, object_a_) + 0.1;
			overlap_min_a = fmin(input_a, object_a_) - 0.1;
			overlap_max_a = floor(overlap_max_a / 0.1) * 0.1;
			overlap_min_a = ceil(overlap_min_a / 0.1) * 0.1;
		}
		pd_line.range_a.first = overlap_min_a;
		pd_line.range_a.second = overlap_max_a;
	} else {
		input_state = 0;
		center_a = object_a_;
		overlap_max_a = fmin(FLAGS_max_pd_acc, object_a_ + FLAGS_upper_acc);
		overlap_min_a = fmax(-FLAGS_max_pd_acc, object_a_ - FLAGS_under_dec);
		pd_line.range_a.first = overlap_min_a;
		pd_line.range_a.second = overlap_max_a;
	}
	
	if (min_a > -10.0 && max_a > -10.0) {
		if (center_a < min_a || center_a > max_a) {
			AERROR<<"Input range a don't contain center a.";
			return;
		}
		overlap_max_a = max_a;
		overlap_min_a = min_a;
	}
	AERROR_IF(FLAGS_log_enable)<<"center a "<<center_a<<" acc "<<object_a_<<" v "
				<<object_v_<<" range a ("<<min_a<<", "<<max_a<<").  overlap a ("<<
				overlap_min_a<<", "<<overlap_max_a<<"). input_state "<<input_state;

	pd_line.st_points.clear();
	pd_line.londiscrete_areas.clear();
	double p_sum = 0.0;
	vector<pair<double, double>> a_p_func;
	double k0 = 1.0 / (center_a - overlap_min_a) / (center_a - overlap_min_a);
	double k1 = 1.0 / (center_a - overlap_max_a) / (center_a - overlap_max_a);
	for (double ai = overlap_max_a; ai > overlap_min_a; ai -= 0.1) {
		double fp = (ai <= center_a)? (k0*(ai-overlap_min_a)*(ai-overlap_min_a)) : 
																	(k1*(ai-overlap_max_a)*(ai-overlap_max_a));
		a_p_func.emplace_back(ai, fp);
		p_sum += fp;
	}
	for (auto &it : a_p_func) {
		it.second = it.second / p_sum;
	}
	for (auto &it : a_p_func) {
		LocalSTArea temp_area;
		temp_area.a = it.first;
		temp_area.p = it.second;
		// AINFO_IF(FLAGS_log_enable)<<"ai "<<it.first<<" p "<<it.second;
		if (it.second < 1e-4) continue;
		AccStBoundary(pd_line, it.first, temp_area.st_boundary, is_following);
		pd_line.londiscrete_areas.push_back(temp_area);
		if (input_state == 0 || input_state == 2) {
			if (fabs(it.first - center_a) < 0.1) {
				temp_area.p = center_p;
				// AINFO<<"center_p "<<center_p;
				OverlapAccStBoundary(temp_area);
				// break;
			}
		} else if (input_state == 1) {
			OverlapAccStBoundary(temp_area);
		}
	}
	for (auto &st_points : obj_st_points_) {
		for (auto &st_point : st_points) {
			if (st_point.p > 1e-4 && st_point.s >= 0 && 
				st_point.s <= (int)((FLAGS_s_range)/FLAGS_scale_s)) {
				st_point.p = fmin(st_point.p, 1.0);
				pd_line.st_points.push_back(st_point);
			}
		}
	}
}

bool STDdiscret::AccStBoundary(PredictionTrajectory &pd_line, double a, 
				vector<pair<Vec2d, Vec2d>> &sim_st_boundary, bool is_following) {
	sim_st_boundary.clear();
	if (pd_line.st_boundary.size() < 2) return true;
	for (int i = 0; i < pd_line.st_boundary.size(); i++) {
		auto &s_t = pd_line.st_boundary.at(i);
		auto &pd_s = pd_line.range_pd_s.at(i);
		double new_t = 0.0;
		if (FuncSA_T(pd_s, a, new_t)) {
			// AERROR_IF(FLAGS_log_enable)<<"s "<<pd_s<<" old t "<<s_t.second.y()<<" new t "<<new_t;
			Vec2d up(s_t.second.x(), new_t), down(s_t.first.x(), new_t);
			sim_st_boundary.emplace_back(down, up);
		} else if (is_following && !sim_st_boundary.empty()) {
			// 对于following障碍物的
			Vec2d up(sim_st_boundary.back().second.x(), s_t.second.y());
			Vec2d down(sim_st_boundary.back().first.x(), s_t.first.y()); 
			sim_st_boundary.emplace_back(down, up);
		}
	}
	if (is_following && sim_st_boundary.size() > 0 &&
			pd_line.st_boundary.back().first.y() > 7.0 &&
			sim_st_boundary.back().first.y() < 7.0) {
		auto &final_st = sim_st_boundary.back();
		auto sim_final_t = pd_line.st_boundary.back().first.y();
		double down_s = object_v_ * (sim_final_t - final_st.first.y()) + 
							 			final_st.first.x();
		double up_s = object_v_ * (sim_final_t - final_st.second.y()) + 
							 		final_st.second.x();
		AERROR_IF(FLAGS_log_enable)<<"Sim follow st, ("<<down_s<<", "<<up_s<<", "<<sim_final_t<<").";
		Vec2d up(up_s, sim_final_t), down(down_s, sim_final_t);					 					 			
		sim_st_boundary.emplace_back(down, up);
	}
	return true;
}

bool STDdiscret::FuncSA_T(double s, double a, double &t) {
	t = 0.0;
	if (fabs(a) < 1e-2) {
		t = s / object_v_;
		return true;
	}
	if (a < 0.0 && object_v_ + a * FLAGS_break_t <= 1e-3) {
		if (0.5 * object_v_ * object_v_ / fabs(a) < s) {
			return false;
		}
		t = (sqrt(2.0 * a * s + object_v_ * object_v_) - object_v_) / a;
		return true;
	}
	t = (s + 0.5 * a * FLAGS_break_t * FLAGS_break_t) / 
			(object_v_ + a * FLAGS_break_t);
	return true;
}

void STDdiscret::OverlapAccStBoundary(LocalSTArea &input_area) {
	vector<ST> temp_stpoints;
	st_common_.RoundST(input_area.st_boundary, temp_stpoints, input_area.p);
	for (auto &point : temp_stpoints) {
		if (point.t < obj_st_points_.size()) {
			int delta_s = point.s - area_min_s_;
			if (delta_s >= 0 && delta_s < obj_st_points_.at(point.t).size()) {
				obj_st_points_.at(point.t).at(delta_s).p += point.p;// 还是叠加吧
			}
		} 
	}
}



void STDdiscret::Latdiscret(const MapEngineLineList *line_data_ptr, double &min_l,
														PredictionTrajectory &pd_line, int type) {
	if (!pd_line.st_boundary.empty()) {
		st_common_.RoundST(pd_line);
		return;
	}
	double p = 1.2 - min_l;
	p = fmin(p, 0.8);
	p = fmax(p, 0.0);
	// p = fmax(p, 0.4);// for test
	Paralleldiscret(line_data_ptr, pd_line, p);
}

void STDdiscret::Paralleldiscret(const MapEngineLineList *line_data_ptr, 
														PredictionTrajectory &pd_line, double &p) {
	int start_id = -1, end_id = -1;
	for (int i = 0; i < pd_line.points.size(); i++) {
		bool same_road = false;
		int pd_lane_index = pd_line.points.at(i).lane_id.back() - '0';
		for (auto &lane : line_data_ptr->front_lane_ids) {
			if (LaneToRoad(lane) == LaneToRoad(pd_line.points.at(i).lane_id)) {
				int lane_index = lane.back() - '0';
				if (fabs(lane_index - pd_lane_index) > 1) continue;
				same_road = true;
				break;
			}
		}
		if (same_road) {
			if (start_id < 0) start_id = i;
			end_id = i;
		}
	}
	if (start_id < 0 || end_id - start_id < 3) {
		AWARN_IF(FLAGS_log_enable)<<"Parallel obj has no parallel lane with reference frame.";
		return;
	}
	AINFO_IF(FLAGS_log_enable)<<"pd same road start id "<<start_id<<" end id "<<end_id;
	auto &pd_start = pd_line.points.at(start_id);
	auto &pd_end = pd_line.points.at(end_id);
	double min_dis0 = 1000.0, min_dis1 = 1000.0;
	int min_index = -1, max_index = -1;
	for (int i = 0; i < line_data_ptr->path_points.size(); i+=5) {
		auto &pathpoint = line_data_ptr->path_points.at(i);
		double dis = hypot(pd_start.xg - pathpoint.xg, pd_start.yg - pathpoint.yg);
		if (dis < min_dis0) {
			min_dis0 = dis;
			min_index = i;
		}
		if (dis > min_dis0 + 5.0) break;
	}
	if (min_index < 0) return;
	for (int i = min_index; i < line_data_ptr->path_points.size(); i+=5) {
		auto &pathpoint = line_data_ptr->path_points.at(i);
		double dis = hypot(pd_end.xg - pathpoint.xg, pd_end.yg - pathpoint.yg);
		if (dis < min_dis1) {
			min_dis1 = dis;
			max_index = i;
		}
		if (dis > min_dis1 + 5.0) break;
	}
	if (max_index < min_index + 3) return;
	AINFO_IF(FLAGS_log_enable)<<"reference line min_index "<<min_index<<" max_index "<<max_index;
	pd_line.st_boundary.clear();//原本应该也是空的
	pd_line.st_points.clear();
	Vec2d start_up(line_data_ptr->path_points.at(min_index).length+FLAGS_front_buff, pd_start.t);
	Vec2d start_down(line_data_ptr->path_points.at(min_index).length-FLAGS_front_buff, pd_start.t);
	pd_line.st_boundary.emplace_back(start_down, start_up);
	Vec2d end_up(line_data_ptr->path_points.at(max_index).length+FLAGS_front_buff, pd_end.t);
	Vec2d end_down(line_data_ptr->path_points.at(max_index).length-FLAGS_front_buff, pd_end.t);
	pd_line.st_boundary.emplace_back(end_down, end_up);
	st_common_.RoundST(pd_line.st_boundary, pd_line.st_points, p);
}



}
}