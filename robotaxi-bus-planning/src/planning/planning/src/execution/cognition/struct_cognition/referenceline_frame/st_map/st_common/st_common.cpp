#include "st_common.h"

namespace acu {
namespace planning {

STCommon::STCommon() {
}

void STCommon::GetLineBoxes(const CarModel *car_model_ptr, const SiteVec &path_points, 
														vector<BoxStruct> &line_boxes) {
	line_boxes.clear();
	auto &front_l = car_model_ptr->front_over_hang;
	auto &back_l = car_model_ptr->back_over_hang;
	auto &width = car_model_ptr->car_width;
	for (int i = 0; i < path_points.size(); i++) {
		auto &point = path_points.at(i);
		if (line_boxes.empty() || 
				point.length - line_boxes.back().length >= FLAGS_scale_s) {
			double pi_angle = point.globalangle * M_PI / 180.0;
			Vec2d center = Vec2d((front_l - back_l)/2.0, 0.0).rotate(pi_angle) +
										 Vec2d(point.xg, point.yg);
			Box2d temp_box(center, pi_angle, front_l + back_l, width);
			line_boxes.push_back(BoxStruct(temp_box, point.length, 0.0));
		}
		if (point.length > FLAGS_s_range + 10.0) break;
	}
}

void STCommon::GetPdBoxes(const PredictionTrajectory &pd_trajectory, const double &obj_length, 
													const double &obj_width, const bool &is_reverse_traveling,
													vector<BoxStruct> &pd_boxes) {
	pd_boxes.clear();
	for (auto &pd_point : pd_trajectory.points) {
		Vec2d center(pd_point.xg, pd_point.yg);
		double pi_angle = pd_point.globalangle * M_PI / 180.0;
		Box2d temp_box(center, pi_angle, obj_length, obj_width);
		pd_boxes.push_back(BoxStruct(temp_box, pd_point.x, pd_point.t));
		if (pd_point.t > FLAGS_t_range) break;
		if (pd_trajectory.lane_ids.size() == 1 && pd_point.t > FLAGS_t_range/2.0 &&
				pd_trajectory.lane_ids.front() == "PEDESTRIAN" && pd_point.x > 6.0) {
			break;
		}
		if (is_reverse_traveling && pd_point.t > FLAGS_t_range/2.0 && pd_point.x > 6.0) {
			break;
		}
	}
}

void STCommon::BoxST(const vector<BoxStruct> &pd_boxes,  const vector<BoxStruct> &line_boxes,  
										 vector<double> &range_pd_s, vector<pair<Vec2d, Vec2d>> &st_boundary) {
	st_boundary.clear();
	range_pd_s.clear();
	if (line_boxes.empty() || pd_boxes.empty()) return;
	bool overlap_flag = false;
	double min_s = 0.0, max_s = 0.0;
	for (int i = 0; i < pd_boxes.size(); i++) {
		for (int j = 0; j < line_boxes.size(); j++) {
			if (line_boxes[j].box.HasOverlap(pd_boxes[i].box)) {
				// if (!overlap_flag) {
				// 	range_pd_s.first = pd_boxes[i].length;
				// }
				overlap_flag = true;
				min_s = line_boxes[j].length;
				double range_s = line_boxes[j].box.length() + 
												 line_boxes[j].box.width() +
								   	 		 pd_boxes[i].box.length() + 
								   	 		 pd_boxes[i].box.width() +
								   	 		 line_boxes[j].length;
				for (int k = j; k < line_boxes.size(); k++) {
					if (line_boxes[k].length > range_s) break;
					if (line_boxes[k].box.HasOverlap(pd_boxes[i].box)) {
						max_s = line_boxes[k].length;
					}
				}
				max_s = fmax(max_s, min_s + 0.1);
				Vec2d up(max_s, pd_boxes[i].t), down(min_s, pd_boxes[i].t);
				range_pd_s.push_back(pd_boxes[i].length);
				st_boundary.emplace_back(down, up);
				break;
			}
		}
	}
	if (st_boundary.size() <= 1) return;
	SegmentLineFit(st_boundary, range_pd_s);
	for(auto &s_t : st_boundary) {
		AERROR_IF(FLAGS_log_enable) <<"cognition st ("<<s_t.first.x()
								<<", "<<s_t.second.x()<<", "<<s_t.first.y()<<")";
		
	}
}

void STCommon::RoundST(PredictionTrajectory &pd_trajectory) {
	auto &st_boundary = pd_trajectory.st_boundary;
	auto &st_points = pd_trajectory.st_points;
	auto &p = pd_trajectory.probability;
	RoundST(st_boundary, st_points, p);
}

void STCommon::RoundST(vector<pair<Vec2d, Vec2d>> &st_boundary, 
											 vector<ST> &st_points, double p) {
	st_points.clear();
	if (st_boundary.size() < 2) return;
	for (int i = 1; i < st_boundary.size(); i++) {
		double down_bd_k, down_bd_b, up_bd_k, up_bd_b;
		int front_t, back_t;
		STBoundaryToLine(st_boundary.at(i-1), st_boundary.at(i), up_bd_k, up_bd_b, 
										 down_bd_k, down_bd_b, front_t, back_t);
		// AINFO<<"up_bd_k "<<up_bd_k<<" up_bd_b "<<up_bd_b
		// 			<<" down_bd_k "<<down_bd_k<<" down_bd_b "<<down_bd_b
		// 			<<" front t "<<front_t<<" back_t "<<back_t;
		for (int t = max(0, front_t); t <= back_t; t++) {
			if (i + 1 < st_boundary.size() && t == back_t) break;
			int min_s = floor(down_bd_k * t + down_bd_b);
			int max_s = ceil(up_bd_k * t + up_bd_b);
			for (int s = max(min_s, 0); s <= max_s; s++) {
				st_points.emplace_back(s, t, p); 
			}
		}
	}
}

void STCommon::SegmentLineFit(vector<pair<Vec2d, Vec2d>> &st_boundary, 
															vector<double> &pd_s) {
	if (st_boundary.size() < 3) return;
	vector<double> min_list_s, max_list_s, list_t;
	for (auto &st : st_boundary) {
		min_list_s.push_back(st.first.x());
		max_list_s.push_back(st.second.x());
		list_t.push_back(st.second.y());
	}
	vector<int> min_indexes, max_indexes, final_indexes;
	FindBreakIndex(list_t, min_list_s, (int)st_boundary.size()-1, min_indexes);
	FindBreakIndex(list_t, max_list_s, (int)st_boundary.size()-1, max_indexes);
	while(min_indexes.size() > 0 || max_indexes.size() > 0) {
		if (min_indexes.empty()) {
			final_indexes.insert(final_indexes.end(),
				max_indexes.begin(), max_indexes.end());
			max_indexes.clear();
			break;
		}
		if (max_indexes.empty()) {
			final_indexes.insert(final_indexes.end(),
				min_indexes.begin(), min_indexes.end());
			min_indexes.clear();
			break;
		}
		if (min_indexes.front() < max_indexes.front()) {
			final_indexes.push_back(min_indexes.front());
			min_indexes.erase(min_indexes.begin());
		} else if (min_indexes.front() == max_indexes.front()) {
			final_indexes.push_back(min_indexes.front());
			min_indexes.erase(min_indexes.begin());
			max_indexes.erase(max_indexes.begin());
		} else {
			final_indexes.push_back(max_indexes.front());
			max_indexes.erase(max_indexes.begin());
		}
	} 
	vector<double> simplify_pd_s;
	vector<pair<Vec2d, Vec2d>> simplify_st;
	simplify_st.push_back(st_boundary.front());
	simplify_pd_s.push_back(pd_s.front());
	for (int i = 0; i < final_indexes.size(); i++) {
		// AINFO<<"final seg index "<<final_indexes.at(i);
		simplify_st.push_back(st_boundary.at(final_indexes.at(i)));
		simplify_pd_s.push_back(pd_s.at(final_indexes.at(i)));
	}
	pd_s = simplify_pd_s;
	st_boundary = simplify_st;
}

void STCommon::FindBreakIndex(vector<double> &list_t, vector<double> &list_s, 
															int final_index, vector<int> &seg_indexes) {
	double k, b;
	vector<double> deltas;
	PointsSimLine(list_t, list_s, k, b);
	for (int i = 0; i < list_t.size(); i++) {
		deltas.push_back(k * list_t.at(i) + b - list_s.at(i));
	}
	seg_indexes.clear();
	for (int i = 1; i + 1< deltas.size(); i++) {
		if (deltas.at(i) > deltas.at(i+1) && deltas.at(i) > deltas.at(i-1)) {
			seg_indexes.push_back(i);
		}
		if (deltas.at(i) < deltas.at(i+1) && deltas.at(i) < deltas.at(i-1)) {
			seg_indexes.push_back(i);
		}		
	}
	seg_indexes.push_back(final_index);
	for (int i = 0; i + 1 < seg_indexes.size(); i++) {//去除太近的
		if (seg_indexes.at(i+1) - seg_indexes.at(i) < 2) {
			seg_indexes.erase(seg_indexes.begin() + i);
			i--;
		}
	}
	for (int i = 0; i + 1 < seg_indexes.size(); i++) {
		int start = (i == 0)? 0 : seg_indexes.at(i-1);
		int end = seg_indexes.at(i + 1);
		double k1, b1;
		PointsSimLine(list_t, list_s, k1, b1, start, end);
		double cost = 0.0;
		// AINFO<<"from start "<<start<<" to end "<<end<<" k "<<k1<<" b "<<b1;
		for (int j = start; j < end; j++) {
			cost += fabs(k1 * list_t.at(j) + b1 - list_s.at(j));
		}
		// AINFO<<i<<" index "<<seg_indexes.at(i)<<" cost "<<cost;
		if (cost/(end - start) < 0.4) {// 去除影响不大的
			seg_indexes.erase(seg_indexes.begin() + i);
			i--;
		}
	}
	// for (int i = 0; i < seg_indexes.size(); i++) {
	// 	AINFO<<"index "<<seg_indexes.at(i);
	// }
}

void STCommon::STBoundaryToLine(pair<Vec2d, Vec2d> &start_bd, pair<Vec2d, Vec2d> &end_pd, 
																double &up_bd_k, double &up_bd_b, double &down_bd_k, 
																double &down_bd_b, int &front_t, int &back_t) {
	int front_min_s = RoundN(start_bd.first.x(), FLAGS_scale_s);
	int front_max_s = RoundN(start_bd.second.x(), FLAGS_scale_s);
	front_t = RoundN(start_bd.first.y(), FLAGS_scale_t);
	// AERROR_IF(FLAGS_log_enable)<<"front_min_s "<<front_min_s<<" front_max_s "<<front_max_s<<" t "<<front_t;
	int back_min_s = RoundN(end_pd.first.x(), FLAGS_scale_s);
	int back_max_s = RoundN(end_pd.second.x(), FLAGS_scale_s);
	back_t = RoundN(end_pd.first.y(), FLAGS_scale_t);
	back_t = max(back_t, front_t + 1);
	// AERROR_IF(FLAGS_log_enable)<<"back_min_s "<<back_min_s<<" back_max_s "<<back_max_s<<" t "<<back_t;
	down_bd_k = 0.0;
	down_bd_b = 0.0;
	Point2Line(front_t, front_min_s, back_t, back_min_s, down_bd_k, down_bd_b);
	// AERROR_IF(FLAGS_log_enable)<<"down_bd_k "<<down_bd_k<<" down_bd_b "<<down_bd_b;
	up_bd_k = 0.0;
	up_bd_b = 0.0;
	Point2Line(front_t, front_max_s, back_t, back_max_s, up_bd_k, up_bd_b);
	// AERROR_IF(FLAGS_log_enable)<<"up_bd_k "<<up_bd_k<<" up_bd_b "<<up_bd_b;
}


/*----------------------api-------------------------*/


}
}
