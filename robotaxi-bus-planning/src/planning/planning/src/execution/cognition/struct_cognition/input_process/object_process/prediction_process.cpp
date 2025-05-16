#include "prediction_process.h"

namespace acu{
namespace planning {

PredictionProcess::PredictionProcess() {
	vectormap_ = map::MapLoader::GetVectorMapPtr();
	if (vectormap_ == nullptr) {
		return;
	}
} 

PredictionProcess::~PredictionProcess() {}

void PredictionProcess::AddPredictionInfo(double perception_time, 
												std::map<int, LineObject> &lines_objects,
												PredictionData &prediction) {
	if (lines_objects.empty()) {
		return;
	}
	prediction_perception_time_ = prediction.perception_time;
	AlignPredictionTime(perception_time, &prediction.prediction_objects);
	MatchingPrediction(lines_objects, prediction);
}

void PredictionProcess::AlignPredictionTime(const double perception_time,
                          vector<PredictionObject> *prediction_obstacles) {
  for (auto &obstacle : *prediction_obstacles) {
    double prediction_time = prediction_perception_time_;
    // if (prediction_time < perception_time) continue;
    for (auto &trajectory : obstacle.trajectories) {
      for (auto &point : trajectory.points) {
        //point.t = prediction_time + point.t - perception_time;
      }
      if (!trajectory.points.empty() && trajectory.points.begin()->t < 0) {
        auto it = trajectory.points.begin();
        while (it != trajectory.points.end() && it->t < 0) {
          ++it;
        }
        trajectory.points.erase(trajectory.points.begin(), it);
      }
    }
  }
}

void PredictionProcess::MatchingPrediction(std::map<int, LineObject> &lines_objects, 
																					 PredictionData &prediction) {
	for (auto it = lines_objects.begin(); it != lines_objects.end(); it++) {
		it->second.prediction.Reset();
	}
	if (prediction.prediction_objects.empty()) return;
	for (auto &pd_obj : prediction.prediction_objects) {
		if (lines_objects.count(pd_obj.id)) {
			double p_sum = 0.0;
			for (auto &pd_line : pd_obj.trajectories) {
				p_sum += pd_line.probability;// 概率归一化
			}
			for (auto &pd_line : pd_obj.trajectories) {
				pd_line.probability = pd_line.probability / p_sum;// 概率归一化
			}
			lines_objects.at(pd_obj.id).prediction = pd_obj;
		}
	}
}




}
}