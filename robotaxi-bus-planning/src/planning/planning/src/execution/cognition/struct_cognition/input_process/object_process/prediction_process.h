#include "datapool/include/common_typedef.h"

using namespace acu::common;
using namespace acu::hdmap;
using namespace acu::common::math;

namespace acu{
namespace planning {

class PredictionProcess {
 public:
	PredictionProcess();
	~PredictionProcess();
	void AddPredictionInfo(double perception_time, std::map<int, LineObject> &lines_objects,
							PredictionData &prediction);

 private:
 	void AlignPredictionTime(const double perception_time,
           vector<PredictionObject> *prediction_obstacles);
 	void MatchingPrediction(std::map<int, LineObject> &lines_objects, PredictionData &prediction);
 	// void FilterSimilarPrediction(std::map<int, LineObject> &lines_objects);
	// void ExpandPredictionlines(PredictionObject &pd_object, LineObject& object);
	// void OffsetPoint(Site &input, Site &outeput, double dis);
	// bool GenerateCurve(SiteVec &input, vector<PredictionPoint> &points, double &v, double &a);
	

 	private:
 	std::map<int, LineObject>* lines_objects_ptr_;
 	vector<PredictionObject>* prediction_ptr_;
 	LineObject* object_ptr_;
 	vectormap::VectorMap* vectormap_;
 	double prediction_perception_time_;
};


}
}