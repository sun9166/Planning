#ifndef SRC_EXECUTION_COGNITION_BASE_H_
#define SRC_EXECUTION_COGNITION_BASE_H_

#include <string>
#include <vector>
#include <map>
#include <list>
#include <memory>
#include <cmath>
#include <limits>
#include "datapool/include/locperception_input.h"
#include "conf/cognition_gflags.h" 

using namespace std;

namespace acu{
namespace planning {

int RoundN(const double &a, const double &b);
double NormalizeAngle(const double angle);
double IncludeAngle(double angle1, double angle2);
string LaneToRoad(string lane);
double PointsSimLine(vector<pair<double, double> > &points);
double PointsSimLine(vector<double> &list_t, vector<double> &list_s, 
                     double &k, double &b, int start = 0, int end = 1e4);
void Point2Line(int x1, int y1, int x2, int y2, double &k, double &b);
void GetVerticalLine(const Site &change_point, double &a, double &b, double &c);
Box2d CellsToBox(CallbackObject &per_obj);
Box2d GetBox(CallbackObject &per_obj);
void OverlayObjProbability(vector<ST> &old_points, vector<ST> &new_points, 
                           int &update_s, int &update_t, int expand_state = 0);
double VT(double& init_v, double &t, double a = FLAGS_upper_acc);

} // namespace planning
} // namespace acu


#endif
