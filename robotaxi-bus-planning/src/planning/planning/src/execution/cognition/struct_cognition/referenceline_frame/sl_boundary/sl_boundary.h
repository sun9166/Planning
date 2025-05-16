#ifndef SRC_EXECUTION_COGNITION_SL_BOUNDARY_H_
#define SRC_EXECUTION_COGNITION_SL_BOUNDARY_H_

#include <string>
#include <vector>
#include <map>
#include <list>
#include <memory>
#include <cmath>
#include <limits>
#include "datapool/include/mapengine_input.h"
#include "datapool/include/locperception_input.h"

namespace acu{
namespace planning {

bool GetSLBoundary(const MapEngineLineList &map_info, const LineObject &temp_obj, StructSLBoundary& sl_bdy);
bool XYToSL(const MapEngineLineList &map_info, const Site &input_p, double &s, double &l);
bool XYToSL(const SiteVec &path_points, const Site &input_p, int center_index, double &s, double &l);
bool BoxToSL(const MapEngineLineList &map_info, const LineObject &temp_obj, StructSLBoundary &sl_bdy);
bool BoxToSL(const MapEngineLineList &mapinfo, const Box2d &box, StructSLBoundary &sl_bdy);
bool ObjectToSL(const MapEngineLineList &map_info, const LineObject &temp_obj, StructSLBoundary &sl_bdy);
bool PredictionToSL(const MapEngineLineList &map_info, LineObject &temp_obj);
bool GetFrenetOriginIndex(const MapEngineLineList &map_info, const Site &input_p, int &min_index);


}
}

#endif
