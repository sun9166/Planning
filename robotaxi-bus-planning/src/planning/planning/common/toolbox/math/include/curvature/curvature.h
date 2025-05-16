
#ifndef COMMON_TOOLBOX_MATH_INCLUDE_CURvATURE_H_
#define COMMON_TOOLBOX_MATH_INCLUDE_CURvATURE_H_

#include "common/toolbox/geometry/include/geoheader.h"

namespace acu {
namespace planning {
namespace math {

using geometry::Site;
using geometry::SiteVec;


class Curvature {
  public:
  Curvature();
  ~Curvature();
  
  /**
   * @brief   caculate curve of points in list
   * @param  list    vector to be caculated
   * @return   0 success
   * @return  -1 failure
  **/
  int Curve(SiteVec& list);
  int CurveUTM(SiteVec& list);

private:
  double curve_end_;
  void CalculateLengthOfPoint(SiteVec &list);

};


} // math
} // planning
} // acu

#endif 
