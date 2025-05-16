
#ifndef __COMMON_TOOLBOX_GEOMETRY_INCLUDE_BEZIER_BASE_H__
#define __COMMON_TOOLBOX_GEOMETRY_INCLUDE_BEZIER_BASE_H__

#include <cmath>
#include <vector>
#include <functional>
#include "common/toolbox/geometry/include/site.h"

namespace geometry {
  using SiteVec = std::vector<Site>;

  class Bezier 
  {

  public:
        Bezier()
        {
          _reselution = 0.05;
        }

        ~Bezier() = default;

        void GetCtrlPoints(SiteVec &CtrlPoints)
        {
          CtrlPoints.clear();
          CtrlPoints.emplace_back(_p0);
          CtrlPoints.emplace_back(_p1);
          CtrlPoints.emplace_back(_p2);
          CtrlPoints.emplace_back(_p3);
        }

        int CalculateCtrlPoints();

        int CalculateCurve(SiteVec & curve);

        int GenerateBezierCurve(const Site &start,const Site &end,SiteVec& curve);

      private:
        Site _p0, _p1, _p2, _p3;
        float _reselution;
      };
      typedef Bezier* BezierPtr;
      typedef Bezier& BezierRef;

}  // namespace geometry

#endif
