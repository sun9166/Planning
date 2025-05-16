
#ifndef __COMMON_TOOBOX_GEOMETRY_INCLUDE_CIRCULARMODEL_BASE_H__
#define __COMMON_TOOBOX_GEOMETRY_INCLUDE_CIRCULARMODEL_BASE_H__

#include <cmath>
#include <functional>
#include "common/toolbox/geometry/include/site.h"

namespace geometry {

class CircularModel 
{

public:
  CircularModel() 
  {
    // _ori(0.0, 0.0);
    _radius = 0.0;
  }

  CircularModel(const CircularModel &cm) 
  {
    _ori = cm.GetOri();
    _radius = cm.GetRadius();
  }

  CircularModel(const Site &ori, const float& radius) 
  {
    _ori = ori;
    _radius = radius;
  }

  CircularModel(const float& xo, const float& yo, const float& zo, const float& radius) 
  {
    _ori = Site(xo, yo, zo);
    _radius = radius;
  }

  ~CircularModel() = default;

  Site GetOri() const
  {
    return _ori;
  }

  float GetRadius() const
  {
    return _radius;
  }

  void SetOri(const Site& ori)
  {
    _ori = ori;
  }

  void SetOri(const float& xo, const float& yo, const float& zo)
  {
    _ori = Site(xo, yo, zo);
  }

  void SetOri(const float& xo, const float& yo)
  {
    _ori = Site(xo, yo);
  }

  void SetRadius(const float& radius)
  {
    _radius = radius;
  }

  void operator=(const CircularModel &cm) 
  {
    _ori = cm.GetOri();
    _radius = cm.GetRadius();
  }

  bool operator==(const CircularModel &cm) const 
  {
    double radius_precision = std::fabs(_radius - cm.GetRadius());
    return (radius_precision < 1e-3) && (_ori == cm.GetOri());
  }

private:
  Site _ori;
  float _radius;
};
typedef CircularModel* CircularModelPtr;
typedef CircularModel& CircularModelRef;

struct CircularModelVec
{
  std::vector<CircularModel> circle_vec;
};
typedef CircularModelVec* CircularModelVecPtr;
typedef CircularModelVec& CircularModelVecRef;

struct CircularModelVecGroup
{
  std::vector<CircularModelVec> circle_group;
};
typedef CircularModelVecGroup* CircularModelVecGroupPtr;
typedef CircularModelVecGroup& CircularModelVecGroupRef;
typedef std::vector<CircularModelVec>::iterator CircularModelVecGroupIter;


}  // namespace geometry

#endif
