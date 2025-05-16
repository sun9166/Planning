#ifndef COMMON_TOOLBOX_PATH_RESTRICTION_H_INCLUDE_PATH_RESTRICTION_H_
#define COMMON_TOOLBOX_PATH_RESTRICTION_H_INCLUDE_PATH_RESTRICTION_H_
#include "common/toolbox/geometry/include/geoheader.h"
#include "datapool/include/common_typedef.h"
#include "common/base/log/include/log.h"



using namespace acu::planning;
using geometry::Site;
using geometry::SiteVec;
class PathRestriction {
 public:
  PathRestriction();
  ~PathRestriction() {};

  //assiagn restrictions for pathpoints and path
  int AssignPathRestriction(PathData&  path_no_restriction);

  //generate boudaries for path with path's restrictions
  int GeneratePathRestrictionLine(const std::list<Site>& path) const;

  //remove points with un-qualified angles
  int FiltPathByAngle(std::list<Site>& path, const std::list<Site>& path_ori, 
                      const float& move_dis, const float& move_dis_right) const;

  //generate parallel paths
  int GenerateParallelPath(const std::list<Site>& base_path, std::list<Site>& rt_path,
                           const int& direction, const float& move_dis) const;

  //get parallel distance by collision-detect & fs
  int GenerateParallelDistance(const std::list<Site>& base_path, 
                               std::pair<float, float>& dis_pair) const;

  //check whther necessary to re-plan the restricts
  int CheckNecessity(std::list<Site>& path);

  //check the availability by current behavior state
  bool CheckAvailabilitybyState();

  //check the availability by current behavior state for releasing
  bool CheckAvailabilitybyStateForReleasing();


  int SetLenthThreshold(const float& lth)
  {
    int rtvalue = -1;
    if(lth > 0)
    {
      _length_threshold = lth;
      rtvalue = 0;
    }
    return rtvalue;
  }

  int SetWidthThreshold(const float& wth)
  {
    int rtvalue = -1;
    if(wth > 0)
    {
      _width_threshold = wth;
      rtvalue = 0;
    }
    return rtvalue;
  }

  int SetWidthResolution(const float& wrs)
  {
    int rtvalue = -1;
    if(wrs > 0)
    {
      _width_resolution = wrs;
      rtvalue = 0;
    }
    return rtvalue;
  }

  int SetWidthLimit(const float& wlm)
  {
    int rtvalue = -1;
    if(wlm > 0)
    {
      _width_limit = wlm;
      rtvalue = 0;
    }
    return rtvalue;
  }

  int SetLastResrictLeft(const float& lrl)
  {
    int rtvalue = -1;
    if(lrl > _width_limit)
    {
      _last_left_restrict = _width_limit;
      rtvalue = 1;
    }
    else if(lrl > 0)
    {
      _last_left_restrict = lrl;
      rtvalue = 0;
    }
    return rtvalue;
  }

  int SetLastResrictRight(const float& lrr)
  {
    int rtvalue = -1;
    if(lrr > _width_limit)
    {
      _last_right_restrict = _width_limit;
      rtvalue = 1;
    }
    else if(lrr > 0)
    {
      _last_right_restrict = lrr;
      rtvalue = 0;
    }
    return rtvalue;
  }  

private:
  double NormalizeAngle(const double& heading) const
  {
    double a = std::fmod(heading + 180.0, 2.0 * 180.0);
    if (a < 0.0) 
    {
      a += 2.0 * 180.0;
    }
    return a - 180.0;
  }

  //generate front corner points' lines using the  restrict lines.
  int GenerateFrontCornerLine(const PathData& ll, const PathData& rl,
                              PathData& rll, PathData& rrl) const;

  //distance threshold when checking ALONG the path
  float _length_threshold;

  //distance threshold when checking NORMAL the path
  float _width_threshold;

  //distance resolution when checking NORMAL the path(obstacle)
  float _width_resolution; 

  //distance threshold when outputting NORMAL the path
  float _width_limit;

  //trigger for this function
  bool _if_enable;

  //last loop's results
  float _last_left_restrict, _last_right_restrict;

  //initial pos of checking
  float _initial_left, _initial_right;


  int SetInitialLeft(const float& il)
  {
    int rtvalue = -1;
    if(il > 0)
    {
      _initial_left = il;
      rtvalue = 0;
    }
    return rtvalue;
  }

  int SetInitialRight(const float& ir)
  {
    int rtvalue = -1;
    if(ir > 0)
    {
      _initial_right = ir;
      rtvalue = 0;
    }
    return rtvalue;
  }

  void ResetInitialValue()
  {
    _initial_left = 0.46;
    _initial_right = 0.46;
  }

public:
  typedef PathRestriction* PathRestrictionPtr;
  typedef PathRestriction& PathRestrictionRef;


};

#endif