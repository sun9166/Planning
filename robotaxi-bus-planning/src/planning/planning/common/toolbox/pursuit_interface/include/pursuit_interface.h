#ifndef SRC_KERNEL_IVPATHPLAN_COMMON_TOOBOX_PURSUIT_INTERFACE_INCLUDE_PURSUIT_INTERFACE_H
#define SRC_KERNEL_IVPATHPLAN_COMMON_TOOBOX_PURSUIT_INTERFACE_INCLUDE_PURSUIT_INTERFACE_H
#include "common/toolbox/geometry/include/site.h"
#include "common/toolbox/optimizer/include/pure_pursuit_angle.h"
#include "common/toolbox/math/include/math_common.h"
#include "common/mapcheck/geotool/include/coordtransform.h"
#include "datapool/include/common_config.h"
#include "common/base/log/include/log.h"
#include <locale>

using namespace acu::planning;

class PursuitInterface {
 public:
  PursuitInterface();
  ~PursuitInterface();
  
  static double CalculateOffset(const double &curvature) {
    double offset;
    if (curvature < 0.0) {
      offset = 0.0;
    } else if (curvature < 0.005) {
      offset = 0.05;
    } else if (curvature < 0.05) {
      offset = 0.1;
    } else if (curvature < 0.1) {
      offset = 0.15;
    } else if (curvature < 0.2) {
      offset = 0.2;
    } else if (curvature < 0.5) {
      offset = 0.25;
    } else if (curvature < 0.667) {
      offset = 0.35;
    } else {
      offset = 0.4;
    }
    return offset;
  }

  static int GeneratePurePursuitPath(const std::list<geometry::Site> &orig_path,
                                     const geometry::Site &ego_pos,
                                     const CarModel &car_model,
                                     std::list<geometry::Site> &result_path) {
    result_path.clear();
    std::vector<geometry::Site> pure_pursuit_path;
    if (orig_path.size() < 3) return -1;
    if (orig_path.front().reverse) return -1;
    int orig_size = orig_path.size();
    pure_pursuit_path.reserve(orig_size);
    double dis = 0.0;
    auto it = orig_path.begin();
    auto bk_it = orig_path.begin();
    for (; it != orig_path.end() && std::next(it, 1) != orig_path.end(); it = std::next(it), bk_it = std::next(bk_it)) {
      double temp = std::hypot(it->x - std::next(it, 1)->x , it->y - std::next(it, 1)->y);
      dis += temp;
      if (dis > 15.0) break;
      if (it->reverse != std::next(it, 1)->reverse) break;
      pure_pursuit_path.push_back(*it);
    }
    if (pure_pursuit_path.size() < 3) return -1;
    geometry::Site cur_loc_vcs(0.0, 0.0, 0.0);
    auto pp_smoother = std::make_shared<toolbox::optimizer::ppangle::PurePursuitAngle>();
    pp_smoother->Init(cur_loc_vcs,
                      car_model.length_wheelbase,
                      car_model.min_turning_radius+0.3,
                      car_model.length);
    int inter_ind = pp_smoother->DisSmooth(pure_pursuit_path);
    
    // need to calculate the xg, yg, global angle
    CoordTransform *coordtransform = CoordTransform::Instance();
    geometry::Site glob;
    for (auto &p : pure_pursuit_path) {
      coordtransform->VCS2GCCS (ego_pos, p, glob);
      p.xg = glob.xg;
      p.yg = glob.yg;
      p.globalangle = glob.globalangle;
    }
    for (; bk_it != orig_path.end(); bk_it = std::next(bk_it)) {
      pure_pursuit_path.push_back(*bk_it);
    }
    if (!pure_pursuit_path.empty()) {
      result_path.insert(result_path.end(), pure_pursuit_path.begin(), pure_pursuit_path.end());
    }
    return inter_ind;
  }

  // set the velocity param as the offset
  static int GetPursuitPath(const std::list<geometry::Site> &orig_path,
                            const geometry::Site &ego_pos,
                            const CarModel &car_model,
                            std::list<geometry::Site> &result_path) {
    int ind = GeneratePurePursuitPath(orig_path, ego_pos, car_model, result_path);
    if (ind < 0 || ind >= result_path.size()) {
      AERROR << "Generateoffset error";
      return -1;
    }
    for (auto &p : result_path) {
      p.velocity = CalculateOffset(p.curvature);
    }
    return ind;
  }
};

#endif