#ifndef SRC_EXECUTION_BUSINESS_INCLUDE_STRUCT_BUSINESS_H_
#define SRC_EXECUTION_BUSINESS_INCLUDE_STRUCT_BUSINESS_H_

#include "business_base.h"
#include "common/base/log/include/log.h"
#include "datapool/include/data_pool.h"
// #include "src/application/execution/dummy/include/dummy_pathplan.h"
#include "common/math/vec2d.h"
#include "common/math/box2d.h"
using geometry::Site;
using geometry::SiteVec;

namespace acu {
namespace planning {

class StructBusiness: public BusinessBase {
 public:
  StructBusiness();
  ~StructBusiness() {};

  bool Init();
  bool Process();

  void PullData();
  void PushData();

private:
  // BusinessParam business_param_;
  bool business_error_;
  bool business_exucute_over_;
  bool short_lane_flag_;
  double car_velocity_;
  double dis2missionpoint_;
  CarModel car_model_;
  SiteVec frontrefpath_;
  int no_map_counter_;
};

}
}

#endif