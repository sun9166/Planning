#ifndef SRC_ALGORITHM_INTERFACE_INTERFACE_BASE_ALGORITHM_BASE_H_
#define SRC_ALGORITHM_INTERFACE_INTERFACE_BASE_ALGORITHM_BASE_H_

#include "datapool/include/public_typedef.h"

namespace acu {
namespace planning {

#define PATHPLAN_ERROR  (-1)
#define PATHPLAN_SUCCESS (0)
#define FRONT_PATH  (1)
#define BACK_PATH   (2)
#define PATHPLAN_BUSY_STATUS  (1)

enum class eAlgorithmStatus {
  DEFAULT = 0,    /*!< default is zero */
  ALGORITHM_INIT_STATE,      /*!< init state */
  ALGORITHM_BUSY_STATUS,     /*!< busy status */
  ALGORITHM_CHECK_STATUS  /*!< check status */
};

enum class eCheckAlgthmResult {
  ERROR = -1,
  SUCCESS = 0,
  BUSY = 1,
  IDLE = 2
};

enum class eAlgorithmType {
  DEFAULT = 0,    /*!< default is zero */
  DUMMY,
  BACK_AVOID,
  BACK_FOLLOWING,
  FORCE_AVOID,
  FORWARD_FOLLOWING,
  FORWARD_AVOID,
  NARROW_TURN,
  STAY_STILL,
  YIELD
};

typedef struct AlgorithmType
{
  eAlgorithmType type;
  std::string type_str;

  void SetDummy() {
    type = eAlgorithmType::DUMMY;
    type_str = "dummy";
  }
  void SetBackAvoid() {
    type = eAlgorithmType::BACK_AVOID;
    type_str = "back_avoid";
  }

  void SetBackFollowing() {
    type = eAlgorithmType::BACK_FOLLOWING;
    type_str = "back_following";
  }

  void SetForceAvoid() {
    type = eAlgorithmType::FORCE_AVOID;
    type_str = "force avoid";
  }

  void SetForwardFollowing() {
    type = eAlgorithmType::FORWARD_FOLLOWING;
    type_str = "ForwardFollowing";
  }

  void SetForwardAvoid() {
    type = eAlgorithmType::FORWARD_AVOID;
    type_str = "ForwardAvoid";
  }

  void SetNarrowTurn() {
    type = eAlgorithmType::NARROW_TURN;
    type_str = "NarrowTurn";
  }

  void SetStayStill() {
    type = eAlgorithmType::STAY_STILL;
    type_str = "StayStill";
  }

  void SetYield() {
    type = eAlgorithmType::YIELD;
    type_str = "Yield";
  }
} AlgorithmType;


class AlgorithmBase
{
public:
  AlgorithmBase() {
    is_init_over_ = false;
    status_ = eAlgorithmStatus::DEFAULT;
  }
  virtual ~AlgorithmBase() {};


  // void SetAlgorithmData(const Paths &paths, const LocPerception &loc_perception, const CarStatus &car_status) {
  //   paths_ = paths;
  //   loc_perception_ = loc_perception;
  //   car_status_ = car_status;
  // }

  virtual int InitAlgorithm() = 0; //error -1 or other,  success 0   , busy 1
  virtual int CheckAlgorithmState() = 0;  //error -1 or other,  success 0   , busy 1

  void GetResult(PathData &result_path) { // return 1 front  2 back  other : error
    result_path = result_path_;
  }

  bool IsNewPath() {
    return result_path_.is_new;
  }

  void ResetNewPath() {
    result_path_.is_new = false;
  }

  bool IsInitOver() {
    return is_init_over_;
  }

  bool is_init_over_;

  eAlgorithmStatus status_;
  AlgorithmType algorithm_type_;
  PathData result_path_;
};





}
}







#endif