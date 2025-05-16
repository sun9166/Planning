#ifndef SRC_ALGORITHM_INTERFACE_DUMMY_INCLUDE_DUMMY_ALGORITHM_H_
#define SRC_ALGORITHM_INTERFACE_DUMMY_INCLUDE_DUMMY_ALGORITHM_H_


#include "src/algorithm/interface/interface_base/include/algorithm_base.h"
namespace acu {
namespace planning {


class DummyAlgorithm : public AlgorithmBase {
 public:
  DummyAlgorithm() {
    algorithm_type_.SetDummy();
  }
  ~DummyAlgorithm() {}

  int InitAlgorithm() { //error -1 or other,  success 0   , busy 1
    is_init_over_ = true;
    return 0;
  }

  int CheckAlgorithmState() { //error -1 or other,  success 0   , busy 1
    return 0;
  }
};


}
}

#endif