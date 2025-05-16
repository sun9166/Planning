#ifndef DATAPOOL_INCLUDE_CAN_INPUT_H_
#define DATAPOOL_INCLUDE_CAN_INPUT_H_

#include <iostream>
// #include "public_typedef.h"
#include "common/base/time/include/node_time.h"
namespace acu {
namespace planning {

struct VehicleCanStatus {
  double steerangle;
  double speed;
  int shift_position;
  int epb_status;
  int current_drive_mode;
  int control_switch_state;
  double vehcle_accel;
  double break_pressure;
  VehicleCanStatus() {
    Reset();
  }
  void Reset() {
    steerangle = 0.0;
    speed = 0.0;
    shift_position = 0;
    epb_status = 0;
    current_drive_mode = 1;
    control_switch_state = 0;
    vehcle_accel = 0.0;
    break_pressure = 0.0;
  }
};

} // namespace planning
} // namespace acu

#endif // DATAPOOL_INCLUDE_CAN_INPUT_H_