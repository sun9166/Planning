#ifndef SRC_EXECUTION_BUSINESS_INCLUDE_BUSINESS_HEADER_H_
#define SRC_EXECUTION_BUSINESS_INCLUDE_BUSINESS_HEADER_H_
#include <string>
#include <memory>
#include <stddef.h>
#include <functional>


namespace acu {
namespace planning {

enum class eBusinessStatus {
  IDLE = 0,
  RUNNING,
  SUCCESS,
  FAILURE
};


typedef struct BusinessStatus {
  eBusinessStatus status;
  std::string status_str;
  BusinessStatus() {
    status = eBusinessStatus::IDLE;
    status_str = "idle";
  }

  void SetBusinessStatusIdle() {
    status = eBusinessStatus::IDLE;
    status_str = "idle";
  }

  void SetBusinessStatusRunning() {
    status = eBusinessStatus::RUNNING;
    status_str = "RUNNING";
  }

  void SetBusinessStatusSuccess() {
    status = eBusinessStatus::SUCCESS;
    status_str = "SUCCESS";
  }

  void SetBusinessStatusFailure() {
    status = eBusinessStatus::FAILURE;
    status_str = "FAILURE";
  }
} BusinessStatus;


enum class eBusinessType {
  IDLE = 0,
  DUMMY,
  UNSTRUCT_BUSINESS,
  STRUCT_BUSINESS,
  CALIBRATION
};

typedef struct BusinessType
{
  eBusinessType type;
  std::string type_str;
  BusinessType() {
    type = eBusinessType::IDLE;
    type_str = "idle";
  }

  void SetBusinessTypeIdle() {
    type = eBusinessType::IDLE;
    type_str = "idle";
  }
  void SetBusinessTypeUnstruct() {
    type = eBusinessType::UNSTRUCT_BUSINESS;
    type_str = "UNSTRUCT_BUSINESS";
  }

  void SetBusinessTypeStruct() {
    type = eBusinessType::STRUCT_BUSINESS;
    type_str = "STRUCT_BUSINESS";
  }
  
  void SetBusinessTypeCalibration() {
    type = eBusinessType::CALIBRATION;
    type_str = "Calibration";
  }
} BusinessType;


typedef struct BusinessMapInfo {
  std::string target_name;
  int index;
  BusinessMapInfo() {
    target_name = "";
    index = 0;
  }
} BusinessMapInfo;


// class PathplanBase;
// typedef struct BusinessParam {
//   std::shared_ptr<PathplanBase> pathplan_ptr;
//   BusinessParam() {
//     pathplan_ptr.reset();
//   }
// } BusinessParam;

}
}

#endif