#ifndef SRC_EXECUTION_BUSINESS_INCLUDE_BUSINESS_BASS_H_
#define SRC_EXECUTION_BUSINESS_INCLUDE_BUSINESS_BASS_H_

#include "business_header.h"

namespace acu {
namespace planning {

class BusinessBase {
public:
  BusinessBase() {
    is_init_ = false;
  }
  virtual ~BusinessBase() {};

public:

  virtual bool Init() = 0;
  virtual bool Process() = 0;
  bool IsInit() {
    return is_init_;
  }

  /**
   * @brief : get/set data from datapoll
   * @param : none
   * @return: none
  **/
  virtual void PullData() = 0;
  virtual void PushData() = 0;
  BusinessType business_type_;

public:
  /**
   * @brief : get/set the name/type of this business
   * @param : none
   * @return: none
  **/
  BusinessType GetBusinessType() {return business_type_;}
  BusinessStatus GetBusinessStatus() {return business_status_;}


  void SetBusinessStatus(const BusinessStatus &business_status) {
    business_status_ = business_status;
  }
  void SetBusinessType(const BusinessType &business_type) {
    business_type_ = business_type;
  }
  // void SetBusinessParam(const BusinessParam &business_param) {
  //   business_param_ = business_param;
  // }
public:
  bool is_init_;
private:
  BusinessStatus business_status_;

  // BusinessParam business_param_;

};

}
}


#endif