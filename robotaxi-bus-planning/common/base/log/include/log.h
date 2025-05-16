/******************************************************************************
 * Copyright (C) 2018-2023, idriverplus(BeiJing ZhiXingZhe, Inc.)
 *
 * History:
 * lbh          2019/03/28    1.0.0        refer to apollo3.5
 *****************************************************************************/

/**
 * @log
 */

#ifndef ACU_COMMON_LOG_H_
#define ACU_COMMON_LOG_H_

#define GLOG_USE_GLOG_EXPORT

#include "log_impl.h"
#include "acu_node.h"


#define AINFO ALOG_MODULE(MODULE_NAME, INFO)
#define ADEBUG ALOG_MODULE(MODULE_NAME, INFO)
#define AWARN ALOG_MODULE(MODULE_NAME, WARN)
#define AERROR ALOG_MODULE(MODULE_NAME, ERROR)

#define AINFO_IF(cond) ALOG_IF(INFO, cond, MODULE_NAME)
#define AWARN_IF(cond) ALOG_IF(WARN, cond, MODULE_NAME)
#define AERROR_IF(cond) ALOG_IF(ERROR, cond, MODULE_NAME)

#define AINFO_EVERY(freq) \
  LOG_EVERY_N(INFO, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET
#define AWARN_EVERY(freq) \
  LOG_EVERY_N(WARNING, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET
#define AERROR_EVERY(freq) \
  LOG_EVERY_N(ERROR, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET

#define ACHECK(cond) CHECK(cond) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET

#define RETURN_IF_NULL(ptr)          \
  if (ptr == nullptr) {              \
    AWARN << #ptr << " is nullptr."; \
    return;                          \
  }

#define RETURN_VAL_IF_NULL(ptr, val) \
  if (ptr == nullptr) {              \
    AWARN << #ptr << " is nullptr."; \
    return val;                      \
  }

#define RETURN_IF(condition)               \
  if (condition) {                         \
    AWARN << #condition << " is not met."; \
    return;                                \
  }

#define RETURN_VAL_IF(condition, val)      \
  if (condition) {                         \
    AWARN << #condition << " is not met."; \
    return val;                            \
  }

#define DELETE_PTR(ptr)          \
  if (ptr != nullptr) {          \
    AWARN << " delete " << #ptr; \
    delete ptr;                  \
  }

#endif  // ACU_COMMON_LOG_H_
