/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
/******************************************************************************
 * Copyright (C) 2018-2023, idriverplus(BeiJing ZhiXingZhe, Inc.)
 *
 * History:
 * lbh          2019/03/28    1.0.0        refer to apollo3.5
 *****************************************************************************/

#ifndef ACU_COMMON_LOG_IMPL_H_
#define ACU_COMMON_LOG_IMPL_H_

#include <stdarg.h>

#ifndef GLOG_USE_GLOG_EXPORT
   #define GLOG_USE_GLOG_EXPORT
#endif

#include "glog/logging.h"
#include "glog/raw_logging.h"

#define LEFT_BRACKET "["
#define RIGHT_BRACKET "] "

#ifndef MODULE_NAME
#define MODULE_NAME acu::common::AcuNode::GetNodeName().c_str()
#endif

#ifndef ALOG_MODULE_STREAM
#define ALOG_MODULE_STREAM(log_severity) ALOG_MODULE_STREAM_##log_severity
#endif

#ifndef ALOG_MODULE
#define ALOG_MODULE(module, log_severity) \
  ALOG_MODULE_STREAM(log_severity)(module)
#endif

#define ALOG_MODULE_STREAM_INFO(module)                         \
  google::LogMessage(__FILE__, __LINE__, google::INFO).stream() \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define ALOG_MODULE_STREAM_WARN(module)                            \
  google::LogMessage(__FILE__, __LINE__, google::WARNING).stream() \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define ALOG_MODULE_STREAM_ERROR(module)                         \
  google::LogMessage(__FILE__, __LINE__, google::ERROR).stream() \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define ALOG_MODULE_STREAM_FATAL(module)                         \
  google::LogMessage(__FILE__, __LINE__, google::FATAL).stream() \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define ALOG_IF(severity, cond, module) \
  !(cond) ? (void)0                     \
           : google::logging::internal::LogMessageVoidify() & ALOG_MODULE(module, severity)        // huangchuo
          //: google::LogMessageVoidify() & ALOG_MODULE(module, severity)
          

#endif  // ACU_COMMON_LOG_IMPL_H_
