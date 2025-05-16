/******************************************************************************
 * Copyright (C) 2018-2023, idriverplus(BeiJing ZhiXingZhe, Inc.)
 *
 * History:
 * lbh          2019/03/28    1.0.0        build
 *****************************************************************************/

#ifndef ACU_COMMON_INIT_H_
#define ACU_COMMON_INIT_H_

#include <iostream>
#include "log.h"

namespace acu {
namespace common {

class AcuNode {
public:
  static bool Init(const std::string node_name, bool Enable_GLOG_Screen = false,
                   const std::string info_log_file_path = "",
                   const std::string warning_log_file_path = "",
                   const std::string error_log_file_path = "",
                   const std::string output_log_level= "") {
    SetNodeName(node_name);
    google::InitGoogleLogging(node_name.c_str());
    google::SetLogDestination(google::INFO, info_log_file_path.c_str());
    google::SetLogDestination(google::WARNING, warning_log_file_path.c_str());
    google::SetLogDestination(google::ERROR, error_log_file_path.c_str());
    FLAGS_logtostderr = false; //add by zhubin 20221101 是否将日志输出到stderr而非文件
    FLAGS_colorlogtostderr = Enable_GLOG_Screen; ///chanage by zhubin 20221101 
    FLAGS_minloglevel = google::GLOG_INFO;
    FLAGS_stderrthreshold = google::ERROR; //输出到stderr的限值，默认为2（ERROR），默认ERORR以下的信息(INFO、WARNING)不打印到终端
    FLAGS_logbufsecs = 3; //add by zhubin 20221101
    FLAGS_alsologtostderr =  Enable_GLOG_Screen; //是否将日志输出到文件和stderr，如果：true，忽略FLAGS_stderrthreshold的限制，所有信息打印到终端
    FLAGS_max_log_size = 20;  //add by zhubin 20220401 max log file is 20M 设置最大日志文件大小（以MB为单位）
    FLAGS_stop_logging_if_full_disk = true; //add by zhubin 20220401 设置是否在磁盘已满时避免日志记录到磁盘。
    if(output_log_level != "")
    {
      if(output_log_level == "Warning")
        FLAGS_minloglevel = google::GLOG_WARNING;
      if(output_log_level == "Error")
        FLAGS_minloglevel = google::GLOG_ERROR;
    }
    return true;
  }
  static std::string GetNodeName() { return GetNameRef(); }
  static void SetNodeName(const std::string& name) { GetNameRef() = name; }
private:
  static std::string& GetNameRef() {
    static std::string node_name;
    return node_name;
  }
};
}  // namespace acu
}  // namespace common

#endif  // ACU_COMMON_INIT_H_
