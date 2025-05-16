#ifndef MODULES_COMMON_ROS_UTIL_H_
#define MODULES_COMMON_ROS_UTIL_H_

#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <string.h>
#include <utility>
#include <vector>

namespace  acu {
namespace common {
namespace util {

bool ExeLinuxCmdBlock(const std::string cmd);
int IsFileExist(const std::string &file_path);
int GetFreeDiskSize(const std::string &file_path);
int GetDirectSize(const std::string &file_path);
std::string GetSystemTime();
void StayDirSize(const std::string &dir_name, int max_size = 10 * 1024 * 1024) ;
int myexec(const char *cmd, std::vector<std::string> &resvec);
bool GetSrcPath(std::string &res);
bool getPackagePath(std::string package_name, std::string &res) ;
bool lookupService(const std::string& service, std::string&uri);
bool LookupService(const std::string &node_name, const std::string& service, std::string &uri) ;

}  // namespace util
}  // namespace common
}  // namespace  acu

#endif  // MODULES_COMMON_UTIL_H_