/******************************************************************************
 * Copyright 2017 The  acu Authors. All Rights Reserved.
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

#include "common/util/util.h"

#include <cmath>

namespace  acu {
namespace common {
namespace util {

SLPoint MakeSLPoint(const double s, const double l) {
  SLPoint sl;
  sl.set_s(s);
  sl.set_l(l);
  return sl;
}

PointENU MakePointENU(const double x, const double y, const double z) {
  PointENU point_enu;
  point_enu.set_x(x);
  point_enu.set_y(y);
  point_enu.set_z(z);
  return point_enu;
}

PointENU MakePointENU(const math::Vec2d& xy) {
  PointENU point_enu;
  point_enu.set_x(xy.x());
  point_enu.set_y(xy.y());
  point_enu.set_z(0.0);
  return point_enu;
}

/*

 acu::perception::Point MakePerceptionPoint(const double x, const double y,
                                              const double z) {
  perception::Point point3d;
  point3d.set_x(x);
  point3d.set_y(y);
  point3d.set_z(z);
  return point3d;
}*/

SpeedPoint MakeSpeedPoint(const double s, const double t, const double v,
                          const double a, const double da) {
  SpeedPoint speed_point;
  speed_point.set_s(s);
  speed_point.set_t(t);
  speed_point.set_v(v);
  speed_point.set_a(a);
  speed_point.set_da(da);
  return speed_point;
}

PathPoint MakePathPoint(const double x, const double y, const double z,
                        const double theta, const double kappa,
                        const double dkappa, const double ddkappa) {
  PathPoint path_point;
  path_point.set_x(x);
  path_point.set_y(y);
  path_point.set_z(z);
  path_point.set_theta(theta);
  path_point.set_kappa(kappa);
  path_point.set_dkappa(dkappa);
  path_point.set_ddkappa(ddkappa);
  return path_point;
}

double Distance2D(const PathPoint& a, const PathPoint& b) {
  return std::hypot(a.x() - b.x(), a.y() - b.y());
}


//execute shell command
//执行一个shell命令，输出结果逐行存储在resvec中，并返回行数
int myexec(const char *cmd, std::vector<std::string> &resvec) {
  resvec.clear();
  FILE *pp = popen(cmd, "r"); //建立管道
  if (!pp) {
    return -1;
  }
  char tmp[1024]; //设置一个合适的长度，以存储每一行输出
  while (fgets(tmp, sizeof(tmp), pp) != NULL) {
    if (tmp[strlen(tmp) - 1] == '\n') {
      tmp[strlen(tmp) - 1] = '\0'; //去除换行符
    }
    resvec.push_back(tmp);
  }
  pclose(pp); //关闭管道
  return resvec.size();
}


bool getPackagePath(std::string package_name, std::string &res) {
  std::vector<std::string> resvec;
  std::string cmd = "rospack find " + package_name;
  myexec(cmd.c_str(), resvec);
  if (resvec.size() == 0)
    return false;
  res = resvec[0];
  return true;
}


bool GetSrcPath(std::string &res) {
  std::string proto_path;
  if (getPackagePath("proto", proto_path ) == false) {
    return false;
  }
  res = proto_path + "/../../";
  return true;
}



}  // namespace util
}  // namespace common
}  // namespace  acu
