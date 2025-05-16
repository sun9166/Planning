
#include "common/toolbox/optimizer/include/pure_pursuit_angle.h"
#include "common/toolbox/math/include/math_common.h"
// #include <glog/logging.h> // TO DELETE
// #include <gflags/gflags.h> // TO DELETE

#include <cmath>
#include <memory>
#include <fstream>
#include <iostream>


namespace toolbox   {
namespace optimizer {
namespace ppangle   {

PurePursuitAngle::PurePursuitAngle() {
  L_  = 0.9;
  R_  = 1.5;

  target_index_ = 0;
  last_near_    =  0;
  Lf_           = 1.5;
}

// return -1: 路径太短或者跟踪得有问题，还是原始路径
// return 0： 正常
int PurePursuitAngle::DisSmooth(SiteVec& path) {
  // modified by jf(input check)
  if(path.size() < 3) return 0;
  // 1. get raw path
  raw_path_ = path;
  auto math_tool = std::make_shared<math::Math>();
  math_tool->InterpolateWithAngle(raw_path_, 0.05);
  // 2. recalculate the curvature
  auto cdt = std::make_shared<CurveDetection>();
  double min_radius = cdt->RunDetection(raw_path_);
  const double radius_min_limit = 1.3;
  const double curvature_max_limit = 0.769;
  int orig_purpuit_path_size = raw_path_.size();
  int bk_ind = 0;
  if (min_radius < radius_min_limit) {
    for (int i = orig_purpuit_path_size - 1; i > 0; --i) {
      if (raw_path_[i].curvature > curvature_max_limit) {
        bk_ind = i;
        break;
      }
    }
  }

  // 3.lengthen the raw path
  AddPreviewDistPoint(raw_path_);


  // 3. get target index;
  DisTargetInd4FirstTime();
  // 4. rutine iteration
  // std::ofstream out_file("/Users/fengyounan/Desktop/delta");
  double delta = 0;
  int count = 0;
  bool pursuit_succuss_flag = false;
  while (last_near_ + 1 < orig_purpuit_path_size) {
    DisControl(delta);
    DisUpdate(delta);
    count++;
    if (count > raw_path_.size() * 2) break;
    if (MeetThreshold() && count > 40 && last_near_ >= bk_ind) {
      pursuit_succuss_flag = true;
      break;
    }
  }

  if (update_path_.size() < 5 || count > raw_path_.size() * 2) {
    return -1;
  } else {
    if (pursuit_succuss_flag) {
      for (auto it = update_path_.begin(); it != update_path_.end()-1; ++it) it->angle = (*(it+1) - *it).inerangle();
      update_path_.back().angle = update_path_[update_path_.size()-2].angle;
      int inter_ind = update_path_.size();
      for (int i = last_near_ + 1; i < orig_purpuit_path_size; i++) {
        update_path_.push_back(raw_path_.at(i));
      }
      update_path_.swap(path);
      return inter_ind;
    } else {
      for (auto it = update_path_.begin(); it != update_path_.end()-1; ++it) it->angle = (*(it+1) - *it).inerangle();
      update_path_.back().angle = update_path_[update_path_.size()-2].angle;
      update_path_.swap(path);
      return path.size()-1;
    }
  }

  return 0;
}


void PurePursuitAngle::Init(const Site status,    const double param_L, 
                       const double param_R, const double param_Lf) {
  status_ = status;
  status_.angle = status_.angle * M_PI / 180;
  L_  = param_L;
  R_  = param_R;
  Lf_ = param_Lf;
}


int PurePursuitAngle::AddPreviewDistPoint(SiteVec &path) {
  if (path.size() < 2) return -1;
  double back_angle;
  Site addpoint;
  double angle_err = std::fabs(NormalizeAngle(path[path.size()-1].angle) -
                               NormalizeAngle(path[path.size()-2].angle));
  if (angle_err > 180.0) {
    angle_err = 360.0 -angle_err;
  }
  if (angle_err < 60.0) {
    back_angle = NormalizeAngle(path[path.size()-1].angle);
    addpoint = path.back();
  } else {
    back_angle = NormalizeAngle(path[path.size()-2].angle);
    addpoint = path[path.size()-2];
  }
  double length = 0.0;
  for (int i = 0; i < 40; i++) {
    addpoint.x  = path.back().x  + 0.05 * std::cos(back_angle * M_PI / 180.0);
    addpoint.y  = path.back().y  + 0.05 * std::sin(back_angle * M_PI / 180.0);
    addpoint.angle = back_angle;
    path.push_back(addpoint);
  }
  return 0;
}

int PurePursuitAngle::DisTargetInd() {
  // 1. find nearest index from raw_path
  double min         = MAX;
  size_t nearest_ind = last_near_;
  if (last_near_ + 20 <= raw_path_.size()) {
    for (size_t i = last_near_;  i < last_near_ + 20; ++i) {
      double tmp_dis = (status_ - raw_path_.at(i)).mold();
      if (tmp_dis <= min) {
        min         = tmp_dis;
        nearest_ind = i;
      } 
    } 
  } else {
    for (size_t i = last_near_; i < raw_path_.size(); ++i){
      double tmp_dis = (status_ - raw_path_.at(i)).mold();
      if (tmp_dis <= min) {
        min         = tmp_dis;
        nearest_ind = i;
      } 
    }
  }
  // out_file.close();
  last_near_    = nearest_ind;
  target_index_ = nearest_ind;
  pursuit_dis_err_ = min;
  pursuit_angle_err_ = std::fabs(NormalizeRad(status_.angle) -
                                NormalizeRad(raw_path_.at(nearest_ind).angle * M_PI /180.0));
  if (pursuit_angle_err_ > M_PI) {
    pursuit_angle_err_ = 2.0 * M_PI - pursuit_angle_err_;
  }

  // 2. search look ahead target point index
  double length = 0;
  while(Lf_ > length && target_index_+1 < raw_path_.size()) {
    length += (raw_path_.at(target_index_ + 1) - raw_path_.at(target_index_)).mold();
    ++target_index_;
  }
  return 0;
}

int PurePursuitAngle::DisTargetInd4FirstTime() {
  // 1. find nearest index from raw_path
  double min         = MAX;
  size_t nearest_ind = last_near_;
  for (size_t i = last_near_; i < raw_path_.size() && i < last_near_+30; ++i){
    double tmp_dis = (status_ - raw_path_.at(i)).mold();
    if (tmp_dis <= min) {
      min         = tmp_dis;
      nearest_ind = i;
    } 
  }

  // out_file.close();
  last_near_    = nearest_ind;
  target_index_ = nearest_ind;
  pursuit_dis_err_ = min;
  pursuit_angle_err_ = std::fabs(NormalizeRad(status_.angle) -
                                NormalizeRad(raw_path_.at(nearest_ind).angle * M_PI /180.0));
  if (pursuit_angle_err_ > M_PI) {
    pursuit_angle_err_ = 2.0 * M_PI - pursuit_angle_err_;
  }

  // 2. search look ahead target point index
  double length = 0;
  while(Lf_ > length && target_index_+1 < raw_path_.size()) {
    length += (raw_path_.at(target_index_ + 1) - raw_path_.at(target_index_)).mold();
    ++target_index_;
  }
  return 0;
}

int PurePursuitAngle::DisControl(double& kai) {
  size_t last_index = target_index_;
  DisTargetInd();

  if (last_index >= target_index_) target_index_ = last_index;

  Site target;
  if (target_index_ < raw_path_.size()) {
    target = raw_path_.at(target_index_);
  } else {
    target = raw_path_.back();
  }
  PreviewGlobal2Local(target);
  if (target.x < 0.0) {
    if (target.y > 0.0) {
      kai = 1.0 / R_;
    } else {
      kai = -1.0 / R_; 
    }
  } else {
    // double alpha = std::atan2(target.y - status_.y, target.x - status_.x) - status_.angle;
    // kai = (2.0 * std::sin(alpha))/((target - status_).mold());
  
    kai = 2.0 * target.y / (target.x * target.x + target.y * target.y);
    if(kai > 1.0/R_) {
      kai = 1.0/R_;
    } else if(kai < -1.0/R_) {
      kai = -1.0/R_;
    }
  }
  return 0;
}

int PurePursuitAngle::PreviewGlobal2Local(Site &target) {
  Site result_point;
  double length, angle, delta_angle;

  length = (target - status_).mold();
  angle = std::atan2(target.y - status_.y, target.x - status_.x);
  delta_angle = angle - status_.angle;

  result_point.x = length * cos(delta_angle);
  result_point.y = length * sin(delta_angle);
  result_point.angle = target.angle;
  target = result_point;
  return 0;
}

int PurePursuitAngle::DisUpdate(const double kai) {
  update_path_.push_back(status_);
  if (std::fabs(kai) < 1e-6) {
    status_.x += dt*cos(status_.angle);
    status_.y += dt*sin(status_.angle);
    status_.angle += dt*kai;
  } else {
    double center_x = status_.x - sin(status_.angle)/kai;
    double center_y = status_.y + cos(status_.angle)/kai;
    status_.angle += dt*kai;
    status_.x = center_x + sin(status_.angle)/kai;
    status_.y = center_y - cos(status_.angle)/kai;
  }

  return 0;
}

bool PurePursuitAngle::MeetThreshold() {
  if (pursuit_dis_err_ < 0.05 &&
      pursuit_angle_err_ < 10.0 * M_PI /180.0) {
    return true;
  }
  return false;
}

double PurePursuitAngle::NormalizeRad(const double heading) {
  double a = std::fmod(heading + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += 2.0 * M_PI;
  }
  return a - M_PI;
}

double PurePursuitAngle::NormalizeAngle(const double heading) {
  double a = std::fmod(heading + 180.0, 2.0 * 180.0);
  if (a < 0.0) {
    a += 2.0 * 180.0;
  }
  return a - 180.0;
}


} // namespace ppangle
} // namespace optimizer
} // namespace toolbox