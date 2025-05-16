
#include "common/toolbox/optimizer/include/pure_pursuit.h"
#include "common/toolbox/math/include/math_common.h"

// #include <glog/logging.h> // TO DELETE
// #include <gflags/gflags.h> // TO DELETE

#include <cmath>
#include <memory>
#include <fstream>
#include <iostream>


namespace toolbox   {
namespace optimizer {
namespace pp        {

PurePursuit::PurePursuit() {
  L_  = 0.7;
  R_  = 1.4;

  target_index_ = 0;
  last_near_    =  0;
  Lf_           = 1.0;
}

int PurePursuit::Smooth(SiteVec& path) {
  // modified by jf(input check)
  if(path.size() < 2) return 0;
  if (path.size() == 2) {
    double dis = (path[0]-path[1]).mold();
    if (dis < 0.2) return 0;
  }
  // 1. get raw path
  raw_path_ = path;
  auto math_tool = std::make_shared<math::Math>();
  math_tool->Interpolate(raw_path_);

  // 2. get target index;
  TargetInd4FirstTime();
  // 3. rutine iteration
  // std::ofstream out_file("/Users/fengyounan/Desktop/delta");
  double delta = 0;
  while(last_near_ + 2 < raw_path_.size()) {
    Control(delta);
    Update(delta);
    // out_file << delta << std::endl;
  }
  // out_file.close();

  // update_path_.push_back(path.back());
  // math_tool->Interpolate(update_path_);
  for (auto it = update_path_.begin(); it != update_path_.end()-1; ++it) it->angle = (*(it+1) - *it).inerangle();
  update_path_.back().angle = update_path_[update_path_.size()-2].angle;
  update_path_.push_back(Site(raw_path_.back().x, raw_path_.back().y, raw_path_.back().angle));
  update_path_.swap(path);

  return 0;
}


void PurePursuit::Init(const Site status,    const double param_L, 
                       const double param_R, const double param_Lf) {
  status_ = status;
  status_.angle = status_.angle * M_PI / 180;
  L_  = param_L;
  R_  = param_R;
  Lf_ = param_Lf;
}


int PurePursuit::TargetInd() {
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
}else {
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

  // 2. search look ahead target point index
  double length = 0;
  while(Lf_ > length && target_index_+1 < raw_path_.size()) {
    length += (raw_path_.at(target_index_ + 1) - raw_path_.at(target_index_)).mold();
    ++target_index_;
  }
  return 0;
}

int PurePursuit::TargetInd4FirstTime() {
  // 1. find nearest index from raw_path
  double min         = MAX;
  size_t nearest_ind = last_near_;
  for (size_t i = last_near_; i < raw_path_.size(); ++i){
    double tmp_dis = (status_ - raw_path_.at(i)).mold();
    if (tmp_dis <= min) {
      min         = tmp_dis;
      nearest_ind = i;
    } 
  }

  // out_file.close();
  last_near_    = nearest_ind;
  target_index_ = nearest_ind;

  // 2. search look ahead target point index
  double length = 0;
  while(Lf_ > length && target_index_+1 < raw_path_.size()) {
    length += (raw_path_.at(target_index_ + 1) - raw_path_.at(target_index_)).mold();
    ++target_index_;
  }
  return 0;
}

int PurePursuit::Control(double& delta) {
  size_t last_index = target_index_;
  TargetInd();

  if (last_index >= target_index_) target_index_ = last_index;

  Site target;
  if (target_index_ < raw_path_.size()) {
    target = raw_path_.at(target_index_);
  } else {
    target = raw_path_.back();
  }

  double alpha = std::atan2(target.y - status_.y, target.x - status_.x) - status_.angle;
  // delta = std::atan2(2.0 * L_ * std::sin(alpha) / (Lf + k*V), Lf);
  delta = std::atan2(2.0 * L_ * std::sin(alpha), (target - status_).mold());
  

  if (std::fabs(delta) > MaxDelta()) {
    if (delta > 0) {
      delta = MaxDelta();
    } else {
      delta = MaxDelta() * -1;
    }
  }
  return 0;
}

int PurePursuit::Update(const double delta) {
  update_path_.push_back(status_);
  
  status_.x     += V * std::cos(status_.angle) * dt;
  status_.y     += V * std::sin(status_.angle) * dt;
  status_.angle += V / L_ * std::tan(delta) * dt;

  return 0;
}

} // namespace pp
} // namespace optimizer
} // namespace toolbox