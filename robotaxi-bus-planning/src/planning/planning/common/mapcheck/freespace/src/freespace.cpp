#include "common/mapcheck/freespace/include/freespace.h"

using acu::map::MapLoader;

namespace acu {
namespace planning {

Freespace::Freespace() {
  basemap_ = nullptr;
}

void Freespace::Init() {
  return;
}

bool Freespace::Check() {
  if (nullptr != Instance()->basemap_)  return true;
  if (nullptr == Instance()->basemap_) Instance()->basemap_ = MapLoader::GetBasemapPtr();
  return (nullptr != Instance()->basemap_);
}

bool Freespace::IsInFreespace(const Site &car_pos) {
  if (!Check()) return false;
  PointGCCS gccs;
  PointVCS vcs;
  gccs.xg = car_pos.xg;
  gccs.yg = car_pos.yg;
  gccs.angle = car_pos.globalangle;
  vcs.x = 0.0;
  vcs.y = 0.0;
  return Instance()->basemap_->IsInFreespace(gccs, vcs);
}

bool Freespace::IsInFreespace(const Site &car_pos, const Site &vcs_pos) {
  if (!Check()) return false;
  PointGCCS gccs;
  PointVCS vcs;
  gccs.xg = car_pos.xg;
  gccs.yg = car_pos.yg;
  gccs.angle = car_pos.globalangle;
  vcs.x = vcs_pos.x;
  vcs.y = vcs_pos.y;
  return Instance()->basemap_->IsInFreespace(gccs, vcs);
}

bool Freespace::IsInFreespace(const PointGCCS &car_pos) {
  if (!Check()) return false;
  PointVCS vcs_pos;
  vcs_pos.x = 0.0;
  vcs_pos.y = 0.0;
  return Instance()->basemap_->IsInFreespace(car_pos, vcs_pos);
}

bool Freespace::IsInFreespace(const PointGCCS &car_pos, const PointVCS &vcs_pos) {
  if (!Check()) return false;
  return Instance()->basemap_->IsInFreespace(car_pos, vcs_pos);
}

bool Freespace::MODLFSCheck(const Site &gccs, const CarModel &car_model) {
  if (!Check()) return false;
  PointGCCS car_pos;
  PointVCS  fl, fr, rl, rr, center_pt;
  car_pos.xg     = gccs.xg; // + instance()->map_x_offset_;
  car_pos.yg     = gccs.yg; // + instance()->map_y_offset_;
  car_pos.angle  = gccs.globalangle;
  fl.x =  1 * car_model.front_over_hang;
  fl.y =  1 * car_model.half_wheel;
  fr.x =  1 * car_model.front_over_hang;
  fr.y = -1 * car_model.half_wheel;
  rl.x = -1 * car_model.back_over_hang;
  rl.y =  1 * car_model.half_wheel;
  rr.x = -1 * car_model.back_over_hang;
  rr.y = -1 * car_model.half_wheel;
  center_pt.x = 1.0 * car_model.front_over_hang;
  center_pt.y = 0.0;

  bool result_fl = Instance()->basemap_->IsInFreespace(car_pos, fl);
  bool result_fr = Instance()->basemap_->IsInFreespace(car_pos, fr);
  bool result_rl = Instance()->basemap_->IsInFreespace(car_pos, rl);
  bool result_rr = Instance()->basemap_->IsInFreespace(car_pos, rr);
  bool result_center = Instance()->basemap_->IsInFreespace(car_pos, center_pt);
  // AERROR << "out:" << result_fl << "," 
  //                  << result_fr << "," 
  //                  << result_rl << "," 
  //                  << result_rr << "," 
  //                  << result_center;

  return (result_fl && result_fr && result_rl && result_rr && result_center);

}

bool Freespace::MODLPathFSCheck(const std::list<Site> &gccs_list,
                                const CarModel &car_model,
                                const double &check_dis,
                                CollisionInfo &collision_info) {
  collision_info.Reset();
  // // debug
  // return true;
  if (!Check()) return false;
  if (gccs_list.empty()) return true;
  if (gccs_list.size() == 1) {
    if (!MODLFSCheck(gccs_list.front(), car_model)) {
      collision_info.path_pt_info = gccs_list.front();
      collision_info.path_pt_info.index = 0;
      collision_info.path_pt_info.length = 0.0;
      collision_info.cell_info.x = gccs_list.front().x;
      collision_info.cell_info.y = gccs_list.front().y;
      collision_info.cell_info.height = 2.0;
      return false;
    }
    return true;
  }
  auto it = gccs_list.begin();
  double dis = 0.0;
  int counter = 0;
  for (; it != gccs_list.end() && std::next(it) != gccs_list.end(); it = std::next(it)) {
    if (dis > check_dis) break;
    if (!MODLFSCheck(*it, car_model)) {
      // AERROR << "[FS_OUT]" << std::fixed << it->xg << "," << it->yg << "," << it->globalangle;
      // AERROR << "[FS_OUT]" << std::fixed << it->x << "," << it->y << "," << it->angle;
      collision_info.path_pt_info = *it;
      collision_info.path_pt_info.index = counter;
      collision_info.path_pt_info.length = dis;
      collision_info.cell_info.x = it->x;
      collision_info.cell_info.y = it->y;
      collision_info.cell_info.height = 2.0;
      return false;
    }
    dis += std::hypot(it->x - std::next(it)->x, it->y - std::next(it)->y);
    counter++;
  }
  return true;
}

}
}