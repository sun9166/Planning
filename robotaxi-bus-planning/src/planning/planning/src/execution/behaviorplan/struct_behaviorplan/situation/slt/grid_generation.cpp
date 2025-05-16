#include "grid_generation.h"

namespace acu {
namespace planning {

GridGeneration::GridGeneration() {}

GridGeneration::~GridGeneration() {}

void GridGeneration::InitProbGrid() {
  reference_line_ = &context_->cognition_info_->reference_line_info;
  current_line_ = context_->reference_line_map_[reference_line_->current_line_id];
  half_width_ = 0.5 * context_->planning_config_.car_model.car_width;
  context_->grid_info_.clear();
  context_->static_objects_.clear();
  context_->dynamic_objects_.clear();
  lines_.push_back(&reference_line_->reverse_reference_line);
  for (auto& line : context_->reference_line_map_) {
    if (line.first < 20 || line.first >= 30) {
      lines_.push_back(line.second);
    }
  }
  AddGridInfo();
  AddObjectInfo();
}

void GridGeneration::AddGridInfo() {
  std::vector<std::vector<GridInfoStruct>> sl_grids;
  if (context_->reference_line_map_.size() > 1) {
    AddMultiLaneGrid(sl_grids);
  } else {
    AddSingleLaneGrid(sl_grids);
  }
  for (int t_id = 0; t_id <= (int)MAX_T; t_id++) {
    for (int s_id = 0; s_id < sl_grids.size(); s_id++) {
      for (int l_id = 0; l_id < sl_grids[s_id].size(); l_id++) {
        sl_grids[s_id][l_id].s_id = s_id;
        sl_grids[s_id][l_id].l_id = l_id;
        sl_grids[s_id][l_id].t = 1.0 * t_id;
      }
    }
    context_->grid_info_.push_back(sl_grids);
  }
  for (int i = 0; i < sl_grids.size(); i = i + 10) {
    // AINFO << "s = " << i << " left = " << sl_grids[i].front().l + half_width_ << 
    //     " right = " << sl_grids[i].back().l - half_width_;
  }
}

void GridGeneration::AddMultiLaneGrid(std::vector<std::vector<GridInfoStruct>>& sl_grids) {
  int index;
  double left_w, right_w, s, l;
  Site temp_site;
  GridInfoStruct grid;
  for (int s_id = 0; s_id <= (int)(MAX_S / STEP_S); s_id++) {
    std::vector<GridInfoStruct> grids;
    grid.s = s_id * STEP_S;
    for (int i = 0; i < lines_.size(); i++) {
      if (lines_[i]->mapinfo.dis_to_end < grid.s) {
        continue;
      }
      lines_[i]->GetWidthToLaneBoundary(left_w, right_w, grid.s);
      lines_[i]->GetNearestPoint(grid.s, temp_site, index);
      XYToSL(current_line_->mapinfo, temp_site, s, l);
      left_w += FLAGS_boundary_width;
      right_w += FLAGS_boundary_width;
      grid.l_solid = lines_[i]->left_bds_.lower_bound(grid.s)->second;
      grid.r_solid = lines_[i]->right_bds_.lower_bound(grid.s)->second;
      const double on_line_width = 0.3;
      grid.l = l + left_w - half_width_;
      if ((grids.empty() || grid.l < grids.back().l) && fabs(grid.l) < 3.0) {
        AddGridLineIds(i, grid);
        grids.push_back(grid);
      }
      grid.l = l;
      if ((grids.empty() || grid.l < grids.back().l) && fabs(grid.l) < 2.0) {
        AddGridLineIds(i, grid);
        grids.push_back(grid);
      }
      grid.l = l + half_width_ - right_w;
      if ((grids.empty() || grid.l < grids.back().l) && fabs(grid.l) < 2.0) {
        AddGridLineIds(i, grid);
        grids.push_back(grid);
      }
      grid.l = l + half_width_ - right_w - on_line_width;
      if (!grid.r_solid && (grids.empty() || grid.l < grids.back().l) && fabs(grid.l) < 2.0) {
        AddGridLineIds(i, grid);
        grids.push_back(grid);
      }
      grid.l = l + half_width_ - right_w - 3.0 * on_line_width;
      if (!grid.r_solid && (grids.empty() || grid.l < grids.back().l) && fabs(grid.l) < 2.0) {
        AddGridLineIds(i, grid);
        grids.push_back(grid);
      }
      grid.l = l + half_width_ - right_w - 5.0 * on_line_width;
      if (!grid.r_solid && (grids.empty() || grid.l < grids.back().l) && fabs(grid.l) < 2.5) {
        AddGridLineIds(i, grid);
        grids.push_back(grid);
      }
    }
    if (grids.empty()) {
      break;
    }
    sl_grids.push_back(grids);
  }
}

void GridGeneration::AddSingleLaneGrid(std::vector<std::vector<GridInfoStruct>>& sl_grids) {
  int index;
  double left_w, right_w, s, l, min_l, max_l;
  Site temp_site;
  GridInfoStruct grid;
  auto reverse_line = &reference_line_->reverse_reference_line;
  for (int s_id = 0; s_id <= (int)(MAX_S / STEP_S); s_id++) {
    std::vector<GridInfoStruct> grids;
    grid.s = s_id * STEP_S;
    grid.l_solid = false;
    grid.r_solid = false;
    if (current_line_->mapinfo.dis_to_end < grid.s) {
      break;
    }
    current_line_->GetWidthToLaneBoundary(left_w, right_w, grid.s);
    if (reverse_line->mapinfo.dis_to_end < s && 
        current_line_->left_bds_.lower_bound(grid.s)->second == false) {
      current_line_->GetNearestPoint(grid.s, temp_site, index);
      reverse_line->GetWidthToLaneBoundary(max_l, right_w, grid.s);
      max_l = std::min(right_w - half_width_, 2.5);
    } else {
      max_l = std::min(left_w - half_width_, 2.5);
    }
    min_l = half_width_ - right_w;
    double step_l = (max_l - min_l) / 6.0;
    AWARN_IF(s_id == 0) << "max_l = " << max_l << " min_l = " << min_l << " step_l = " << step_l;
    for (int i = 0; i <= 6; i++) {
      grid.l = max_l - step_l * i;
      int index = grid.l <= left_w ? lines_.size() - 1 : 0;
      AddGridLineIds(lines_.size() - 1, grid);
      grids.push_back(grid);
    }
    if (grids.empty()) {
      AERROR << "Boundary at " << grid.s << " is abnormal!";
      break;
    }
    sl_grids.push_back(grids);
  }
}

void GridGeneration::AddGridLineIds(const int index, GridInfoStruct& grid) {
  grid.line_ids.clear();
  grid.line_ids.push_back(lines_[index]->reference_lane_id);
  if (lines_[index]->reference_lane_id == 50) {
    grid.line_ids.back() = 5;
  }
  double left_w, right_w, s, l;
  Site temp_site;
  int id, nearest_index;
  for (int i = index + 1; i < lines_.size(); i++) {
    lines_[i]->GetWidthToLaneBoundary(left_w, right_w, grid.s);
    lines_[i]->GetNearestPoint(grid.s, temp_site, id);
    XYToSL(current_line_->mapinfo, temp_site, s, l);
    if (grid.s < lines_[i]->mapinfo.dis_to_end && grid.l < left_w + l) {
      grid.line_ids.push_back(lines_[index]->reference_lane_id);
    } else {
      break;
    }
  }
  current_line_->GetNearestPoint(grid.s, temp_site, nearest_index);
  grid.xg = temp_site.xg - grid.l * sin(temp_site.globalangle * M_PI / 180.0);
  grid.yg = temp_site.yg + grid.l * cos(temp_site.globalangle * M_PI / 180.0);
}

void GridGeneration::AddObjectInfo() {
  for (auto& temp_object : current_line_->objects_) {
    auto& object = temp_object.second;
    if (object.is_static) {
      if (SLInBoundary(object.sl_boundary)) {
        AINFO_IF(FLAGS_log_enable)<< "static_object = " << object.id;
        context_->static_objects_[object.id] = object.box;
      }
      continue;
    } 
    if (object.sl_boundary.max_s < 0.0) {
      if (object.conflict_type == 1 || 
          current_line_->car_info_.count(object.sl_boundary.min_s)) {
        continue;
      }
    }
    double s, l;
    Site temp_site;
    std::map<double, std::pair<Box2d, StructSLBoundary>> dynamic_sl;
    for (auto& trajectory : object.prediction.trajectories) {
      for (auto& point : trajectory.points) {
        temp_site.set_g(point.xg, point.yg);
        XYToSL(current_line_->mapinfo, temp_site, s, l);
        if (s + 0.5 * object.box.length() < 0.0) {
          continue;
        }
        Vec2d center(point.xg, point.yg);
        double angle = point.globalangle * M_PI / 180.0;
        Box2d box(center, angle, object.box.length(), object.box.width());
        StructSLBoundary sl_boundary;
        BoxToSL(current_line_->mapinfo, box, sl_boundary);
        if (SLInBoundary(sl_boundary)) {
          // AWARN << object.id << " t = " << point.t << " min_s = " << 
          //     sl_boundary.min_s << " max_s = " << sl_boundary.max_s << " min_l = " << 
          //     sl_boundary.min_l << " max_l = " << sl_boundary.max_l;
          dynamic_sl[point.t] = std::make_pair(box, sl_boundary);
        }
      }
    }
    if (dynamic_sl.size()) {
      AINFO_IF(FLAGS_log_enable)<< object.id << " have " << dynamic_sl.size() << " frames";
      context_->dynamic_objects_[object.id] = dynamic_sl;
    }
  }
}

bool GridGeneration::SLInBoundary(const StructSLBoundary& sl_boundary) {
  double safety_dis = half_width_ + FLAGS_collision_buff;
  int index = std::max((int)(sl_boundary.min_s / STEP_S), 0);
  if (sl_boundary.min_s < MAX_S && 
      sl_boundary.max_s > 0.0 && index < context_->grid_info_[0].size()) {
    double max_l = context_->grid_info_[0][index].front().l + safety_dis;
    double min_l = context_->grid_info_[0][index].back().l - safety_dis;
    if (sl_boundary.min_l < max_l && sl_boundary.max_l > min_l) {
      return true;
    }
  }
  return false;
}

}  //  namespace planning
}  //  namespace acu
