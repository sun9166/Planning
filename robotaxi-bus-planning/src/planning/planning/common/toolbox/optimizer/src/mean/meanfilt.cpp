
#include "common/toolbox/optimizer/include/meanfilt.h"
#include <fstream>
#include <iostream>

namespace toolbox {
namespace optimizer {
// debug by zhangzhuo
// origin fuction is faild when input points is too dense, output only two points
void MeanFilt::Interpolation(SiteVec &anchors, double density) {
  SiteVec buffer;
  buffer.reserve(5000);
  if (anchors.size() < 1 || density < 0.001) return;

  buffer.push_back(*(anchors.begin()));

  for (auto it = anchors.begin(); it != anchors.end()-1; ++it) {
    Site current_point(*it);
    Site next_point(*(it + 1));
    double tmp_dis = (next_point - current_point).mold();
    // test_i = tmp_dis;
    if (tmp_dis < density) {
      buffer.push_back(*(it + 1));
      continue;
    }

    int insert_num = round(tmp_dis / density);

    double dis = (*it - buffer.back()).mold();
    if (it != anchors.begin() && dis > 0.001 ) {
      buffer.push_back(*it);
    }

    Site delta = (next_point - current_point) / (insert_num);
    
    SiteVec insert_list;
    insert_list.clear();

    for (int i = 1; i <= insert_num; ++i) {
      insert_list.push_back((current_point + delta * i));
    }
    buffer.insert(buffer.end(), insert_list.begin(), insert_list.end());
  }
  if (std::fabs(anchors.back().x - buffer.back().x) < 1e-4 &&
      std::fabs(anchors.back().y - buffer.back().y) < 1e-4) {
  // if (anchors.back() == buffer.back()) {
    anchors.swap(buffer);
  } else {
    buffer.push_back(anchors.back());
    anchors.swap(buffer);
  }
  
}

void MeanFilt::Filt(SiteVec &to_smooth_path, double density) {
  iner_path_.clear();
  update_path_.clear();

  Interpolation(to_smooth_path, density);
  if (to_smooth_path.size() < 25) {
    return;
  }
  for (auto &i : to_smooth_path) {
    iner_path_.push_back(i);
  }

  int site_num = iner_path_.size();
  update_path_.push_back(iner_path_.front());

  if (site_num > 40) {
    for (int i = 1; i < site_num - 1; ++i) {
      Site new_site(0, 0);
      if (i < 20) {
        for (int j = 0; j < i + 1; ++j) {
          new_site += (iner_path_[i - j] + iner_path_[i + j]) / (2 * i + 2);
        }
        update_path_.push_back(new_site);
      } else if (i > site_num - 21) {
        int filt_num = 2 * (site_num - i);
        for (int j = 0; j < (site_num - i); ++j) {
          new_site += (iner_path_[i - j] + iner_path_[i + j]) / filt_num;
        }
        update_path_.push_back(new_site);
      } else {
        for (int j = 0; j < 21; ++j) {
          new_site += (iner_path_[i - j] + iner_path_[i + j]) / 42;
        }
        update_path_.push_back(new_site);
      }
    }
    update_path_.push_back(iner_path_.back());
  } else if(site_num > 20) {
    for (int i = 1; i < site_num - 1; ++i) {
      Site new_site(0, 0);
      if (i < 10) {
        for (int j = 0; j < i + 1; ++j) {
          new_site += (iner_path_[i - j] + iner_path_[i + j]) / (2 * i + 2);
        }
        update_path_.push_back(new_site);
      } else if (i > site_num - 11) {
        int filt_num = 2 * (site_num - i);
        for (int j = 0; j < (site_num - i); ++j) {
          new_site += (iner_path_[i - j] + iner_path_[i + j]) / filt_num;
        }
        update_path_.push_back(new_site);
      } else {
        for (int j = 0; j < 11; ++j) {
          new_site += (iner_path_[i - j] + iner_path_[i + j]) / 22;
        }
        update_path_.push_back(new_site);
      }
    }
    update_path_.push_back(iner_path_.back());
  } else {
    update_path_ = iner_path_;
  }

  to_smooth_path.swap(update_path_);
}

SiteVec MeanFilt::FiltSmooth(std::vector<tempPath> &smooth_path, SiteVec smoothPoints, double density) {
  // std::cout << "zzzzzzz" << std::endl;
  iner_path_.clear();
  update_path_.clear();
  std::pair<double, double> tempLimit;

  std::vector<tempPath> copy_smooth_path;
  // for (auto &i : smooth_path) {
  //   tempLimit = xy_limit(i);
  //   // if(i.Point.xg <= 0.0001 && i.Point.xg >= -0.0001 && i.Point.yg <= 0.0001 && i.Point.yg >= -0.0001) continue;
  //   // if(std::isnan(tempLimit.first) && std::isnan(tempLimit.second)) continue;
  //   copy_smooth_path.push_back(i);
  //   // iner_path_.push_back(i.Point);
  //   // std::cout << "copy_smooth_path::" << i.Point.xg << "," << i.Point.x << std::endl;
  // }
  iner_path_ = smoothPoints;

  // smooth_path.clear();

  // Interpolation(iner_path_, density);
  if (iner_path_.size() < 25) {
    return iner_path_;
  }

  int site_num = iner_path_.size();
  update_path_.push_back(iner_path_.front());

  if (site_num > 40) {
    for (int i = 1; i < site_num - 1; ++i) {
      Site new_site(0, 0);
      if (i < 20) {
        int point_count = i + 1;
        for (int j = 0; j < i + 1; ++j) {
          new_site += (iner_path_[i - j] + iner_path_[i + j]) * (point_count) / (2 * i + 2 + (1 + i + 1) * ( i + 1)) ;
          point_count--;
        }
        // std::cout << "ll" << new_site.xg << "," << new_site.x << std::endl;
        // new_site.xg = new_site.x;
        // new_site.yg = new_site.y;
        bool successFlag = nodeAfterSmooth(copy_smooth_path[i], iner_path_[i], new_site);
        if(successFlag){
          // if(new_site.xg <= 0.0001 && new_site.xg >= -0.0001 && new_site.yg <= 0.0001 && new_site.yg >= -0.0001) continue;
          update_path_.push_back(new_site);
        }
        continue;
      } else if (i > site_num - 21) {
        int filt_num = 2 * (site_num - i);
        int point_count = site_num - i;
        for (int j = 0; j < (site_num - i); ++j) {
          new_site += (iner_path_[i - j] + iner_path_[i + j]) * (point_count) / ( filt_num + (1 + site_num - i) * ( site_num - i));
        }
        // new_site.xg = new_site.x;
        // new_site.yg = new_site.y;
        bool successFlag = nodeAfterSmooth(copy_smooth_path[i], iner_path_[i], new_site);
        if(successFlag){
           // if(new_site.xg <= 0.0001 && new_site.xg >= -0.0001 && new_site.yg <= 0.0001 && new_site.yg >= -0.0001) continue;
          update_path_.push_back(new_site);
        }
        continue;
      } else {
        for (int j = 0; j < 21; ++j) {
          new_site += (iner_path_[i - j] + iner_path_[i + j]) / 42;
        }
        // new_site.xg = new_site.x;
        // new_site.yg = new_site.y;
        bool successFlag = nodeAfterSmooth(copy_smooth_path[i], iner_path_[i], new_site);
        if(successFlag){
           // if(new_site.xg <= 0.0001 && new_site.xg >= -0.0001 && new_site.yg <= 0.0001 && new_site.yg >= -0.0001) continue;
          update_path_.push_back(new_site);
        }
        continue;
      }
    }
    update_path_.push_back(iner_path_.back());
  } else if(site_num > 20) {
    for (int i = 1; i < site_num - 1; ++i) {
      Site new_site(0, 0);
      if (i < 10) {
         int point_count = i + 1;
        for (int j = 0; j < i + 1; ++j) {
          new_site += (iner_path_[i - j] + iner_path_[i + j]) * (point_count) / (2 * i + 2 + (1 + i + 1) * ( i + 1)) ;
        }
        // new_site.xg = new_site.x;
        // new_site.yg = new_site.y;
        bool successFlag = nodeAfterSmooth(copy_smooth_path[i], iner_path_[i], new_site);
        if(successFlag){
           // if(new_site.xg <= 0.0001 && new_site.xg >= -0.0001 && new_site.yg <= 0.0001 && new_site.yg >= -0.0001) continue;
          update_path_.push_back(new_site);
        }
        continue;
      } else if (i > site_num - 11) {
        int filt_num = 2 * (site_num - i);
        int point_count = site_num - i;
        for (int j = 0; j < (site_num - i); ++j) {
          new_site += (iner_path_[i - j] + iner_path_[i + j]) * (point_count) / ( filt_num + (1 + site_num - i) * ( site_num - i));
        }
        // new_site.xg = new_site.x;
        // new_site.yg = new_site.y;
        bool successFlag = nodeAfterSmooth(copy_smooth_path[i], iner_path_[i], new_site);
        if(successFlag){
           // if(new_site.xg <= 0.0001 && new_site.xg >= -0.0001 && new_site.yg <= 0.0001 && new_site.yg >= -0.0001) continue;
          update_path_.push_back(new_site);
        }
        continue;
      } else {
        for (int j = 0; j < 11; ++j) {
          new_site += (iner_path_[i - j] + iner_path_[i + j]) / 22;
        }
        // new_site.xg = new_site.x;
        // new_site.yg = new_site.y;
        bool successFlag = nodeAfterSmooth(copy_smooth_path[i], iner_path_[i], new_site);
        if(successFlag){
           // if(new_site.xg <= 0.0001 && new_site.xg >= -0.0001 && new_site.yg <= 0.0001 && new_site.yg >= -0.0001) continue;
          update_path_.push_back(new_site);
        }
        continue;
      }
    }
    update_path_.push_back(iner_path_.back());
  } else {
    update_path_ = iner_path_;
  }

  for (int i = 0; i < update_path_.size() - 1; ++i) update_path_[i].angle = (update_path_[i + 1] - update_path_[i]).inerangle();
  if (update_path_.size() > 2) update_path_.back().angle = update_path_[update_path_.size() - 2].angle;
  
  std::ofstream  fo;
  std::string path_final22 = "/home/idriver/temp/BIT-CHECK/IDP_1804_WXB_acu/final22.txt";
  std::remove(path_final22.c_str());
  // fo.open(path_final22.c_str(),std::ios::app);
    
  //   // angleFilter(update_path_);
  //   // Interpolation(update_path_);
  //   // for (int i=0; i<update_path_.size()-1; ++i) update_path_[i].angle = (update_path_[i+1] - update_path_[i]).inerangle();
  //   // if (update_path_.size() > 2)  update_path_.back().angle = update_path_[update_path_.size() - 2].angle;
  //     for (auto &p : update_path_) {
  //     // p.xg = p.x;
  //     // p.yg = p.y;
  //     fo << p.x << "," << p.y << "," << p.angle << "," << p.x << "," << p.y << "\n";
  // }
  // fo.close();
  return update_path_;
}

// SiteVec MeanFilt::FiltSmooth(std::vector<tempPath> &smooth_path, double density) {
//   // std::cout << "zzzzzzz" << std::endl;
//   iner_path_.clear();
//   update_path_.clear();
//   std::pair<double, double> tempLimit;


//   for (auto &i : smooth_path) {
//     tempLimit = xy_limit(i);
//     if(std::isnan(tempLimit.first) && std::isnan(tempLimit.second)) continue;
//     iner_path_.push_back(i.Point);
//   }

//   // Interpolation(iner_path_, density);
//   if (iner_path_.size() < 25) {
//     return update_path_;
//   }
//   // for (auto &i : to_smooth_path) {

//   int site_num = iner_path_.size();
//   update_path_.push_back(iner_path_.front());

//   if (site_num > 40) {
//     for (int i = 1; i < site_num - 1; ++i) {
//       Site new_site(0, 0);
//       if (i < 20) {
//         for (int j = 0; j < i + 1; ++j) {
//           new_site += (iner_path_[i - j] + iner_path_[i + j]) / (2 * i + 2);
//         }
//         // new_site.x = iner_path_[i - 1].x;

//         tempLimit = xy_limit(smooth_path[i - 1]);
//         std::cout << "tempYDis22::"  << tempLimit.first << "," << tempLimit.second << std::endl;
//         if(std::isnan(tempLimit.first) && std::isnan(tempLimit.second)) continue;
//         double tempYDis = new_site.y - iner_path_[i - 1].y;
//         std::cout << "isnan::" << std::endl;
//         std::cout << "tempYDis::" << tempYDis << "," << tempLimit.first << "," << tempLimit.second << std::endl;
//         if(std::isnan(tempLimit.first) && !std::isnan(tempLimit.second) && tempYDis >= 0) {
//           update_path_.push_back(iner_path_[i - 1]);
//           continue; 
//         }
//         if(std::isnan(tempLimit.second) && !std::isnan(tempLimit.first) && tempYDis <= 0) {
//           update_path_.push_back(iner_path_[i - 1]);
//           continue;
//         }
//         if(tempYDis >  tempLimit.first){
//           new_site.y = iner_path_[i - 1].y + tempLimit.first;
//           update_path_.push_back(new_site);
//           continue;
//         }
//         if(tempYDis <  tempLimit.second){
//           new_site.y = iner_path_[i - 1].y + tempLimit.second;
//           update_path_.push_back(new_site);
//           continue;
//         }
//         update_path_.push_back(new_site);
//       } else if (i > site_num - 21) {
//         int filt_num = 2 * (site_num - i);
//         for (int j = 0; j < (site_num - i); ++j) {
//           new_site += (iner_path_[i - j] + iner_path_[i + j]) / filt_num;
//         }
//         // new_site.x = iner_path_[i - 1].x;

//         tempLimit = xy_limit(smooth_path[i - 1]);
//         if(std::isnan(tempLimit.first) && std::isnan(tempLimit.second)) continue;
//         std::cout << "isnan::" << std::endl;
//         double tempYDis = new_site.y - iner_path_[i - 1].y;
//         if(std::isnan(tempLimit.first) && !std::isnan(tempLimit.second) && tempYDis >= 0) {
//           update_path_.push_back(iner_path_[i - 1]);
//           continue; 
//         }
//         if(std::isnan(tempLimit.second) && !std::isnan(tempLimit.first) && tempYDis <= 0) {
//           update_path_.push_back(iner_path_[i - 1]);
//           continue;
//         }
//         if(tempYDis >  tempLimit.first){
//           new_site.y = iner_path_[i - 1].y + tempLimit.first;
//           update_path_.push_back(new_site);
//           continue;
//         }
//         update_path_.push_back(new_site);
//       } else {
//         for (int j = 0; j < 21; ++j) {
//           new_site += (iner_path_[i - j] + iner_path_[i + j]) / 42;
//         }
//         // new_site.x = iner_path_[i - 1].x;

//         tempLimit = xy_limit(smooth_path[i - 1]);
//         if(std::isnan(tempLimit.first) && std::isnan(tempLimit.second)) continue;
//         std::cout << "isnan::" << std::endl;
//         double tempYDis = new_site.y - iner_path_[i - 1].y;
//         if(std::isnan(tempLimit.first) && !std::isnan(tempLimit.second) && tempYDis >= 0) {
//           update_path_.push_back(iner_path_[i - 1]);
//           continue; 
//         }
//         if(std::isnan(tempLimit.second) && !std::isnan(tempLimit.first) && tempYDis <= 0) {
//           update_path_.push_back(iner_path_[i - 1]);
//           continue;
//         }
//         if(tempYDis >  tempLimit.first){
//           new_site.y = iner_path_[i - 1].y + tempLimit.first;
//           update_path_.push_back(new_site);
//           continue;
//         }
//         if(tempYDis <  tempLimit.second){
//           new_site.y = iner_path_[i - 1].y + tempLimit.second;
//           update_path_.push_back(new_site);
//           continue;
//         }
//         update_path_.push_back(new_site);
//       }
//     }
//     update_path_.push_back(iner_path_.back());
//   } else if(site_num > 20) {
//     for (int i = 1; i < site_num - 1; ++i) {
//       Site new_site(0, 0);
//       if (i < 10) {
//         for (int j = 0; j < i + 1; ++j) {
//           new_site += (iner_path_[i - j] + iner_path_[i + j]) / (2 * i + 2);
//         }
//         new_site.x = iner_path_[i - 1].x;

//         tempLimit = xy_limit(smooth_path[i - 1]);
//         if(std::isnan(tempLimit.first) && std::isnan(tempLimit.second)) continue;
//         std::cout << "isnan::" << std::endl;
//         double tempYDis = new_site.y - iner_path_[i - 1].y;
//         if(std::isnan(tempLimit.first) && !std::isnan(tempLimit.second) && tempYDis >= 0) {
//           update_path_.push_back(iner_path_[i - 1]);
//           continue; 
//         }
//         if(std::isnan(tempLimit.second) && !std::isnan(tempLimit.first) && tempYDis <= 0) {
//           update_path_.push_back(iner_path_[i - 1]);
//           continue;
//         }
//         if(tempYDis >  tempLimit.first){
//           new_site.y = iner_path_[i - 1].y + tempLimit.first;
//           update_path_.push_back(new_site);
//           continue;
//         }
//         if(tempYDis <  tempLimit.second){
//           new_site.y = iner_path_[i - 1].y + tempLimit.second;
//           update_path_.push_back(new_site);
//           continue;
//         }
//         update_path_.push_back(new_site);
//       } else if (i > site_num - 11) {
//         int filt_num = 2 * (site_num - i);
//         for (int j = 0; j < (site_num - i); ++j) {
//           new_site += (iner_path_[i - j] + iner_path_[i + j]) / filt_num;
//         }
//         // new_site.x = iner_path_[i - 1].x;

//         tempLimit = xy_limit(smooth_path[i - 1]);
//         if(std::isnan(tempLimit.first) && std::isnan(tempLimit.second)) continue;
//         std::cout << "isnan::" << std::endl;
//         double tempYDis = new_site.y - iner_path_[i - 1].y;
//         if(std::isnan(tempLimit.first) && !std::isnan(tempLimit.second) && tempYDis >= 0) {
//           update_path_.push_back(iner_path_[i - 1]);
//           continue; 
//         }
//         if(std::isnan(tempLimit.second) && !std::isnan(tempLimit.first) && tempYDis <= 0) {
//           update_path_.push_back(iner_path_[i - 1]);
//           continue;
//         }
//         if(tempYDis >  tempLimit.first){
//           new_site.y = iner_path_[i - 1].y + tempLimit.first;
//           update_path_.push_back(new_site);
//           continue;
//         }
//         if(tempYDis <  tempLimit.second){
//           new_site.y = iner_path_[i - 1].y + tempLimit.second;
//           update_path_.push_back(new_site);
//           continue;
//         }
//         update_path_.push_back(new_site);
//       } else {
//         for (int j = 0; j < 11; ++j) {
//           new_site += (iner_path_[i - j] + iner_path_[i + j]) / 22;
//         }
//         // new_site.x = iner_path_[i - 1].x;

//         tempLimit = xy_limit(smooth_path[i - 1]);
//         if(std::isnan(tempLimit.first) && std::isnan(tempLimit.second)) continue;
//         std::cout << "isnan::" << std::endl;
//         double tempYDis = new_site.y - iner_path_[i - 1].y;
//         if(std::isnan(tempLimit.first) && !std::isnan(tempLimit.second) && tempYDis >= 0) {
//           update_path_.push_back(iner_path_[i - 1]);
//           continue; 
//         }
//         if(std::isnan(tempLimit.second) && !std::isnan(tempLimit.first) && tempYDis <= 0) {
//           update_path_.push_back(iner_path_[i - 1]);
//           continue;
//         }
//         if(tempYDis >  tempLimit.first){
//           new_site.y = iner_path_[i - 1].y + tempLimit.first;
//           update_path_.push_back(new_site);
//           continue;
//         }
//         if(tempYDis <  tempLimit.second){
//           new_site.y = iner_path_[i - 1].y + tempLimit.second;
//           update_path_.push_back(new_site);
//           continue;
//         }
//         update_path_.push_back(new_site);
//       }
//     }
//     update_path_.push_back(iner_path_.back());
//   } else {
//     update_path_ = iner_path_;
//   }

//   for (int i = 0; i < update_path_.size() - 1; ++i) update_path_[i].angle = (update_path_[i + 1] - update_path_[i]).inerangle();
//   if (update_path_.size() > 2) update_path_.back().angle = update_path_[update_path_.size() - 2].angle;
  
//   std::ofstream  fo;
//   std::string path_final22 = "/home/idriver/temp/BIT-CHECK/IDP_1804_WXB_acu/final22.txt";
//   std::remove(path_final22.c_str());
//   fo.open(path_final22.c_str(),std::ios::app);

//   for (auto &p : update_path_) {
//       p.xg = p.x;
//       p.yg = p.y;
//       fo << p.xg << "," << p.yg << "," << p.angle << "\n";
//   }
//     fo.close();

//   return update_path_;
//   // to_smooth_path.swap(update_path_);
// }

void MeanFilt::FiltList(SiteList &path, double density){
  // std::cout << "to_smooth_path11::" << path.size();
  std::vector<Site> to_smooth_path;
  for(auto &i : path){
    to_smooth_path.push_back(i);
  }
  int currentDirection = path.front().reverse;
  // std::cout << "to_smooth_path22::" << to_smooth_path.size();
  Filt(to_smooth_path, density);

  if(currentDirection == 0){
    for (int i=0; i<to_smooth_path.size()-1; ++i) to_smooth_path[i].angle = (to_smooth_path[i+1] - to_smooth_path[i]).inerangle();
    if (to_smooth_path.size() > 2)  to_smooth_path.back().angle = to_smooth_path[to_smooth_path.size() - 2].angle;
  }
  else{
    for (int i = 0; i < to_smooth_path.size() - 1; ++i) to_smooth_path[i].angle = (to_smooth_path[i] - to_smooth_path[i + 1]).inerangle();
    if (to_smooth_path.size() > 2) to_smooth_path.back().angle = to_smooth_path[to_smooth_path.size() - 2].angle;
  }
  // Interpolation(to_smooth_path, 0.05);
  // std::cout << "to_smooth_path33::" << to_smooth_path.size();
  path.clear();

  Site point;
  for(auto &i : to_smooth_path){
    point.xg = i.x;
    point.yg = i.y;
    point.globalangle = i.angle;
    // std::cout << "point::" << point.xg << "," << point.yg << "," << point.angle << "," << std::endl;
    point.reverse = currentDirection;
    path.push_back(point);
  }
// std::cout << "to_smooth_path44::" << path.size();
}

// std::pair<double, double> MeanFilt::xy_limit(tempPath &pointInf){
//   double r_f = 1.3;
//   double r_h_w = 0.4;
//   std::pair<double, double> move_limit;
//   std::pair<double, double> dis_limit = pointInf.dis_l_r;
//   double left_limit = dis_limit.first;
//   double right_limit = dis_limit.second;
//   if(left_limit == -1.0) left_limit = 5;
//   if(right_limit == -1.0) right_limit = 5;
//   std::cout << "xy_limit::" << left_limit << "," << right_limit << std::endl;
//   if(r_f >= left_limit && r_h_w >= left_limit){
//     std::cout << "xxxxxx" << std::endl;
//     move_limit.first = NAN;
//   }
//   else if(r_f > left_limit && r_h_w < left_limit){
//     move_limit.first = (std::fabs(left_limit - (r_f + r_h_w) / 2));
//   }
//   else if(r_f < left_limit){
//     move_limit.first = (left_limit - r_f);
//   }

//   if(r_f >= right_limit && r_h_w >= right_limit){
//     std::cout << "yyyyyy" << std::endl;
//     move_limit.second = NAN;
//   }
//   else if(r_f > right_limit && r_h_w < right_limit){
//     move_limit.second = -(std::fabs(right_limit - (r_f + r_h_w) / 2));
//   }
//   else if(r_f < right_limit){
//     move_limit.second =  r_f - right_limit;
//   }

//   return move_limit;
// }
std::pair<double, double> MeanFilt::xy_limit(tempPath &pointInf){
  double r_h = 0.03;
  // double r_h_w = 0.4;
  std::pair<double, double> move_limit;
  move_limit.first = 0;
  move_limit.second = 0;
  std::pair<double, double> dis_limit = pointInf.dis_l_r;
  double left_limit = dis_limit.first;
  double right_limit = dis_limit.second;
  if(left_limit == -1.0) left_limit = 5;
  if(right_limit == -1.0) right_limit = 5;
  if(r_h >= left_limit && r_h >= right_limit){
    move_limit.first = NAN;
    move_limit.second = NAN;
  }

  return move_limit;
}

bool MeanFilt::nodeAfterSmooth(tempPath &pointInf, const Site &beforeResult, Site &result){
  return true;
  double l_limit, r_limit;
  double dis_bound = 0.5;
  std::pair<double, double> dis_limit = pointInf.dis_l_r;
  double left_dis = dis_limit.first;
  double right_dis = dis_limit.second;
  // std::cout << "left::right_dis"  << left_dis << "," << right_dis << std::endl;
  if(left_dis == -1.0) left_dis = 5;
  if(right_dis == -1.0) right_dis = 5;

  if(left_dis < dis_bound && right_dis < dis_bound){
    // std::cout << "left::right_dis 11" << std::endl;
    result = beforeResult;
    return true;
  }
  else if(left_dis >= dis_bound-0.05 || right_dis >= dis_bound-0.05){
    
    l_limit = left_dis - dis_bound;
    r_limit = dis_bound - right_dis;
    if(l_limit < 0) l_limit = 0;
    if(r_limit > 0) r_limit = 0;
    // std::cout << "left::right_limit"  << l_limit << "," << r_limit << std::endl;
    double d_move;
    Site vcs;
    gccs2vcs(beforeResult, result, vcs);
    // Adapter::GCCS2VCS (beforeResult, result, vcs);
    d_move = vcs.y;

    if(d_move > -0.0001 && d_move < 0.0001) d_move = 0;
    // std::cout << "d_move::" <<  "," << d_move << std::endl;
    if(d_move <= l_limit && d_move >= r_limit) {
      result.x = result.xg;
      result.x = result.yg;
      // std::cout << "left::right_dis22" << "," << result.xg << "," << "," << result.yg << "," << result.x<< std::endl;

      // std::cout << "d_move11::"  << std::endl;
      return true;
    }
    else if(d_move < 0){
      Site resultFinal;
      double x_det = ( r_limit / vcs.y ) * vcs.x;
      resultFinal.x = x_det;
      resultFinal.y = r_limit;
      resultFinal.angle = vcs.angle;
      Site gccs;
      // Adapter::VCS2GCCS (beforeResult, resultFinal, gccs);
      // std::cout << "d_move22start:::"  << beforeResult.xg << "," << beforeResult.yg << "," << resultFinal.x << "," << resultFinal.y << std::endl;
      vcs2gccs(beforeResult, resultFinal, gccs);
      // std::cout << "d_move22::end::"  << gccs.xg << "," << gccs.yg << std::endl;
      result = gccs;
      result.x = result.xg;
      result.y = result.yg;
      // std::cout << "left::right_dis33"  << "," << result.xg << "," << "," << result.yg << std::endl;
      
      return true;

    }
    else if(d_move > 0){
      Site resultFinal;
      double x_det = ( l_limit / vcs.y ) * vcs.x;
      resultFinal.x = x_det;
      resultFinal.y = l_limit;
      resultFinal.angle = vcs.angle;
      Site gccs;
      // Adapter::VCS2GCCS (beforeResult, resultFinal, gccs);
      // std::cout << "left::right_dis44::" << resultFinal.x << "," << resultFinal.y << std::endl;
      vcs2gccs(beforeResult, resultFinal, gccs);
      // std::cout << "d_move33::"  << gccs.xg << "," << gccs.yg << std::endl;
      result = gccs;
      result.x = result.xg;
      result.y = result.yg;
      // std::cout << "left::right_dis44"   << "," << result.xg << "," << "," << result.yg << std::endl;
      
      return true;
    }
  }
  else {
    //  std::cout << "left::right_dis55" << std::endl;
    //  result = beforeResult;
    // return true;
  }
  //  std::cout << "left::right_dis66" << std::endl;
  // std::cout << "left_dis::" << left_dis << "," << "left_dis::" << right_dis << std::endl;
  return false;
}

void MeanFilt::gccs2vcs(const Site &beforeResultGCCS, const Site &resultFinalGCCS, Site &vcs){

  double x_det = resultFinalGCCS.xg - beforeResultGCCS.xg;
  double y_det = resultFinalGCCS.yg - beforeResultGCCS.yg;
  vcs.x = x_det * std::cos(beforeResultGCCS.angle / 180 * M_PII) + y_det * std::sin(beforeResultGCCS.angle / 180 * M_PII);
  vcs.y = y_det * std::cos(beforeResultGCCS.angle / 180 * M_PII) - x_det * std::sin(beforeResultGCCS.angle / 180 * M_PII);
  vcs.angle = resultFinalGCCS.angle - beforeResultGCCS.angle;
  vcs.xg = resultFinalGCCS.xg;
  vcs.yg = resultFinalGCCS.yg;

}

void MeanFilt::vcs2gccs(const Site &beforeResultGCCS, const Site &resultFinalVCS, Site &gccs){
  // std::cout << "vcs2gccs00::" << resultFinalVCS.x << "," << resultFinalVCS.y << std::endl;
  double xg = resultFinalVCS.x * std::cos(-beforeResultGCCS.angle / 180 * M_PII) + resultFinalVCS.y * std::sin(-beforeResultGCCS.angle / 180 * M_PII);
  double yg = resultFinalVCS.y * std::cos(-beforeResultGCCS.angle / 180 * M_PII) - resultFinalVCS.x * std::sin(-beforeResultGCCS.angle / 180 * M_PII);
  // std::cout << "vcs2gccs11::" << xg << "," << yg << std::endl;
  gccs.xg = xg + beforeResultGCCS.xg;
  gccs.yg = yg + beforeResultGCCS.yg;
  gccs.angle = resultFinalVCS.angle + beforeResultGCCS.angle;
  gccs.x = resultFinalVCS.x;
  gccs.y = resultFinalVCS.y;
  // std::cout << "vcs2gccs22::" << gccs.x << "," << gccs.y << std::endl;

}

bool MeanFilt::angleFilter(SiteVec &outPoint){
  if(outPoint.size() < 1) return false;
  SiteVec buffer;
  double tmp_angle = 0.0; //initialized by zhangzhuo [0713]
  for (auto it = outPoint.begin(); it != outPoint.end() - 1; ++it) {
    if (it == outPoint.begin()) {
      tmp_angle = std::atan2((*std::next(it)).yg - (*it).yg, (*std::next(it)).xg - (*it).xg) * 180 / M_PII;
      (*it).x = (*it).xg;
      (*it).y = (*it).yg;
      buffer.push_back(*it);
      continue;
      // if (tmp_angle > 90) continue;
    }
    auto fix_it = it;
    int count = 0;
    while(1){
      double tmp_angle_dif = std::atan2((*std::next(it)).yg - (*fix_it).yg, (*std::next(it)).xg - (*fix_it).xg) * 180 / M_PI;

      if(std::fabs(Normalize(std::fabs(tmp_angle_dif - tmp_angle)))  > 110){
        ++it;
        // count++;
        if (it == outPoint.end()) break;
        // if(count > 4) break;
        continue;
      }
      tmp_angle = tmp_angle_dif;
      (*it).x = (*it).xg;
      (*it).y = (*it).yg;
      buffer.push_back(*it);
      break;
    }
    if (it == outPoint.end()) break;
  }

  outPoint.clear();
  outPoint = buffer;
  return true;
}

double MeanFilt::Normalize(double angle) {
  while (angle < -180) {
    angle += 360;
  }

  while (angle > 180) {
    angle -= 360;
  }

  return angle;
}

}  // optimizer
}  // toolbox 
