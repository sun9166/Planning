#include "common/toolbox/optimizer/include/connect.h"
#include <iostream>
#include <limits>
#include <fstream>

namespace optimizer {
void Connect::GetPath(SiteVec &list, SiteVec &list1, SiteVec &list2) {
  list.clear();
  connect_path.clear();

  GetAnchors(list1, list2);
  mfl.Filt(connect_path);

  if ((connect_path.front() - list1.front()).mold() < 1e-5) {
    list.insert(list.begin(), connect_path.begin(), connect_path.end());
  } else {
    list.insert(list.begin(), list1.begin(), list1_connect_);
    list.insert(list.end(), connect_path.begin(), connect_path.end());
  }
  if ((connect_path.back() - list2.back()).mold() < 1e-5) {
    return;
  } else {
    list.insert(list.end(), list2_connect_, list2.end());
  }
}

void Connect::GetPath(std::vector<std::shared_ptr<Site>> &list,
                      std::vector<std::shared_ptr<Site>> &list1,
                      std::vector<std::shared_ptr<Site>> &list2) {
  // TODO
  list.clear();
  connect_path.clear();

  GetAnchors(list1, list2);
  mfl.Filt(connect_path);


  list = list1;

  for (auto &p : connect_path) {
    std::shared_ptr<Site> tmp_ptr(new Site);
    tmp_ptr->x = p.x;
    tmp_ptr->y = p.y;
    list.push_back(tmp_ptr);
  }

  for (auto &p : list2) list.push_back(p);
}

void Connect::GetAnchors(SiteVec &list1, SiteVec &list2) {
  Site list1_head(list1.front());
  Site list1_tail(list1.back());
  Site list2_head(list2.front());
  Site list2_tail(list2.back());

  list1_connect_ = list1.end() - 1;
  list2_connect_ = list2.begin();

  double min_dis = std::numeric_limits<double>::max();
  for (auto it = list1.end() - 1; it != list1.begin(); --it) {
    double tmp_dis = (*it - list2_head).mold();
    if (tmp_dis < min_dis) {
      min_dis = tmp_dis;
      list1_connect_ = it;
    } else {
      break;
    }
  }

  min_dis = std::numeric_limits<double>::max();
  for (auto it = list2.begin() + 1; it != list2.end(); ++it) {
    double tmp_dis = (*it - list1_tail).mold();
    if (tmp_dis < min_dis) {
      min_dis = tmp_dis;
      list2_connect_ = it;
    } else {
      break;
    }
  }

  connect_path.push_back(*list1_connect_);
  connect_path.push_back(*list2_connect_);

  double total_dis = 0;
  for (auto it = list1_connect_; it != list1.begin(); --it) {
    double tmp_dis = ((*it) - *(it - 1)).mold();
    total_dis += tmp_dis;
    if (total_dis > 2) {
      connect_path.insert(connect_path.begin(), *it);
      list1_connect_ = it;
      break;
    }
  }
  if (connect_path.size() != 3) {
    connect_path.insert(connect_path.begin(), list1_head);
  }

  total_dis = 0;
  for (auto it = list2_connect_; it != list2.end() - 1; ++it) {
    double tmp_dis = ((*it) - *(it - 1)).mold();
    total_dis += tmp_dis;
    if (total_dis > 2) {
      connect_path.push_back(*it);
      list2_connect_ = it;
      break;
    }
  }
  if (connect_path.size() != 4) {
    connect_path.push_back(list2_tail);
  }
}

void Connect::GetAnchors(std::vector<std::shared_ptr<Site>> &list1,
                         std::vector<std::shared_ptr<Site>> &list2) {
  Site list1_head(*list1.front());
  Site list1_tail(*list1.back());
  Site list2_head(*list2.front());
  Site list2_tail(*list2.back());

  auto list1_connect = list1.end() - 1;
  auto list2_connect = list2.begin();

  double min_dis = std::numeric_limits<double>::max();
  for (auto it = list1.end() - 1; it != list1.begin(); --it) {
    double tmp_dis = (**it - list2_head).mold();
    if (tmp_dis < min_dis) {
      min_dis = tmp_dis;
      list1_connect = it;
    } else {
      break;
    }
  }

  min_dis = std::numeric_limits<double>::max();
  for (auto it = list2.begin() + 1; it != list2.end(); ++it) {
    double tmp_dis = (**it - list1_tail).mold();
    if (tmp_dis < min_dis) {
      min_dis = tmp_dis;
      list2_connect = it;
    } else {
      break;
    }
  }

  connect_path.push_back(**list1_connect);
  connect_path.push_back(**list2_connect);

  double total_dis = 0;
  for (auto it = list1_connect; it != list1.begin(); --it) {
    double tmp_dis = ((**it) - **(it - 1)).mold();
    total_dis += tmp_dis;
    if (total_dis > 2) {
      connect_path.insert(connect_path.begin(), **it);
      list1_connect = it;
      break;
    }
  }
  if (connect_path.size() != 3) {
    connect_path.insert(connect_path.begin(), list1_head);
    list1_connect = list1.begin();
  }
  list1.erase(list1_connect,list1.end());

  total_dis = 0;
  for (auto it = list2_connect; it != list2.end() - 1; ++it) {
    double tmp_dis = ((**it) - **(it - 1)).mold();
    total_dis += tmp_dis;
    if (total_dis > 2) {
      connect_path.push_back(**it);
      list2_connect = it;
      break;
    }
  }
  if (connect_path.size() != 4) {
    connect_path.push_back(list2_tail);
    list2_connect = list2.end()-1;
  }
  list2.erase(list2.begin(), list2_connect);

}
}  // namespace optimizer
