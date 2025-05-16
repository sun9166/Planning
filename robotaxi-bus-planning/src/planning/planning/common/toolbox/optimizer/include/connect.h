#ifndef COMMON_TOOLBOX_OPTIMIZER_INCLUDE_CONNECT_H__
#define COMMON_TOOLBOX_OPTIMIZER_INCLUDE_CONNECT_H__

#include "common/toolbox/geometry/include/geoheader.h"
#include "common/toolbox/optimizer/include/meanfilt.h"

#include <memory>
#include <vector>

#include <gtest/gtest.h>

using geometry::Site;
using geometry::SiteVec;
using toolbox::optimizer::MeanFilt;

namespace optimizer {
class Connect {
 public:
  Connect() = default;
  ~Connect() = default;

 public:
  void GetPath(SiteVec &list, SiteVec &list1, SiteVec &list2);
  void GetPath(std::vector<std::shared_ptr<Site>> &list,
               std::vector<std::shared_ptr<Site>> &list1,
               std::vector<std::shared_ptr<Site>> &list2);

 private:
  void GetAnchors(SiteVec &list1, SiteVec &list2);
  void GetAnchors(std::vector<std::shared_ptr<Site>> &list1,
                  std::vector<std::shared_ptr<Site>> &list2);

 private:
  MeanFilt mfl;
  SiteVec connect_path;
  SiteVec::iterator list1_connect_;
  SiteVec::iterator list2_connect_;


private:
  FRIEND_TEST(connectTest, GetAnchorsNormalList);
  FRIEND_TEST(connectTest, GetAnchorsShortList);
  FRIEND_TEST(connectTest, GetAnchorsEquleList);
  FRIEND_TEST(connectTest, pGetAnchorsNormalList);
  FRIEND_TEST(connectTest, pGetAnchorsShortList);
  FRIEND_TEST(connectTest, pGetAnchorsEquleList);
  FRIEND_TEST(connectTest, GetPathNormalList);
  FRIEND_TEST(connectTest, GetPathShortList);
  FRIEND_TEST(connectTest, pGetPathNormalList);
  FRIEND_TEST(connectTest, pGetPathShortList);
};

}  // optimizer
#endif  // __TOOLBOX_OPTIMIZER_CONNECT_H__
