#include <gtest/gtest.h>
#include "map/map_loader/include/map_loader.h"

using namespace acu::xmlmap;
using namespace acu::map;
using namespace basemap_shm_util;

PointVCS GetVCS(int row, int col) {
  PointVCS temp;
  temp.x = col * 0.05;
  temp.y = (1023 - row) * 0.05;
  temp.angle = 0;
  return temp;
}

TEST(BaseMap, CreatSharedMemory2) {
  BaseMap *base_ = MapLoader::GetBasemapPtr();
  auto r = 0;
  r = base_->CreatSharedMemory();
  EXPECT_GE(r, 0);
  r = base_->NewSharedMemoryPtr();
  EXPECT_EQ(r, 0);
}
TEST(BaseMap, GetBasemapPtr2) {
  PointGCCS car = {440780.80000000005 , 4424448.0, 0.0};
  BaseMap *basemap_ = MapLoader::GetBasemapPtr();
  int r;
  for (int i = 0; i < 100; i++) {
    r = basemap_->UpdateShm(440780.80000000005, 4424448.0);
    std::cout<<"update: "<<r<<std::endl;
    sleep(3);
    r=basemap_->UpdateShm(440780.80000000005+570, 4424448.0);
    std::cout<<"update: "<<r<<std::endl;
    sleep(3);
  }
}
TEST(BaseMap, FreeSharedMemory2) {
  BaseMap *base = MapLoader::GetBasemapPtr();
  auto r = base->DeleteSharedMemoryPtr();
  EXPECT_EQ(r, 0);
  r = base->FreeSharedMemory();
  EXPECT_EQ(r, 0);
}
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}