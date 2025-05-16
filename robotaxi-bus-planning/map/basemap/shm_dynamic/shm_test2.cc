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

TEST(BaseMap, GetBasemapPtr2) {
  PointGCCS car = {440780.80000000005, 4424448.0, 0.0};
  BaseMap *basemap_ = MapLoader::GetBasemapPtr();
  eCellInfo cell;
  int8_t au = 1;
  int num = 0;
  for (int j = 0; j < 1000; j++) {
    for (int i = 0; i < 100000; i++) {
      cell = basemap_->GetCellInfo(car, GetVCS(200, 200));
      // if (static_cast<unsigned int>(cell) != 13) {
      //   num++;
      // }
    }
    //std::cout << "not read:" << num << std::endl;
    //num = 0;
    // EXPECT_EQ(static_cast<unsigned int>(cell), 13);
  }
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}