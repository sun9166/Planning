#include "map/basemap/protocol_parser/png_parser.h"
#include <gtest/gtest.h>

using namespace acu::map;
/*
TEST(PNGParser, GetObject1) {
  PNGParser parser;
  cv::Vec4b vec4b = {255, 200, 100, 127};
  eCellInfo r;
  for (int i = 0; i < 1000000; i++) r = (parser.GetObject(vec4b));
  EXPECT_EQ(r, eCellInfo::SOCIAL_SIDEWALK);
}
TEST(PNGParser, GetObject2) {
  PNGParser parser;
  cv::Vec4b vec4b = {255, 250, 100, 127};
  eCellInfo r;
  for (int i = 0; i < 1000000; i++) r = (parser.GetObject(vec4b));
  EXPECT_EQ(r, eCellInfo::SOCIAL_UNDEFINE);
}
TEST(PNGParser, GetObject0) {
  PNGParser parser;
  cv::Vec4b vec4b;
#define basemap_png IDP
  for (int i = 0; i < array_size(basemap_png); i++) {
    vec4b[0] = basemap_png[i][2];
    vec4b[1] = basemap_png[i][1];
    vec4b[2] = basemap_png[i][0];
    vec4b[3] = basemap_png[i][4];
    EXPECT_EQ(static_cast<unsigned int>(parser.GetObject(vec4b)), i);
  }
}
TEST(PNGParser, GetObject00) {
  PNGParser parser;
  cv::Vec4b vec4b = {1, 20, 120, 127};
  int r;
  for (int i = 0; i < 1000; i++)
    r = static_cast<unsigned int>(parser.GetObject(vec4b));
  EXPECT_EQ(r, 0);
}*/
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}