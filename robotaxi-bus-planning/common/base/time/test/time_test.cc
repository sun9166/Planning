#include <iostream>
#include "time/include/duration.h"
#include "time/include/node_time.h"
#include "gtest/gtest.h"

namespace acu {
namespace common {

TEST(TimeTest, constructor) {
  Time time(100UL);
  EXPECT_EQ(100UL, time.ToNanosecond());

  time = Time(1.1);
  EXPECT_EQ(1100000000UL, time.ToNanosecond());
  EXPECT_DOUBLE_EQ(1.1, time.ToSecond());

  time = Time(1, 1);
  EXPECT_EQ(1000000001UL, time.ToNanosecond());
  EXPECT_DOUBLE_EQ(1.000000001, time.ToSecond());

  Time time2(time);
  EXPECT_TRUE(time == time2);
}

TEST(TimeTest, operators) {
  Time t1(100);
  Duration d(200);
  Time t2(300);
  EXPECT_TRUE(t1 != t2);
  EXPECT_TRUE(t1 < t2);
  EXPECT_TRUE(t1 <= t2);
  EXPECT_TRUE(t2 > t1);
  EXPECT_TRUE(t2 >= t1);
  EXPECT_TRUE(t1 + d == t2);
  EXPECT_TRUE(t2 - d == t1);
  EXPECT_TRUE((t1 += d) == t2);
  EXPECT_TRUE(t1 >= t2);
  EXPECT_TRUE(t1 <= t2);
  EXPECT_TRUE(Time(100) == (t1 -= d));
}

TEST(TimeTest, to_string) {
  Time t1(1531225311123456789UL);
  std::cout << t1.ToString().c_str() << std::endl;
}

TEST(TimeTest, now) { std::cout << "Time Now: " << Time::Now() << std::endl; }

TEST(TimeTest, is_zero) {
  Time time;
  EXPECT_TRUE(time.IsZero());
  EXPECT_FALSE(Time::MAX.IsZero());
  EXPECT_FALSE(Time::MIN.IsZero());
}

}  // namespace common
}  // namespace acu
