/******************************************************************************
* Copyright (C) 2015-2020, idriverplus(BeiJing ZhiXingZhe, Inc.)
*
* NodeName: basemap
* FileName:
*
* Description: basemap shared memory update util.
*
* History:
* lbh         2018/07/19    1.0.0    build this module.
******************************************************************************/
#include "shm_update_util.h"

namespace basemap_shm_util {

ull GetGICS(double utmx, double utmy, double length) {
  if (utmx < 0 || utmy < 0 || length <= 0) return 0;
  ull tem;
  *((uint *)&tem) = uint(utmy / length);
  *((uint *)&tem + 1) = uint(utmx / length);
  return std::move(tem);
}

std::string GetFileName(ull *gics, int zone, std::string suffix) {
  std::stringstream name;
  name << *((uint *)gics) << "_" << *((uint *)gics + 1) << "_" << zone
       << suffix;

  return name.str();
}

int GetGICS(double utmx, double utmy, int image_size, double resolution,
            void *gics, int size) {
  if (utmx < 0 || utmy < 0 || resolution <= 1e-3 || image_size <= 0) return -1;
  if (size < 32) return -2;
  uint *g = (uint *)gics;
  double length = resolution * image_size;
  g[0] = uint(utmy / length);
  g[1] = uint(utmx / length);
  ull *gg = (ull *)gics + 1;
  uint t1 = uint(image_size - 1 - uint((utmy - g[0] * length) / resolution));
  uint t2 = uint((utmx - g[1] * length) / resolution);
  gg[0] = t1 * image_size + t2;
  return 0;
}

// pairs is like uint(*)[4]. 0 row, 1 col, 2 row, 3 col.
int GetUpdatePairs(ull now, ull future, uint size, void *pairs,
                   uint size_char) {
  int dr = *((uint *)&future) - *((uint *)&now);
  int dc = *((uint *)&future + 1) - *((uint *)&now + 1);
  if (dr == 0 && dc == 0) return 0;
  if (abs(dr) > size) dr = dr > 0 ? size : -size;
  if (abs(dc) > size) dc = dc > 0 ? size : -size;
  int num = abs(dr) * size + (size - abs(dr)) * abs(dc);
  uint rn = *((uint *)&now) + size / 2;
  uint cn = *((uint *)&now + 1) - size / 2;
  uint rf = *((uint *)&future) - size / 2;
  uint cf = *((uint *)&future + 1) + size / 2;
  if (num * 8 * 2 > size_char) return -1;
  for (int i = 0; i < abs(dr); i++) {
    for (int j = 0; j < size; j++) {
      ((uint *)((char(*)[16])pairs + i * size + j))[0] = rn - i;
      ((uint *)((char(*)[16])pairs + i * size + j))[1] = cn + j;
      ((uint *)((char(*)[16])pairs + i * size + j))[2] = rf + i;
      ((uint *)((char(*)[16])pairs + i * size + j))[3] = cf - j;
    }
  }
  for (int i = abs(dr); i < size; i++) {
    for (int j = 0; j < abs(dc); j++) {
      ((uint *)((char(*)[16])pairs + size * abs(dr) + (i - abs(dr)) * abs(dc) +
                j))[0] = rn - i;
      ((uint *)((char(*)[16])pairs + size * abs(dr) + (i - abs(dr)) * abs(dc) +
                j))[1] = cn + j;
      ((uint *)((char(*)[16])pairs + size * abs(dr) + (i - abs(dr)) * abs(dc) +
                j))[2] = rf + i;
      ((uint *)((char(*)[16])pairs + size * abs(dr) + (i - abs(dr)) * abs(dc) +
                j))[3] = cf - j;
    }
  }
  return num;
}

}  // namespace basemap_shm_util
