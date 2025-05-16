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
#ifndef BASEMAP_SHM_UPDATE_UTIL_H_
#define BASEMAP_SHM_UPDATE_UTIL_H_

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <utility>
#include <vector>
namespace basemap_shm_util {

using ull = unsigned long long;

ull GetGICS(double utmx, double utmy, double length);

std::string GetFileName(ull *gics, int zone, std::string suffix = ".image");

int GetGICS(double utmx, double utmy, int image_size, double resolution,
            void *gics, int size);

int GetUpdatePairs(ull now, ull future, uint size, void *pairs, uint size_char);

}  // namespace basemap_shm_util
#endif