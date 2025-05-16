/******************************************************************************
* Copyright (C) 2015-2020, idriverplus(BeiJing ZhiXingZhe, Inc.)
*
* NodeName: basemap
* FileName:
*
* Description: basemap shared memory util.
*
* History:
* lbh         2018/07/19    1.0.0    build this module.
******************************************************************************/
#ifndef BASEMAP_SHM_UTIL_H_
#define BASEMAP_SHM_UTIL_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/shm.h>
#include <unistd.h>
#include <fstream>
#include <opencv2/opencv.hpp>
// using namespace cv;

namespace basemap_shm_util {
using ull = unsigned long long;
// base api
int CreatMemory(int key, ull size_char);
int NewMemoryPtr(int key, void **shared);
int NewMemoryPtr(void **shared, int shmid);
int WriteMemoryByFile(void *shared, std::string& filename, uint offset_char,
                      ull size_char);
int WriteMemoryByData(void *shared, void *data, uint offset_char,
                      uint size_char);
int DeleteMemoryPtr(void **shared);
int FreeMemory(int shmid);

// derived api
int CreatMemory(int key, uint rows, uint row_chars);
int CreatMemory(int key, std::string filename, uint size_char);
int WriteMemoryRow(void *shared, void *header, std::string filename,
                   uint offset_char, uint header_chars, uint file_chars);
int WriteMemoryByFile(int shmid, std::string& filename, uint offset_char,
                      ull size_char);

int WriteMemoryByImageFile(void *shared, std::string &filename, uint offset_char,
                      ull size_char);

int WriteMemoryByData(int shmid, void *data, uint offset_char, uint size_char);

int WriteMemoryRow(int shmid, void *header, std::string filename,
                   uint offset_char, uint header_chars, uint file_chars);

}  // namespace basemap_shm_util
#endif