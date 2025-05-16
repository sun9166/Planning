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
#include "basemap_shm_util.h"
#include <iostream>

#if 0
#define DEBUG_INFO(x)     std::cout << x << ": " << __LINE__ << std::endl
#else
#define DEBUG_INFO(x)
#endif

namespace basemap_shm_util {
// TODO: must first to creat,because size may smaller than needed
int CreatMemory(int key, ull size_char) {
  int shmid;
  void *shared;
  shmid = shmget((key_t)key, size_char, 0666 | IPC_CREAT);
  if (shmid == -1) return -1;
  return shmid;
}

int NewMemoryPtr(int key, void **shared) {
  int shmid;
  void *sh;
  shmid = shmget((key_t)key, 0, 0);// 0,0  return 0 when having existed shm
  if (shmid == -1) return -1;
  sh = shmat(shmid, 0, 0);
  if (sh == (void *)-1) return -2;
  *shared = sh;
  return shmid;
}

int NewMemoryPtr(void **shared, int shmid) {
  void *sh;
  sh = shmat(shmid, 0, 0);
  if (sh == (void *)-1) return -2;
  *shared = sh;
  return 0;
}

int WriteMemoryByFile(void *shared, std::string &filename, uint offset_char,
                      ull size_char) {
  if (shared == NULL) {
    DEBUG_INFO("shm_util");
    return -2;
  }

  std::ifstream infile;
  infile.open(filename.c_str(), std::ios::binary);
  if (!infile.is_open()) {
    DEBUG_INFO(filename);
    return -3;
  }

  infile.read((char *)shared + offset_char, size_char);
  if (infile.bad()) {
    DEBUG_INFO("shm_util");
    return -4;
  }

  infile.close();
  return 0;
}

int WriteMemoryByImageFile(void *shared, std::string &filename, uint offset_char,
                      ull size_char) {
  if (shared == NULL) {
    DEBUG_INFO("shm_util");
    return -2;
  }

  
  cv::Mat Iface = cv::imread(filename.c_str(), CV_LOAD_IMAGE_UNCHANGED);
  if (Iface.empty()) {
      // std::cout  << "failed to imread : " << filename << std::endl;
      return -3;
  }
  for (int r = 0; r < Iface.rows; r++){
      memcpy((char *)shared + offset_char + r * Iface.rows * Iface.elemSize(), 
                  reinterpret_cast<const char *>(Iface.ptr(r)),
                  Iface.cols * Iface.elemSize());
      // outfile.write(reinterpret_cast<const char *>(Iface.ptr(r)),
      //               Iface.cols * Iface.elemSize());
  }
      

  // infile.read((char *)shared + offset_char, size_char);
  // if (infile.bad()) {
  //   DEBUG_INFO("shm_util");
  //   return -4;
  // }

  // infile.close();
  return 0;
}


int WriteMemoryByData(void *shared, void *data, uint offset_char,
                      uint size_char) {
  if (shared == NULL) return -2;
  memcpy((char *)shared + offset_char, (char *)data, size_char);
  return 0;
}

int DeleteMemoryPtr(void **shared) {
  if (shmdt(*shared) == -1) return -5;
  return 0;
}

// ipcs -m    ipcrm -m [shmid]
int FreeMemory(int shmid) {
  if (shmctl(shmid, IPC_RMID, 0) == -1) return -6;
  return 0;
}

/*--------------------------------------------------------------------------
--------------------------------------------------------------------------*/

int CreatMemory(int key, uint rows, uint row_chars) {
  return CreatMemory(key, rows * row_chars);
}

int CreatMemory(int key, std::string filename, uint size_char) {
  int shmid;
  void *shared;
  shmid = shmget((key_t)key, size_char, 0666 | IPC_CREAT);
  if (shmid == -1) {
    DEBUG_INFO("shm_util");
    return -1;
  }

  shared = shmat(shmid, 0, 0);
  if (shared == (void *)-1) {
    DEBUG_INFO("shm_util");
    return -2;
  }

  std::ifstream infile;

  infile.open(filename.c_str(), std::ios::binary);
  if (!infile.is_open()) {
    DEBUG_INFO("shm_util");
    return -3;
  }

  infile.read((char *)shared, size_char);
  if (infile.bad()) {
    DEBUG_INFO("shm_util");
    return -24;
  }

  infile.close();
  if (shmdt(shared) == -1) {
    DEBUG_INFO("shm_util");
    return -5;
  }

  return shmid;
}

int WriteMemoryRow(void *shared, void *header, std::string filename,
                   uint offset_char, uint header_chars, uint file_chars) {
  int r = WriteMemoryByData(shared, header, offset_char, header_chars);
  if (r < 0) return r;
  r = WriteMemoryByFile(shared, filename, offset_char + header_chars,
                        file_chars);
  return r;
}

int WriteMemoryByFile(int shmid, std::string &filename, uint offset_char,
                      ull size_char) {
  void *shared;
  shared = shmat(shmid, 0, 0);
  if (shared == (void *)-1) {
    DEBUG_INFO("shm_util");
    return -2;
  }

  int r = WriteMemoryByFile(shared, filename, offset_char, size_char);
  if (r < 0) return r;
  if (shmdt(shared) == -1) {
    DEBUG_INFO("shm_util");
    return -5;
  }

  return shmid;
}

int WriteMemoryByData(int shmid, void *data, uint offset_char, uint size_char) {
  void *shared;
  shared = shmat(shmid, 0, 0);
  if (shared == (void *)-1) {
    DEBUG_INFO("shm_util");
    return -2;
  }

  int r = WriteMemoryByData(shared, data, offset_char, size_char);
  if (r < 0) return r;
  if (shmdt(shared) == -1) {
    DEBUG_INFO("shm_util");
    return -5;
  }

  return shmid;
}

int WriteMemoryRow(int shmid, void *header, std::string filename,
                   uint offset_char, uint header_chars, uint file_chars) {
  void *shared;
  shared = shmat(shmid, 0, 0);
  if (shared == (void *)-1) {
    DEBUG_INFO("shm_util");
    return -2;
  }

  int r = WriteMemoryRow(shared, header, filename, offset_char, header_chars,
                         file_chars);
  if (r < 0) return r;
  if (shmdt(shared) == -1) {
    DEBUG_INFO("shm_util");
    return -5;
  }

  return shmid;
}

}  // namespace basemap_shm_util