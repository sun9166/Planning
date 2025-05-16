/******************************************************************************
* Copyright (C) 2015-2020, idriverplus(BeiJing ZhiXingZhe, Inc.)
*
* NodeName: basemap
* FileName:
*
* Description: basemap shared memory parser util.
*
* History:
* lbh         2018/07/19    1.0.0    build this module.
******************************************************************************/
#ifndef BASEMAP_SHM_PARSER_H_
#define BASEMAP_SHM_PARSER_H_

#include "basemap_shm_util.h"
#include "map/basemap/src/memory_manager_base.h"
#include "map/vectormap/include/geotool.h"
#include "map/basemap/shm_dynamic/shm_update_util.h"
#include "common/base/log/include/log.h"

namespace basemap_shm_util {

struct ShmHeader {
  uint initialized;
  uint cur_row;
  uint cur_col;
  uint dtg_initialized;
  uint dtg_cur_row;
  uint dtg_cur_col;
};

struct ShmParam {
  int shm_key = 10001;
  int shm_id;
  ull shm_size_char;

  const uint shm_row_header_chars = 32;
  const uint shm_row_atoint_offset0 = 0;
  const uint shm_row_names_offset1 = 8;
  const uint shm_row_outage_offset2 = 16;
  ull shm_row_img_chars;
  uint shm_row_names_chars;
  ull shm_row_chars;
  uint shm_rows;
  uint dtg_shm_rows;

  uint img_matrix_size = 5;
  uint dtg_img_matrix_size = 3;
  uint img_pixel_size = 0;
  uint img_pixel_chars = 4;
  double img_resolution = -1;
  double img_length = 0;
  std::string img_path;
};
struct UtmXY {
  double x = 1e-3;
  double y = 1e-3;
};

/** shm struct :
 * shm[rows][size]
 * rows the numbers of imgs , = img_matrix_size*img_matrix_size
 * size 0~4 atomic int,
 * size 4~8 img row in utm
 * size 8~12 img col in utm
 * size 12~32 outage bytes
 * size 32~size img pixels = channels*img_row*img_col
 */
using namespace acu::vectormap;
using namespace acu::map;
class ShmParser : public MemoryManagerBase {
 public:
  ShmParser(uint img_matrix_size, uint img_pixel_size, double img_resolution,
            std::string path, uint img_pixel_chars, int zone);
  ~ShmParser();
  int GetShmStatus();
  int CreatSharedMemory();
  int FreeSharedMemory();
  int NewSharedMemoryPtr();
  int DeleteSharedMemoryPtr();

  int UpdateShm(double x, double y);
  int DTGUpdateShm(double x, double y);
  int Init(double x, double y);
  void *ShmPtr() { return shm_ptr_; }
  uint GetCellBGRA(const PointGCCS &car_pgccs, const PointVCS &target_pvcs, bool &flag);
  bool GetDTGValue(const acu::vectormap::PointGCCS &car_pgccs, const acu::vectormap::PointVCS &target_pvcs, 
    double& distance, double& point_x, double& point_y);

 private:
  void ShowMapImgs(ull f);
  int WriteShmRow(void *row);
  int WriteShmRow(int i, void *row);
  int ReadShm(void *gics, void *result);
  //c++ data structure . easy to use by UpdateShm 
  std::vector<ull> GetAroundFuture(uint row, uint col);
  std::vector<ull> GetAroundNow();
  int VectorSubtract(std::vector<ull> &abc, std::vector<ull> &bcd,
                     std::vector<ull> &a, std::vector<ull> &bc,
                     std::vector<ull> &d);

  // atomic operation to atomicint in shm
  bool AtomicReadShmBegin(uint row);
  bool AtomicReadShmEnd(uint row);
  bool AtomicWriteShmBegin(uint row);
  bool AtomicWriteShmEnd(uint row);
  void AtomicInitShm(uint row);

  // easy to get the data in shm
  ShmHeader *GetShmHeader();
  int *HeaderAtomic(uint row);
  ull *HeaderFilename(uint row);
  char *Img(uint row);

  int DTGWriteShmRow(void *row);
  int DTGWriteShmRow(int i, void *row);

    //c++ data structure . easy to use by UpdateShm 
  std::vector<ull> DTGGetAroundFuture(uint row, uint col);
  std::vector<ull> DTGGetAroundNow();

  // atomic operation to atomicint in shm
  bool DTGAtomicReadShmBegin(uint row);
  bool DTGAtomicReadShmEnd(uint row);
  bool DTGAtomicWriteShmBegin(uint row);
  bool DTGAtomicWriteShmEnd(uint row);
  void DTGAtomicInitShm(uint row);

  // easy to get the data in shm
  ShmHeader *DTGGetShmHeader();
  int *DTGHeaderAtomic(uint row);
  ull *DTGHeaderFilename(uint row);

  char *DTGImgDist(uint row);
  char *DTGImgNearest(uint row);

 private:
  ShmParam shm_param_;
  int shm_status_;
  void *shm_ptr_;
  UtmXY utm_old_;
  int zone_;
  std::string img_file_;
};
}  // namespace basemap_shm_util
#endif
