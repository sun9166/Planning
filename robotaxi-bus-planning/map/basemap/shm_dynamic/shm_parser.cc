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
#include "shm_parser.h"
#include <opencv2/core/core.hpp> 
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iomanip>
#include <sys/time.h>

namespace basemap_shm_util {

ShmParser::ShmParser(uint img_matrix_size, uint img_pixel_size,
                     double img_resolution, std::string path,
                     uint img_pixel_chars, int zone) {
  AINFO << "shm parser params:-------------------";
  AINFO << "img_matrix_size: " << img_matrix_size;
  AINFO << "img_pixel_size: " << img_pixel_size;
  AINFO << "img_resolution: " << img_resolution;
  AINFO << "img_pixel_chars: " << img_pixel_chars;
  AINFO << "path: " << path;
  AINFO << "-------------------------------------";
  shm_status_ = 0;
  if ((img_matrix_size % 2) == 0 || (img_matrix_size % 2) < 0) {
    AERROR << __LINE__;
    shm_status_ = -11;
    return;
  }
  if (img_pixel_size <= 0) {
    AERROR << __LINE__;
    shm_status_ = -12;
    return;
  }
  if (img_pixel_chars <= 0) {
    AERROR << __LINE__;
    shm_status_ = -13;
    return;
  }
  if (img_resolution <= 1e-4) {
    AERROR << __LINE__;
    shm_status_ = -14;
    return;
  }
  // if (img_pixel_size % 2 == 1 && img_pixel_size != 1) {
  //   shm_status_ = -2;
  //   return;
  // }
  shm_param_.img_matrix_size = img_matrix_size;
  shm_param_.img_pixel_size = img_pixel_size;
  shm_param_.dtg_img_matrix_size = 3;

  shm_param_.img_resolution = img_resolution;
  shm_param_.img_length = img_resolution * img_pixel_size;
  shm_param_.img_pixel_chars = img_pixel_chars;
  shm_param_.img_path = path;

  shm_param_.shm_rows = img_matrix_size * img_matrix_size;
  shm_param_.dtg_shm_rows = shm_param_.dtg_img_matrix_size * shm_param_.dtg_img_matrix_size;
  shm_param_.shm_row_img_chars =
    img_pixel_size * img_pixel_size * img_pixel_chars;

  shm_param_.shm_row_chars = shm_param_.shm_row_header_chars +
                             shm_param_.shm_row_img_chars + 8 -
                             shm_param_.shm_row_img_chars % 8;
  shm_param_.shm_row_names_chars =
    shm_param_.shm_row_outage_offset2 - shm_param_.shm_row_names_offset1;
  shm_param_.shm_size_char = sizeof(ShmHeader) + shm_param_.shm_row_chars * shm_param_.shm_rows + 
    shm_param_.shm_row_chars * shm_param_.dtg_shm_rows + 
    shm_param_.shm_row_chars * shm_param_.dtg_shm_rows;
  zone_ = zone;
  shm_ptr_ = NULL;
  NewSharedMemoryPtr();
}

ShmParser::~ShmParser() {
  if (shm_ptr_ != NULL) DeleteSharedMemoryPtr();
}

int ShmParser::CreatSharedMemory() {
  if (shm_status_ < 0) {
    std::cout << "shm parser status error " << std::endl;
    return shm_status_;
  }
  int r = NewMemoryPtr(shm_param_.shm_key, &shm_ptr_);
  if (r > -1) {
    std::cout << "WARNING: shared memory had existed... shmid: " << r
              << std::endl;
    return 1;  // shm had existed
  }
  int shmid = CreatMemory(shm_param_.shm_key, shm_param_.shm_size_char);
  std::cout << "Creat shared memory, shmid: " << shmid << std::endl;
  if (shmid == -1) {
    shm_status_ = -10000;  // creating shm failed
    return -1;
  }
  // for (int i = 0; i < shm_param_.shm_rows; i++) {
  //   AtomicInitShm(i);
  // }
  shm_param_.shm_id = shmid;
  return 0;
}

int ShmParser::FreeSharedMemory() {
  std::cout << "free shared memory" << std::endl;
  return FreeMemory(shm_param_.shm_id);
}

int ShmParser::NewSharedMemoryPtr() {
  errno=0;
  int shmid = NewMemoryPtr(shm_param_.shm_key, &shm_ptr_);
  if (shmid < 0) {
    std::cout << "New shared memory ptr (key:" << shm_param_.shm_key << ") failed: " << shmid << std::endl;
    perror("NewSharedMemoryPtr");
    shm_ptr_ = NULL;
    return shmid;
  }
  shm_param_.shm_id = shmid;
  std::cout << "New shared memory ptr succeed" << std::endl;
  return 0;
}

int ShmParser::DeleteSharedMemoryPtr() {
  std::cout << "delete shared memory ptr" << std::endl;
  int r = DeleteMemoryPtr(&shm_ptr_);
  if (r == 0) shm_ptr_ = NULL;
  return r;
}

void distanceTransform1(cv::Mat _src, cv::Mat& _dst, cv::Mat& _nearest,
                            int distType, int maskSize, int labelType);

int ShmParser::GetShmStatus() { return shm_status_; }
int ShmParser::Init(double x, double y)
{
  int ret = UpdateShm(x, y);
  // DTGUpdateShm(x, y); // just used in unstructed map, disabled in XingJi project
  return  ret;
}

int ShmParser::UpdateShm(double x, double y) {
  ull f = GetGICS(x, y, shm_param_.img_length);
  // std::cout << GetFileName((ull *)&f, 50) << std::endl;

  std::vector<ull> fut = GetAroundFuture(((uint *)&f)[0], ((uint *)&f)[1]);
  std::vector<ull> now = GetAroundNow();
  std::vector<ull> a, bc, d;
  VectorSubtract(now, fut, a, bc, d);

  if (a.size() == 0 && d.size() == 0) {
    return 1;
  }
  for (int i = 0; i < a.size(); i++) {
    ull t[2];
    t[0] = a[i];
    t[1] = d[i];
    // std::cout << "========== t[0] " << ((uint *)&t[0])[0] << "_" << ((uint *)&t[0])[1] << std::endl;
    // std::cout << "========== t[1] " << ((uint *)&t[1])[0] << "_" << ((uint *)&t[1])[1] << std::endl;
    // std::cout << t[0] << " , " << t[1] << std::endl;
    int error = WriteShmRow(t);
    if (error == -10000) return -10000;
  }

  return d.size();
}

void ShmParser::ShowMapImgs(ull f)
{
  cv::Mat src = cv::Mat::zeros(shm_param_.img_pixel_size * shm_param_.img_matrix_size, 
      shm_param_.img_pixel_size * shm_param_.img_matrix_size, CV_8UC1);
  cv::Mat dist = cv::Mat::zeros(shm_param_.img_pixel_size * shm_param_.dtg_img_matrix_size,
      shm_param_.img_pixel_size * shm_param_.dtg_img_matrix_size, CV_32F);
  cv::Mat nearest = cv::Mat::zeros(shm_param_.img_pixel_size * shm_param_.dtg_img_matrix_size, 
      shm_param_.img_pixel_size * shm_param_.dtg_img_matrix_size, CV_8UC4);

  for (int i1=0; i1 < shm_param_.img_matrix_size; i1++) {
    for (int j1=0; j1 < shm_param_.img_matrix_size; j1++) {
      ull img_row;
      ((uint*)&img_row)[1] = ((uint*)&f)[1] + i1 - shm_param_.img_matrix_size/2;
      ((uint*)&img_row)[0] = ((uint*)&f)[0] + j1 - shm_param_.img_matrix_size/2;

      if (0 == img_row) continue;

      cv::Rect rect(shm_param_.img_pixel_size * i1, shm_param_.img_pixel_size * j1, 
            shm_param_.img_pixel_size, shm_param_.img_pixel_size);
      cv::Mat roi_src = src(rect);

      for (int x=0; x < shm_param_.shm_rows; x++) {
        // AINFO << "x=" << x;
        if (*HeaderFilename(x) == img_row) {
          // AINFO << "img_row: " << ((uint*)&img_row)[0] << "_" << ((uint*)&img_row)[1];
          for (int i2 = 0; i2 < shm_param_.img_pixel_size; i2++) {
            for (int j2 = 0; j2 < shm_param_.img_pixel_size; j2++) {
              // AINFO << "i2=" << i2 << ", j2=" << j2;
              roi_src.at<uchar>(i2, j2) = (uchar)*(Img(x) + (i2 * shm_param_.img_pixel_size + j2) * shm_param_.img_pixel_chars);
            }
          }
        }
      }
    }
  }

  for (int i1=0; i1 < shm_param_.dtg_img_matrix_size; i1++) {
    for (int j1=0; j1 < shm_param_.dtg_img_matrix_size; j1++) {
      ull img_row;
      ((uint*)&img_row)[1] = ((uint*)&f)[1] + i1 - shm_param_.dtg_img_matrix_size/2;
      ((uint*)&img_row)[0] = ((uint*)&f)[0] + j1 - shm_param_.dtg_img_matrix_size/2;

      if (0 == img_row) continue;

      cv::Rect rect(shm_param_.img_pixel_size * i1, shm_param_.img_pixel_size * j1, 
            shm_param_.img_pixel_size, shm_param_.img_pixel_size);
      cv::Mat roi_dist = dist(rect);
      cv::Mat roi_nearest = nearest(rect);

      for (int x=0; x < shm_param_.dtg_shm_rows; x++) {
        // AINFO << "x=" << x;

        if (*DTGHeaderFilename(x) == img_row) {
          float* dst_ptr = (float*)DTGImgDist(x);
          unsigned int* nearest_ptr = (unsigned int*)DTGImgNearest(x);

          // AINFO << "dtg_img_row: " << ((uint*)&img_row)[0] << "_" << ((uint*)&img_row)[1];
          for (int i2 = 0; i2 < shm_param_.img_pixel_size; i2++) {
            for (int j2 = 0; j2 < shm_param_.img_pixel_size; j2++) {
              // AINFO << "i2=" << i2 << ", j2=" << j2;
              roi_dist.at<float>(i2, j2) = *(dst_ptr + i2 * shm_param_.img_pixel_size + j2);
              roi_nearest.at<cv::Vec2w>(i2, j2)[1] = *(unsigned short*)(nearest_ptr + i2 * shm_param_.img_pixel_size + j2);
              roi_nearest.at<cv::Vec2w>(i2, j2)[0] = *((unsigned short*)(nearest_ptr + i2 * shm_param_.img_pixel_size + j2) + 1);
            }
          }
        }
      }
    }
  }

  std::string file_name;
  file_name = "/work/share/pic/src.bmp";
  cv::imwrite(file_name, src);
  file_name = "/work/share/pic/dst.bmp";
  cv::imwrite(file_name, dist);
  file_name = "/work/share/pic/nearest.bmp";
  cv::imwrite(file_name, nearest);
}

// just used in unstructed map, disabled in XingJi project
int ShmParser::DTGUpdateShm(double x, double y) {
  ull f = GetGICS(x, y, shm_param_.img_length);

  // std::cout << GetFileName((ull *)&f, 50) << std::endl;

  std::vector<ull> fut = DTGGetAroundFuture(((uint *)&f)[0], ((uint *)&f)[1]);
  std::vector<ull> now = DTGGetAroundNow();
  std::vector<ull> a, bc, d;
  VectorSubtract(now, fut, a, bc, d);

  if (a.size() == 0 && d.size() == 0) {
    return 1;
  }
  for (int i = 0; i < a.size(); i++) {
    ull t[2];
    t[0] = a[i];
    t[1] = d[i];
    // std::cout << "========== t[0] " << ((uint *)&t[0])[0] << "_" << ((uint *)&t[0])[1] << std::endl;
    // std::cout << "========== t[1] " << ((uint *)&t[1])[0] << "_" << ((uint *)&t[1])[1] << std::endl;
    // std::cout << t[0] << " , " << t[1] << std::endl;
    int error = DTGWriteShmRow(t);
    if (error == -10000) return -10000;
  }

  // ShowMapImgs(f);

  return d.size();
}

bool ShmParser::GetDTGValue(const acu::vectormap::PointGCCS &car_pgccs, const acu::vectormap::PointVCS &target_pvcs, 
    double& distance, double& point_x, double& point_y) {
  unsigned long long gics[2];

  PointGCCS  gccs;
  PointVCS car_pvcs = {0, 0, 0};
  // AINFO << std::fixed << "car_pgccs (" << car_pgccs.xg << ", " << car_pgccs.yg << ")";
 
  if (GeoTransform(car_pgccs, target_pvcs, gccs) == -1) {
    AERROR << "GeoTransform target_pvcs failed";
    return false;
  }
  if (4 < shm_param_.img_pixel_chars) {
    AINFO << "shm_param_.img_pixel_chars failed";
    return false;
  }

  int r = GetGICS(gccs.xg, gccs.yg, shm_param_.img_pixel_size,
                  shm_param_.img_resolution, gics, 32);
  if (r < 0) {
    AINFO << ", GetGICS gics failed";
    return false;
  }

  double pic_size = shm_param_.img_pixel_size * shm_param_.img_resolution;

  for (int i=0; i < shm_param_.dtg_shm_rows; i++) {
    // AINFO << "i=" << i;
    if (*DTGHeaderFilename(i) == *((ull *)gics)) {
      if (DTGAtomicReadShmBegin(i)) {
        float* dst_ptr = (float*)DTGImgDist(i);
        unsigned int* nearest_ptr = (unsigned int*)DTGImgNearest(i);

        distance = *((float*)dst_ptr + *((ull *)gics + 1));
        point_x = (double)*(unsigned short*)(nearest_ptr + *((ull *)gics + 1));
        point_y = (double)*((unsigned short*)(nearest_ptr + *((ull *)gics + 1)) + 1);

        // AINFO << std::fixed << "1: " << distance << "=(" << point_y << "," << point_y << ") ";
        // AINFO << "gics[0] (" << ((uint*)&gics[0])[1] << "," << ((uint*)&gics[0])[0] << ") ";
        // AINFO << "gics[1] (" << ((uint*)&gics[1])[1] << "," << ((uint*)&gics[1])[0] << ") ";

        distance = distance * shm_param_.img_resolution;
        point_x = (((uint *)gics)[1]) * pic_size + point_x * shm_param_.img_resolution;
        point_y = (((uint *)gics)[0]) * pic_size + 50 - point_y * shm_param_.img_resolution;

        // AINFO << std::fixed << "2: "  << distance << "=(" << point_y << "," << point_y << ") ";

        if (!AtomicReadShmEnd(i)) return false;
        return true;
      } else {
        continue;
      }
    }
  }

  return false;
}


std::vector<ull> ShmParser::GetAroundFuture(uint row, uint col) {
  std::vector<ull> tem;
  int n = shm_param_.img_matrix_size / 2;
  ull te;
  for (int i = -n; i < n + 1; ++i)
    for (int j = -n; j < n + 1; ++j) {
      ((uint *)&te)[0] = row + i;
      ((uint *)&te)[1] = col + j;
      tem.push_back(te);
    }
  return std::move(tem);
}

std::vector<ull> ShmParser::GetAroundNow() {
  std::vector<ull> tem;
  for (int j = 0; j < shm_param_.shm_rows; ++j) {
    tem.push_back(*HeaderFilename(j));
  }
  return std::move(tem);
}

std::vector<ull> ShmParser::DTGGetAroundFuture(uint row, uint col) {
  std::vector<ull> tem;
  int n = shm_param_.dtg_img_matrix_size / 2;
  ull te;
  for (int i = -n; i < n + 1; ++i)
    for (int j = -n; j < n + 1; ++j) {
      ((uint *)&te)[0] = row + i;
      ((uint *)&te)[1] = col + j;
      tem.push_back(te);
    }
  return std::move(tem);
}

std::vector<ull> ShmParser::DTGGetAroundNow() {
  std::vector<ull> tem;
  for (int j = 0; j < shm_param_.dtg_shm_rows; ++j) {
    tem.push_back(*DTGHeaderFilename(j));
  }
  return std::move(tem);
}

int ShmParser::VectorSubtract(std::vector<ull> &abc, std::vector<ull> &bcd,
                              std::vector<ull> &a, std::vector<ull> &bc,
                              std::vector<ull> &d) {
  a.clear();
  bc.clear();
  d.clear();
  for (auto &ai : abc) {
    if (ai != 0 && std::count(abc.begin(), abc.end(), ai) >= 2) ai = 0;
  }
  for (auto ai : abc) {
    if (std::count(bcd.begin(), bcd.end(), ai) == 0)
      a.push_back(ai);
    else
      bc.push_back(ai);
  }
  for (auto di : bcd) {
    if (std::count(bc.begin(), bc.end(), di) == 0) d.push_back(di);
  }
  return 0;
}

int ShmParser::WriteShmRow(void *row) {
  for (int i = 0, j=0; i < shm_param_.shm_rows; i++) {
    if (*(HeaderFilename(i)) == *((ull *)row)) {
      // std::cout << "i: " << i << ", row: " << row << std::endl;
      int ret =  WriteShmRow(i, row);
      return ret;
    }
  }
  return -1;  // no this file in shm, shm needes to check.
}

int ShmParser::WriteShmRow(int i, void *row) {
  if (*HeaderAtomic(i) != -1)
    if (AtomicWriteShmBegin(i)) {
      img_file_ = shm_param_.img_path + GetFileName((ull *)row + 1, zone_);
      
      // struct timeval current_time;
			// gettimeofday(&current_time, NULL);
			// double time1 = current_time.tv_sec + (double)current_time.tv_usec / 1000000.0;
      // int r = WriteMemoryByFile(Img(i), img_file_, 0, shm_param_.shm_row_img_chars);
      int r = WriteMemoryByImageFile(Img(i), img_file_, 0, shm_param_.shm_row_img_chars);

      
      if (r < 0) {
        *HeaderFilename(i) = 0;
      } else {
        // gettimeofday(&current_time, NULL);
        // double time2 = current_time.tv_sec + (double)current_time.tv_usec / 1000000.0;
        // std::cout << "file_name :" << img_file_ ;
        // std::cout << std::setprecision(17) << ", t2-t1: " << (time2 - time1) * 1000 << "(ms)" << std::endl;

        if (WriteMemoryByData(HeaderFilename(i), ((ull *)row + 1), 0,
                              shm_param_.shm_row_names_chars) < 0) {
          *HeaderFilename(i) = 0;
        } else {
          *(HeaderFilename(i)) = *((ull *)row + 1); // save name of new tile
        }
      }

      if (*HeaderAtomic(i) != -1)
        std::cout << "atomicint: " << *HeaderAtomic(i) << std::endl;
      if (!AtomicWriteShmEnd(i)) {
        std::cout << "AtomicWriteShmEnd(i):" << i << "("
                  << ((uint *)HeaderFilename(i))[0] << " , "
                  << ((uint *)HeaderFilename(i))[1]
                  << ") failed. atomicint: " << *HeaderAtomic(i) << std::endl;
        return -10000;  // unbelievable error to end writing
      }

      return 1;  // writing success
    } else {
      return 0;  // other is writing.
    }
}

int ShmParser::DTGWriteShmRow(void *row) {
  for (int i = 0, j=0; i < shm_param_.dtg_shm_rows; i++) {
    if (*(DTGHeaderFilename(i)) == *((ull *)row)) {
      int ret =  DTGWriteShmRow(i, row);
      return ret;
    }
  }
  return -1;  // no this file in shm, shm needes to check.
}

int ShmParser::DTGWriteShmRow(int i, void *row) {
  if (*DTGHeaderAtomic(i) != -1) {
    if (DTGAtomicWriteShmBegin(i)) {
      ull t1[2];
      t1[0] = *((ull *)row);
      t1[1] = *((ull *)row + 1);
      // AINFO << "DTGWriteShmRow old block position: " << ((uint*)&t1[0])[0] << "_" << ((uint*)&t1[0])[1];
      // AINFO << "DTGWriteShmRow new block position: " << ((uint*)&t1[1])[0] << "_" << ((uint*)&t1[1])[1];
      // AINFO << "DTGWriteShmRow data store index : " << i;

      std::string img_name = std::to_string(((uint*)&t1[0])[0]) + "_" + std::to_string(((uint*)&t1[0])[1]);

      // put image t1[1] into rows[i]
      cv::Mat src = cv::Mat::zeros(shm_param_.img_pixel_size * shm_param_.dtg_img_matrix_size, 
          shm_param_.img_pixel_size * shm_param_.dtg_img_matrix_size, CV_8UC1);


      for (int i1=0; i1 < shm_param_.dtg_img_matrix_size; i1++) {
        for (int j1=0; j1 < shm_param_.dtg_img_matrix_size; j1++) {
          ull img_row;
          ((uint*)&img_row)[1] = ((uint*)&t1[1])[1] + i1 - shm_param_.dtg_img_matrix_size/2;
          ((uint*)&img_row)[0] = ((uint*)&t1[1])[0] + j1 - shm_param_.dtg_img_matrix_size/2;
          // AINFO << "DTGWriteShmRow new block position: " << ((uint*)&img_row)[0] << "_" << ((uint*)&img_row)[1];

          if (0 == img_row) continue;

          cv::Rect rect(shm_param_.img_pixel_size * i1, shm_param_.img_pixel_size * j1, 
                shm_param_.img_pixel_size, shm_param_.img_pixel_size);
          cv::Mat roi_src = src(rect);

          for (int x=0; x < shm_param_.shm_rows; x++) {
            // AINFO << "x=" << x;
            if (*HeaderFilename(x) == img_row) {
              AINFO << "img_row: " << ((uint*)&t1[0])[0] << "_" << ((uint*)&t1[0])[1];
              for (int i2 = 0; i2 < shm_param_.img_pixel_size; i2++) {
                for (int j2 = 0; j2 < shm_param_.img_pixel_size; j2++) {
                  // AINFO << "i2=" << i2 << ", j2=" << j2;
                  roi_src.at<uchar>(i2, j2) = (uchar)*(Img(x) + (i2 * shm_param_.img_pixel_size + j2) * shm_param_.img_pixel_chars);
                }
              }
            }
          }
        }
      }

      cv::threshold(src, src, 2, 255, cv::THRESH_BINARY);


      cv::Mat dstImage;
      cv::Mat nearest;
      distanceTransform1(src, dstImage, nearest, CV_DIST_L1, CV_DIST_MASK_3, CV_DIST_LABEL_PIXEL);

      cv::Rect roi_rect(shm_param_.img_pixel_size, shm_param_.img_pixel_size, 
          shm_param_.img_pixel_size, shm_param_.img_pixel_size);
      cv::Mat roi_dstImage = dstImage(roi_rect);
      cv::Mat roi_nearest = nearest(roi_rect);
      // std::cout << "roi start (" << shm_param_.img_pixel_size * i << ", " << shm_param_.img_pixel_size * j << ")" << std::endl;
      // std::cout << "roi size (" << roi_dstImage.rows << ", " << roi_dstImage.cols << ")" << std::endl;
      float* dst_ptr = (float*)DTGImgDist(i);
      unsigned int* nearest_ptr = (unsigned int*)DTGImgNearest(i);

      for (int i1=0; i1<roi_dstImage.rows; i1++) {
        for (int j1=0; j1<roi_dstImage.cols; j1++) {
          *(dst_ptr + i1 * roi_dstImage.rows + j1) = roi_dstImage.at<float>(i1, j1);
          if ((roi_nearest.at<cv::Vec2w>(i1, j1)[1] == 0) && (roi_nearest.at<cv::Vec2w>(i1, j1)[0] == 0)) {
            *(unsigned short*)(nearest_ptr + i1 * roi_dstImage.rows + j1) = 0;
            *((unsigned short*)(nearest_ptr + i1 * roi_dstImage.rows + j1) + 1) = 0;
          } else {
            *(unsigned short*)(nearest_ptr + i1 * roi_dstImage.rows + j1) = 
                roi_nearest.at<cv::Vec2w>(i1, j1)[1] - shm_param_.img_pixel_size;
            *((unsigned short*)(nearest_ptr + i1 * roi_dstImage.rows + j1) + 1) = 
                roi_nearest.at<cv::Vec2w>(i1, j1)[0] - shm_param_.img_pixel_size;
          }
        }
      }

      // std::string file_name;
      // file_name = "/work/share/pic/src_" + img_name + ".bmp";
      // cv::imwrite(file_name, src);
      // file_name = "/work/share/pic/dst_" + img_name + ".bmp";
      // cv::imwrite(file_name, dstImage);
      // file_name = "/work/share/pic/nearest_" + img_name + ".bmp";
      // cv::imwrite(file_name, nearest);

      // file_name = "/work/share/pic/roi_dst_" + img_name + ".bmp";
      // cv::imwrite(file_name, roi_dstImage);
      // file_name = "/work/share/pic/roi_nearest_" + img_name + ".bmp";
      // cv::imwrite(file_name, roi_nearest);

      // save name of new tile
      *(DTGHeaderFilename(i)) = *((ull *)row + 1);


      if (*DTGHeaderAtomic(i) != -1)
        std::cout << "atomicint: " << *DTGHeaderAtomic(i) << std::endl;
      if (!DTGAtomicWriteShmEnd(i)) {
        std::cout << "DTGAtomicWriteShmEnd(i):" << i << "("
                  << ((uint *)DTGHeaderFilename(i))[0] << " , "
                  << ((uint *)DTGHeaderFilename(i))[1]
                  << ") failed. atomicint: " << *DTGHeaderAtomic(i) << std::endl;
        return -10000;  // unbelievable error to end writing
      }

      // std::string file_name;
      // file_name = "/work/share/pic/src_" + img_name + ".bmp";
      // cv::imwrite(file_name, src);
      // file_name = "/work/share/pic/dst_" + img_name + ".bmp";
      // cv::imwrite(file_name, dstImage);
      // file_name = "/work/share/pic/nearest_" + img_name + ".bmp";
      // cv::imwrite(file_name, nearest);

    }
    return 1;  // writing success
  } else {
    return 0;  // other is writing.
  }
}

uint ShmParser::GetCellBGRA(const PointGCCS &car_pgccs,
                            const PointVCS &target_pvcs, 
                            bool &flag) {
  acu::vectormap::PointGCCS gccs;
  flag = false;
  if (GeoTransform(car_pgccs, target_pvcs, gccs) == -1) {
    return 0;
  }
  if (4 < shm_param_.img_pixel_chars) {
    return 0;
  }
  ull gics[2];
  int r = GetGICS(gccs.xg, gccs.yg, shm_param_.img_pixel_size,
                  shm_param_.img_resolution, gics, 32);
  if (r < 0) return 0;
  //debug
  // ROS_ERROR_STREAM("flag  and r: " << flag << " , " << r);
  // ROS_ERROR_STREAM("gccs : " << gccs.xg << " , " << gccs.yg);
  // ROS_ERROR_STREAM("read target pose: " << ((uint *)gics)[0] << " , " << ((uint *)gics)[1]<< "//" << gics[1] );
  uint result;
  r = ReadShm(gics, &result);
  if (r < 0) return 0;
  //debug 
  // uint p_b = uint (*((unsigned char *)&result + 0));
  // uint p_g = uint (*((unsigned char *)&result + 1));
  // uint p_r = uint (*((unsigned char *)&result + 2));
  // uint p_a = uint (*((unsigned char *)&result + 2));
  // ROS_ERROR_STREAM("flag  and r: " << flag << " , " << r);
  // ROS_ERROR_STREAM(" target result :"<< p_b << " , " << p_g << " , " << p_r << " , " << p_a);

  flag = true;

  return std::move(result);
}

int ShmParser::ReadShm(void *gics, void *result) {
  for (int i = 0; i < shm_param_.shm_rows; i++) {
    if (*(HeaderFilename(i)) == *((ull *)gics)) {
      if (AtomicReadShmBegin(i)) {
        // memcpy((char *)result,
        //        (char *)(Img(i)) +
        //            (*((ull *)gics + 1)) * shm_param_.img_pixel_chars,
        //        shm_param_.img_pixel_chars);

        // asign is faster than memcpy for this condition
        // AINFO << std::hex << (uint*)((Img(i)) + * ((ull *)gics + 1) * shm_param_.img_pixel_chars);
        *((uint *)result) =
          *((uint *)((Img(i)) +
                     * ((ull *)gics + 1) * shm_param_.img_pixel_chars));
        if (!AtomicReadShmEnd(i)) return -10000;
        return 0;
      } else {
        continue;
      }
    }
  }
  return -10000;
}

//writing u cannt read, reading u can write. a small small bug.
bool ShmParser::AtomicReadShmBegin(uint row) {
  return __sync_bool_compare_and_swap(HeaderAtomic(row), 0, 0);
}
bool ShmParser::AtomicReadShmEnd(uint row) {
  return true;
}

bool ShmParser::AtomicWriteShmBegin(uint row) {
  return __sync_bool_compare_and_swap(HeaderAtomic(row), 0, -1);
}

bool ShmParser::AtomicWriteShmEnd(uint row) {
  return __sync_bool_compare_and_swap(HeaderAtomic(row), -1, 0);
}

void ShmParser::AtomicInitShm(uint row) {
  int *ptr = HeaderAtomic(row);
  if (*ptr != 0) {
    int t = *ptr;
    __sync_bool_compare_and_swap(ptr, t, 0);
  }
  ull *ptr1 = HeaderFilename(row);
  if (*ptr1 != 0) {
    ull t = *ptr1;
    __sync_bool_compare_and_swap(ptr1, t, 0);
  }
}

bool ShmParser::DTGAtomicReadShmBegin(uint row) {
  return __sync_bool_compare_and_swap(DTGHeaderAtomic(row), 0, 0);
}
bool ShmParser::DTGAtomicReadShmEnd(uint row) {
  return true;
}

bool ShmParser::DTGAtomicWriteShmBegin(uint row) {
  return __sync_bool_compare_and_swap(DTGHeaderAtomic(row), 0, -1);
}

bool ShmParser::DTGAtomicWriteShmEnd(uint row) {
  return __sync_bool_compare_and_swap(DTGHeaderAtomic(row), -1, 0);
}

void ShmParser::DTGAtomicInitShm(uint row) {
  int *ptr = DTGHeaderAtomic(row);
  if (*ptr != 0) {
    int t = *ptr;
    __sync_bool_compare_and_swap(ptr, t, 0);
  }
  ull *ptr1 = HeaderFilename(row);
  if (*ptr1 != 0) {
    ull t = *ptr1;
    __sync_bool_compare_and_swap(ptr1, t, 0);
  }
}

ShmHeader *ShmParser::GetShmHeader() {
  return (ShmHeader *)shm_ptr_;
}

int *ShmParser::HeaderAtomic(uint row) {
  return (int *)((char *)shm_ptr_ + sizeof(ShmHeader) + row * shm_param_.shm_row_chars);
}

int *ShmParser::DTGHeaderAtomic(uint row) {
  return (int *)((char *)shm_ptr_ + sizeof(ShmHeader) + row * shm_param_.shm_row_chars + 
                shm_param_.shm_row_chars * shm_param_.shm_rows);
}

ull *ShmParser::HeaderFilename(uint row) {
  return (ull *)((char *)shm_ptr_ + sizeof(ShmHeader) + shm_param_.shm_row_names_offset1 +
                 row * shm_param_.shm_row_chars);
}

ull *ShmParser::DTGHeaderFilename(uint row) {
  return (ull *)((char *)shm_ptr_ + sizeof(ShmHeader) + shm_param_.shm_row_names_offset1 +
                 row * shm_param_.shm_row_chars + shm_param_.shm_row_chars * shm_param_.shm_rows);
}

char *ShmParser::Img(uint row) {
  return (char *)((char *)shm_ptr_ + sizeof(ShmHeader) + shm_param_.shm_row_header_chars +
                  row * shm_param_.shm_row_chars);
}

char *ShmParser::DTGImgDist(uint row) {
  return (char *)(Img(row) + shm_param_.shm_row_chars * shm_param_.shm_rows);
}

char *ShmParser::DTGImgNearest(uint row) {
  return (char *)(Img(row) + shm_param_.shm_row_chars * shm_param_.shm_rows + shm_param_.shm_row_chars * shm_param_.dtg_shm_rows);
}

}  // namespace basemap_shm_util
