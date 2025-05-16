/******************************************************************************
* Copyright (C) 2015-2020, idriverplus(BeiJing ZhiXingZhe, Inc.)
*
* NodeName: vectormap
* FileName: map/vectormap/src/vectormap/alog.h
*
* Description: ACU use roslog system, apollo use glog, so ...... creat this
file

*
* History:
* lbh         2018/05/11    1.0.0    build this module.
******************************************************************************/

#ifndef ACU_CHECK_H_
#define ACU_CHECK_H_
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>


#define ACU_RETURN_IF_ERROR(con, str)                                              \
  if (con) {                                                                   \
    std::cout<<__FILE__ << "/" << __FUNCTION__ << ":" << __LINE__ << ":" \
                              << #str<<std::endl;                                        \
    return -1;                                                                 \
  }

#define ACU_RETURN_IF_NULL(ptr)                                              \
  if (ptr == nullptr) {                                                       \
    std::cout<<__FILE__ << "/" << __FUNCTION__ << ":" << __LINE__ << ":" \
                             << #ptr << " is nullptr,"                        \
                             << "shutdown()"<<std::endl;                           \
    return -1;                                                          \
  }

#define ACU_RETURN_VAL_IF_NULL(ptr, val)                                     \
  if (ptr == nullptr) {                                                       \
    std::cout<<__FILE__ << "/" << __FUNCTION__ << ":" << __LINE__ << ":" \
                             << #ptr << " is nullptr."<<std::endl;                      \
    return val;                                                               \
  }

#define ACU_RETURN_IF(condition)                                             \
  if (condition) {                                                            \
    std::cout<<__FILE__ << "/" << __FUNCTION__ << ":" << __LINE__ << ":" \
                             << #condition << " is not met,"                  \
                             << " shutdown()"<<std::endl;                           \
    return -1;                                                          \
  }

#define ACU_RETURN_VAL_IF(condition, val)                                    \
  if (condition) {                                                            \
    std::cout<<__FILE__ << "/" << __FUNCTION__ << ":" << __LINE__ << ":" \
                             << #condition << " is not met."<<std::endl;                 \
    return val;                                                               \
  }

#define ACU_CHECK(val)                                                       \
  if (!(val)) {                                                               \
    std::cout<<__FILE__ << "/" << __FUNCTION__ << ":" << __LINE__ << ":" \
                             << #val << " is false,"                          \
                             << " shutdown()"<<std::endl;                          \
    return -1;                                                          \
  }
#define ACU_CHECK_GT(val1, val2)                                             \
  if ((val1) <= (val2)) {                                                     \
    std::cout<<__FILE__ << "/" << __FUNCTION__ << ":" << __LINE__ << ":" \
                             << #val1 << " <= " << #val2                      \
                             << " shutdown()"<<std::endl;                           \
    return -1;                                                          \
  }
#define ACU_CHECK_GE(val1, val2)                                             \
  if ((val1) < (val2)) {                                                      \
    std::cout<<__FILE__ << "/" << __FUNCTION__ << ":" << __LINE__ << ":" \
                             << #val1 << " < " << #val2                       \
                             << " shutdown()"<<std::endl;                           \
    return -1;                                                          \
  }
#define ACU_CHECK_LT(val1, val2)                                             \
  if ((val1) >= (val2)) {                                                     \
    std::cout<<__FILE__ << "/" << __FUNCTION__ << ":" << __LINE__ << ":" \
                             << #val1 << " >= " << #val2                      \
                             << " shutdown()"<<std::endl;                           \
    return -1;                                                          \
  }
#define ACU_CHECK_LE(val1, val2)                                             \
  if ((val1) > (val2)) {                                                      \
    std::cout<<__FILE__ << "/" << __FUNCTION__ << ":" << __LINE__ << ":" \
                             << #val1 << " > " << #val2                       \
                             << " shutdown()"<<std::endl;                          \
    return -1;                                                          \
  }
#define ACU_CHECK_NE(val1, val2)                                             \
  if ((val1) == (val2)) {                                                     \
    std::cout<<__FILE__ << "/" << __FUNCTION__ << ":" << __LINE__ << ":" \
                             << #val1 << " == " << #val2                      \
                             << " shutdown()"<<std::endl;                           \
    return -1;                                                          \
  }
#define ACU_CHECK_EQ(val1, val2)                                             \
  if ((val1) != (val2)) {                                                     \
    std::cout<<__FILE__ << "/" << __FUNCTION__ << ":" << __LINE__ << ":" \
                             << #val1 << " != " << #val2                      \
                             << " shutdown()"<<std::endl;                           \
    return -1;                                                          \
  }
#endif
