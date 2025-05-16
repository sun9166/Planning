/******************************************************************************
 * Copyright (C) 2018-2023, idriverplus(BeiJing ZhiXingZhe, Inc.)
 *
 * NodeName: ivplanner
 * FileName: macro.h
 *
 * Description: macros this node need
 *
 * History:
 * Feng younan          2018/08/17    1.5.1        build this model.
 ******************************************************************************/
#ifndef __IVPLANNER_MACRO_H__
#define __IVPLANNER_MACRO_H__

// FOR SINGLETON
#ifndef DISALLOW_COPY_AND_ASSIGN
#define DISALLOW_COPY_AND_ASSIGN(classname) \
 private:                                   \
  classname(const classname &);             \
  classname &operator=(const classname &);
#endif


#ifndef DISALLOW_IMPLICIT_CONSTRUCTORS
#define DISALLOW_IMPLICIT_CONSTRUCTORS(classname) \
 private:                                         \
  classname();                                   \
  DISALLOW_COPY_AND_ASSIGN(classname);

#endif

#ifndef DECLARE_SINGLETON
#define DECLARE_SINGLETON(classname)        \
 public:                                    \
  static classname *instance() {            \
    static classname instance;              \
    return &instance;                       \
  }                                         \
  DISALLOW_IMPLICIT_CONSTRUCTORS(classname) \
 private:
#endif


#endif  // __IVPLANNER_MACRO_H__
