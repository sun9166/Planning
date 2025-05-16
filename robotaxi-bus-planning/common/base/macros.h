/******************************************************************************
 * Copyright (C) 2018-2023, idriverplus(BeiJing ZhiXingZhe, Inc.)
 *
 * History:
 * lbh          2019/03/28    1.0.0        refer to apollo3.5
 *****************************************************************************/

#ifndef ACU_COMMON_MACROS_H_
#define ACU_COMMON_MACROS_H_

#include <iostream>
#include <memory>
#include <mutex>

#define BASE_DECLARE_SINGLETON(classname)                                      \
 public:                                                                  \
  static classname *Instance() {                                          \
    static classname *instance = nullptr;                                 \
    if (!instance) {                                                      \
      static std::once_flag flag;                                         \
      std::call_once(flag,                                                \
                     [&] { instance = new (std::nothrow) classname(); }); \
    }                                                                     \
    return instance;                                                      \
  }                                                                       \
                                                                          \
 private:                                                                 \
  classname(const classname &) = delete;                                  \
  classname &operator=(const classname &) = delete;


#define DISALLOW_COPY_AND_ASSIGN(classname) \
 private:                                   \
  classname(const classname &);             \
  classname &operator=(const classname &);


/// Defines a "has_member_member_name" class template
///
/// This template can be used to check if its "T" argument
/// has a data or function member called "member_name"
#define DEFINE_has_member(member_name)                                         \
    template <typename T>                                                      \
    class temp_has_member_##member_name                                        \
    {                                                                          \
        typedef char yes_type;                                                 \
        typedef long no_type;                                                  \
        template <typename U> static yes_type test(decltype(&U::member_name)); \
        template <typename U> static no_type  test(...);                       \
    public:                                                                    \
        static constexpr bool value = sizeof(test<T>(0)) == sizeof(yes_type);  \
    }

/// Shorthand for testing if "class_" has a member called "member_name"
///
/// @note "DEFINE_has_member(member_name)" must be used
///       before calling "has_member(class_, member_name)"
#define temp_has_member(class_, member_name)  temp_has_member_##member_name<class_>::value

#endif  // ACU_COMMON_MACROS_H_
