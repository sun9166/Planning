#ifndef CALMCAR_DDS_ANY_H
#define CALMCAR_DDS_ANY_H

#include <iostream>
#include <sstream>
#include <string>
#include <functional>
#include <typeindex>
#include <algorithm>
#include <cmath>

namespace calmcar {
namespace dds {
/* ================ Declaration ================ */
struct Any {
  enum Type {
    kInvalid = 0,
    kBool = 1,
    kChar = 2,
    kInt = 3,
    kUInt = 4,
    kDouble = 5,
    kString = 6,
    kCustom = 7
  };

  explicit Any(Type type = kInvalid);
  template <typename T>
  explicit Any(T t);
  Any(const Any& other);
  Any(Any&& other) noexcept;
  ~Any();

  Any& operator=(const Any& other);
  Any& operator=(Any&& other) noexcept;

  Type type() const noexcept;
  static std::string typeName(Type t);

  template <typename T>
  T value() const;
  template <typename T>
  T get() const;

  template <typename T>
  void setValue(T t);
  template <typename T>
  void set(T t);
  template <typename T>
  static Any fromValue(T t);

  template <typename T>
  friend T& operator<<(T& os, const Any& any);

 private:
  void reset(Type type);

  struct CustomData {
    template <typename T>
    explicit CustomData(const T& data);
    ~CustomData();

    CustomData(const CustomData&) = delete;
    CustomData& operator=(const CustomData&) = delete;

    void* data_ = nullptr;
    std::type_index type_;
    std::function<void(void)> destroyer_;
    std::function<CustomData*(void)> clone_;
  };
  union Data {
    Data() : ptr_(nullptr) {}
    ~Data() {}
    Data(const Data&) = default;
    Data& operator=(const Data&) = default;
    bool bool_;
    char char_;
    int64_t int_;
    uint64_t uint_;
    double double_;
    CustomData* ptr_;
  };
  Data data_ = {};
  Type type_ = kInvalid;
};

template <typename T>
T any_cast(const Any& any) {
  return any.value<T>();
}
/* ================ Declaration ================ */

/* ================ Definition ================ */
template <typename T>
Any::Any(T t) {
  setValue(std::move(t));
}

template <typename T>
T& operator<<(T& os, const Any& any) {
  switch (any.type()) {
    case Any::kInvalid:
      return os << "Invalid";
    case Any::kBool:
      return os << (any.data_.bool_ ? "true" : "false");
    case Any::kChar:
      return os << any.data_.char_;
    case Any::kInt:
      return os << any.data_.int_;
    case Any::kUInt:
      return os << any.data_.uint_;
    case Any::kDouble:
      return os << any.data_.double_;
    case Any::kString:
      return os << '"' << any_cast<std::string>(any) << '"';
    case Any::kCustom:
      return os << "Custom(" << any.data_.ptr_->data_ << ")";
  }
  return os;
}

template <>
inline std::string Any::value<std::string>() const {
  switch (type_) {
    case kInvalid:
    case kCustom:
      break;
    case kBool:
      return std::to_string(data_.bool_);
    case kChar:
      return std::to_string(data_.char_);
    case kInt:
      return std::to_string(data_.int_);
    case kUInt:
      return std::to_string(data_.uint_);
    case kDouble:
      return std::to_string(data_.double_);
    case kString:
      return *static_cast<std::string*>(data_.ptr_->data_);
  }
  return "";
}

template <>
inline bool Any::value<bool>() const {
  switch (type_) {
    case kInvalid:
      break;
    case kBool:
      return data_.bool_;
    case kChar:
      return data_.char_ != 0;
    case kInt:
      return data_.int_ != 0;
    case kUInt:
      return data_.uint_ != 0;
    case kDouble:
      return !(std::abs(data_.double_) <= 0.000000000001);
    case kString: {
      std::string str = value<std::string>();
      std::transform(str.begin(),str.end(),str.begin(), ::tolower);
      return ((str != "false") && (str != "no") && (str != "n") &&
              (str != "off") && (str != "0") && (str != "null") &&
              (str != "nil") && (str != "nan") && (str != "none"));
    };
    case kCustom:
      return true;
  }
  return false;
}

template <>
inline int64_t Any::value<int64_t>() const {
  switch (type_) {
    case kInvalid:
    case kCustom:
      break;
    case kBool:
      return data_.bool_;
    case kChar:
      return int64_t(uint64_t(uint8_t(data_.char_)));
    case kInt:
      return data_.int_;
    case kUInt:
      return data_.uint_;
    case kDouble:
      return int64_t(data_.double_);
    case kString: {
      std::stringstream ss(value<std::string>());
      int64_t ret;
      ss >> ret;
      return ret;
    };
  }
  return 0;
}

template <>
inline int32_t Any::value<int32_t>() const {
  return int32_t(value<int64_t>());
}

template <>
inline int16_t Any::value<int16_t>() const {
  return int16_t(value<int64_t>());
}

template <>
inline int8_t Any::value<int8_t>() const {
  return int8_t(value<int64_t>());
}

template <>
inline char Any::value<char>() const {
  return char(value<int8_t>());
}

template <>
inline uint64_t Any::value<uint64_t>() const {
  switch (type_) {
    case kInvalid:
      break;
    case kBool:
      return data_.bool_;
    case kChar:
      return uint8_t(data_.char_);
    case kInt:
      return data_.int_;
    case kUInt:
      return data_.uint_;
    case kDouble:
      return uint64_t(data_.double_);
    case kString: {
      std::stringstream ss(value<std::string>());
      uint64_t ret;
      ss >> ret;
      return ret;
    };
    default:
      break;
  }
  return 0;
}

template <>
inline uint32_t Any::value<uint32_t>() const {
  return uint32_t(value<uint64_t>());
}

template <>
inline uint16_t Any::value<uint16_t>() const {
  return uint16_t(value<uint64_t>());
}

template <>
inline uint8_t Any::value<uint8_t>() const {
  return uint8_t(value<uint64_t>());
}

template <>
inline double Any::value<double>() const {
  switch (type_) {
    case kInvalid:
    case kCustom:
      break;
    case kBool:
      return data_.bool_;
    case kChar:
      return uint8_t(data_.char_);
    case kInt:
      return double(data_.int_);
    case kUInt:
      return double(data_.uint_);
    case kDouble:
      return data_.double_;
    case kString: {
      std::stringstream ss(value<std::string>());
      double ret;
      ss >> ret;
      return ret;
    };
  }
  return NAN;
}

template <>
inline float Any::value<float>() const {
  return float(value<double>());
}

template <typename T>
inline T Any::value() const {
  return *static_cast<const T* const>(data_.ptr_->data_);
}

template <typename T>
inline T Any::get() const {
  return value<T>();
}

template <>
inline void Any::setValue<bool>(bool b) {
  reset(kBool);
  data_.bool_ = b;
}

template <>
inline void Any::setValue<char>(char c) {
  reset(kChar);
  data_.char_ = c;
}

template <>
inline void Any::setValue<int8_t>(int8_t i) {
  reset(kInt);
  data_.int_ = i;
}

template <>
inline void Any::setValue<int16_t>(int16_t i) {
  reset(kInt);
  data_.int_ = i;
}

template <>
inline void Any::setValue<int32_t>(int32_t i) {
  reset(kInt);
  data_.int_ = i;
}

template <>
inline void Any::setValue<int64_t>(int64_t i) {
  reset(kInt);
  data_.int_ = i;
}

template <>
inline void Any::setValue<uint8_t>(uint8_t u) {
  reset(kUInt);
  data_.uint_ = u;
}

template <>
inline void Any::setValue<uint16_t>(uint16_t u) {
  reset(kUInt);
  data_.uint_ = u;
}

template <>
inline void Any::setValue<uint32_t>(uint32_t u) {
  reset(kUInt);
  data_.uint_ = u;
}

template <>
inline void Any::setValue<uint64_t>(uint64_t u) {
  reset(kUInt);
  data_.uint_ = u;
}

template <>
inline void Any::setValue<float>(float f) {
  reset(kDouble);
  data_.double_ = f;
}

template <>
inline void Any::setValue<double>(double d) {
  reset(kDouble);
  data_.double_ = d;
}

template <>
inline void Any::setValue<std::string>(std::string s) {
  reset(kString);
  data_.ptr_ = new CustomData(std::move(s));
}

template <>
inline void Any::setValue<char*>(char* s) {
  setValue(std::string(s));
}

template <>
inline void Any::setValue<const char*>(const char* s) {
  setValue(std::string(s));
}

template <>
inline void Any::setValue<char* const>(char* const s) {
  setValue(std::string(s));
}

template <>
inline void Any::setValue<const char* const>(const char* const s) {
  setValue(std::string(s));
}

template <typename T>
inline void Any::setValue(T t) {
  reset(kCustom);
  data_.ptr_ = new CustomData(std::move(t));
}

template <typename T>
inline void Any::set(T value) {
  setValue<T>(std::move(value));
}

template <typename T>
inline Any Any::fromValue(T value) {
  Any any;
  any.set(std::move(value));
  return any;
}

template <typename T>
inline Any::CustomData::CustomData(const T& data)
    : data_(new T(data)),
      type_(typeid(T)),
      destroyer_([this] { delete static_cast<T*>(data_); }),
      clone_([this]() -> CustomData* {
        return new CustomData(*static_cast<T*>(data_));
      }) {}
template Any::CustomData::CustomData(const std::string&);

inline Any::CustomData::~CustomData() { destroyer_(); }
/* ================ Definition ================ */
}  // namespace utils
}  // namespace calmcar

#endif  // CALMCAR_UTILS_UTILITIES_ANY_HPP
