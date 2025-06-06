// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: gps.proto

#ifndef PROTOBUF_gps_2eproto__INCLUDED
#define PROTOBUF_gps_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3005000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3005001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)

namespace protobuf_gps_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[2];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
void InitDefaultsImuImpl();
void InitDefaultsImu();
void InitDefaultsGpsImpl();
void InitDefaultsGps();
inline void InitDefaults() {
  InitDefaultsImu();
  InitDefaultsGps();
}
}  // namespace protobuf_gps_2eproto
namespace gps {
namespace proto {
class Gps;
class GpsDefaultTypeInternal;
extern GpsDefaultTypeInternal _Gps_default_instance_;
class Imu;
class ImuDefaultTypeInternal;
extern ImuDefaultTypeInternal _Imu_default_instance_;
}  // namespace proto
}  // namespace gps
namespace gps {
namespace proto {

// ===================================================================

class Imu : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gps.proto.Imu) */ {
 public:
  Imu();
  virtual ~Imu();

  Imu(const Imu& from);

  inline Imu& operator=(const Imu& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Imu(Imu&& from) noexcept
    : Imu() {
    *this = ::std::move(from);
  }

  inline Imu& operator=(Imu&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const Imu& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Imu* internal_default_instance() {
    return reinterpret_cast<const Imu*>(
               &_Imu_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(Imu* other);
  friend void swap(Imu& a, Imu& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Imu* New() const PROTOBUF_FINAL { return New(NULL); }

  Imu* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const Imu& from);
  void MergeFrom(const Imu& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(Imu* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // double accelerometer_x = 1;
  void clear_accelerometer_x();
  static const int kAccelerometerXFieldNumber = 1;
  double accelerometer_x() const;
  void set_accelerometer_x(double value);

  // double accelerometer_y = 2;
  void clear_accelerometer_y();
  static const int kAccelerometerYFieldNumber = 2;
  double accelerometer_y() const;
  void set_accelerometer_y(double value);

  // double accelerometer_z = 3;
  void clear_accelerometer_z();
  static const int kAccelerometerZFieldNumber = 3;
  double accelerometer_z() const;
  void set_accelerometer_z(double value);

  // double gyroscope_x = 4;
  void clear_gyroscope_x();
  static const int kGyroscopeXFieldNumber = 4;
  double gyroscope_x() const;
  void set_gyroscope_x(double value);

  // double gyroscope_y = 5;
  void clear_gyroscope_y();
  static const int kGyroscopeYFieldNumber = 5;
  double gyroscope_y() const;
  void set_gyroscope_y(double value);

  // double gyroscope_z = 6;
  void clear_gyroscope_z();
  static const int kGyroscopeZFieldNumber = 6;
  double gyroscope_z() const;
  void set_gyroscope_z(double value);

  // uint64 timestamp = 7;
  void clear_timestamp();
  static const int kTimestampFieldNumber = 7;
  ::google::protobuf::uint64 timestamp() const;
  void set_timestamp(::google::protobuf::uint64 value);

  // @@protoc_insertion_point(class_scope:gps.proto.Imu)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  double accelerometer_x_;
  double accelerometer_y_;
  double accelerometer_z_;
  double gyroscope_x_;
  double gyroscope_y_;
  double gyroscope_z_;
  ::google::protobuf::uint64 timestamp_;
  mutable int _cached_size_;
  friend struct ::protobuf_gps_2eproto::TableStruct;
  friend void ::protobuf_gps_2eproto::InitDefaultsImuImpl();
};
// -------------------------------------------------------------------

class Gps : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:gps.proto.Gps) */ {
 public:
  Gps();
  virtual ~Gps();

  Gps(const Gps& from);

  inline Gps& operator=(const Gps& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Gps(Gps&& from) noexcept
    : Gps() {
    *this = ::std::move(from);
  }

  inline Gps& operator=(Gps&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const Gps& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Gps* internal_default_instance() {
    return reinterpret_cast<const Gps*>(
               &_Gps_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    1;

  void Swap(Gps* other);
  friend void swap(Gps& a, Gps& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Gps* New() const PROTOBUF_FINAL { return New(NULL); }

  Gps* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const Gps& from);
  void MergeFrom(const Gps& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(Gps* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // repeated .gps.proto.Imu imu = 18;
  int imu_size() const;
  void clear_imu();
  static const int kImuFieldNumber = 18;
  const ::gps::proto::Imu& imu(int index) const;
  ::gps::proto::Imu* mutable_imu(int index);
  ::gps::proto::Imu* add_imu();
  ::google::protobuf::RepeatedPtrField< ::gps::proto::Imu >*
      mutable_imu();
  const ::google::protobuf::RepeatedPtrField< ::gps::proto::Imu >&
      imu() const;

  // double latitude = 1;
  void clear_latitude();
  static const int kLatitudeFieldNumber = 1;
  double latitude() const;
  void set_latitude(double value);

  // double longitude = 2;
  void clear_longitude();
  static const int kLongitudeFieldNumber = 2;
  double longitude() const;
  void set_longitude(double value);

  // double height = 3;
  void clear_height();
  static const int kHeightFieldNumber = 3;
  double height() const;
  void set_height(double value);

  // int64 hour = 6;
  void clear_hour();
  static const int kHourFieldNumber = 6;
  ::google::protobuf::int64 hour() const;
  void set_hour(::google::protobuf::int64 value);

  // int64 minute = 7;
  void clear_minute();
  static const int kMinuteFieldNumber = 7;
  ::google::protobuf::int64 minute() const;
  void set_minute(::google::protobuf::int64 value);

  // int64 second = 8;
  void clear_second();
  static const int kSecondFieldNumber = 8;
  ::google::protobuf::int64 second() const;
  void set_second(::google::protobuf::int64 value);

  // int64 millisecond = 9;
  void clear_millisecond();
  static const int kMillisecondFieldNumber = 9;
  ::google::protobuf::int64 millisecond() const;
  void set_millisecond(::google::protobuf::int64 value);

  // int64 day = 10;
  void clear_day();
  static const int kDayFieldNumber = 10;
  ::google::protobuf::int64 day() const;
  void set_day(::google::protobuf::int64 value);

  // int64 month = 11;
  void clear_month();
  static const int kMonthFieldNumber = 11;
  ::google::protobuf::int64 month() const;
  void set_month(::google::protobuf::int64 value);

  // int64 year = 12;
  void clear_year();
  static const int kYearFieldNumber = 12;
  ::google::protobuf::int64 year() const;
  void set_year(::google::protobuf::int64 value);

  // double yaw = 19;
  void clear_yaw();
  static const int kYawFieldNumber = 19;
  double yaw() const;
  void set_yaw(double value);

  // double pitch = 20;
  void clear_pitch();
  static const int kPitchFieldNumber = 20;
  double pitch() const;
  void set_pitch(double value);

  // double roll = 21;
  void clear_roll();
  static const int kRollFieldNumber = 21;
  double roll() const;
  void set_roll(double value);

  // double vh = 22;
  void clear_vh();
  static const int kVhFieldNumber = 22;
  double vh() const;
  void set_vh(double value);

  // double vu = 23;
  void clear_vu();
  static const int kVuFieldNumber = 23;
  double vu() const;
  void set_vu(double value);

  // double vn = 24;
  void clear_vn();
  static const int kVnFieldNumber = 24;
  double vn() const;
  void set_vn(double value);

  // double ve = 25;
  void clear_ve();
  static const int kVeFieldNumber = 25;
  double ve() const;
  void set_ve(double value);

  // uint64 timestamp = 27;
  void clear_timestamp();
  static const int kTimestampFieldNumber = 27;
  ::google::protobuf::uint64 timestamp() const;
  void set_timestamp(::google::protobuf::uint64 value);

  // int32 status = 26;
  void clear_status();
  static const int kStatusFieldNumber = 26;
  ::google::protobuf::int32 status() const;
  void set_status(::google::protobuf::int32 value);

  // int32 time_week = 28;
  void clear_time_week();
  static const int kTimeWeekFieldNumber = 28;
  ::google::protobuf::int32 time_week() const;
  void set_time_week(::google::protobuf::int32 value);

  // double time_second = 29;
  void clear_time_second();
  static const int kTimeSecondFieldNumber = 29;
  double time_second() const;
  void set_time_second(double value);

  // @@protoc_insertion_point(class_scope:gps.proto.Gps)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::RepeatedPtrField< ::gps::proto::Imu > imu_;
  double latitude_;
  double longitude_;
  double height_;
  ::google::protobuf::int64 hour_;
  ::google::protobuf::int64 minute_;
  ::google::protobuf::int64 second_;
  ::google::protobuf::int64 millisecond_;
  ::google::protobuf::int64 day_;
  ::google::protobuf::int64 month_;
  ::google::protobuf::int64 year_;
  double yaw_;
  double pitch_;
  double roll_;
  double vh_;
  double vu_;
  double vn_;
  double ve_;
  ::google::protobuf::uint64 timestamp_;
  ::google::protobuf::int32 status_;
  ::google::protobuf::int32 time_week_;
  double time_second_;
  mutable int _cached_size_;
  friend struct ::protobuf_gps_2eproto::TableStruct;
  friend void ::protobuf_gps_2eproto::InitDefaultsGpsImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Imu

// double accelerometer_x = 1;
inline void Imu::clear_accelerometer_x() {
  accelerometer_x_ = 0;
}
inline double Imu::accelerometer_x() const {
  // @@protoc_insertion_point(field_get:gps.proto.Imu.accelerometer_x)
  return accelerometer_x_;
}
inline void Imu::set_accelerometer_x(double value) {
  
  accelerometer_x_ = value;
  // @@protoc_insertion_point(field_set:gps.proto.Imu.accelerometer_x)
}

// double accelerometer_y = 2;
inline void Imu::clear_accelerometer_y() {
  accelerometer_y_ = 0;
}
inline double Imu::accelerometer_y() const {
  // @@protoc_insertion_point(field_get:gps.proto.Imu.accelerometer_y)
  return accelerometer_y_;
}
inline void Imu::set_accelerometer_y(double value) {
  
  accelerometer_y_ = value;
  // @@protoc_insertion_point(field_set:gps.proto.Imu.accelerometer_y)
}

// double accelerometer_z = 3;
inline void Imu::clear_accelerometer_z() {
  accelerometer_z_ = 0;
}
inline double Imu::accelerometer_z() const {
  // @@protoc_insertion_point(field_get:gps.proto.Imu.accelerometer_z)
  return accelerometer_z_;
}
inline void Imu::set_accelerometer_z(double value) {
  
  accelerometer_z_ = value;
  // @@protoc_insertion_point(field_set:gps.proto.Imu.accelerometer_z)
}

// double gyroscope_x = 4;
inline void Imu::clear_gyroscope_x() {
  gyroscope_x_ = 0;
}
inline double Imu::gyroscope_x() const {
  // @@protoc_insertion_point(field_get:gps.proto.Imu.gyroscope_x)
  return gyroscope_x_;
}
inline void Imu::set_gyroscope_x(double value) {
  
  gyroscope_x_ = value;
  // @@protoc_insertion_point(field_set:gps.proto.Imu.gyroscope_x)
}

// double gyroscope_y = 5;
inline void Imu::clear_gyroscope_y() {
  gyroscope_y_ = 0;
}
inline double Imu::gyroscope_y() const {
  // @@protoc_insertion_point(field_get:gps.proto.Imu.gyroscope_y)
  return gyroscope_y_;
}
inline void Imu::set_gyroscope_y(double value) {
  
  gyroscope_y_ = value;
  // @@protoc_insertion_point(field_set:gps.proto.Imu.gyroscope_y)
}

// double gyroscope_z = 6;
inline void Imu::clear_gyroscope_z() {
  gyroscope_z_ = 0;
}
inline double Imu::gyroscope_z() const {
  // @@protoc_insertion_point(field_get:gps.proto.Imu.gyroscope_z)
  return gyroscope_z_;
}
inline void Imu::set_gyroscope_z(double value) {
  
  gyroscope_z_ = value;
  // @@protoc_insertion_point(field_set:gps.proto.Imu.gyroscope_z)
}

// uint64 timestamp = 7;
inline void Imu::clear_timestamp() {
  timestamp_ = GOOGLE_ULONGLONG(0);
}
inline ::google::protobuf::uint64 Imu::timestamp() const {
  // @@protoc_insertion_point(field_get:gps.proto.Imu.timestamp)
  return timestamp_;
}
inline void Imu::set_timestamp(::google::protobuf::uint64 value) {
  
  timestamp_ = value;
  // @@protoc_insertion_point(field_set:gps.proto.Imu.timestamp)
}

// -------------------------------------------------------------------

// Gps

// double latitude = 1;
inline void Gps::clear_latitude() {
  latitude_ = 0;
}
inline double Gps::latitude() const {
  // @@protoc_insertion_point(field_get:gps.proto.Gps.latitude)
  return latitude_;
}
inline void Gps::set_latitude(double value) {
  
  latitude_ = value;
  // @@protoc_insertion_point(field_set:gps.proto.Gps.latitude)
}

// double longitude = 2;
inline void Gps::clear_longitude() {
  longitude_ = 0;
}
inline double Gps::longitude() const {
  // @@protoc_insertion_point(field_get:gps.proto.Gps.longitude)
  return longitude_;
}
inline void Gps::set_longitude(double value) {
  
  longitude_ = value;
  // @@protoc_insertion_point(field_set:gps.proto.Gps.longitude)
}

// double height = 3;
inline void Gps::clear_height() {
  height_ = 0;
}
inline double Gps::height() const {
  // @@protoc_insertion_point(field_get:gps.proto.Gps.height)
  return height_;
}
inline void Gps::set_height(double value) {
  
  height_ = value;
  // @@protoc_insertion_point(field_set:gps.proto.Gps.height)
}

// int64 hour = 6;
inline void Gps::clear_hour() {
  hour_ = GOOGLE_LONGLONG(0);
}
inline ::google::protobuf::int64 Gps::hour() const {
  // @@protoc_insertion_point(field_get:gps.proto.Gps.hour)
  return hour_;
}
inline void Gps::set_hour(::google::protobuf::int64 value) {
  
  hour_ = value;
  // @@protoc_insertion_point(field_set:gps.proto.Gps.hour)
}

// int64 minute = 7;
inline void Gps::clear_minute() {
  minute_ = GOOGLE_LONGLONG(0);
}
inline ::google::protobuf::int64 Gps::minute() const {
  // @@protoc_insertion_point(field_get:gps.proto.Gps.minute)
  return minute_;
}
inline void Gps::set_minute(::google::protobuf::int64 value) {
  
  minute_ = value;
  // @@protoc_insertion_point(field_set:gps.proto.Gps.minute)
}

// int64 second = 8;
inline void Gps::clear_second() {
  second_ = GOOGLE_LONGLONG(0);
}
inline ::google::protobuf::int64 Gps::second() const {
  // @@protoc_insertion_point(field_get:gps.proto.Gps.second)
  return second_;
}
inline void Gps::set_second(::google::protobuf::int64 value) {
  
  second_ = value;
  // @@protoc_insertion_point(field_set:gps.proto.Gps.second)
}

// int64 millisecond = 9;
inline void Gps::clear_millisecond() {
  millisecond_ = GOOGLE_LONGLONG(0);
}
inline ::google::protobuf::int64 Gps::millisecond() const {
  // @@protoc_insertion_point(field_get:gps.proto.Gps.millisecond)
  return millisecond_;
}
inline void Gps::set_millisecond(::google::protobuf::int64 value) {
  
  millisecond_ = value;
  // @@protoc_insertion_point(field_set:gps.proto.Gps.millisecond)
}

// int64 day = 10;
inline void Gps::clear_day() {
  day_ = GOOGLE_LONGLONG(0);
}
inline ::google::protobuf::int64 Gps::day() const {
  // @@protoc_insertion_point(field_get:gps.proto.Gps.day)
  return day_;
}
inline void Gps::set_day(::google::protobuf::int64 value) {
  
  day_ = value;
  // @@protoc_insertion_point(field_set:gps.proto.Gps.day)
}

// int64 month = 11;
inline void Gps::clear_month() {
  month_ = GOOGLE_LONGLONG(0);
}
inline ::google::protobuf::int64 Gps::month() const {
  // @@protoc_insertion_point(field_get:gps.proto.Gps.month)
  return month_;
}
inline void Gps::set_month(::google::protobuf::int64 value) {
  
  month_ = value;
  // @@protoc_insertion_point(field_set:gps.proto.Gps.month)
}

// int64 year = 12;
inline void Gps::clear_year() {
  year_ = GOOGLE_LONGLONG(0);
}
inline ::google::protobuf::int64 Gps::year() const {
  // @@protoc_insertion_point(field_get:gps.proto.Gps.year)
  return year_;
}
inline void Gps::set_year(::google::protobuf::int64 value) {
  
  year_ = value;
  // @@protoc_insertion_point(field_set:gps.proto.Gps.year)
}

// repeated .gps.proto.Imu imu = 18;
inline int Gps::imu_size() const {
  return imu_.size();
}
inline void Gps::clear_imu() {
  imu_.Clear();
}
inline const ::gps::proto::Imu& Gps::imu(int index) const {
  // @@protoc_insertion_point(field_get:gps.proto.Gps.imu)
  return imu_.Get(index);
}
inline ::gps::proto::Imu* Gps::mutable_imu(int index) {
  // @@protoc_insertion_point(field_mutable:gps.proto.Gps.imu)
  return imu_.Mutable(index);
}
inline ::gps::proto::Imu* Gps::add_imu() {
  // @@protoc_insertion_point(field_add:gps.proto.Gps.imu)
  return imu_.Add();
}
inline ::google::protobuf::RepeatedPtrField< ::gps::proto::Imu >*
Gps::mutable_imu() {
  // @@protoc_insertion_point(field_mutable_list:gps.proto.Gps.imu)
  return &imu_;
}
inline const ::google::protobuf::RepeatedPtrField< ::gps::proto::Imu >&
Gps::imu() const {
  // @@protoc_insertion_point(field_list:gps.proto.Gps.imu)
  return imu_;
}

// double yaw = 19;
inline void Gps::clear_yaw() {
  yaw_ = 0;
}
inline double Gps::yaw() const {
  // @@protoc_insertion_point(field_get:gps.proto.Gps.yaw)
  return yaw_;
}
inline void Gps::set_yaw(double value) {
  
  yaw_ = value;
  // @@protoc_insertion_point(field_set:gps.proto.Gps.yaw)
}

// double pitch = 20;
inline void Gps::clear_pitch() {
  pitch_ = 0;
}
inline double Gps::pitch() const {
  // @@protoc_insertion_point(field_get:gps.proto.Gps.pitch)
  return pitch_;
}
inline void Gps::set_pitch(double value) {
  
  pitch_ = value;
  // @@protoc_insertion_point(field_set:gps.proto.Gps.pitch)
}

// double roll = 21;
inline void Gps::clear_roll() {
  roll_ = 0;
}
inline double Gps::roll() const {
  // @@protoc_insertion_point(field_get:gps.proto.Gps.roll)
  return roll_;
}
inline void Gps::set_roll(double value) {
  
  roll_ = value;
  // @@protoc_insertion_point(field_set:gps.proto.Gps.roll)
}

// double vh = 22;
inline void Gps::clear_vh() {
  vh_ = 0;
}
inline double Gps::vh() const {
  // @@protoc_insertion_point(field_get:gps.proto.Gps.vh)
  return vh_;
}
inline void Gps::set_vh(double value) {
  
  vh_ = value;
  // @@protoc_insertion_point(field_set:gps.proto.Gps.vh)
}

// double vu = 23;
inline void Gps::clear_vu() {
  vu_ = 0;
}
inline double Gps::vu() const {
  // @@protoc_insertion_point(field_get:gps.proto.Gps.vu)
  return vu_;
}
inline void Gps::set_vu(double value) {
  
  vu_ = value;
  // @@protoc_insertion_point(field_set:gps.proto.Gps.vu)
}

// double vn = 24;
inline void Gps::clear_vn() {
  vn_ = 0;
}
inline double Gps::vn() const {
  // @@protoc_insertion_point(field_get:gps.proto.Gps.vn)
  return vn_;
}
inline void Gps::set_vn(double value) {
  
  vn_ = value;
  // @@protoc_insertion_point(field_set:gps.proto.Gps.vn)
}

// double ve = 25;
inline void Gps::clear_ve() {
  ve_ = 0;
}
inline double Gps::ve() const {
  // @@protoc_insertion_point(field_get:gps.proto.Gps.ve)
  return ve_;
}
inline void Gps::set_ve(double value) {
  
  ve_ = value;
  // @@protoc_insertion_point(field_set:gps.proto.Gps.ve)
}

// int32 status = 26;
inline void Gps::clear_status() {
  status_ = 0;
}
inline ::google::protobuf::int32 Gps::status() const {
  // @@protoc_insertion_point(field_get:gps.proto.Gps.status)
  return status_;
}
inline void Gps::set_status(::google::protobuf::int32 value) {
  
  status_ = value;
  // @@protoc_insertion_point(field_set:gps.proto.Gps.status)
}

// uint64 timestamp = 27;
inline void Gps::clear_timestamp() {
  timestamp_ = GOOGLE_ULONGLONG(0);
}
inline ::google::protobuf::uint64 Gps::timestamp() const {
  // @@protoc_insertion_point(field_get:gps.proto.Gps.timestamp)
  return timestamp_;
}
inline void Gps::set_timestamp(::google::protobuf::uint64 value) {
  
  timestamp_ = value;
  // @@protoc_insertion_point(field_set:gps.proto.Gps.timestamp)
}

// int32 time_week = 28;
inline void Gps::clear_time_week() {
  time_week_ = 0;
}
inline ::google::protobuf::int32 Gps::time_week() const {
  // @@protoc_insertion_point(field_get:gps.proto.Gps.time_week)
  return time_week_;
}
inline void Gps::set_time_week(::google::protobuf::int32 value) {
  
  time_week_ = value;
  // @@protoc_insertion_point(field_set:gps.proto.Gps.time_week)
}

// double time_second = 29;
inline void Gps::clear_time_second() {
  time_second_ = 0;
}
inline double Gps::time_second() const {
  // @@protoc_insertion_point(field_get:gps.proto.Gps.time_second)
  return time_second_;
}
inline void Gps::set_time_second(double value) {
  
  time_second_ = value;
  // @@protoc_insertion_point(field_set:gps.proto.Gps.time_second)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace proto
}  // namespace gps

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_gps_2eproto__INCLUDED
