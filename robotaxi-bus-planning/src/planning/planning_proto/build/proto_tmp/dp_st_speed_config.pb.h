// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: dp_st_speed_config.proto

#ifndef PROTOBUF_dp_5fst_5fspeed_5fconfig_2eproto__INCLUDED
#define PROTOBUF_dp_5fst_5fspeed_5fconfig_2eproto__INCLUDED

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
#include "st_boundary_config.pb.h"
// @@protoc_insertion_point(includes)

namespace protobuf_dp_5fst_5fspeed_5fconfig_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[1];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
void InitDefaultsDpStSpeedConfigImpl();
void InitDefaultsDpStSpeedConfig();
inline void InitDefaults() {
  InitDefaultsDpStSpeedConfig();
}
}  // namespace protobuf_dp_5fst_5fspeed_5fconfig_2eproto
namespace acu {
namespace planning {
class DpStSpeedConfig;
class DpStSpeedConfigDefaultTypeInternal;
extern DpStSpeedConfigDefaultTypeInternal _DpStSpeedConfig_default_instance_;
}  // namespace planning
}  // namespace acu
namespace acu {
namespace planning {

// ===================================================================

class DpStSpeedConfig : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:acu.planning.DpStSpeedConfig) */ {
 public:
  DpStSpeedConfig();
  virtual ~DpStSpeedConfig();

  DpStSpeedConfig(const DpStSpeedConfig& from);

  inline DpStSpeedConfig& operator=(const DpStSpeedConfig& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  DpStSpeedConfig(DpStSpeedConfig&& from) noexcept
    : DpStSpeedConfig() {
    *this = ::std::move(from);
  }

  inline DpStSpeedConfig& operator=(DpStSpeedConfig&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _internal_metadata_.unknown_fields();
  }
  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return _internal_metadata_.mutable_unknown_fields();
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const DpStSpeedConfig& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const DpStSpeedConfig* internal_default_instance() {
    return reinterpret_cast<const DpStSpeedConfig*>(
               &_DpStSpeedConfig_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(DpStSpeedConfig* other);
  friend void swap(DpStSpeedConfig& a, DpStSpeedConfig& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline DpStSpeedConfig* New() const PROTOBUF_FINAL { return New(NULL); }

  DpStSpeedConfig* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const DpStSpeedConfig& from);
  void MergeFrom(const DpStSpeedConfig& from);
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
  void InternalSwap(DpStSpeedConfig* other);
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

  // optional .acu.planning.StBoundaryConfig st_boundary_config = 50;
  bool has_st_boundary_config() const;
  void clear_st_boundary_config();
  static const int kStBoundaryConfigFieldNumber = 50;
  const ::acu::planning::StBoundaryConfig& st_boundary_config() const;
  ::acu::planning::StBoundaryConfig* release_st_boundary_config();
  ::acu::planning::StBoundaryConfig* mutable_st_boundary_config();
  void set_allocated_st_boundary_config(::acu::planning::StBoundaryConfig* st_boundary_config);

  // optional double speed_weight = 5 [default = 0];
  bool has_speed_weight() const;
  void clear_speed_weight();
  static const int kSpeedWeightFieldNumber = 5;
  double speed_weight() const;
  void set_speed_weight(double value);

  // optional double reference_weight = 9 [default = 0];
  bool has_reference_weight() const;
  void clear_reference_weight();
  static const int kReferenceWeightFieldNumber = 9;
  double reference_weight() const;
  void set_reference_weight(double value);

  // optional double total_path_length = 1 [default = 0.1];
  bool has_total_path_length() const;
  void clear_total_path_length();
  static const int kTotalPathLengthFieldNumber = 1;
  double total_path_length() const;
  void set_total_path_length(double value);

  // optional double total_time = 2 [default = 3];
  bool has_total_time() const;
  void clear_total_time();
  static const int kTotalTimeFieldNumber = 2;
  double total_time() const;
  void set_total_time(double value);

  // optional int32 matrix_dimension_s = 3 [default = 100];
  bool has_matrix_dimension_s() const;
  void clear_matrix_dimension_s();
  static const int kMatrixDimensionSFieldNumber = 3;
  ::google::protobuf::int32 matrix_dimension_s() const;
  void set_matrix_dimension_s(::google::protobuf::int32 value);

  // optional int32 matrix_dimension_t = 4 [default = 10];
  bool has_matrix_dimension_t() const;
  void clear_matrix_dimension_t();
  static const int kMatrixDimensionTFieldNumber = 4;
  ::google::protobuf::int32 matrix_dimension_t() const;
  void set_matrix_dimension_t(::google::protobuf::int32 value);

  // optional double accel_weight = 6 [default = 10];
  bool has_accel_weight() const;
  void clear_accel_weight();
  static const int kAccelWeightFieldNumber = 6;
  double accel_weight() const;
  void set_accel_weight(double value);

  // optional double jerk_weight = 7 [default = 10];
  bool has_jerk_weight() const;
  void clear_jerk_weight();
  static const int kJerkWeightFieldNumber = 7;
  double jerk_weight() const;
  void set_jerk_weight(double value);

  // optional double obstacle_weight = 8 [default = 1];
  bool has_obstacle_weight() const;
  void clear_obstacle_weight();
  static const int kObstacleWeightFieldNumber = 8;
  double obstacle_weight() const;
  void set_obstacle_weight(double value);

  // optional double go_down_buffer = 10 [default = 5];
  bool has_go_down_buffer() const;
  void clear_go_down_buffer();
  static const int kGoDownBufferFieldNumber = 10;
  double go_down_buffer() const;
  void set_go_down_buffer(double value);

  // optional double go_up_buffer = 11 [default = 5];
  bool has_go_up_buffer() const;
  void clear_go_up_buffer();
  static const int kGoUpBufferFieldNumber = 11;
  double go_up_buffer() const;
  void set_go_up_buffer(double value);

  // optional double default_obstacle_cost = 12 [default = 10000000000];
  bool has_default_obstacle_cost() const;
  void clear_default_obstacle_cost();
  static const int kDefaultObstacleCostFieldNumber = 12;
  double default_obstacle_cost() const;
  void set_default_obstacle_cost(double value);

  // optional double default_speed_cost = 13 [default = 1];
  bool has_default_speed_cost() const;
  void clear_default_speed_cost();
  static const int kDefaultSpeedCostFieldNumber = 13;
  double default_speed_cost() const;
  void set_default_speed_cost(double value);

  // optional double exceed_speed_penalty = 14 [default = 10];
  bool has_exceed_speed_penalty() const;
  void clear_exceed_speed_penalty();
  static const int kExceedSpeedPenaltyFieldNumber = 14;
  double exceed_speed_penalty() const;
  void set_exceed_speed_penalty(double value);

  // optional double low_speed_penalty = 15 [default = 2.5];
  bool has_low_speed_penalty() const;
  void clear_low_speed_penalty();
  static const int kLowSpeedPenaltyFieldNumber = 15;
  double low_speed_penalty() const;
  void set_low_speed_penalty(double value);

  // optional double keep_clear_low_speed_penalty = 16 [default = 10];
  bool has_keep_clear_low_speed_penalty() const;
  void clear_keep_clear_low_speed_penalty();
  static const int kKeepClearLowSpeedPenaltyFieldNumber = 16;
  double keep_clear_low_speed_penalty() const;
  void set_keep_clear_low_speed_penalty(double value);

  // optional double accel_penalty = 20 [default = 2];
  bool has_accel_penalty() const;
  void clear_accel_penalty();
  static const int kAccelPenaltyFieldNumber = 20;
  double accel_penalty() const;
  void set_accel_penalty(double value);

  // optional double decel_penalty = 21 [default = 2];
  bool has_decel_penalty() const;
  void clear_decel_penalty();
  static const int kDecelPenaltyFieldNumber = 21;
  double decel_penalty() const;
  void set_decel_penalty(double value);

  // optional double positive_jerk_coeff = 30 [default = 1];
  bool has_positive_jerk_coeff() const;
  void clear_positive_jerk_coeff();
  static const int kPositiveJerkCoeffFieldNumber = 30;
  double positive_jerk_coeff() const;
  void set_positive_jerk_coeff(double value);

  // optional double negative_jerk_coeff = 31 [default = 300];
  bool has_negative_jerk_coeff() const;
  void clear_negative_jerk_coeff();
  static const int kNegativeJerkCoeffFieldNumber = 31;
  double negative_jerk_coeff() const;
  void set_negative_jerk_coeff(double value);

  // optional double max_acceleration = 40 [default = 4.5];
  bool has_max_acceleration() const;
  void clear_max_acceleration();
  static const int kMaxAccelerationFieldNumber = 40;
  double max_acceleration() const;
  void set_max_acceleration(double value);

  // optional double max_deceleration = 41 [default = -4.5];
  bool has_max_deceleration() const;
  void clear_max_deceleration();
  static const int kMaxDecelerationFieldNumber = 41;
  double max_deceleration() const;
  void set_max_deceleration(double value);

  // @@protoc_insertion_point(class_scope:acu.planning.DpStSpeedConfig)
 private:
  void set_has_total_path_length();
  void clear_has_total_path_length();
  void set_has_total_time();
  void clear_has_total_time();
  void set_has_matrix_dimension_s();
  void clear_has_matrix_dimension_s();
  void set_has_matrix_dimension_t();
  void clear_has_matrix_dimension_t();
  void set_has_speed_weight();
  void clear_has_speed_weight();
  void set_has_accel_weight();
  void clear_has_accel_weight();
  void set_has_jerk_weight();
  void clear_has_jerk_weight();
  void set_has_obstacle_weight();
  void clear_has_obstacle_weight();
  void set_has_reference_weight();
  void clear_has_reference_weight();
  void set_has_go_down_buffer();
  void clear_has_go_down_buffer();
  void set_has_go_up_buffer();
  void clear_has_go_up_buffer();
  void set_has_default_obstacle_cost();
  void clear_has_default_obstacle_cost();
  void set_has_default_speed_cost();
  void clear_has_default_speed_cost();
  void set_has_exceed_speed_penalty();
  void clear_has_exceed_speed_penalty();
  void set_has_low_speed_penalty();
  void clear_has_low_speed_penalty();
  void set_has_keep_clear_low_speed_penalty();
  void clear_has_keep_clear_low_speed_penalty();
  void set_has_accel_penalty();
  void clear_has_accel_penalty();
  void set_has_decel_penalty();
  void clear_has_decel_penalty();
  void set_has_positive_jerk_coeff();
  void clear_has_positive_jerk_coeff();
  void set_has_negative_jerk_coeff();
  void clear_has_negative_jerk_coeff();
  void set_has_max_acceleration();
  void clear_has_max_acceleration();
  void set_has_max_deceleration();
  void clear_has_max_deceleration();
  void set_has_st_boundary_config();
  void clear_has_st_boundary_config();

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::internal::HasBits<1> _has_bits_;
  mutable int _cached_size_;
  ::acu::planning::StBoundaryConfig* st_boundary_config_;
  double speed_weight_;
  double reference_weight_;
  double total_path_length_;
  double total_time_;
  ::google::protobuf::int32 matrix_dimension_s_;
  ::google::protobuf::int32 matrix_dimension_t_;
  double accel_weight_;
  double jerk_weight_;
  double obstacle_weight_;
  double go_down_buffer_;
  double go_up_buffer_;
  double default_obstacle_cost_;
  double default_speed_cost_;
  double exceed_speed_penalty_;
  double low_speed_penalty_;
  double keep_clear_low_speed_penalty_;
  double accel_penalty_;
  double decel_penalty_;
  double positive_jerk_coeff_;
  double negative_jerk_coeff_;
  double max_acceleration_;
  double max_deceleration_;
  friend struct ::protobuf_dp_5fst_5fspeed_5fconfig_2eproto::TableStruct;
  friend void ::protobuf_dp_5fst_5fspeed_5fconfig_2eproto::InitDefaultsDpStSpeedConfigImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// DpStSpeedConfig

// optional double total_path_length = 1 [default = 0.1];
inline bool DpStSpeedConfig::has_total_path_length() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void DpStSpeedConfig::set_has_total_path_length() {
  _has_bits_[0] |= 0x00000008u;
}
inline void DpStSpeedConfig::clear_has_total_path_length() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void DpStSpeedConfig::clear_total_path_length() {
  total_path_length_ = 0.1;
  clear_has_total_path_length();
}
inline double DpStSpeedConfig::total_path_length() const {
  // @@protoc_insertion_point(field_get:acu.planning.DpStSpeedConfig.total_path_length)
  return total_path_length_;
}
inline void DpStSpeedConfig::set_total_path_length(double value) {
  set_has_total_path_length();
  total_path_length_ = value;
  // @@protoc_insertion_point(field_set:acu.planning.DpStSpeedConfig.total_path_length)
}

// optional double total_time = 2 [default = 3];
inline bool DpStSpeedConfig::has_total_time() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void DpStSpeedConfig::set_has_total_time() {
  _has_bits_[0] |= 0x00000010u;
}
inline void DpStSpeedConfig::clear_has_total_time() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void DpStSpeedConfig::clear_total_time() {
  total_time_ = 3;
  clear_has_total_time();
}
inline double DpStSpeedConfig::total_time() const {
  // @@protoc_insertion_point(field_get:acu.planning.DpStSpeedConfig.total_time)
  return total_time_;
}
inline void DpStSpeedConfig::set_total_time(double value) {
  set_has_total_time();
  total_time_ = value;
  // @@protoc_insertion_point(field_set:acu.planning.DpStSpeedConfig.total_time)
}

// optional int32 matrix_dimension_s = 3 [default = 100];
inline bool DpStSpeedConfig::has_matrix_dimension_s() const {
  return (_has_bits_[0] & 0x00000020u) != 0;
}
inline void DpStSpeedConfig::set_has_matrix_dimension_s() {
  _has_bits_[0] |= 0x00000020u;
}
inline void DpStSpeedConfig::clear_has_matrix_dimension_s() {
  _has_bits_[0] &= ~0x00000020u;
}
inline void DpStSpeedConfig::clear_matrix_dimension_s() {
  matrix_dimension_s_ = 100;
  clear_has_matrix_dimension_s();
}
inline ::google::protobuf::int32 DpStSpeedConfig::matrix_dimension_s() const {
  // @@protoc_insertion_point(field_get:acu.planning.DpStSpeedConfig.matrix_dimension_s)
  return matrix_dimension_s_;
}
inline void DpStSpeedConfig::set_matrix_dimension_s(::google::protobuf::int32 value) {
  set_has_matrix_dimension_s();
  matrix_dimension_s_ = value;
  // @@protoc_insertion_point(field_set:acu.planning.DpStSpeedConfig.matrix_dimension_s)
}

// optional int32 matrix_dimension_t = 4 [default = 10];
inline bool DpStSpeedConfig::has_matrix_dimension_t() const {
  return (_has_bits_[0] & 0x00000040u) != 0;
}
inline void DpStSpeedConfig::set_has_matrix_dimension_t() {
  _has_bits_[0] |= 0x00000040u;
}
inline void DpStSpeedConfig::clear_has_matrix_dimension_t() {
  _has_bits_[0] &= ~0x00000040u;
}
inline void DpStSpeedConfig::clear_matrix_dimension_t() {
  matrix_dimension_t_ = 10;
  clear_has_matrix_dimension_t();
}
inline ::google::protobuf::int32 DpStSpeedConfig::matrix_dimension_t() const {
  // @@protoc_insertion_point(field_get:acu.planning.DpStSpeedConfig.matrix_dimension_t)
  return matrix_dimension_t_;
}
inline void DpStSpeedConfig::set_matrix_dimension_t(::google::protobuf::int32 value) {
  set_has_matrix_dimension_t();
  matrix_dimension_t_ = value;
  // @@protoc_insertion_point(field_set:acu.planning.DpStSpeedConfig.matrix_dimension_t)
}

// optional double speed_weight = 5 [default = 0];
inline bool DpStSpeedConfig::has_speed_weight() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void DpStSpeedConfig::set_has_speed_weight() {
  _has_bits_[0] |= 0x00000002u;
}
inline void DpStSpeedConfig::clear_has_speed_weight() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void DpStSpeedConfig::clear_speed_weight() {
  speed_weight_ = 0;
  clear_has_speed_weight();
}
inline double DpStSpeedConfig::speed_weight() const {
  // @@protoc_insertion_point(field_get:acu.planning.DpStSpeedConfig.speed_weight)
  return speed_weight_;
}
inline void DpStSpeedConfig::set_speed_weight(double value) {
  set_has_speed_weight();
  speed_weight_ = value;
  // @@protoc_insertion_point(field_set:acu.planning.DpStSpeedConfig.speed_weight)
}

// optional double accel_weight = 6 [default = 10];
inline bool DpStSpeedConfig::has_accel_weight() const {
  return (_has_bits_[0] & 0x00000080u) != 0;
}
inline void DpStSpeedConfig::set_has_accel_weight() {
  _has_bits_[0] |= 0x00000080u;
}
inline void DpStSpeedConfig::clear_has_accel_weight() {
  _has_bits_[0] &= ~0x00000080u;
}
inline void DpStSpeedConfig::clear_accel_weight() {
  accel_weight_ = 10;
  clear_has_accel_weight();
}
inline double DpStSpeedConfig::accel_weight() const {
  // @@protoc_insertion_point(field_get:acu.planning.DpStSpeedConfig.accel_weight)
  return accel_weight_;
}
inline void DpStSpeedConfig::set_accel_weight(double value) {
  set_has_accel_weight();
  accel_weight_ = value;
  // @@protoc_insertion_point(field_set:acu.planning.DpStSpeedConfig.accel_weight)
}

// optional double jerk_weight = 7 [default = 10];
inline bool DpStSpeedConfig::has_jerk_weight() const {
  return (_has_bits_[0] & 0x00000100u) != 0;
}
inline void DpStSpeedConfig::set_has_jerk_weight() {
  _has_bits_[0] |= 0x00000100u;
}
inline void DpStSpeedConfig::clear_has_jerk_weight() {
  _has_bits_[0] &= ~0x00000100u;
}
inline void DpStSpeedConfig::clear_jerk_weight() {
  jerk_weight_ = 10;
  clear_has_jerk_weight();
}
inline double DpStSpeedConfig::jerk_weight() const {
  // @@protoc_insertion_point(field_get:acu.planning.DpStSpeedConfig.jerk_weight)
  return jerk_weight_;
}
inline void DpStSpeedConfig::set_jerk_weight(double value) {
  set_has_jerk_weight();
  jerk_weight_ = value;
  // @@protoc_insertion_point(field_set:acu.planning.DpStSpeedConfig.jerk_weight)
}

// optional double obstacle_weight = 8 [default = 1];
inline bool DpStSpeedConfig::has_obstacle_weight() const {
  return (_has_bits_[0] & 0x00000200u) != 0;
}
inline void DpStSpeedConfig::set_has_obstacle_weight() {
  _has_bits_[0] |= 0x00000200u;
}
inline void DpStSpeedConfig::clear_has_obstacle_weight() {
  _has_bits_[0] &= ~0x00000200u;
}
inline void DpStSpeedConfig::clear_obstacle_weight() {
  obstacle_weight_ = 1;
  clear_has_obstacle_weight();
}
inline double DpStSpeedConfig::obstacle_weight() const {
  // @@protoc_insertion_point(field_get:acu.planning.DpStSpeedConfig.obstacle_weight)
  return obstacle_weight_;
}
inline void DpStSpeedConfig::set_obstacle_weight(double value) {
  set_has_obstacle_weight();
  obstacle_weight_ = value;
  // @@protoc_insertion_point(field_set:acu.planning.DpStSpeedConfig.obstacle_weight)
}

// optional double reference_weight = 9 [default = 0];
inline bool DpStSpeedConfig::has_reference_weight() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void DpStSpeedConfig::set_has_reference_weight() {
  _has_bits_[0] |= 0x00000004u;
}
inline void DpStSpeedConfig::clear_has_reference_weight() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void DpStSpeedConfig::clear_reference_weight() {
  reference_weight_ = 0;
  clear_has_reference_weight();
}
inline double DpStSpeedConfig::reference_weight() const {
  // @@protoc_insertion_point(field_get:acu.planning.DpStSpeedConfig.reference_weight)
  return reference_weight_;
}
inline void DpStSpeedConfig::set_reference_weight(double value) {
  set_has_reference_weight();
  reference_weight_ = value;
  // @@protoc_insertion_point(field_set:acu.planning.DpStSpeedConfig.reference_weight)
}

// optional double go_down_buffer = 10 [default = 5];
inline bool DpStSpeedConfig::has_go_down_buffer() const {
  return (_has_bits_[0] & 0x00000400u) != 0;
}
inline void DpStSpeedConfig::set_has_go_down_buffer() {
  _has_bits_[0] |= 0x00000400u;
}
inline void DpStSpeedConfig::clear_has_go_down_buffer() {
  _has_bits_[0] &= ~0x00000400u;
}
inline void DpStSpeedConfig::clear_go_down_buffer() {
  go_down_buffer_ = 5;
  clear_has_go_down_buffer();
}
inline double DpStSpeedConfig::go_down_buffer() const {
  // @@protoc_insertion_point(field_get:acu.planning.DpStSpeedConfig.go_down_buffer)
  return go_down_buffer_;
}
inline void DpStSpeedConfig::set_go_down_buffer(double value) {
  set_has_go_down_buffer();
  go_down_buffer_ = value;
  // @@protoc_insertion_point(field_set:acu.planning.DpStSpeedConfig.go_down_buffer)
}

// optional double go_up_buffer = 11 [default = 5];
inline bool DpStSpeedConfig::has_go_up_buffer() const {
  return (_has_bits_[0] & 0x00000800u) != 0;
}
inline void DpStSpeedConfig::set_has_go_up_buffer() {
  _has_bits_[0] |= 0x00000800u;
}
inline void DpStSpeedConfig::clear_has_go_up_buffer() {
  _has_bits_[0] &= ~0x00000800u;
}
inline void DpStSpeedConfig::clear_go_up_buffer() {
  go_up_buffer_ = 5;
  clear_has_go_up_buffer();
}
inline double DpStSpeedConfig::go_up_buffer() const {
  // @@protoc_insertion_point(field_get:acu.planning.DpStSpeedConfig.go_up_buffer)
  return go_up_buffer_;
}
inline void DpStSpeedConfig::set_go_up_buffer(double value) {
  set_has_go_up_buffer();
  go_up_buffer_ = value;
  // @@protoc_insertion_point(field_set:acu.planning.DpStSpeedConfig.go_up_buffer)
}

// optional double default_obstacle_cost = 12 [default = 10000000000];
inline bool DpStSpeedConfig::has_default_obstacle_cost() const {
  return (_has_bits_[0] & 0x00001000u) != 0;
}
inline void DpStSpeedConfig::set_has_default_obstacle_cost() {
  _has_bits_[0] |= 0x00001000u;
}
inline void DpStSpeedConfig::clear_has_default_obstacle_cost() {
  _has_bits_[0] &= ~0x00001000u;
}
inline void DpStSpeedConfig::clear_default_obstacle_cost() {
  default_obstacle_cost_ = 10000000000;
  clear_has_default_obstacle_cost();
}
inline double DpStSpeedConfig::default_obstacle_cost() const {
  // @@protoc_insertion_point(field_get:acu.planning.DpStSpeedConfig.default_obstacle_cost)
  return default_obstacle_cost_;
}
inline void DpStSpeedConfig::set_default_obstacle_cost(double value) {
  set_has_default_obstacle_cost();
  default_obstacle_cost_ = value;
  // @@protoc_insertion_point(field_set:acu.planning.DpStSpeedConfig.default_obstacle_cost)
}

// optional double default_speed_cost = 13 [default = 1];
inline bool DpStSpeedConfig::has_default_speed_cost() const {
  return (_has_bits_[0] & 0x00002000u) != 0;
}
inline void DpStSpeedConfig::set_has_default_speed_cost() {
  _has_bits_[0] |= 0x00002000u;
}
inline void DpStSpeedConfig::clear_has_default_speed_cost() {
  _has_bits_[0] &= ~0x00002000u;
}
inline void DpStSpeedConfig::clear_default_speed_cost() {
  default_speed_cost_ = 1;
  clear_has_default_speed_cost();
}
inline double DpStSpeedConfig::default_speed_cost() const {
  // @@protoc_insertion_point(field_get:acu.planning.DpStSpeedConfig.default_speed_cost)
  return default_speed_cost_;
}
inline void DpStSpeedConfig::set_default_speed_cost(double value) {
  set_has_default_speed_cost();
  default_speed_cost_ = value;
  // @@protoc_insertion_point(field_set:acu.planning.DpStSpeedConfig.default_speed_cost)
}

// optional double exceed_speed_penalty = 14 [default = 10];
inline bool DpStSpeedConfig::has_exceed_speed_penalty() const {
  return (_has_bits_[0] & 0x00004000u) != 0;
}
inline void DpStSpeedConfig::set_has_exceed_speed_penalty() {
  _has_bits_[0] |= 0x00004000u;
}
inline void DpStSpeedConfig::clear_has_exceed_speed_penalty() {
  _has_bits_[0] &= ~0x00004000u;
}
inline void DpStSpeedConfig::clear_exceed_speed_penalty() {
  exceed_speed_penalty_ = 10;
  clear_has_exceed_speed_penalty();
}
inline double DpStSpeedConfig::exceed_speed_penalty() const {
  // @@protoc_insertion_point(field_get:acu.planning.DpStSpeedConfig.exceed_speed_penalty)
  return exceed_speed_penalty_;
}
inline void DpStSpeedConfig::set_exceed_speed_penalty(double value) {
  set_has_exceed_speed_penalty();
  exceed_speed_penalty_ = value;
  // @@protoc_insertion_point(field_set:acu.planning.DpStSpeedConfig.exceed_speed_penalty)
}

// optional double low_speed_penalty = 15 [default = 2.5];
inline bool DpStSpeedConfig::has_low_speed_penalty() const {
  return (_has_bits_[0] & 0x00008000u) != 0;
}
inline void DpStSpeedConfig::set_has_low_speed_penalty() {
  _has_bits_[0] |= 0x00008000u;
}
inline void DpStSpeedConfig::clear_has_low_speed_penalty() {
  _has_bits_[0] &= ~0x00008000u;
}
inline void DpStSpeedConfig::clear_low_speed_penalty() {
  low_speed_penalty_ = 2.5;
  clear_has_low_speed_penalty();
}
inline double DpStSpeedConfig::low_speed_penalty() const {
  // @@protoc_insertion_point(field_get:acu.planning.DpStSpeedConfig.low_speed_penalty)
  return low_speed_penalty_;
}
inline void DpStSpeedConfig::set_low_speed_penalty(double value) {
  set_has_low_speed_penalty();
  low_speed_penalty_ = value;
  // @@protoc_insertion_point(field_set:acu.planning.DpStSpeedConfig.low_speed_penalty)
}

// optional double keep_clear_low_speed_penalty = 16 [default = 10];
inline bool DpStSpeedConfig::has_keep_clear_low_speed_penalty() const {
  return (_has_bits_[0] & 0x00010000u) != 0;
}
inline void DpStSpeedConfig::set_has_keep_clear_low_speed_penalty() {
  _has_bits_[0] |= 0x00010000u;
}
inline void DpStSpeedConfig::clear_has_keep_clear_low_speed_penalty() {
  _has_bits_[0] &= ~0x00010000u;
}
inline void DpStSpeedConfig::clear_keep_clear_low_speed_penalty() {
  keep_clear_low_speed_penalty_ = 10;
  clear_has_keep_clear_low_speed_penalty();
}
inline double DpStSpeedConfig::keep_clear_low_speed_penalty() const {
  // @@protoc_insertion_point(field_get:acu.planning.DpStSpeedConfig.keep_clear_low_speed_penalty)
  return keep_clear_low_speed_penalty_;
}
inline void DpStSpeedConfig::set_keep_clear_low_speed_penalty(double value) {
  set_has_keep_clear_low_speed_penalty();
  keep_clear_low_speed_penalty_ = value;
  // @@protoc_insertion_point(field_set:acu.planning.DpStSpeedConfig.keep_clear_low_speed_penalty)
}

// optional double accel_penalty = 20 [default = 2];
inline bool DpStSpeedConfig::has_accel_penalty() const {
  return (_has_bits_[0] & 0x00020000u) != 0;
}
inline void DpStSpeedConfig::set_has_accel_penalty() {
  _has_bits_[0] |= 0x00020000u;
}
inline void DpStSpeedConfig::clear_has_accel_penalty() {
  _has_bits_[0] &= ~0x00020000u;
}
inline void DpStSpeedConfig::clear_accel_penalty() {
  accel_penalty_ = 2;
  clear_has_accel_penalty();
}
inline double DpStSpeedConfig::accel_penalty() const {
  // @@protoc_insertion_point(field_get:acu.planning.DpStSpeedConfig.accel_penalty)
  return accel_penalty_;
}
inline void DpStSpeedConfig::set_accel_penalty(double value) {
  set_has_accel_penalty();
  accel_penalty_ = value;
  // @@protoc_insertion_point(field_set:acu.planning.DpStSpeedConfig.accel_penalty)
}

// optional double decel_penalty = 21 [default = 2];
inline bool DpStSpeedConfig::has_decel_penalty() const {
  return (_has_bits_[0] & 0x00040000u) != 0;
}
inline void DpStSpeedConfig::set_has_decel_penalty() {
  _has_bits_[0] |= 0x00040000u;
}
inline void DpStSpeedConfig::clear_has_decel_penalty() {
  _has_bits_[0] &= ~0x00040000u;
}
inline void DpStSpeedConfig::clear_decel_penalty() {
  decel_penalty_ = 2;
  clear_has_decel_penalty();
}
inline double DpStSpeedConfig::decel_penalty() const {
  // @@protoc_insertion_point(field_get:acu.planning.DpStSpeedConfig.decel_penalty)
  return decel_penalty_;
}
inline void DpStSpeedConfig::set_decel_penalty(double value) {
  set_has_decel_penalty();
  decel_penalty_ = value;
  // @@protoc_insertion_point(field_set:acu.planning.DpStSpeedConfig.decel_penalty)
}

// optional double positive_jerk_coeff = 30 [default = 1];
inline bool DpStSpeedConfig::has_positive_jerk_coeff() const {
  return (_has_bits_[0] & 0x00080000u) != 0;
}
inline void DpStSpeedConfig::set_has_positive_jerk_coeff() {
  _has_bits_[0] |= 0x00080000u;
}
inline void DpStSpeedConfig::clear_has_positive_jerk_coeff() {
  _has_bits_[0] &= ~0x00080000u;
}
inline void DpStSpeedConfig::clear_positive_jerk_coeff() {
  positive_jerk_coeff_ = 1;
  clear_has_positive_jerk_coeff();
}
inline double DpStSpeedConfig::positive_jerk_coeff() const {
  // @@protoc_insertion_point(field_get:acu.planning.DpStSpeedConfig.positive_jerk_coeff)
  return positive_jerk_coeff_;
}
inline void DpStSpeedConfig::set_positive_jerk_coeff(double value) {
  set_has_positive_jerk_coeff();
  positive_jerk_coeff_ = value;
  // @@protoc_insertion_point(field_set:acu.planning.DpStSpeedConfig.positive_jerk_coeff)
}

// optional double negative_jerk_coeff = 31 [default = 300];
inline bool DpStSpeedConfig::has_negative_jerk_coeff() const {
  return (_has_bits_[0] & 0x00100000u) != 0;
}
inline void DpStSpeedConfig::set_has_negative_jerk_coeff() {
  _has_bits_[0] |= 0x00100000u;
}
inline void DpStSpeedConfig::clear_has_negative_jerk_coeff() {
  _has_bits_[0] &= ~0x00100000u;
}
inline void DpStSpeedConfig::clear_negative_jerk_coeff() {
  negative_jerk_coeff_ = 300;
  clear_has_negative_jerk_coeff();
}
inline double DpStSpeedConfig::negative_jerk_coeff() const {
  // @@protoc_insertion_point(field_get:acu.planning.DpStSpeedConfig.negative_jerk_coeff)
  return negative_jerk_coeff_;
}
inline void DpStSpeedConfig::set_negative_jerk_coeff(double value) {
  set_has_negative_jerk_coeff();
  negative_jerk_coeff_ = value;
  // @@protoc_insertion_point(field_set:acu.planning.DpStSpeedConfig.negative_jerk_coeff)
}

// optional double max_acceleration = 40 [default = 4.5];
inline bool DpStSpeedConfig::has_max_acceleration() const {
  return (_has_bits_[0] & 0x00200000u) != 0;
}
inline void DpStSpeedConfig::set_has_max_acceleration() {
  _has_bits_[0] |= 0x00200000u;
}
inline void DpStSpeedConfig::clear_has_max_acceleration() {
  _has_bits_[0] &= ~0x00200000u;
}
inline void DpStSpeedConfig::clear_max_acceleration() {
  max_acceleration_ = 4.5;
  clear_has_max_acceleration();
}
inline double DpStSpeedConfig::max_acceleration() const {
  // @@protoc_insertion_point(field_get:acu.planning.DpStSpeedConfig.max_acceleration)
  return max_acceleration_;
}
inline void DpStSpeedConfig::set_max_acceleration(double value) {
  set_has_max_acceleration();
  max_acceleration_ = value;
  // @@protoc_insertion_point(field_set:acu.planning.DpStSpeedConfig.max_acceleration)
}

// optional double max_deceleration = 41 [default = -4.5];
inline bool DpStSpeedConfig::has_max_deceleration() const {
  return (_has_bits_[0] & 0x00400000u) != 0;
}
inline void DpStSpeedConfig::set_has_max_deceleration() {
  _has_bits_[0] |= 0x00400000u;
}
inline void DpStSpeedConfig::clear_has_max_deceleration() {
  _has_bits_[0] &= ~0x00400000u;
}
inline void DpStSpeedConfig::clear_max_deceleration() {
  max_deceleration_ = -4.5;
  clear_has_max_deceleration();
}
inline double DpStSpeedConfig::max_deceleration() const {
  // @@protoc_insertion_point(field_get:acu.planning.DpStSpeedConfig.max_deceleration)
  return max_deceleration_;
}
inline void DpStSpeedConfig::set_max_deceleration(double value) {
  set_has_max_deceleration();
  max_deceleration_ = value;
  // @@protoc_insertion_point(field_set:acu.planning.DpStSpeedConfig.max_deceleration)
}

// optional .acu.planning.StBoundaryConfig st_boundary_config = 50;
inline bool DpStSpeedConfig::has_st_boundary_config() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void DpStSpeedConfig::set_has_st_boundary_config() {
  _has_bits_[0] |= 0x00000001u;
}
inline void DpStSpeedConfig::clear_has_st_boundary_config() {
  _has_bits_[0] &= ~0x00000001u;
}
inline const ::acu::planning::StBoundaryConfig& DpStSpeedConfig::st_boundary_config() const {
  const ::acu::planning::StBoundaryConfig* p = st_boundary_config_;
  // @@protoc_insertion_point(field_get:acu.planning.DpStSpeedConfig.st_boundary_config)
  return p != NULL ? *p : *reinterpret_cast<const ::acu::planning::StBoundaryConfig*>(
      &::acu::planning::_StBoundaryConfig_default_instance_);
}
inline ::acu::planning::StBoundaryConfig* DpStSpeedConfig::release_st_boundary_config() {
  // @@protoc_insertion_point(field_release:acu.planning.DpStSpeedConfig.st_boundary_config)
  clear_has_st_boundary_config();
  ::acu::planning::StBoundaryConfig* temp = st_boundary_config_;
  st_boundary_config_ = NULL;
  return temp;
}
inline ::acu::planning::StBoundaryConfig* DpStSpeedConfig::mutable_st_boundary_config() {
  set_has_st_boundary_config();
  if (st_boundary_config_ == NULL) {
    st_boundary_config_ = new ::acu::planning::StBoundaryConfig;
  }
  // @@protoc_insertion_point(field_mutable:acu.planning.DpStSpeedConfig.st_boundary_config)
  return st_boundary_config_;
}
inline void DpStSpeedConfig::set_allocated_st_boundary_config(::acu::planning::StBoundaryConfig* st_boundary_config) {
  ::google::protobuf::Arena* message_arena = GetArenaNoVirtual();
  if (message_arena == NULL) {
    delete reinterpret_cast< ::google::protobuf::MessageLite*>(st_boundary_config_);
  }
  if (st_boundary_config) {
    ::google::protobuf::Arena* submessage_arena = NULL;
    if (message_arena != submessage_arena) {
      st_boundary_config = ::google::protobuf::internal::GetOwnedMessage(
          message_arena, st_boundary_config, submessage_arena);
    }
    set_has_st_boundary_config();
  } else {
    clear_has_st_boundary_config();
  }
  st_boundary_config_ = st_boundary_config;
  // @@protoc_insertion_point(field_set_allocated:acu.planning.DpStSpeedConfig.st_boundary_config)
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__

// @@protoc_insertion_point(namespace_scope)

}  // namespace planning
}  // namespace acu

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_dp_5fst_5fspeed_5fconfig_2eproto__INCLUDED
