// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: perception_types.proto

#ifndef PROTOBUF_perception_5ftypes_2eproto__INCLUDED
#define PROTOBUF_perception_5ftypes_2eproto__INCLUDED

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

namespace protobuf_perception_5ftypes_2eproto {
// Internal implementation detail -- do not use these members.
struct TableStruct {
  static const ::google::protobuf::internal::ParseTableField entries[];
  static const ::google::protobuf::internal::AuxillaryParseTableField aux[];
  static const ::google::protobuf::internal::ParseTable schema[5];
  static const ::google::protobuf::internal::FieldMetadata field_metadata[];
  static const ::google::protobuf::internal::SerializationTable serialization_table[];
  static const ::google::protobuf::uint32 offsets[];
};
void AddDescriptors();
void InitDefaultsPointImpl();
void InitDefaultsPoint();
void InitDefaultsPoint2iImpl();
void InitDefaultsPoint2i();
void InitDefaultsRectImpl();
void InitDefaultsRect();
void InitDefaultsSize2fImpl();
void InitDefaultsSize2f();
void InitDefaultsMatrix2dImpl();
void InitDefaultsMatrix2d();
inline void InitDefaults() {
  InitDefaultsPoint();
  InitDefaultsPoint2i();
  InitDefaultsRect();
  InitDefaultsSize2f();
  InitDefaultsMatrix2d();
}
}  // namespace protobuf_perception_5ftypes_2eproto
namespace calmcar {
namespace perception {
class Matrix2d;
class Matrix2dDefaultTypeInternal;
extern Matrix2dDefaultTypeInternal _Matrix2d_default_instance_;
class Point;
class PointDefaultTypeInternal;
extern PointDefaultTypeInternal _Point_default_instance_;
class Point2i;
class Point2iDefaultTypeInternal;
extern Point2iDefaultTypeInternal _Point2i_default_instance_;
class Rect;
class RectDefaultTypeInternal;
extern RectDefaultTypeInternal _Rect_default_instance_;
class Size2f;
class Size2fDefaultTypeInternal;
extern Size2fDefaultTypeInternal _Size2f_default_instance_;
}  // namespace perception
}  // namespace calmcar
namespace calmcar {
namespace perception {

// ===================================================================

class Point : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:calmcar.perception.Point) */ {
 public:
  Point();
  virtual ~Point();

  Point(const Point& from);

  inline Point& operator=(const Point& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Point(Point&& from) noexcept
    : Point() {
    *this = ::std::move(from);
  }

  inline Point& operator=(Point&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const Point& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Point* internal_default_instance() {
    return reinterpret_cast<const Point*>(
               &_Point_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    0;

  void Swap(Point* other);
  friend void swap(Point& a, Point& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Point* New() const PROTOBUF_FINAL { return New(NULL); }

  Point* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const Point& from);
  void MergeFrom(const Point& from);
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
  void InternalSwap(Point* other);
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

  // double x = 1;
  void clear_x();
  static const int kXFieldNumber = 1;
  double x() const;
  void set_x(double value);

  // double y = 2;
  void clear_y();
  static const int kYFieldNumber = 2;
  double y() const;
  void set_y(double value);

  // double z = 3;
  void clear_z();
  static const int kZFieldNumber = 3;
  double z() const;
  void set_z(double value);

  // @@protoc_insertion_point(class_scope:calmcar.perception.Point)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  double x_;
  double y_;
  double z_;
  mutable int _cached_size_;
  friend struct ::protobuf_perception_5ftypes_2eproto::TableStruct;
  friend void ::protobuf_perception_5ftypes_2eproto::InitDefaultsPointImpl();
};
// -------------------------------------------------------------------

class Point2i : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:calmcar.perception.Point2i) */ {
 public:
  Point2i();
  virtual ~Point2i();

  Point2i(const Point2i& from);

  inline Point2i& operator=(const Point2i& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Point2i(Point2i&& from) noexcept
    : Point2i() {
    *this = ::std::move(from);
  }

  inline Point2i& operator=(Point2i&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const Point2i& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Point2i* internal_default_instance() {
    return reinterpret_cast<const Point2i*>(
               &_Point2i_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    1;

  void Swap(Point2i* other);
  friend void swap(Point2i& a, Point2i& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Point2i* New() const PROTOBUF_FINAL { return New(NULL); }

  Point2i* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const Point2i& from);
  void MergeFrom(const Point2i& from);
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
  void InternalSwap(Point2i* other);
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

  // int64 x = 1;
  void clear_x();
  static const int kXFieldNumber = 1;
  ::google::protobuf::int64 x() const;
  void set_x(::google::protobuf::int64 value);

  // int64 y = 2;
  void clear_y();
  static const int kYFieldNumber = 2;
  ::google::protobuf::int64 y() const;
  void set_y(::google::protobuf::int64 value);

  // @@protoc_insertion_point(class_scope:calmcar.perception.Point2i)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::int64 x_;
  ::google::protobuf::int64 y_;
  mutable int _cached_size_;
  friend struct ::protobuf_perception_5ftypes_2eproto::TableStruct;
  friend void ::protobuf_perception_5ftypes_2eproto::InitDefaultsPoint2iImpl();
};
// -------------------------------------------------------------------

class Rect : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:calmcar.perception.Rect) */ {
 public:
  Rect();
  virtual ~Rect();

  Rect(const Rect& from);

  inline Rect& operator=(const Rect& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Rect(Rect&& from) noexcept
    : Rect() {
    *this = ::std::move(from);
  }

  inline Rect& operator=(Rect&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const Rect& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Rect* internal_default_instance() {
    return reinterpret_cast<const Rect*>(
               &_Rect_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    2;

  void Swap(Rect* other);
  friend void swap(Rect& a, Rect& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Rect* New() const PROTOBUF_FINAL { return New(NULL); }

  Rect* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const Rect& from);
  void MergeFrom(const Rect& from);
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
  void InternalSwap(Rect* other);
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

  // double x = 1;
  void clear_x();
  static const int kXFieldNumber = 1;
  double x() const;
  void set_x(double value);

  // double y = 2;
  void clear_y();
  static const int kYFieldNumber = 2;
  double y() const;
  void set_y(double value);

  // double width = 3;
  void clear_width();
  static const int kWidthFieldNumber = 3;
  double width() const;
  void set_width(double value);

  // double height = 4;
  void clear_height();
  static const int kHeightFieldNumber = 4;
  double height() const;
  void set_height(double value);

  // @@protoc_insertion_point(class_scope:calmcar.perception.Rect)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  double x_;
  double y_;
  double width_;
  double height_;
  mutable int _cached_size_;
  friend struct ::protobuf_perception_5ftypes_2eproto::TableStruct;
  friend void ::protobuf_perception_5ftypes_2eproto::InitDefaultsRectImpl();
};
// -------------------------------------------------------------------

class Size2f : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:calmcar.perception.Size2f) */ {
 public:
  Size2f();
  virtual ~Size2f();

  Size2f(const Size2f& from);

  inline Size2f& operator=(const Size2f& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Size2f(Size2f&& from) noexcept
    : Size2f() {
    *this = ::std::move(from);
  }

  inline Size2f& operator=(Size2f&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const Size2f& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Size2f* internal_default_instance() {
    return reinterpret_cast<const Size2f*>(
               &_Size2f_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    3;

  void Swap(Size2f* other);
  friend void swap(Size2f& a, Size2f& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Size2f* New() const PROTOBUF_FINAL { return New(NULL); }

  Size2f* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const Size2f& from);
  void MergeFrom(const Size2f& from);
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
  void InternalSwap(Size2f* other);
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

  // double width = 1;
  void clear_width();
  static const int kWidthFieldNumber = 1;
  double width() const;
  void set_width(double value);

  // double length = 2;
  void clear_length();
  static const int kLengthFieldNumber = 2;
  double length() const;
  void set_length(double value);

  // @@protoc_insertion_point(class_scope:calmcar.perception.Size2f)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  double width_;
  double length_;
  mutable int _cached_size_;
  friend struct ::protobuf_perception_5ftypes_2eproto::TableStruct;
  friend void ::protobuf_perception_5ftypes_2eproto::InitDefaultsSize2fImpl();
};
// -------------------------------------------------------------------

class Matrix2d : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:calmcar.perception.Matrix2d) */ {
 public:
  Matrix2d();
  virtual ~Matrix2d();

  Matrix2d(const Matrix2d& from);

  inline Matrix2d& operator=(const Matrix2d& from) {
    CopyFrom(from);
    return *this;
  }
  #if LANG_CXX11
  Matrix2d(Matrix2d&& from) noexcept
    : Matrix2d() {
    *this = ::std::move(from);
  }

  inline Matrix2d& operator=(Matrix2d&& from) noexcept {
    if (GetArenaNoVirtual() == from.GetArenaNoVirtual()) {
      if (this != &from) InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }
  #endif
  static const ::google::protobuf::Descriptor* descriptor();
  static const Matrix2d& default_instance();

  static void InitAsDefaultInstance();  // FOR INTERNAL USE ONLY
  static inline const Matrix2d* internal_default_instance() {
    return reinterpret_cast<const Matrix2d*>(
               &_Matrix2d_default_instance_);
  }
  static PROTOBUF_CONSTEXPR int const kIndexInFileMessages =
    4;

  void Swap(Matrix2d* other);
  friend void swap(Matrix2d& a, Matrix2d& b) {
    a.Swap(&b);
  }

  // implements Message ----------------------------------------------

  inline Matrix2d* New() const PROTOBUF_FINAL { return New(NULL); }

  Matrix2d* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const Matrix2d& from);
  void MergeFrom(const Matrix2d& from);
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
  void InternalSwap(Matrix2d* other);
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

  // repeated double data = 3;
  int data_size() const;
  void clear_data();
  static const int kDataFieldNumber = 3;
  double data(int index) const;
  void set_data(int index, double value);
  void add_data(double value);
  const ::google::protobuf::RepeatedField< double >&
      data() const;
  ::google::protobuf::RepeatedField< double >*
      mutable_data();

  // int64 rows = 1;
  void clear_rows();
  static const int kRowsFieldNumber = 1;
  ::google::protobuf::int64 rows() const;
  void set_rows(::google::protobuf::int64 value);

  // int64 columns = 2;
  void clear_columns();
  static const int kColumnsFieldNumber = 2;
  ::google::protobuf::int64 columns() const;
  void set_columns(::google::protobuf::int64 value);

  // @@protoc_insertion_point(class_scope:calmcar.perception.Matrix2d)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::google::protobuf::RepeatedField< double > data_;
  mutable int _data_cached_byte_size_;
  ::google::protobuf::int64 rows_;
  ::google::protobuf::int64 columns_;
  mutable int _cached_size_;
  friend struct ::protobuf_perception_5ftypes_2eproto::TableStruct;
  friend void ::protobuf_perception_5ftypes_2eproto::InitDefaultsMatrix2dImpl();
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Point

// double x = 1;
inline void Point::clear_x() {
  x_ = 0;
}
inline double Point::x() const {
  // @@protoc_insertion_point(field_get:calmcar.perception.Point.x)
  return x_;
}
inline void Point::set_x(double value) {
  
  x_ = value;
  // @@protoc_insertion_point(field_set:calmcar.perception.Point.x)
}

// double y = 2;
inline void Point::clear_y() {
  y_ = 0;
}
inline double Point::y() const {
  // @@protoc_insertion_point(field_get:calmcar.perception.Point.y)
  return y_;
}
inline void Point::set_y(double value) {
  
  y_ = value;
  // @@protoc_insertion_point(field_set:calmcar.perception.Point.y)
}

// double z = 3;
inline void Point::clear_z() {
  z_ = 0;
}
inline double Point::z() const {
  // @@protoc_insertion_point(field_get:calmcar.perception.Point.z)
  return z_;
}
inline void Point::set_z(double value) {
  
  z_ = value;
  // @@protoc_insertion_point(field_set:calmcar.perception.Point.z)
}

// -------------------------------------------------------------------

// Point2i

// int64 x = 1;
inline void Point2i::clear_x() {
  x_ = GOOGLE_LONGLONG(0);
}
inline ::google::protobuf::int64 Point2i::x() const {
  // @@protoc_insertion_point(field_get:calmcar.perception.Point2i.x)
  return x_;
}
inline void Point2i::set_x(::google::protobuf::int64 value) {
  
  x_ = value;
  // @@protoc_insertion_point(field_set:calmcar.perception.Point2i.x)
}

// int64 y = 2;
inline void Point2i::clear_y() {
  y_ = GOOGLE_LONGLONG(0);
}
inline ::google::protobuf::int64 Point2i::y() const {
  // @@protoc_insertion_point(field_get:calmcar.perception.Point2i.y)
  return y_;
}
inline void Point2i::set_y(::google::protobuf::int64 value) {
  
  y_ = value;
  // @@protoc_insertion_point(field_set:calmcar.perception.Point2i.y)
}

// -------------------------------------------------------------------

// Rect

// double x = 1;
inline void Rect::clear_x() {
  x_ = 0;
}
inline double Rect::x() const {
  // @@protoc_insertion_point(field_get:calmcar.perception.Rect.x)
  return x_;
}
inline void Rect::set_x(double value) {
  
  x_ = value;
  // @@protoc_insertion_point(field_set:calmcar.perception.Rect.x)
}

// double y = 2;
inline void Rect::clear_y() {
  y_ = 0;
}
inline double Rect::y() const {
  // @@protoc_insertion_point(field_get:calmcar.perception.Rect.y)
  return y_;
}
inline void Rect::set_y(double value) {
  
  y_ = value;
  // @@protoc_insertion_point(field_set:calmcar.perception.Rect.y)
}

// double width = 3;
inline void Rect::clear_width() {
  width_ = 0;
}
inline double Rect::width() const {
  // @@protoc_insertion_point(field_get:calmcar.perception.Rect.width)
  return width_;
}
inline void Rect::set_width(double value) {
  
  width_ = value;
  // @@protoc_insertion_point(field_set:calmcar.perception.Rect.width)
}

// double height = 4;
inline void Rect::clear_height() {
  height_ = 0;
}
inline double Rect::height() const {
  // @@protoc_insertion_point(field_get:calmcar.perception.Rect.height)
  return height_;
}
inline void Rect::set_height(double value) {
  
  height_ = value;
  // @@protoc_insertion_point(field_set:calmcar.perception.Rect.height)
}

// -------------------------------------------------------------------

// Size2f

// double width = 1;
inline void Size2f::clear_width() {
  width_ = 0;
}
inline double Size2f::width() const {
  // @@protoc_insertion_point(field_get:calmcar.perception.Size2f.width)
  return width_;
}
inline void Size2f::set_width(double value) {
  
  width_ = value;
  // @@protoc_insertion_point(field_set:calmcar.perception.Size2f.width)
}

// double length = 2;
inline void Size2f::clear_length() {
  length_ = 0;
}
inline double Size2f::length() const {
  // @@protoc_insertion_point(field_get:calmcar.perception.Size2f.length)
  return length_;
}
inline void Size2f::set_length(double value) {
  
  length_ = value;
  // @@protoc_insertion_point(field_set:calmcar.perception.Size2f.length)
}

// -------------------------------------------------------------------

// Matrix2d

// int64 rows = 1;
inline void Matrix2d::clear_rows() {
  rows_ = GOOGLE_LONGLONG(0);
}
inline ::google::protobuf::int64 Matrix2d::rows() const {
  // @@protoc_insertion_point(field_get:calmcar.perception.Matrix2d.rows)
  return rows_;
}
inline void Matrix2d::set_rows(::google::protobuf::int64 value) {
  
  rows_ = value;
  // @@protoc_insertion_point(field_set:calmcar.perception.Matrix2d.rows)
}

// int64 columns = 2;
inline void Matrix2d::clear_columns() {
  columns_ = GOOGLE_LONGLONG(0);
}
inline ::google::protobuf::int64 Matrix2d::columns() const {
  // @@protoc_insertion_point(field_get:calmcar.perception.Matrix2d.columns)
  return columns_;
}
inline void Matrix2d::set_columns(::google::protobuf::int64 value) {
  
  columns_ = value;
  // @@protoc_insertion_point(field_set:calmcar.perception.Matrix2d.columns)
}

// repeated double data = 3;
inline int Matrix2d::data_size() const {
  return data_.size();
}
inline void Matrix2d::clear_data() {
  data_.Clear();
}
inline double Matrix2d::data(int index) const {
  // @@protoc_insertion_point(field_get:calmcar.perception.Matrix2d.data)
  return data_.Get(index);
}
inline void Matrix2d::set_data(int index, double value) {
  data_.Set(index, value);
  // @@protoc_insertion_point(field_set:calmcar.perception.Matrix2d.data)
}
inline void Matrix2d::add_data(double value) {
  data_.Add(value);
  // @@protoc_insertion_point(field_add:calmcar.perception.Matrix2d.data)
}
inline const ::google::protobuf::RepeatedField< double >&
Matrix2d::data() const {
  // @@protoc_insertion_point(field_list:calmcar.perception.Matrix2d.data)
  return data_;
}
inline ::google::protobuf::RepeatedField< double >*
Matrix2d::mutable_data() {
  // @@protoc_insertion_point(field_mutable_list:calmcar.perception.Matrix2d.data)
  return &data_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------

// -------------------------------------------------------------------

// -------------------------------------------------------------------

// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace perception
}  // namespace calmcar

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_perception_5ftypes_2eproto__INCLUDED
