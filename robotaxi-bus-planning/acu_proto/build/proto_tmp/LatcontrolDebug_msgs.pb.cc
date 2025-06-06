// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: LatcontrolDebug_msgs.proto

#include "LatcontrolDebug_msgs.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// This is a temporary google only hack
#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
#include "third_party/protobuf/version.h"
#endif
// @@protoc_insertion_point(includes)
namespace LatcontrolDebug_msgs {
class LatcontrolDebugDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<LatcontrolDebug>
      _instance;
} _LatcontrolDebug_default_instance_;
}  // namespace LatcontrolDebug_msgs
namespace protobuf_LatcontrolDebug_5fmsgs_2eproto {
void InitDefaultsLatcontrolDebugImpl() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
  ::google::protobuf::internal::InitProtobufDefaultsForceUnique();
#else
  ::google::protobuf::internal::InitProtobufDefaults();
#endif  // GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
  {
    void* ptr = &::LatcontrolDebug_msgs::_LatcontrolDebug_default_instance_;
    new (ptr) ::LatcontrolDebug_msgs::LatcontrolDebug();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::LatcontrolDebug_msgs::LatcontrolDebug::InitAsDefaultInstance();
}

void InitDefaultsLatcontrolDebug() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &InitDefaultsLatcontrolDebugImpl);
}

::google::protobuf::Metadata file_level_metadata[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::LatcontrolDebug_msgs::LatcontrolDebug, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::LatcontrolDebug_msgs::LatcontrolDebug, midback_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::LatcontrolDebug_msgs::LatcontrolDebug, midfront_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::LatcontrolDebug_msgs::LatcontrolDebug, lat_err_front_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::LatcontrolDebug_msgs::LatcontrolDebug, heading_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::LatcontrolDebug_msgs::LatcontrolDebug, leftfront_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::LatcontrolDebug_msgs::LatcontrolDebug, rightfront_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::LatcontrolDebug_msgs::LatcontrolDebug, leftback_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::LatcontrolDebug_msgs::LatcontrolDebug, rightback_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::LatcontrolDebug_msgs::LatcontrolDebug, dist_preview_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::LatcontrolDebug_msgs::LatcontrolDebug, x_previewpoint_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::LatcontrolDebug_msgs::LatcontrolDebug, y_previewpoint_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::LatcontrolDebug_msgs::LatcontrolDebug, r_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::LatcontrolDebug_msgs::LatcontrolDebug, eps_speed_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::LatcontrolDebug_msgs::LatcontrolDebug, temp1_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::LatcontrolDebug_msgs::LatcontrolDebug, temp2_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::LatcontrolDebug_msgs::LatcontrolDebug, temp3_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::LatcontrolDebug_msgs::LatcontrolDebug, temp4_),
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::LatcontrolDebug_msgs::LatcontrolDebug)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::LatcontrolDebug_msgs::_LatcontrolDebug_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  ::google::protobuf::MessageFactory* factory = NULL;
  AssignDescriptors(
      "LatcontrolDebug_msgs.proto", schemas, file_default_instances, TableStruct::offsets, factory,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_PROTOBUF_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 1);
}

void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
      "\n\032LatcontrolDebug_msgs.proto\022\024Latcontrol"
      "Debug_msgs\"\310\002\n\017LatcontrolDebug\022\017\n\007midbac"
      "k\030\001 \001(\002\022\020\n\010midfront\030\002 \001(\002\022\025\n\rlat_err_fro"
      "nt\030\003 \001(\002\022\017\n\007heading\030\004 \001(\002\022\021\n\tleftfront\030\005"
      " \001(\002\022\022\n\nrightfront\030\006 \001(\002\022\020\n\010leftback\030\007 \001"
      "(\002\022\021\n\trightback\030\010 \001(\002\022\024\n\014dist_preview\030\t "
      "\001(\002\022\026\n\016x_previewpoint\030\n \001(\002\022\026\n\016y_preview"
      "point\030\013 \001(\002\022\t\n\001R\030\014 \001(\002\022\021\n\teps_speed\030\r \001("
      "\002\022\r\n\005temp1\030\016 \001(\002\022\r\n\005temp2\030\017 \001(\002\022\r\n\005temp3"
      "\030\020 \001(\002\022\r\n\005temp4\030\021 \001(\002b\006proto3"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 389);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "LatcontrolDebug_msgs.proto", &protobuf_RegisterTypes);
}

void AddDescriptors() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at dynamic initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;
}  // namespace protobuf_LatcontrolDebug_5fmsgs_2eproto
namespace LatcontrolDebug_msgs {

// ===================================================================

void LatcontrolDebug::InitAsDefaultInstance() {
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int LatcontrolDebug::kMidbackFieldNumber;
const int LatcontrolDebug::kMidfrontFieldNumber;
const int LatcontrolDebug::kLatErrFrontFieldNumber;
const int LatcontrolDebug::kHeadingFieldNumber;
const int LatcontrolDebug::kLeftfrontFieldNumber;
const int LatcontrolDebug::kRightfrontFieldNumber;
const int LatcontrolDebug::kLeftbackFieldNumber;
const int LatcontrolDebug::kRightbackFieldNumber;
const int LatcontrolDebug::kDistPreviewFieldNumber;
const int LatcontrolDebug::kXPreviewpointFieldNumber;
const int LatcontrolDebug::kYPreviewpointFieldNumber;
const int LatcontrolDebug::kRFieldNumber;
const int LatcontrolDebug::kEpsSpeedFieldNumber;
const int LatcontrolDebug::kTemp1FieldNumber;
const int LatcontrolDebug::kTemp2FieldNumber;
const int LatcontrolDebug::kTemp3FieldNumber;
const int LatcontrolDebug::kTemp4FieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

LatcontrolDebug::LatcontrolDebug()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    ::protobuf_LatcontrolDebug_5fmsgs_2eproto::InitDefaultsLatcontrolDebug();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:LatcontrolDebug_msgs.LatcontrolDebug)
}
LatcontrolDebug::LatcontrolDebug(const LatcontrolDebug& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _cached_size_(0) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&midback_, &from.midback_,
    static_cast<size_t>(reinterpret_cast<char*>(&temp4_) -
    reinterpret_cast<char*>(&midback_)) + sizeof(temp4_));
  // @@protoc_insertion_point(copy_constructor:LatcontrolDebug_msgs.LatcontrolDebug)
}

void LatcontrolDebug::SharedCtor() {
  ::memset(&midback_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&temp4_) -
      reinterpret_cast<char*>(&midback_)) + sizeof(temp4_));
  _cached_size_ = 0;
}

LatcontrolDebug::~LatcontrolDebug() {
  // @@protoc_insertion_point(destructor:LatcontrolDebug_msgs.LatcontrolDebug)
  SharedDtor();
}

void LatcontrolDebug::SharedDtor() {
}

void LatcontrolDebug::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* LatcontrolDebug::descriptor() {
  ::protobuf_LatcontrolDebug_5fmsgs_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_LatcontrolDebug_5fmsgs_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const LatcontrolDebug& LatcontrolDebug::default_instance() {
  ::protobuf_LatcontrolDebug_5fmsgs_2eproto::InitDefaultsLatcontrolDebug();
  return *internal_default_instance();
}

LatcontrolDebug* LatcontrolDebug::New(::google::protobuf::Arena* arena) const {
  LatcontrolDebug* n = new LatcontrolDebug;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void LatcontrolDebug::Clear() {
// @@protoc_insertion_point(message_clear_start:LatcontrolDebug_msgs.LatcontrolDebug)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&midback_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&temp4_) -
      reinterpret_cast<char*>(&midback_)) + sizeof(temp4_));
  _internal_metadata_.Clear();
}

bool LatcontrolDebug::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:LatcontrolDebug_msgs.LatcontrolDebug)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(16383u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // float midback = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(13u /* 13 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &midback_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // float midfront = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(21u /* 21 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &midfront_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // float lat_err_front = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(29u /* 29 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &lat_err_front_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // float heading = 4;
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(37u /* 37 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &heading_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // float leftfront = 5;
      case 5: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(45u /* 45 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &leftfront_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // float rightfront = 6;
      case 6: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(53u /* 53 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &rightfront_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // float leftback = 7;
      case 7: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(61u /* 61 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &leftback_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // float rightback = 8;
      case 8: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(69u /* 69 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &rightback_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // float dist_preview = 9;
      case 9: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(77u /* 77 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &dist_preview_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // float x_previewpoint = 10;
      case 10: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(85u /* 85 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &x_previewpoint_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // float y_previewpoint = 11;
      case 11: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(93u /* 93 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &y_previewpoint_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // float R = 12;
      case 12: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(101u /* 101 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &r_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // float eps_speed = 13;
      case 13: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(109u /* 109 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &eps_speed_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // float temp1 = 14;
      case 14: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(117u /* 117 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &temp1_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // float temp2 = 15;
      case 15: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(125u /* 125 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &temp2_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // float temp3 = 16;
      case 16: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(133u /* 133 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &temp3_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // float temp4 = 17;
      case 17: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(141u /* 141 & 0xFF */)) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &temp4_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormat::SkipField(
              input, tag, _internal_metadata_.mutable_unknown_fields()));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:LatcontrolDebug_msgs.LatcontrolDebug)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:LatcontrolDebug_msgs.LatcontrolDebug)
  return false;
#undef DO_
}

void LatcontrolDebug::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:LatcontrolDebug_msgs.LatcontrolDebug)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // float midback = 1;
  if (this->midback() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(1, this->midback(), output);
  }

  // float midfront = 2;
  if (this->midfront() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(2, this->midfront(), output);
  }

  // float lat_err_front = 3;
  if (this->lat_err_front() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(3, this->lat_err_front(), output);
  }

  // float heading = 4;
  if (this->heading() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(4, this->heading(), output);
  }

  // float leftfront = 5;
  if (this->leftfront() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(5, this->leftfront(), output);
  }

  // float rightfront = 6;
  if (this->rightfront() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(6, this->rightfront(), output);
  }

  // float leftback = 7;
  if (this->leftback() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(7, this->leftback(), output);
  }

  // float rightback = 8;
  if (this->rightback() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(8, this->rightback(), output);
  }

  // float dist_preview = 9;
  if (this->dist_preview() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(9, this->dist_preview(), output);
  }

  // float x_previewpoint = 10;
  if (this->x_previewpoint() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(10, this->x_previewpoint(), output);
  }

  // float y_previewpoint = 11;
  if (this->y_previewpoint() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(11, this->y_previewpoint(), output);
  }

  // float R = 12;
  if (this->r() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(12, this->r(), output);
  }

  // float eps_speed = 13;
  if (this->eps_speed() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(13, this->eps_speed(), output);
  }

  // float temp1 = 14;
  if (this->temp1() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(14, this->temp1(), output);
  }

  // float temp2 = 15;
  if (this->temp2() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(15, this->temp2(), output);
  }

  // float temp3 = 16;
  if (this->temp3() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(16, this->temp3(), output);
  }

  // float temp4 = 17;
  if (this->temp4() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(17, this->temp4(), output);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), output);
  }
  // @@protoc_insertion_point(serialize_end:LatcontrolDebug_msgs.LatcontrolDebug)
}

::google::protobuf::uint8* LatcontrolDebug::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:LatcontrolDebug_msgs.LatcontrolDebug)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // float midback = 1;
  if (this->midback() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(1, this->midback(), target);
  }

  // float midfront = 2;
  if (this->midfront() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(2, this->midfront(), target);
  }

  // float lat_err_front = 3;
  if (this->lat_err_front() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(3, this->lat_err_front(), target);
  }

  // float heading = 4;
  if (this->heading() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(4, this->heading(), target);
  }

  // float leftfront = 5;
  if (this->leftfront() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(5, this->leftfront(), target);
  }

  // float rightfront = 6;
  if (this->rightfront() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(6, this->rightfront(), target);
  }

  // float leftback = 7;
  if (this->leftback() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(7, this->leftback(), target);
  }

  // float rightback = 8;
  if (this->rightback() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(8, this->rightback(), target);
  }

  // float dist_preview = 9;
  if (this->dist_preview() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(9, this->dist_preview(), target);
  }

  // float x_previewpoint = 10;
  if (this->x_previewpoint() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(10, this->x_previewpoint(), target);
  }

  // float y_previewpoint = 11;
  if (this->y_previewpoint() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(11, this->y_previewpoint(), target);
  }

  // float R = 12;
  if (this->r() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(12, this->r(), target);
  }

  // float eps_speed = 13;
  if (this->eps_speed() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(13, this->eps_speed(), target);
  }

  // float temp1 = 14;
  if (this->temp1() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(14, this->temp1(), target);
  }

  // float temp2 = 15;
  if (this->temp2() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(15, this->temp2(), target);
  }

  // float temp3 = 16;
  if (this->temp3() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(16, this->temp3(), target);
  }

  // float temp4 = 17;
  if (this->temp4() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(17, this->temp4(), target);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:LatcontrolDebug_msgs.LatcontrolDebug)
  return target;
}

size_t LatcontrolDebug::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:LatcontrolDebug_msgs.LatcontrolDebug)
  size_t total_size = 0;

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()));
  }
  // float midback = 1;
  if (this->midback() != 0) {
    total_size += 1 + 4;
  }

  // float midfront = 2;
  if (this->midfront() != 0) {
    total_size += 1 + 4;
  }

  // float lat_err_front = 3;
  if (this->lat_err_front() != 0) {
    total_size += 1 + 4;
  }

  // float heading = 4;
  if (this->heading() != 0) {
    total_size += 1 + 4;
  }

  // float leftfront = 5;
  if (this->leftfront() != 0) {
    total_size += 1 + 4;
  }

  // float rightfront = 6;
  if (this->rightfront() != 0) {
    total_size += 1 + 4;
  }

  // float leftback = 7;
  if (this->leftback() != 0) {
    total_size += 1 + 4;
  }

  // float rightback = 8;
  if (this->rightback() != 0) {
    total_size += 1 + 4;
  }

  // float dist_preview = 9;
  if (this->dist_preview() != 0) {
    total_size += 1 + 4;
  }

  // float x_previewpoint = 10;
  if (this->x_previewpoint() != 0) {
    total_size += 1 + 4;
  }

  // float y_previewpoint = 11;
  if (this->y_previewpoint() != 0) {
    total_size += 1 + 4;
  }

  // float R = 12;
  if (this->r() != 0) {
    total_size += 1 + 4;
  }

  // float eps_speed = 13;
  if (this->eps_speed() != 0) {
    total_size += 1 + 4;
  }

  // float temp1 = 14;
  if (this->temp1() != 0) {
    total_size += 1 + 4;
  }

  // float temp2 = 15;
  if (this->temp2() != 0) {
    total_size += 1 + 4;
  }

  // float temp3 = 16;
  if (this->temp3() != 0) {
    total_size += 2 + 4;
  }

  // float temp4 = 17;
  if (this->temp4() != 0) {
    total_size += 2 + 4;
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void LatcontrolDebug::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:LatcontrolDebug_msgs.LatcontrolDebug)
  GOOGLE_DCHECK_NE(&from, this);
  const LatcontrolDebug* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const LatcontrolDebug>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:LatcontrolDebug_msgs.LatcontrolDebug)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:LatcontrolDebug_msgs.LatcontrolDebug)
    MergeFrom(*source);
  }
}

void LatcontrolDebug::MergeFrom(const LatcontrolDebug& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:LatcontrolDebug_msgs.LatcontrolDebug)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.midback() != 0) {
    set_midback(from.midback());
  }
  if (from.midfront() != 0) {
    set_midfront(from.midfront());
  }
  if (from.lat_err_front() != 0) {
    set_lat_err_front(from.lat_err_front());
  }
  if (from.heading() != 0) {
    set_heading(from.heading());
  }
  if (from.leftfront() != 0) {
    set_leftfront(from.leftfront());
  }
  if (from.rightfront() != 0) {
    set_rightfront(from.rightfront());
  }
  if (from.leftback() != 0) {
    set_leftback(from.leftback());
  }
  if (from.rightback() != 0) {
    set_rightback(from.rightback());
  }
  if (from.dist_preview() != 0) {
    set_dist_preview(from.dist_preview());
  }
  if (from.x_previewpoint() != 0) {
    set_x_previewpoint(from.x_previewpoint());
  }
  if (from.y_previewpoint() != 0) {
    set_y_previewpoint(from.y_previewpoint());
  }
  if (from.r() != 0) {
    set_r(from.r());
  }
  if (from.eps_speed() != 0) {
    set_eps_speed(from.eps_speed());
  }
  if (from.temp1() != 0) {
    set_temp1(from.temp1());
  }
  if (from.temp2() != 0) {
    set_temp2(from.temp2());
  }
  if (from.temp3() != 0) {
    set_temp3(from.temp3());
  }
  if (from.temp4() != 0) {
    set_temp4(from.temp4());
  }
}

void LatcontrolDebug::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:LatcontrolDebug_msgs.LatcontrolDebug)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void LatcontrolDebug::CopyFrom(const LatcontrolDebug& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:LatcontrolDebug_msgs.LatcontrolDebug)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool LatcontrolDebug::IsInitialized() const {
  return true;
}

void LatcontrolDebug::Swap(LatcontrolDebug* other) {
  if (other == this) return;
  InternalSwap(other);
}
void LatcontrolDebug::InternalSwap(LatcontrolDebug* other) {
  using std::swap;
  swap(midback_, other->midback_);
  swap(midfront_, other->midfront_);
  swap(lat_err_front_, other->lat_err_front_);
  swap(heading_, other->heading_);
  swap(leftfront_, other->leftfront_);
  swap(rightfront_, other->rightfront_);
  swap(leftback_, other->leftback_);
  swap(rightback_, other->rightback_);
  swap(dist_preview_, other->dist_preview_);
  swap(x_previewpoint_, other->x_previewpoint_);
  swap(y_previewpoint_, other->y_previewpoint_);
  swap(r_, other->r_);
  swap(eps_speed_, other->eps_speed_);
  swap(temp1_, other->temp1_);
  swap(temp2_, other->temp2_);
  swap(temp3_, other->temp3_);
  swap(temp4_, other->temp4_);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata LatcontrolDebug::GetMetadata() const {
  protobuf_LatcontrolDebug_5fmsgs_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_LatcontrolDebug_5fmsgs_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace LatcontrolDebug_msgs

// @@protoc_insertion_point(global_scope)
