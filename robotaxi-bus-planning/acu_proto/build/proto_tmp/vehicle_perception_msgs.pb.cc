// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: vehicle_perception_msgs.proto

#include "vehicle_perception_msgs.pb.h"

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
namespace vehicle_perception_msgs {
class VehiclePerceptionDefaultTypeInternal {
 public:
  ::google::protobuf::internal::ExplicitlyConstructed<VehiclePerception>
      _instance;
} _VehiclePerception_default_instance_;
}  // namespace vehicle_perception_msgs
namespace protobuf_vehicle_5fperception_5fmsgs_2eproto {
void InitDefaultsVehiclePerceptionImpl() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

#ifdef GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
  ::google::protobuf::internal::InitProtobufDefaultsForceUnique();
#else
  ::google::protobuf::internal::InitProtobufDefaults();
#endif  // GOOGLE_PROTOBUF_ENFORCE_UNIQUENESS
  protobuf_common_5fmsgs_2eproto::InitDefaultsHeader();
  protobuf_perception_5fobjects_5fmsgs_2eproto::InitDefaultsPerceptionObjects();
  protobuf_perception_5fenvironment_5fmsgs_2eproto::InitDefaultsPerceptionEnvironment();
  protobuf_sensor_5fimu_5fmsgs_2eproto::InitDefaultsSensorImu();
  {
    void* ptr = &::vehicle_perception_msgs::_VehiclePerception_default_instance_;
    new (ptr) ::vehicle_perception_msgs::VehiclePerception();
    ::google::protobuf::internal::OnShutdownDestroyMessage(ptr);
  }
  ::vehicle_perception_msgs::VehiclePerception::InitAsDefaultInstance();
}

void InitDefaultsVehiclePerception() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &InitDefaultsVehiclePerceptionImpl);
}

::google::protobuf::Metadata file_level_metadata[1];

const ::google::protobuf::uint32 TableStruct::offsets[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::vehicle_perception_msgs::VehiclePerception, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::vehicle_perception_msgs::VehiclePerception, header_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::vehicle_perception_msgs::VehiclePerception, obstacle_info_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::vehicle_perception_msgs::VehiclePerception, environment_info_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(::vehicle_perception_msgs::VehiclePerception, localizaion_),
};
static const ::google::protobuf::internal::MigrationSchema schemas[] GOOGLE_PROTOBUF_ATTRIBUTE_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::vehicle_perception_msgs::VehiclePerception)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&::vehicle_perception_msgs::_VehiclePerception_default_instance_),
};

void protobuf_AssignDescriptors() {
  AddDescriptors();
  ::google::protobuf::MessageFactory* factory = NULL;
  AssignDescriptors(
      "vehicle_perception_msgs.proto", schemas, file_default_instances, TableStruct::offsets, factory,
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
      "\n\035vehicle_perception_msgs.proto\022\027vehicle"
      "_perception_msgs\032\021common_msgs.proto\032\035per"
      "ception_objects_msgs.proto\032!perception_e"
      "nvironment_msgs.proto\032\025sensor_imu_msgs.p"
      "roto\"\306\001\n\021VehiclePerception\022#\n\006header\030\001 \001"
      "(\0132\023.common_msgs.Header\022)\n\robstacle_info"
      "\030\002 \001(\0132\022.PerceptionObjects\0220\n\020environmen"
      "t_info\030\003 \001(\0132\026.PerceptionEnvironment\022/\n\013"
      "localizaion\030\004 \001(\0132\032.sensor_imu_msgs.Sens"
      "orImub\006proto3"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 373);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "vehicle_perception_msgs.proto", &protobuf_RegisterTypes);
  ::protobuf_common_5fmsgs_2eproto::AddDescriptors();
  ::protobuf_perception_5fobjects_5fmsgs_2eproto::AddDescriptors();
  ::protobuf_perception_5fenvironment_5fmsgs_2eproto::AddDescriptors();
  ::protobuf_sensor_5fimu_5fmsgs_2eproto::AddDescriptors();
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
}  // namespace protobuf_vehicle_5fperception_5fmsgs_2eproto
namespace vehicle_perception_msgs {

// ===================================================================

void VehiclePerception::InitAsDefaultInstance() {
  ::vehicle_perception_msgs::_VehiclePerception_default_instance_._instance.get_mutable()->header_ = const_cast< ::common_msgs::Header*>(
      ::common_msgs::Header::internal_default_instance());
  ::vehicle_perception_msgs::_VehiclePerception_default_instance_._instance.get_mutable()->obstacle_info_ = const_cast< ::PerceptionObjects*>(
      ::PerceptionObjects::internal_default_instance());
  ::vehicle_perception_msgs::_VehiclePerception_default_instance_._instance.get_mutable()->environment_info_ = const_cast< ::PerceptionEnvironment*>(
      ::PerceptionEnvironment::internal_default_instance());
  ::vehicle_perception_msgs::_VehiclePerception_default_instance_._instance.get_mutable()->localizaion_ = const_cast< ::sensor_imu_msgs::SensorImu*>(
      ::sensor_imu_msgs::SensorImu::internal_default_instance());
}
void VehiclePerception::clear_header() {
  if (GetArenaNoVirtual() == NULL && header_ != NULL) {
    delete header_;
  }
  header_ = NULL;
}
void VehiclePerception::clear_obstacle_info() {
  if (GetArenaNoVirtual() == NULL && obstacle_info_ != NULL) {
    delete obstacle_info_;
  }
  obstacle_info_ = NULL;
}
void VehiclePerception::clear_environment_info() {
  if (GetArenaNoVirtual() == NULL && environment_info_ != NULL) {
    delete environment_info_;
  }
  environment_info_ = NULL;
}
void VehiclePerception::clear_localizaion() {
  if (GetArenaNoVirtual() == NULL && localizaion_ != NULL) {
    delete localizaion_;
  }
  localizaion_ = NULL;
}
#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int VehiclePerception::kHeaderFieldNumber;
const int VehiclePerception::kObstacleInfoFieldNumber;
const int VehiclePerception::kEnvironmentInfoFieldNumber;
const int VehiclePerception::kLocalizaionFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

VehiclePerception::VehiclePerception()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    ::protobuf_vehicle_5fperception_5fmsgs_2eproto::InitDefaultsVehiclePerception();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:vehicle_perception_msgs.VehiclePerception)
}
VehiclePerception::VehiclePerception(const VehiclePerception& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _cached_size_(0) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from.has_header()) {
    header_ = new ::common_msgs::Header(*from.header_);
  } else {
    header_ = NULL;
  }
  if (from.has_obstacle_info()) {
    obstacle_info_ = new ::PerceptionObjects(*from.obstacle_info_);
  } else {
    obstacle_info_ = NULL;
  }
  if (from.has_environment_info()) {
    environment_info_ = new ::PerceptionEnvironment(*from.environment_info_);
  } else {
    environment_info_ = NULL;
  }
  if (from.has_localizaion()) {
    localizaion_ = new ::sensor_imu_msgs::SensorImu(*from.localizaion_);
  } else {
    localizaion_ = NULL;
  }
  // @@protoc_insertion_point(copy_constructor:vehicle_perception_msgs.VehiclePerception)
}

void VehiclePerception::SharedCtor() {
  ::memset(&header_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&localizaion_) -
      reinterpret_cast<char*>(&header_)) + sizeof(localizaion_));
  _cached_size_ = 0;
}

VehiclePerception::~VehiclePerception() {
  // @@protoc_insertion_point(destructor:vehicle_perception_msgs.VehiclePerception)
  SharedDtor();
}

void VehiclePerception::SharedDtor() {
  if (this != internal_default_instance()) delete header_;
  if (this != internal_default_instance()) delete obstacle_info_;
  if (this != internal_default_instance()) delete environment_info_;
  if (this != internal_default_instance()) delete localizaion_;
}

void VehiclePerception::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* VehiclePerception::descriptor() {
  ::protobuf_vehicle_5fperception_5fmsgs_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_vehicle_5fperception_5fmsgs_2eproto::file_level_metadata[kIndexInFileMessages].descriptor;
}

const VehiclePerception& VehiclePerception::default_instance() {
  ::protobuf_vehicle_5fperception_5fmsgs_2eproto::InitDefaultsVehiclePerception();
  return *internal_default_instance();
}

VehiclePerception* VehiclePerception::New(::google::protobuf::Arena* arena) const {
  VehiclePerception* n = new VehiclePerception;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void VehiclePerception::Clear() {
// @@protoc_insertion_point(message_clear_start:vehicle_perception_msgs.VehiclePerception)
  ::google::protobuf::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  if (GetArenaNoVirtual() == NULL && header_ != NULL) {
    delete header_;
  }
  header_ = NULL;
  if (GetArenaNoVirtual() == NULL && obstacle_info_ != NULL) {
    delete obstacle_info_;
  }
  obstacle_info_ = NULL;
  if (GetArenaNoVirtual() == NULL && environment_info_ != NULL) {
    delete environment_info_;
  }
  environment_info_ = NULL;
  if (GetArenaNoVirtual() == NULL && localizaion_ != NULL) {
    delete localizaion_;
  }
  localizaion_ = NULL;
  _internal_metadata_.Clear();
}

bool VehiclePerception::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:vehicle_perception_msgs.VehiclePerception)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // .common_msgs.Header header = 1;
      case 1: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(10u /* 10 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_header()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // .PerceptionObjects obstacle_info = 2;
      case 2: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(18u /* 18 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_obstacle_info()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // .PerceptionEnvironment environment_info = 3;
      case 3: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(26u /* 26 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_environment_info()));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // .sensor_imu_msgs.SensorImu localizaion = 4;
      case 4: {
        if (static_cast< ::google::protobuf::uint8>(tag) ==
            static_cast< ::google::protobuf::uint8>(34u /* 34 & 0xFF */)) {
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessage(
               input, mutable_localizaion()));
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
  // @@protoc_insertion_point(parse_success:vehicle_perception_msgs.VehiclePerception)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:vehicle_perception_msgs.VehiclePerception)
  return false;
#undef DO_
}

void VehiclePerception::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:vehicle_perception_msgs.VehiclePerception)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // .common_msgs.Header header = 1;
  if (this->has_header()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, *this->header_, output);
  }

  // .PerceptionObjects obstacle_info = 2;
  if (this->has_obstacle_info()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      2, *this->obstacle_info_, output);
  }

  // .PerceptionEnvironment environment_info = 3;
  if (this->has_environment_info()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      3, *this->environment_info_, output);
  }

  // .sensor_imu_msgs.SensorImu localizaion = 4;
  if (this->has_localizaion()) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      4, *this->localizaion_, output);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    ::google::protobuf::internal::WireFormat::SerializeUnknownFields(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), output);
  }
  // @@protoc_insertion_point(serialize_end:vehicle_perception_msgs.VehiclePerception)
}

::google::protobuf::uint8* VehiclePerception::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic; // Unused
  // @@protoc_insertion_point(serialize_to_array_start:vehicle_perception_msgs.VehiclePerception)
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // .common_msgs.Header header = 1;
  if (this->has_header()) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        1, *this->header_, deterministic, target);
  }

  // .PerceptionObjects obstacle_info = 2;
  if (this->has_obstacle_info()) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        2, *this->obstacle_info_, deterministic, target);
  }

  // .PerceptionEnvironment environment_info = 3;
  if (this->has_environment_info()) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        3, *this->environment_info_, deterministic, target);
  }

  // .sensor_imu_msgs.SensorImu localizaion = 4;
  if (this->has_localizaion()) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageToArray(
        4, *this->localizaion_, deterministic, target);
  }

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    target = ::google::protobuf::internal::WireFormat::SerializeUnknownFieldsToArray(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()), target);
  }
  // @@protoc_insertion_point(serialize_to_array_end:vehicle_perception_msgs.VehiclePerception)
  return target;
}

size_t VehiclePerception::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:vehicle_perception_msgs.VehiclePerception)
  size_t total_size = 0;

  if ((_internal_metadata_.have_unknown_fields() &&  ::google::protobuf::internal::GetProto3PreserveUnknownsDefault())) {
    total_size +=
      ::google::protobuf::internal::WireFormat::ComputeUnknownFieldsSize(
        (::google::protobuf::internal::GetProto3PreserveUnknownsDefault()   ? _internal_metadata_.unknown_fields()   : _internal_metadata_.default_instance()));
  }
  // .common_msgs.Header header = 1;
  if (this->has_header()) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSize(
        *this->header_);
  }

  // .PerceptionObjects obstacle_info = 2;
  if (this->has_obstacle_info()) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSize(
        *this->obstacle_info_);
  }

  // .PerceptionEnvironment environment_info = 3;
  if (this->has_environment_info()) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSize(
        *this->environment_info_);
  }

  // .sensor_imu_msgs.SensorImu localizaion = 4;
  if (this->has_localizaion()) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::MessageSize(
        *this->localizaion_);
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void VehiclePerception::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:vehicle_perception_msgs.VehiclePerception)
  GOOGLE_DCHECK_NE(&from, this);
  const VehiclePerception* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const VehiclePerception>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:vehicle_perception_msgs.VehiclePerception)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:vehicle_perception_msgs.VehiclePerception)
    MergeFrom(*source);
  }
}

void VehiclePerception::MergeFrom(const VehiclePerception& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:vehicle_perception_msgs.VehiclePerception)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::google::protobuf::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (from.has_header()) {
    mutable_header()->::common_msgs::Header::MergeFrom(from.header());
  }
  if (from.has_obstacle_info()) {
    mutable_obstacle_info()->::PerceptionObjects::MergeFrom(from.obstacle_info());
  }
  if (from.has_environment_info()) {
    mutable_environment_info()->::PerceptionEnvironment::MergeFrom(from.environment_info());
  }
  if (from.has_localizaion()) {
    mutable_localizaion()->::sensor_imu_msgs::SensorImu::MergeFrom(from.localizaion());
  }
}

void VehiclePerception::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:vehicle_perception_msgs.VehiclePerception)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void VehiclePerception::CopyFrom(const VehiclePerception& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:vehicle_perception_msgs.VehiclePerception)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool VehiclePerception::IsInitialized() const {
  return true;
}

void VehiclePerception::Swap(VehiclePerception* other) {
  if (other == this) return;
  InternalSwap(other);
}
void VehiclePerception::InternalSwap(VehiclePerception* other) {
  using std::swap;
  swap(header_, other->header_);
  swap(obstacle_info_, other->obstacle_info_);
  swap(environment_info_, other->environment_info_);
  swap(localizaion_, other->localizaion_);
  _internal_metadata_.Swap(&other->_internal_metadata_);
  swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata VehiclePerception::GetMetadata() const {
  protobuf_vehicle_5fperception_5fmsgs_2eproto::protobuf_AssignDescriptorsOnce();
  return ::protobuf_vehicle_5fperception_5fmsgs_2eproto::file_level_metadata[kIndexInFileMessages];
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace vehicle_perception_msgs

// @@protoc_insertion_point(global_scope)
