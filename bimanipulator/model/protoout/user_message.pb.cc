// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: user_message.proto

#include "user_message.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
extern PROTOBUF_INTERNAL_EXPORT_user_5fmessage_2eproto ::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_JointState_user_5fmessage_2eproto;
namespace user_messages {
namespace msgs {
class JointStateDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<JointState> _instance;
} _JointState_default_instance_;
class JointStateArrayDefaultTypeInternal {
 public:
  ::PROTOBUF_NAMESPACE_ID::internal::ExplicitlyConstructed<JointStateArray> _instance;
} _JointStateArray_default_instance_;
}  // namespace msgs
}  // namespace user_messages
static void InitDefaultsscc_info_JointState_user_5fmessage_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::user_messages::msgs::_JointState_default_instance_;
    new (ptr) ::user_messages::msgs::JointState();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::user_messages::msgs::JointState::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<0> scc_info_JointState_user_5fmessage_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 0, 0, InitDefaultsscc_info_JointState_user_5fmessage_2eproto}, {}};

static void InitDefaultsscc_info_JointStateArray_user_5fmessage_2eproto() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  {
    void* ptr = &::user_messages::msgs::_JointStateArray_default_instance_;
    new (ptr) ::user_messages::msgs::JointStateArray();
    ::PROTOBUF_NAMESPACE_ID::internal::OnShutdownDestroyMessage(ptr);
  }
  ::user_messages::msgs::JointStateArray::InitAsDefaultInstance();
}

::PROTOBUF_NAMESPACE_ID::internal::SCCInfo<1> scc_info_JointStateArray_user_5fmessage_2eproto =
    {{ATOMIC_VAR_INIT(::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase::kUninitialized), 1, 0, InitDefaultsscc_info_JointStateArray_user_5fmessage_2eproto}, {
      &scc_info_JointState_user_5fmessage_2eproto.base,}};

static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_user_5fmessage_2eproto[2];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_user_5fmessage_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_user_5fmessage_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_user_5fmessage_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::user_messages::msgs::JointState, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::user_messages::msgs::JointState, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::user_messages::msgs::JointState, coord_),
  PROTOBUF_FIELD_OFFSET(::user_messages::msgs::JointState, name_),
  ~0u,
  0,
  PROTOBUF_FIELD_OFFSET(::user_messages::msgs::JointStateArray, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::user_messages::msgs::JointStateArray, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::user_messages::msgs::JointStateArray, name_),
  PROTOBUF_FIELD_OFFSET(::user_messages::msgs::JointStateArray, state_),
  0,
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 7, sizeof(::user_messages::msgs::JointState)},
  { 9, 16, sizeof(::user_messages::msgs::JointStateArray)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::user_messages::msgs::_JointState_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::user_messages::msgs::_JointStateArray_default_instance_),
};

const char descriptor_table_protodef_user_5fmessage_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\022user_message.proto\022\022user_messages.msgs"
  "\")\n\nJointState\022\r\n\005coord\030\001 \003(\001\022\014\n\004name\030\002 "
  "\002(\t\"N\n\017JointStateArray\022\014\n\004name\030\001 \002(\t\022-\n\005"
  "state\030\002 \003(\0132\036.user_messages.msgs.JointSt"
  "ate"
  ;
static const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable*const descriptor_table_user_5fmessage_2eproto_deps[1] = {
};
static ::PROTOBUF_NAMESPACE_ID::internal::SCCInfoBase*const descriptor_table_user_5fmessage_2eproto_sccs[2] = {
  &scc_info_JointState_user_5fmessage_2eproto.base,
  &scc_info_JointStateArray_user_5fmessage_2eproto.base,
};
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_user_5fmessage_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_user_5fmessage_2eproto = {
  false, false, descriptor_table_protodef_user_5fmessage_2eproto, "user_message.proto", 163,
  &descriptor_table_user_5fmessage_2eproto_once, descriptor_table_user_5fmessage_2eproto_sccs, descriptor_table_user_5fmessage_2eproto_deps, 2, 0,
  schemas, file_default_instances, TableStruct_user_5fmessage_2eproto::offsets,
  file_level_metadata_user_5fmessage_2eproto, 2, file_level_enum_descriptors_user_5fmessage_2eproto, file_level_service_descriptors_user_5fmessage_2eproto,
};

// Force running AddDescriptors() at dynamic initialization time.
static bool dynamic_init_dummy_user_5fmessage_2eproto = (static_cast<void>(::PROTOBUF_NAMESPACE_ID::internal::AddDescriptors(&descriptor_table_user_5fmessage_2eproto)), true);
namespace user_messages {
namespace msgs {

// ===================================================================

void JointState::InitAsDefaultInstance() {
}
class JointState::_Internal {
 public:
  using HasBits = decltype(std::declval<JointState>()._has_bits_);
  static void set_has_name(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static bool MissingRequiredFields(const HasBits& has_bits) {
    return ((has_bits[0] & 0x00000001) ^ 0x00000001) != 0;
  }
};

JointState::JointState(::PROTOBUF_NAMESPACE_ID::Arena* arena)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena),
  coord_(arena) {
  SharedCtor();
  RegisterArenaDtor(arena);
  // @@protoc_insertion_point(arena_constructor:user_messages.msgs.JointState)
}
JointState::JointState(const JointState& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_),
      coord_(from.coord_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  name_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (from._internal_has_name()) {
    name_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), from._internal_name(),
      GetArena());
  }
  // @@protoc_insertion_point(copy_constructor:user_messages.msgs.JointState)
}

void JointState::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_JointState_user_5fmessage_2eproto.base);
  name_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}

JointState::~JointState() {
  // @@protoc_insertion_point(destructor:user_messages.msgs.JointState)
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

void JointState::SharedDtor() {
  GOOGLE_DCHECK(GetArena() == nullptr);
  name_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}

void JointState::ArenaDtor(void* object) {
  JointState* _this = reinterpret_cast< JointState* >(object);
  (void)_this;
}
void JointState::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void JointState::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const JointState& JointState::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_JointState_user_5fmessage_2eproto.base);
  return *internal_default_instance();
}


void JointState::Clear() {
// @@protoc_insertion_point(message_clear_start:user_messages.msgs.JointState)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  coord_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    name_.ClearNonDefaultToEmpty();
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* JointState::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  ::PROTOBUF_NAMESPACE_ID::Arena* arena = GetArena(); (void)arena;
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // repeated double coord = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 9)) {
          ptr -= 1;
          do {
            ptr += 1;
            _internal_add_coord(::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr));
            ptr += sizeof(double);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<9>(ptr));
        } else if (static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10) {
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::PackedDoubleParser(_internal_mutable_coord(), ptr, ctx);
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // required string name = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          auto str = _internal_mutable_name();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          #ifndef NDEBUG
          ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "user_messages.msgs.JointState.name");
          #endif  // !NDEBUG
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag,
            _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
            ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  _has_bits_.Or(has_bits);
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* JointState::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:user_messages.msgs.JointState)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // repeated double coord = 1;
  for (int i = 0, n = this->_internal_coord_size(); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(1, this->_internal_coord(i), target);
  }

  cached_has_bits = _has_bits_[0];
  // required string name = 2;
  if (cached_has_bits & 0x00000001u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_name().data(), static_cast<int>(this->_internal_name().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "user_messages.msgs.JointState.name");
    target = stream->WriteStringMaybeAliased(
        2, this->_internal_name(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:user_messages.msgs.JointState)
  return target;
}

size_t JointState::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:user_messages.msgs.JointState)
  size_t total_size = 0;

  // required string name = 2;
  if (_internal_has_name()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
        this->_internal_name());
  }
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated double coord = 1;
  {
    unsigned int count = static_cast<unsigned int>(this->_internal_coord_size());
    size_t data_size = 8UL * count;
    total_size += 1 *
                  ::PROTOBUF_NAMESPACE_ID::internal::FromIntSize(this->_internal_coord_size());
    total_size += data_size;
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void JointState::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:user_messages.msgs.JointState)
  GOOGLE_DCHECK_NE(&from, this);
  const JointState* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<JointState>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:user_messages.msgs.JointState)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:user_messages.msgs.JointState)
    MergeFrom(*source);
  }
}

void JointState::MergeFrom(const JointState& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:user_messages.msgs.JointState)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  coord_.MergeFrom(from.coord_);
  if (from._internal_has_name()) {
    _internal_set_name(from._internal_name());
  }
}

void JointState::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:user_messages.msgs.JointState)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void JointState::CopyFrom(const JointState& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:user_messages.msgs.JointState)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool JointState::IsInitialized() const {
  if (_Internal::MissingRequiredFields(_has_bits_)) return false;
  return true;
}

void JointState::InternalSwap(JointState* other) {
  using std::swap;
  _internal_metadata_.Swap<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  coord_.InternalSwap(&other->coord_);
  name_.Swap(&other->name_, &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArena());
}

::PROTOBUF_NAMESPACE_ID::Metadata JointState::GetMetadata() const {
  return GetMetadataStatic();
}


// ===================================================================

void JointStateArray::InitAsDefaultInstance() {
}
class JointStateArray::_Internal {
 public:
  using HasBits = decltype(std::declval<JointStateArray>()._has_bits_);
  static void set_has_name(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
  static bool MissingRequiredFields(const HasBits& has_bits) {
    return ((has_bits[0] & 0x00000001) ^ 0x00000001) != 0;
  }
};

JointStateArray::JointStateArray(::PROTOBUF_NAMESPACE_ID::Arena* arena)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena),
  state_(arena) {
  SharedCtor();
  RegisterArenaDtor(arena);
  // @@protoc_insertion_point(arena_constructor:user_messages.msgs.JointStateArray)
}
JointStateArray::JointStateArray(const JointStateArray& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_),
      state_(from.state_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  name_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  if (from._internal_has_name()) {
    name_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), from._internal_name(),
      GetArena());
  }
  // @@protoc_insertion_point(copy_constructor:user_messages.msgs.JointStateArray)
}

void JointStateArray::SharedCtor() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&scc_info_JointStateArray_user_5fmessage_2eproto.base);
  name_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}

JointStateArray::~JointStateArray() {
  // @@protoc_insertion_point(destructor:user_messages.msgs.JointStateArray)
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

void JointStateArray::SharedDtor() {
  GOOGLE_DCHECK(GetArena() == nullptr);
  name_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}

void JointStateArray::ArenaDtor(void* object) {
  JointStateArray* _this = reinterpret_cast< JointStateArray* >(object);
  (void)_this;
}
void JointStateArray::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void JointStateArray::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}
const JointStateArray& JointStateArray::default_instance() {
  ::PROTOBUF_NAMESPACE_ID::internal::InitSCC(&::scc_info_JointStateArray_user_5fmessage_2eproto.base);
  return *internal_default_instance();
}


void JointStateArray::Clear() {
// @@protoc_insertion_point(message_clear_start:user_messages.msgs.JointStateArray)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  state_.Clear();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    name_.ClearNonDefaultToEmpty();
  }
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* JointStateArray::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  ::PROTOBUF_NAMESPACE_ID::Arena* arena = GetArena(); (void)arena;
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    CHK_(ptr);
    switch (tag >> 3) {
      // required string name = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 10)) {
          auto str = _internal_mutable_name();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          #ifndef NDEBUG
          ::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "user_messages.msgs.JointStateArray.name");
          #endif  // !NDEBUG
          CHK_(ptr);
        } else goto handle_unusual;
        continue;
      // repeated .user_messages.msgs.JointState state = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 18)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_state(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<18>(ptr));
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag & 7) == 4 || tag == 0) {
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag,
            _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
            ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  _has_bits_.Or(has_bits);
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* JointStateArray::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:user_messages.msgs.JointStateArray)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  cached_has_bits = _has_bits_[0];
  // required string name = 1;
  if (cached_has_bits & 0x00000001u) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::VerifyUTF8StringNamedField(
      this->_internal_name().data(), static_cast<int>(this->_internal_name().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::SERIALIZE,
      "user_messages.msgs.JointStateArray.name");
    target = stream->WriteStringMaybeAliased(
        1, this->_internal_name(), target);
  }

  // repeated .user_messages.msgs.JointState state = 2;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_state_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(2, this->_internal_state(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:user_messages.msgs.JointStateArray)
  return target;
}

size_t JointStateArray::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:user_messages.msgs.JointStateArray)
  size_t total_size = 0;

  // required string name = 1;
  if (_internal_has_name()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
        this->_internal_name());
  }
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .user_messages.msgs.JointState state = 2;
  total_size += 1UL * this->_internal_state_size();
  for (const auto& msg : this->state_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

void JointStateArray::MergeFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:user_messages.msgs.JointStateArray)
  GOOGLE_DCHECK_NE(&from, this);
  const JointStateArray* source =
      ::PROTOBUF_NAMESPACE_ID::DynamicCastToGenerated<JointStateArray>(
          &from);
  if (source == nullptr) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:user_messages.msgs.JointStateArray)
    ::PROTOBUF_NAMESPACE_ID::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:user_messages.msgs.JointStateArray)
    MergeFrom(*source);
  }
}

void JointStateArray::MergeFrom(const JointStateArray& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:user_messages.msgs.JointStateArray)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  state_.MergeFrom(from.state_);
  if (from._internal_has_name()) {
    _internal_set_name(from._internal_name());
  }
}

void JointStateArray::CopyFrom(const ::PROTOBUF_NAMESPACE_ID::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:user_messages.msgs.JointStateArray)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void JointStateArray::CopyFrom(const JointStateArray& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:user_messages.msgs.JointStateArray)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool JointStateArray::IsInitialized() const {
  if (_Internal::MissingRequiredFields(_has_bits_)) return false;
  if (!::PROTOBUF_NAMESPACE_ID::internal::AllAreInitialized(state_)) return false;
  return true;
}

void JointStateArray::InternalSwap(JointStateArray* other) {
  using std::swap;
  _internal_metadata_.Swap<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  state_.InternalSwap(&other->state_);
  name_.Swap(&other->name_, &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArena());
}

::PROTOBUF_NAMESPACE_ID::Metadata JointStateArray::GetMetadata() const {
  return GetMetadataStatic();
}


// @@protoc_insertion_point(namespace_scope)
}  // namespace msgs
}  // namespace user_messages
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::user_messages::msgs::JointState* Arena::CreateMaybeMessage< ::user_messages::msgs::JointState >(Arena* arena) {
  return Arena::CreateMessageInternal< ::user_messages::msgs::JointState >(arena);
}
template<> PROTOBUF_NOINLINE ::user_messages::msgs::JointStateArray* Arena::CreateMaybeMessage< ::user_messages::msgs::JointStateArray >(Arena* arena) {
  return Arena::CreateMessageInternal< ::user_messages::msgs::JointStateArray >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
