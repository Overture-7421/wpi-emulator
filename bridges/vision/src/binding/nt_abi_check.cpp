#include "nt_binder.h"

// The one translation unit of the vision bridge that sees the real ntcore C headers (staged at configure time from the
// ntcore-cpp / wpiutil-cpp "headers" artifacts, CMake option WPIEMU_VISION_HAS_NTCORE). It contains no code that runs:
// it proves, at compile time, that the declarations the binding is written against (nt_abi.h, nt_functions.h) are the
// ABI of those headers. The first WPILib release that changes a layout or a signature fails here, loudly, instead of
// corrupting memory in the robot process.

#ifdef WPIEMU_VISION_HAS_NTCORE

#include <ntcore_c.h>
#include <ntcore_c_types.h>
#include <wpi/string.h>

#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

namespace wpiemu::bridges::vision::binding {

namespace {

// ---- ABI facts the binding relies on -----------------------------------------------------------------------------
static_assert(std::is_same_v<NT_Bool, int>, "NT_Bool is an int, not a bool");
static_assert(std::is_same_v<NT_Handle, unsigned int> && std::is_same_v<NT_Inst, NT_Handle> &&
                  std::is_same_v<NT_Topic, NT_Handle> && std::is_same_v<NT_Publisher, NT_Handle> &&
                  std::is_same_v<NT_Subscriber, NT_Handle>,
              "ntcore handles are unsigned ints");
#if INTPTR_MAX == INT64_MAX
static_assert(sizeof(WPI_String) == 16 && sizeof(NT_Value) == 40 && sizeof(NT_TopicInfo) == 64 &&
                  sizeof(NT_PubSubOptions) == 56,
              "the x64 layout of the ntcore structs changed");
#endif

// ---- our declarations have the layout of the real ones -------------------------------------------------------------
template <class Mine, class Real>
constexpr bool sameSize() {
    return sizeof(Mine) == sizeof(Real) && alignof(Mine) == alignof(Real);
}

#define WPIEMU_SAME_FIELD(Mine, Real, mine, real)                                                               \
    static_assert(offsetof(Mine, mine) == offsetof(Real, real), #Mine "::" #mine " moved");                      \
    static_assert(sizeof(Mine::mine) == sizeof(Real::real), #Mine "::" #mine " changed size");

static_assert(sameSize<abi::String, WPI_String>());
WPIEMU_SAME_FIELD(abi::String, WPI_String, str, str)
WPIEMU_SAME_FIELD(abi::String, WPI_String, len, len)

static_assert(sizeof(abi::Type) == sizeof(NT_Type), "NT_Type changed size");
#define WPIEMU_SAME_TYPE(mine, real)                                                    \
    static_assert(static_cast<unsigned int>(abi::Type::mine) == static_cast<unsigned int>(real), \
                  "NT_Type " #real " changed");
WPIEMU_SAME_TYPE(Unassigned, NT_UNASSIGNED)
WPIEMU_SAME_TYPE(Boolean, NT_BOOLEAN)
WPIEMU_SAME_TYPE(Double, NT_DOUBLE)
WPIEMU_SAME_TYPE(String, NT_STRING)
WPIEMU_SAME_TYPE(Raw, NT_RAW)
WPIEMU_SAME_TYPE(BooleanArray, NT_BOOLEAN_ARRAY)
WPIEMU_SAME_TYPE(DoubleArray, NT_DOUBLE_ARRAY)
WPIEMU_SAME_TYPE(StringArray, NT_STRING_ARRAY)
WPIEMU_SAME_TYPE(Rpc, NT_RPC)
WPIEMU_SAME_TYPE(Integer, NT_INTEGER)
WPIEMU_SAME_TYPE(Float, NT_FLOAT)
WPIEMU_SAME_TYPE(IntegerArray, NT_INTEGER_ARRAY)
WPIEMU_SAME_TYPE(FloatArray, NT_FLOAT_ARRAY)
#undef WPIEMU_SAME_TYPE

static_assert(sameSize<abi::Value, NT_Value>());
WPIEMU_SAME_FIELD(abi::Value, NT_Value, type, type)
WPIEMU_SAME_FIELD(abi::Value, NT_Value, lastChange, last_change)
WPIEMU_SAME_FIELD(abi::Value, NT_Value, serverTime, server_time)
WPIEMU_SAME_FIELD(abi::Value, NT_Value, data, data)

// The members of the value union (all of them live at offset 0), and the {pointer, size} pairs inside them.
#define WPIEMU_SAME_UNION_MEMBER(mine, real) \
    static_assert(sizeof(abi::Value::Data::mine) == sizeof(std::declval<NT_Value>().data.real), \
                  "NT_Value::data." #real " changed size");
WPIEMU_SAME_UNION_MEMBER(boolean, v_boolean)
WPIEMU_SAME_UNION_MEMBER(integer, v_int)
WPIEMU_SAME_UNION_MEMBER(float32, v_float)
WPIEMU_SAME_UNION_MEMBER(float64, v_double)
WPIEMU_SAME_UNION_MEMBER(string, v_string)
WPIEMU_SAME_UNION_MEMBER(raw, v_raw)
WPIEMU_SAME_UNION_MEMBER(booleans, arr_boolean)
WPIEMU_SAME_UNION_MEMBER(doubles, arr_double)
WPIEMU_SAME_UNION_MEMBER(floats, arr_float)
WPIEMU_SAME_UNION_MEMBER(integers, arr_int)
WPIEMU_SAME_UNION_MEMBER(strings, arr_string)
#undef WPIEMU_SAME_UNION_MEMBER

#define WPIEMU_SAME_PAIR(mine, pointerField, real) \
    static_assert(offsetof(decltype(abi::Value::Data::mine), pointerField) == \
                          offsetof(decltype(std::declval<NT_Value>().data.real), pointerField) && \
                      offsetof(decltype(abi::Value::Data::mine), size) == \
                          offsetof(decltype(std::declval<NT_Value>().data.real), size), \
                  "NT_Value::data." #real " changed layout");
WPIEMU_SAME_PAIR(raw, data, v_raw)
WPIEMU_SAME_PAIR(booleans, arr, arr_boolean)
WPIEMU_SAME_PAIR(doubles, arr, arr_double)
WPIEMU_SAME_PAIR(floats, arr, arr_float)
WPIEMU_SAME_PAIR(integers, arr, arr_int)
WPIEMU_SAME_PAIR(strings, arr, arr_string)
#undef WPIEMU_SAME_PAIR

// The payload types we read (a pointer to the same element type, so that what we read is what ntcore wrote).
static_assert(std::is_same_v<decltype(std::declval<NT_Value>().data.v_boolean), NT_Bool> &&
                  std::is_same_v<decltype(std::declval<NT_Value>().data.v_int), std::int64_t> &&
                  std::is_same_v<decltype(std::declval<NT_Value>().data.v_float), float> &&
                  std::is_same_v<decltype(std::declval<NT_Value>().data.v_double), double> &&
                  std::is_same_v<decltype(std::declval<NT_Value>().data.v_raw.data), std::uint8_t*> &&
                  std::is_same_v<decltype(std::declval<NT_Value>().data.arr_double.arr), double*> &&
                  std::is_same_v<decltype(std::declval<NT_Value>().data.arr_float.arr), float*> &&
                  std::is_same_v<decltype(std::declval<NT_Value>().data.arr_int.arr), std::int64_t*> &&
                  std::is_same_v<decltype(std::declval<NT_Value>().data.arr_boolean.arr), NT_Bool*> &&
                  std::is_same_v<decltype(std::declval<NT_Value>().data.v_string), WPI_String>,
              "the payload members of NT_Value changed type");

static_assert(sameSize<abi::TopicInfo, NT_TopicInfo>());
WPIEMU_SAME_FIELD(abi::TopicInfo, NT_TopicInfo, topic, topic)
WPIEMU_SAME_FIELD(abi::TopicInfo, NT_TopicInfo, name, name)
WPIEMU_SAME_FIELD(abi::TopicInfo, NT_TopicInfo, type, type)
WPIEMU_SAME_FIELD(abi::TopicInfo, NT_TopicInfo, typeStr, type_str)
WPIEMU_SAME_FIELD(abi::TopicInfo, NT_TopicInfo, properties, properties)

static_assert(sameSize<abi::PubSubOptions, NT_PubSubOptions>());
WPIEMU_SAME_FIELD(abi::PubSubOptions, NT_PubSubOptions, structSize, structSize)
WPIEMU_SAME_FIELD(abi::PubSubOptions, NT_PubSubOptions, pollStorage, pollStorage)
WPIEMU_SAME_FIELD(abi::PubSubOptions, NT_PubSubOptions, periodic, periodic)
WPIEMU_SAME_FIELD(abi::PubSubOptions, NT_PubSubOptions, excludePublisher, excludePublisher)
WPIEMU_SAME_FIELD(abi::PubSubOptions, NT_PubSubOptions, sendAll, sendAll)
WPIEMU_SAME_FIELD(abi::PubSubOptions, NT_PubSubOptions, topicsOnly, topicsOnly)
WPIEMU_SAME_FIELD(abi::PubSubOptions, NT_PubSubOptions, prefixMatch, prefixMatch)
WPIEMU_SAME_FIELD(abi::PubSubOptions, NT_PubSubOptions, keepDuplicates, keepDuplicates)
WPIEMU_SAME_FIELD(abi::PubSubOptions, NT_PubSubOptions, disableRemote, disableRemote)
WPIEMU_SAME_FIELD(abi::PubSubOptions, NT_PubSubOptions, disableLocal, disableLocal)
WPIEMU_SAME_FIELD(abi::PubSubOptions, NT_PubSubOptions, excludeSelf, excludeSelf)
WPIEMU_SAME_FIELD(abi::PubSubOptions, NT_PubSubOptions, hidden, hidden)
#undef WPIEMU_SAME_FIELD

// ---- every entry of the table has the signature of the exported C function of the same name ----------------------
// ToReal rewrites one of our function types into the ntcore types, so that std::is_same_v compares exact C types.
template <class T>
struct ToReal {
    using type = T;
};
template <>
struct ToReal<abi::String> {
    using type = WPI_String;
};
template <>
struct ToReal<abi::Value> {
    using type = NT_Value;
};
template <>
struct ToReal<abi::TopicInfo> {
    using type = NT_TopicInfo;
};
template <>
struct ToReal<abi::PubSubOptions> {
    using type = NT_PubSubOptions;
};
template <>
struct ToReal<abi::Type> {
    using type = NT_Type;
};
template <class T>
struct ToReal<T*> {
    using type = typename ToReal<T>::type*;
};
template <class T>
struct ToReal<const T> {
    using type = const typename ToReal<T>::type;
};
template <class Result, class... Args>
struct ToReal<Result(Args...)> {
    using type = typename ToReal<Result>::type(typename ToReal<Args>::type...);
};

#define WPIEMU_NT_SIGNATURE(name, result, parameters)                                                              \
    static_assert(std::is_same_v<ToReal<std::remove_pointer_t<decltype(NtFunctions::name)>>::type, decltype(::name)>, \
                  #name ": the signature differs from the one in the ntcore header");
WPIEMU_NT_FUNCTIONS(WPIEMU_NT_SIGNATURE)
#undef WPIEMU_NT_SIGNATURE

}  // namespace

bool abiVerifiedAgainstHeaders() { return true; }

}  // namespace wpiemu::bridges::vision::binding

#else  // !WPIEMU_VISION_HAS_NTCORE

namespace wpiemu::bridges::vision::binding {

bool abiVerifiedAgainstHeaders() { return false; }

}  // namespace wpiemu::bridges::vision::binding

#endif
