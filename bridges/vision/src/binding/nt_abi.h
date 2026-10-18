#pragma once

// The part of ntcore's C ABI the binding uses, declared by us (internal to src/binding).
//
// The declarations follow the layout of the official public headers of WPILib 2026.2.1 (ntcore_c.h,
// ntcore_c_types.h, wpi/string.h; BSD licensed, used only as a published C ABI). They are standalone on purpose:
// the marshalling code (NtCoreApi) and its tests build and run on every machine, with or without WPILib. When the
// real headers are staged at configure time (WPIEMU_VISION_HAS_NTCORE), nt_abi_check.cpp proves at compile time that
// every type below has the layout of the real one and every NtFunctions entry has exactly the real signature, so a
// WPILib release that changes the ABI breaks the build instead of corrupting the robot's memory.

#include <cstddef>
#include <cstdint>

namespace wpiemu::bridges::vision::binding::abi {

using Handle = unsigned int;  // NT_Handle / NT_Inst / NT_Topic / NT_Publisher / NT_Subscriber; 0 = invalid
using Bool = int;             // NT_Bool: an int, not a bool

// WPI_String: length delimited, need not be NUL terminated, only has to outlive the call. Strings that ntcore hands
// out are freed by the array that holds them (NT_DisposeTopicInfoArray / NT_DisposeValueArray).
struct String {
    const char* str;
    std::size_t len;
};

enum class Type : unsigned int {  // NT_Type
    Unassigned = 0,
    Boolean = 0x01,
    Double = 0x02,
    String = 0x04,
    Raw = 0x08,
    BooleanArray = 0x10,
    DoubleArray = 0x20,
    StringArray = 0x40,
    Rpc = 0x80,
    Integer = 0x100,
    Float = 0x200,
    IntegerArray = 0x400,
    FloatArray = 0x800,
};

// NT_Value: a typed union. `lastChange` is the timestamp of the value (NT time, microseconds).
struct Value {
    Type type;
    std::int64_t lastChange;
    std::int64_t serverTime;
    union Data {
        Bool boolean;
        std::int64_t integer;
        float float32;
        double float64;
        String string;
        struct {
            std::uint8_t* data;
            std::size_t size;
        } raw;
        struct {
            Bool* arr;
            std::size_t size;
        } booleans;
        struct {
            double* arr;
            std::size_t size;
        } doubles;
        struct {
            float* arr;
            std::size_t size;
        } floats;
        struct {
            std::int64_t* arr;
            std::size_t size;
        } integers;
        struct {
            String* arr;
            std::size_t size;
        } strings;
    } data;
};

struct TopicInfo {  // NT_TopicInfo
    Handle topic;
    String name;
    Type type;
    String typeStr;
    String properties;  // JSON object text
};

// NT_PubSubOptions. ntcore dereferences the pointer unconditionally (NULL crashes the robot), so every call gets a
// zero-initialised struct with structSize set.
struct PubSubOptions {
    unsigned int structSize;
    unsigned int pollStorage;
    double periodic;
    Handle excludePublisher;
    Bool sendAll;
    Bool topicsOnly;
    Bool prefixMatch;
    Bool keepDuplicates;
    Bool disableRemote;
    Bool disableLocal;
    Bool excludeSelf;
    Bool hidden;
};

// The sizes the binding relies on (x64: LLP64 Windows and LP64 Linux/macOS agree).
#if INTPTR_MAX == INT64_MAX
static_assert(sizeof(String) == 16, "WPI_String");
static_assert(sizeof(Value) == 40, "NT_Value");
static_assert(sizeof(TopicInfo) == 64, "NT_TopicInfo");
static_assert(sizeof(PubSubOptions) == 56, "NT_PubSubOptions");
#endif
static_assert(sizeof(Handle) == 4 && sizeof(Bool) == 4 && sizeof(Type) == 4);

}  // namespace wpiemu::bridges::vision::binding::abi
