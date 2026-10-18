#pragma once

// The ntcore C functions the binding calls (internal to src/binding), behind one small table.
//
// The real table is filled from the ntcore module of the robot process (nt_binder.h); tests fill it with fakes, so
// NtCoreApi never knows where its entry points come from. Every entry is declared with the exact C signature of the
// function of the same name in ntcore_c.h / ntcore_c_types.h (nt_abi_check.cpp proves it against the real headers
// whenever they are staged). The functions are listed once, in WPIEMU_NT_FUNCTIONS, and the table, the symbol
// resolution and the signature check are all generated from that list.

#include <cstddef>
#include <cstdint>

#include "nt_abi.h"

// X(name, result, parameters): `name` is the exported C symbol.
#define WPIEMU_NT_FUNCTIONS(X)                                                                                      \
    X(NT_GetDefaultInstance, abi::Handle, ())                                                                       \
    X(NT_Now, std::int64_t, ())                                                                                     \
    X(NT_GetTopic, abi::Handle, (abi::Handle inst, const abi::String* name))                                        \
    X(NT_GetTopicExists, abi::Bool, (abi::Handle topicOrPubSub))                                                    \
    X(NT_GetTopicInfos, abi::TopicInfo*,                                                                            \
      (abi::Handle inst, const abi::String* prefix, unsigned int types, std::size_t* count))                        \
    X(NT_DisposeTopicInfoArray, void, (abi::TopicInfo* infos, std::size_t count))                                  \
    X(NT_PublishEx, abi::Handle,                                                                                    \
      (abi::Handle topic, abi::Type type, const abi::String* typeStr, const abi::String* properties,                \
       const abi::PubSubOptions* options))                                                                          \
    X(NT_Subscribe, abi::Handle,                                                                                    \
      (abi::Handle topic, abi::Type type, const abi::String* typeStr, const abi::PubSubOptions* options))           \
    X(NT_SetTopicProperty, abi::Bool, (abi::Handle topic, const abi::String* name, const abi::String* jsonValue))  \
    X(NT_SetBoolean, abi::Bool, (abi::Handle pub, std::int64_t time, abi::Bool value))                              \
    X(NT_SetInteger, abi::Bool, (abi::Handle pub, std::int64_t time, std::int64_t value))                           \
    X(NT_SetDouble, abi::Bool, (abi::Handle pub, std::int64_t time, double value))                                  \
    X(NT_SetString, abi::Bool, (abi::Handle pub, std::int64_t time, const abi::String* value))                      \
    X(NT_SetRaw, abi::Bool, (abi::Handle pub, std::int64_t time, const std::uint8_t* value, std::size_t length))    \
    X(NT_SetDoubleArray, abi::Bool, (abi::Handle pub, std::int64_t time, const double* value, std::size_t length))  \
    X(NT_ReadQueueValue, abi::Value*, (abi::Handle sub, std::size_t* count))                                        \
    X(NT_DisposeValueArray, void, (abi::Value* values, std::size_t count))

namespace wpiemu::bridges::vision::binding {

struct NtFunctions {
#define WPIEMU_NT_MEMBER(name, result, parameters) result(*name) parameters = nullptr;
    WPIEMU_NT_FUNCTIONS(WPIEMU_NT_MEMBER)
#undef WPIEMU_NT_MEMBER

    // Calls `visitor(const char* symbolName, <function pointer member>&)` for every entry, in list order.
    template <class Visitor>
    void visit(Visitor&& visitor) {
#define WPIEMU_NT_VISIT(name, result, parameters) visitor(#name, name);
        WPIEMU_NT_FUNCTIONS(WPIEMU_NT_VISIT)
#undef WPIEMU_NT_VISIT
    }

    // The name of the first entry that is not set, or nullptr when the table is complete.
    const char* firstMissing() const {
#define WPIEMU_NT_CHECK(name, result, parameters) \
    if (name == nullptr) return #name;
        WPIEMU_NT_FUNCTIONS(WPIEMU_NT_CHECK)
#undef WPIEMU_NT_CHECK
        return nullptr;
    }
};

}  // namespace wpiemu::bridges::vision::binding
