#include <cstdio>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <utility>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <dlfcn.h>
#endif

#include <ntcore_c.h>

#include "wpiemu/bridges/vision/nt_binding.h"

namespace wpiemu::bridges::vision {

namespace {

// Attaches to an already loaded module (no reference is taken on Windows) and looks symbols up in it.
class Module {
public:
    Module() = default;
    Module(const Module&) = delete;
    Module& operator=(const Module&) = delete;
    ~Module() { release(); }

    bool open(const std::string& name, bool loadIfAbsent) {
        if (handle_ != nullptr) return true;
#if defined(_WIN32)
        handle_ = reinterpret_cast<void*>(GetModuleHandleA(name.c_str()));
        if (handle_ == nullptr && loadIfAbsent) {
            handle_ = reinterpret_cast<void*>(LoadLibraryA(name.c_str()));
            owned_ = handle_ != nullptr;
        }
#else
        handle_ = dlopen(name.c_str(), RTLD_NOLOAD | RTLD_LAZY);  // takes a reference only if already loaded
        if (handle_ == nullptr && loadIfAbsent) handle_ = dlopen(name.c_str(), RTLD_LAZY | RTLD_LOCAL);
        owned_ = handle_ != nullptr;
#endif
        return handle_ != nullptr;
    }

    void* symbol(const char* name) const {
        if (handle_ == nullptr) return nullptr;
#if defined(_WIN32)
        return reinterpret_cast<void*>(GetProcAddress(reinterpret_cast<HMODULE>(handle_), name));
#else
        return dlsym(handle_, name);
#endif
    }

    bool isOpen() const { return handle_ != nullptr; }

private:
    void release() {
        if (handle_ != nullptr && owned_) {
#if defined(_WIN32)
            FreeLibrary(reinterpret_cast<HMODULE>(handle_));
#else
            dlclose(handle_);
#endif
        }
        handle_ = nullptr;
    }

    void* handle_ = nullptr;
    bool owned_ = false;
};

std::string defaultNtcoreModule() {
#if defined(_WIN32)
    return "ntcore.dll";
#elif defined(__APPLE__)
    return "libntcore.dylib";
#else
    return "libntcore.so";
#endif
}

std::string defaultHalModule() {
#if defined(_WIN32)
    return "wpiHal.dll";
#elif defined(__APPLE__)
    return "libwpiHal.dylib";
#else
    return "libwpiHal.so";
#endif
}

// Function pointer types are taken from the ntcore headers (unevaluated, so nothing is linked).
struct NtApi {
    decltype(&NT_GetDefaultInstance) getDefaultInstance = nullptr;
    decltype(&NT_GetTopic) getTopic = nullptr;
    decltype(&NT_SetTopicProperty) setTopicProperty = nullptr;
    decltype(&NT_Publish) publish = nullptr;
    decltype(&NT_Unpublish) unpublish = nullptr;
    decltype(&NT_GetEntry) getEntry = nullptr;
    decltype(&NT_GetEntryValue) getEntryValue = nullptr;
    decltype(&NT_DisposeValue) disposeValue = nullptr;
    decltype(&NT_Now) now = nullptr;
    decltype(&NT_SetBoolean) setBoolean = nullptr;
    decltype(&NT_SetInteger) setInteger = nullptr;
    decltype(&NT_SetDouble) setDouble = nullptr;
    decltype(&NT_SetString) setString = nullptr;
    decltype(&NT_SetDoubleArray) setDoubleArray = nullptr;
    decltype(&NT_SetRaw) setRaw = nullptr;
};

using HalGetFpgaTimeFn = std::uint64_t (*)(std::int32_t* status);

WPI_String toWpiString(std::string_view s) { return WPI_String{s.data(), s.size()}; }

NT_Type typeOf(const NtValue& v) {
    switch (v.index()) {
        case 0: return NT_BOOLEAN;
        case 1: return NT_INTEGER;
        case 2: return NT_DOUBLE;
        case 3: return NT_STRING;
        case 4: return NT_DOUBLE_ARRAY;
        default: return NT_RAW;
    }
}

const char* defaultTypeString(NT_Type type) {
    switch (type) {
        case NT_BOOLEAN: return "boolean";
        case NT_INTEGER: return "int";
        case NT_DOUBLE: return "double";
        case NT_STRING: return "string";
        case NT_DOUBLE_ARRAY: return "double[]";
        default: return "raw";
    }
}

std::optional<NtValue> toValue(const NT_Value& v) {
    switch (v.type) {
        case NT_BOOLEAN: return ntBool(v.data.v_boolean != 0);
        case NT_INTEGER: return ntInt(v.data.v_int);
        case NT_FLOAT: return ntDouble(static_cast<double>(v.data.v_float));
        case NT_DOUBLE: return ntDouble(v.data.v_double);
        case NT_STRING: return ntString(std::string(v.data.v_string.str != nullptr ? v.data.v_string.str : "", v.data.v_string.len));
        case NT_RAW: return ntRaw(NtBytes(v.data.v_raw.data, v.data.v_raw.data + v.data.v_raw.size));
        case NT_DOUBLE_ARRAY: return ntDoubles(NtDoubles(v.data.arr_double.arr, v.data.arr_double.arr + v.data.arr_double.size));
        case NT_FLOAT_ARRAY: {
            NtDoubles out;
            out.reserve(v.data.arr_float.size);
            for (std::size_t i = 0; i < v.data.arr_float.size; ++i) out.push_back(static_cast<double>(v.data.arr_float.arr[i]));
            return ntDoubles(std::move(out));
        }
        case NT_INTEGER_ARRAY: {
            NtDoubles out;
            out.reserve(v.data.arr_int.size);
            for (std::size_t i = 0; i < v.data.arr_int.size; ++i) out.push_back(static_cast<double>(v.data.arr_int.arr[i]));
            return ntDoubles(std::move(out));
        }
        default: return std::nullopt;
    }
}

class NtcoreBackend final : public INtBackend {
public:
    explicit NtcoreBackend(NtcoreBackendOptions options) : options_(std::move(options)) {
        if (options_.ntcoreModule.empty()) options_.ntcoreModule = defaultNtcoreModule();
        if (options_.halModule.empty()) options_.halModule = defaultHalModule();
    }

    bool available() override {
        if (bound_) return true;
        if (failed_) return false;
        if (!nt_.open(options_.ntcoreModule, options_.loadIfAbsent)) return false;  // not loaded (yet): retry later
        if (!bind()) {
            failed_ = true;
            return false;
        }
        return true;
    }

    std::int64_t nowUs() override {
        if (!available()) return 0;
        if (halFpgaTime_ == nullptr) attachHal();
        if (halFpgaTime_ != nullptr) {
            std::int32_t status = 0;
            const std::uint64_t t = halFpgaTime_(&status);
            if (status == 0) return static_cast<std::int64_t>(t);
        }
        return api_.now();
    }

    sdk::Status publish(std::string_view path, const NtValue& value, std::int64_t timestampUs,
                        std::string_view typeString) override {
        if (!available()) return sdk::makeError(sdk::ErrorCode::InvalidState, "ntcore is not loaded");
        const NT_Type type = typeOf(value);
        const std::string key(path);

        auto it = pubs_.find(key);
        if (it != pubs_.end() && it->second.type != type) {
            return sdk::makeError(sdk::ErrorCode::InvalidArgument, "topic type changed: " + key);
        }
        if (it == pubs_.end()) {
            const std::string typeName = !typeString.empty() && type == NT_RAW ? std::string(typeString) : defaultTypeString(type);
            const WPI_String name = toWpiString(key);
            const NT_Topic topic = api_.getTopic(inst_, &name);
            const WPI_String wpiTypeName = toWpiString(typeName);
            NT_PubSubOptions options;
            std::memset(&options, 0, sizeof options);
            options.structSize = sizeof options;
            options.periodic = 0.01;
            options.sendAll = 1;
            options.keepDuplicates = 1;
            const NT_Publisher handle = topic != 0 ? api_.publish(topic, type, &wpiTypeName, &options) : 0;
            if (handle == 0) return sdk::makeError(sdk::ErrorCode::Internal, "could not publish " + key);
            it = pubs_.emplace(key, Pub{handle, type}).first;
        }

        // 0 means "now" to ntcore; the robot time base starts at 0.
        const std::int64_t ts = timestampUs > 0 ? timestampUs : 1;
        const NT_Handle h = it->second.handle;
        bool ok = false;
        switch (value.index()) {
            case 0: ok = api_.setBoolean(h, ts, std::get<bool>(value) ? 1 : 0) != 0; break;
            case 1: ok = api_.setInteger(h, ts, std::get<std::int64_t>(value)) != 0; break;
            case 2: ok = api_.setDouble(h, ts, std::get<double>(value)) != 0; break;
            case 3: {
                const std::string& s = std::get<std::string>(value);
                const WPI_String ws = toWpiString(s);
                ok = api_.setString(h, ts, &ws) != 0;
                break;
            }
            case 4: {
                const NtDoubles& a = std::get<NtDoubles>(value);
                ok = api_.setDoubleArray(h, ts, a.data(), a.size()) != 0;
                break;
            }
            default: {
                const NtBytes& b = std::get<NtBytes>(value);
                ok = api_.setRaw(h, ts, b.data(), b.size()) != 0;
                break;
            }
        }
        if (!ok) return sdk::makeError(sdk::ErrorCode::Internal, "ntcore rejected the value of " + key);
        return sdk::okStatus();
    }

    sdk::Status setTopicProperty(std::string_view path, std::string_view name, std::string_view jsonValue) override {
        if (!available()) return sdk::makeError(sdk::ErrorCode::InvalidState, "ntcore is not loaded");
        const std::string key(path);
        if (pubs_.find(key) == pubs_.end()) return sdk::makeError(sdk::ErrorCode::NotFound, "topic is not published: " + key);
        const WPI_String topicName = toWpiString(key);
        const WPI_String propName = toWpiString(name);
        const WPI_String value = toWpiString(jsonValue);
        if (api_.setTopicProperty(api_.getTopic(inst_, &topicName), &propName, &value) == 0) {
            return sdk::makeError(sdk::ErrorCode::Internal, "ntcore rejected property " + std::string(name) + " of " + key);
        }
        return sdk::okStatus();
    }

    std::optional<NtValue> read(std::string_view path) override {
        if (!available()) return std::nullopt;
        const std::string key(path);
        auto it = entries_.find(key);
        if (it == entries_.end()) {
            const WPI_String name = toWpiString(key);
            const NT_Entry entry = api_.getEntry(inst_, &name);
            if (entry == 0) return std::nullopt;
            it = entries_.emplace(key, entry).first;
        }
        NT_Value v;
        std::memset(&v, 0, sizeof v);
        api_.getEntryValue(it->second, &v);
        std::optional<NtValue> out = toValue(v);
        api_.disposeValue(&v);
        return out;
    }

    void unpublish(std::string_view pathPrefix) override {
        if (!bound_) return;
        for (auto it = pubs_.begin(); it != pubs_.end();) {
            if (it->first.compare(0, pathPrefix.size(), pathPrefix) == 0) {
                api_.unpublish(it->second.handle);
                it = pubs_.erase(it);
            } else {
                ++it;
            }
        }
    }

private:
    struct Pub {
        NT_Handle handle = 0;
        NT_Type type = NT_UNASSIGNED;
    };

    template <class Fn>
    bool load(Fn& out, const char* name) {
        out = reinterpret_cast<Fn>(nt_.symbol(name));
        if (out == nullptr) {
            std::fprintf(stderr, "wpiemu: vision: %s is missing from %s; ntcore publishing is disabled\n", name,
                         options_.ntcoreModule.c_str());
        }
        return out != nullptr;
    }

    bool bind() {
        bool ok = load(api_.getDefaultInstance, "NT_GetDefaultInstance");
        ok = load(api_.getTopic, "NT_GetTopic") && ok;
        ok = load(api_.setTopicProperty, "NT_SetTopicProperty") && ok;
        ok = load(api_.publish, "NT_Publish") && ok;
        ok = load(api_.unpublish, "NT_Unpublish") && ok;
        ok = load(api_.getEntry, "NT_GetEntry") && ok;
        ok = load(api_.getEntryValue, "NT_GetEntryValue") && ok;
        ok = load(api_.disposeValue, "NT_DisposeValue") && ok;
        ok = load(api_.now, "NT_Now") && ok;
        ok = load(api_.setBoolean, "NT_SetBoolean") && ok;
        ok = load(api_.setInteger, "NT_SetInteger") && ok;
        ok = load(api_.setDouble, "NT_SetDouble") && ok;
        ok = load(api_.setString, "NT_SetString") && ok;
        ok = load(api_.setDoubleArray, "NT_SetDoubleArray") && ok;
        ok = load(api_.setRaw, "NT_SetRaw") && ok;
        if (!ok) return false;
        inst_ = api_.getDefaultInstance();
        bound_ = true;
        return true;
    }

    void attachHal() {
        // wpiHal is loaded before the extension runs (HAL_Initialize loads it); never load it from here.
        if (hal_.open(options_.halModule, false)) {
            halFpgaTime_ = reinterpret_cast<HalGetFpgaTimeFn>(hal_.symbol("HAL_GetFPGATime"));
        }
    }

    NtcoreBackendOptions options_;
    Module nt_;
    Module hal_;
    NtApi api_;
    NT_Inst inst_ = 0;
    bool bound_ = false;
    bool failed_ = false;
    HalGetFpgaTimeFn halFpgaTime_ = nullptr;
    std::map<std::string, Pub> pubs_;
    std::map<std::string, NT_Entry> entries_;
};

}  // namespace

std::shared_ptr<INtBackend> makeNtcoreBackend(NtcoreBackendOptions options) {
    return std::make_shared<NtcoreBackend>(std::move(options));
}

sdk::Status registerNtVisionBridges(sdk::VisionBridgeRegistry& registry, VisionBridgeOptions options) {
    return registerVisionBridges(registry, makeNtcoreBackend(), options);
}

}  // namespace wpiemu::bridges::vision
