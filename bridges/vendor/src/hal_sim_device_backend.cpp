#include "wpiemu/bridges/vendor/hal_sim_device_backend.h"

#ifdef WPIEMU_VENDOR_HAS_WPILIB

#include <atomic>
#include <cmath>
#include <cstdint>
#include <map>
#include <mutex>
#include <set>
#include <unordered_map>

#include "wpiemu/hal_abi/sim_device_includes.h"

namespace wpiemu::bridges::vendor {

namespace {

constexpr const char* kSimDevicePrefix = "CAN";  // every CTRE SimDevice name starts with "CAN"

struct HalApi {
    int32_t (*registerCreated)(const char*, void*, HALSIM_SimDeviceCallback, HAL_Bool) = nullptr;
    void (*cancelCreated)(int32_t) = nullptr;
    int32_t (*registerFreed)(const char*, void*, HALSIM_SimDeviceCallback, HAL_Bool) = nullptr;
    void (*cancelFreed)(int32_t) = nullptr;
    int32_t (*registerValueCreated)(HAL_SimDeviceHandle, void*, HALSIM_SimValueCallback, HAL_Bool) = nullptr;
    void (*cancelValueCreated)(int32_t) = nullptr;
    void (*enumerateDevices)(const char*, void*, HALSIM_SimDeviceCallback) = nullptr;
    void (*enumerateValues)(HAL_SimDeviceHandle, void*, HALSIM_SimValueCallback) = nullptr;
    void (*getValue)(HAL_SimValueHandle, struct HAL_Value*) = nullptr;
    void (*setValue)(HAL_SimValueHandle, const struct HAL_Value*) = nullptr;

    bool complete() const {
        return registerCreated && cancelCreated && registerFreed && cancelFreed && registerValueCreated &&
               cancelValueCreated && enumerateDevices && enumerateValues && getValue && setValue;
    }
};

template <class Fn>
void bindSymbol(const ISymbolResolver& r, const char* name, Fn& slot) {
    slot = reinterpret_cast<Fn>(r.resolve(name));
}

// Threading contract (the reason this class looks the way it does): HAL invokes our callbacks on ROBOT threads
// while it holds its own lock, and the robot may be creating devices/values at any moment a poll runs. So
//  * callbacks only bump an atomic generation counter (no locks, no HAL calls, no allocation);
//  * everything else runs on the poll thread and never calls into HAL while holding mutex_ (lock order is always
//    HAL -> mutex_, taken by the collect callbacks);
//  * handles are only dereferenced by HAL itself; a stale handle makes HAL_GetSimValue/HAL_SetSimValue and the
//    enumerators no-ops, never a crash;
//  * a device is reported the moment it exists; values it gains later are picked up through per-device
//    value-created callbacks (generation bump) + IncrementalSimDeviceBackend (no timing heuristics).
class HalSimDeviceBackend final : public sdk::ISimDeviceBackend {
public:
    HalSimDeviceBackend(std::shared_ptr<const ISymbolResolver> resolver, HalApi api)
        : resolver_(std::move(resolver)), api_(api) {
        createdUid_ = api_.registerCreated(kSimDevicePrefix, this, &HalSimDeviceBackend::onCreated, /*initialNotify=*/0);
        freedUid_ = api_.registerFreed(kSimDevicePrefix, this, &HalSimDeviceBackend::onFreed, /*initialNotify=*/0);
    }

    ~HalSimDeviceBackend() override {
        if (createdUid_ != 0) api_.cancelCreated(createdUid_);
        if (freedUid_ != 0) api_.cancelFreed(freedUid_);
        for (const auto& [device, uid] : valueCallbacks_)
            if (uid != 0) api_.cancelValueCreated(uid);
    }

    std::uint64_t generation() const override { return generation_.load(std::memory_order_acquire); }

    void listDevices(std::vector<sdk::SimDeviceInfo>& out) const override {
        std::vector<sdk::SimDeviceInfo> all;
        api_.enumerateDevices(kSimDevicePrefix, &all, &HalSimDeviceBackend::collectDevice);

        std::set<std::int32_t> present;
        for (auto& dev : all) {
            const auto key = static_cast<std::int32_t>(dev.handle.value);
            present.insert(key);
            bool needsCallback = false;
            {
                std::lock_guard lock(mutex_);
                needsCallback = valueCallbacks_.count(key) == 0;
            }
            if (needsCallback) {
                // Registered BEFORE the consumer lists the values of this device (the consumer does it right after
                // this call), so a value created in between is seen by the enumeration or by the callback.
                const std::int32_t uid = api_.registerValueCreated(key, const_cast<HalSimDeviceBackend*>(this),
                                                                  &HalSimDeviceBackend::onValueCreated, /*initialNotify=*/0);
                std::lock_guard lock(mutex_);
                valueCallbacks_[key] = uid;
            }
            out.push_back(std::move(dev));
        }
        std::lock_guard lock(mutex_);  // freed devices took their HAL callbacks with them
        for (auto it = valueCallbacks_.begin(); it != valueCallbacks_.end();)
            it = present.count(it->first) != 0 ? std::next(it) : valueCallbacks_.erase(it);
    }

    void listValues(sdk::SimDeviceHandle device, std::vector<sdk::SimValueInfo>& out) const override {
        ValueCtx ctx{this, &out};
        api_.enumerateValues(static_cast<HAL_SimDeviceHandle>(device.value), &ctx, &HalSimDeviceBackend::collectValue);
    }

    double getValue(sdk::SimValueHandle value) const override {
        HAL_Value v{};
        api_.getValue(static_cast<HAL_SimValueHandle>(value.value), &v);
        switch (v.type) {
            case HAL_BOOLEAN: return v.data.v_boolean != 0 ? 1.0 : 0.0;
            case HAL_DOUBLE: return v.data.v_double;
            case HAL_ENUM: return static_cast<double>(v.data.v_enum);
            case HAL_INT: return static_cast<double>(v.data.v_int);
            case HAL_LONG: return static_cast<double>(v.data.v_long);
            default: return 0.0;
        }
    }

    void setValue(sdk::SimValueHandle value, double d) override {
        HAL_Type type = HAL_DOUBLE;
        {
            std::lock_guard lock(mutex_);
            const auto it = types_.find(static_cast<std::int32_t>(value.value));
            if (it != types_.end()) type = it->second;
        }
        HAL_Value v;
        switch (type) {
            case HAL_BOOLEAN: v = HAL_MakeBoolean(d != 0.0 ? 1 : 0); break;
            case HAL_ENUM: v = HAL_MakeEnum(static_cast<int>(std::llround(d))); break;
            case HAL_INT: v = HAL_MakeInt(static_cast<int>(std::llround(d))); break;
            case HAL_LONG: v = HAL_MakeLong(static_cast<int64_t>(std::llround(d))); break;
            default: v = HAL_MakeDouble(d); break;
        }
        api_.setValue(static_cast<HAL_SimValueHandle>(value.value), &v);
    }

private:
    struct ValueCtx {
        const HalSimDeviceBackend* self;
        std::vector<sdk::SimValueInfo>* out;
    };

    // HAL callbacks run on robot threads while HAL holds its lock: bump a counter, nothing else.
    static void onCreated(const char*, void* param, HAL_SimDeviceHandle) {
        static_cast<HalSimDeviceBackend*>(param)->generation_.fetch_add(1, std::memory_order_acq_rel);
    }
    static void onFreed(const char*, void* param, HAL_SimDeviceHandle) {
        static_cast<HalSimDeviceBackend*>(param)->generation_.fetch_add(1, std::memory_order_acq_rel);
    }
    static void onValueCreated(const char*, void* param, HAL_SimValueHandle, int32_t, const struct HAL_Value*) {
        static_cast<HalSimDeviceBackend*>(param)->generation_.fetch_add(1, std::memory_order_acq_rel);
    }
    static void collectDevice(const char* name, void* param, HAL_SimDeviceHandle handle) {
        static_cast<std::vector<sdk::SimDeviceInfo>*>(param)->push_back(
            {sdk::SimDeviceHandle(static_cast<std::uint64_t>(handle)), name != nullptr ? name : ""});
    }
    static void collectValue(const char* name, void* param, HAL_SimValueHandle handle, int32_t direction,
                             const struct HAL_Value* value) {
        auto* ctx = static_cast<ValueCtx*>(param);
        sdk::SimValueInfo info;
        info.handle = sdk::SimValueHandle(static_cast<std::uint64_t>(handle));
        info.name = name != nullptr ? name : "";
        switch (direction) {
            case hal_abi::kSimValueInput: info.direction = sdk::SignalDirection::Input; break;
            case hal_abi::kSimValueOutput: info.direction = sdk::SignalDirection::Output; break;
            default: info.direction = sdk::SignalDirection::Bidirectional; break;
        }
        const HAL_Type type = value != nullptr ? value->type : HAL_DOUBLE;
        info.type = type == HAL_BOOLEAN ? sdk::SignalType::Bool
                                        : (type == HAL_DOUBLE ? sdk::SignalType::Double : sdk::SignalType::Int);
        {
            std::lock_guard lock(ctx->self->mutex_);
            ctx->self->types_[handle] = type;
        }
        ctx->out->push_back(std::move(info));
    }

    std::shared_ptr<const ISymbolResolver> resolver_;
    HalApi api_;
    std::int32_t createdUid_ = 0;
    std::int32_t freedUid_ = 0;
    std::atomic<std::uint64_t> generation_{0};
    mutable std::mutex mutex_;
    mutable std::map<std::int32_t, std::int32_t> valueCallbacks_;  // device handle -> value-created callback uid
    mutable std::unordered_map<std::int32_t, HAL_Type> types_;
};

}  // namespace

bool halBindingCompiledIn() { return true; }

std::unique_ptr<sdk::ISimDeviceBackend> makeHalSimDeviceBackend(std::shared_ptr<const ISymbolResolver> hal,
                                                                std::string* error) {
    if (!hal) {
        std::unique_ptr<ModuleSymbolResolver> module =
            ModuleSymbolResolver::open(wpiHalModuleName(), ModuleLoadMode::AttachOnly, error);
        if (!module) return nullptr;
        hal = std::move(module);
    }
    HalApi api;
    bindSymbol(*hal, "HALSIM_RegisterSimDeviceCreatedCallback", api.registerCreated);
    bindSymbol(*hal, "HALSIM_CancelSimDeviceCreatedCallback", api.cancelCreated);
    bindSymbol(*hal, "HALSIM_RegisterSimDeviceFreedCallback", api.registerFreed);
    bindSymbol(*hal, "HALSIM_CancelSimDeviceFreedCallback", api.cancelFreed);
    bindSymbol(*hal, "HALSIM_RegisterSimValueCreatedCallback", api.registerValueCreated);
    bindSymbol(*hal, "HALSIM_CancelSimValueCreatedCallback", api.cancelValueCreated);
    bindSymbol(*hal, "HALSIM_EnumerateSimDevices", api.enumerateDevices);
    bindSymbol(*hal, "HALSIM_EnumerateSimValues", api.enumerateValues);
    bindSymbol(*hal, "HAL_GetSimValue", api.getValue);
    bindSymbol(*hal, "HAL_SetSimValue", api.setValue);
    if (!api.complete()) {
        if (error != nullptr) *error = "wpiHal does not export the HALSIM SimDevice API";
        return nullptr;
    }
    return std::make_unique<HalSimDeviceBackend>(std::move(hal), api);
}

}  // namespace wpiemu::bridges::vendor

#else  // !WPIEMU_VENDOR_HAS_WPILIB

namespace wpiemu::bridges::vendor {

bool halBindingCompiledIn() { return false; }

std::unique_ptr<sdk::ISimDeviceBackend> makeHalSimDeviceBackend(std::shared_ptr<const ISymbolResolver>, std::string* error) {
    if (error != nullptr) *error = "built without WPILib headers (set WPIEMU_WPILIB_ROOT)";
    return nullptr;
}

}  // namespace wpiemu::bridges::vendor

#endif
