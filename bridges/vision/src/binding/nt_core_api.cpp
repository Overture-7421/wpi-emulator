#include "nt_core_api.h"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <utility>

#include "wpiemu/bridges/vision/nt_binding.h"
#include "wpiemu/bridges/vision/null_nt_api.h"

namespace wpiemu::bridges::vision::binding {

namespace {

// A WPI_String over `text` (never a NULL pointer, even for an empty or default-constructed view).
abi::String toNative(std::string_view text) {
    return abi::String{text.data() != nullptr ? text.data() : "", text.size()};
}

// ntcore hands out {nullptr, 0} for an empty string.
std::string_view toView(const abi::String& s) {
    return s.str != nullptr ? std::string_view(s.str, s.len) : std::string_view();
}

abi::Type toNative(NtType type) {
    switch (type) {
        case NtType::Boolean: return abi::Type::Boolean;
        case NtType::Integer: return abi::Type::Integer;
        case NtType::Double: return abi::Type::Double;
        case NtType::String: return abi::Type::String;
        case NtType::Raw: return abi::Type::Raw;
        case NtType::DoubleArray: return abi::Type::DoubleArray;
    }
    return abi::Type::Unassigned;
}

// ntcore converts the period to an unsigned number of milliseconds, which is undefined for a negative or non-finite
// one.
double periodOrDefault(double periodicS) {
    if (!(periodicS > 0.0)) return 0.0;  // 0 = ntcore's default, also for NaN
    return std::min(periodicS, 1000.0);
}

// An array that ntcore returned: iterable, and disposed exactly once by the function ntcore provides for it, whatever
// happens while it is read.
template <class Element, class Dispose>
class NativeArray {
public:
    NativeArray(Element* data, std::size_t count, Dispose dispose)
        : data_(data), count_(data != nullptr ? count : 0), dispose_(dispose) {}
    NativeArray(const NativeArray&) = delete;
    NativeArray& operator=(const NativeArray&) = delete;
    ~NativeArray() {
        if (data_ != nullptr) dispose_(data_, count_);
    }

    const Element* begin() const { return data_; }
    const Element* end() const { return data_ + count_; }

private:
    Element* data_;
    std::size_t count_;
    Dispose dispose_;
};

// Converts one value the robot wrote. What NtValue cannot represent (boolean[], string[], RPC) and the empty value
// ntcore queues for a topic that has none (Unassigned) are skipped; integer and float arrays widen to doubles.
bool convert(const abi::Value& in, NtValue& out) {
    out.timeUs = in.lastChange;
    switch (in.type) {
        case abi::Type::Boolean:
            out.type = NtType::Boolean;
            out.number = in.data.boolean != 0 ? 1.0 : 0.0;
            return true;
        case abi::Type::Integer:
            out.type = NtType::Integer;
            out.number = static_cast<double>(in.data.integer);
            return true;
        case abi::Type::Float:
            out.type = NtType::Double;
            out.number = static_cast<double>(in.data.float32);
            return true;
        case abi::Type::Double:
            out.type = NtType::Double;
            out.number = in.data.float64;
            return true;
        case abi::Type::String:
            out.type = NtType::String;
            out.text = std::string(toView(in.data.string));
            return true;
        case abi::Type::Raw:
            out.type = NtType::Raw;
            if (in.data.raw.data != nullptr) out.raw.assign(in.data.raw.data, in.data.raw.data + in.data.raw.size);
            return true;
        case abi::Type::DoubleArray:
            out.type = NtType::DoubleArray;
            if (in.data.doubles.arr != nullptr)
                out.numbers.assign(in.data.doubles.arr, in.data.doubles.arr + in.data.doubles.size);
            return true;
        case abi::Type::IntegerArray:
            out.type = NtType::DoubleArray;
            if (in.data.integers.arr != nullptr)
                for (std::size_t i = 0; i < in.data.integers.size; ++i)
                    out.numbers.push_back(static_cast<double>(in.data.integers.arr[i]));
            return true;
        case abi::Type::FloatArray:
            out.type = NtType::DoubleArray;
            if (in.data.floats.arr != nullptr)
                for (std::size_t i = 0; i < in.data.floats.size; ++i)
                    out.numbers.push_back(static_cast<double>(in.data.floats.arr[i]));
            return true;
        case abi::Type::Unassigned:
        case abi::Type::BooleanArray:
        case abi::Type::StringArray:
        case abi::Type::Rpc:
            break;
    }
    return false;
}

}  // namespace

NtCoreApi::NtCoreApi(Binder binder) : binder_(std::move(binder)) {}

bool NtCoreApi::tryBind() const {
    if (!binder_) return false;
    NtFunctions candidate;
    BindOutcome outcome = binder_(candidate);
    if (outcome.bound()) {
        // A binder that claims success must have filled every entry: a table with a hole would crash the robot.
        if (const char* missing = candidate.firstMissing()) {
            outcome.kind = BindOutcome::Kind::SymbolMissing;
            outcome.symbol = missing;
        }
    }
    if (!outcome.bound()) {
        rememberFailure(outcome);
        return false;
    }
    const abi::Handle instance = candidate.NT_GetDefaultInstance();
    if (instance == 0) {
        // Not ready to hand out its instance: the next call tries again.
        lastKind_ = BindOutcome::Kind::Bound;  // marks reason_ as custom, so the next failure rebuilds it
        reason_.assign("ntcore has no default instance yet");
        return false;
    }
    fn_ = candidate;
    instance_ = instance;
    bound_ = true;
    reason_.clear();
    return true;
}

void NtCoreApi::rememberFailure(const BindOutcome& outcome) const {
    const std::string_view module = outcome.module;
    const std::string_view symbol = outcome.symbol;
    if (outcome.kind == lastKind_ && module == lastModule_ && symbol == lastSymbol_) return;  // same cause: no work
    lastKind_ = outcome.kind;
    lastModule_ = module;
    lastSymbol_ = symbol;
    switch (outcome.kind) {
        case BindOutcome::Kind::ModuleNotLoaded:
            reason_ = std::string(module) + " is not loaded in this process (yet)";
            break;
        case BindOutcome::Kind::SymbolMissing:
            reason_ = std::string(module) + " does not export " + std::string(symbol) +
                      " (a different WPILib than the one this build was pinned to?)";
            break;
        case BindOutcome::Kind::Unsupported:
            reason_ = "binding to the ntcore of the robot process is not supported on this platform";
            break;
        case BindOutcome::Kind::Bound:
            break;  // not a failure
    }
}

bool NtCoreApi::available() const { return ready(); }

std::string_view NtCoreApi::unavailableReason() const {
    return ready() ? std::string_view() : std::string_view(reason_);
}

std::int64_t NtCoreApi::nowUs() const { return ready() ? fn_.NT_Now() : 0; }

abi::Handle NtCoreApi::topicHandle(std::string_view topic) const {
    if (topic.empty()) return 0;  // ntcore has no topic without a name
    const abi::String name = toNative(topic);
    return fn_.NT_GetTopic(instance_, &name);
}

NtPublisher NtCoreApi::publish(std::string_view topic, NtType type, const NtPublishOptions& options) {
    if (!ready()) return {};
    const std::string_view typeString = options.typeString.empty() ? ntTypeString(type) : options.typeString;
    if (typeString.empty()) return {};  // not a type this API knows
    const abi::Handle handleOfTopic = topicHandle(topic);
    if (handleOfTopic == 0) return {};

    abi::PubSubOptions native{};
    native.structSize = static_cast<unsigned int>(sizeof(native));
    native.periodic = periodOrDefault(options.periodicS);
    native.sendAll = options.sendAll ? 1 : 0;
    native.keepDuplicates = options.keepDuplicates ? 1 : 0;
    const abi::String typeStr = toNative(typeString);
    const abi::String properties = toNative(options.propertiesJson);  // "" = none, never NULL
    const abi::Handle handle = fn_.NT_PublishEx(handleOfTopic, toNative(type), &typeStr, &properties, &native);
    if (handle == 0) return {};  // malformed properties JSON, too many publishers on the topic, ...

    publishers_.push_back(PublisherRecord{handle, type});
    if (ownTopics_.find(topic) == ownTopics_.end()) ownTopics_.emplace(topic);
    return NtPublisher{static_cast<std::uint32_t>(publishers_.size())};
}

const NtCoreApi::PublisherRecord* NtCoreApi::publisherFor(NtPublisher pub, NtType type) const {
    if (!bound_ || pub.id == 0 || pub.id > publishers_.size()) return nullptr;
    const PublisherRecord& record = publishers_[pub.id - 1];
    return record.type == type ? &record : nullptr;
}

bool NtCoreApi::setBoolean(NtPublisher pub, std::int64_t timeUs, bool value) {
    const PublisherRecord* record = publisherFor(pub, NtType::Boolean);
    return record != nullptr && fn_.NT_SetBoolean(record->handle, timeUs, value ? 1 : 0) != 0;
}

bool NtCoreApi::setInteger(NtPublisher pub, std::int64_t timeUs, std::int64_t value) {
    const PublisherRecord* record = publisherFor(pub, NtType::Integer);
    return record != nullptr && fn_.NT_SetInteger(record->handle, timeUs, value) != 0;
}

bool NtCoreApi::setDouble(NtPublisher pub, std::int64_t timeUs, double value) {
    const PublisherRecord* record = publisherFor(pub, NtType::Double);
    return record != nullptr && fn_.NT_SetDouble(record->handle, timeUs, value) != 0;
}

bool NtCoreApi::setString(NtPublisher pub, std::int64_t timeUs, std::string_view value) {
    const PublisherRecord* record = publisherFor(pub, NtType::String);
    if (record == nullptr) return false;
    const abi::String native = toNative(value);
    return fn_.NT_SetString(record->handle, timeUs, &native) != 0;
}

bool NtCoreApi::setRaw(NtPublisher pub, std::int64_t timeUs, std::span<const std::uint8_t> value) {
    const PublisherRecord* record = publisherFor(pub, NtType::Raw);
    return record != nullptr && fn_.NT_SetRaw(record->handle, timeUs, value.data(), value.size()) != 0;
}

bool NtCoreApi::setDoubleArray(NtPublisher pub, std::int64_t timeUs, std::span<const double> value) {
    const PublisherRecord* record = publisherFor(pub, NtType::DoubleArray);
    return record != nullptr && fn_.NT_SetDoubleArray(record->handle, timeUs, value.data(), value.size()) != 0;
}

bool NtCoreApi::setTopicProperty(std::string_view topic, std::string_view name, std::string_view jsonValue) {
    if (!ready()) return false;
    const abi::Handle handleOfTopic = topicHandle(topic);
    if (handleOfTopic == 0) return false;
    const abi::String nameStr = toNative(name);
    const abi::String valueStr = toNative(jsonValue);
    // 0, and nothing stored, for text that is not JSON.
    return fn_.NT_SetTopicProperty(handleOfTopic, &nameStr, &valueStr) != 0;
}

NtSubscriber NtCoreApi::subscribe(std::string_view topic, std::size_t queueDepth) {
    if (!ready()) return {};
    const abi::Handle handleOfTopic = topicHandle(topic);
    if (handleOfTopic == 0) return {};

    // No type filter (Unassigned + empty type string), every update queued (sendAll), duplicates included: what reaches
    // the queue is then decided by the writer's own publisher options alone.
    abi::PubSubOptions native{};
    native.structSize = static_cast<unsigned int>(sizeof(native));
    native.pollStorage = static_cast<unsigned int>(std::clamp<std::size_t>(queueDepth, 1, kMaxQueueDepth));
    native.sendAll = 1;
    native.keepDuplicates = 1;
    const abi::String anyType = toNative(std::string_view());
    const abi::Handle handle = fn_.NT_Subscribe(handleOfTopic, abi::Type::Unassigned, &anyType, &native);
    if (handle == 0) return {};
    subscribers_.push_back(handle);
    return NtSubscriber{static_cast<std::uint32_t>(subscribers_.size())};
}

void NtCoreApi::read(NtSubscriber sub, std::vector<NtValue>& out) {
    if (!bound_ || sub.id == 0 || sub.id > subscribers_.size()) return;
    std::size_t count = 0;
    abi::Value* values = fn_.NT_ReadQueueValue(subscribers_[sub.id - 1], &count);
    const NativeArray<abi::Value, decltype(fn_.NT_DisposeValueArray)> queued(values, count, fn_.NT_DisposeValueArray);
    for (const abi::Value& value : queued) {
        NtValue converted;
        if (convert(value, converted)) out.push_back(std::move(converted));
    }
}

void NtCoreApi::topics(std::string_view prefix, std::vector<NtTopicInfo>& out) {
    if (!ready()) return;
    const abi::String native = toNative(prefix);
    std::size_t count = 0;
    abi::TopicInfo* infos = fn_.NT_GetTopicInfos(instance_, &native, 0 /* any type */, &count);
    const NativeArray<abi::TopicInfo, decltype(fn_.NT_DisposeTopicInfoArray)> listed(infos, count,
                                                                                    fn_.NT_DisposeTopicInfoArray);
    const std::size_t first = out.size();
    for (const abi::TopicInfo& info : listed) {
        NtTopicInfo entry;
        entry.name = std::string(toView(info.name));
        entry.typeString = std::string(toView(info.typeStr));
        entry.propertiesJson = info.properties.len != 0 ? std::string(toView(info.properties)) : std::string("{}");
        out.push_back(std::move(entry));
    }
    // ntcore lists topics in creation order; a stable order keeps what the bridge discovers reproducible.
    std::sort(out.begin() + static_cast<std::ptrdiff_t>(first), out.end(),
              [](const NtTopicInfo& a, const NtTopicInfo& b) { return a.name < b.name; });
}

bool NtCoreApi::hasForeignPublisher(std::string_view topic) {
    if (!ready() || ownTopics_.find(topic) != ownTopics_.end()) return false;
    const abi::Handle handle = topicHandle(topic);
    // A topic exists while it has a publisher; the robot's own subscribers do not make it exist.
    return handle != 0 && fn_.NT_GetTopicExists(handle) != 0;
}

}  // namespace wpiemu::bridges::vision::binding

namespace wpiemu::bridges::vision {

std::unique_ptr<INtApi> makeNtCoreApi() {
    if (!binding::abiVerifiedAgainstHeaders()) return std::make_unique<NullNtApi>(binding::kBuiltWithoutHeadersReason);
    return std::make_unique<binding::NtCoreApi>(&binding::bindProcessNtcore);
}

}  // namespace wpiemu::bridges::vision
