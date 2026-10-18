#include <doctest/doctest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <vector>

// The marshalling layer of the real binding, exercised through a FAKE ntcore: a table of C functions written here with
// the exact C signatures (nt_functions.h). No ntcore library, no ntcore header and no WPILib is needed, so these tests
// run on every machine. tests/bridges_vision/test_nt_binding_real.cpp runs the same INtApi scenarios against the real
// ntcore.
#include "../../bridges/vision/src/binding/nt_core_api.h"

namespace {

using namespace wpiemu::bridges::vision;
namespace nb = wpiemu::bridges::vision::binding;
namespace abi = wpiemu::bridges::vision::binding::abi;

// ---- the fake ntcore ----------------------------------------------------------------------------------------------

struct QueuedValue {  // one entry of the queue NT_ReadQueueValue returns
    abi::Type type = abi::Type::Unassigned;
    std::int64_t timeUs = 0;
    double number = 0.0;
    std::int64_t integer = 0;
    std::string text;
    std::vector<std::uint8_t> raw;
    std::vector<double> doubles;
    std::vector<std::int64_t> integers;
    std::vector<float> floats;
};

struct TopicSpec {
    std::string name, typeStr, properties;
};

struct PublishCall {
    abi::Handle topic = 0;
    abi::Type type = abi::Type::Unassigned;
    bool typeStrValid = false, propertiesValid = false, optionsValid = false;
    std::string typeStr, properties;
    abi::PubSubOptions options{};
};

struct SubscribeCall {
    abi::Handle topic = 0;
    abi::Type type = abi::Type::Unassigned;
    bool typeStrValid = false, optionsValid = false;
    std::string typeStr;
    abi::PubSubOptions options{};
};

struct SetCall {
    std::string function;
    abi::Handle pub = 0;
    std::int64_t timeUs = 0;
    double number = 0.0;
    std::int64_t integer = 0;
    bool valueValid = true;  // the pointer of a string / raw / array value
    std::string text;
    std::vector<std::uint8_t> raw;
    std::vector<double> doubles;
};

struct PropertyCall {
    abi::Handle topic = 0;
    std::string name, value;
};

struct FakeNtcore {
    // scripted behaviour
    abi::Handle instance = 777;
    std::int64_t now = 123456789;
    bool rejectPublish = false;    // like malformed properties JSON
    bool rejectSubscribe = false;
    bool rejectProperty = false;   // like property text that is not JSON
    bool refuseSets = false;
    std::set<std::string> existing;  // the topics that have a publisher
    std::vector<QueuedValue> queue;  // what the next NT_ReadQueueValue returns
    std::vector<TopicSpec> topicList;

    // records
    int calls = 0;  // every ntcore call except NT_GetDefaultInstance
    int badArguments = 0;
    std::vector<std::string> topicLookups;
    std::vector<PublishCall> publishes;
    std::vector<SubscribeCall> subscriptions;
    std::vector<SetCall> sets;
    std::vector<PropertyCall> properties;
    std::vector<std::string> prefixes;
    std::vector<unsigned int> typeMasks;
    std::vector<abi::Handle> reads;
    int valueArraysDisposed = 0, topicArraysDisposed = 0, badDisposals = 0;
    std::set<void*> live;  // arrays handed out and not disposed yet

    std::map<std::string, abi::Handle> topicHandles;
    std::map<abi::Handle, std::string> topicNames;
    abi::Handle nextTopic = 1000, lastPublisher = 5000, lastSubscriber = 9000;

    nb::NtFunctions table();
    std::string nameOf(abi::Handle topic) const {
        const auto it = topicNames.find(topic);
        return it == topicNames.end() ? std::string() : it->second;
    }
};

FakeNtcore* g = nullptr;  // the fake the C functions below talk to (they have no context argument)

struct ActiveFake {
    explicit ActiveFake(FakeNtcore& fake) { g = &fake; }
    ~ActiveFake() { g = nullptr; }
};

bool valid(const abi::String* s) { return s != nullptr && s->str != nullptr; }
std::string copy(const abi::String* s) { return valid(s) ? std::string(s->str, s->len) : std::string(); }

abi::Handle fGetDefaultInstance() { return g->instance; }
std::int64_t fNow() {
    ++g->calls;
    return g->now;
}
abi::Handle fGetTopic(abi::Handle inst, const abi::String* name) {
    ++g->calls;
    if (inst != g->instance || !valid(name)) {
        ++g->badArguments;
        return 0;
    }
    const std::string text = copy(name);
    g->topicLookups.push_back(text);
    const auto [it, inserted] = g->topicHandles.try_emplace(text, g->nextTopic);
    if (inserted) g->topicNames[g->nextTopic++] = text;
    return it->second;
}
abi::Bool fGetTopicExists(abi::Handle topic) {
    ++g->calls;
    return g->existing.count(g->nameOf(topic)) != 0 ? 1 : 0;
}
abi::Handle fPublishEx(abi::Handle topic, abi::Type type, const abi::String* typeStr, const abi::String* properties,
                       const abi::PubSubOptions* options) {
    ++g->calls;
    PublishCall call;
    call.topic = topic;
    call.type = type;
    call.typeStrValid = valid(typeStr);
    call.typeStr = copy(typeStr);
    call.propertiesValid = valid(properties);
    call.properties = copy(properties);
    call.optionsValid = options != nullptr;
    if (options != nullptr) call.options = *options;
    g->publishes.push_back(call);
    return g->rejectPublish ? 0 : ++g->lastPublisher;
}
abi::Handle fSubscribe(abi::Handle topic, abi::Type type, const abi::String* typeStr,
                       const abi::PubSubOptions* options) {
    ++g->calls;
    SubscribeCall call;
    call.topic = topic;
    call.type = type;
    call.typeStrValid = valid(typeStr);
    call.typeStr = copy(typeStr);
    call.optionsValid = options != nullptr;
    if (options != nullptr) call.options = *options;
    g->subscriptions.push_back(call);
    return g->rejectSubscribe ? 0 : ++g->lastSubscriber;
}
abi::Bool fSetTopicProperty(abi::Handle topic, const abi::String* name, const abi::String* jsonValue) {
    ++g->calls;
    if (!valid(name) || !valid(jsonValue)) ++g->badArguments;
    g->properties.push_back(PropertyCall{topic, copy(name), copy(jsonValue)});
    return g->rejectProperty ? 0 : 1;
}

SetCall& record(const char* function, abi::Handle pub, std::int64_t timeUs) {
    ++g->calls;
    g->sets.emplace_back();
    SetCall& call = g->sets.back();
    call.function = function;
    call.pub = pub;
    call.timeUs = timeUs;
    return call;
}
abi::Bool fSetBoolean(abi::Handle pub, std::int64_t timeUs, abi::Bool value) {
    record("NT_SetBoolean", pub, timeUs).integer = value;
    return g->refuseSets ? 0 : 1;
}
abi::Bool fSetInteger(abi::Handle pub, std::int64_t timeUs, std::int64_t value) {
    record("NT_SetInteger", pub, timeUs).integer = value;
    return g->refuseSets ? 0 : 1;
}
abi::Bool fSetDouble(abi::Handle pub, std::int64_t timeUs, double value) {
    record("NT_SetDouble", pub, timeUs).number = value;
    return g->refuseSets ? 0 : 1;
}
abi::Bool fSetString(abi::Handle pub, std::int64_t timeUs, const abi::String* value) {
    SetCall& call = record("NT_SetString", pub, timeUs);
    call.valueValid = valid(value);
    call.text = copy(value);
    return g->refuseSets ? 0 : 1;
}
abi::Bool fSetRaw(abi::Handle pub, std::int64_t timeUs, const std::uint8_t* value, std::size_t length) {
    SetCall& call = record("NT_SetRaw", pub, timeUs);
    call.valueValid = value != nullptr || length == 0;
    if (value != nullptr) call.raw.assign(value, value + length);
    return g->refuseSets ? 0 : 1;
}
abi::Bool fSetDoubleArray(abi::Handle pub, std::int64_t timeUs, const double* value, std::size_t length) {
    SetCall& call = record("NT_SetDoubleArray", pub, timeUs);
    call.valueValid = value != nullptr || length == 0;
    if (value != nullptr) call.doubles.assign(value, value + length);
    return g->refuseSets ? 0 : 1;
}

// Strings and arrays are malloc'ed like ntcore does it, and the dispose functions free them: a leak or a double free
// of what ntcore handed out shows up as a live entry or a bad disposal.
char* duplicate(const std::string& text) {
    char* copyOfText = static_cast<char*>(std::malloc(text.size() + 1));
    std::memcpy(copyOfText, text.c_str(), text.size() + 1);
    return copyOfText;
}
template <class T>
T* duplicate(const std::vector<T>& items) {
    T* out = static_cast<T*>(std::malloc((items.empty() ? 1 : items.size()) * sizeof(T)));
    if (!items.empty()) std::memcpy(out, items.data(), items.size() * sizeof(T));
    return out;
}

abi::Value* fReadQueueValue(abi::Handle sub, std::size_t* count) {
    ++g->calls;
    g->reads.push_back(sub);
    const std::size_t n = g->queue.size();
    *count = n;
    if (n == 0) return nullptr;  // ntcore returns NULL for an empty queue
    auto* values = static_cast<abi::Value*>(std::calloc(n, sizeof(abi::Value)));
    for (std::size_t i = 0; i < n; ++i) {
        const QueuedValue& q = g->queue[i];
        abi::Value& v = values[i];
        v.type = q.type;
        v.lastChange = q.timeUs;
        v.serverTime = q.timeUs + 1;
        switch (q.type) {
            case abi::Type::Boolean: v.data.boolean = q.integer != 0 ? 1 : 0; break;
            case abi::Type::Integer: v.data.integer = q.integer; break;
            case abi::Type::Float: v.data.float32 = static_cast<float>(q.number); break;
            case abi::Type::Double: v.data.float64 = q.number; break;
            case abi::Type::String:
                v.data.string.str = q.text.empty() ? nullptr : duplicate(q.text);  // ntcore: {NULL, 0} for ""
                v.data.string.len = q.text.size();
                break;
            case abi::Type::Raw:
                v.data.raw.data = duplicate(q.raw);
                v.data.raw.size = q.raw.size();
                break;
            case abi::Type::DoubleArray:
                v.data.doubles.arr = duplicate(q.doubles);
                v.data.doubles.size = q.doubles.size();
                break;
            case abi::Type::IntegerArray:
                v.data.integers.arr = duplicate(q.integers);
                v.data.integers.size = q.integers.size();
                break;
            case abi::Type::FloatArray:
                v.data.floats.arr = duplicate(q.floats);
                v.data.floats.size = q.floats.size();
                break;
            case abi::Type::BooleanArray:
                v.data.booleans.arr = static_cast<abi::Bool*>(std::calloc(2, sizeof(abi::Bool)));
                v.data.booleans.size = 2;
                break;
            case abi::Type::StringArray:
                v.data.strings.arr = static_cast<abi::String*>(std::calloc(1, sizeof(abi::String)));
                v.data.strings.arr[0].str = duplicate(std::string("s"));
                v.data.strings.arr[0].len = 1;
                v.data.strings.size = 1;
                break;
            case abi::Type::Unassigned:
            case abi::Type::Rpc: break;
        }
    }
    g->queue.clear();
    g->live.insert(values);
    return values;
}
void fDisposeValueArray(abi::Value* values, std::size_t count) {
    ++g->calls;
    if (g->live.erase(values) != 1) {
        ++g->badDisposals;
        return;
    }
    ++g->valueArraysDisposed;
    for (std::size_t i = 0; i < count; ++i) {
        abi::Value& v = values[i];
        switch (v.type) {
            case abi::Type::String: std::free(const_cast<char*>(v.data.string.str)); break;
            case abi::Type::Raw: std::free(v.data.raw.data); break;
            case abi::Type::DoubleArray: std::free(v.data.doubles.arr); break;
            case abi::Type::IntegerArray: std::free(v.data.integers.arr); break;
            case abi::Type::FloatArray: std::free(v.data.floats.arr); break;
            case abi::Type::BooleanArray: std::free(v.data.booleans.arr); break;
            case abi::Type::StringArray:
                for (std::size_t s = 0; s < v.data.strings.size; ++s) {
                    std::free(const_cast<char*>(v.data.strings.arr[s].str));
                }
                std::free(v.data.strings.arr);
                break;
            default: break;
        }
    }
    std::free(values);
}

abi::String owned(const std::string& text) {
    return abi::String{text.empty() ? nullptr : duplicate(text), text.size()};
}
abi::TopicInfo* fGetTopicInfos(abi::Handle inst, const abi::String* prefix, unsigned int types, std::size_t* count) {
    ++g->calls;
    if (inst != g->instance || !valid(prefix)) ++g->badArguments;
    const std::string wanted = copy(prefix);
    g->prefixes.push_back(wanted);
    g->typeMasks.push_back(types);
    std::vector<const TopicSpec*> matching;
    for (const TopicSpec& spec : g->topicList)
        if (spec.name.compare(0, wanted.size(), wanted) == 0) matching.push_back(&spec);
    *count = matching.size();
    if (matching.empty()) return nullptr;
    auto* infos = static_cast<abi::TopicInfo*>(std::calloc(matching.size(), sizeof(abi::TopicInfo)));
    for (std::size_t i = 0; i < matching.size(); ++i) {
        infos[i].topic = 1;
        infos[i].name = owned(matching[i]->name);
        infos[i].type = abi::Type::Double;
        infos[i].typeStr = owned(matching[i]->typeStr);
        infos[i].properties = owned(matching[i]->properties);
    }
    g->live.insert(infos);
    return infos;
}
void fDisposeTopicInfoArray(abi::TopicInfo* infos, std::size_t count) {
    ++g->calls;
    if (g->live.erase(infos) != 1) {
        ++g->badDisposals;
        return;
    }
    ++g->topicArraysDisposed;
    for (std::size_t i = 0; i < count; ++i) {
        std::free(const_cast<char*>(infos[i].name.str));
        std::free(const_cast<char*>(infos[i].typeStr.str));
        std::free(const_cast<char*>(infos[i].properties.str));
    }
    std::free(infos);
}

nb::NtFunctions FakeNtcore::table() {
    nb::NtFunctions t;
    t.NT_GetDefaultInstance = &fGetDefaultInstance;
    t.NT_Now = &fNow;
    t.NT_GetTopic = &fGetTopic;
    t.NT_GetTopicExists = &fGetTopicExists;
    t.NT_GetTopicInfos = &fGetTopicInfos;
    t.NT_DisposeTopicInfoArray = &fDisposeTopicInfoArray;
    t.NT_PublishEx = &fPublishEx;
    t.NT_Subscribe = &fSubscribe;
    t.NT_SetTopicProperty = &fSetTopicProperty;
    t.NT_SetBoolean = &fSetBoolean;
    t.NT_SetInteger = &fSetInteger;
    t.NT_SetDouble = &fSetDouble;
    t.NT_SetString = &fSetString;
    t.NT_SetRaw = &fSetRaw;
    t.NT_SetDoubleArray = &fSetDoubleArray;
    t.NT_ReadQueueValue = &fReadQueueValue;
    t.NT_DisposeValueArray = &fDisposeValueArray;
    return t;
}

// ---- helpers -------------------------------------------------------------------------------------------------------

nb::BindOutcome bound(const char* module = "fake_ntcore.dll") {
    nb::BindOutcome outcome;
    outcome.kind = nb::BindOutcome::Kind::Bound;
    outcome.module = module;
    return outcome;
}

nb::BindOutcome notLoaded() {
    nb::BindOutcome outcome;
    outcome.kind = nb::BindOutcome::Kind::ModuleNotLoaded;
    outcome.module = "fake_ntcore.dll";
    return outcome;
}

// An API bound to the fake right away.
struct Rig {
    FakeNtcore fake;
    ActiveFake active{fake};
    nb::NtCoreApi api{[this](nb::NtFunctions& out) {
        out = fake.table();
        return bound();
    }};
};

NtPublishOptions options(std::string_view typeString = {}, std::string_view propertiesJson = {}) {
    NtPublishOptions o;
    o.typeString = typeString;
    o.propertiesJson = propertiesJson;
    return o;
}

// ---- binding -----------------------------------------------------------------------------------------------------

TEST_CASE("nt binding: ntcore is bound lazily, retried until it succeeds, and never un-bound") {
    FakeNtcore fake;
    ActiveFake active(fake);
    int attempts = 0;
    int failuresLeft = 3;  // the Java robot maps ntcore a few polls after the extension loaded
    bool fail = true;
    nb::NtCoreApi api([&](nb::NtFunctions& out) {
        ++attempts;
        if (failuresLeft > 0) {
            --failuresLeft;
            return notLoaded();
        }
        if (!fail) return notLoaded();
        out = fake.table();
        return bound();
    });
    CHECK(attempts == 0);  // nothing happens before the first call

    CHECK_FALSE(api.available());
    CHECK(attempts == 1);
    const std::string reason(api.unavailableReason());  // asks once more
    CHECK(reason.find("fake_ntcore.dll") != std::string::npos);
    CHECK(reason.find("not loaded") != std::string::npos);
    CHECK_FALSE(api.available());
    CHECK(attempts == 3);

    CHECK(api.available());  // the fourth attempt finds ntcore
    CHECK(attempts == 4);
    CHECK(api.unavailableReason().empty());
    CHECK(api.nowUs() == fake.now);

    fail = false;  // from here on the binder would fail: it must not even be asked
    CHECK(api.available());
    CHECK(api.nowUs() == fake.now);
    CHECK(attempts == 4);
}

TEST_CASE("nt binding: a missing symbol keeps it unavailable and is named in the reason") {
    FakeNtcore fake;
    ActiveFake active(fake);
    const nb::NtFunctions complete = fake.table();
    bool omit = true;
    // The real binder resolves names through bindSymbols: here the "module" lacks NT_SetRaw and NT_Now.
    nb::NtCoreApi api([&](nb::NtFunctions& out) {
        return nb::bindSymbols(out, "fake_ntcore.dll", [&](const char* name) -> void* {
            const std::string symbol = name;
            if (omit && (symbol == "NT_SetRaw" || symbol == "NT_Now")) return nullptr;
            void* found = nullptr;
            nb::NtFunctions copyOfTable = complete;
            copyOfTable.visit([&](const char* entryName, auto& entry) {
                if (symbol == entryName) found = reinterpret_cast<void*>(entry);
            });
            return found;
        });
    });
    CHECK_FALSE(api.available());
    const std::string reason(api.unavailableReason());
    CHECK(reason.find("NT_Now") != std::string::npos);  // the first missing symbol of the table, in list order
    CHECK(reason.find("fake_ntcore.dll") != std::string::npos);
    CHECK_FALSE(api.publish("/t", NtType::Double, options()));
    CHECK(fake.calls == 0);

    omit = false;  // ntcore turns out to be complete after all
    CHECK(api.available());
    CHECK(api.publish("/t", NtType::Double, options()));
}

TEST_CASE("nt binding: bindSymbols fills nothing unless every symbol resolves") {
    FakeNtcore fake;
    ActiveFake active(fake);
    nb::NtFunctions out;
    const nb::BindOutcome failed = nb::bindSymbols(out, "m.dll", [&](const char* name) -> void* {
        return std::string(name) == "NT_PublishEx" ? nullptr : reinterpret_cast<void*>(&fGetDefaultInstance);
    });
    CHECK(failed.kind == nb::BindOutcome::Kind::SymbolMissing);
    CHECK(std::string(failed.symbol) == "NT_PublishEx");
    CHECK(std::string(failed.module) == "m.dll");
    CHECK(out.NT_GetDefaultInstance == nullptr);  // no half-bound table
    CHECK(out.firstMissing() != nullptr);
}

// A module every process of the platform has loaded: it is found, and it has no ntcore symbols.
#if defined(_WIN32)
constexpr const char* kSystemModule = "kernel32.dll";
#elif defined(__APPLE__)
constexpr const char* kSystemModule = "libSystem.B.dylib";
#else
constexpr const char* kSystemModule = "libc.so.6";
#endif

TEST_CASE("nt binding: the process binder tells an absent module from a module without ntcore symbols") {
    nb::NtFunctions out;
    const nb::BindOutcome absent = nb::bindProcessModule(out, "wpiemu_no_such_module.dll");
    CHECK(absent.kind == nb::BindOutcome::Kind::ModuleNotLoaded);
    CHECK(std::string(absent.module) == "wpiemu_no_such_module.dll");

    const nb::BindOutcome foreign = nb::bindProcessModule(out, kSystemModule);
    CHECK(foreign.kind == nb::BindOutcome::Kind::SymbolMissing);
    CHECK(std::string(foreign.module) == kSystemModule);
    CHECK(std::string(foreign.symbol) == "NT_GetDefaultInstance");
    CHECK(out.firstMissing() != nullptr);  // nothing was stored

    // Through NtCoreApi both end up as an unavailable API with a reason that says which one it is.
    nb::NtCoreApi missingModule(
        [](nb::NtFunctions& table) { return nb::bindProcessModule(table, "wpiemu_no_such_module.dll"); });
    CHECK_FALSE(missingModule.available());
    CHECK(missingModule.unavailableReason().find("wpiemu_no_such_module.dll") != std::string_view::npos);
    nb::NtCoreApi wrongModule([](nb::NtFunctions& table) { return nb::bindProcessModule(table, kSystemModule); });
    CHECK_FALSE(wrongModule.available());
    CHECK(wrongModule.unavailableReason().find("does not export NT_GetDefaultInstance") != std::string_view::npos);
}

TEST_CASE("nt binding: a table with a hole or without a default instance is not accepted") {
    FakeNtcore fake;
    ActiveFake active(fake);
    bool leaveHole = true;
    nb::NtCoreApi holey([&](nb::NtFunctions& out) {
        out = fake.table();
        if (leaveHole) out.NT_Subscribe = nullptr;
        return bound();
    });
    CHECK_FALSE(holey.available());
    CHECK(holey.unavailableReason().find("NT_Subscribe") != std::string_view::npos);
    leaveHole = false;
    CHECK(holey.available());

    nb::NtCoreApi noInstance([&](nb::NtFunctions& out) {
        out = fake.table();
        return bound();
    });
    fake.instance = 0;  // ntcore cannot hand out its default instance yet
    CHECK_FALSE(noInstance.available());
    CHECK_FALSE(noInstance.unavailableReason().empty());
    CHECK_FALSE(noInstance.publish("/t", NtType::Double, options()));
    fake.instance = 5;
    CHECK(noInstance.available());
    CHECK(noInstance.publish("/t", NtType::Double, options()));
}

TEST_CASE("nt binding: while unavailable every call is a no-op that never reaches ntcore") {
    FakeNtcore fake;
    ActiveFake active(fake);
    nb::NtCoreApi api([](nb::NtFunctions&) { return notLoaded(); });
    const std::uint8_t bytes[] = {1, 2, 3};
    const double numbers[] = {1.0, 2.0};
    std::vector<NtValue> values;
    std::vector<NtTopicInfo> topics;

    CHECK_FALSE(api.available());
    const std::string_view reason = api.unavailableReason();
    CHECK_FALSE(reason.empty());
    CHECK(api.nowUs() == 0);
    CHECK_FALSE(api.publish("/t", NtType::Double, options()));
    CHECK_FALSE(api.setBoolean({}, 1, true));
    CHECK_FALSE(api.setInteger(NtPublisher{1}, 1, 1));
    CHECK_FALSE(api.setDouble(NtPublisher{1}, 1, 1.0));
    CHECK_FALSE(api.setString(NtPublisher{1}, 1, "x"));
    CHECK_FALSE(api.setRaw(NtPublisher{1}, 1, bytes));
    CHECK_FALSE(api.setDoubleArray(NtPublisher{1}, 1, numbers));
    CHECK_FALSE(api.setTopicProperty("/t", "p", "1"));
    CHECK_FALSE(api.subscribe("/t", kDefaultNtQueueDepth));
    api.read(NtSubscriber{1}, values);
    api.topics("/", topics);
    CHECK_FALSE(api.hasForeignPublisher("/t"));
    CHECK(values.empty());
    CHECK(topics.empty());
    CHECK(fake.calls == 0);

    // The same failure again must not rebuild the reason (that would allocate): the text stays where it is.
    for (int i = 0; i < 5; ++i) CHECK_FALSE(api.available());
    CHECK(api.unavailableReason().data() == reason.data());
}

// ---- publishing -------------------------------------------------------------------------------------------------

TEST_CASE("nt binding: publish passes a complete, never-NULL option set and the topic, type and properties as given") {
    Rig rig;
    const std::string backing = "/photonvision/cam/rawBytes-and-trailing-text";
    const std::string_view topic(backing.data(), std::string_view("/photonvision/cam/rawBytes").size());  // no NUL
    NtPublishOptions o = options("photonstruct:PhotonPipelineResult:4b2f", R"({"message_uuid":"4b2f"})");
    o.sendAll = true;
    o.keepDuplicates = true;
    o.periodicS = 0.01;

    const NtPublisher pub = rig.api.publish(topic, NtType::Raw, o);
    REQUIRE(pub);
    REQUIRE(rig.fake.publishes.size() == 1);
    const PublishCall& call = rig.fake.publishes[0];
    CHECK(rig.fake.topicLookups == std::vector<std::string>{"/photonvision/cam/rawBytes"});
    CHECK(rig.fake.nameOf(call.topic) == "/photonvision/cam/rawBytes");
    CHECK(call.type == abi::Type::Raw);
    CHECK(call.typeStrValid);
    CHECK(call.typeStr == "photonstruct:PhotonPipelineResult:4b2f");
    CHECK(call.propertiesValid);
    CHECK(call.properties == R"({"message_uuid":"4b2f"})");  // JSON text, passed through untouched
    REQUIRE(call.optionsValid);
    CHECK(call.options.structSize == sizeof(abi::PubSubOptions));
    CHECK(call.options.sendAll == 1);
    CHECK(call.options.keepDuplicates == 1);
    CHECK(call.options.periodic == 0.01);
    CHECK(call.options.pollStorage == 0);
    CHECK(call.options.excludePublisher == 0);
    CHECK(call.options.topicsOnly == 0);
    CHECK(call.options.prefixMatch == 0);
    CHECK(call.options.disableRemote == 0);
    CHECK(call.options.disableLocal == 0);
    CHECK(call.options.excludeSelf == 0);
    CHECK(call.options.hidden == 0);
    CHECK(rig.fake.badArguments == 0);

    // Defaults: nothing is queued or kept, and "no properties" is an empty string, not a NULL pointer.
    REQUIRE(rig.api.publish("/plain", NtType::Double, options()));
    const PublishCall& plain = rig.fake.publishes[1];
    CHECK(plain.propertiesValid);
    CHECK(plain.properties.empty());
    CHECK(plain.typeStrValid);
    REQUIRE(plain.optionsValid);
    CHECK(plain.options.structSize == sizeof(abi::PubSubOptions));
    CHECK(plain.options.sendAll == 0);
    CHECK(plain.options.keepDuplicates == 0);
    CHECK(plain.options.periodic == 0.0);
}

TEST_CASE("nt binding: every NtType gets its canonical type string and the matching ntcore type") {
    Rig rig;
    struct Expected {
        NtType type;
        abi::Type native;
        const char* typeString;
    };
    const Expected expected[] = {
        {NtType::Boolean, abi::Type::Boolean, "boolean"},   {NtType::Integer, abi::Type::Integer, "int"},
        {NtType::Double, abi::Type::Double, "double"},      {NtType::String, abi::Type::String, "string"},
        {NtType::Raw, abi::Type::Raw, "raw"},               {NtType::DoubleArray, abi::Type::DoubleArray, "double[]"},
    };
    for (const Expected& e : expected) REQUIRE(rig.api.publish("/t/" + std::string(e.typeString), e.type, options()));
    REQUIRE(rig.fake.publishes.size() == 6);
    for (std::size_t i = 0; i < 6; ++i) {
        CHECK(rig.fake.publishes[i].type == expected[i].native);
        CHECK(rig.fake.publishes[i].typeStr == expected[i].typeString);
    }
}

TEST_CASE("nt binding: ntcore rejecting a publish gives an invalid handle and leaves the topic foreign") {
    Rig rig;
    rig.fake.existing.insert("/bad");
    rig.fake.rejectPublish = true;  // what ntcore does for properties that are not JSON
    CHECK_FALSE(rig.api.publish("/bad", NtType::Raw, options("photonstruct:X:1", R"({"message_uuid":abc})")));
    CHECK(rig.fake.publishes.size() == 1);
    CHECK(rig.api.hasForeignPublisher("/bad"));  // the failed call published nothing: still somebody else's topic
    CHECK_FALSE(rig.api.setRaw(NtPublisher{1}, 1, {}));
}

TEST_CASE("nt binding: publish sanitizes the period and refuses an empty topic or an unknown type") {
    Rig rig;
    const double periods[] = {-1.0, std::numeric_limits<double>::quiet_NaN(), -0.0, 5000.0, 0.25};
    for (const double period : periods) {
        NtPublishOptions o;
        o.periodicS = period;
        REQUIRE(rig.api.publish("/p", NtType::Double, o));
    }
    REQUIRE(rig.fake.publishes.size() == 5);
    CHECK(rig.fake.publishes[0].options.periodic == 0.0);  // ntcore turns a negative period into undefined behaviour
    CHECK(rig.fake.publishes[1].options.periodic == 0.0);
    CHECK(rig.fake.publishes[2].options.periodic == 0.0);
    CHECK(rig.fake.publishes[3].options.periodic == 1000.0);
    CHECK(rig.fake.publishes[4].options.periodic == 0.25);

    const int callsBefore = rig.fake.calls;
    CHECK_FALSE(rig.api.publish("", NtType::Double, options()));  // ntcore has no unnamed topics
    CHECK_FALSE(rig.api.publish("/p", static_cast<NtType>(200), options()));
    CHECK(rig.fake.calls == callsBefore);
}

// ---- setters ----------------------------------------------------------------------------------------------------

TEST_CASE("nt binding: setters marshal their values with the publisher's handle and timestamp") {
    Rig rig;
    const NtPublisher b = rig.api.publish("/b", NtType::Boolean, options());
    const NtPublisher i = rig.api.publish("/i", NtType::Integer, options());
    const NtPublisher d = rig.api.publish("/d", NtType::Double, options());
    const NtPublisher s = rig.api.publish("/s", NtType::String, options());
    const NtPublisher r = rig.api.publish("/r", NtType::Raw, options());
    const NtPublisher a = rig.api.publish("/a", NtType::DoubleArray, options());
    REQUIRE((b && i && d && s && r && a));
    const abi::Handle firstHandle = 5001;  // what the fake's NT_PublishEx hands out, in order

    const std::string withNul("a\0b", 3);
    const std::uint8_t bytes[] = {0x00, 0xFF, 0x7F};
    const double numbers[] = {1.5, -2.5, 1e300};
    CHECK(rig.api.setBoolean(b, 1001, true));
    CHECK(rig.api.setBoolean(b, 1002, false));
    CHECK(rig.api.setInteger(i, 1003, -9007199254740993LL));
    CHECK(rig.api.setDouble(d, 1004, 3.25));
    CHECK(rig.api.setString(s, 1005, withNul));
    CHECK(rig.api.setString(s, 1006, ""));
    CHECK(rig.api.setRaw(r, 1007, bytes));
    CHECK(rig.api.setRaw(r, 1008, {}));
    CHECK(rig.api.setDoubleArray(a, 1009, numbers));
    CHECK(rig.api.setDoubleArray(a, 1010, {}));

    REQUIRE(rig.fake.sets.size() == 10);
    const auto& sets = rig.fake.sets;
    CHECK(sets[0].function == "NT_SetBoolean");
    CHECK(sets[0].pub == firstHandle);
    CHECK(sets[0].timeUs == 1001);
    CHECK(sets[0].integer == 1);
    CHECK(sets[1].integer == 0);
    CHECK(sets[2].function == "NT_SetInteger");
    CHECK(sets[2].pub == firstHandle + 1);
    CHECK(sets[2].integer == -9007199254740993LL);  // 64 bit, not squeezed through a double
    CHECK(sets[3].function == "NT_SetDouble");
    CHECK(sets[3].pub == firstHandle + 2);
    CHECK(sets[3].timeUs == 1004);
    CHECK(sets[3].number == 3.25);
    CHECK(sets[4].function == "NT_SetString");
    CHECK(sets[4].pub == firstHandle + 3);
    CHECK(sets[4].text == withNul);  // length delimited: an embedded NUL survives
    CHECK(sets[5].valueValid);       // "" is {"", 0}, never a NULL string
    CHECK(sets[5].text.empty());
    CHECK(sets[6].function == "NT_SetRaw");
    CHECK(sets[6].pub == firstHandle + 4);
    CHECK(sets[6].raw == std::vector<std::uint8_t>(std::begin(bytes), std::end(bytes)));
    CHECK(sets[7].valueValid);
    CHECK(sets[7].raw.empty());
    CHECK(sets[8].function == "NT_SetDoubleArray");
    CHECK(sets[8].pub == firstHandle + 5);
    CHECK(sets[8].doubles == std::vector<double>(std::begin(numbers), std::end(numbers)));
    CHECK(sets[9].valueValid);
    CHECK(sets[9].doubles.empty());
    CHECK(rig.fake.badArguments == 0);

    rig.fake.refuseSets = true;  // the verdict of ntcore is the verdict of the call
    CHECK_FALSE(rig.api.setDouble(d, 1011, 1.0));
    CHECK_FALSE(rig.api.setBoolean(b, 1012, true));
}

TEST_CASE("nt binding: setters refuse a wrong type or a bad handle without calling ntcore") {
    Rig rig;
    const NtPublisher integer = rig.api.publish("/i", NtType::Integer, options());
    const NtPublisher doubles = rig.api.publish("/d", NtType::Double, options());
    REQUIRE((integer && doubles));
    const int callsBefore = rig.fake.calls;
    const std::uint8_t bytes[] = {1};
    const double numbers[] = {1.0};

    // ntcore itself would quietly convert between the numeric types: the contract says the call fails.
    CHECK_FALSE(rig.api.setDouble(integer, 1, 1.0));
    CHECK_FALSE(rig.api.setInteger(doubles, 1, 1));
    CHECK_FALSE(rig.api.setBoolean(integer, 1, true));
    CHECK_FALSE(rig.api.setString(doubles, 1, "x"));
    CHECK_FALSE(rig.api.setRaw(doubles, 1, bytes));
    CHECK_FALSE(rig.api.setDoubleArray(doubles, 1, numbers));
    CHECK_FALSE(rig.api.setDouble(NtPublisher{}, 1, 1.0));                // invalid
    CHECK_FALSE(rig.api.setDouble(NtPublisher{99}, 1, 1.0));              // never handed out
    CHECK_FALSE(rig.api.setDouble(NtPublisher{0xFFFFFFFFu}, 1, 1.0));
    CHECK(rig.fake.calls == callsBefore);

    CHECK(rig.api.setInteger(integer, 2, 5));
    CHECK(rig.api.setDouble(doubles, 2, 5.0));
}

TEST_CASE("nt binding: setTopicProperty passes JSON text through and reports ntcore's verdict") {
    Rig rig;
    CHECK(rig.api.setTopicProperty("/photonvision/cam/rawBytes", "message_uuid", "\"4b2f\""));
    REQUIRE(rig.fake.properties.size() == 1);
    CHECK(rig.fake.nameOf(rig.fake.properties[0].topic) == "/photonvision/cam/rawBytes");
    CHECK(rig.fake.properties[0].name == "message_uuid");
    CHECK(rig.fake.properties[0].value == "\"4b2f\"");  // quoted: the value is JSON text
    CHECK(rig.fake.badArguments == 0);

    rig.fake.rejectProperty = true;
    CHECK_FALSE(rig.api.setTopicProperty("/t", "message_uuid", "unquoted"));
    CHECK(rig.fake.properties.size() == 2);

    const int callsBefore = rig.fake.calls;
    CHECK_FALSE(rig.api.setTopicProperty("", "p", "1"));
    CHECK(rig.fake.calls == callsBefore);
}

// ---- reading ----------------------------------------------------------------------------------------------------

TEST_CASE("nt binding: subscribe is type-unfiltered, queues every update and never passes NULL") {
    Rig rig;
    const NtSubscriber sub = rig.api.subscribe("/limelight/robot_orientation_set", 64);
    REQUIRE(sub);
    REQUIRE(rig.fake.subscriptions.size() == 1);
    const SubscribeCall& call = rig.fake.subscriptions[0];
    CHECK(rig.fake.nameOf(call.topic) == "/limelight/robot_orientation_set");
    CHECK(call.type == abi::Type::Unassigned);  // whatever type the robot publishes is delivered
    CHECK(call.typeStrValid);
    CHECK(call.typeStr.empty());
    REQUIRE(call.optionsValid);
    CHECK(call.options.structSize == sizeof(abi::PubSubOptions));
    CHECK(call.options.pollStorage == 64);
    CHECK(call.options.sendAll == 1);
    CHECK(call.options.keepDuplicates == 1);
    CHECK(call.options.hidden == 0);
    CHECK(call.options.topicsOnly == 0);
    CHECK(rig.fake.badArguments == 0);

    // A queue of 0 would mean "ntcore's default"; the API's contract is "up to queueDepth updates, at least one".
    REQUIRE(rig.api.subscribe("/a", 0));
    CHECK(rig.fake.subscriptions[1].options.pollStorage == 1);
    REQUIRE(rig.api.subscribe("/b", std::numeric_limits<std::size_t>::max()));
    CHECK(rig.fake.subscriptions[2].options.pollStorage == nb::kMaxQueueDepth);

    rig.fake.rejectSubscribe = true;
    CHECK_FALSE(rig.api.subscribe("/c", 8));
    CHECK_FALSE(rig.api.subscribe("", 8));
}

QueuedValue number(abi::Type type, std::int64_t timeUs, double value, std::int64_t integer = 0) {
    QueuedValue q;
    q.type = type;
    q.timeUs = timeUs;
    q.number = value;
    q.integer = integer;
    return q;
}

TEST_CASE("nt binding: read converts every representable value, skips the rest and disposes the array once") {
    Rig rig;
    const NtSubscriber sub = rig.api.subscribe("/t", kDefaultNtQueueDepth);
    REQUIRE(sub);

    QueuedValue text;
    text.type = abi::Type::String;
    text.timeUs = 15;
    text.text = "hello";
    QueuedValue emptyText = text;
    emptyText.timeUs = 16;
    emptyText.text.clear();
    QueuedValue nulText = text;
    nulText.timeUs = 17;
    nulText.text = std::string("a\0b", 3);
    QueuedValue raw;
    raw.type = abi::Type::Raw;
    raw.timeUs = 18;
    raw.raw = {0x00, 0xFF, 0x01};
    QueuedValue emptyRaw = raw;
    emptyRaw.timeUs = 19;
    emptyRaw.raw.clear();
    QueuedValue doubles;
    doubles.type = abi::Type::DoubleArray;
    doubles.timeUs = 20;
    doubles.doubles = {1.0, 2.0, 3.0};
    QueuedValue emptyDoubles = doubles;
    emptyDoubles.timeUs = 21;
    emptyDoubles.doubles.clear();
    QueuedValue integers;
    integers.type = abi::Type::IntegerArray;
    integers.timeUs = 22;
    integers.integers = {7, -8};
    QueuedValue floats;
    floats.type = abi::Type::FloatArray;
    floats.timeUs = 23;
    floats.floats = {0.5f, -1.5f};

    rig.fake.queue = {
        number(abi::Type::Unassigned, 0, 0.0),  // the empty value ntcore queues for a topic that has none
        number(abi::Type::Boolean, 10, 0.0, 1),
        number(abi::Type::Boolean, 11, 0.0, 0),
        number(abi::Type::Integer, 12, 0.0, -42),
        number(abi::Type::Float, 13, 1.5),
        number(abi::Type::Double, 14, 2.25),
        text,
        emptyText,
        nulText,
        raw,
        emptyRaw,
        doubles,
        emptyDoubles,
        integers,
        floats,
        number(abi::Type::BooleanArray, 24, 0.0),
        number(abi::Type::StringArray, 25, 0.0),
    };

    std::vector<NtValue> out;
    NtValue previous;
    previous.text = "kept";
    out.push_back(previous);  // read() appends, it never clears
    rig.api.read(sub, out);
    REQUIRE(out.size() == 1 + 14);  // boolean[] and string[] cannot be represented; the empty initial value is skipped
    CHECK(out[0].text == "kept");
    const auto at = [&](std::size_t k) -> const NtValue& { return out[k]; };
    CHECK(at(1).type == NtType::Boolean);
    CHECK(at(1).number == 1.0);
    CHECK(at(1).timeUs == 10);
    CHECK(at(2).number == 0.0);
    CHECK(at(3).type == NtType::Integer);
    CHECK(at(3).number == -42.0);
    CHECK(at(4).type == NtType::Double);  // float widens to double
    CHECK(at(4).number == 1.5);
    CHECK(at(5).type == NtType::Double);
    CHECK(at(5).number == 2.25);
    CHECK(at(5).timeUs == 14);
    CHECK(at(6).type == NtType::String);
    CHECK(at(6).text == "hello");
    CHECK(at(7).text.empty());  // ntcore's {NULL, 0}
    CHECK(at(8).text == std::string("a\0b", 3));
    CHECK(at(9).type == NtType::Raw);
    CHECK(at(9).raw == std::vector<std::uint8_t>{0x00, 0xFF, 0x01});
    CHECK(at(10).type == NtType::Raw);
    CHECK(at(10).raw.empty());
    CHECK(at(11).type == NtType::DoubleArray);
    CHECK(at(11).numbers == std::vector<double>{1.0, 2.0, 3.0});
    CHECK(at(12).numbers.empty());
    CHECK(at(13).type == NtType::DoubleArray);  // integer arrays widen
    CHECK(at(13).numbers == std::vector<double>{7.0, -8.0});
    CHECK(at(14).numbers == std::vector<double>{0.5, -1.5});

    CHECK(rig.fake.reads == std::vector<abi::Handle>{9001});
    CHECK(rig.fake.valueArraysDisposed == 1);
    CHECK(rig.fake.badDisposals == 0);
    CHECK(rig.fake.live.empty());
}

TEST_CASE("nt binding: read disposes an array whose entries are all skipped, and ignores bad subscribers") {
    Rig rig;
    const NtSubscriber sub = rig.api.subscribe("/t", kDefaultNtQueueDepth);
    REQUIRE(sub);
    rig.fake.queue = {number(abi::Type::Unassigned, 0, 0.0)};
    std::vector<NtValue> out;
    rig.api.read(sub, out);
    CHECK(out.empty());
    CHECK(rig.fake.valueArraysDisposed == 1);  // still handed out by ntcore, still ours to free
    CHECK(rig.fake.live.empty());

    rig.api.read(sub, out);  // an empty queue is NULL: nothing to free
    CHECK(rig.fake.valueArraysDisposed == 1);
    CHECK(rig.fake.badDisposals == 0);

    const int callsBefore = rig.fake.calls;
    rig.api.read(NtSubscriber{}, out);
    rig.api.read(NtSubscriber{42}, out);
    CHECK(rig.fake.calls == callsBefore);
    CHECK(out.empty());
}

// ---- discovery --------------------------------------------------------------------------------------------------

TEST_CASE("nt binding: topics are copied, sorted by name and the array is disposed exactly once") {
    Rig rig;
    rig.fake.topicList = {
        {"/limelight/tv", "double", "{}"},
        {"/photonvision/cam/rawBytes", "photonstruct:PhotonPipelineResult:4b2f", R"({"message_uuid":"4b2f"})"},
        {"/photonvision/cam/driverModeRequest", "boolean", ""},  // ntcore always sends "{}", but be safe
        {"/photonvision/version", "string", "{}"},
    };
    std::vector<NtTopicInfo> out;
    NtTopicInfo previous;
    previous.name = "/kept";
    out.push_back(previous);
    rig.api.topics("/photonvision/", out);
    REQUIRE(out.size() == 1 + 3);
    CHECK(out[0].name == "/kept");  // appended after what was there, and only the new part is sorted
    CHECK(out[1].name == "/photonvision/cam/driverModeRequest");
    CHECK(out[1].typeString == "boolean");
    CHECK(out[1].propertiesJson == "{}");
    CHECK(out[2].name == "/photonvision/cam/rawBytes");
    CHECK(out[2].typeString == "photonstruct:PhotonPipelineResult:4b2f");
    CHECK(out[2].propertiesJson == R"({"message_uuid":"4b2f"})");
    CHECK(out[3].name == "/photonvision/version");
    CHECK(rig.fake.prefixes == std::vector<std::string>{"/photonvision/"});
    CHECK(rig.fake.typeMasks == std::vector<unsigned int>{0});  // any type
    CHECK(rig.fake.topicArraysDisposed == 1);

    out.clear();
    rig.api.topics("", out);  // everything
    CHECK(out.size() == 4);
    CHECK(out.front().name == "/limelight/tv");
    CHECK(rig.fake.topicArraysDisposed == 2);

    out.clear();
    rig.api.topics("/nothing/", out);  // ntcore returns NULL: nothing to free
    CHECK(out.empty());
    CHECK(rig.fake.topicArraysDisposed == 2);
    CHECK(rig.fake.badDisposals == 0);
    CHECK(rig.fake.live.empty());
    CHECK(rig.fake.badArguments == 0);
}

TEST_CASE("nt binding: only a topic that exists and that we did not publish has a foreign publisher") {
    Rig rig;
    rig.fake.existing = {"/robot/topic", "/ours"};
    CHECK(rig.api.hasForeignPublisher("/robot/topic"));
    CHECK_FALSE(rig.api.hasForeignPublisher("/nobody/yet"));  // a topic nobody publishes does not exist
    CHECK_FALSE(rig.api.hasForeignPublisher(""));

    REQUIRE(rig.api.publish("/ours", NtType::Double, options()));
    CHECK_FALSE(rig.api.hasForeignPublisher("/ours"));  // ntcore cannot tell our publisher from another one

    REQUIRE(rig.api.publish("/robot/topic", NtType::Double, options()));  // we joined the robot's topic
    CHECK_FALSE(rig.api.hasForeignPublisher("/robot/topic"));

    REQUIRE(rig.api.subscribe("/nobody/yet", 4));  // a subscriber alone does not make it exist either
    CHECK_FALSE(rig.api.hasForeignPublisher("/nobody/yet"));
}

TEST_CASE("nt binding: nowUs is the ntcore clock") {
    Rig rig;
    rig.fake.now = 42'000'000;
    CHECK(rig.api.nowUs() == 42'000'000);
    rig.fake.now = 42'000'250;
    CHECK(rig.api.nowUs() == 42'000'250);
}

}  // namespace
