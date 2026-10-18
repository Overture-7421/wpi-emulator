#pragma once

// NtCoreApi: the real INtApi (internal to src/binding; makeNtCoreApi() in nt_binding.h is the way to get one). It
// turns the SDK-facing calls into ntcore C calls through an NtFunctions table, and nothing else: no vendor knowledge.
//
// Rules it implements (nt_api.h and the nt spike, files/spike-nt-findings.md sections 2-3):
//  * options and properties pointers are never NULL (ntcore dereferences them): zero-initialised option structs with
//    structSize set, WPI_String{"", 0} for "none";
//  * property values are JSON text, passed through as given; ntcore rejects malformed text (invalid handle / false);
//  * subscriptions are type-unfiltered; the empty initial value ntcore queues for a topic without one is skipped;
//  * returned arrays are disposed exactly once (NT_DisposeValueArray / NT_DisposeTopicInfoArray);
//  * setters check the publisher's NtType themselves (ntcore would silently convert between numeric types);
//  * a topic this instance published is never "foreign": ntcore cannot tell the publishers of a topic apart;
//  * handles are never released (no NT_Unpublish / NT_Unsubscribe, no dlclose): ntcore may be torn down at any time
//    while the robot exits, so this class must not call into it from a destructor.
//
// Bind from the extension's poll() once the robot program has started: by then ntcore (which a Java robot maps a few
// milliseconds after HALSIM_InitExtension returns) is fully initialised, whatever the language of the robot.

#include <cstddef>
#include <cstdint>
#include <functional>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "nt_binder.h"
#include "wpiemu/bridges/vision/nt_api.h"

namespace wpiemu::bridges::vision::binding {

// What unavailableReason() says when this build has no ntcore headers to pin the ABI with.
inline constexpr std::string_view kBuiltWithoutHeadersReason =
    "built without the ntcore headers (stage WPILib: -DWPIEMU_WPILIB_ROOT=<maven cache or extracted root> or "
    "-DWPIEMU_WPILIB_DOWNLOAD=ON)";

// ntcore allocates one queue slot per update up front: subscribe() caps the depth so that a caller asking for an
// absurd one cannot take the robot down with an allocation failure.
inline constexpr std::size_t kMaxQueueDepth = 4096;

class NtCoreApi final : public INtApi {
public:
    // Fills the table when the robot's ntcore can be reached. It is called again on every available() (and on every
    // call that needs ntcore) until it succeeds; once bound the API never un-binds.
    using Binder = std::function<BindOutcome(NtFunctions&)>;
    explicit NtCoreApi(Binder binder);

    bool available() const override;
    std::string_view unavailableReason() const override;
    std::int64_t nowUs() const override;

    NtPublisher publish(std::string_view topic, NtType type, const NtPublishOptions& options) override;
    bool setBoolean(NtPublisher pub, std::int64_t timeUs, bool value) override;
    bool setInteger(NtPublisher pub, std::int64_t timeUs, std::int64_t value) override;
    bool setDouble(NtPublisher pub, std::int64_t timeUs, double value) override;
    bool setString(NtPublisher pub, std::int64_t timeUs, std::string_view value) override;
    bool setRaw(NtPublisher pub, std::int64_t timeUs, std::span<const std::uint8_t> value) override;
    bool setDoubleArray(NtPublisher pub, std::int64_t timeUs, std::span<const double> value) override;
    bool setTopicProperty(std::string_view topic, std::string_view name, std::string_view jsonValue) override;

    NtSubscriber subscribe(std::string_view topic, std::size_t queueDepth) override;
    void read(NtSubscriber sub, std::vector<NtValue>& out) override;

    void topics(std::string_view prefix, std::vector<NtTopicInfo>& out) override;
    bool hasForeignPublisher(std::string_view topic) override;

private:
    struct PublisherRecord {
        abi::Handle handle = 0;
        NtType type = NtType::Double;
    };

    bool ready() const { return bound_ || tryBind(); }
    bool tryBind() const;
    void rememberFailure(const BindOutcome& outcome) const;
    const PublisherRecord* publisherFor(NtPublisher pub, NtType type) const;
    abi::Handle topicHandle(std::string_view topic) const;

    Binder binder_;
    mutable NtFunctions fn_;
    mutable bool bound_ = false;
    mutable abi::Handle instance_ = 0;
    // Why the last attempt failed. Rebuilt only when the cause changes: repeating a failed attempt allocates nothing.
    mutable std::string reason_;
    mutable BindOutcome::Kind lastKind_ = BindOutcome::Kind::Bound;
    mutable std::string_view lastModule_;
    mutable std::string_view lastSymbol_;

    std::vector<PublisherRecord> publishers_;  // NtPublisher::id - 1
    std::vector<abi::Handle> subscribers_;     // NtSubscriber::id - 1
    std::set<std::string, std::less<>> ownTopics_;
};

}  // namespace wpiemu::bridges::vision::binding
