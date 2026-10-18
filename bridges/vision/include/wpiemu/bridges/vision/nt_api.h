#pragma once

// NetworkTables access seam of the vision bridge (ADR 0017).
//
// The robot process already runs ntcore (WPILib's NetworkTables library): the vision bridge publishes
// the entries a PhotonVision / Limelight coprocessor would publish, and reads what the robot writes to
// them, through this interface. Everything vendor specific (topic names, message layouts) lives in the
// publishers on top of it; nothing above this header knows about the ntcore C API, and nothing below it
// knows about cameras:
//
//   publishers (photon/, limelight/)  ->  INtApi  ->  NtCoreApi (real, binding/)  or  FakeNtApi (tests)
//
// Contract for every implementation:
//   * Used from ONE thread only: the extension's clock thread, while the robot is quiescent. Calls never
//     block and never throw. Time is NT time, which equals the HAL FPGA time in simulation.
//   * While `available()` is false (ntcore is not bound yet or not present) every other call is a harmless
//     no-op: it returns an invalid handle, false, zero or nothing. A caller never needs to check first.
//   * Handles stay valid for the life of the process; publishers and subscribers are never released
//     (ntcore may be torn down at any moment while the robot exits).
//   * Timestamps are explicit: pass the time of the frame, never 0.
//
// Facts about real ntcore that the contract (and FakeNtApi) reproduce, verified by the nt spike
// (files/spike-nt-findings.md in the session, sections 2-3):
//   * A value whose timestamp is OLDER than the topic's last value is dropped silently (the call still
//     succeeds); equal timestamps are fine. A value equal to the previous one is dropped unless the
//     publisher was created with keepDuplicates.
//   * Topic properties given to publish() apply only when this is the first publisher of a topic that has
//     no publisher yet; they must be a JSON object text. A second publisher whose type string differs is
//     accepted but inactive.
//   * A topic "exists" when it has a publisher; a subscriber alone does not make it exist.
//   * A new subscriber receives the topic's latest value first, and ntcore queues an empty value for
//     topics without one: read() skips it.

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/sdk/core/interface.h"

namespace wpiemu::bridges::vision {

enum class NtType : std::uint8_t { Boolean, Integer, Double, String, Raw, DoubleArray };

// Queue depth for subscribers that must not lose a robot write between two polls (a poll happens every tick).
inline constexpr std::size_t kDefaultNtQueueDepth = 64;

// Canonical NT type string ("boolean", "int", "double", "string", "raw", "double[]"). Subscribers match
// on it, so a publisher with any other string for these types is silently ignored by the robot.
constexpr std::string_view ntTypeString(NtType type) {
    switch (type) {
        case NtType::Boolean: return "boolean";
        case NtType::Integer: return "int";
        case NtType::Double: return "double";
        case NtType::String: return "string";
        case NtType::Raw: return "raw";
        case NtType::DoubleArray: return "double[]";
    }
    return "";
}

struct NtPublisher {
    std::uint32_t id = 0;  // 0 = invalid
    explicit operator bool() const { return id != 0; }
    bool operator==(const NtPublisher&) const = default;
};

struct NtSubscriber {
    std::uint32_t id = 0;  // 0 = invalid
    explicit operator bool() const { return id != 0; }
    bool operator==(const NtSubscriber&) const = default;
};

struct NtPublishOptions {
    // "" = the canonical string of the type. Raw topics carry their message type here, for example
    // "struct:Transform3d" or "photonstruct:PhotonPipelineResult:<uuid>".
    std::string_view typeString;
    // "" = none, otherwise a JSON OBJECT text such as {"message_uuid":"abc"} (string values are quoted).
    std::string_view propertiesJson;
    bool sendAll = false;         // queue every update for subscribers instead of only the latest
    bool keepDuplicates = false;  // publish a value even when it equals the previous one
    double periodicS = 0.0;       // minimum interval ntcore batches updates over; 0 = its default
};

// A value written by someone else (the robot) and read back through read().
struct NtValue {
    NtType type = NtType::Double;
    std::int64_t timeUs = 0;  // when the writer set it, NT time
    // Boolean (0/1), Integer and Double: the value (an integer above 2^53 loses precision; irrelevant here).
    double number = 0.0;
    std::string text;                // String
    std::vector<std::uint8_t> raw;   // Raw
    std::vector<double> numbers;     // DoubleArray
};

// One topic that currently exists (has a publisher).
struct NtTopicInfo {
    std::string name;            // full path with the leading '/'
    std::string typeString;      // as published ("double", "photonstruct:...")
    std::string propertiesJson;  // "{}" when none
};

class INtApi {
    WPIEMU_INTERFACE(INtApi)
public:
    // True once ntcore is bound and usable. The real implementation retries binding on each call until it
    // succeeds (the robot process may map ntcore after the extension loaded), so poll it freely.
    virtual bool available() const = 0;
    // Why available() is false (for one log line); empty when available or unknown.
    virtual std::string_view unavailableReason() const { return {}; }
    // NT_Now(): the current NT time in microseconds (== HAL FPGA time in simulation). 0 when unavailable.
    virtual std::int64_t nowUs() const = 0;

    // --- publishing -------------------------------------------------------------------------------
    // Returns an invalid handle when unavailable or when ntcore rejects the request (for example
    // malformed propertiesJson). Publishing the same topic again returns another handle.
    virtual NtPublisher publish(std::string_view topic, NtType type, const NtPublishOptions& options) = 0;
    // The setters return false for an invalid handle, a value of the wrong type for the publisher, or an
    // unavailable ntcore; true otherwise (even when ntcore dropped a stale or duplicate value).
    virtual bool setBoolean(NtPublisher pub, std::int64_t timeUs, bool value) = 0;
    virtual bool setInteger(NtPublisher pub, std::int64_t timeUs, std::int64_t value) = 0;
    virtual bool setDouble(NtPublisher pub, std::int64_t timeUs, double value) = 0;
    virtual bool setString(NtPublisher pub, std::int64_t timeUs, std::string_view value) = 0;
    virtual bool setRaw(NtPublisher pub, std::int64_t timeUs, std::span<const std::uint8_t> value) = 0;
    virtual bool setDoubleArray(NtPublisher pub, std::int64_t timeUs, std::span<const double> value) = 0;
    // `jsonValue` is JSON text: a string property is set with its QUOTED text, "\"abc\"". Returns false
    // (and stores nothing) for text that is not valid JSON.
    virtual bool setTopicProperty(std::string_view topic, std::string_view name, std::string_view jsonValue) = 0;

    // --- reading what the robot writes ------------------------------------------------------------
    // Subscribes WITHOUT a type filter (whatever type the robot publishes is delivered) with a queue of
    // up to `queueDepth` updates (kDefaultNtQueueDepth is a good default). Works before the topic exists.
    virtual NtSubscriber subscribe(std::string_view topic, std::size_t queueDepth) = 0;
    // Appends every update queued since the previous call, oldest first (the empty initial value of a
    // topic that has none is skipped).
    virtual void read(NtSubscriber sub, std::vector<NtValue>& out) = 0;

    // --- discovery --------------------------------------------------------------------------------
    // Appends the topics that exist (have a publisher) and whose name starts with `prefix`.
    virtual void topics(std::string_view prefix, std::vector<NtTopicInfo>& out) = 0;
    // True when a publisher that is NOT ours exists for the topic (a robot-side PhotonCameraSim, a real
    // coprocessor...). Only meaningful for topics this instance has not published itself: once it did, ntcore
    // cannot tell its own publisher from another one, so the answer is false.
    virtual bool hasForeignPublisher(std::string_view topic) = 0;
};

}  // namespace wpiemu::bridges::vision
