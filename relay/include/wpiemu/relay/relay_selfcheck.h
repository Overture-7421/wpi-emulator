#pragma once

// The container liveness probe: connect to a running relay as a client, send JoinRoom for the reserved health-ping room
// (sdk/match/relay_messages.h: kRelayPingRoomId, a room that never exists) and expect a JoinResult{ok = false, NoSuchRoom} on
// the control channel. The relay answers without creating anything or changing any state, and does NOT count the join as a
// failed one toward the source address's lockout, so the probe can run as often as needed from any address.

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "wpiemu/sdk/match/relay_messages.h"

namespace wpiemu::relay {

// Where a failed check stopped; the probe turns it into the advice it prints.
enum class SelfCheckFailure : std::uint8_t {
    None = 0,
    Resolve,      // the host name did not resolve
    Unreachable,  // no ENet connection within the timeout: nothing answers on that UDP port, or UDP is blocked
    Connect,      // the connection was refused or failed for another reason
    NoAnswer,     // connected, but JoinRoom got no answer in time
    HungUp,       // connected, then the peer closed before it answered
    Protocol,     // an answer that is not a relay's JoinResult
};

struct SelfCheckResult {
    bool ok = false;
    std::string detail;  // what happened, for the probe's stderr
    SelfCheckFailure failure = SelfCheckFailure::None;
    // The IP address the connection went to, once there was one.
    std::string resolvedAddress;
    // The relay's JoinResult, when one arrived. `ok` is true for any refusal: the relay is alive.
    std::optional<sdk::match::relay::JoinResult> answer;
};

SelfCheckResult runSelfCheck(const std::string& host, std::uint16_t port, std::chrono::milliseconds timeout);

}  // namespace wpiemu::relay
