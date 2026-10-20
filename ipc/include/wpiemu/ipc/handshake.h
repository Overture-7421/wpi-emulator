#pragma once

// Hello / HelloAck exchange (ARCHITECTURE section 4): version check, session identity, watchdog
// negotiation. Both ends call exactly one of these right after the transport connects; they are
// blocking and bounded by `timeout`.

#include <chrono>

#include "wpiemu/sdk/ipc/transport.h"

namespace wpiemu::ipc {

// What the emulator decides; the caller owns the monotonic counters (sessionId per accepted
// connection, runEpoch bumped on every Reset and reconnect) so they survive reconnects.
struct ServerHandshakeConfig {
    sdk::proto::ClockMode clockMode = sdk::proto::ClockMode::Lockstep;
    std::uint32_t stepUs = static_cast<std::uint32_t>(sdk::kDefaultStepUs);
    bool discoveryOnly = false;
    bool enableL3Hooks = false;
    std::uint32_t preferredWatchdogMs = 0;  // 0 = no preference
    sdk::proto::WatchdogPolicy watchdogPolicy = sdk::proto::WatchdogPolicy::ResumeRealtime;
    std::uint64_t sessionId = 0;  // must be non-zero
    std::uint64_t runEpoch = sdk::proto::kFirstRunEpoch;
};

struct ServerHandshakeResult {
    sdk::proto::Hello hello;
    sdk::proto::HelloAck ack;  // what was sent (accepted == true)
};

// Waits for Hello and answers HelloAck. A peer older than kMinCompatibleProtocolVersion, or a first
// message that is not Hello, is refused (HelloAck{accepted=false, reason}) and an error returned.
sdk::Result<ServerHandshakeResult> performServerHandshake(sdk::proto::MessageChannel& channel,
                                                          const ServerHandshakeConfig& config,
                                                          std::chrono::milliseconds timeout);

// Sends Hello and waits for HelloAck. Returns an error when the emulator refuses or the exchange
// times out. HelloAck::protocolVersion is the version both sides speak (the lower of the two).
sdk::Result<sdk::proto::HelloAck> performClientHandshake(sdk::proto::MessageChannel& channel,
                                                         const sdk::proto::Hello& hello,
                                                         std::chrono::milliseconds timeout);

}  // namespace wpiemu::ipc
