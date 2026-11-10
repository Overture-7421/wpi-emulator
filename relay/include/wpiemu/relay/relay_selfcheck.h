#pragma once

// The container liveness probe: connect to a running relay as a client, send JoinRoom for a random room id that cannot
// exist, and expect a JoinResult{ok = false} on the control channel. The relay answers without creating anything or
// changing any state (a failed join of a source address is only counted toward that source's lockout, and loopback
// sources, which this probe normally is, are exempt).

#include <chrono>
#include <cstdint>
#include <string>

namespace wpiemu::relay {

struct SelfCheckResult {
    bool ok = false;
    std::string detail;  // what happened, for the probe's stderr
};

SelfCheckResult runSelfCheck(const std::string& host, std::uint16_t port, std::chrono::milliseconds timeout);

}  // namespace wpiemu::relay
