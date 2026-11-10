#pragma once

// The relay probe (wpiemu_relay_probe): a black-box check of a RUNNING relay from the outside, as a player's machine would
// use it. The deploy pipeline runs it against the relay's address after every deployment.
//
//   reach      connect over ENet (UDP) and send JoinRoom for a random room that cannot exist; the relay must answer
//              JoinResult{ok=false, NoSuchRoom}. Proves DNS, UDP reachability and a relay that speaks our protocol;
//              creates nothing.
//   full       reach, then the whole player path: register a room as a host with a fresh invite (or --invite), join it
//              as a client, run the encrypted handshake, exchange --count ping/echo messages on both delivery classes
//              (round-trip times, unreliable loss), round-trip --payload-bytes reliably, and leave cleanly (the room is
//              not left to expire). At most ONE room is created.
//   selfcheck  what `wpiemu_relay --selfcheck` does: the relay is alive when it answers JoinRoom with any refusal.
//
// Exit codes: 0 the probe passed, 1 it ran and failed, 2 usage error. Every step is bounded by one deadline (--timeout-ms);
// cleanup after it (closing the links, at most about a second) comes on top. The invite secret is never printed unless
// --print-invite is given; only the room id derived from it is.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iosfwd>
#include <optional>
#include <string>
#include <vector>

#include "wpiemu/net/secure_transport.h"
#include "wpiemu/relay/relay_options.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/match/relay_messages.h"

namespace wpiemu::relay {

enum class ProbeMode : std::uint8_t { Full, Reach, SelfCheck };

inline constexpr const char* kProbeRelayEnvironment = "WPIEMU_RELAY_PROBE_RELAY";
inline constexpr std::chrono::milliseconds kProbeDefaultTimeout{10'000};
inline constexpr std::size_t kProbeDefaultCount = 20;
inline constexpr std::size_t kProbeDefaultPayloadBytes = 64 * 1024;
inline constexpr std::size_t kProbeMaxCount = 1000;
// The largest application payload that makes the whole reliable round trip: the host's frame carries the relay envelope and
// every frame the encryption record, and the relay's reliable limit is kRelayMaxReliablePacket.
inline constexpr std::size_t kProbeMaxPayloadBytes = sdk::match::relay::kRelayMaxReliablePacket -
                                                     sdk::match::relay::kRelayEnvelopeBytes -
                                                     net::SecureTransport::kRecordOverhead;

struct ProbeOptions {
    std::string relayHost;
    std::uint16_t relayPort = kDefaultRelayPort;
    ProbeMode mode = ProbeMode::Full;
    std::chrono::milliseconds timeout = kProbeDefaultTimeout;
    std::size_t count = kProbeDefaultCount;
    std::size_t payloadBytes = kProbeDefaultPayloadBytes;
    std::string invite;  // full mode: the invite text to use instead of a fresh one
    bool printInvite = false;
    bool json = false;
};

struct ProbeStep {
    std::string name;
    bool ok = false;
    std::int64_t ms = 0;
    std::string detail;
};

struct RttSummary {
    std::size_t samples = 0;
    double minMs = 0;
    double avgMs = 0;
    double p95Ms = 0;
    double maxMs = 0;
};

struct ProbeResult {
    bool ok = false;
    ProbeMode mode = ProbeMode::Full;
    std::string relay;            // "host:port" as probed
    std::string resolvedAddress;  // the IP address that answered, once known
    RttSummary rtt;               // every echo of the full mode, both delivery classes
    std::uint32_t unreliableLossPermille = 0;
    std::uint64_t reliableBytes = 0;  // payload bytes that made the reliable round trip intact
    std::vector<ProbeStep> steps;
    std::string error;            // why the probe failed, with advice; empty when it passed
    std::string invite;           // only with printInvite
    std::string room;             // the room id of the invite, full mode
};

const char* probeModeName(ProbeMode mode);

// Percentile of ascending-sorted samples by the nearest-rank method; 0 for an empty list.
double percentileOfSorted(const std::vector<double>& sorted, double fraction);

// What the command line asked for.
enum class ProbeCommand : std::uint8_t { Run, Help, Version };

struct ProbeInvocation {
    ProbeCommand command = ProbeCommand::Run;
    ProbeOptions options;
};

// `args` excludes the program name. InvalidArgument names the offending option; the relay comes from --relay or from the
// environment variable WPIEMU_RELAY_PROBE_RELAY (the command line wins).
sdk::Result<ProbeInvocation> parseProbeArgs(const std::vector<std::string>& args, const EnvLookup& env);

std::string probeUsage();

// Runs the probe; never throws, never hangs (see the header comment).
ProbeResult runProbe(const ProbeOptions& options);

std::string formatProbeText(const ProbeResult& result);
// One line of JSON: {ok, mode, relay, resolvedAddress?, rttMs{min,avg,p95,max}, unreliableLossPermille, reliableBytes,
// steps:[{name,ok,ms,detail}], error?}; "room" and "invite" are added when known / requested.
std::string formatProbeJson(const ProbeResult& result);

// The whole command line behind main(): parses `args`, runs the probe, prints the report to `out` (the usage error to `err`)
// and returns the exit code. With `watchdog` a timer thread ends the process with exit code 1 when the probe overruns its
// deadline by more than a few seconds (a name lookup that never returns).
int runProbeCli(const std::vector<std::string>& args, const EnvLookup& env, std::ostream& out, std::ostream& err,
                bool watchdog = false);

}  // namespace wpiemu::relay
