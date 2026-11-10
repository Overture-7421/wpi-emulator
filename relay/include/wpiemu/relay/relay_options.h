#pragma once

// Command line and environment configuration of wpiemu_relay. The environment wins over the command line (the container
// deployment sets WPIEMU_RELAY_*), the command line over the defaults. Every value is validated against sane bounds; an
// invalid one is an error that names the setting, never a silent clamp.

#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "wpiemu/net/enet_transport.h"
#include "wpiemu/relay/relay_log.h"
#include "wpiemu/relay/relay_server.h"
#include "wpiemu/sdk/core/result.h"

namespace wpiemu::relay {

enum class RelayCommand : std::uint8_t { Run, Help, Version, SelfCheck };

inline constexpr std::uint16_t kDefaultRelayPort = 7777;
inline constexpr const char* kDefaultRelayBind = "::";
inline constexpr const char* kRelayVersion = "1.0.0";
// ENet addresses at most 4095 peers; the relay needs maxRooms * (maxPeers + 1) of them.
inline constexpr std::size_t kMaxConfigurableRooms = 500;

struct RelayOptions {
    RelayCommand command = RelayCommand::Run;
    std::string bindHost = kDefaultRelayBind;
    std::uint16_t port = kDefaultRelayPort;
    LogFormat logFormat = LogFormat::Json;
    RelayServerConfig server;
    // Connections the transport accepts at once (derived from the room and peer limits).
    std::size_t maxConnections = 0;
    // --selfcheck: where the running relay is, and how long to wait for its answer.
    std::string selfCheckHost = "127.0.0.1";
    std::chrono::milliseconds selfCheckTimeout{2000};
};

using EnvLookup = std::function<std::optional<std::string>(const std::string& name)>;

// `args` excludes the program name. InvalidArgument names the offending option or variable.
sdk::Result<RelayOptions> parseRelayOptions(const std::vector<std::string>& args, const EnvLookup& env);

std::string relayUsage();

// The ENet server settings of a production relay (wpiemu_relay and the tests that mirror it use this one function). The
// relay prepends the 2-byte envelope to a client's frame before it hands it to the host, so the link must carry frames of
// kRelayMaxReliablePacket / kRelayMaxDatagram PLUS the envelope.
net::EnetServerOptions transportOptionsFor(const RelayOptions& options);

}  // namespace wpiemu::relay
