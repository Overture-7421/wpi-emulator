#pragma once

// Hosting a practice match from the desktop app: the options a user chooses and the immutable status the UI shows.
// Standard library and SDK only (no Qt, no net headers), so the Qt layer can use these types directly. The behaviour
// lives in match_hosting.h; the options are applied when a robot session starts (RobotSession::start).

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/ipc/driver_station.h"
#include "wpiemu/sdk/match/match_protocol.h"
#include "wpiemu/sdk/match/robot_appearance.h"

namespace wpiemu::app::host {

enum class HostingMode {
    Off,      // nothing is opened or composed; the session behaves exactly as without practice matches
    Direct,   // listen on a UDP port (LAN or port forwarding); clients use host:port plus the invite
    Relay,    // register a room with the relay; clients use only the invite
};

inline constexpr std::uint16_t kDefaultHostingPort = 47300;
inline constexpr std::size_t kMinHostingPlayers = 2;   // the host's own robot and one remote robot

struct HostingOptions {
    HostingMode mode = HostingMode::Off;
    std::uint16_t port = kDefaultHostingPort;   // Direct: the UDP port to listen on (0 = a free port, reported in the status)
    // Relay: where the relay runs, "host:port" or "relay://host:port" (a trailing "/ROOM" is ignored: the room comes from the
    // invite). IPv6 literals use brackets. Empty is an error in Relay mode.
    std::string relayEndpoint;
    // Participants including the host's own robot: kMinHostingPlayers..sdk::match::kMaxRobotsPerMatch.
    std::size_t maxPlayers = sdk::match::kMaxRobotsPerMatch;
    // Direct: the local address to bind; "::" is every interface (IPv4 and IPv6), "127.0.0.1" only this computer.
    std::string bindHost = "::";
    // HUD key of the game's match phase, copied into the snapshots' MatchInfo (empty = none). The game supplies it.
    std::string phaseHudKey;

    bool operator==(const HostingOptions&) const = default;
};

// Ok, or InvalidArgument with a user-presentable message (bad player count, port, or relay endpoint).
sdk::Status validateHosting(const HostingOptions& options);
// `options` made valid where a value is merely out of range (player count clamped); the relay endpoint and the mode stay as given.
HostingOptions normalizedHosting(HostingOptions options);

// The persisted key of a mode ("off", "direct", "relay") and its inverse (`fallback` for an unknown key).
const char* hostingModeKey(HostingMode mode);
HostingMode hostingModeFromKey(const std::string& key, HostingMode fallback = HostingMode::Off);

struct RelayAddress {
    std::string host;           // IPv6 literals without brackets
    std::uint16_t port = 0;
};
// The relay named by HostingOptions::relayEndpoint. Syntactic: never resolves a name. InvalidArgument with the reason otherwise.
sdk::Result<RelayAddress> parseRelayAddress(const std::string& text);

enum class HostingPhase {
    Off,          // not hosting
    Starting,     // opening the listener or registering the room with the relay
    Lobby,        // clients may join; the robot program is not running in lock-step yet (or the session is over)
    Live,         // the match is running
    Paused,       // the host paused the simulation
    HostStalled,  // the host's robot stopped answering
    Error,        // the listener or the relay registration failed or was lost; `detail` says why
};

struct HostingParticipant {
    sdk::match::ParticipantId id = sdk::match::kHostParticipantId;
    std::string name;
    sdk::proto::AllianceStation station = sdk::proto::AllianceStation::Red1;
    bool isHost = false;
    bool connected = true;           // false: dropped, its robot is kept for a while so the client can resume
    std::uint32_t pingMs = 0;        // the transport's smoothed round trip (0 for the host)
    std::uint32_t lossPermille = 0;  // recent packet loss, parts per thousand (1000 while dropped)
    std::uint16_t snapshotRateHz = 0;
    // How to draw this participant's robot (the host's accepted spec; null for the host itself and until known). Shared
    // between statuses, so equal pointers mean an unchanged robot.
    std::shared_ptr<const sdk::match::RobotAppearance> appearance;
    bool operator==(const HostingParticipant&) const = default;
};

// What the UI shows. Immutable once published; the invite is the ONLY place the secret appears.
struct HostingStatus {
    HostingMode mode = HostingMode::Off;
    HostingPhase phase = HostingPhase::Off;
    std::string detail;               // one line for the user: why Starting/Error, or what the lobby waits for
    std::string invite;               // "XXXXX-XXXXX-XXXXX-XXXXX": share it with the players (empty when Off)
    std::string endpoint;             // enet://host:port (Direct) or relay://host:port/ROOM (Relay); empty until listening
    std::uint16_t port = 0;           // Direct: the bound UDP port
    std::size_t maxPlayers = 0;
    std::uint64_t runEpoch = 0;
    std::vector<HostingParticipant> roster;   // the host's robot first, then the remote participants by id
    bool timeScaleLocked = false;     // clients are connected: the simulation runs at 1x
    bool stepDisabled = false;        // clients are connected: single-stepping is disabled
    std::uint64_t joinsAccepted = 0;
    std::uint64_t joinsRejected = 0;
    std::uint64_t handshakesFailed = 0;   // connections that did not prove they hold the invite
    std::uint64_t kicks = 0;

    bool hosting() const { return phase != HostingPhase::Off; }
    // Clients can reach the host (the listener or the room is up).
    bool accepting() const {
        return phase == HostingPhase::Lobby || phase == HostingPhase::Live || phase == HostingPhase::Paused ||
               phase == HostingPhase::HostStalled;
    }
    std::size_t remoteCount() const {
        std::size_t n = 0;
        for (const HostingParticipant& p : roster)
            if (!p.isHost) ++n;
        return n;
    }

    bool operator==(const HostingStatus&) const = default;
};

}  // namespace wpiemu::app::host
