#pragma once

// What the window shows for a connection, as a pure function of the session's state: which screen, which banner and which
// buttons. The MatchClient's ConnectionState is the source of truth; this only decides how to present it. Text is chosen by the
// Qt layer from these enums (client_text.h), so every string is translated.

#include <cstdint>
#include <optional>
#include <string>

#include "client/client_session.h"
#include "runview/run_control.h"

namespace wpiemu::app::client {

enum class Screen : std::uint8_t {
    Join,    // the form (also while a join is in flight, and after one that failed)
    Drive,   // the field: from the host's acceptance until the player leaves
};

enum class Banner : std::uint8_t {
    None,           // live: nothing to say
    Connecting,     // dialling, handshaking or waiting for the host's answer
    Reconnecting,   // the link broke; the client is dialling the host again
    Lobby,          // joined; the host has not started
    Paused,         // the host paused
    HostStalled,    // the host's robot stopped answering
    LinkLost,       // the connection broke and no reconnect succeeded
    HostSilent,     // the host stopped sending
    Kicked,         // the host removed this player
    ServerClosing,  // the host closed the room
    Refused,        // the host (or this client) refused the join: see rejectReason / detail
    ProtocolError,  // the host sent something the protocol does not allow
};

struct Presentation {
    Screen screen = Screen::Join;
    Banner banner = Banner::None;
    runview::Tone tone = runview::Tone::Neutral;
    bool busy = false;                 // something is in progress (progress bar)
    bool controlsEnabled = false;      // drive, zone buttons
    bool canLeave = false;             // Leave (Drive) or Cancel (Join, while connecting)
    bool canReconnect = false;         // the host may still hold this player's robot: Reconnect
    bool joining = false;              // Join screen: a join is in flight (the form is locked)
    std::string detail;                // the host's or the local explanation, verbatim
    std::optional<sdk::match::JoinRejectReason> rejectReason;
    core::match::DisconnectReason reason = core::match::DisconnectReason::None;
    std::uint32_t reconnectAttempt = 0;

    bool operator==(const Presentation&) const = default;
};

Presentation present(const SessionState& state);

}  // namespace wpiemu::app::client
