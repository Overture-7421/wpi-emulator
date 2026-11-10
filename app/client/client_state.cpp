#include "client/client_state.h"

namespace wpiemu::app::client {
namespace {

namespace mc = core::match;
using runview::Tone;

bool mayResume(const mc::ConnectionStatus& status) {
    if (status.state != mc::ConnectionState::Disconnected) return false;
    return status.reason == mc::DisconnectReason::TransportLost || status.reason == mc::DisconnectReason::HostSilent ||
           status.reason == mc::DisconnectReason::JoinTimeout;
}

Banner bannerOfDisconnect(const mc::ConnectionStatus& status) {
    switch (status.reason) {
        case mc::DisconnectReason::Kicked: return Banner::Kicked;
        case mc::DisconnectReason::ServerClosing: return Banner::ServerClosing;
        case mc::DisconnectReason::TransportLost: return Banner::LinkLost;
        case mc::DisconnectReason::JoinTimeout: return Banner::LinkLost;
        case mc::DisconnectReason::HostSilent: return Banner::HostSilent;
        case mc::DisconnectReason::ProtocolError: return Banner::ProtocolError;
        case mc::DisconnectReason::JoinRejected:
        case mc::DisconnectReason::BadSpec: return Banner::Refused;
        case mc::DisconnectReason::Left:
        case mc::DisconnectReason::None: break;
    }
    return Banner::LinkLost;
}

}  // namespace

Presentation present(const SessionState& state) {
    Presentation p;
    const mc::ConnectionStatus& status = state.status;
    p.detail = status.detail;
    p.rejectReason = status.rejectReason;
    p.reason = status.reason;
    p.reconnectAttempt = status.reconnectAttempt;

    if (state.phase == LinkPhase::Idle) return p;

    if (state.phase == LinkPhase::Opening) {
        p.banner = Banner::Connecting;
        p.tone = Tone::Info;
        p.busy = p.joining = p.canLeave = true;
        return p;
    }
    if (state.phase == LinkPhase::Failed) {
        p.banner = Banner::LinkLost;
        p.tone = Tone::Error;
        p.detail = state.openError;
        return p;
    }

    // Active: the MatchClient has the link. Before the host accepted the player the form stays up.
    const bool joined = state.everJoined;
    p.screen = joined ? Screen::Drive : Screen::Join;
    switch (status.state) {
        case mc::ConnectionState::Connecting:
            p.banner = status.reconnecting ? Banner::Reconnecting : Banner::Connecting;
            p.tone = status.reconnecting ? Tone::Warning : Tone::Info;
            p.busy = true;
            p.joining = !joined;
            p.canLeave = true;
            break;
        case mc::ConnectionState::Lobby:
            p.banner = Banner::Lobby;
            p.tone = Tone::Info;
            p.controlsEnabled = true;
            p.canLeave = true;
            break;
        case mc::ConnectionState::Live:
            p.tone = Tone::Success;
            p.controlsEnabled = true;
            p.canLeave = true;
            break;
        case mc::ConnectionState::Paused:
            p.banner = Banner::Paused;
            p.tone = Tone::Warning;
            p.canLeave = true;
            break;
        case mc::ConnectionState::HostStalled:
            p.banner = Banner::HostStalled;
            p.tone = Tone::Warning;
            p.canLeave = true;
            break;
        case mc::ConnectionState::Disconnected:
            p.banner = bannerOfDisconnect(status);
            p.tone = Tone::Error;
            p.canReconnect = joined && mayResume(status) && !state.reopening;
            if (!state.openError.empty()) p.detail = state.openError;   // the last reconnect could not dial
            break;
    }
    if (state.reopening) {
        p.banner = Banner::Reconnecting;
        p.tone = Tone::Warning;
        p.busy = true;
        p.canLeave = true;
    }
    return p;
}

}  // namespace wpiemu::app::client
