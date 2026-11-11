// The presentation state machine: ConnectionState (and the link phase) to screen, banner, tone and buttons.
#include <doctest/doctest.h>

#include "client/client_state.h"

using namespace wpiemu;
using namespace wpiemu::app;
using namespace wpiemu::app::client;
using runview::Tone;
namespace mc = wpiemu::core::match;
namespace wire = wpiemu::sdk::match;

namespace {

SessionState active(mc::ConnectionState state, bool joined, mc::DisconnectReason reason = mc::DisconnectReason::None,
                    std::string detail = {}) {
    SessionState s;
    s.phase = LinkPhase::Active;
    s.everJoined = joined;
    s.status.state = state;
    s.status.reason = reason;
    s.status.detail = std::move(detail);
    return s;
}

}  // namespace

TEST_CASE("client state: idle is the join form with nothing to say") {
    const Presentation p = present(SessionState{});
    CHECK(p.screen == Screen::Join);
    CHECK(p.banner == Banner::None);
    CHECK_FALSE(p.joining);
    CHECK_FALSE(p.controlsEnabled);
    CHECK_FALSE(p.canLeave);
}

TEST_CASE("client state: dialling keeps the form up, locked, with a cancel") {
    SessionState s;
    s.phase = LinkPhase::Opening;
    const Presentation p = present(s);
    CHECK(p.screen == Screen::Join);
    CHECK(p.banner == Banner::Connecting);
    CHECK(p.tone == Tone::Info);
    CHECK(p.busy);
    CHECK(p.joining);
    CHECK(p.canLeave);
}

TEST_CASE("client state: a link that could not be opened is an error on the form") {
    SessionState s;
    s.phase = LinkPhase::Failed;
    s.openError = "invite handshake failed";
    const Presentation p = present(s);
    CHECK(p.screen == Screen::Join);
    CHECK(p.banner == Banner::LinkLost);
    CHECK(p.tone == Tone::Error);
    CHECK(p.detail == "invite handshake failed");
    CHECK_FALSE(p.joining);
}

TEST_CASE("client state: waiting for the host's answer stays on the form") {
    const Presentation p = present(active(mc::ConnectionState::Connecting, false));
    CHECK(p.screen == Screen::Join);
    CHECK(p.banner == Banner::Connecting);
    CHECK(p.joining);
}

TEST_CASE("client state: lobby, live, paused and host stalled on the drive screen") {
    Presentation p = present(active(mc::ConnectionState::Lobby, true));
    CHECK(p.screen == Screen::Drive);
    CHECK(p.banner == Banner::Lobby);
    CHECK(p.controlsEnabled);

    p = present(active(mc::ConnectionState::Live, true));
    CHECK(p.screen == Screen::Drive);
    CHECK(p.banner == Banner::None);
    CHECK(p.tone == Tone::Success);
    CHECK(p.controlsEnabled);
    CHECK(p.canLeave);

    p = present(active(mc::ConnectionState::Paused, true));
    CHECK(p.banner == Banner::Paused);
    CHECK(p.tone == Tone::Warning);
    CHECK_FALSE(p.controlsEnabled);

    p = present(active(mc::ConnectionState::HostStalled, true));
    CHECK(p.banner == Banner::HostStalled);
    CHECK(p.tone == Tone::Warning);
    CHECK_FALSE(p.controlsEnabled);
}

TEST_CASE("client state: a lost link while playing can be reconnected, a kick or a closed room cannot") {
    Presentation p = present(active(mc::ConnectionState::Disconnected, true, mc::DisconnectReason::TransportLost, "link lost"));
    CHECK(p.screen == Screen::Drive);
    CHECK(p.banner == Banner::LinkLost);
    CHECK(p.tone == Tone::Error);
    CHECK(p.canReconnect);
    CHECK(p.detail == "link lost");

    p = present(active(mc::ConnectionState::Disconnected, true, mc::DisconnectReason::HostSilent));
    CHECK(p.banner == Banner::HostSilent);
    CHECK(p.canReconnect);

    p = present(active(mc::ConnectionState::Disconnected, true, mc::DisconnectReason::Kicked, "Behave."));
    CHECK(p.banner == Banner::Kicked);
    CHECK_FALSE(p.canReconnect);
    CHECK(p.detail == "Behave.");

    p = present(active(mc::ConnectionState::Disconnected, true, mc::DisconnectReason::ServerClosing));
    CHECK(p.banner == Banner::ServerClosing);
    CHECK_FALSE(p.canReconnect);

    p = present(active(mc::ConnectionState::Disconnected, true, mc::DisconnectReason::ProtocolError));
    CHECK(p.banner == Banner::ProtocolError);
    CHECK_FALSE(p.canReconnect);
}

TEST_CASE("client state: the host's reject reasons stay on the form with its explanation") {
    SessionState s = active(mc::ConnectionState::Disconnected, false, mc::DisconnectReason::JoinRejected, "Room is full");
    s.status.rejectReason = wire::JoinRejectReason::RoomFull;
    const Presentation p = present(s);
    CHECK(p.screen == Screen::Join);
    CHECK(p.banner == Banner::Refused);
    CHECK(p.rejectReason == wire::JoinRejectReason::RoomFull);
    CHECK(p.detail == "Room is full");
    CHECK_FALSE(p.canReconnect);
    CHECK_FALSE(p.joining);

    const Presentation spec = present(active(mc::ConnectionState::Disconnected, false, mc::DisconnectReason::BadSpec, "wheels: too few"));
    CHECK(spec.screen == Screen::Join);
    CHECK(spec.banner == Banner::Refused);

    const Presentation timeout = present(active(mc::ConnectionState::Disconnected, false, mc::DisconnectReason::JoinTimeout));
    CHECK(timeout.screen == Screen::Join);
    CHECK_FALSE(timeout.canReconnect);   // never joined: nothing to resume
}

TEST_CASE("client state: reconnecting shows the attempt and is not offered again meanwhile") {
    SessionState s = active(mc::ConnectionState::Connecting, true);
    s.status.reconnecting = true;
    s.status.reconnectAttempt = 3;
    Presentation p = present(s);
    CHECK(p.screen == Screen::Drive);
    CHECK(p.banner == Banner::Reconnecting);
    CHECK(p.tone == Tone::Warning);
    CHECK(p.reconnectAttempt == 3);
    CHECK(p.busy);

    SessionState manual = active(mc::ConnectionState::Disconnected, true, mc::DisconnectReason::TransportLost);
    manual.reopening = true;
    p = present(manual);
    CHECK(p.banner == Banner::Reconnecting);
    CHECK_FALSE(p.canReconnect);
    CHECK(p.canLeave);
}

TEST_CASE("client state: a reconnect that could not dial keeps the failure text") {
    SessionState s = active(mc::ConnectionState::Disconnected, true, mc::DisconnectReason::TransportLost, "link lost");
    s.openError = "no route to the host";
    const Presentation p = present(s);
    CHECK(p.canReconnect);
    CHECK(p.detail == "no route to the host");
}
