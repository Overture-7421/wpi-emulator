#include "client/client_text.h"

#include "ui/qt_text.h"

namespace wpiemu::app::client {
namespace {

constexpr ui::Tr tr{"Client"};

}  // namespace

QString bannerTitle(Banner banner) {
    switch (banner) {
        case Banner::None: return {};
        case Banner::Connecting: return tr("Connecting to the host…");
        case Banner::Reconnecting: return tr("Connection lost: reconnecting…");
        case Banner::Lobby: return tr("Waiting for the host to start the match");
        case Banner::Paused: return tr("The host paused the match");
        case Banner::HostStalled: return tr("Host stalled: the host's robot stopped answering");
        case Banner::LinkLost: return tr("Connection lost");
        case Banner::HostSilent: return tr("The host stopped responding");
        case Banner::Kicked: return tr("The host removed you from the match");
        case Banner::ServerClosing: return tr("The host closed the match");
        case Banner::Refused: return tr("The host did not accept the join");
        case Banner::ProtocolError: return tr("The host sent something this client does not understand");
    }
    return {};
}

QString rejectReasonText(sdk::match::JoinRejectReason reason) {
    using R = sdk::match::JoinRejectReason;
    switch (reason) {
        case R::BadProtocol: return tr("This client and the host speak different versions of the match protocol. Update the older one.");
        case R::WrongPlugin: return tr("The host plays a different game than your robot description names.");
        case R::WrongSeason: return tr("The host runs a different WPILib season than this robot description.");
        case R::RoomFull: return tr("The match is full.");
        case R::BadSpec: return tr("The host cannot use this robot description.");
        case R::Kicked: return tr("The host removed you earlier in this session; ask for a new invite.");
        case R::Banned: return tr("The host does not allow you to join.");
        case R::HostClosing: return tr("The host is closing the match.");
        case R::BadInvite: return tr("The invite is not the one this host is using.");
        case R::BadResumeToken: return tr("The host no longer holds your robot. Join again.");
    }
    return {};
}

QString bannerDetail(const Presentation& p) {
    QString detail = QString::fromStdString(p.detail);
    if (p.banner == Banner::Refused && p.rejectReason) {
        const QString why = rejectReasonText(*p.rejectReason);
        return detail.isEmpty() ? why : why + QStringLiteral(" (") + detail + QLatin1Char(')');
    }
    if (p.banner == Banner::Reconnecting && p.reconnectAttempt > 0)
        return tr("Attempt %1").arg(p.reconnectAttempt) + (detail.isEmpty() ? QString() : QStringLiteral(" · ") + detail);
    return detail;
}

QString fieldErrorText(const JoinFieldError& e) {
    const QString detail = QString::fromStdString(e.detail);
    const auto with = [&](const QString& text) { return detail.isEmpty() ? text : text + QStringLiteral(" ") + detail; };
    if (e.key == "invite.empty") return tr("Paste the invite the host shared (XXXXX-XXXXX-XXXXX-XXXXX).");
    if (e.key == "invite.format") return with(tr("This is not an invite: it has 20 letters and digits in four groups of five."));
    if (e.key == "direct.address") return with(tr("The host's address is not valid. Use host or host:port."));
    if (e.key == "relay.unconfigured")
        return tr("This build does not know the relay address. Open Advanced and enter one, or give the host's address for a direct connection.");
    if (e.key == "relay.address") return with(tr("The relay address is not valid. Use host:port."));
    if (e.key == "name.empty") return tr("Enter the name other players will see.");
    if (e.key == "name.too_long") return tr("The name is too long.");
    if (e.key == "robot.missing") return tr("Choose your robot.json.");
    if (e.key == "robot.invalid") return with(tr("This robot description cannot be used."));
    return with(QString::fromStdString(e.key));
}

QString stateWord(core::match::ConnectionState state) {
    using S = core::match::ConnectionState;
    switch (state) {
        case S::Connecting: return tr("Connecting");
        case S::Lobby: return tr("Lobby");
        case S::Live: return tr("Live");
        case S::Paused: return tr("Paused");
        case S::HostStalled: return tr("Host stalled");
        case S::Disconnected: return tr("Disconnected");
    }
    return {};
}

}  // namespace wpiemu::app::client
