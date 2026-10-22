#include "shell/update_button.h"

#include <QContextMenuEvent>
#include <QCoreApplication>
#include <QMenu>
#include <QStyle>

namespace wpiemu::app {
namespace {

QString tr(const char* text) { return QCoreApplication::translate("UpdateButton", text); }

}  // namespace

UpdatePresentation presentUpdate(UpdateState state, const UpdateOffer& offer, int percent, InstallKind kind, const QString& error) {
    UpdatePresentation p;
    const QString version = offer.version.toString();
    switch (state) {
        case UpdateState::Available:
            p.visible = true;
            p.text = tr("Update");
            p.menuText = tr("&Update to %1…").arg(version);
            p.tone = QStringLiteral("success");
            p.menuEnabled = true;
            p.tooltip = offer.fileName.isEmpty() ? tr("Version %1 is available. Click to open the release page.").arg(version)
                                                 : tr("Version %1 is available. Click to download it.").arg(version);
            break;
        case UpdateState::Downloading:
            p.visible = true;
            p.text = percent >= 0 ? tr("Downloading %1%").arg(percent) : tr("Downloading…");
            p.menuText = percent >= 0 ? tr("Downloading update… %1%").arg(percent) : tr("Downloading update…");
            p.tone = QStringLiteral("accent");
            p.tooltip = tr("Downloading version %1. Right-click to cancel.").arg(version);
            break;
        case UpdateState::Verifying:
            p.visible = true;
            p.text = tr("Verifying…");
            p.menuText = tr("Verifying update…");
            p.tone = QStringLiteral("accent");
            p.tooltip = tr("Checking the download against its published checksum.");
            break;
        case UpdateState::Ready: {
            const bool restarts = updateRestartsApp(kind);
            p.visible = true;
            p.text = restarts ? tr("Restart to update") : tr("Open update");
            p.menuText = restarts ? tr("Restart to &update") : tr("Open &update");
            p.tone = QStringLiteral("success");
            p.menuEnabled = true;
            p.tooltip = restarts ? tr("Version %1 is ready. WPI Emulator will close and run the installer.").arg(version)
                                 : tr("Version %1 is ready. Click to open the downloaded disk image.").arg(version);
            break;
        }
        case UpdateState::Installing:
            p.visible = true;
            p.text = tr("Updating…");
            p.menuText = tr("Updating…");
            p.tone = QStringLiteral("accent");
            p.tooltip = tr("WPI Emulator is closing to run the installer.");
            break;
        case UpdateState::Failed:
            // A failed manual check has no offer to retry and shows its message instead.
            if (offer.tag.isEmpty()) break;
            p.visible = true;
            p.text = tr("Update failed");
            p.menuText = tr("Retry &update to %1…").arg(version);
            p.tone = QStringLiteral("error");
            p.menuEnabled = true;
            p.tooltip = tr("%1\nClick to try again.").arg(error);
            break;
        case UpdateState::Idle:
        case UpdateState::Checking:
        case UpdateState::UpToDate:
        case UpdateState::Unsupported: break;
    }
    return p;
}

UpdateButton::UpdateButton(UpdateService& service, QWidget* parent) : QToolButton(parent), service_(service) {
    setObjectName(QStringLiteral("UpdateButton"));
    setProperty("updateButton", true);
    setToolButtonStyle(Qt::ToolButtonTextOnly);
    setCursor(Qt::PointingHandCursor);
    connect(this, &QToolButton::clicked, this, [this] { service_.install(); });
    connect(&service_, &UpdateService::stateChanged, this, &UpdateButton::refresh);
    connect(&service_, &UpdateService::progressChanged, this, &UpdateButton::refresh);
    refresh();
}

void UpdateButton::refresh() {
    const UpdatePresentation p = presentUpdate(service_.state(), service_.offer(), service_.percent(), service_.installInfo().kind, service_.error());
    setVisible(p.visible);
    setText(p.text);
    setToolTip(p.tooltip);
    setAccessibleName(p.text);
    if (property("tone").toString() != p.tone) {
        setProperty("tone", p.tone);
        style()->unpolish(this);
        style()->polish(this);
    }
}

QMenu* UpdateButton::buildContextMenu() {
    auto* menu = new QMenu(this);
    const UpdateState state = service_.state();
    if (state == UpdateState::Downloading) connect(menu->addAction(tr("Cancel download")), &QAction::triggered, this, [this] { service_.cancelDownload(); });
    connect(menu->addAction(tr("Release notes")), &QAction::triggered, this, [this] { service_.showReleasePage(); });
    if (state != UpdateState::Verifying && state != UpdateState::Installing)
        connect(menu->addAction(tr("Skip this version")), &QAction::triggered, this, [this] { service_.skipThisVersion(); });
    return menu;
}

void UpdateButton::contextMenuEvent(QContextMenuEvent* event) {
    QMenu* menu = buildContextMenu();
    menu->setAttribute(Qt::WA_DeleteOnClose);
    menu->popup(event->globalPos());
}

}  // namespace wpiemu::app
