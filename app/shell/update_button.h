#pragma once

// The Update button shown in the menu bar while a newer release exists (docs/reference/app.md).

#include <QString>
#include <QToolButton>

#include "common/update_service.h"

namespace wpiemu::app {

struct UpdatePresentation {
    bool visible = false;
    QString text;       // the button
    QString menuText;   // the mirrored Help menu action ("&" marks the mnemonic)
    QString tooltip;
    QString tone;       // "success", "accent" or "error": a QSS role
    bool menuEnabled = false;
};

// Pure: what the button and its menu mirror show for a service state. `percent` is -1 while the size is unknown.
UpdatePresentation presentUpdate(UpdateState state, const UpdateOffer& offer, int percent, InstallKind kind, const QString& error);

class UpdateButton : public QToolButton {
    Q_OBJECT
public:
    explicit UpdateButton(UpdateService& service, QWidget* parent = nullptr);

    // Right-click menu (release notes, skip, cancel), for tests to inspect.
    QMenu* buildContextMenu();

protected:
    void contextMenuEvent(QContextMenuEvent* event) override;

private:
    void refresh();

    UpdateService& service_;
};

}  // namespace wpiemu::app
