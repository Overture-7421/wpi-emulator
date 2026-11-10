#pragma once

// The two cards that float over the drive view: the connection banner (lobby, paused, host stalled, reconnecting, disconnected,
// with the host's reasons and the buttons that make sense) and the connection diagnostics (ping, loss, prediction error ...).

#include <QFrame>
#include <QString>

#include "client/client_state.h"
#include "wpiemu/core/match/client_view.h"

class QLabel;
class QProgressBar;
class QPushButton;

namespace wpiemu::app::client {

class ClientBanner : public QFrame {
    Q_OBJECT
public:
    explicit ClientBanner(QWidget* parent = nullptr);

    // Shows the banner for `presentation`, or hides it when there is nothing to say.
    void setPresentation(const Presentation& presentation);

    QLabel* titleLabel() const { return title_; }
    QLabel* detailLabel() const { return detail_; }
    QPushButton* reconnectButton() const { return reconnect_; }
    QPushButton* leaveButton() const { return leave_; }

signals:
    void reconnectRequested();
    void leaveRequested();

private:
    QLabel* title_;
    QLabel* detail_;
    QProgressBar* busy_;
    QPushButton* reconnect_;
    QPushButton* leave_;
};

// The numbers the overlay shows, as translated lines "label: value" (one per row), for a view. Pure, so it is tested.
QStringList diagnosticsLines(const core::match::MatchClientView& view);

class DiagnosticsOverlay : public QFrame {
    Q_OBJECT
public:
    explicit DiagnosticsOverlay(QWidget* parent = nullptr);
    void setView(const core::match::MatchClientView* view);   // null clears
    QLabel* textLabel() const { return text_; }

private:
    QLabel* text_;
};

}  // namespace wpiemu::app::client
