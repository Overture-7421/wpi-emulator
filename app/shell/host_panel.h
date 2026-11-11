#pragma once

// The "Practice match" panel: host a match (mode, port, relay, player limit), share the invite and watch who joined. Widgets
// only; the logic is HostPanelModel. The shell docks it next to the run view.

#include <QComboBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QTableWidget>
#include <QToolButton>
#include <QWidget>

#include "common/ui_helpers.h"
#include "common/ui_resources.h"
#include "shell/host_panel_model.h"

namespace wpiemu::app {

class HostPanel : public QWidget {
    Q_OBJECT
public:
    HostPanel(HostPanelModel& model, const UiResources& ui, QWidget* parent = nullptr);

    HostPanelModel& model() const { return model_; }

    // Introspection used by tests.
    QComboBox* modeCombo() const { return mode_; }
    QSpinBox* portSpin() const { return port_; }
    QSpinBox* playersSpin() const { return players_; }
    QToolButton* advancedToggle() const { return advancedToggle_; }
    QComboBox* relayCombo() const { return relay_; }
    QLineEdit* customRelayEdit() const { return customRelay_; }
    QLineEdit* bindEdit() const { return bind_; }
    Banner* problemBanner() const { return problem_; }
    Banner* hintBanner() const { return hint_; }
    Banner* phaseBanner() const { return phase_; }
    QLineEdit* inviteEdit() const { return invite_; }
    QPushButton* copyButton() const { return copy_; }
    QTableWidget* rosterTable() const { return roster_; }
    // The Kick button of a roster row (nullptr for the host's row).
    QPushButton* kickButton(int row) const;

private:
    void refresh();

    HostPanelModel& model_;
    QComboBox* mode_;
    QLabel* portLabel_;
    QSpinBox* port_;
    QSpinBox* players_;
    QLabel* relayInfo_;
    QToolButton* advancedToggle_;
    QWidget* advanced_;
    QLabel* relayLabel_;
    QComboBox* relay_;
    QLabel* customRelayLabel_;
    QLineEdit* customRelay_;
    QLabel* bindLabel_;
    QLineEdit* bind_;
    Banner* problem_;
    Banner* hint_;
    Banner* phase_;
    QWidget* inviteRow_;
    QLineEdit* invite_;
    QPushButton* copy_;
    QLabel* endpoint_;
    QTableWidget* roster_;
};

}  // namespace wpiemu::app
