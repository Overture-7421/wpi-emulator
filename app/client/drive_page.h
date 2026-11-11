#pragma once

// The drive screen: the shared run-view scene (the own robot, the other robots, fuel and the game's HUD, drawn by the same rigs the
// desktop app uses), the zone controls, the connection banner and the diagnostics overlay. It reads what the controller publishes
// and forwards the player's clicks; the controller sends the commands.

#include <QElapsedTimer>
#include <QToolButton>
#include <QWidget>

#include <memory>

#include "client/client_controller.h"
#include "client/drive_overlays.h"
#include "client/zone_panel.h"
#include "runview/hud_model.h"
#include "runview/joystick_manager.h"
#include "runview/run_viewport.h"
#include "runview/scene_model.h"

class QAction;
class QLabel;
class QMenu;
class QPushButton;
class QToolButton;

namespace wpiemu::app::client {

class DrivePage : public QWidget {
    Q_OBJECT
public:
    // `joysticks` is borrowed: the page feeds it the keyboard (null: no keyboard control).
    DrivePage(ClientController& controller, runview::JoystickManager* joysticks, QWidget* parent = nullptr);
    ~DrivePage() override;

    runview::RunViewport* viewport() const { return viewport_; }
    runview::SceneModel& scene() { return scene_; }
    runview::HudModel& hud() { return hud_; }
    ZonePanel* zonePanel() const { return zones_; }
    ClientBanner* banner() const { return banner_; }
    DiagnosticsOverlay* diagnostics() const { return diagnostics_; }
    QPushButton* leaveButton() const { return leave_; }
    QLabel* whoLabel() const { return who_; }
    QToolButton* diagnosticsButton() const { return diagnosticsButton_; }

    bool diagnosticsVisible() const { return diagnosticsButton_->isChecked(); }
    void setDiagnosticsVisible(bool visible) { diagnosticsButton_->setChecked(visible); }

    // Key events (the window's event filter or tests) feed the keyboard joystick. True when consumed.
    bool handleKey(int qtKey, bool down);

    // Redraws from the controller's latest picture (the controller's signals call it; tests call it directly).
    void refreshScene();
    void refreshView();
    void refreshPresentation();

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    void build();
    void placeOverlays();
    bool keysReach() const;

    ClientController& controller_;
    runview::JoystickManager* joysticks_;
    runview::ViewportTheme theme_;
    runview::ViewportOptions options_;
    runview::SceneModel scene_;
    runview::HudModel hud_;
    runview::RunViewport* viewport_ = nullptr;
    ZonePanel* zones_ = nullptr;
    ClientBanner* banner_ = nullptr;
    DiagnosticsOverlay* diagnostics_ = nullptr;
    QLabel* who_ = nullptr;
    QLabel* state_ = nullptr;
    QPushButton* leave_ = nullptr;
    QToolButton* diagnosticsButton_ = nullptr;
    QMenu* cameraMenu_ = nullptr;
    QAction* homeCamera_ = nullptr;
    QAction* driverCamera_ = nullptr;
    QAction* followCamera_ = nullptr;

    std::uint64_t sceneRevision_ = ~std::uint64_t{0};
    std::uint64_t rosterRevision_ = ~std::uint64_t{0};
    bool cameraPlaced_ = false;
    qint64 lastCameraNs_ = 0;
    QElapsedTimer clock_;
};

}  // namespace wpiemu::app::client
