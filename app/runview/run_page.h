#pragma once

#include <QList>
#include <QSettings>
#include <QWidget>

#include <memory>

#include "console_panel.h"
#include "driver_station_model.h"
#include "driver_station_panel.h"
#include "hud_model.h"
#include "joystick_manager.h"
#include "mechanism_overlay_model.h"
#include "run_backend.h"
#include "run_control.h"
#include "run_viewport.h"
#include "scene_model.h"
#include "sim_stats.h"
#include "status_chip.h"

class QAction;
class QComboBox;
class QLabel;
class QMenu;
class QSplitter;
class QTimer;
class QToolBar;

namespace wpiemu::app::runview {

struct RunPageOptions {
    QSettings* settings = nullptr;               // borrowed; null = a default QSettings owned by the page
    IJoystickProvider* gamepads = nullptr;       // borrowed; null = see useDefaultGamepadBackend
    bool useDefaultGamepadBackend = true;        // create the build's gamepad backend (SDL2) when `gamepads` is null
};

// The run-time view: live 3D scene with game HUD, console and the built-in
// Driver Station. All logic lives in view models; the page only wires them to an IRunBackend.
class RunPage : public QWidget {
    Q_OBJECT
public:
    explicit RunPage(IRunBackend& backend, QWidget* parent = nullptr);
    RunPage(IRunBackend& backend, RunPageOptions options, QWidget* parent = nullptr);
    ~RunPage() override;

    // ---- collaborators (also used by tests and the demo) ----
    ConsolePanel* console() const { return console_; }
    DriverStationPanel* driverStationPanel() const { return dsPanel_; }
    DriverStationModel& driverStation() { return ds_; }
    JoystickManager& joysticks() { return joysticks_; }
    SceneModel& scene() { return scene_; }
    HudModel& hud() { return hud_; }
    MechanismOverlayModel& mechanisms() { return mechanisms_; }
    RunViewport* viewport() const { return viewport_; }
    const SimStatsValues& stats() const { return stats_.values(); }

    QAction* restartAction() const { return restartAction_; }
    QAction* restartRobotAction() const { return restartRobotAction_; }
    // Overlay toggles, the zone submenu and the console toggle; listed in the shell's View menu.
    QList<QAction*> viewActions() const;
    QComboBox* speedCombo() const { return speedCombo_; }

    // Polls the backend once (normally driven by a ~60 Hz timer): snapshot -> view models.
    void pollNow();
    // Samples joysticks and publishes the DS state once (normally a 50 Hz timer).
    void publishDriverStation();
    void setConsoleVisible(bool visible);
    bool consoleVisible() const;

    // Key events from the application feed the keyboard joystick (see eventFilter). Public for tests.
    bool handleKey(int qtKey, bool down);

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;
    void showEvent(QShowEvent*) override;
    void hideEvent(QHideEvent*) override;
    void changeEvent(QEvent*) override;

private:
    void build();
    void buildActions();
    void applyStatus();
    void refreshControls(const sdk::ISimSnapshot* snapshot);
    void refreshStatsLabels();
    void rebuildZoneMenu();
    void onSceneInfoChanged();

    IRunBackend& backend_;
    RunPageOptions options_;
    std::unique_ptr<QSettings> ownedSettings_;
    std::unique_ptr<IJoystickProvider> ownedProvider_;
    QSettings* settings_;

    DriverStationModel ds_;
    JoystickManager joysticks_;
    SceneModel scene_;
    HudModel hud_;
    MechanismOverlayModel mechanisms_;
    ViewportTheme theme_;
    ViewportOptions viewOptions_;
    SimStats stats_;

    ConsolePanel* console_ = nullptr;
    DriverStationPanel* dsPanel_ = nullptr;
    RunViewport* viewport_ = nullptr;
    QSplitter* verticalSplit_ = nullptr;
    QComboBox* speedCombo_ = nullptr;
    QMenu* zoneMenu_ = nullptr;
    QAction* restartAction_ = nullptr;
    QAction* restartRobotAction_ = nullptr;
    QMenu* restartMenu_ = nullptr;
    QAction* consoleAction_ = nullptr;
    QAction* hudAction_ = nullptr;
    QAction* mechAction_ = nullptr;
    QAction* zonesAction_ = nullptr;
    QTimer* pollTimer_ = nullptr;
    QTimer* dsTimer_ = nullptr;

    qint64 lastLabelMs_ = 0;
    bool consoleAutoShown_ = false;
};

}  // namespace wpiemu::app::runview
