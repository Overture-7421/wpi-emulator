#pragma once

#include <QList>
#include <QElapsedTimer>
#include <QSettings>
#include <QWidget>

#include <memory>
#include <vector>

#include "console_panel.h"
#include "contract/runtime_host.h"
#include "driver_station_model.h"
#include "driver_station_panel.h"
#include "hud_model.h"
#include "joystick_manager.h"
#include "run_control.h"
#include "run_viewport.h"
#include "scene_model.h"
#include "sfx_output.h"
#include "sfx_player.h"
#include "sim_stats.h"
#include "state_window.h"
#include "status_chip.h"

class QAction;
class QActionGroup;
class QComboBox;
class QLabel;
class QMenu;
class QSplitter;
class QTimer;
class QToolBar;
class QToolButton;

namespace wpiemu::app::runview {

struct RunPageOptions {
    QSettings* settings = nullptr;               // borrowed; null = a default QSettings owned by the page
    IJoystickProvider* gamepads = nullptr;       // borrowed; null = see useDefaultGamepadBackend
    bool useDefaultGamepadBackend = true;        // create the build's gamepad backend (SDL2) when `gamepads` is null
    ISfxOutput* sfx = nullptr;                   // borrowed; null = see useDefaultSfxBackend
    bool useDefaultSfxBackend = true;            // create the build's sound output (Qt Multimedia) when `sfx` is null; false = silent
};

// The run-time view: live 3D scene with game HUD, console and the built-in
// Driver Station. All logic lives in view models; the page only wires them to an IRuntimeHost.
class RunPage : public QWidget {
    Q_OBJECT
public:
    explicit RunPage(IRuntimeHost& host, QWidget* parent = nullptr);
    RunPage(IRuntimeHost& host, RunPageOptions options, QWidget* parent = nullptr);
    ~RunPage() override;

    // ---- collaborators (also used by tests and the demo) ----
    ConsolePanel* console() const { return console_; }
    DriverStationPanel* driverStationPanel() const { return dsPanel_; }
    DriverStationModel& driverStation() { return ds_; }
    JoystickManager& joysticks() { return joysticks_; }
    SfxPlayer& sfx() { return sfx_; }
    // The user's sound preferences (File > Settings); the shell applies them at startup and when they change.
    void setSoundEffects(bool enabled, int volumePercent);
    SceneModel& scene() { return scene_; }
    HudModel& hud() { return hud_; }
    ViewportOptions& viewOptions() { return viewOptions_; }
    RunViewport* viewport() const { return viewport_; }
    StateWindow* stateWindow() const { return stateWindow_; }
    QAction* stateAction() const { return stateAction_; }
    const SimStatsValues& stats() const { return stats_.values(); }

    QAction* restartAction() const { return restartAction_; }
    QAction* restartRobotAction() const { return restartRobotAction_; }
    // Overlay toggles, the zone submenu, the Robot state window and the console toggle; listed in the shell's View menu.
    QList<QAction*> viewActions() const;
    QComboBox* speedCombo() const { return speedCombo_; }

    // Polls the host once (normally driven by a ~144 Hz timer): snapshot -> view models.
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

signals:
    // The overlay's fix button for a failure the shell can help with (a ProblemAction: 1 = open the setup, 2 = open Settings).
    void problemActionRequested(int action);

private:
    void build();
    bool keysReachJoystick() const;
    void buildActions();
    void applyStatus();
    void refreshControls(const sdk::ISimSnapshot* snapshot);
    void refreshStatsLabels();
    void rebuildZoneMenu();
    void rebuildStartMenu();
    void selectStartPosition(const QString& id);
    void onSceneChanged();
    void applyProjectDefaults(const HostScene& scene);
    void setStateWindowVisible(bool visible);
    void saveStateWindowGeometry();
    void updateDriverPovLabel();
    void syncCameraActions();

    IRuntimeHost& host_;
    RunPageOptions options_;
    std::unique_ptr<QSettings> ownedSettings_;
    std::unique_ptr<IJoystickProvider> ownedProvider_;
    std::unique_ptr<ISfxOutput> ownedSfx_;
    QSettings* settings_;

    DriverStationModel ds_;
    JoystickManager joysticks_;
    SfxPlayer sfx_;
    SceneModel scene_;
    HudModel hud_;
    ViewportTheme theme_;
    ViewportOptions viewOptions_;
    SimStats stats_;

    ConsolePanel* console_ = nullptr;
    DriverStationPanel* dsPanel_ = nullptr;
    RunViewport* viewport_ = nullptr;
    QSplitter* verticalSplit_ = nullptr;
    QComboBox* speedCombo_ = nullptr;
    QMenu* zoneMenu_ = nullptr;
    QMenu* startMenu_ = nullptr;
    QMenu* cameraMenu_ = nullptr;
    QActionGroup* cameraActionGroup_ = nullptr;
    QAction* homeCameraAction_ = nullptr;
    QAction* driverCameraAction_ = nullptr;
    QAction* followCameraAction_ = nullptr;
    std::vector<QAction*> stationCameraActions_;
    QToolButton* startButton_ = nullptr;
    QToolButton* allianceButton_ = nullptr;
    void updateAllianceButton();
    std::vector<sdk::StartPosition> startPositions_;
    QAction* restartAction_ = nullptr;
    QAction* restartRobotAction_ = nullptr;
    QMenu* restartMenu_ = nullptr;
    QAction* consoleAction_ = nullptr;
    QAction* hudAction_ = nullptr;
    QAction* stateAction_ = nullptr;
    StateWindow* stateWindow_ = nullptr;
    QAction* zonesAction_ = nullptr;
    QAction* tagsAction_ = nullptr;
    QTimer* pollTimer_ = nullptr;
    QTimer* dsTimer_ = nullptr;

    qint64 lastLabelMs_ = 0;
    QElapsedTimer cameraClock_;
    qint64 lastCameraNs_ = 0;
    CameraView initialCameraView_;
    bool cameraViewRestored_ = false;
    bool consoleAutoShown_ = false;
    // The project (and its default station) last applied to the Driver Station panel.
    QString defaultsProject_;
    std::optional<sdk::proto::AllianceStation> defaultsStation_;
};

}  // namespace wpiemu::app::runview
