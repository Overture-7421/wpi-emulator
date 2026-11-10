#pragma once

// The game client's window: the join screen until the host accepts the player, then the drive screen. It composes the pieces (services,
// settings, joystick bindings, controller) and runs the controller's timer; everything it decides is in the controller.

#include <QMainWindow>

#include <memory>

#include "client/client_controller.h"
#include "client/drive_page.h"
#include "client/join_page.h"
#include "client/joystick_input.h"
#include "runview/joystick_manager.h"
#include "ui/key_value_store.h"

class QSettings;
class QStackedWidget;
class QTimer;

namespace wpiemu::app::client {

struct ClientWindowOptions {
    QString settingsFile;                          // empty: the user's settings (QSettings); a path: that ini file (tests, portable)
    std::unique_ptr<IClientSession> session;       // null: the network session; the demo hands a scripted one
    bool useGamepads = true;                       // the build's gamepad backend (SDL2) when there is one
    std::shared_ptr<sdk::IWallClock> clock;        // null: the steady clock
    int tickIntervalMs = 16;
};

class ClientWindow : public QMainWindow {
    Q_OBJECT
public:
    // Fails (message) when the registries or plugins cannot be set up; the window is then not usable.
    explicit ClientWindow(ClientWindowOptions options = {}, QWidget* parent = nullptr);
    ~ClientWindow() override;

    ClientController& controller() { return *controller_; }
    JoinPage* joinPage() const { return join_; }
    DrivePage* drivePage() const { return drive_; }
    QStackedWidget* stack() const { return stack_; }
    runview::JoystickManager& joysticks() { return *joysticks_; }
    bool showingDrive() const;
    QTimer* timer() const { return timer_; }

private:
    void showScreen();

    std::unique_ptr<ClientServices> services_;
    std::unique_ptr<QSettings> settings_;
    std::unique_ptr<IKeyValueStore> store_;
    std::unique_ptr<runview::IJoystickProvider> gamepads_;
    std::unique_ptr<runview::JoystickManager> joysticks_;
    std::unique_ptr<JoystickClientInput> input_;
    std::unique_ptr<ClientController> controller_;
    QStackedWidget* stack_ = nullptr;
    JoinPage* join_ = nullptr;
    DrivePage* drive_ = nullptr;
    QTimer* timer_ = nullptr;
};

}  // namespace wpiemu::app::client
