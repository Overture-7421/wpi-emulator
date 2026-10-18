#pragma once

#include <QObject>
#include <QString>

#include <memory>
#include <vector>

#include "joystick_types.h"

namespace wpiemu::app::runview {

struct JoystickSourceInfo {
    QString id;            // stable per device: "keyboard", "pad:<name>#<n>"
    QString name;          // "Keyboard", "Xbox Controller"
    bool keyboard = false;
    int axisCount = 0;
    int buttonCount = 0;
    int povCount = 0;

    bool operator==(const JoystickSourceInfo&) const = default;
};

// One physical input device. Gamepad sources report the WPILib XboxController layout:
//   axes    0 LX, 1 LY, 2 LT (0..1), 3 RT (0..1), 4 RX, 5 RY   (up/left = negative, like WPILib)
//   buttons 0 A, 1 B, 2 X, 3 Y, 4 LB, 5 RB, 6 Back, 7 Start, 8 L-stick, 9 R-stick
//   pov     0 d-pad in degrees (-1 released)
class IJoystickSource {
public:
    virtual ~IJoystickSource() = default;
    virtual JoystickSourceInfo info() const = 0;
    virtual bool connected() const = 0;
    virtual joy::RawInput poll() = 0;   // current state
};

// Enumerates gamepads (SDL2, a fake in tests, ...). Owned by the composition root; the run view only
// borrows it.
class IJoystickProvider : public QObject {
    Q_OBJECT
public:
    using QObject::QObject;
    virtual QString backendName() const = 0;
    virtual void pump() = 0;   // process hot-plug events; call regularly from the UI thread
    virtual std::vector<std::shared_ptr<IJoystickSource>> sources() const = 0;

signals:
    void sourcesChanged();
};

// Keys held down in the run view act as a joystick.
class KeyboardJoystickSource final : public IJoystickSource {
public:
    JoystickSourceInfo info() const override;
    bool connected() const override { return true; }
    joy::RawInput poll() override;

    void setKey(int qtKey, bool down);
    void releaseAll() { keys_.clear(); }

private:
    std::set<int> keys_;
};

// Gamepad backend for this build (SDL2), or nullptr when the build has none / it failed to initialise.
// Keyboard input always works regardless.
std::unique_ptr<IJoystickProvider> createDefaultJoystickProvider();

}  // namespace wpiemu::app::runview
