#pragma once

// Joystick binding model: how raw inputs (keyboard keys, gamepad axes / buttons / d-pad) map onto the
// DS joystick a robot program sees (axes, buttons, POV). Qt-core only; fully unit-tested.

#include <QString>
#include <QVariantMap>

#include <array>
#include <cstdint>
#include <set>
#include <vector>

#include "wpiemu/sdk/ipc/driver_station.h"

namespace wpiemu::app::runview::joy {

inline constexpr int kSlotCount = 6;     // DS joystick ports 0..5
inline constexpr int kAxisCount = 6;     // WPILib XboxController layout: LX LY LT RT RX RY
inline constexpr int kButtonCount = 12;
inline constexpr int kPovCount = 1;

struct Input {
    enum class Kind { None, Key, PadAxis, PadButton, PadPov };
    Kind kind = Kind::None;
    int code = 0;   // Qt::Key for Key; axis / button / pov index otherwise

    static Input none() { return {}; }
    static Input key(int qtKey) { return {Kind::Key, qtKey}; }
    static Input padAxis(int axis) { return {Kind::PadAxis, axis}; }
    static Input padButton(int button) { return {Kind::PadButton, button}; }
    static Input padPov(int pov = 0) { return {Kind::PadPov, pov}; }

    bool isNone() const { return kind == Kind::None; }
    bool operator==(const Input&) const = default;

    QString toString() const;                       // stable persisted form: "key:87", "axis:0", "none"
    static Input fromString(const QString& text);
    QString displayText() const;                    // user-facing: "W", "Axis 3", "Button 1", "D-pad"
};

struct AxisBinding {
    Input first;    // PadAxis, or the key driving the axis toward -1
    Input second;   // key driving the axis toward +1 (keyboard only)
    bool invert = false;
    float deadband = 0.08f;   // gamepad axes only
    bool operator==(const AxisBinding&) const = default;
};

struct PovBinding {
    Input up, right, down, left;   // keys
    Input pad;                     // PadPov
    bool operator==(const PovBinding&) const = default;
};

struct JoystickProfile {
    std::array<AxisBinding, kAxisCount> axes;
    std::array<Input, kButtonCount> buttons;
    PovBinding pov;

    bool operator==(const JoystickProfile&) const = default;

    static JoystickProfile defaultKeyboard();
    static JoystickProfile defaultGamepad();

    QVariantMap toVariant() const;
    static JoystickProfile fromVariant(const QVariantMap& map, const JoystickProfile& fallback);
};

// Latest raw device state. A keyboard source fills `keys`; a gamepad fills axes/buttons/povs
// (WPILib Xbox ordering, see joystick_source.h).
struct RawInput {
    std::set<int> keys;
    std::vector<float> axes;
    std::vector<std::uint8_t> buttons;
    std::vector<std::int16_t> povs;   // degrees, -1 released
};

float evaluateAxis(const AxisBinding& binding, const RawInput& raw);
bool evaluateButton(const Input& input, const RawInput& raw);
std::int16_t evaluatePov(const PovBinding& binding, const RawInput& raw);

// `present` = a device is assigned and connected; absent joysticks report present=false and empty arrays.
sdk::proto::JoystickState evaluate(const JoystickProfile& profile, const RawInput& raw, bool present);

QString axisName(int axis);   // "Left X", "Left Y", "Left trigger", ...
QString buttonName(int button);

}  // namespace wpiemu::app::runview::joy
