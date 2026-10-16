#include "joystick_source.h"

#include <QCoreApplication>

#include "sdl_gamepad_provider.h"

namespace wpiemu::app::runview {

JoystickSourceInfo KeyboardJoystickSource::info() const {
    return {QStringLiteral("keyboard"), QCoreApplication::translate("Joystick", "Keyboard"), true, joy::kAxisCount,
            joy::kButtonCount, joy::kPovCount};
}

joy::RawInput KeyboardJoystickSource::poll() {
    joy::RawInput raw;
    raw.keys = keys_;
    return raw;
}

void KeyboardJoystickSource::setKey(int qtKey, bool down) {
    if (down) keys_.insert(qtKey);
    else keys_.erase(qtKey);
}

std::unique_ptr<IJoystickProvider> createDefaultJoystickProvider() {
#ifdef WPIEMU_HAS_SDL2
    auto provider = std::make_unique<SdlGamepadProvider>();
    if (provider->initialized()) return provider;
#endif
    return nullptr;
}

}  // namespace wpiemu::app::runview
