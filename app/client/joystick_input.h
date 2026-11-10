#pragma once

// The run view's joystick bindings as the client's drive input: a gamepad when one is present (the first one), else the keyboard.
// The same JoystickManager and profiles as the desktop app, so a player's bindings carry over.

#include "client/client_controller.h"
#include "runview/joystick_manager.h"

namespace wpiemu::app::client {

class JoystickClientInput final : public IClientInput {
public:
    explicit JoystickClientInput(runview::JoystickManager& joysticks) : joysticks_(joysticks) {}

    sdk::proto::JoystickState sample() override;

private:
    runview::JoystickManager& joysticks_;
};

}  // namespace wpiemu::app::client
