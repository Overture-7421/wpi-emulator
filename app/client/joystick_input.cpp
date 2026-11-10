#include "client/joystick_input.h"

namespace wpiemu::app::client {

sdk::proto::JoystickState JoystickClientInput::sample() {
    const auto& states = joysticks_.sample();
    const sdk::proto::JoystickState* keyboard = nullptr;
    for (int slot = 0; slot < runview::joy::kSlotCount; ++slot) {
        const sdk::proto::JoystickState& state = states[static_cast<std::size_t>(slot)];
        if (!state.present) continue;
        if (!joysticks_.usesKeyboard(slot)) return state;
        if (keyboard == nullptr) keyboard = &state;
    }
    return keyboard != nullptr ? *keyboard : sdk::proto::JoystickState{};
}

}  // namespace wpiemu::app::client
