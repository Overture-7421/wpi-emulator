#include "sdl_gamepad_provider.h"

#ifdef WPIEMU_HAS_SDL2

#define SDL_MAIN_HANDLED
#include <SDL.h>

#include <algorithm>

namespace wpiemu::app::runview {

class SdlGamepadSource final : public IJoystickSource {
public:
    SdlGamepadSource(SDL_GameController* controller, QString id, QString name)
        : controller_(controller), id_(std::move(id)), name_(std::move(name)),
          instanceId_(SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(controller))) {}
    ~SdlGamepadSource() override { release(); }

    JoystickSourceInfo info() const override { return {id_, name_, false, joy::kAxisCount, 10, 1}; }
    bool connected() const override { return controller_ != nullptr; }
    int instanceId() const { return instanceId_; }

    joy::RawInput poll() override {
        joy::RawInput raw;
        raw.axes.assign(joy::kAxisCount, 0.f);
        raw.buttons.assign(10, 0);
        raw.povs.assign(1, -1);
        if (controller_ == nullptr) return raw;
        auto stick = [&](SDL_GameControllerAxis a) {
            const int v = SDL_GameControllerGetAxis(controller_, a);
            return std::clamp(static_cast<float>(v) / (v < 0 ? 32768.f : 32767.f), -1.f, 1.f);
        };
        auto trigger = [&](SDL_GameControllerAxis a) { return std::clamp(static_cast<float>(SDL_GameControllerGetAxis(controller_, a)) / 32767.f, 0.f, 1.f); };
        raw.axes[0] = stick(SDL_CONTROLLER_AXIS_LEFTX);
        raw.axes[1] = stick(SDL_CONTROLLER_AXIS_LEFTY);
        raw.axes[2] = trigger(SDL_CONTROLLER_AXIS_TRIGGERLEFT);
        raw.axes[3] = trigger(SDL_CONTROLLER_AXIS_TRIGGERRIGHT);
        raw.axes[4] = stick(SDL_CONTROLLER_AXIS_RIGHTX);
        raw.axes[5] = stick(SDL_CONTROLLER_AXIS_RIGHTY);
        static constexpr SDL_GameControllerButton kButtons[10] = {
            SDL_CONTROLLER_BUTTON_A,          SDL_CONTROLLER_BUTTON_B,         SDL_CONTROLLER_BUTTON_X,
            SDL_CONTROLLER_BUTTON_Y,          SDL_CONTROLLER_BUTTON_LEFTSHOULDER, SDL_CONTROLLER_BUTTON_RIGHTSHOULDER,
            SDL_CONTROLLER_BUTTON_BACK,       SDL_CONTROLLER_BUTTON_START,     SDL_CONTROLLER_BUTTON_LEFTSTICK,
            SDL_CONTROLLER_BUTTON_RIGHTSTICK};
        for (int i = 0; i < 10; ++i) raw.buttons[static_cast<std::size_t>(i)] = SDL_GameControllerGetButton(controller_, kButtons[i]);
        const int up = SDL_GameControllerGetButton(controller_, SDL_CONTROLLER_BUTTON_DPAD_UP);
        const int down = SDL_GameControllerGetButton(controller_, SDL_CONTROLLER_BUTTON_DPAD_DOWN);
        const int left = SDL_GameControllerGetButton(controller_, SDL_CONTROLLER_BUTTON_DPAD_LEFT);
        const int right = SDL_GameControllerGetButton(controller_, SDL_CONTROLLER_BUTTON_DPAD_RIGHT);
        const int dx = right - left, dy = up - down;
        if (dx != 0 || dy != 0) {
            static constexpr int kAngles[3][3] = {{225, 270, 315}, {180, -1, 0}, {135, 90, 45}};  // [dx + 1][dy + 1], dy > 0 = up
            raw.povs[0] = static_cast<std::int16_t>(kAngles[dx + 1][dy + 1]);
        }
        return raw;
    }

    void release() {
        if (controller_ != nullptr) {
            SDL_GameControllerClose(controller_);
            controller_ = nullptr;
        }
    }

private:
    SDL_GameController* controller_;
    QString id_, name_;
    int instanceId_;
};

SdlGamepadProvider::SdlGamepadProvider(QObject* parent) : IJoystickProvider(parent) {
    SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
    initialized_ = SDL_InitSubSystem(SDL_INIT_GAMECONTROLLER | SDL_INIT_EVENTS) == 0;
}

SdlGamepadProvider::~SdlGamepadProvider() {
    pads_.clear();
    if (initialized_) SDL_QuitSubSystem(SDL_INIT_GAMECONTROLLER | SDL_INIT_EVENTS);
}

void SdlGamepadProvider::open(int deviceIndex) {
    SDL_GameController* c = SDL_GameControllerOpen(deviceIndex);
    if (c == nullptr) return;
    const QString name = QString::fromUtf8(SDL_GameControllerName(c));
    int ordinal = 0;
    for (const auto& p : pads_)
        if (p->info().name == name) ++ordinal;
    pads_.push_back(std::make_shared<SdlGamepadSource>(c, QStringLiteral("pad:%1#%2").arg(name).arg(ordinal), name));
    emit sourcesChanged();
}

void SdlGamepadProvider::close(int instanceId) {
    const auto it = std::find_if(pads_.begin(), pads_.end(), [&](const auto& p) { return p->instanceId() == instanceId; });
    if (it == pads_.end()) return;
    (*it)->release();
    pads_.erase(it);
    emit sourcesChanged();
}

void SdlGamepadProvider::pump() {
    if (!initialized_) return;
    SDL_Event e;
    while (SDL_PollEvent(&e) != 0) {
        if (e.type == SDL_CONTROLLERDEVICEADDED) open(e.cdevice.which);
        else if (e.type == SDL_CONTROLLERDEVICEREMOVED) close(e.cdevice.which);
    }
}

std::vector<std::shared_ptr<IJoystickSource>> SdlGamepadProvider::sources() const {
    return {pads_.begin(), pads_.end()};
}

}  // namespace wpiemu::app::runview

#endif  // WPIEMU_HAS_SDL2
