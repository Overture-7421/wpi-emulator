#pragma once

#include "joystick_source.h"

#ifdef WPIEMU_HAS_SDL2

#include <memory>
#include <vector>

namespace wpiemu::app::runview {

class SdlGamepadSource;

// SDL2 GameController backend: normalises every recognised gamepad to the WPILib Xbox layout.
class SdlGamepadProvider final : public IJoystickProvider {
    Q_OBJECT
public:
    explicit SdlGamepadProvider(QObject* parent = nullptr);
    ~SdlGamepadProvider() override;

    bool initialized() const { return initialized_; }
    QString backendName() const override { return QStringLiteral("SDL2"); }
    void pump() override;
    std::vector<std::shared_ptr<IJoystickSource>> sources() const override;

private:
    void open(int deviceIndex);
    void close(int instanceId);

    bool initialized_ = false;
    std::vector<std::shared_ptr<SdlGamepadSource>> pads_;
};

}  // namespace wpiemu::app::runview

#endif  // WPIEMU_HAS_SDL2
