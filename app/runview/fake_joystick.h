#pragma once

// Scriptable gamepad provider for tests, the demo and machines without a gamepad backend.

#include "joystick_source.h"

namespace wpiemu::app::runview {

class FakeGamepadSource final : public IJoystickSource {
public:
    FakeGamepadSource(QString id, QString name);

    JoystickSourceInfo info() const override;
    bool connected() const override { return connected_; }
    joy::RawInput poll() override { return state_; }

    void setConnected(bool c) { connected_ = c; }
    void setAxis(int index, float value);
    void setButton(int index, bool down);
    void setPov(int degrees) { state_.povs[0] = static_cast<std::int16_t>(degrees); }

private:
    QString id_, name_;
    bool connected_ = true;
    joy::RawInput state_;
};

class FakeJoystickProvider final : public IJoystickProvider {
    Q_OBJECT
public:
    explicit FakeJoystickProvider(QObject* parent = nullptr) : IJoystickProvider(parent) {}

    QString backendName() const override { return QStringLiteral("fake"); }
    void pump() override {}
    std::vector<std::shared_ptr<IJoystickSource>> sources() const override;

    std::shared_ptr<FakeGamepadSource> addGamepad(const QString& name);
    void removeGamepad(const QString& id);

private:
    std::vector<std::shared_ptr<FakeGamepadSource>> pads_;
};

}  // namespace wpiemu::app::runview
