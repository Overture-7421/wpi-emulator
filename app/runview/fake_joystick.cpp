#include "fake_joystick.h"

#include <QCoreApplication>

#include <algorithm>

namespace wpiemu::app::runview {

FakeGamepadSource::FakeGamepadSource(QString id, QString name) : id_(std::move(id)), name_(std::move(name)) {
    state_.axes.assign(joy::kAxisCount, 0.f);
    state_.buttons.assign(10, 0);
    state_.povs.assign(1, -1);
}

JoystickSourceInfo FakeGamepadSource::info() const { return {id_, name_, false, joy::kAxisCount, 10, 1}; }

void FakeGamepadSource::setAxis(int index, float value) {
    if (index >= 0 && index < static_cast<int>(state_.axes.size())) state_.axes[static_cast<std::size_t>(index)] = value;
}

void FakeGamepadSource::setButton(int index, bool down) {
    if (index >= 0 && index < static_cast<int>(state_.buttons.size())) state_.buttons[static_cast<std::size_t>(index)] = down ? 1 : 0;
}

std::vector<std::shared_ptr<IJoystickSource>> FakeJoystickProvider::sources() const {
    return {pads_.begin(), pads_.end()};
}

std::shared_ptr<FakeGamepadSource> FakeJoystickProvider::addGamepad(const QString& name) {
    int ordinal = 0;
    for (const auto& p : pads_)
        if (p->info().name == name) ++ordinal;
    auto pad = std::make_shared<FakeGamepadSource>(QStringLiteral("pad:%1#%2").arg(name).arg(ordinal), name);
    pads_.push_back(pad);
    emit sourcesChanged();
    return pad;
}

void FakeJoystickProvider::removeGamepad(const QString& id) {
    const auto it = std::find_if(pads_.begin(), pads_.end(), [&](const auto& p) { return p->info().id == id; });
    if (it == pads_.end()) return;
    (*it)->setConnected(false);
    pads_.erase(it);
    emit sourcesChanged();
}

}  // namespace wpiemu::app::runview
