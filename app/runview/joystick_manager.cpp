#include "joystick_manager.h"

#include <QJsonDocument>

#include <algorithm>
#include <cmath>

namespace wpiemu::app::runview {
namespace {

QString toJson(const joy::JoystickProfile& p) {
    return QString::fromUtf8(QJsonDocument::fromVariant(p.toVariant()).toJson(QJsonDocument::Compact));
}

joy::JoystickProfile fromJson(const QString& text, const joy::JoystickProfile& fallback) {
    if (text.isEmpty()) return fallback;
    const QJsonDocument doc = QJsonDocument::fromJson(text.toUtf8());
    return doc.isObject() ? joy::JoystickProfile::fromVariant(doc.toVariant().toMap(), fallback) : fallback;
}

QString slotKey(int slot, const char* leaf) { return QStringLiteral("runview/joystick/%1/%2").arg(slot).arg(QLatin1String(leaf)); }

}  // namespace

JoystickManager::JoystickManager(IJoystickProvider* provider, QSettings* settings, QObject* parent)
    : QObject(parent), provider_(provider), settings_(settings) {
    qRegisterMetaType<JoystickSourceInfo>();
    load();
    if (provider_ != nullptr) {
        provider_->pump();
        connect(provider_, &IJoystickProvider::sourcesChanged, this, [this] { onSourcesChanged(false); });
        onSourcesChanged(true);
    }
}

void JoystickManager::load() {
    firstRun_ = settings_ == nullptr || !settings_->contains(QStringLiteral("runview/joystick/initialized"));
    for (int i = 0; i < joy::kSlotCount; ++i) {
        SlotConfig cfg;
        if (settings_ != nullptr) {
            cfg.sourceId = settings_->value(slotKey(i, "source")).toString();
            cfg.keyboard = fromJson(settings_->value(slotKey(i, "keyboard")).toString(), cfg.keyboard);
            cfg.gamepad = fromJson(settings_->value(slotKey(i, "gamepad")).toString(), cfg.gamepad);
        }
        if (firstRun_ && i == 0) cfg.sourceId = QLatin1String(kKeyboardId);
        slots_[static_cast<std::size_t>(i)] = std::move(cfg);
    }
}

void JoystickManager::save() const {
    if (settings_ == nullptr) return;
    settings_->setValue(QStringLiteral("runview/joystick/initialized"), true);
    for (int i = 0; i < joy::kSlotCount; ++i) {
        const SlotConfig& cfg = slots_[static_cast<std::size_t>(i)];
        settings_->setValue(slotKey(i, "source"), cfg.sourceId);
        settings_->setValue(slotKey(i, "keyboard"), toJson(cfg.keyboard));
        settings_->setValue(slotKey(i, "gamepad"), toJson(cfg.gamepad));
    }
}

std::shared_ptr<IJoystickSource> JoystickManager::sourceFor(const QString& id) const {
    if (id.isEmpty() || provider_ == nullptr) return nullptr;
    for (const auto& s : provider_->sources())
        if (s->info().id == id) return s;
    return nullptr;
}

const joy::JoystickProfile& JoystickManager::profile(int slotIndex) const {
    const SlotConfig& cfg = slot(slotIndex);
    return cfg.sourceId == QLatin1String(kKeyboardId) ? cfg.keyboard : cfg.gamepad;
}

joy::JoystickProfile& JoystickManager::activeProfile(int slotIndex) {
    SlotConfig& cfg = slots_[static_cast<std::size_t>(slotIndex)];
    return cfg.sourceId == QLatin1String(kKeyboardId) ? cfg.keyboard : cfg.gamepad;
}

void JoystickManager::setSourceId(int slotIndex, const QString& id) {
    if (slotIndex < 0 || slotIndex >= joy::kSlotCount || slot(slotIndex).sourceId == id) return;
    if (learning()) cancelLearn();
    slots_[static_cast<std::size_t>(slotIndex)].sourceId = id;
    save();
    emit slotChanged(slotIndex);
}

void JoystickManager::setProfile(int slotIndex, const joy::JoystickProfile& p) {
    if (slotIndex < 0 || slotIndex >= joy::kSlotCount) return;
    activeProfile(slotIndex) = p;
    save();
    emit slotChanged(slotIndex);
}

void JoystickManager::resetProfile(int slotIndex) {
    setProfile(slotIndex, usesKeyboard(slotIndex) ? joy::JoystickProfile::defaultKeyboard() : joy::JoystickProfile::defaultGamepad());
}

std::vector<JoystickSourceInfo> JoystickManager::gamepads() const {
    std::vector<JoystickSourceInfo> out;
    if (provider_ != nullptr)
        for (const auto& s : provider_->sources())
            if (s->connected()) out.push_back(s->info());
    return out;
}

std::vector<JoystickSourceInfo> JoystickManager::availableSources() const {
    std::vector<JoystickSourceInfo> out{keyboard_.info()};
    const auto pads = gamepads();
    out.insert(out.end(), pads.begin(), pads.end());
    return out;
}

bool JoystickManager::slotConnected(int slotIndex) const {
    const QString& id = slot(slotIndex).sourceId;
    if (id == QLatin1String(kKeyboardId)) return true;
    const auto src = sourceFor(id);
    return src != nullptr && src->connected();
}

QString JoystickManager::slotDeviceName(int slotIndex) const {
    const QString& id = slot(slotIndex).sourceId;
    if (id.isEmpty()) return {};
    if (id == QLatin1String(kKeyboardId)) return keyboard_.info().name;
    if (const auto src = sourceFor(id)) return src->info().name;
    // Not currently connected: show the remembered name from the id ("pad:<name>#n").
    return id.mid(4).section(QLatin1Char('#'), 0, 0);
}

const std::array<sdk::proto::JoystickState, joy::kSlotCount>& JoystickManager::sample() {
    if (provider_ != nullptr) provider_->pump();
    for (int i = 0; i < joy::kSlotCount; ++i) {
        const auto idx = static_cast<std::size_t>(i);
        const QString& id = slots_[idx].sourceId;
        bool present = false;
        joy::RawInput raw;
        if (id == QLatin1String(kKeyboardId)) {
            present = true;
            raw = keyboard_.poll();
        } else if (const auto src = sourceFor(id); src != nullptr && src->connected()) {
            present = true;
            raw = src->poll();
        }
        if (learning() && learnSlot_ == i && present && id != QLatin1String(kKeyboardId)) pollLearning(i, raw);
        states_[idx] = joy::evaluate(profile(i), raw, present);
        raw_[idx] = std::move(raw);
    }
    return states_;
}

bool JoystickManager::handleKey(int qtKey, bool down) {
    if (learning() && down) {
        if (qtKey == Qt::Key_Escape) {
            cancelLearn();
            return true;
        }
        if (usesKeyboard(learnSlot_)) {
            applyLearned(joy::Input::key(qtKey));
            return true;
        }
    }
    keyboard_.setKey(qtKey, down);
    return false;
}

void JoystickManager::releaseAllKeys() { keyboard_.releaseAll(); }

void JoystickManager::beginLearn(int slotIndex, LearnTarget target) {
    if (slotIndex < 0 || slotIndex >= joy::kSlotCount || slot(slotIndex).sourceId.isEmpty()) return;
    learnSlot_ = slotIndex;
    learnTarget_ = target;
    learnBaselineValid_ = false;
    emit learningChanged();
}

void JoystickManager::cancelLearn() {
    if (!learning()) return;
    learnSlot_ = -1;
    emit learningChanged();
}

void JoystickManager::applyLearned(const joy::Input& input) {
    const int slotIndex = learnSlot_;
    if (slotIndex < 0) return;
    joy::JoystickProfile& p = activeProfile(slotIndex);
    const auto idx = static_cast<std::size_t>(std::clamp(learnTarget_.index, 0, std::max(joy::kAxisCount, joy::kButtonCount) - 1));
    using Kind = LearnTarget::Kind;
    switch (learnTarget_.kind) {
        case Kind::AxisFirst: if (idx < p.axes.size()) p.axes[idx].first = input; break;
        case Kind::AxisSecond: if (idx < p.axes.size()) p.axes[idx].second = input; break;
        case Kind::Button: if (idx < p.buttons.size()) p.buttons[idx] = input; break;
        case Kind::PovUp: p.pov.up = input; break;
        case Kind::PovRight: p.pov.right = input; break;
        case Kind::PovDown: p.pov.down = input; break;
        case Kind::PovLeft: p.pov.left = input; break;
        case Kind::PovPad: p.pov.pad = input; break;
    }
    learnSlot_ = -1;
    save();
    emit learningChanged();
    emit slotChanged(slotIndex);
}

void JoystickManager::pollLearning(int slotIndex, const joy::RawInput& raw) {
    if (!learnBaselineValid_) {
        learnBaseline_ = raw;
        learnBaselineValid_ = true;
        return;
    }
    (void)slotIndex;
    using Kind = LearnTarget::Kind;
    if (learnTarget_.kind == Kind::AxisFirst) {
        for (std::size_t i = 0; i < raw.axes.size() && i < learnBaseline_.axes.size(); ++i)
            if (std::fabs(raw.axes[i] - learnBaseline_.axes[i]) > 0.6f) {
                applyLearned(joy::Input::padAxis(static_cast<int>(i)));
                return;
            }
    } else if (learnTarget_.kind == Kind::Button) {
        for (std::size_t i = 0; i < raw.buttons.size() && i < learnBaseline_.buttons.size(); ++i)
            if (raw.buttons[i] != 0 && learnBaseline_.buttons[i] == 0) {
                applyLearned(joy::Input::padButton(static_cast<int>(i)));
                return;
            }
    } else if (learnTarget_.kind == Kind::PovPad) {
        if (!raw.povs.empty() && raw.povs[0] >= 0 && (learnBaseline_.povs.empty() || learnBaseline_.povs[0] < 0)) {
            applyLearned(joy::Input::padPov(0));
            return;
        }
    }
    // Keep the baseline tracking releases so a held input at start does not block detection forever.
    for (std::size_t i = 0; i < raw.buttons.size() && i < learnBaseline_.buttons.size(); ++i)
        if (raw.buttons[i] == 0) learnBaseline_.buttons[i] = 0;
}

void JoystickManager::onSourcesChanged(bool initial) {
    const auto pads = gamepads();
    for (const auto& info : pads) {
        if (std::find(knownPads_.begin(), knownPads_.end(), info.id) != knownPads_.end()) continue;
        knownPads_.push_back(info.id);
        const bool referenced = std::any_of(slots_.begin(), slots_.end(), [&](const SlotConfig& s) { return s.sourceId == info.id; });
        if (referenced || (initial && !firstRun_)) continue;
        for (int i = 0; i < joy::kSlotCount; ++i) {
            if (!slot(i).sourceId.isEmpty()) continue;
            setSourceId(i, info.id);
            emit gamepadConnected(info, i);
            break;
        }
    }
    // Forget unplugged pads so replugging the same device is announced again.
    knownPads_.erase(std::remove_if(knownPads_.begin(), knownPads_.end(),
                                    [&](const QString& id) {
                                        return std::none_of(pads.begin(), pads.end(), [&](const auto& p) { return p.id == id; });
                                    }),
                     knownPads_.end());
    emit sourcesChanged();
}

}  // namespace wpiemu::app::runview
