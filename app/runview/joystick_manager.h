#pragma once

#include <QObject>
#include <QSettings>

#include <array>
#include <memory>
#include <vector>

#include "joystick_source.h"

namespace wpiemu::app::runview {

// Owns the six DS joystick slots: which device feeds each slot, the per-slot binding profiles
// (persisted through the injected QSettings), keyboard state, and the "learn the next input" flow of
// the binding editor. No singletons: the provider and settings are borrowed from the composition root.
class JoystickManager : public QObject {
    Q_OBJECT
public:
    static constexpr const char* kKeyboardId = "keyboard";

    struct SlotConfig {
        QString sourceId;   // "" = none, "keyboard", or a gamepad id
        joy::JoystickProfile keyboard = joy::JoystickProfile::defaultKeyboard();
        joy::JoystickProfile gamepad = joy::JoystickProfile::defaultGamepad();
    };

    struct LearnTarget {
        enum class Kind { AxisFirst, AxisSecond, Button, PovUp, PovRight, PovDown, PovLeft, PovPad };
        Kind kind = Kind::Button;
        int index = 0;   // axis or button index
        bool operator==(const LearnTarget&) const = default;
    };

    // `provider` and `settings` may be null (no gamepads / nothing persisted); neither is owned.
    JoystickManager(IJoystickProvider* provider, QSettings* settings, QObject* parent = nullptr);

    const SlotConfig& slot(int index) const { return slots_[static_cast<std::size_t>(index)]; }
    bool usesKeyboard(int slotIndex) const { return slot(slotIndex).sourceId == QLatin1String(kKeyboardId); }
    void setSourceId(int slotIndex, const QString& id);
    const joy::JoystickProfile& profile(int slotIndex) const;   // profile of the slot's active device kind
    void setProfile(int slotIndex, const joy::JoystickProfile& profile);
    void resetProfile(int slotIndex);

    std::vector<JoystickSourceInfo> availableSources() const;   // keyboard first, then connected gamepads
    std::vector<JoystickSourceInfo> gamepads() const;
    bool slotConnected(int slotIndex) const;
    QString slotDeviceName(int slotIndex) const;
    bool gamepadBackendAvailable() const { return provider_ != nullptr; }
    QString gamepadBackendName() const { return provider_ != nullptr ? provider_->backendName() : QString(); }

    // Polls every slot's device and evaluates its profile. Cheap; call at ~50 Hz.
    const std::array<sdk::proto::JoystickState, joy::kSlotCount>& sample();
    const std::array<sdk::proto::JoystickState, joy::kSlotCount>& lastStates() const { return states_; }
    const joy::RawInput& lastRaw(int slotIndex) const { return raw_[static_cast<std::size_t>(slotIndex)]; }

    // Keyboard input from the run view. Returns true when the key was consumed by "learn" mode.
    bool handleKey(int qtKey, bool down);
    void releaseAllKeys();

    void beginLearn(int slotIndex, LearnTarget target);
    void cancelLearn();
    bool learning() const { return learnSlot_ >= 0; }
    int learnSlot() const { return learnSlot_; }
    LearnTarget learnTarget() const { return learnTarget_; }

    void load();
    void save() const;

signals:
    void slotChanged(int slotIndex);
    void sourcesChanged();
    void learningChanged();
    void gamepadConnected(const wpiemu::app::runview::JoystickSourceInfo& info, int assignedSlot);

private:
    std::shared_ptr<IJoystickSource> sourceFor(const QString& id) const;
    joy::JoystickProfile& activeProfile(int slotIndex);
    void onSourcesChanged(bool initial);
    void applyLearned(const joy::Input& input);
    void pollLearning(int slotIndex, const joy::RawInput& raw);

    IJoystickProvider* provider_;
    QSettings* settings_;
    KeyboardJoystickSource keyboard_;
    std::array<SlotConfig, joy::kSlotCount> slots_;
    std::array<sdk::proto::JoystickState, joy::kSlotCount> states_;
    std::array<joy::RawInput, joy::kSlotCount> raw_;
    std::vector<QString> knownPads_;
    bool firstRun_ = true;

    int learnSlot_ = -1;
    LearnTarget learnTarget_;
    joy::RawInput learnBaseline_;
    bool learnBaselineValid_ = false;
};

}  // namespace wpiemu::app::runview

Q_DECLARE_METATYPE(wpiemu::app::runview::JoystickSourceInfo)
