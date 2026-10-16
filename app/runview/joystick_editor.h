#pragma once

#include <QWidget>

#include <array>

#include "joystick_manager.h"

class QCheckBox;
class QComboBox;
class QGridLayout;
class QLabel;
class QPushButton;
class QTimer;

namespace wpiemu::app::runview {

// Small live indicator: a centred bar for axes (-1..1) or an on/off LED for buttons.
class ValueBar : public QWidget {
    Q_OBJECT
public:
    explicit ValueBar(bool led, QWidget* parent = nullptr);
    void setValue(float value);
    float value() const { return value_; }
    QSize sizeHint() const override;

protected:
    void paintEvent(QPaintEvent*) override;

private:
    bool led_;
    float value_ = 0.f;
};

// Axis / button / POV binding editor for the six DS joystick slots with live preview and "press the
// input you want" learning. All state lives in the JoystickManager.
class JoystickEditor : public QWidget {
    Q_OBJECT
public:
    explicit JoystickEditor(JoystickManager& manager, QWidget* parent = nullptr);

    int currentSlot() const;
    void setCurrentSlot(int slot);
    // Test hooks: the widgets the user would click.
    QPushButton* axisFirstButton(int axis) const { return axisFirst_[static_cast<std::size_t>(axis)]; }
    QPushButton* buttonBindButton(int button) const { return buttonBind_[static_cast<std::size_t>(button)]; }
    QComboBox* deviceCombo() const { return device_; }
    QLabel* notice() const { return notice_; }
    void refreshPreview();   // pulls live values from the manager (also driven by an internal timer)

protected:
    void showEvent(QShowEvent*) override;
    void hideEvent(QHideEvent*) override;

private:
    void rebuildDevices();
    void refreshBindings();
    void toggleLearn(JoystickManager::LearnTarget target, QPushButton* source);
    QPushButton* makeBindButton(JoystickManager::LearnTarget target);

    JoystickManager& manager_;
    QComboBox* slot_;
    QComboBox* device_;
    QLabel* notice_;
    QLabel* hint_;
    QTimer* previewTimer_;

    struct BindRef {
        QPushButton* button;
        JoystickManager::LearnTarget target;
    };
    std::vector<BindRef> binds_;
    std::array<QPushButton*, joy::kAxisCount> axisFirst_{};
    std::array<QPushButton*, joy::kAxisCount> axisSecond_{};
    std::array<QCheckBox*, joy::kAxisCount> axisInvert_{};
    std::array<ValueBar*, joy::kAxisCount> axisBar_{};
    std::array<QPushButton*, joy::kButtonCount> buttonBind_{};
    std::array<ValueBar*, joy::kButtonCount> buttonLed_{};
    QPushButton* povUp_ = nullptr;
    QPushButton* povRight_ = nullptr;
    QPushButton* povDown_ = nullptr;
    QPushButton* povLeft_ = nullptr;
    QPushButton* povPad_ = nullptr;
    QLabel* povValue_ = nullptr;
    bool updating_ = false;
};

}  // namespace wpiemu::app::runview
