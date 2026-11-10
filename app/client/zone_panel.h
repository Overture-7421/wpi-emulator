#pragma once

// The zone controls of the drive screen: one button per zone of the player's robot (hold or toggle it active) and, for the selected
// zone, sliders that move it in the chassis frame. The joystick does the same with buttons A, B, X, Y ... and the d-pad (see
// input_mapping.h); this panel shows what is on and where each zone is.

#include <QFrame>

#include <vector>

#include "client/client_controller.h"

class QLabel;
class QPushButton;
class QSlider;
class QVBoxLayout;

namespace wpiemu::app::client {

class ZonePanel : public QFrame {
    Q_OBJECT
public:
    explicit ZonePanel(ClientController& controller, QWidget* parent = nullptr);

    // Rebuilds the buttons from the player's robot (call when the scene changed).
    void rebuild();
    // Brings the buttons and sliders in line with the latest picture (zones on, poses moved by keys).
    void refresh();

    std::size_t zoneCount() const { return buttons_.size(); }
    QPushButton* zoneButton(std::size_t zone) const { return zone < buttons_.size() ? buttons_[zone] : nullptr; }
    QSlider* slider(int axis) const { return axis >= 0 && axis < 4 ? sliders_[axis] : nullptr; }   // 0 x, 1 y, 2 z, 3 yaw
    QPushButton* resetButton() const { return reset_; }
    QLabel* hint() const { return hint_; }

signals:
    void zoneSelected(std::size_t zone);

private:
    void selectZone(std::size_t zone);
    void onSlider();
    void syncSliders();

    ClientController& controller_;
    QVBoxLayout* list_ = nullptr;
    QWidget* poseBox_ = nullptr;
    QLabel* poseTitle_ = nullptr;
    QLabel* hint_ = nullptr;
    std::vector<QPushButton*> buttons_;
    std::vector<QPushButton*> selectors_;
    QSlider* sliders_[4] = {};
    QLabel* sliderValues_[4] = {};
    QPushButton* reset_ = nullptr;
    bool syncing_ = false;
};

}  // namespace wpiemu::app::client
