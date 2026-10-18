#pragma once

#include <QIcon>
#include <QImage>
#include <functional>
#include <QWidget>

#include "hud_model.h"
#include "mechanism_overlay_model.h"
#include "run_control.h"
#include "scene_model.h"

class QLabel;
class QAction;
class QComboBox;
class QMenu;
class QHBoxLayout;
class QProgressBar;
class QToolButton;
class QPushButton;
class QQuickWidget;

namespace wpiemu::app::runview {

QIcon reloadIcon();
QIcon chevronIcon();

// Translucent card over the viewport for "not running" / "starting" / error states.
class StateOverlay : public QWidget {
    Q_OBJECT
public:
    explicit StateOverlay(QWidget* parent = nullptr);
    void setPresentation(const StatusPresentation& presentation);

    QLabel* titleLabel() const { return title_; }
    QLabel* detailLabel() const { return detail_; }
    QPushButton* restartButton() const { return restart_; }

signals:
    void restartRequested();
    void showConsoleRequested();

protected:
    void paintEvent(QPaintEvent*) override;

private:
    QLabel* icon_;
    QLabel* title_;
    QLabel* detail_;
    QProgressBar* busy_;
    QPushButton* restart_;
    QPushButton* console_;
};

// Qt Quick 3D scene hosted in a QQuickWidget plus the state overlay.
class RunViewport : public QWidget {
    Q_OBJECT
public:
    RunViewport(SceneModel& scene, HudModel& hud, MechanismOverlayModel& mechanisms, ViewportTheme& theme,
                ViewportOptions& options, QWidget* parent = nullptr);

    StateOverlay* overlay() const { return overlay_; }
    bool sceneLoaded() const;
    QString sceneError() const { return error_; }
    QImage grabScene();
    QToolButton* homeButton() const { return home_; }
    // Adds a control to the button cluster at the bottom-right of the view (left of the home button).
    void addControl(QWidget* w);
    // Restart button joined with a chevron that opens `menu` (restart options).
    void addRestartControl(QAction* restart, QMenu* menu);
    // Moves the speed combo into the cluster, styled with centred text.
    void addSpeedControl(QComboBox* combo);
    // Match phase and time left at the top-center of the view; hidden when `text` is empty.
    void setMatchClock(const QString& text);
    QLabel* matchClockLabel() const { return matchClock_; }

signals:
    void homeRequested();

protected:
    void resizeEvent(QResizeEvent*) override;
    bool eventFilter(QObject* obj, QEvent* e) override;

private:
    void placeControls();

    QComboBox* speedCombo_ = nullptr;
    std::function<void()> speedSync_;
    QQuickWidget* quick_;
    StateOverlay* overlay_;
    QLabel* errorLabel_;
    QToolButton* home_;
    QWidget* controls_;
    QLabel* matchClock_;
    QHBoxLayout* controlsLayout_;
    QString error_;
};

}  // namespace wpiemu::app::runview
