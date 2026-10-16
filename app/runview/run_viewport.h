#pragma once

#include <QImage>
#include <QWidget>

#include "hud_model.h"
#include "mechanism_overlay_model.h"
#include "run_control.h"
#include "scene_model.h"

class QLabel;
class QProgressBar;
class QPushButton;
class QQuickWidget;

namespace wpiemu::app::runview {

// Translucent card over the viewport for "not running" / "starting" / error states.
class StateOverlay : public QWidget {
    Q_OBJECT
public:
    explicit StateOverlay(QWidget* parent = nullptr);
    void setPresentation(const StatusPresentation& presentation);

    QLabel* titleLabel() const { return title_; }
    QLabel* detailLabel() const { return detail_; }
    QPushButton* startButton() const { return start_; }
    QPushButton* restartButton() const { return restart_; }
    QPushButton* consoleButton() const { return console_; }

signals:
    void startRequested();
    void restartRequested();
    void showConsoleRequested();

protected:
    void paintEvent(QPaintEvent*) override;

private:
    QLabel* icon_;
    QLabel* title_;
    QLabel* detail_;
    QProgressBar* busy_;
    QPushButton* start_;
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

protected:
    void resizeEvent(QResizeEvent*) override;

private:
    QQuickWidget* quick_;
    StateOverlay* overlay_;
    QLabel* errorLabel_;
    QString error_;
};

}  // namespace wpiemu::app::runview
