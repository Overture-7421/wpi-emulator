#pragma once

// The viewport column: transform / snap / view toolbar, numeric transform entry, the Qt Quick 3D
// scene hosted in a QQuickWidget and a status line. Falls back to an explanatory label when Qt Quick
// 3D cannot start (no GPU, remote desktop) so the rest of the editor stays usable.

#include <QDoubleSpinBox>
#include <QImage>
#include <QLabel>
#include <QQuickWidget>
#include <QToolButton>
#include <QWidget>

#include <array>

#include "viewmodel/editor_controller.h"
#include "viewmodel/viewport_controller.h"

namespace wpiemu::app::editor {

class ViewportWidget : public QWidget {
    Q_OBJECT
public:
    ViewportWidget(EditorController& editor, ViewportController& viewport, QWidget* parent = nullptr);

    QQuickWidget* quick() const { return quick_; }
    // True once the QML scene loaded without errors.
    bool sceneReady() const;
    QString failureReason() const { return failure_; }
    QImage grabScene();

protected:
    void changeEvent(QEvent* event) override;

private:
    void buildToolbar(QLayout* layout);
    void syncToolbar();
    void applyTheme();
    void onStatusChanged();

    EditorController& editor_;
    ViewportController& viewport_;
    QQuickWidget* quick_;
    QLabel* fallback_;
    QLabel* info_;
    QToolButton* modeButtons_[3] = {};
    QToolButton* snapButton_ = nullptr;
    QLabel* fieldCaption_ = nullptr;
    std::array<QDoubleSpinBox*, 3> fields_{};
    QString failure_;
};

}  // namespace wpiemu::app::editor
