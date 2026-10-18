#pragma once

// The "Cameras" section of the detected-devices panel (ADR 0017): the PhotonVision / Limelight cameras the robot code uses that the
// project does not list, each with an "Add camera" button, and a hint when a configured camera of the same vendor goes unused (the name
// in the project may differ from the name in the code). Hidden while there is nothing to show.

#include <QFrame>
#include <QLabel>
#include <QString>
#include <QVBoxLayout>

#include "model/detected_cameras.h"
#include "viewmodel/editor_controller.h"
#include "views/icon_factory.h"

namespace wpiemu::app::editor {

class DetectedCamerasSection : public QFrame {
    Q_OBJECT
public:
    DetectedCamerasSection(EditorController& controller, const IconFactory& icons, QWidget* parent = nullptr);

    int rowCount() const { return rowCount_; }

private:
    void refresh();
    QWidget* makeRow(const UnconfiguredCamera& row);

    EditorController& controller_;
    const IconFactory& icons_;
    QLabel* title_;
    QVBoxLayout* rows_;
    QString shown_;   // what the rows were built from: they are rebuilt only when it changes
    int rowCount_ = 0;
};

}  // namespace wpiemu::app::editor
