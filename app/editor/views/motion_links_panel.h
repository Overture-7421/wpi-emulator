#pragma once

#include <QWidget>

#include "viewmodel/editor_controller.h"

namespace wpiemu::app::editor {

class MotionLinksPanel final : public QWidget {
public:
    explicit MotionLinksPanel(EditorController& controller, QWidget* parent = nullptr);
};

}  // namespace wpiemu::app::editor
