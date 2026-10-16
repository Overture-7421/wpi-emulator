#include "viewmodel/selection_state.h"

namespace wpiemu::app::editor {

bool SelectionState::set(const SelectionRef& ref, const QString& fieldKey) {
    const bool changedNow = !(ref == current_);
    current_ = ref;
    if (changedNow) Q_EMIT changed(current_);
    if (!fieldKey.isEmpty()) Q_EMIT fieldFocusRequested(fieldKey);
    return changedNow;
}

void EditorSettings::setUnits(UnitSystem u) {
    if (u == units_) return;
    units_ = u;
    Q_EMIT changed();
}

void EditorSettings::setSnapEnabled(bool on) {
    if (on == snapEnabled_) return;
    snapEnabled_ = on;
    Q_EMIT changed();
}

void EditorSettings::setRotateStepDeg(double deg) {
    if (deg == rotateStepDeg_ || deg <= 0.0) return;
    rotateStepDeg_ = deg;
    Q_EMIT changed();
}

}  // namespace wpiemu::app::editor
