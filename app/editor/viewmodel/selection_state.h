#pragma once

// View-state shared by the tree, viewport, inspector and validation list: the current selection and
// the user's display preferences. Plain QObjects so every view observes the same instance.

#include <QObject>

#include "model/selection.h"
#include "model/units.h"

namespace wpiemu::app::editor {

class SelectionState : public QObject {
    Q_OBJECT
public:
    explicit SelectionState(QObject* parent = nullptr) : QObject(parent) {}
    const SelectionRef& current() const { return current_; }
    // Returns true when the selection changed. `fieldKey` asks the inspector to focus a field.
    bool set(const SelectionRef& ref, const QString& fieldKey = {});

Q_SIGNALS:
    void changed(const wpiemu::app::editor::SelectionRef& ref);
    void fieldFocusRequested(const QString& key);

private:
    SelectionRef current_;
};

class EditorSettings : public QObject {
    Q_OBJECT
public:
    explicit EditorSettings(QObject* parent = nullptr) : QObject(parent) {}

    UnitSystem units() const { return units_; }
    void setUnits(UnitSystem u);

    bool snapEnabled() const { return snapEnabled_; }
    void setSnapEnabled(bool on);
    // The AprilTags of the selected layout in the viewport. They are always shown while a camera or the vision settings are selected.
    bool showTags() const { return showTags_; }
    void setShowTags(bool on);
    // Snap increments in SI (metres, degrees) derived from the unit system: 1 in or 1 cm; 15 degrees.
    double moveStepM() const { return units_ == UnitSystem::Imperial ? 0.0254 : 0.01; }
    double rotateStepDeg() const { return rotateStepDeg_; }
    void setRotateStepDeg(double deg);

Q_SIGNALS:
    void changed();

private:
    UnitSystem units_ = UnitSystem::Imperial;
    bool snapEnabled_ = true;
    bool showTags_ = false;
    double rotateStepDeg_ = 15.0;
};

}  // namespace wpiemu::app::editor

Q_DECLARE_METATYPE(wpiemu::app::editor::SelectionRef)
