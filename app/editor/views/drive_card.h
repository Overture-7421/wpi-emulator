#pragma once

// Body of the inspector's Drive card for a wheel or a mechanism. For each motion axis: what actuates it
// (its own motors, or another part through a ratio, ADR 0012) and, for own motors, the motor model /
// count / gearing form plus one compact row per controller (device-agnostic: Motor 1..N) (Reversed, unmap).
// Controllers are never picked here, they are dragged onto the project tree.

#include <QWidget>

#include <functional>

#include "model/selection.h"
#include "viewmodel/editor_controller.h"
#include "views/device_row.h"
#include "views/property_form.h"

namespace wpiemu::app::editor {

class DriveCard : public QWidget {
    Q_OBJECT
public:
    // The inspector registers the forms / rows it must keep in sync (refresh without rebuild).
    struct Registrar {
        std::function<void(PropertyFormWidget*)> form;
        std::function<void(DeviceRowWidget*)> row;
    };

    // Object names for tests, per axis address string A: "drive.own:A" / "drive.linked:A" (QToolButton),
    // "drivenBy.source:A" (QComboBox), "drivenBy.ratio:A" (QDoubleSpinBox), "drivenBy.units:A", "drivenBy.error:A" (QLabel).
    DriveCard(EditorController& controller, const IconFactory& icons, const SelectionRef& owner, Registrar registrar = {},
              QWidget* parent = nullptr);

    // True when there is something to show (the owner has motion axes).
    static bool applicable(EditorController& controller, const SelectionRef& owner);
};

}  // namespace wpiemu::app::editor
