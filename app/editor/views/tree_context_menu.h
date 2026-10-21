#pragma once

// The context menu of a project-tree row: the edits that make sense for that kind of node (duplicate, delete, attach to,
// add parts, zones, cameras, sensors...). Every entry runs through the controller, so it is one undo step.

class QPoint;
class QWidget;

namespace wpiemu::app::editor {

class EditorController;
class IconFactory;
struct SelectionRef;

// Builds the menu for `ref` as a child of `owner` and shows it at `globalPos`; rows whose entries act on the selection select
// `ref` first, and a row without entries shows nothing. Returns when the menu is dismissed.
void showTreeContextMenu(QWidget* owner, EditorController& controller, const IconFactory& icons, const SelectionRef& ref,
                         const QPoint& globalPos);

}  // namespace wpiemu::app::editor
