#pragma once

// Right-hand inspector. Everything shown is generated from the selection: property groups rendered
// by PropertyFormWidget (chassis, wheels, mechanism templates and plugin panels all look the same),
// device slot rows with drag-and-drop mapping, shape lists and quick actions. Empty state = setup
// checklist for the robot.

#include <QLabel>
#include <QScrollArea>
#include <QWidget>

#include <memory>
#include <vector>

#include "viewmodel/editor_controller.h"
#include "views/icon_factory.h"
#include "views/property_form.h"
#include "views/slot_row.h"

namespace wpiemu::app::editor {

class InspectorPanel : public QWidget {
    Q_OBJECT
public:
    InspectorPanel(EditorController& controller, const IconFactory& icons, QWidget* parent = nullptr);

    // ---- introspection for tests ----
    PropertyFormWidget* form(const QString& groupId) const;
    std::vector<SlotRowWidget*> slotRows() const { return slotRows_; }
    QString titleText() const { return title_ != nullptr ? title_->text() : QString(); }
    int rebuildCount() const { return rebuilds_; }

private:
    void rebuild();
    void refresh();
    QString signature(const std::vector<PropertyGroup>& groups) const;
    void clearContent();
    void focusField(const QString& key);

    QWidget* addSection(const QString& title, QWidget* body, bool collapsed = false);
    void addHeader(const SelectionRef& target, const std::vector<PropertyGroup>& groups);
    void addChecklist();
    void addSlots(const SelectionRef& target);
    void addShapes(const std::string& owner);
    void addWheelList();
    void addChildren(const std::string& mechanismId);
    void addDeviceCard(const SelectionRef& sel);
    QWidget* linkButton(const QString& text, const std::function<void()>& action);
    QMap<QString, QPair<int, QString>> issuesFor(const SelectionRef& target) const;

    EditorController& controller_;
    const IconFactory& icons_;
    QScrollArea* scroll_;
    QWidget* content_ = nullptr;
    QVBoxLayout* layout_ = nullptr;
    QLabel* title_ = nullptr;
    std::vector<PropertyFormWidget*> forms_;
    std::vector<SlotRowWidget*> slotRows_;
    QString signature_;
    SelectionRef shown_;
    int rebuilds_ = 0;
    UnitSystem shownUnits_ = UnitSystem::Imperial;
    QString pendingFocus_;
};

}  // namespace wpiemu::app::editor
