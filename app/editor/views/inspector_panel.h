#pragma once

// Right-hand inspector. Everything shown is generated from the selection: property groups rendered
// by PropertyFormWidget (chassis, wheels, mechanism templates and plugin panels all look the same),
// device slot rows with drag-and-drop mapping, shape lists and quick actions. Empty state = setup
// checklist for the robot.

#include <QLabel>
#include <QLineEdit>
#include <QScrollArea>
#include <QWidget>

#include <memory>
#include <vector>

#include "viewmodel/editor_controller.h"
#include "views/icon_factory.h"
#include "views/property_form.h"
#include "views/device_row.h"
#include "views/section_card.h"

namespace wpiemu::app::editor {

class ActivationCard;

class InspectorPanel : public QWidget {
    Q_OBJECT
public:
    InspectorPanel(EditorController& controller, const IconFactory& icons, QWidget* parent = nullptr);

    // ---- introspection for tests ----
    PropertyFormWidget* form(const QString& groupId) const;
    std::vector<DeviceRowWidget*> slotRows() const { return slotRows_; }
    QString titleText() const {
        return nameEdit_ != nullptr ? nameEdit_->text() : title_ != nullptr ? title_->text() : QString();
    }
    QLineEdit* nameEdit() const { return nameEdit_; }
    QLabel* robotToCameraLabel() const { return robotToCameraLabel_; }
    int rebuildCount() const { return rebuilds_; }
    ActivationCard* activationCard() const { return activationCard_; }

private:
    void rebuild();
    void refresh(bool authoritative = false);
    QString signature(const std::vector<PropertyGroup>& groups) const;
    void clearContent();
    void focusField(const QString& key);

    QWidget* addSection(const QString& title, QWidget* body, bool collapsed = false, SectionRole role = SectionRole::Info);
    PropertyFormWidget* makeForm(const PropertyGroup& group, QWidget* parent);
    DeviceRowWidget* makeRow(const SlotInfo& info, QWidget* parent);
    void commitName(const SelectionRef& target, const QString& text);
    QString editableName(const SelectionRef& target) const;
    void addPartCards(const SelectionRef& target, const std::vector<PropertyGroup>& groups);
    void addZoneCards(const SelectionRef& target, const std::vector<PropertyGroup>& groups);
    void addCameraCards(const SelectionRef& target, const std::vector<PropertyGroup>& groups);
    void updateRobotToCamera(const sdk::CameraConfig& cam, const sdk::RobotConfig& c, const EditorCatalog& cat);
    void addHeader(const SelectionRef& target, const std::vector<PropertyGroup>& groups);
    void addShapes(const std::string& owner);
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
    QLineEdit* nameEdit_ = nullptr;
    QLabel* robotToCameraLabel_ = nullptr;
    QMap<QString, QPair<int, QString>> issues_;
    ActivationCard* activationCard_ = nullptr;
    std::vector<PropertyFormWidget*> forms_;
    std::vector<DeviceRowWidget*> slotRows_;
    QString signature_;
    SelectionRef shown_;
    int rebuilds_ = 0;
    UnitSystem shownUnits_ = UnitSystem::Imperial;
    QString pendingFocus_;
};

}  // namespace wpiemu::app::editor
