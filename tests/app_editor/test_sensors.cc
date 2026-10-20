#include <algorithm>

#include "model/editor_document.h"
#include "model/property_groups.h"
#include "model/slot_status.h"
#include "test_support.h"
#include "viewmodel/editor_controller.h"
#include "viewmodel/project_tree_model.h"

using namespace wpiemu::app::editor;
using namespace wpiemu::app::editor::testing;
namespace sdk = wpiemu::sdk;

TEST_CASE("sensor document: add, duplicate and remove are undoable and ids stay unique") {
    EditorDocument doc;
    doc.setCatalog(catalog());
    doc.load(robot());
    const std::size_t base = doc.config().sensors.size();

    const QString id = doc.addSensor(sdk::SensorKind::BeamBreak);
    REQUIRE_FALSE(id.isEmpty());
    CHECK(doc.config().sensors.size() == base + 1);
    CHECK(doc.config().findSensor(id.toStdString())->kind == sdk::SensorKind::BeamBreak);
    doc.undoStack()->undo();
    CHECK(doc.config().sensors.size() == base);
    doc.undoStack()->redo();

    const QString copy = doc.duplicateSensor(id);
    REQUIRE_FALSE(copy.isEmpty());
    CHECK(copy != id);
    CHECK(doc.duplicateSensor("ghost").isEmpty());

    REQUIRE(doc.removeSensor(copy));
    CHECK(doc.config().findSensor(copy.toStdString()) == nullptr);
    doc.undoStack()->undo();
    CHECK(doc.config().findSensor(copy.toStdString()) != nullptr);
}

TEST_CASE("sensor controller: adding selects the sensor and delete acts on it") {
    FakeEditorBackend backend;
    EditorController c(backend);
    c.setAutosaveDelayMs(0);
    const QString id = c.addSensor(sdk::SensorKind::Distance);
    REQUIRE_FALSE(id.isEmpty());
    CHECK(c.selection().current() == SelectionRef::sensor(id.toStdString()));
    CHECK(c.selectionExists(SelectionRef::sensor(id.toStdString())));
    REQUIRE(c.deleteSelection());
    CHECK_FALSE(c.selectionExists(SelectionRef::sensor(id.toStdString())));
}

TEST_CASE("sensor property groups: the source lists game values and flags an unavailable one") {
    const auto cat = catalog();
    auto c = robot();
    const SelectionRef sel = SelectionRef::sensor("hopper_sensor");

    auto groups = propertyGroupsFor(c, cat, sel);
    std::vector<std::string> ids;
    for (const auto& g : groups) ids.push_back(g.id);
    CHECK(ids == std::vector<std::string>{"sensor.general", "sensor.source", "sensor.props"});
    const auto* source = groups[1].schema.find("source");
    REQUIRE(source != nullptr);
    CHECK(source->options.size() == cat.gameValues.size() + 1);

    CHECK(applyProperty(c, cat, sel, "sensor.source", "source", sdk::Value("game_2026.gone")));
    groups = propertyGroupsFor(c, cat, sel);
    const auto& options = groups[1].schema.find("source")->options;
    CHECK(options.size() == cat.gameValues.size() + 2);
    CHECK(options.back().value == "game_2026.gone");
    CHECK(options.back().label.find("(unavailable)") != std::string::npos);
}

TEST_CASE("sensor tree: sensors are listed in the sensors group with a slot row") {
    EditorDocument doc;
    doc.setCatalog(catalog());
    doc.load(robot());
    DeviceDirectory devices;
    ProjectTreeModel tree(doc, devices);

    const QModelIndex group = tree.indexForKey("sensors");
    REQUIRE(group.isValid());
    CHECK(tree.data(group).toString() == "Sensors");
    const QModelIndex node = tree.indexFor(SelectionRef::sensor("hopper_sensor"));
    REQUIRE(node.isValid());
    CHECK(tree.parent(node) == group);
    CHECK(tree.data(node, ProjectTreeModel::KindRole).toInt() == static_cast<int>(NodeKind::Sensor));
    CHECK(tree.rowCount(node) == 1);

    tree.setIssues({{sdk::Severity::Warning, "sensors[0].source", "x", "sensor_source_unknown"}});
    CHECK(tree.data(node, ProjectTreeModel::WarningRole).toBool());
}

TEST_CASE("sensor validation: issue paths are located to their sensor and field") {
    const auto cat = catalog();
    const auto c = robot();
    const auto at = [&](const std::string& path) { return locateIssue(c, cat, {sdk::Severity::Error, path, "x", {}}); };
    CHECK(at("sensors[0].source").selection == SelectionRef::sensor("hopper_sensor"));
    CHECK(at("sensors[0].source").fieldKey == "source");
    CHECK(at("sensors[1].properties.threshold").fieldKey == "threshold");
}
