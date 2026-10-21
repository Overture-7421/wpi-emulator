#include "model/config_ops.h"
#include "model/editor_document.h"
#include "model/property_groups.h"
#include "model/slot_status.h"
#include "test_support.h"
#include "viewmodel/editor_controller.h"
#include "viewmodel/project_tree_model.h"

using namespace wpiemu::app::editor;
using namespace wpiemu::app::editor::testing;
namespace sdk = wpiemu::sdk;

TEST_CASE("start position ops: add, duplicate and remove keep ids unique") {
    auto c = robot();
    const std::string a = ops::addStartPosition(c);
    const std::string b = ops::addStartPosition(c);
    CHECK(a != b);
    const std::string copy = ops::duplicateStartPosition(c, a);
    REQUIRE_FALSE(copy.empty());
    CHECK(copy != a);
    CHECK(ops::duplicateStartPosition(c, "ghost").empty());
    CHECK(ops::removeStartPosition(c, copy));
    CHECK_FALSE(ops::removeStartPosition(c, copy));
}

TEST_CASE("start position copy tool: the other alliance gets the flipped pose and a swapped label") {
    auto c = robot();
    const sdk::FieldFrame frame = catalog().fieldFrame;
    const std::string id = ops::addStartPosition(c);
    sdk::StartPosition* p = ops::findStartPosition(c, id);
    p->label = "Blue right";
    p->alliance = sdk::Alliance::Blue;
    p->pose = {4.51f, 0.527f, 0.f};

    const std::string red = ops::copyStartPositionToOtherAlliance(c, id, frame);
    REQUIRE_FALSE(red.empty());
    const sdk::StartPosition* copy = c.findStartPosition(red);
    CHECK(copy->alliance == sdk::Alliance::Red);
    CHECK(copy->label == "Red right");
    CHECK(copy->pose.xM == doctest::Approx(frame.lengthM - 4.51f));
    CHECK(copy->pose.yM == doctest::Approx(frame.widthM - 0.527f));

    // and back: an unrecognised label gets a suffix, no symmetry means no tool
    ops::findStartPosition(c, id)->label = "Left lane";
    const std::string back = ops::copyStartPositionToOtherAlliance(c, red, frame);
    CHECK(c.findStartPosition(back)->alliance == sdk::Alliance::Blue);
    CHECK(c.findStartPosition(back)->label == "Blue right");
    CHECK(ops::copyStartPositionToOtherAlliance(c, id, sdk::FieldFrame{}).empty());
    const std::string suffixed = ops::copyStartPositionToOtherAlliance(c, id, frame);
    CHECK(c.findStartPosition(suffixed)->label == "Left lane (Red)");
}

TEST_CASE("start position document and controller: add selects, the copy tool selects the copy and everything is undoable") {
    FakeEditorBackend backend;
    EditorController c(backend);
    c.setAutosaveDelayMs(0);
    const QString id = c.addStartPosition();
    REQUIRE_FALSE(id.isEmpty());
    CHECK(c.selection().current() == SelectionRef::startPosition(id.toStdString()));
    CHECK(c.selectionExists(SelectionRef::startPosition(id.toStdString())));

    REQUIRE(c.copySelectedStartPositionToOtherAlliance());
    const SelectionRef copy = c.selection().current();
    REQUIRE(copy.kind == SelKind::StartPosition);
    CHECK(copy.owner != id.toStdString());
    CHECK(c.document().config().findStartPosition(copy.owner)->alliance == sdk::Alliance::Red);
    c.document().undoStack()->undo();
    CHECK(c.document().config().findStartPosition(copy.owner) == nullptr);

    c.selection().set(SelectionRef::startPosition(id.toStdString()));
    REQUIRE(c.deleteSelection());
    CHECK_FALSE(c.selectionExists(SelectionRef::startPosition(id.toStdString())));
}

TEST_CASE("start position property groups edit the label, alliance and pose, and the tree lists them") {
    const auto cat = catalog();
    auto c = robot();
    const std::string id = ops::addStartPosition(c);
    const SelectionRef sel = SelectionRef::startPosition(id);

    const auto groups = propertyGroupsFor(c, cat, sel);
    REQUIRE(groups.size() == 2);
    CHECK(groups[0].id == "startpos.general");
    CHECK(groups[1].id == "startpos.placement");
    CHECK(applyProperty(c, cat, sel, "startpos.general", "alliance", sdk::Value("red")));
    CHECK(applyProperty(c, cat, sel, "startpos.placement", "xM", sdk::Value(12.0)));
    CHECK(applyProperty(c, cat, sel, "startpos.placement", "headingRad", sdk::Value(3.0)));
    const sdk::StartPosition* p = c.findStartPosition(id);
    CHECK(p->alliance == sdk::Alliance::Red);
    CHECK(p->pose.xM == doctest::Approx(12.f));
    CHECK(p->pose.headingRad == doctest::Approx(3.f));

    EditorDocument doc;
    doc.setCatalog(cat);
    doc.load(c);
    DeviceDirectory devices;
    ProjectTreeModel tree(doc, devices);
    CHECK(tree.indexForKey("startPositions").isValid());
    CHECK(tree.indexForKey(QStringLiteral("start:") + QString::fromStdString(id)).isValid());

    sdk::Issue issue;
    issue.path = "startPositions[0].pose";
    CHECK(locateIssue(c, cat, issue).selection == sel);
}
