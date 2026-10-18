#include <QAbstractButton>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QToolButton>

#include <cmath>
#include <optional>

#include "editor_page.h"
#include "model/property_groups.h"
#include "model/scene_builder.h"
#include "model/slot_status.h"
#include "test_support.h"
#include "viewmodel/editor_controller.h"
#include "viewmodel/viewport_controller.h"
#include "views/inspector_panel.h"
#include "views/property_form.h"

using namespace wpiemu::app::editor;
using namespace wpiemu::app::editor::testing;
namespace sdk = wpiemu::sdk;

namespace {

constexpr float kPi = static_cast<float>(sdk::kPi);

struct ViewFixture {
    FakeEditorBackend backend;
    EditorController editor{backend};
    ViewportController view{editor};
    ViewFixture() {
        editor.setAutosaveDelayMs(0);
        view.setViewSize(800, 600);
        editor.settings().setSnapEnabled(false);
    }
    QPointF screen(const sdk::Vec3& world) {
        const auto px = view.camera().project(world, 800.f, 600.f);
        REQUIRE(px);
        return {static_cast<qreal>(px->first), static_cast<qreal>(px->second)};
    }
    const sdk::RobotConfig& config() { return editor.document().config(); }
};

}  // namespace

// ---- model --------------------------------------------------------------------------------------------

TEST_CASE("start pose: the effective pose is the configured one, else the catalog's default, and edits report changes") {
    const auto cat = catalog();
    auto c = robot();
    REQUIRE_FALSE(c.sim.startPose.has_value());
    CHECK(ops::effectiveStartPose(c, cat) == cat.defaultStartPose);
    CHECK_FALSE(ops::setStartPose(c, std::nullopt));   // nothing to clear
    CHECK(ops::setStartPose(c, sdk::StartPose{3.f, 2.f, 0.5f}));
    CHECK(ops::effectiveStartPose(c, cat) == sdk::StartPose{3.f, 2.f, 0.5f});
    CHECK_FALSE(ops::setStartPose(c, sdk::StartPose{3.f, 2.f, 0.5f}));   // unchanged
    CHECK(ops::setStartPose(c, std::nullopt));
    CHECK_FALSE(c.sim.startPose.has_value());
}

TEST_CASE("start pose: the Robot page and the marker's page show the effective pose, a field edit stores the whole pose") {
    const auto cat = catalog();
    auto c = robot();
    const auto groups = propertyGroupsFor(c, cat, SelectionRef::robot());
    REQUIRE(groups.size() == 2);
    CHECK(groups[0].id == "robot");
    REQUIRE(groups[1].id == "robot.start");
    CHECK(groups[1].values.find("startX")->asDouble() == doctest::Approx(cat.defaultStartPose.xM));
    CHECK(groups[1].values.find("startY")->asDouble() == doctest::Approx(cat.defaultStartPose.yM));
    CHECK(sdk::validateProperties(groups[1].schema, groups[1].values).empty());
    CHECK(groups[1].schema.find("startYaw")->unit == sdk::PropertyUnit::Radians);   // shown in degrees, stored in radians
    CHECK(groups[1].schema.find("startX")->unit == sdk::PropertyUnit::Meters);

    const auto own = propertyGroupsFor(c, cat, SelectionRef::startPose());
    REQUIRE(own.size() == 1);
    CHECK(own[0].id == "robot.start");
    CHECK(describeSelection(c, cat, SelectionRef::startPose()) == "Start pose");

    // editing one field writes the pose, the other parts keep their effective values
    REQUIRE(applyProperty(c, cat, SelectionRef::robot(), "robot.start", "startX", sdk::Value(4.0)));
    REQUIRE(c.sim.startPose.has_value());
    CHECK(c.sim.startPose->xM == 4.f);
    CHECK(c.sim.startPose->yM == cat.defaultStartPose.yM);
    CHECK(c.sim.startPose->yawRad == cat.defaultStartPose.yawRad);
    // the heading is stored in radians and wrapped into (-pi, pi]
    REQUIRE(applyProperty(c, cat, SelectionRef::startPose(), "robot.start", "startYaw", sdk::Value(sdk::degToRad(270.0))));
    CHECK(c.sim.startPose->yawRad == doctest::Approx(sdk::degToRad(-90.0)).epsilon(1e-5));
    CHECK(c.sim.startPose->xM == 4.f);
    // the issue the validator raises for an unusable pose leads to the Robot page
    sdk::Issue issue;
    issue.path = "sim.startPose";
    issue.code = "start_pose_invalid";
    const IssueLocation where = locateIssue(c, cat, issue);
    CHECK(where.selection == SelectionRef::robot());
    CHECK(where.fieldKey == "startX");
}

TEST_CASE("start pose: edits are undoable, a drag is one undo step and going back to the default is its own step") {
    FakeEditorBackend backend;
    EditorController editor(backend);
    editor.setAutosaveDelayMs(0);
    auto& doc = editor.document();
    REQUIRE(doc.undoStack()->count() == 0);

    for (const float x : {1.f, 1.5f, 2.f}) doc.setStartPose(sdk::StartPose{x, 2.f, 0.3f}, "g1");
    CHECK(doc.undoStack()->count() == 1);
    REQUIRE(doc.config().sim.startPose.has_value());
    CHECK(*doc.config().sim.startPose == sdk::StartPose{2.f, 2.f, 0.3f});
    doc.undoStack()->undo();
    CHECK_FALSE(doc.config().sim.startPose.has_value());
    doc.undoStack()->redo();
    CHECK(doc.config().sim.startPose.has_value());

    CHECK(doc.clearStartPose());
    CHECK_FALSE(doc.config().sim.startPose.has_value());
    CHECK(doc.undoStack()->count() == 2);
    CHECK_FALSE(doc.clearStartPose());   // nothing to clear: no undo entry
    CHECK(doc.undoStack()->count() == 2);
    doc.undoStack()->undo();
    CHECK(*doc.config().sim.startPose == sdk::StartPose{2.f, 2.f, 0.3f});
}

// ---- the marker in the scene ----------------------------------------------------------------------------

TEST_CASE("start pose marker: a footprint of the hull at the effective pose with a heading arrow, fainter while it is the default") {
    const auto cat = catalog();
    auto c = robot();
    auto scene = buildScene(c, cat, SceneTheme::dark());
    const SceneItem* foot = scene.find(kStartPoseFootprintId);
    const SceneItem* nose = scene.find(kStartPoseNoseId);
    REQUIRE(foot != nullptr);
    REQUIRE(nose != nullptr);
    CHECK(foot->ref == SelectionRef::startPose());
    CHECK(nose->ref == SelectionRef::startPose());
    CHECK(foot->pickable);
    CHECK(nose->pickable);
    CHECK(approx(foot->world.position.x, cat.defaultStartPose.xM));
    CHECK(approx(foot->world.position.y, cat.defaultStartPose.yM));
    CHECK(foot->world.position.z < 0.05f);   // on the floor
    CHECK(approx(foot->size.x, c.chassis.sizeM.x));
    CHECK(approx(foot->size.y, c.chassis.sizeM.y));
    CHECK(nose->world.position.x > foot->world.position.x + c.chassis.sizeM.x * 0.5f);   // ahead of the footprint
    const float fainter = foot->opacity;

    c.sim.startPose = sdk::StartPose{-2.f, 3.f, kPi / 2.f};
    scene = buildScene(c, cat, SceneTheme::dark());
    foot = scene.find(kStartPoseFootprintId);
    nose = scene.find(kStartPoseNoseId);
    REQUIRE(foot != nullptr);
    REQUIRE(nose != nullptr);
    CHECK(approx(foot->world.position.x, -2.f));
    CHECK(approx(foot->world.position.y, 3.f));
    CHECK(approx(foot->world.rotation.yaw(), kPi / 2.f, 1e-4f));
    CHECK(nose->world.position.y > foot->world.position.y + c.chassis.sizeM.x * 0.5f);   // the heading points along +Y now
    CHECK(foot->opacity > fainter);                                                      // an explicit pose is drawn stronger

    // the footprint is the hull's, which sits behind a COM that is offset forward
    c.sim.startPose = sdk::StartPose{1.f, 1.f, 0.f};
    c.chassis.comOffsetM = {0.1f, 0.f, 0.f};
    scene = buildScene(c, cat, SceneTheme::dark());
    CHECK(approx(scene.find(kStartPoseFootprintId)->world.position.x, 0.9f));
}

TEST_CASE("start pose marker: picking prefers solid parts but a volume does not hide the marker") {
    ViewFixture f;
    CHECK(f.view.chooseItem({"i:startpose"}) == SelectionRef::startPose());
    CHECK(f.view.chooseItem({"i:startpose:nose"}) == SelectionRef::startPose());
    CHECK(f.view.chooseItem({"i:startpose:nose:shaft"}) == SelectionRef::startPose());
    CHECK(f.view.chooseItem({"i:hull", "i:startpose"}) == SelectionRef::chassis());   // the nearer solid wins
    CHECK(f.view.chooseItem({"i:overlay:intake:pickupZone", "i:startpose"}) == SelectionRef::startPose());
}

TEST_CASE("start pose marker: framing the robot ignores a far start pose, framing the marker goes to it") {
    ViewFixture f;
    f.editor.document().setStartPose(sdk::StartPose{9.f, 4.f, 0.f});   // far from the robot-centred scene origin
    f.editor.selection().set(SelectionRef::robot());
    f.view.frameSelection();
    CHECK(sdk::length(f.view.camera().target) < 1.f);

    f.editor.selection().set(SelectionRef::startPose());
    f.view.frameSelection();
    CHECK(sdk::length(f.view.camera().target - sdk::Vec3{9.f, 4.f, 0.f}) < 1.f);
}

// ---- dragging the marker ----------------------------------------------------------------------------------

TEST_CASE("start pose drag: pressing the footprint selects it, dragging moves it over the floor as one undo step") {
    ViewFixture f;
    const sdk::StartPose start = ops::effectiveStartPose(f.config(), f.editor.document().catalog());
    const QPointF grab = f.screen({start.xM, start.yM, 0.f});

    // a plain click only selects: the default placement stays the default
    f.view.press(grab.x(), grab.y(), Qt::LeftButton, 0, {"i:startpose"});
    f.view.release(grab.x(), grab.y(), Qt::LeftButton);
    CHECK(f.editor.selection().current() == SelectionRef::startPose());
    CHECK_FALSE(f.config().sim.startPose.has_value());
    CHECK(f.editor.document().undoStack()->count() == 0);

    f.view.press(grab.x(), grab.y(), Qt::LeftButton, 0, {"i:startpose"});
    for (int i = 1; i <= 4; ++i) {
        const QPointF p = f.screen({start.xM + 0.25f * static_cast<float>(i), start.yM + 0.1f * static_cast<float>(i), 0.f});
        f.view.move(p.x(), p.y(), Qt::LeftButton, 0);
    }
    f.view.release(0, 0, Qt::LeftButton);
    REQUIRE(f.config().sim.startPose.has_value());
    CHECK(approx(f.config().sim.startPose->xM, start.xM + 1.0f, 3e-3f));
    CHECK(approx(f.config().sim.startPose->yM, start.yM + 0.4f, 3e-3f));
    CHECK(approx(f.config().sim.startPose->yawRad, start.yawRad));   // moving never turns it
    CHECK(f.editor.document().undoStack()->count() == 1);
    f.editor.document().undoStack()->undo();
    CHECK_FALSE(f.config().sim.startPose.has_value());
}

TEST_CASE("start pose drag: the grid snaps the position unless Ctrl flips it") {
    ViewFixture f;
    f.editor.settings().setSnapEnabled(true);
    const float step = static_cast<float>(f.editor.settings().moveStepM());
    REQUIRE(step > 0.f);
    const sdk::StartPose start = ops::effectiveStartPose(f.config(), f.editor.document().catalog());
    const QPointF grab = f.screen({start.xM, start.yM, 0.f});
    REQUIRE(f.view.beginStartPoseDrag("startpose", grab.x(), grab.y()));
    const sdk::Vec3 target{start.xM + 0.4137f, start.yM - 0.2951f, 0.f};
    const QPointF p = f.screen(target);
    f.view.updateStartPoseDrag(p.x(), p.y(), false);
    const sdk::StartPose snapped = *f.config().sim.startPose;
    CHECK(approx(std::fmod(std::fabs(snapped.xM) / step + 1e-3f, 1.f), 0.f, 0.01f));
    CHECK(approx(std::fmod(std::fabs(snapped.yM) / step + 1e-3f, 1.f), 0.f, 0.01f));
    f.view.updateStartPoseDrag(p.x(), p.y(), true);   // Ctrl: snapping off
    CHECK(approx(f.config().sim.startPose->xM, target.x, 3e-3f));
    CHECK(approx(f.config().sim.startPose->yM, target.y, 3e-3f));
    f.view.endStartPoseDrag();
}

TEST_CASE("start pose drag: the arrow's tip turns the pose to face the cursor and leaves the position alone") {
    ViewFixture f;
    f.editor.settings().setSnapEnabled(false);
    sdk::StartPose start{1.f, -1.f, 0.f};
    f.editor.document().setStartPose(start);
    const QPointF grab = f.screen({start.xM + 0.6f, start.yM, 0.02f});
    f.view.press(grab.x(), grab.y(), Qt::LeftButton, 0, {"i:startpose:nose"});
    const QPointF left = f.screen({start.xM, start.yM + 2.f, 0.f});   // straight to the robot's left
    f.view.move(left.x(), left.y(), Qt::LeftButton, 0);
    f.view.release(left.x(), left.y(), Qt::LeftButton);
    REQUIRE(f.config().sim.startPose.has_value());
    CHECK(approx(f.config().sim.startPose->yawRad, kPi / 2.f, 0.02f));
    CHECK(approx(f.config().sim.startPose->xM, 1.f));
    CHECK(approx(f.config().sim.startPose->yM, -1.f));
    CHECK(f.editor.document().undoStack()->count() == 2);   // setting it, then the turn
    CHECK(f.view.infoText().contains("Start pose"));
}

// ---- the inspector -----------------------------------------------------------------------------------------

TEST_CASE("start pose card: shows the default until a value is typed, then stores the pose and offers the way back") {
    FakeEditorBackend backend;
    EditorPage page{backend};
    page.setAutosaveDelayMs(0);
    page.resize(1400, 900);
    page.show();
    pumpEvents(60);
    auto& c = page.controller();
    auto* inspector = page.inspector();
    c.selection().set(SelectionRef::robot());
    pumpEvents(20);

    auto* form = inspector->form("robot.start");
    REQUIRE(form != nullptr);
    REQUIRE(inspector->startPoseResetButton() != nullptr);
    REQUIRE(inspector->startPoseHint() != nullptr);
    CHECK_FALSE(inspector->startPoseResetButton()->isEnabled());   // already the default
    CHECK(inspector->startPoseHint()->text().contains("default placement"));
    auto* spin = qobject_cast<QDoubleSpinBox*>(form->editorFor("startX")->focusTarget());
    REQUIRE(spin != nullptr);
    const UnitSystem units = c.settings().units();
    CHECK(spin->value() == doctest::Approx(toDisplay(c.document().catalog().defaultStartPose.xM, sdk::PropertyUnit::Meters, units)).epsilon(1e-3));

    spin->setValue(toDisplay(3.0, sdk::PropertyUnit::Meters, units));   // the user types 3 m
    pumpEvents(20);
    REQUIRE(c.document().config().sim.startPose.has_value());
    CHECK(c.document().config().sim.startPose->xM == doctest::Approx(3.f).epsilon(1e-3));
    CHECK(c.document().config().sim.startPose->yM == c.document().catalog().defaultStartPose.yM);
    CHECK(inspector->startPoseResetButton()->isEnabled());
    CHECK_FALSE(inspector->startPoseHint()->text().contains("Not set"));

    inspector->startPoseResetButton()->click();
    pumpEvents(20);
    CHECK_FALSE(c.document().config().sim.startPose.has_value());
    CHECK_FALSE(inspector->startPoseResetButton()->isEnabled());
    spin = qobject_cast<QDoubleSpinBox*>(inspector->form("robot.start")->editorFor("startX")->focusTarget());
    CHECK(spin->value() == doctest::Approx(toDisplay(c.document().catalog().defaultStartPose.xM, sdk::PropertyUnit::Meters, units)).epsilon(1e-3));

    // the marker has its own page with the same card
    c.selection().set(SelectionRef::startPose());
    pumpEvents(20);
    CHECK(inspector->titleText() == "Start pose");
    CHECK(inspector->form("robot.start") != nullptr);
    CHECK(inspector->form("robot") == nullptr);
}
