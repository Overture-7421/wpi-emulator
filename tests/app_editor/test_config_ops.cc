#include "test_support.h"

#include "model/config_ops.h"
#include "model/slot_status.h"
#include "wpiemu/sdk/config/value_codec.h"
#include "wpiemu/sdk/mechanisms/attachment.h"

using namespace wpiemu::app::editor;
using namespace wpiemu::app::editor::testing;
namespace sdk = wpiemu::sdk;

TEST_CASE("editor ops: ids are derived from the template type and kept unique") {
    auto c = robot();
    CHECK(ops::baseIdForType("game_2026.fuel_intake") == "fuel_intake");
    CHECK(ops::baseIdForType("mechanism") == "mechanism");
    CHECK(ops::uniqueMechanismId(c, "mechanism") == "mechanism");
    CHECK(ops::uniqueMechanismId(c, "intake") == "intake_2");
    CHECK(ops::uniqueMechanismId(c, "chassis") == "chassis_2");   // reserved owner names are never handed out
    CHECK(ops::uniqueMechanismId(c, "drivetrain") == "drivetrain_2");
}

TEST_CASE("editor ops: adding a mechanism attaches it and defaults its properties") {
    const auto cat = catalog();
    auto c = robot();
    const auto* mechanism = cat.findTemplate("mechanism");
    REQUIRE(mechanism != nullptr);

    const std::string id = ops::addMechanism(c, *mechanism, {});
    const auto* m = c.findMechanism(id);
    REQUIRE(m != nullptr);
    CHECK(m->type == "mechanism");
    CHECK(m->attachment.onChassis());
    CHECK(m->properties.find("gearRatio") != nullptr);
    CHECK(m->geometry.size() == mechanism->defaultGeometry.size());
    CHECK(m->mappings.empty());

    const std::string child = ops::addMechanism(c, *mechanism, {"turret", "mover"});
    CHECK(c.findMechanism(child)->attachment.parentLink == "mover");
}

TEST_CASE("editor ops: a variant presets the choice property and names the mechanism after it") {
    const auto cat = catalog();
    auto c = robot();
    const auto* mechanism = cat.findTemplate("mechanism");
    REQUIRE(mechanism != nullptr);
    const std::string arm = ops::addMechanism(c, *mechanism, {}, {"motion", "arm"});
    CHECK(arm == "arm");
    CHECK(c.findMechanism(arm)->properties.find("motion")->asString() == "arm");
    CHECK(c.findMechanism(arm)->label == "Arm");
    const std::string linear = ops::addMechanism(c, *mechanism, {}, {"motion", "linear"});
    CHECK(c.findMechanism(linear)->properties.find("motion")->asString() == "linear");
    CHECK(ops::addMechanism(c, *mechanism, {}, {"motion", "arm"}) == "arm_2");
}

TEST_CASE("editor ops: normalizeOrder lists parents before children") {
    auto c = robot();
    // Move the turret (parent of the shooter) behind its child.
    std::swap(c.mechanisms[2], c.mechanisms[3]);
    CHECK(c.mechanisms[2].id == "shooter");
    ops::normalizeOrder(c);
    std::size_t turret = 0, shooter = 0;
    for (std::size_t i = 0; i < c.mechanisms.size(); ++i) {
        if (c.mechanisms[i].id == "turret") turret = i;
        if (c.mechanisms[i].id == "shooter") shooter = i;
    }
    CHECK(turret < shooter);
}

TEST_CASE("editor ops: reparenting keeps the world placement and refuses cycles") {
    const auto cat = catalog();
    auto c = robot();
    const sdk::Vec3 before = rootWorld(c, cat, "climber");

    REQUIRE(ops::reparentMechanism(c, cat, "climber", "turret", "mover"));
    CHECK(c.findMechanism("climber")->attachment.parentMechanismId == "turret");
    CHECK(approx(rootWorld(c, cat, "climber"), before));

    CHECK_FALSE(ops::canReparent(c, "turret", "shooter"));      // shooter hangs below the turret
    CHECK_FALSE(ops::reparentMechanism(c, cat, "turret", "shooter", "root"));
    CHECK_FALSE(ops::canReparent(c, "turret", "turret"));
    CHECK_FALSE(ops::reparentMechanism(c, cat, "climber", "turret", "no_such_link"));
    CHECK(ops::reparentMechanism(c, cat, "climber", "chassis", "root"));
    CHECK(approx(rootWorld(c, cat, "climber"), before));
}

TEST_CASE("editor ops: attachments produced by the editor satisfy the SDK validation contract") {
    const auto cat = catalog();
    auto c = robot();
    ops::reparentMechanism(c, cat, "climber", "turret", "mover");
    FakeEditorBackend backend;
    for (const auto& issue : backend.validate(c)) CHECK(issue.severity != sdk::Severity::Error);
}

TEST_CASE("editor ops: removing a mechanism re-attaches its children without moving them") {
    const auto cat = catalog();
    auto c = robot();
    const sdk::Vec3 shooterBefore = rootWorld(c, cat, "shooter");
    REQUIRE(ops::removeMechanism(c, cat, "turret"));
    CHECK(c.findMechanism("turret") == nullptr);
    REQUIRE(c.findMechanism("shooter") != nullptr);
    CHECK(c.findMechanism("shooter")->attachment.onChassis());
    CHECK(approx(rootWorld(c, cat, "shooter"), shooterBefore));
    CHECK_FALSE(ops::removeMechanism(c, cat, "turret"));
}

TEST_CASE("editor ops: duplicating copies the sub-tree, not the device mappings") {
    auto c = robot();
    const std::size_t before = c.mechanisms.size();
    const std::string copy = ops::duplicateMechanism(c, "turret");
    CHECK(copy == "turret_2");
    CHECK(c.mechanisms.size() == before + 2);   // turret + its shooter child
    const auto* shooter2 = c.findMechanism("shooter_2");
    REQUIRE(shooter2 != nullptr);
    CHECK(shooter2->attachment.parentMechanismId == "turret_2");
    CHECK(shooter2->mappings.empty());
    CHECK(c.findMechanism("shooter")->mappings.size() == 1);   // the original keeps its mapping
}

TEST_CASE("editor ops: moving the COM re-expresses everything so the robot looks the same") {
    const auto cat = catalog();
    auto c = robot();
    const auto chassisBefore = ops::chassisFrameWorld(c.chassis);
    const sdk::Vec3 wheelBefore = ops::wheelWorld(c.chassis, c.drivetrain.spec.wheels[0]).position;
    const sdk::Vec3 intakeBefore = rootWorld(c, cat, "intake");
    const sdk::Vec3 shooterBefore = rootWorld(c, cat, "shooter");
    const sdk::Vec3 bumperBefore = (chassisBefore * c.chassis.primitives[0].local).position;

    ops::setCom(c, 0.05f, -0.03f, 0.30f, true, &cat);
    CHECK(approx(c.chassis.comOffsetM.x, 0.05f));
    CHECK(approx(ops::wheelWorld(c.chassis, c.drivetrain.spec.wheels[0]).position, wheelBefore));
    CHECK(approx(rootWorld(c, cat, "intake"), intakeBefore));
    CHECK(approx(rootWorld(c, cat, "shooter"), shooterBefore));
    CHECK(approx((ops::chassisFrameWorld(c.chassis) * c.chassis.primitives[0].local).position, bumperBefore));

    // Without compensation only the COM itself moves.
    auto d = robot();
    ops::setCom(d, 0.05f, 0.f, d.chassis.comHeightM, false);
    CHECK(approx(d.drivetrain.spec.wheels[0].positionM.x, robot().drivetrain.spec.wheels[0].positionM.x));
}

TEST_CASE("editor ops: plugin volumes are moved along with the COM") {
    const auto cat = catalog();
    auto c = robot();
    auto vols = ops::volumesFor(c, cat, "");
    REQUIRE_FALSE(vols.empty());
    auto worldOf = [&](const sdk::RobotConfig& cfg) {
        sdk::Box3 box;
        const auto* store = ops::volumeStore(cfg, cat, "");
        const sdk::Value* raw = store != nullptr ? store->find(vols[0].key) : nullptr;
        sdk::fromValue(raw != nullptr ? *raw : vols[0].defaultValue, box);
        return (ops::chassisFrameWorld(cfg.chassis) * box.pose).position;
    };
    const sdk::Vec3 before = worldOf(c);
    ops::setCom(c, 0.04f, 0.02f, c.chassis.comHeightM + 0.05f, true, &cat);
    CHECK(approx(worldOf(c), before));
}

TEST_CASE("editor ops: drivetrain kind changes steering and drops stale steer mappings") {
    auto c = robot();
    REQUIRE(c.drivetrain.mappings.size() == 8);
    ops::setDrivetrainKind(c, "tank");
    for (const auto& w : c.drivetrain.spec.wheels) CHECK_FALSE(w.steered);
    CHECK(c.drivetrain.mappings.size() == 4);   // only the drive motors remain mapped
    for (const auto& m : c.drivetrain.mappings) CHECK(m.slot.slot.find(".drive") != std::string::npos);
    ops::setDrivetrainKind(c, "swerve");
    CHECK(c.drivetrain.spec.wheels[0].steered);
    CHECK(c.drivetrain.spec.wheels[0].kind == sdk::WheelKind::Swerve);
}

TEST_CASE("editor ops: wheels can be added, renamed and removed with their mappings") {
    auto c = robot();
    const std::string added = ops::addWheel(c);
    CHECK(c.drivetrain.spec.wheels.size() == 5);
    CHECK(ops::findWheel(c, added) != nullptr);
    CHECK(ops::renameWheel(c, "fl", "front_left"));
    CHECK(ops::findWheel(c, "fl") == nullptr);
    bool renamed = false;
    for (const auto& m : c.drivetrain.mappings) renamed = renamed || m.slot.slot == "front_left.drive";
    CHECK(renamed);
    CHECK_FALSE(ops::renameWheel(c, "front_left", "fr"));   // taken
    CHECK(ops::removeWheel(c, "front_left"));
    for (const auto& m : c.drivetrain.mappings) CHECK(m.slot.slot.rfind("front_left", 0) != 0);
    CHECK_FALSE(ops::removeWheel(c, "front_left"));
}

TEST_CASE("editor ops: primitives are added, duplicated and removed per owner") {
    auto c = robot();
    const std::size_t chassisShapes = c.chassis.primitives.size();
    const int box = ops::addPrimitive(c, ops::kChassisOwner, sdk::PrimitiveKind::Box);
    CHECK(box == static_cast<int>(chassisShapes));
    const int cyl = ops::addPrimitive(c, "intake", sdk::PrimitiveKind::Cylinder);
    CHECK(cyl >= 0);
    CHECK(c.findMechanism("intake")->geometry.back().kind == sdk::PrimitiveKind::Cylinder);
    const int dup = ops::duplicatePrimitive(c, ops::kChassisOwner, 0);
    CHECK(dup == 1);
    CHECK(c.chassis.primitives[1].name != c.chassis.primitives[0].name);
    CHECK(ops::removePrimitive(c, ops::kChassisOwner, 1));
    CHECK_FALSE(ops::removePrimitive(c, ops::kChassisOwner, 99));
    CHECK(ops::addPrimitive(c, "nobody", sdk::PrimitiveKind::Box) == -1);
}

TEST_CASE("editor ops: Box3 properties of templates and plugin panels become viewport volumes") {
    const auto cat = catalog();
    auto c = robot();
    const auto intakeVolumes = ops::volumesFor(c, cat, "intake");
    REQUIRE(intakeVolumes.size() == 1);
    CHECK(intakeVolumes[0].key == "pickupZone");
    const auto pluginVolumes = ops::volumesFor(c, cat, "");
    REQUIRE(pluginVolumes.size() == 1);
    CHECK(pluginVolumes[0].key == "hopperVolume");
    CHECK(pluginVolumes[0].color == "#44cc88");   // colour comes from the panel's overlay descriptor
    CHECK(ops::volumesFor(c, cat, "turret").empty());
    CHECK(ops::volumeStore(c, cat, "") != nullptr);
}

TEST_CASE("editor ops: slot specs come from templates, wheels and the robot") {
    const auto cat = catalog();
    auto c = robot();
    CHECK(ops::slotSpecsFor(c, cat, "intake").size() == 3);
    CHECK(ops::slotSpecsFor(c, cat, "robot").size() == 1);
    CHECK(ops::slotSpecsFor(c, cat, "drivetrain").size() ==
          12);  // 4 swerve modules x (drive, steer)
    CHECK(ops::slotSpecsFor(c, cat, "nobody").empty());
    ops::setDrivetrainKind(c, "tank");
    CHECK(ops::slotSpecsFor(c, cat, "drivetrain").size() == 4);
    CHECK(ops::slotOwners(c).size() == 2 + c.mechanisms.size() + c.sensors.size());
}

TEST_CASE("editor ops: mapping a device reuses an existing alias of the same physical device") {
    auto c = robot();
    const sdk::DeviceKey existing{"phoenix6", "talonfx", 1, "canivore", -1};
    const sdk::DeviceKey sameDeviceOtherBus{"phoenix6", "talonfx", 1, "rio", -1};
    CHECK(ops::declaredKeyFor(c, sameDeviceOtherBus) == existing);
    const sdk::DeviceKey fresh{"phoenix6", "talonfx", 40, "rio", -1};
    CHECK(ops::declaredKeyFor(c, fresh) == fresh);

    CHECK(ops::setMapping(c, {"intake", "deploy"}, fresh));
    CHECK(ops::findMapping(c, {"intake", "deploy"})->device == fresh);
    CHECK(ops::setMapping(c, {"intake", "deploy"}, existing));   // replaces, never duplicates
    int count = 0;
    for (const auto& m : c.findMechanism("intake")->mappings)
        if (m.slot.slot == "deploy") ++count;
    CHECK(count == 1);
    CHECK(ops::clearMapping(c, {"intake", "deploy"}));
    CHECK_FALSE(ops::clearMapping(c, {"intake", "deploy"}));
    CHECK_FALSE(ops::setMapping(c, {"nobody", "x"}, fresh));
}
