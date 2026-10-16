#include "test_support.h"

#include "model/property_groups.h"
#include "model/slot_status.h"
#include "model/spec_values.h"
#include "model/transform_access.h"
#include "wpiemu/sdk/config/value_codec.h"

using namespace wpiemu::app::editor;
using namespace wpiemu::app::editor::testing;
namespace sdk = wpiemu::sdk;

namespace {
DeviceDirectory directoryOf(const std::vector<sdk::DiscoveredDevice>& devices) {
    DeviceDirectory d;
    d.setDevices(devices);
    return d;
}
}  // namespace

TEST_CASE("slot status: unmapped required / optional, mapped, not detected") {
    const auto cat = catalog();
    auto c = robot();
    const auto devices = directoryOf(FakeEditorBackend::sampleDevices());

    const auto infos = slotInfos(c, cat, devices, "intake");
    REQUIRE(infos.size() == 3);
    CHECK(infos[0].spec.name == "motor");
    CHECK(infos[0].state == SlotState::NotDetected);     // mapped to TalonFX 10, which the sample discovery has not seen
    CHECK(infos[1].state == SlotState::OptionalEmpty);

    const auto shooter = slotInfos(c, cat, devices, "shooter");
    CHECK(shooter[1].state == SlotState::Unmapped);      // required feeder motor is empty

    const auto wheel = slotInfos(c, cat, devices, "drivetrain");
    CHECK(wheel.front().state == SlotState::Mapped);

    // With no discovery data at all, mapped slots are simply "mapped".
    const auto noDevices = directoryOf({});
    CHECK(slotInfos(c, cat, noDevices, "intake")[0].state == SlotState::Mapped);
}

TEST_CASE("slot status: incompatible devices and alias conflicts are flagged") {
    const auto cat = catalog();
    auto c = robot();
    auto devices = FakeEditorBackend::sampleDevices();
    // Point the intake roller at the Pigeon 2: an IMU cannot drive a motor.
    ops::setMapping(c, {"intake", "motor"}, {"phoenix6", "pigeon2", 13, "canivore", -1});
    CHECK(slotInfos(c, cat, directoryOf(devices), "intake")[0].state == SlotState::Incompatible);

    // The same TalonFX addressed through two buses and mapped twice.
    auto d = robot();
    ops::setMapping(d, {"intake", "motor"}, {"phoenix6", "talonfx", 1, "rio", -1});
    CHECK(slotInfos(d, cat, directoryOf(devices), "intake")[0].state == SlotState::AliasConflict);
    CHECK(slotInfos(d, cat, directoryOf(devices), "drivetrain")[0].state == SlotState::AliasConflict);
}

TEST_CASE("slot status: the best slot for a dropped device prefers unfilled required slots") {
    const auto cat = catalog();
    auto c = robot();
    const sdk::CapabilitySet motor = sdk::Capability::MotorOutput;
    const auto best = bestSlotFor(c, cat, "shooter", motor);
    REQUIRE(best);
    CHECK(best->slot == "feeder");                                     // flywheel is mapped, feeder is empty and required
    CHECK(bestSlotFor(c, cat, "intake", motor)->slot == "deploy");     // motor taken: only the optional deploy motor is free
    CHECK_FALSE(bestSlotFor(c, cat, "indexer", sdk::Capability::Imu));
    ops::clearMapping(c, {"drivetrain", "fl.steer"});
    CHECK(bestSlotFor(c, cat, "drivetrain", motor, "fl.")->slot == "fl.steer");
    ops::setMapping(c, {"shooter", "feeder"}, {"phoenix6", "talonfx", 30, "rio", -1});
    CHECK_FALSE(bestSlotFor(c, cat, "shooter", motor));                // nothing left that accepts a motor
}

TEST_CASE("local validation reports unfilled slots, loopback devices and incompatible mappings") {
    const auto cat = catalog();
    auto c = robot();
    auto devices = FakeEditorBackend::sampleDevices();
    devices.push_back(FakeEditorBackend::makeDevice("phoenix6", "talonfx", 77, sdk::kBusUnknown, sdk::Capability::MotorOutput));
    const auto issues = localIssues(c, cat, directoryOf(devices));
    auto has = [&](const char* code, const std::string& pathPart) {
        for (const auto& i : issues)
            if (i.code == code && i.path.find(pathPart) != std::string::npos) return true;
        return false;
    };
    CHECK(has("slot_unfilled", "shooter.feeder"));
    CHECK(has("unmapped_loopback", "talonfx[77]"));
    CHECK_FALSE(has("unmapped_loopback", "talonfx[1]"));   // mapped devices are not loopback
    CHECK(has("device_not_detected", "intake.motor"));

    ops::setMapping(c, {"intake", "motor"}, {"phoenix6", "pigeon2", 13, "canivore", -1});
    const auto after = localIssues(c, cat, directoryOf(devices));
    bool incompatible = false;
    for (const auto& i : after) incompatible = incompatible || (i.code == "slot_incompatible" && i.severity == sdk::Severity::Error);
    CHECK(incompatible);
}

TEST_CASE("local validation: the same device driving two motor slots is a warning") {
    const auto cat = catalog();
    auto c = robot();
    const sdk::DeviceKey key{"phoenix6", "talonfx", 1, "canivore", -1};
    ops::setMapping(c, {"indexer", "motor"}, key);   // TalonFX 1 already drives fl.drive
    bool seen = false;
    for (const auto& i : localIssues(c, cat, directoryOf(FakeEditorBackend::sampleDevices()))) seen = seen || i.code == "device_multi_mapped";
    CHECK(seen);
}

TEST_CASE("issues are merged without duplicates and located back to selections") {
    const auto cat = catalog();
    auto c = robot();
    sdk::Issue a{sdk::Severity::Error, "intake.motor", "a", "slot_incompatible"};
    sdk::Issue b{sdk::Severity::Error, "intake.motor", "different wording", "slot_incompatible"};
    CHECK(mergeIssues({{a}, {b}}).size() == 1);
    CHECK(mergeIssues({{a}, {{sdk::Severity::Warning, "intake.motor", "a", "slot_incompatible"}}}).size() == 2);

    auto at = [&](const std::string& path) { return locateIssue(c, cat, {sdk::Severity::Error, path, "x", {}}); };
    CHECK(at("mechanisms[2].properties.gearRatio").selection == SelectionRef::mechanism("turret"));
    CHECK(at("mechanisms[2].properties.gearRatio").fieldKey == "gearRatio");
    CHECK(at("mechanisms[3].attachment").fieldKey == "parent");
    CHECK(at("intake.motor").selection == SelectionRef::slot("intake", "motor"));
    CHECK(at("drivetrain.fl.drive").selection == SelectionRef::slot("drivetrain", "fl.drive"));
    CHECK(at("robot.imu").selection == SelectionRef::slot("robot", "imu"));
    CHECK(at("chassis.massKg").selection == SelectionRef::chassis());
    CHECK(at("chassis.massKg").fieldKey == "massKg");
    CHECK(at("plugins.game_2026.data.preloadFuel").selection == SelectionRef::plugin());
    CHECK(at("plugins.game_2026.data.preloadFuel").fieldKey == "preloadFuel");
    CHECK(at("device:phoenix6/talonfx[7]").selection == SelectionRef::device("phoenix6/talonfx[7]"));
    CHECK(at("totally.unknown").selection.isNone());
}

TEST_CASE("property groups: generic schema forms for chassis, wheels, mechanisms and the plugin") {
    const auto cat = catalog();
    auto c = robot();

    auto chassis = propertyGroupsFor(c, cat, SelectionRef::chassis());
    REQUIRE(chassis.size() == 1);
    CHECK(chassis[0].schema.find("massKg") != nullptr);
    CHECK(sdk::validateProperties(chassis[0].schema, chassis[0].values).empty());

    auto wheel = propertyGroupsFor(c, cat, SelectionRef::wheel("fl"));
    REQUIRE(wheel.size() == 1);
    CHECK(sdk::validateProperties(wheel[0].schema, wheel[0].values).empty());

    // A slot edits its owner; the shooter has its own template group plus a placement group.
    auto shooter = propertyGroupsFor(c, cat, SelectionRef::slot("shooter", "flywheel"));
    std::vector<std::string> ids;
    for (const auto& g : shooter) ids.push_back(g.id);
    CHECK(ids == std::vector<std::string>{"mech.general", "mech.placement", "mech.props"});
    CHECK(formTargetOf(c, SelectionRef::slot("shooter", "flywheel")) == SelectionRef::mechanism("shooter"));
    CHECK(formTargetOf(c, SelectionRef::slot("drivetrain", "fl.drive")) == SelectionRef::wheel("fl"));
    CHECK(formTargetOf(c, SelectionRef::slot("robot", "imu")) == SelectionRef::robot());
    CHECK(formTargetOf(c, SelectionRef::overlay("", "hopperVolume")) == SelectionRef::plugin());

    // Plugin panels for indexer/roller mechanisms and the plugin section.
    auto indexer = propertyGroupsFor(c, cat, SelectionRef::mechanism("indexer"));
    bool feeding = false;
    for (const auto& g : indexer) feeding = feeding || g.id == "panel:game_2026.feeding";
    CHECK(feeding);
    auto plugin = propertyGroupsFor(c, cat, SelectionRef::plugin());
    bool hopper = false;
    for (const auto& g : plugin) hopper = hopper || g.id == "panel:game_2026.hopper";
    CHECK(hopper);
    CHECK(propertyGroupsFor(c, cat, SelectionRef::mechanism("nobody")).empty());
}

TEST_CASE("property groups: placement offers every non-descendant as a parent") {
    const auto cat = catalog();
    auto c = robot();
    auto groups = propertyGroupsFor(c, cat, SelectionRef::mechanism("turret"));
    const auto* placement = &groups[1];
    REQUIRE(placement->id == "mech.placement");
    std::vector<std::string> options;
    for (const auto& o : placement->schema.find("parent")->options) options.push_back(o.value);
    CHECK(std::find(options.begin(), options.end(), "chassis") != options.end());
    CHECK(std::find(options.begin(), options.end(), "climber") != options.end());
    CHECK(std::find(options.begin(), options.end(), "shooter") == options.end());   // would create a cycle
    CHECK(std::find(options.begin(), options.end(), "turret") == options.end());

    auto shooterGroups = propertyGroupsFor(c, cat, SelectionRef::mechanism("shooter"));
    std::vector<std::string> links;
    for (const auto& o : shooterGroups[1].schema.find("parentLink")->options) links.push_back(o.value);
    CHECK(links == std::vector<std::string>{"root", "top"});   // parent is the turret
}

TEST_CASE("property edits write through to the right place") {
    const auto cat = catalog();
    auto c = robot();
    const auto sel = SelectionRef::mechanism("turret");
    CHECK(applyProperty(c, cat, sel, "mech.props", "gearRatio", sdk::Value(55.0)));
    CHECK(c.findMechanism("turret")->properties.find("gearRatio")->asDouble() == 55.0);
    CHECK(applyProperty(c, cat, sel, "mech.general", "label", sdk::Value("Yaw")));
    CHECK(c.findMechanism("turret")->label == "Yaw");
    CHECK(applyProperty(c, cat, sel, "mech.general", "id", sdk::Value("yaw")));
    CHECK(c.findMechanism("yaw") != nullptr);
    CHECK(c.findMechanism("shooter")->resolvedAttachment().parentMechanismId == "yaw");

    sdk::Pose3 mount;
    mount.position = {0.1f, 0.2f, 0.3f};
    CHECK(applyProperty(c, cat, SelectionRef::mechanism("yaw"), "mech.placement", "mount", sdk::toValue(mount)));
    CHECK(approx(c.findMechanism("yaw")->mount.position, {0.1f, 0.2f, 0.3f}));

    CHECK(applyProperty(c, cat, SelectionRef::plugin(), "plugin", "preloadFuel", sdk::Value(12)));
    CHECK(c.findPlugin("game_2026")->data.find("preloadFuel")->asInt() == 12);
    CHECK(applyProperty(c, cat, SelectionRef::plugin(), "panel:game_2026.hopper", "capacity", sdk::Value(55)));
    CHECK(c.findPlugin("game_2026")->data.find("capacity")->asInt() == 55);

    CHECK(applyProperty(c, cat, SelectionRef::wheel("fl"), "wheel", "radius", sdk::Value(0.05)));
    CHECK(approx(ops::findWheel(c, "fl")->radiusM, 0.05f));
    CHECK(applyProperty(c, cat, SelectionRef::wheel("fl"), "wheel", "steered", sdk::Value(false)));
    CHECK(ops::findMapping(c, {"drivetrain", "fl.steer"}) == nullptr);   // the steer mapping goes away with the steering
    CHECK(applyProperty(c, cat, SelectionRef::drivetrain(), "drivetrain", "kind", sdk::Value("tank")));
    CHECK(c.drivetrain.spec.kind == "tank");
    CHECK(applyProperty(c, cat, SelectionRef::robot(), "robot", "name", sdk::Value("Bot")));
    CHECK(c.name == "Bot");
    CHECK_FALSE(applyProperty(c, cat, SelectionRef::mechanism("nobody"), "mech.props", "x", sdk::Value(1)));
}

TEST_CASE("changing the COM through the inspector keeps the robot in place") {
    const auto cat = catalog();
    auto c = robot();
    const sdk::Vec3 wheelBefore = ops::wheelWorld(c.chassis, c.drivetrain.spec.wheels[1]).position;
    const sdk::Vec3 intakeBefore = rootWorld(c, cat, "intake");
    CHECK(applyProperty(c, cat, SelectionRef::chassis(), "chassis", "comOffsetX", sdk::Value(0.06)));
    CHECK(applyProperty(c, cat, SelectionRef::chassis(), "chassis", "comHeight", sdk::Value(0.3)));
    CHECK(approx(ops::wheelWorld(c.chassis, c.drivetrain.spec.wheels[1]).position, wheelBefore));
    CHECK(approx(rootWorld(c, cat, "intake"), intakeBefore));
}

TEST_CASE("transforms: read and write for chassis, COM, wheels, shapes, mechanisms and overlays") {
    const auto cat = catalog();
    auto c = robot();

    auto hull = readTransform(c, cat, SelectionRef::chassis());
    REQUIRE(hull);
    CHECK(hull->transform.canScale);
    CHECK_FALSE(hull->transform.canMove);
    auto t = hull->transform;
    t.size.x = 0.9f;
    CHECK(writeTransform(c, cat, SelectionRef::chassis(), t));
    CHECK(approx(c.chassis.sizeM.x, 0.9f));

    auto com = readTransform(c, cat, SelectionRef::com());
    REQUIRE(com);
    CHECK_FALSE(com->transform.canRotate);

    auto wheel = readTransform(c, cat, SelectionRef::wheel("fl"));
    REQUIRE(wheel);
    t = wheel->transform;
    t.local.position.x += 0.02f;
    t.size.x = 0.2f;   // diameter
    CHECK(writeTransform(c, cat, SelectionRef::wheel("fl"), t));
    CHECK(approx(ops::findWheel(c, "fl")->radiusM, 0.1f));
    CHECK(approx(ops::findWheel(c, "fl")->positionM.x, 0.30f));

    auto mech = readTransform(c, cat, SelectionRef::mechanism("shooter"));
    REQUIRE(mech);
    CHECK(approx(mech->parentWorld.position, ops::mechanismParentWorld(c, cat, *c.findMechanism("shooter")).position));
    t = mech->transform;
    t.local.position.z = 0.5f;
    CHECK(writeTransform(c, cat, SelectionRef::mechanism("shooter"), t));
    CHECK(approx(c.findMechanism("shooter")->mount.position.z, 0.5f));

    // Cylinder shapes: scaling X changes the radius, Z changes the height.
    auto shape = readTransform(c, cat, SelectionRef::primitive("intake", 0));
    REQUIRE(shape);
    REQUIRE(shape->transform.sizeKind == SizeKind::Cylinder);
    t = shape->transform;
    t.size.x = 0.2f;
    t.size.z = 0.7f;
    CHECK(writeTransform(c, cat, SelectionRef::primitive("intake", 0), t));
    CHECK(approx(c.findMechanism("intake")->geometry[0].radiusM, 0.1f));
    CHECK(approx(c.findMechanism("intake")->geometry[0].heightM, 0.7f));

    auto overlay = readTransform(c, cat, SelectionRef::overlay("intake", "pickupZone"));
    REQUIRE(overlay);
    t = overlay->transform;
    t.local.position.x = 0.55f;
    t.size.y = 0.8f;
    CHECK(writeTransform(c, cat, SelectionRef::overlay("intake", "pickupZone"), t));
    sdk::Box3 box;
    REQUIRE(sdk::fromValue(*c.findMechanism("intake")->properties.find("pickupZone"), box));
    CHECK(approx(box.pose.position.x, 0.55f));
    CHECK(approx(box.sizeM.y, 0.8f));

    CHECK_FALSE(readTransform(c, cat, SelectionRef::plugin()));
    CHECK_FALSE(readTransform(c, cat, SelectionRef::mechanism("nobody")));
}
