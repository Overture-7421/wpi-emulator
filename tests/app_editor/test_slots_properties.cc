#include "test_support.h"

#include "model/motion_links.h"
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

TEST_CASE("slot status: the motor slot of a following axis is not listed and issues point at the follower") {
    auto c = robot();
    const auto cat = catalog();
    REQUIRE(applyDrivenBy(c, cat, {"intake", "axis"}, {"drivetrain", "fl.drive"}, 1.0).empty());
    const DeviceDirectory none;

    for (const auto& info : slotInfos(c, cat, none, "intake")) CHECK(info.spec.name != "motor");
    CHECK(slotInfos(c, cat, none, "intake").size() < slotInfos(robot(), cat, none, "intake").size());
    const auto best = bestSlotFor(c, cat, "intake", sdk::CapabilitySet(sdk::Capability::MotorOutput));
    CHECK((!best || best->slot != "motor"));

    const MotionIndex motion(c, cat);
    const OwnerMotion intake = motion.ownerMotion(SelectionRef::mechanism("intake"));
    REQUIRE(intake.primarySource);
    CHECK(*intake.primarySource == SelectionRef::wheel("fl"));
    const OwnerMotion wheel = motion.ownerMotion(SelectionRef::wheel("fl"));
    REQUIRE(wheel.drives.size() == 1);
    CHECK(wheel.drives[0] == SelectionRef::mechanism("intake"));

    const auto where = locateIssue(c, cat, {sdk::Severity::Error, "motionLinks." + c.motionLinks[0].id, "x", "motion_link_invalid"});
    CHECK(where.selection == SelectionRef::mechanism("intake"));
    CHECK(where.fieldKey == "drivenBy");
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
    devices.push_back(FakeEditorBackend::makeDevice("wpilib", "roborio", 0, "rio", sdk::Capability::SupplyVoltage));
    const auto issues = localIssues(c, cat, directoryOf(devices));
    auto has = [&](const char* code, const std::string& pathPart) {
        for (const auto& i : issues)
            if (i.code == code && i.path.find(pathPart) != std::string::npos) return true;
        return false;
    };
    CHECK(has("slot_unfilled", "shooter.feeder"));
    CHECK(has("unmapped_loopback", "talonfx[77]"));
    CHECK_FALSE(has("unmapped_loopback", "talonfx[1]"));   // mapped devices are not loopback
    CHECK_FALSE(has("unmapped_loopback", "roborio"));      // nothing can be mapped to the roboRIO: no warning
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
    CHECK(at("plugins.demo_game.data.preloadFuel").selection == SelectionRef::plugin());
    CHECK(at("plugins.demo_game.data.preloadFuel").fieldKey == "preloadFuel");
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
    REQUIRE(wheel.size() == 3);   // drive, configure, placement
    CHECK(wheel[0].id == "wheel.drive");
    CHECK(wheel[1].id == "wheel");
    CHECK(wheel[2].id == "wheel.placement");
    CHECK(wheel[1].schema.properties.front().key == "kind");   // the type leads Configure
    for (const auto& g : wheel) CHECK(sdk::validateProperties(g.schema, g.values).empty());

    // A slot edits its owner; the shooter has its own template group plus a placement group.
    auto shooter = propertyGroupsFor(c, cat, SelectionRef::slot("shooter", "flywheel"));
    std::vector<std::string> ids;
    for (const auto& g : shooter) ids.push_back(g.id);
    CHECK(ids == std::vector<std::string>{"mech.drive:axis", "mech.props", "mech.placement", "panel:demo_game.aiming"});
    CHECK(formTargetOf(c, SelectionRef::slot("shooter", "flywheel")) == SelectionRef::mechanism("shooter"));
    CHECK(formTargetOf(c, SelectionRef::slot("drivetrain", "fl.drive")) == SelectionRef::wheel("fl"));
    CHECK(formTargetOf(c, SelectionRef::slot("robot", "imu")) == SelectionRef::robot());
    CHECK(formTargetOf(c, SelectionRef::overlay("", "hopperVolume")) == SelectionRef::plugin());

    // Plugin panels reach matching templates only, plus the plugin section.
    for (const auto& g : propertyGroupsFor(c, cat, SelectionRef::mechanism("indexer"))) CHECK(g.id != "panel:demo_game.aiming");
    auto plugin = propertyGroupsFor(c, cat, SelectionRef::plugin());
    bool hopper = false;
    for (const auto& g : plugin) hopper = hopper || g.id == "panel:demo_game.hopper";
    CHECK(hopper);
    CHECK(propertyGroupsFor(c, cat, SelectionRef::mechanism("nobody")).empty());
}

TEST_CASE("property groups: placement offers every non-descendant as a parent") {
    const auto cat = catalog();
    auto c = robot();
    auto groups = propertyGroupsFor(c, cat, SelectionRef::mechanism("turret"));
    const auto it = std::find_if(groups.begin(), groups.end(), [](const auto& g) { return g.id == "mech.placement"; });
    REQUIRE(it != groups.end());
    const auto* placement = &*it;
    std::vector<std::string> options;
    for (const auto& o : placement->schema.find("parent")->options) options.push_back(o.value);
    CHECK(std::find(options.begin(), options.end(), "chassis") != options.end());
    CHECK(std::find(options.begin(), options.end(), "climber") != options.end());
    CHECK(std::find(options.begin(), options.end(), "shooter") == options.end());   // would create a cycle
    CHECK(std::find(options.begin(), options.end(), "turret") == options.end());

    auto shooterGroups = propertyGroupsFor(c, cat, SelectionRef::mechanism("shooter"));
    std::vector<std::string> links;
    const auto shooterPlacement = std::find_if(shooterGroups.begin(), shooterGroups.end(), [](const auto& g) { return g.id == "mech.placement"; });
    REQUIRE(shooterPlacement != shooterGroups.end());
    for (const auto& o : shooterPlacement->schema.find("parentLink")->options) links.push_back(o.value);
    CHECK(links == std::vector<std::string>{"root", "mover"});   // parent is the turret
}

TEST_CASE("property groups: a mechanism whose parent no longer exists still shows it") {
    const auto cat = catalog();
    auto c = robot();
    for (auto& m : c.mechanisms)
        if (m.id == "shooter") m.attachTo("ghost");
    const auto groups = propertyGroupsFor(c, cat, SelectionRef::mechanism("shooter"));
    const auto placement = std::find_if(groups.begin(), groups.end(), [](const auto& g) { return g.id == "mech.placement"; });
    REQUIRE(placement != groups.end());
    const auto& options = placement->schema.find("parent")->options;
    const auto ghost = std::find_if(options.begin(), options.end(), [](const auto& o) { return o.value == "ghost"; });
    REQUIRE(ghost != options.end());
    CHECK(ghost->label == "ghost (missing)");   // as for zones and cameras, instead of a blank combo box
}

TEST_CASE("property edits write through to the right place") {
    const auto cat = catalog();
    auto c = robot();
    const auto sel = SelectionRef::mechanism("turret");
    CHECK(applyProperty(c, cat, sel, "mech.props", "gearRatio", sdk::Value(55.0)));
    CHECK(c.findMechanism("turret")->properties.find("gearRatio")->asDouble() == 55.0);
    CHECK(applyProperty(c, cat, sel, "mech.general", "label", sdk::Value("Yaw")));
    CHECK(c.findMechanism("turret")->label == "Yaw");
    CHECK_FALSE(applyProperty(c, cat, sel, "mech.general", "id", sdk::Value("yaw")));   // the id is internal, not editable
    CHECK(c.findMechanism("turret") != nullptr);

    sdk::Pose3 mount;
    mount.position = {0.1f, 0.2f, 0.3f};
    CHECK(applyProperty(c, cat, SelectionRef::mechanism("turret"), "mech.placement", "mount", sdk::toValue(mount)));
    CHECK(approx(c.findMechanism("turret")->mount.position, {0.1f, 0.2f, 0.3f}));

    CHECK(applyProperty(c, cat, SelectionRef::plugin(), "plugin", "preloadFuel", sdk::Value(12)));
    CHECK(c.findPlugin("demo_game")->data.find("preloadFuel")->asInt() == 12);
    CHECK(applyProperty(c, cat, SelectionRef::plugin(), "panel:demo_game.hopper", "capacity", sdk::Value(55)));
    CHECK(c.findPlugin("demo_game")->data.find("capacity")->asInt() == 55);

    CHECK(applyProperty(c, cat, SelectionRef::wheel("fl"), "wheel", "radius", sdk::Value(0.05)));
    CHECK(approx(ops::findWheel(c, "fl")->radiusM, 0.05f));
    CHECK(ops::findWheel(c, "fl")->steered);
    CHECK(ops::setMapping(c, {"drivetrain", "fl.steerEncoder"},
                          {"phoenix6", "cancoder", 12, "rio", -1}));
    CHECK(ops::wheelOfSlot("fl.steerEncoder") == "fl");
    CHECK(applyProperty(c, cat, SelectionRef::wheel("fl"), "wheel", "driveMotorCount",
                        sdk::Value(2)));
    CHECK(ops::findMapping(c, {"drivetrain", "fl.steerEncoder"}) != nullptr);
    CHECK(applyProperty(c, cat, SelectionRef::wheel("fl"), "wheel", "kind", sdk::Value("standard")));
    CHECK_FALSE(ops::findWheel(c, "fl")->steered);   // only a swerve module steers
    CHECK(ops::findMapping(c, {"drivetrain", "fl.steer"}) == nullptr);   // the steer mapping goes away with the steering
    CHECK(ops::findMapping(c, {"drivetrain", "fl.steerEncoder"}) == nullptr);
    CHECK(applyProperty(c, cat, SelectionRef::wheel("fl"), "wheel", "kind", sdk::Value("swerve")));
    CHECK(ops::findWheel(c, "fl")->steered);
    CHECK(applyProperty(c, cat, SelectionRef::drivetrain(), "drivetrain", "kind", sdk::Value("tank")));
    CHECK(c.drivetrain.spec.kind == "tank");
    CHECK(applyProperty(c, cat, SelectionRef::robot(), "robot", "name", sdk::Value("Bot")));
    CHECK(c.name == "Bot");
    CHECK(applyProperty(c, cat, SelectionRef::robot(), "robot", "startX", sdk::Value(4.51)));
    CHECK(applyProperty(c, cat, SelectionRef::robot(), "robot", "startY", sdk::Value(0.52654)));
    CHECK(applyProperty(c, cat, SelectionRef::robot(), "robot", "startHeading", sdk::Value(1.5)));
    REQUIRE(c.startPose);
    CHECK(c.startPose->xM == doctest::Approx(4.51f));
    CHECK(c.startPose->yM == doctest::Approx(0.52654f));
    CHECK(c.startPose->headingRad == doctest::Approx(1.5f));
    CHECK(applyProperty(c, cat, SelectionRef::robot(), "robot", "customStartPose",
                        sdk::Value(false)));
    CHECK_FALSE(c.startPose);
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

namespace {
std::vector<std::string> motorSlotNames(const sdk::RobotConfig& c, const EditorCatalog& cat, const std::string& owner, const std::string& prefix = {}) {
    std::vector<std::string> names;
    for (const auto& s : ops::slotSpecsFor(c, cat, owner))
        if (s.type == sdk::SlotType::Motor && s.name.rfind(prefix, 0) == 0) names.push_back(s.name);
    return names;
}
}  // namespace

TEST_CASE("motors: slot rows follow the motor count and the motors that disappear are unmapped") {
    const auto cat = catalog();
    auto c = robot();
    CHECK(motorSlotNames(c, cat, "climber") == std::vector<std::string>{"motor"});
    REQUIRE(ops::setMotorCount(c, cat, "climber", "motorCount", 3));
    CHECK(motorSlotNames(c, cat, "climber") == std::vector<std::string>{"motor", "motor2", "motor3"});
    const auto groups = ops::motorGroups(c, cat, "climber");
    REQUIRE(groups.size() == 1);
    CHECK(groups[0].count == 3);
    CHECK(groups[0].maxCount == 3);

    ops::setMapping(c, {"climber", "motor3"}, {"phoenix6", "talonfx", 32, "rio", -1});
    CHECK(ops::setMotorCount(c, cat, "climber", "motorCount", 2));
    CHECK(ops::findMapping(c, {"climber", "motor3"}) == nullptr);
    CHECK_FALSE(ops::setMotorCount(c, cat, "climber", "motorCount", 2));   // unchanged
    CHECK(ops::setMotorCount(c, cat, "climber", "motorCount", 99));        // clamped to the template's slots
    CHECK(ops::motorGroups(c, cat, "climber")[0].count == 3);

    // Editing the property directly prunes as well.
    ops::setMapping(c, {"climber", "motor2"}, {"phoenix6", "talonfx", 31, "rio", -1});
    REQUIRE(applyProperty(c, cat, SelectionRef::mechanism("climber"), "mech.props", "motorCount", sdk::Value(1)));
    CHECK(ops::findMapping(c, {"climber", "motor2"}) == nullptr);
}

TEST_CASE("wheel motor dropdowns offer every motor of the SDK table and apply its numbers") {
    auto c = robot();
    const sdk::WheelSpec* fl = ops::findWheel(c, "fl");
    REQUIRE(fl != nullptr);
    const sdk::PropertySchema schema = spec::wheelSchema(*fl);
    for (const char* key : {"driveMotor", "steerMotor"}) {
        INFO(key);
        const auto it = std::find_if(schema.properties.begin(), schema.properties.end(),
                                     [&](const sdk::PropertySpec& p) { return p.key == key; });
        REQUIRE(it != schema.properties.end());
        const auto& table = sdk::motors::presets();
        REQUIRE(it->options.size() == table.size());   // the sample wheel uses a table motor: no "(custom)" entry
        for (std::size_t i = 0; i < table.size(); ++i) {
            CHECK(it->options[i].value == table[i].spec.name);
            CHECK(it->options[i].label == table[i].label);
        }
    }

    spec::applyWheel(c, "fl", "steerMotor", sdk::Value("Kraken X44 (FOC)"));
    const sdk::MotorPreset* x44 = sdk::motors::findPreset("kraken_x44_foc");
    REQUIRE(x44 != nullptr);
    const sdk::WheelSpec* after = ops::findWheel(c, "fl");
    REQUIRE(after != nullptr);
    CHECK(after->steerMotor.name == x44->spec.name);
    CHECK(after->steerMotor.stallTorqueNm == x44->spec.stallTorqueNm);
    CHECK(after->steerMotor.freeSpeedRadPerSec == x44->spec.freeSpeedRadPerSec);
}

TEST_CASE("motors: a wheel gets one drive row per motor and the reversed flag survives re-mapping") {
    const auto cat = catalog();
    auto c = robot();
    CHECK(motorSlotNames(c, cat, "drivetrain", "fl.") == std::vector<std::string>{"fl.drive", "fl.steer"});
    spec::applyWheel(c, "fl", "driveMotorCount", sdk::Value(3));
    CHECK(motorSlotNames(c, cat, "drivetrain", "fl.") ==
          std::vector<std::string>{"fl.drive", "fl.drive2", "fl.drive3", "fl.steer"});
    const auto groups = ops::motorGroups(c, cat, "drivetrain");
    CHECK(std::any_of(groups.begin(), groups.end(), [](const ops::MotorGroup& g) { return g.countKey == "fl.driveMotorCount" && g.count == 3 && g.maxCount == 8; }));

    const sdk::SlotAddress second{"drivetrain", "fl.drive2"};
    ops::setMapping(c, second, {"phoenix6", "talonfx", 30, "rio", -1});
    CHECK(ops::setMappingReversed(c, second, true));
    ops::setMapping(c, second, {"phoenix6", "talonfx", 31, "rio", -1});   // swap the controller
    CHECK(ops::findMapping(c, second)->reversed);
    CHECK_FALSE(ops::setMappingReversed(c, {"drivetrain", "fl.drive3"}, true));   // unmapped

    CHECK(ops::renameWheel(c, "fl", "front"));
    CHECK(ops::findMapping(c, {"drivetrain", "front.drive2"}) != nullptr);
    CHECK(ops::wheelOfSlot("front.drive12") == "front");
    CHECK(ops::wheelOfSlot("front.driver").empty());

    spec::applyWheel(c, "front", "driveMotorCount", sdk::Value(1));
    CHECK(ops::findMapping(c, {"drivetrain", "front.drive2"}) == nullptr);
    CHECK(ops::findMapping(c, {"drivetrain", "front.drive"}) != nullptr);   // the first motor stays
    CHECK(ops::findMapping(c, {"drivetrain", "front.steer"}) != nullptr);
}

TEST_CASE("motors: slot status shows reversed motors and which unmapped motors mirror motor 1") {
    const auto cat = catalog();
    auto c = robot();
    ops::setMotorCount(c, cat, "climber", "motorCount", 2);
    ops::setMapping(c, {"climber", "motor"}, {"phoenix6", "talonfx", 5, "rio", -1});
    ops::setMappingReversed(c, {"climber", "motor"}, true);
    const auto infos = slotInfos(c, cat, directoryOf({}), "climber");
    REQUIRE(infos.size() >= 2);
    CHECK(infos[0].reversed);
    CHECK(mappedLabel(infos[0]).contains("reversed"));
    CHECK(infos[1].spec.name == "motor2");
    CHECK(infos[1].state == SlotState::OptionalEmpty);
    CHECK(infos[1].detail.contains("mirrors"));
}