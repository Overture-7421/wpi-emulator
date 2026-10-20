#include <doctest/doctest.h>

#include <algorithm>
#include <limits>
#include "runtime_fixtures.h"
#include "wpiemu/core/config/config_pipeline.h"
#include "wpiemu/core/config/config_validator.h"
#include "wpiemu/core/config/json_codec.h"

using namespace wpiemu::sdk;
using namespace wpiemu::core::config;
using wpiemu::test_runtime::Registries;
using wpiemu::test_runtime::sampleConfig;

namespace {

bool hasIssue(const std::vector<Issue>& issues, const char* code, Severity severity = Severity::Error) {
    return std::any_of(issues.begin(), issues.end(), [&](const Issue& i) { return i.code == code && i.severity == severity; });
}

std::vector<Issue> validate(const RobotConfig& c, Registries& r, const ConfigValidator& v = ConfigValidator{}) {
    if (IGameModule* p = selectPlugin(c, r.plugins)) REQUIRE(r.registrar.ensureRegistered(*p).ok());
    return v.validate(c, r.mechanisms, r.drivetrains, r.plugins);
}

}  // namespace

TEST_CASE("validator: the sample config is clean") {
    Registries reg;
    const auto issues = validate(sampleConfig(), reg);
    for (const Issue& i : issues) MESSAGE(i.path << ": " << i.message << " [" << i.code << "]");
    CHECK_FALSE(ConfigValidator::hasErrors(issues));
}

TEST_CASE("validator: chassis, wheel, sim and battery sanity") {
    Registries reg;
    RobotConfig c = sampleConfig();
    c.chassis.massKg = 0.f;
    c.chassis.sizeM.x = -1.f;
    c.drivetrain.spec.wheels[0].radiusM = 0.f;
    c.drivetrain.spec.wheels[1].name = "fl";   // duplicate
    c.drivetrain.spec.wheels[2].name = "a.b";  // dot would break slot addresses
    c.sim.stepUs = 7000;                       // does not divide 20 ms
    c.battery.brownoutVoltageV = 20.0;
    c.startPose = RobotStartPose{0.f, 0.f, std::numeric_limits<float>::infinity()};
    const auto issues = validate(c, reg);
    CHECK(hasIssue(issues, issue_code::kChassisInvalid));
    CHECK(hasIssue(issues, issue_code::kWheelInvalid));
    CHECK(hasIssue(issues, issue_code::kWheelDuplicate));
    CHECK(hasIssue(issues, issue_code::kSimInvalid));
    CHECK(hasIssue(issues, issue_code::kBatteryInvalid));
    CHECK(hasIssue(issues, issue_code::kStartPoseInvalid));
}

TEST_CASE("validator: start positions need unique ids and finite poses") {
    Registries reg;
    RobotConfig c = sampleConfig();
    c.startPositions = {{"a", "A", Alliance::Blue, {1.f, 2.f, 0.f}}};
    CHECK_FALSE(hasIssue(validate(c, reg), issue_code::kStartPositionIdDuplicate));
    c.startPositions.push_back({"a", "A2", Alliance::Red, {1.f, 2.f, 0.f}});
    c.startPositions.push_back({"", "no id", Alliance::Red, {1.f, 2.f, 0.f}});
    c.startPositions.push_back({"nan", "nan", Alliance::Red, {std::numeric_limits<float>::infinity(), 2.f, 0.f}});
    const auto issues = validate(c, reg);
    CHECK(hasIssue(issues, issue_code::kStartPositionIdDuplicate));
    CHECK(hasIssue(issues, issue_code::kStartPositionIdInvalid));
    CHECK(hasIssue(issues, issue_code::kStartPositionInvalid));
}

TEST_CASE("validator: tire and motor parameters must be finite and physically meaningful") {
    Registries reg;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const double dnan = std::numeric_limits<double>::quiet_NaN();
    auto hasPath = [](const std::vector<Issue>& issues, const char* code, const std::string& path) {
        return std::any_of(issues.begin(), issues.end(), [&](const Issue& i) { return i.code == code && i.path == path; });
    };

    {
        RobotConfig c = sampleConfig();
        c.drivetrain.spec.wheels[0].tire.peakSlipLong = 0.f;
        c.drivetrain.spec.wheels[0].tire.peakSlipLat = -1.f;
        c.drivetrain.spec.wheels[0].tire.vRefMps = 0.f;
        c.drivetrain.spec.wheels[0].tire.lateralScale = nan;
        c.drivetrain.spec.wheels[0].tire.slideRatio = std::numeric_limits<float>::infinity();
        const auto issues = validate(c, reg);
        for (const char* field : {"peakSlipLong", "peakSlipLat", "vRefMps", "lateralScale", "slideRatio"})
            CHECK(hasPath(issues, issue_code::kTireInvalid, std::string("drivetrain.wheels[0].tire.") + field));
    }
    {
        RobotConfig c = sampleConfig();
        MotorSpec& m = c.drivetrain.spec.wheels[0].driveMotor;
        m.nominalVoltageV = 0.0;
        m.stallTorqueNm = -1.0;
        m.stallCurrentA = dnan;
        m.freeSpeedRadPerSec = 0.0;
        m.freeCurrentA = -0.5;
        const auto issues = validate(c, reg);
        for (const char* field : {"nominalVoltageV", "stallTorqueNm", "stallCurrentA", "freeSpeedRadPerSec", "freeCurrentA"})
            CHECK(hasPath(issues, issue_code::kMotorInvalid, std::string("drivetrain.wheels[0].driveMotor.") + field));
    }
    {
        RobotConfig c = sampleConfig();
        c.drivetrain.spec.wheels[0].driveMotor.freeCurrentA = c.drivetrain.spec.wheels[0].driveMotor.stallCurrentA;  // kv would divide by zero
        CHECK(hasIssue(validate(c, reg), issue_code::kMotorInvalid));
    }
    {
        RobotConfig c = sampleConfig();
        WheelSpec& w = c.drivetrain.spec.wheels[0];
        w.steered = true;
        w.steerGearRatio = 0.f;
        w.steerMotor.stallTorqueNm = 0.0;
        const auto issues = validate(c, reg);
        CHECK(hasIssue(issues, issue_code::kWheelInvalid));
        CHECK(hasIssue(issues, issue_code::kMotorInvalid));

        WheelSpec& plain = c.drivetrain.spec.wheels[1];  // a bad steer motor on a wheel that never steers is irrelevant
        plain.steerGearRatio = 0.f;
        plain.steerMotor.stallTorqueNm = 0.0;
        CHECK_FALSE(hasPath(validate(c, reg), issue_code::kMotorInvalid, "drivetrain.wheels[1].steerMotor.stallTorqueNm"));
    }
}

TEST_CASE("validator: drivetrain kind and tire model must be registered") {
    Registries reg;
    RobotConfig c = sampleConfig();
    c.drivetrain.spec.kind = "hover";
    c.drivetrain.spec.tireModel = "slick";
    const ConfigValidator withTires(&reg.tires, nullptr);
    const auto issues = validate(c, reg, withTires);
    CHECK(hasIssue(issues, issue_code::kDrivetrainKindUnknown));
    CHECK(hasIssue(issues, issue_code::kTireModelUnknown));
    CHECK_FALSE(hasIssue(validate(c, reg), issue_code::kTireModelUnknown));  // not checked without a registry

    c.drivetrain.spec.wheels.clear();
    CHECK(hasIssue(validate(c, reg), issue_code::kDrivetrainNoWheels, Severity::Warning));
}

TEST_CASE("validator: mechanism ids, types and properties") {
    Registries reg;
    RobotConfig c = sampleConfig();
    MechanismConfig dup = c.mechanisms[0];
    c.mechanisms.push_back(dup);                       // duplicate id
    MechanismConfig empty;
    empty.type = "roller";
    c.mechanisms.push_back(empty);                     // empty id
    MechanismConfig reserved;
    reserved.id = "chassis";
    reserved.type = "roller";
    c.mechanisms.push_back(reserved);
    MechanismConfig unknown;
    unknown.id = "mystery";
    unknown.type = "teleporter";
    c.mechanisms.push_back(unknown);
    MechanismConfig badProps;
    badProps.id = "badprops";
    badProps.type = "roller";
    badProps.properties["gearRatio"] = 5000.0;         // out of range
    badProps.properties["colour"] = "red";             // unknown key
    badProps.mappings.push_back({{"badprops", "motor"}, DeviceKey{"phoenix6", "talonfx", 50, "rio", -1}});
    c.mechanisms.push_back(badProps);
    MechanismConfig dotted;
    dotted.id = "a.b";
    dotted.type = "roller";
    c.mechanisms.push_back(dotted);

    const auto issues = validate(c, reg);
    CHECK(hasIssue(issues, issue_code::kMechanismIdDuplicate));
    CHECK(hasIssue(issues, issue_code::kMechanismIdInvalid));
    CHECK(hasIssue(issues, validation_code::kMechanismIdReserved));  // from validateMechanismAttachments
    CHECK(hasIssue(issues, issue_code::kMechanismTypeUnknown));
    CHECK(hasIssue(issues, issue_code::kPropertyInvalid));
    CHECK(hasIssue(issues, issue_code::kPropertyInvalid, Severity::Warning));
}

TEST_CASE("validator: attachments are validated through the SDK contract") {
    Registries reg;
    RobotConfig c = sampleConfig();
    MechanismConfig a;
    a.id = "a";
    a.type = "roller";
    a.mappings.push_back({{"a", "motor"}, DeviceKey{"hal", "pwm", 1, "rio", -1}});
    a.attachTo("ghost");
    c.mechanisms.push_back(a);
    MechanismConfig b = a;
    b.id = "b";
    b.mappings = {{{"b", "motor"}, DeviceKey{"hal", "pwm", 2, "rio", -1}}};
    b.attachTo("b");  // self parent: cycle
    c.mechanisms.push_back(b);
    MechanismConfig d = a;
    d.id = "d";
    d.mappings = {{{"d", "motor"}, DeviceKey{"hal", "pwm", 3, "rio", -1}}};
    d.attachTo("intake", "nolink");
    c.mechanisms.push_back(d);
    const auto issues = validate(c, reg);
    CHECK(hasIssue(issues, validation_code::kAttachmentParentMissing));
    CHECK(hasIssue(issues, validation_code::kAttachmentCycle));
    CHECK(hasIssue(issues, validation_code::kAttachmentLinkMissing));
}

TEST_CASE("validator: slots - unknown, owner mismatch, required, duplicates") {
    Registries reg;
    RobotConfig c = sampleConfig();
    c.mechanisms[0].mappings.push_back({{"intake", "sideways"}, DeviceKey{"hal", "pwm", 3, "rio", -1}});  // unknown slot
    c.mechanisms[0].mappings.push_back({{"other", "motor"}, DeviceKey{"hal", "pwm", 4, "rio", -1}});      // wrong owner
    c.drivetrain.mappings.push_back({{slot_owner::kDrivetrain, "zz.drive"}, DeviceKey{"hal", "pwm", 5, "rio", -1}});
    c.robotMappings.push_back({{slot_owner::kRobot, "gizmo"}, DeviceKey{"hal", "dio", 1, "rio", -1}});  // robot: only a warning
    auto issues = validate(c, reg);
    CHECK(hasIssue(issues, issue_code::kSlotUnknown));
    CHECK(hasIssue(issues, issue_code::kSlotOwnerMismatch));
    CHECK(hasIssue(issues, issue_code::kSlotUnknown, Severity::Warning));

    RobotConfig unmapped = sampleConfig();
    unmapped.mechanisms[0].mappings.clear();
    issues = validate(unmapped, reg);
    CHECK(hasIssue(issues, issue_code::kSlotRequiredUnmapped));      // roller.motor is required
    CHECK_FALSE(std::any_of(issues.begin(), issues.end(), [](const Issue& i) { return i.path.find("encoder") != std::string::npos; }));  // optional slot: silent

    RobotConfig noDrive = sampleConfig();
    noDrive.drivetrain.mappings.clear();
    CHECK(hasIssue(validate(noDrive, reg), issue_code::kSlotRequiredUnmapped, Severity::Warning));  // drivetrain: warning only

    RobotConfig dup = sampleConfig();
    dup.mechanisms[0].mappings.push_back({{"intake", "motor"}, DeviceKey{"hal", "pwm", 9, "rio", -1}});  // same slot twice
    dup.mechanisms[0].mappings.push_back({{"intake", "encoder"}, dup.mechanisms[0].mappings[0].device});   // same device twice
    issues = validate(dup, reg);
    CHECK(hasIssue(issues, issue_code::kSlotDuplicateMapping));
    CHECK(hasIssue(issues, issue_code::kDeviceDuplicateMapping));
}

TEST_CASE("validator: drivetrain slots come from the registered kind, so a tank declares no steer slots") {
    Registries reg;
    REQUIRE(reg.drivetrains
                .registerDrivetrain("tank",
                                    {"Tank", "", [](const DrivetrainSpec& spec) { return tractionSlotSpecs(spec, false); }},
                                    [](const DrivetrainBuildArgs& a) { return std::make_unique<wpiemu::test_runtime::TestDrivetrain>(a); })
                .ok());
    RobotConfig c = sampleConfig();
    c.drivetrain.spec.wheels[0].steered = true;
    c.drivetrain.mappings.clear();
    const std::string wheelName = c.drivetrain.spec.wheels[0].name;
    c.drivetrain.mappings.push_back({{slot_owner::kDrivetrain, steerSlotName(wheelName)}, DeviceKey{"hal", "pwm", 5, "rio", -1}});

    c.drivetrain.spec.kind = "tank";   // steered=true is ignored: there is no such slot
    CHECK(hasIssue(validate(c, reg), issue_code::kSlotUnknown, Severity::Error));
    c.drivetrain.spec.kind = "test";   // a kind without a callback follows the traction convention, which steers flagged wheels
    CHECK_FALSE(hasIssue(validate(c, reg), issue_code::kSlotUnknown));
    c.drivetrain.spec.kind = "hover";  // an unknown kind is reported once and its mappings still validate by convention
    const auto issues = validate(c, reg);
    CHECK(hasIssue(issues, issue_code::kDrivetrainKindUnknown));
    CHECK_FALSE(hasIssue(issues, issue_code::kSlotUnknown));
}

TEST_CASE("validator: motor counts, inactive motor slots and the reversed flag") {
    Registries reg;
    RobotConfig c = sampleConfig();
    c.mechanisms[0].mappings.push_back({{"intake", "motor2"}, DeviceKey{"hal", "pwm", 3, "rio", -1}, true});   // motor count is 1
    auto issues = validate(c, reg);
    CHECK(hasIssue(issues, issue_code::kSlotMotorInactive, Severity::Warning));
    c.mechanisms[0].properties["motorCount"] = 2;
    CHECK_FALSE(hasIssue(validate(c, reg), issue_code::kSlotMotorInactive));

    c.mechanisms[0].mappings.push_back({{"intake", "encoder"}, DeviceKey{"phoenix6", "cancoder", 12, "rio", -1}, true});
    CHECK(hasIssue(validate(c, reg), issue_code::kSlotReversedInvalid, Severity::Warning));   // only motors can be reversed

    RobotConfig wheels = sampleConfig();
    wheels.drivetrain.spec.wheels[0].driveMotorCount = 9;
    CHECK(hasIssue(validate(wheels, reg), issue_code::kWheelInvalid, Severity::Error));
    wheels.drivetrain.spec.wheels[0].driveMotorCount = 3;
    wheels.drivetrain.mappings.push_back({{slot_owner::kDrivetrain, driveSlotName(wheels.drivetrain.spec.wheels[0].name, 2)},
                                          DeviceKey{"hal", "pwm", 6, "rio", -1}});
    CHECK_FALSE(hasIssue(validate(wheels, reg), issue_code::kSlotUnknown));   // drive3 exists now
    wheels.drivetrain.spec.wheels[0].driveMotorCount = 2;
    CHECK(hasIssue(validate(wheels, reg), issue_code::kSlotUnknown));         // drive3 is gone
}
TEST_CASE("validator: aliases of one physical device mapped independently are rejected") {
    Registries reg;
    RobotConfig c = sampleConfig();
    c.mechanisms[0].mappings.push_back({{"intake", "encoder"}, DeviceKey{"phoenix6", "talonfx", 10, "canivore", -1}});
    const auto issues = validate(c, reg);
    CHECK(hasIssue(issues, validation_code::kDeviceAliasConflict));
}

TEST_CASE("validator: slot/device capability check uses the injected catalog and skips unknown devices") {
    Registries reg;
    RobotConfig c = sampleConfig();
    c.mechanisms[0].mappings.push_back({{"intake", "encoder"}, DeviceKey{"hal", "pwm", 3, "rio", -1}});
    const ConfigValidator::DeviceCapabilityLookup catalog = [](const DeviceKey& k) -> std::optional<CapabilitySet> {
        if (k.type == "pwm") return CapabilitySet(Capability::PwmOutput);  // cannot serve a PositionSensor slot
        if (k.type == "talonfx") return Capability::MotorOutput | Capability::PositionSensor;
        return std::nullopt;
    };
    CHECK(hasIssue(validate(c, reg, ConfigValidator(nullptr, catalog)), issue_code::kSlotDeviceIncompatible));
    CHECK_FALSE(hasIssue(validate(c, reg), issue_code::kSlotDeviceIncompatible));  // no catalog: skipped
    RobotConfig ok = sampleConfig();
    CHECK_FALSE(hasIssue(validate(ok, reg, ConfigValidator(nullptr, catalog)), issue_code::kSlotDeviceIncompatible));
}

TEST_CASE("validator: plugin sections - unknown, unsupported version, invalid data") {
    Registries reg;
    RobotConfig c = sampleConfig();
    c.gamePluginId = "fake_game";
    c.plugins["fake_game"] = PluginSection{1, Value::makeObject()};
    c.plugins["fake_game"].data["capacity"] = 1000;  // schema max 100
    c.plugins["ghost_game"] = PluginSection{1, Value::makeObject()};
    auto issues = validate(c, reg);
    CHECK(hasIssue(issues, issue_code::kPluginPropertyInvalid));
    CHECK(hasIssue(issues, issue_code::kPluginUnknownSection, Severity::Warning));

    for (const int version : {0, 2}) {
        c.plugins["fake_game"].schemaVersion = version;
        CHECK(hasIssue(validate(c, reg), issue_code::kPluginSectionVersion));
    }

    RobotConfig missing = sampleConfig();
    missing.gamePluginId = "nope";
    CHECK(hasIssue(validate(missing, reg), issue_code::kPluginNotFound, Severity::Warning));
}

namespace {
ZoneConfig zone(const std::string& id) {
    ZoneConfig z;
    z.id = id;
    z.kind = "fake_game.fake_zone";
    return z;
}
}  // namespace

TEST_CASE("validator: a well-formed zone is clean") {
    Registries reg;
    RobotConfig c = sampleConfig();
    ZoneConfig z = zone("pickup");
    z.attachment.parentMechanismId = "intake";
    z.attachment.parentLink = "tip";
    z.properties["capacity"] = 10;
    z.activation.conditions.push_back({"intake", "position", ConditionOp::Above, 1.0, 0.1, true});
    c.zones.push_back(z);
    const auto issues = validate(c, reg);
    for (const Issue& i : issues) MESSAGE(i.path << ": " << i.message << " [" << i.code << "]");
    CHECK(issues.empty());
}

TEST_CASE("validator: zone ids, attachments, kinds and geometry") {
    Registries reg;
    RobotConfig c = sampleConfig();
    c.zones.push_back(zone(""));
    c.zones.push_back(zone("dup"));
    c.zones.push_back(zone("dup"));
    c.zones.push_back(zone("intake"));  // clashes with a mechanism id
    ZoneConfig noParent = zone("noParent");
    noParent.attachment.parentMechanismId = "ghost";
    c.zones.push_back(noParent);
    ZoneConfig noLink = zone("noLink");
    noLink.attachment.parentMechanismId = "intake";
    noLink.attachment.parentLink = "nolink";
    c.zones.push_back(noLink);
    ZoneConfig foreign = zone("foreign");
    foreign.kind = "other_game.pickup";
    c.zones.push_back(foreign);
    ZoneConfig flat = zone("flat");
    flat.sizeM.y = 0.f;
    c.zones.push_back(flat);
    ZoneConfig nan = zone("nan");
    nan.local.position.x = std::numeric_limits<float>::quiet_NaN();
    c.zones.push_back(nan);

    const auto issues = validate(c, reg);
    const auto count = [&](const char* code, const std::string& pathPart) {
        return std::count_if(issues.begin(), issues.end(), [&](const Issue& i) {
            return i.code == code && i.severity == Severity::Error && i.path.find(pathPart) != std::string::npos;
        });
    };
    CHECK(count(issue_code::kZoneIdInvalid, "zones[0].id") == 1);
    CHECK(count(issue_code::kZoneIdDuplicate, "zones[2].id") == 1);
    CHECK(count(issue_code::kZoneIdDuplicate, "zones[3].id") == 1);
    CHECK(count(validation_code::kAttachmentParentMissing, "zones[4].attachment") == 1);
    CHECK(count(validation_code::kAttachmentLinkMissing, "zones[5].attachment") == 1);
    CHECK(count(issue_code::kZoneKindNotOwned, "zones[6].kind") == 1);
    CHECK(count(issue_code::kZoneGeometryInvalid, "zones[7].sizeM") == 1);
    CHECK(count(issue_code::kZoneGeometryInvalid, "zones[8].local") == 1);
}

namespace {
CameraConfig camera(const std::string& id, const std::string& name = "") {
    CameraConfig c;
    c.id = id;
    c.name = name.empty() ? id : name;
    return c;
}
}  // namespace

TEST_CASE("validator: a well-formed camera is clean") {
    Registries reg;
    RobotConfig c = sampleConfig();
    CameraConfig cam = camera("front");
    cam.attachment.parentMechanismId = "intake";
    cam.attachment.parentLink = "tip";
    c.cameras.push_back(cam);
    c.cameras.push_back(camera(""));
    c.cameras.back().id = "rear";
    c.cameras.back().name = "";
    const auto issues = validate(c, reg);
    for (const Issue& i : issues) MESSAGE(i.path << ": " << i.message << " [" << i.code << "]");
    CHECK(issues.empty());
}

TEST_CASE("validator: camera ids, names, attachments, geometry and optics") {
    Registries reg;
    RobotConfig c = sampleConfig();
    c.zones.push_back(zone("pickup"));
    c.cameras.push_back(camera(""));                  // 0
    c.cameras.push_back(camera("dup"));               // 1
    c.cameras.push_back(camera("dup", "other"));      // 2
    c.cameras.push_back(camera("intake"));            // 3: mechanism id
    c.cameras.push_back(camera("pickup"));            // 4: zone id
    c.cameras.push_back(camera("a", "same"));         // 5
    c.cameras.push_back(camera("b", "same"));         // 6: same name, same kind
    c.cameras.push_back(camera("c", "same"));         // 7: same name, other kind
    c.cameras.back().kind = CameraKind::PhotonVision;
    CameraConfig noParent = camera("noParent");       // 8
    noParent.attachment.parentMechanismId = "ghost";
    c.cameras.push_back(noParent);
    CameraConfig noLink = camera("noLink");           // 9
    noLink.attachment.parentMechanismId = "intake";
    noLink.attachment.parentLink = "nolink";
    c.cameras.push_back(noLink);
    CameraConfig nan = camera("nan");                 // 10
    nan.local.position.x = std::numeric_limits<float>::quiet_NaN();
    c.cameras.push_back(nan);
    CameraConfig bad = camera("bad");                 // 11
    bad.hFovDeg = 180.0;
    bad.vFovDeg = 0.0;
    bad.maxRangeM = 0.0;
    bad.maxIncidenceDeg = 91.0;
    bad.fps = 241.0;
    bad.latencyMs = 2001.0;
    bad.widthPx = 0;
    bad.heightPx = -1;
    bad.positionNoiseM = -0.1;
    bad.yawNoiseDeg = std::numeric_limits<double>::quiet_NaN();
    c.cameras.push_back(bad);
    CameraConfig edge = camera("edge");               // 12: boundary values are valid
    edge.maxIncidenceDeg = 90.0;
    edge.fps = 240.0;
    edge.latencyMs = 0.0;
    c.cameras.push_back(edge);

    const auto issues = validate(c, reg);
    const auto count = [&](const char* code, const std::string& path) {
        return std::count_if(issues.begin(), issues.end(), [&](const Issue& i) {
            return i.code == code && i.severity == Severity::Error && i.path == path;
        });
    };
    CHECK(count(issue_code::kCameraIdInvalid, "cameras[0].id") == 1);
    CHECK(count(issue_code::kCameraIdDuplicate, "cameras[2].id") == 1);
    CHECK(count(issue_code::kCameraIdDuplicate, "cameras[3].id") == 1);
    CHECK(count(issue_code::kCameraIdDuplicate, "cameras[4].id") == 1);
    CHECK(count(issue_code::kZoneIdDuplicate, "zones[0].id") == 1);  // flagged from the zone side too
    CHECK(count(issue_code::kCameraNameDuplicate, "cameras[6].name") == 1);
    CHECK(count(issue_code::kCameraNameDuplicate, "cameras[7].name") == 0);
    CHECK(count(validation_code::kAttachmentParentMissing, "cameras[8].attachment") == 1);
    CHECK(count(validation_code::kAttachmentLinkMissing, "cameras[9].attachment") == 1);
    CHECK(count(issue_code::kCameraGeometryInvalid, "cameras[10].pose") == 1);
    for (const char* field : {"hFovDeg", "vFovDeg", "maxRangeM", "maxIncidenceDeg", "fps", "latencyMs", "widthPx", "heightPx",
                              "positionNoiseM", "yawNoiseDeg"})
        CHECK_MESSAGE(count(issue_code::kCameraOpticsInvalid, std::string("cameras[11].") + field) == 1, field);
    for (const Issue& i : issues) CHECK_MESSAGE(i.path.find("cameras[12]") == std::string::npos, i.path);
}

TEST_CASE("validator: a zone that shares a camera id is rejected") {
    Registries reg;
    RobotConfig c = sampleConfig();
    c.cameras.push_back(camera("shared"));
    c.zones.push_back(zone("shared"));
    CHECK(hasIssue(validate(c, reg), issue_code::kZoneIdDuplicate));
}

TEST_CASE("validator: zones need a game plugin and are checked against its kind schema") {
    RobotConfig c = sampleConfig();
    c.zones.push_back(zone("pickup"));
    {
        Registries none(false);
        CHECK(hasIssue(validate(c, none), issue_code::kZoneKindNotOwned));
    }
    Registries reg;
    c.zones[0].properties["capacity"] = 1000;  // schema max 100
    CHECK(hasIssue(validate(c, reg), issue_code::kZonePropertyInvalid));
    c.zones[0].properties["capacity"] = 5;
    CHECK_FALSE(ConfigValidator::hasErrors(validate(c, reg)));
    // owned prefix but not declared: warning only, properties cannot be checked
    c.zones[0].kind = "fake_game.mystery";
    CHECK(hasIssue(validate(c, reg), issue_code::kZoneKindUnknown, Severity::Warning));
    CHECK_FALSE(ConfigValidator::hasErrors(validate(c, reg)));
}

TEST_CASE("validator: zone activation conditions") {
    Registries reg;
    RobotConfig c = sampleConfig();
    ZoneConfig z = zone("shooter");
    z.activation.conditions.push_back({"ghost", "velocity", ConditionOp::Above, 1.0, 0.1, true});
    z.activation.conditions.push_back({"intake", "", ConditionOp::Above, 1.0, 0.1, true});
    c.zones.push_back(z);
    const auto issues = validate(c, reg);
    CHECK(hasIssue(issues, issue_code::kZoneConditionMechanismUnknown, Severity::Warning));
    CHECK(hasIssue(issues, issue_code::kZoneConditionInvalid));
    CHECK(std::count_if(issues.begin(), issues.end(), [](const Issue& i) { return i.code == issue_code::kZoneConditionMechanismUnknown; }) == 1);
}

TEST_CASE("validator: schema version must match") {
    Registries reg;
    RobotConfig c = sampleConfig();
    c.schemaVersion = 7;
    CHECK(hasIssue(validate(c, reg), issue_code::kSchemaVersion));
}

// ---- pipeline ---------------------------------------------------------------------------------------------------

namespace {
struct PipelineFixture {
    Registries reg;
    RobotConfigCodec codec;
    ConfigValidator validator;
    ConfigPipeline pipeline{codec, validator, reg.registrar, reg.drivetrains, reg.plugins};

    Value stored(const RobotConfig& c) const { return codec.toValue(c); }
};
}  // namespace

TEST_CASE("pipeline: selects the plugin by year, registers its mechanisms once, applies defaults") {
    PipelineFixture f;
    RobotConfig c = sampleConfig();
    c.plugins.clear();
    MechanismConfig m;
    m.id = "x";
    m.type = "fake_game.fake_mechanism";
    m.mappings.push_back({{"x", "motor"}, DeviceKey{"hal", "pwm", 1, "rio", -1}});
    c.mechanisms.push_back(m);
    const auto loaded = f.pipeline.load(f.stored(c));
    REQUIRE_MESSAGE(loaded.ok(), (loaded.ok() ? "" : loaded.error().message));
    const LoadedConfig& l = loaded.value();
    CHECK(l.pluginId == "fake_game");  // by project.wpilibYear == 2999
    CHECK(f.reg.fakeCounters->registerMechanismsCalls == 1);
    REQUIRE(f.pipeline.load(f.stored(c)).ok());
    CHECK(f.reg.fakeCounters->registerMechanismsCalls == 1);  // idempotent
    CHECK(l.valid());
    // plugin section created with defaults at the current version, mechanism property defaults filled
    REQUIRE(l.config.findPlugin("fake_game") != nullptr);
    CHECK(l.config.findPlugin("fake_game")->schemaVersion == 1);
    CHECK(l.config.findPlugin("fake_game")->data.find("capacity")->asInt() == 3);
    CHECK(l.config.mechanisms[0].properties.find("gearRatio")->asDouble() == 1.0);
}

TEST_CASE("pipeline: a plugin section with another schema version is refused, there is no migration") {
    for (const int version : {0, 2, 5}) {
        PipelineFixture f;
        RobotConfig c = sampleConfig();
        c.plugins["fake_game"] = PluginSection{version, Value::makeObject()};
        const auto r = f.pipeline.load(f.stored(c));
        REQUIRE_FALSE(r.ok());
        CHECK(r.error().code == ErrorCode::VersionMismatch);
    }
}
TEST_CASE("pipeline: no plugin for the year -> core features only, other sections kept") {
    PipelineFixture f;
    RobotConfig c = sampleConfig();
    c.project.wpilibYear = 1999;
    c.plugins["fake_game"] = PluginSection{1, Value::makeObject()};
    const auto r = f.pipeline.load(f.stored(c));
    REQUIRE(r.ok());
    CHECK(r.value().pluginId.empty());
    CHECK(r.value().config.findPlugin("fake_game")->schemaVersion == 1);  // untouched
}

TEST_CASE("pipeline: validation problems come back as issues next to the config") {
    PipelineFixture f;
    RobotConfig c = sampleConfig();
    c.chassis.massKg = -1.f;
    const auto r = f.pipeline.load(f.stored(c));
    REQUIRE(r.ok());
    CHECK_FALSE(r.value().valid());
    CHECK(hasIssue(r.value().issues, issue_code::kChassisInvalid));
}

TEST_CASE("pipeline: parse errors are fatal") {
    PipelineFixture f;
    CHECK_FALSE(f.pipeline.load(Value(1)).ok());
}

TEST_CASE("pipeline: zone property defaults are filled from the plugin's kind schema") {
    PipelineFixture f;
    RobotConfig c = sampleConfig();
    c.zones.push_back(zone("pickup"));
    const auto loaded = f.pipeline.load(f.stored(c));
    REQUIRE_MESSAGE(loaded.ok(), (loaded.ok() ? "" : loaded.error().message));
    CHECK(loaded.value().config.zones[0].properties.find("capacity")->asInt() == 3);
}
