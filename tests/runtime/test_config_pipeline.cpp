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
    const auto issues = validate(c, reg);
    CHECK(hasIssue(issues, issue_code::kChassisInvalid));
    CHECK(hasIssue(issues, issue_code::kWheelInvalid));
    CHECK(hasIssue(issues, issue_code::kWheelDuplicate));
    CHECK(hasIssue(issues, issue_code::kSimInvalid));
    CHECK(hasIssue(issues, issue_code::kBatteryInvalid));
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

TEST_CASE("validator: plugin sections - unknown, newer, outdated, invalid data") {
    Registries reg;
    RobotConfig c = sampleConfig();
    c.gamePluginId = "fake_game";
    c.plugins["fake_game"] = PluginSection{2, Value::makeObject()};
    c.plugins["fake_game"].data["capacity"] = 1000;  // schema max 100
    c.plugins["ghost_game"] = PluginSection{1, Value::makeObject()};
    auto issues = validate(c, reg);
    CHECK(hasIssue(issues, issue_code::kPluginPropertyInvalid));
    CHECK(hasIssue(issues, issue_code::kPluginUnknownSection, Severity::Warning));

    c.plugins["fake_game"].schemaVersion = 3;
    CHECK(hasIssue(validate(c, reg), issue_code::kPluginSectionNewer));
    c.plugins["fake_game"].schemaVersion = 1;
    CHECK(hasIssue(validate(c, reg), issue_code::kPluginSectionOutdated));

    RobotConfig missing = sampleConfig();
    missing.gamePluginId = "nope";
    CHECK(hasIssue(validate(missing, reg), issue_code::kPluginNotFound, Severity::Warning));
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
    CHECK(l.config.findPlugin("fake_game")->schemaVersion == 2);
    CHECK(l.config.findPlugin("fake_game")->data.find("capacity")->asInt() == 3);
    CHECK(l.config.mechanisms[0].properties.find("gearRatio")->asDouble() == 1.0);
    CHECK_FALSE(l.migrated());
}

TEST_CASE("pipeline: plugin migration chain then whole-config hook, in that order") {
    PipelineFixture f;
    RobotConfig c = sampleConfig();
    c.gamePluginId = "fake_game";
    c.plugins["fake_game"] = PluginSection{1, Value::makeObject()};
    c.plugins["fake_game"].data["cap"] = 9;  // v1 name, the migrator renames it
    MechanismConfig legacy;
    legacy.id = "old";
    legacy.type = "fake_game.legacy_mechanism";  // renamed by migrateRobotConfig
    legacy.mappings.push_back({{"old", "motor"}, DeviceKey{"hal", "pwm", 1, "rio", -1}});
    c.mechanisms.push_back(legacy);

    const auto loaded = f.pipeline.load(f.stored(c));
    REQUIRE_MESSAGE(loaded.ok(), (loaded.ok() ? "" : loaded.error().message));
    const LoadedConfig& l = loaded.value();
    CHECK(l.pluginMigrated);
    CHECK(l.migrated());
    CHECK(l.persistedPluginVersion == 1);
    CHECK(f.reg.fakeCounters->migrateRobotConfigCalls == 1);
    CHECK(f.reg.fakeCounters->lastPersistedVersion == 1);
    CHECK(l.config.findPlugin("fake_game")->schemaVersion == 2);
    CHECK(l.config.findPlugin("fake_game")->data.find("capacity")->asInt() == 9);
    CHECK(l.config.findPlugin("fake_game")->data.find("cap") == nullptr);
    CHECK(l.config.findMechanism("old")->type == "fake_game.fake_mechanism");
    CHECK(l.valid());

    // a current section does not run the hook
    PipelineFixture g;
    RobotConfig current = sampleConfig();
    current.plugins["fake_game"] = PluginSection{2, Value::makeObject()};
    const auto again = g.pipeline.load(g.stored(current));
    REQUIRE(again.ok());
    CHECK_FALSE(again.value().pluginMigrated);
    CHECK(g.reg.fakeCounters->migrateRobotConfigCalls == 0);
}

TEST_CASE("pipeline: a newer plugin section is refused before any hook runs") {
    PipelineFixture f;
    RobotConfig c = sampleConfig();
    c.plugins["fake_game"] = PluginSection{5, Value::makeObject()};
    const auto r = f.pipeline.load(f.stored(c));
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().code == ErrorCode::VersionMismatch);
    CHECK(f.reg.fakeCounters->migrateRobotConfigCalls == 0);
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

TEST_CASE("pipeline: a plugin hook that touches data it does not own is rejected") {
    class Greedy final : public testing::FakeGameModule {
    public:
        using FakeGameModule::FakeGameModule;
        Status migrateRobotConfig(RobotConfig& config, int) const override {
            config.battery.nominalVoltageV = 13.0;  // core field: not the plugin's
            return okStatus();
        }
    };
    class Meddler final : public testing::FakeGameModule {
    public:
        using FakeGameModule::FakeGameModule;
        Status migrateRobotConfig(RobotConfig& config, int) const override {
            for (MechanismConfig& m : config.mechanisms) m.label = "hijacked";  // core mechanism: not owned
            return okStatus();
        }
    };
    for (int variant = 0; variant < 2; ++variant) {
        Registries reg(false);
        if (variant == 0) REQUIRE(reg.plugins.add(std::make_unique<Greedy>("fake_game", 2999)).ok());
        else REQUIRE(reg.plugins.add(std::make_unique<Meddler>("fake_game", 2999)).ok());
        RobotConfigCodec codec;
        ConfigValidator validator;
        PluginRegistrar registrar(reg.mechanisms);
        ConfigPipeline pipeline(codec, validator, registrar, reg.drivetrains, reg.plugins);
        RobotConfig c = sampleConfig();
        c.plugins["fake_game"] = PluginSection{1, Value::makeObject()};
        const auto r = pipeline.load(codec.toValue(c));
        REQUIRE_FALSE(r.ok());
        CHECK(r.error().code == ErrorCode::InvalidState);
        CHECK(r.error().message.find("does not own") != std::string::npos);
    }
}

TEST_CASE("pipeline: a failing plugin section migration is reported") {
    class BrokenMigrator final : public IConfigMigrator {
    public:
        int fromVersion() const override { return 1; }
        int toVersion() const override { return 2; }
        Status migrate(Value&) const override { return makeError(ErrorCode::InvalidArgument, "section is garbage"); }
    };
    class BadModule final : public testing::FakeGameModule {
    public:
        using FakeGameModule::FakeGameModule;
        std::vector<std::unique_ptr<IConfigMigrator>> configMigrators() const override {
            std::vector<std::unique_ptr<IConfigMigrator>> m;
            m.push_back(std::make_unique<BrokenMigrator>());
            return m;
        }
    };
    Registries reg(false);
    REQUIRE(reg.plugins.add(std::make_unique<BadModule>("fake_game", 2999)).ok());
    RobotConfigCodec codec;
    ConfigValidator validator;
    PluginRegistrar registrar(reg.mechanisms);
    ConfigPipeline pipeline(codec, validator, registrar, reg.drivetrains, reg.plugins);
    RobotConfig c = sampleConfig();
    c.plugins["fake_game"] = PluginSection{1, Value::makeObject()};
    const auto r = pipeline.load(codec.toValue(c));
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().message.find("section is garbage") != std::string::npos);
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
