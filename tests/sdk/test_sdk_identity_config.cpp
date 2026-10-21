#include <doctest/doctest.h>

#include <algorithm>
#include <memory>
#include <span>
#include <string>
#include <vector>

#include "wpiemu/sdk/config/plugin_config.h"
#include "wpiemu/sdk/config/validation_codes.h"
#include "wpiemu/sdk/devices/device_identity.h"
#include "wpiemu/sdk/game/plugin_registry.h"
#include "wpiemu/sdk/mechanisms/attachment.h"
#include "wpiemu/sdk/testing/fake_device_hub.h"
#include "wpiemu/sdk/testing/fake_game_module.h"
#include "wpiemu/sdk/testing/fake_mechanism_directory.h"
#include "wpiemu/sdk/testing/fake_sim_environment.h"
#include "wpiemu/sdk/version.h"

using namespace wpiemu::sdk;

namespace {

bool hasCode(const std::vector<Issue>& issues, const char* code) {
    return std::any_of(issues.begin(), issues.end(), [&](const Issue& i) { return i.code == code; });
}

bool hasWarning(std::span<const HubWarning> warnings, const char* code) {
    return std::any_of(warnings.begin(), warnings.end(), [&](const HubWarning& w) { return w.code == code; });
}

DeviceDescriptor talon(int id) {
    DeviceDescriptor d;
    d.family = "phoenix6";
    d.type = "talonfx";
    d.vendor = "ctre";
    d.id = id;
    d.bus = kBusUnknown;
    d.capabilities = Capability::MotorOutput | Capability::RotorFeedback;
    return d;
}

// A mechanism whose link poses the test scripts.
class LinkedMechanism final : public IMechanism {
public:
    LinkedMechanism(std::string id, std::string type) : id_(std::move(id)), type_(std::move(type)) {}
    const std::string& id() const override { return id_; }
    std::string_view type() const override { return type_; }
    void tick(SimContext&, double) override {}
    void reset() override {}
    MechanismState state() const override {
        MechanismState s;
        s.id = id_;
        s.type = type_;
        s.links = links;
        return s;
    }
    std::vector<LinkPose> links;

private:
    std::string id_;
    std::string type_;
};

MechanismConfig mechanism(const std::string& id, const std::string& type = "elevator") {
    MechanismConfig m;
    m.id = id;
    m.type = type;
    return m;
}

MechanismFactory factoryWithElevator() {
    MechanismFactory f;
    MechanismTemplateInfo info;
    info.type = "elevator";
    info.links = {LinkSpec{"carriage", "Carriage", Pose3{Vec3{0.f, 0.f, 0.2f}, Quat{}}, ""}};
    REQUIRE(f.registerTemplate(info, [](const MechanismBuildContext& ctx) {
                 return std::make_unique<testing::FakeMechanism>(ctx);
             }).ok());
    MechanismTemplateInfo leaf;
    leaf.type = "roller";
    REQUIRE(f.registerTemplate(leaf, [](const MechanismBuildContext& ctx) {
                 return std::make_unique<testing::FakeMechanism>(ctx);
             }).ok());
    return f;
}

}  // namespace

// ---- 1. device identity --------------------------------------------------------------------------

TEST_CASE("physical identity has no bus, declared addresses do") {
    const DeviceKey rio{"phoenix6", "talonfx", 4, "rio", -1};
    const DeviceKey canivore{"phoenix6", "talonfx", 4, "canivore", -1};
    CHECK(rio != canivore);
    CHECK(rio.physical() == canivore.physical());
    CHECK(rio.physical() == talon(4).physicalId());
    CHECK(rio.physical().toString() == "phoenix6/talonfx[4]");
    CHECK(DeviceKey{"phoenix6", "talonfx", 5, "rio", -1}.physical() != rio.physical());
    CHECK(DeviceKey{"hal", "solenoid", 1, "rio", 0}.physical() != DeviceKey{"hal", "solenoid", 1, "rio", 1}.physical());
}

TEST_CASE("alias state classification") {
    CHECK(classifyAliasState(0, 0) == AliasState::None);
    CHECK(classifyAliasState(1, 1) == AliasState::None);
    CHECK(classifyAliasState(2, 1) == AliasState::Aliased);
    CHECK(classifyAliasState(3, 0) == AliasState::Aliased);
    CHECK(classifyAliasState(2, 2) == AliasState::Conflict);
}

TEST_CASE("validator contract: independent mappings of conflicting aliases are rejected with a code") {
    const DeviceKey rio{"phoenix6", "talonfx", 4, "rio", -1};
    const DeviceKey canivore{"phoenix6", "talonfx", 4, "canivore", -1};
    const DeviceKey other{"phoenix6", "talonfx", 5, "rio", -1};

    std::vector<SlotMapping> ok{{{"intake", "motor"}, rio}, {{"shooter", "motor"}, other}};
    CHECK(findAliasConflicts(ok).empty());
    CHECK(validateDeviceAliases(ok).empty());

    std::vector<SlotMapping> bad = ok;
    bad.push_back({{"climber", "motor"}, canivore});
    const auto conflicts = findAliasConflicts(bad);
    REQUIRE(conflicts.size() == 1);
    CHECK(conflicts[0].physical == rio.physical());
    CHECK(conflicts[0].mappings.size() == 2);

    const auto issues = validateDeviceAliases(bad);
    REQUIRE(issues.size() == 1);
    CHECK(issues[0].severity == Severity::Error);
    CHECK(issues[0].code == validation_code::kDeviceAliasConflict);
    CHECK(issues[0].path == "intake.motor");

    // the same through RobotConfig::allMappings(), as core's validator will call it
    RobotConfig config;
    MechanismConfig a = mechanism("intake", "roller");
    a.mappings = {{{"intake", "motor"}, rio}};
    MechanismConfig b = mechanism("climber", "roller");
    b.mappings = {{{"climber", "motor"}, canivore}};
    config.mechanisms = {a, b};
    CHECK(hasCode(validateDeviceAliases(config.allMappings()), validation_code::kDeviceAliasConflict));
}

TEST_CASE("hub safe behaviour: a conflicted device is disabled, looped back and reported") {
    testing::FakeDeviceHub hub;
    const DeviceId id = hub.addDevice(talon(4));  // bus unknown: nothing fabricated
    const DeviceKey rio{"phoenix6", "talonfx", 4, "rio", -1};
    const DeviceKey canivore{"phoenix6", "talonfx", 4, "canivore", -1};
    const SlotAddress a{"intake", "motor"};
    const SlotAddress b{"climber", "motor"};

    REQUIRE(hub.bind(a, rio).ok());
    CHECK(hub.find(canivore) == hub.find(id));  // resolution ignores the bus
    CHECK(hub.findPhysical(rio.physical()) == hub.find(id));
    CHECK(hub.find(id)->aliasState == AliasState::None);
    CHECK(hub.isMapped(id));
    hub.setMotorOutput(a, MotorOutput{true, 0.5, 6.0, 0.0, false});
    CHECK(hub.motorOutput(a).voltageV == 6.0);

    REQUIRE(hub.bind(b, canivore).ok());  // accepted (saved configs), but unsafe
    const DiscoveredDevice* d = hub.find(id);
    REQUIRE(d != nullptr);
    CHECK(d->aliasState == AliasState::Conflict);
    CHECK(d->aliases.size() == 2);
    CHECK_FALSE(hub.isMapped(id));                 // => LoopbackSet in the real hub
    CHECK_FALSE(hub.motorOutput(a).valid);          // slots read neutral...
    hub.writeEncoder(a, EncoderFeedback{});
    CHECK_FALSE(hub.lastEncoder(a).has_value());    // ...and ignore writes
    CHECK_FALSE(hub.resolve(a).has_value());
    CHECK(hub.boundDevice(a).has_value());          // mappings are kept
    CHECK(hasWarning(hub.warnings(), validation_code::kDeviceAliasConflict));

    hub.unbind(b);  // fixing the config re-enables the device
    CHECK(hub.find(id)->aliasState == AliasState::None);
    CHECK(hub.isMapped(id));
    CHECK(hub.motorOutput(a).valid);
    CHECK_FALSE(hasWarning(hub.warnings(), validation_code::kDeviceAliasConflict));
}

TEST_CASE("aliases that share one mapping are benign, observed bus is a label and never an identity") {
    testing::FakeDeviceHub hub;
    const DeviceId id = hub.addDevice(talon(7));
    const SlotAddress slot{"arm", "motor"};
    REQUIRE(hub.bind(slot, DeviceKey{"phoenix6", "talonfx", 7, "rio", -1}).ok());

    hub.declareAlias(id, DeviceKey{"phoenix6", "talonfx", 7, "canivore", -1});
    CHECK(hub.find(id)->aliasState == AliasState::Aliased);
    CHECK(hub.isMapped(id));
    CHECK(hasWarning(hub.warnings(), validation_code::kDeviceAliased));
    CHECK_FALSE(hasWarning(hub.warnings(), validation_code::kDeviceAliasConflict));

    CHECK(hub.find(id)->observedBus.empty());
    hub.observeBus(id, "canivore");
    CHECK(hub.find(id)->observedBus == "canivore");
    CHECK(hasWarning(hub.warnings(), validation_code::kDeviceBusMismatch));  // the rio alias contradicts it
    CHECK(hub.isMapped(id));                                                 // never disables anything
}

TEST_CASE("DeviceAnnounce can carry an observed bus and DeviceDescriptor.bus can be unknown") {
    proto::DeviceAnnounce a;
    a.descriptor = talon(1);
    CHECK(a.observedBus.empty());
    CHECK(a.descriptor.bus == kBusUnknown);
    CHECK(a.descriptor.key().bus.empty());
}

// ---- 2. plugin ownership --------------------------------------------------------------------------

TEST_CASE("a plugin owns only its section and the ids under its own prefix") {
    CHECK(ownsPluginId("game_2026", "game_2026.shooter"));
    CHECK_FALSE(ownsPluginId("game_2026", "roller"));
    CHECK_FALSE(ownsPluginId("game_2026", "game_2026"));
    CHECK_FALSE(ownsPluginId("game_2026", "game_2026."));
    CHECK_FALSE(ownsPluginId("game_2026", "game_20260.shooter"));  // prefix must end at the dot
    CHECK_FALSE(ownsPluginId("game_2026", "game_2027.shooter"));
    CHECK_FALSE(ownsPluginId("", ".x"));
}

TEST_CASE("plugins can require an SDK minor, and the SDK stays at 1.0 until release") {
    CHECK(kSdkVersionMajor == 1);
    CHECK(kSdkVersionMinor == 0);

    class NeedsFuture final : public testing::FakeGameModule {
    public:
        NeedsFuture() : FakeGameModule("game_future", 2997) {}
        GameMetadata metadata() const override {
            GameMetadata m = FakeGameModule::metadata();
            m.minSdkMinor = kSdkVersionMinor + 1;
            return m;
        }
    };
    GamePluginRegistry registry;
    CHECK(registry.add(std::make_unique<NeedsFuture>()).error().code == ErrorCode::VersionMismatch);
    CHECK(registry.add(std::make_unique<testing::FakeGameModule>("game_ok", 2996)).ok());
}

// ---- 3. mechanism attachment -----------------------------------------------------------------------

TEST_CASE("attachTo sets the structured attachment and a mechanism starts on the chassis root") {
    MechanismConfig m = mechanism("arm");
    CHECK(m.attachment.onChassis());
    CHECK(m.attachment.parentLink == kRootLink);

    m.attachTo("elevator", "carriage");
    CHECK(m.attachment.parentMechanismId == "elevator");
    CHECK(m.attachment.parentLink == "carriage");

    m.attachTo("");  // back to the chassis
    CHECK(m.attachment.onChassis());
    CHECK(m.attachment.parentMechanismId == "chassis");
}
TEST_CASE("every template has the implicit root link and cannot redeclare it") {
    MechanismTemplateInfo info;
    info.type = "x";
    CHECK(hasLink(info, "root"));
    CHECK_FALSE(hasLink(info, "tip"));
    info.links = {LinkSpec{"tip", "Tip", {}, ""}};
    CHECK(hasLink(info, "tip"));
    CHECK(templateLinkIssues(info).empty());

    MechanismFactory factory;
    auto make = [](const MechanismBuildContext& ctx) { return std::make_unique<testing::FakeMechanism>(ctx); };
    info.links = {LinkSpec{"root", "Root", {}, ""}};
    CHECK(factory.registerTemplate(info, make).error().code == ErrorCode::InvalidArgument);
    info.links = {LinkSpec{"tip", "", {}, ""}, LinkSpec{"tip", "", {}, ""}};
    CHECK(factory.registerTemplate(info, make).error().code == ErrorCode::InvalidArgument);
    info.links = {LinkSpec{"", "", {}, ""}};
    CHECK(hasCode(templateLinkIssues(info), validation_code::kTemplateLinkInvalid));
    info.links = {LinkSpec{"tip", "", {}, ""}};
    CHECK(factory.registerTemplate(info, make).ok());
}

TEST_CASE("attachment validation: missing parent, missing link, cycles, reserved id") {
    const MechanismFactory factory = factoryWithElevator();
    RobotConfig config;
    config.mechanisms = {mechanism("lift"), mechanism("arm", "roller"), mechanism("wrist", "roller")};
    config.mechanisms[1].attachTo("lift", "carriage");
    config.mechanisms[2].attachTo("arm");  // roller has only root
    CHECK(validateMechanismAttachments(config, factory).empty());

    config.mechanisms[1].attachTo("lift", "nope");
    CHECK(hasCode(validateMechanismAttachments(config, factory), validation_code::kAttachmentLinkMissing));
    config.mechanisms[1].attachTo("lift", "carriage");

    config.mechanisms[2].attachTo("arm", "tip");  // roller declares no "tip"
    CHECK(hasCode(validateMechanismAttachments(config, factory), validation_code::kAttachmentLinkMissing));
    config.mechanisms[2].attachTo("arm");

    config.mechanisms[2].attachTo("ghost");
    CHECK(hasCode(validateMechanismAttachments(config, factory), validation_code::kAttachmentParentMissing));
    config.mechanisms[2].attachTo("arm");

    MechanismConfig onChassis = mechanism("bumper", "roller");
    onChassis.attachTo("chassis", "left_corner");  // the chassis only has root
    config.mechanisms.push_back(onChassis);
    CHECK(hasCode(validateMechanismAttachments(config, factory), validation_code::kAttachmentLinkMissing));
    config.mechanisms.pop_back();

    config.mechanisms.push_back(mechanism("chassis", "roller"));
    CHECK(hasCode(validateMechanismAttachments(config, factory), validation_code::kMechanismIdReserved));
    config.mechanisms.pop_back();

    // cycle lift -> wrist -> arm -> lift, and a self-parent
    config.mechanisms[0].attachTo("wrist");
    auto issues = validateMechanismAttachments(config, factory);
    CHECK(std::count_if(issues.begin(), issues.end(), [](const Issue& i) { return i.code == validation_code::kAttachmentCycle; }) == 3);
    config.mechanisms[0].attachTo("chassis");
    config.mechanisms[1].attachTo("arm");
    issues = validateMechanismAttachments(config, factory);
    CHECK(hasCode(issues, validation_code::kAttachmentCycle));
    for (const Issue& i : issues) CHECK(i.severity == Severity::Error);
}

TEST_CASE("resolved link transforms follow the attachment chain and the live link poses") {
    const MechanismFactory factory = factoryWithElevator();
    RobotConfig config;
    config.mechanisms = {mechanism("lift"), mechanism("arm", "roller")};
    config.mechanisms[0].mount = Pose3{Vec3{1.f, 0.f, 0.f}, Quat::fromYaw(1.5707964f)};  // 1 m forward, turned left
    config.mechanisms[1].attachTo("lift", "carriage");
    config.mechanisms[1].mount = Pose3{Vec3{0.1f, 0.f, 0.f}, Quat{}};

    LinkedMechanism lift("lift", "elevator");
    LinkedMechanism arm("arm", "roller");
    lift.links = {LinkPose{"carriage", Pose3{Vec3{0.f, 0.f, 0.5f}, Quat{}}}};  // raised to 0.5 m

    testing::FakeMechanismDirectory dir(config, &factory);
    dir.add(&lift);
    dir.add(&arm);
    IMechanismDirectory& iface = dir;

    const auto liftRoot = iface.linkPoseInChassis("lift", "root");
    REQUIRE(liftRoot.has_value());
    CHECK(liftRoot->position.x == doctest::Approx(1.f));

    const auto armRoot = iface.linkPoseInChassis("arm", "root");
    REQUIRE(armRoot.has_value());
    // carriage is 0.5 m up in the lift frame, the arm is 0.1 m along the lift's +x, which is the chassis +y
    CHECK(armRoot->position.x == doctest::Approx(1.f).epsilon(1e-4));
    CHECK(armRoot->position.y == doctest::Approx(0.1f).epsilon(1e-4));
    CHECK(armRoot->position.z == doctest::Approx(0.5f));

    lift.links[0].pose.position.z = 0.9f;  // the carriage moved: the resolved transform follows
    CHECK(iface.linkPoseInChassis("arm", "root")->position.z == doctest::Approx(0.9f));

    // static rest pose from the template while the mechanism has not published the link
    lift.links.clear();
    CHECK(iface.linkPoseInChassis("arm", "root")->position.z == doctest::Approx(0.2f));

    CHECK_FALSE(iface.linkPoseInChassis("arm", "unknown").has_value());
    CHECK_FALSE(iface.linkPoseInChassis("ghost", "root").has_value());

    // broken chains never recurse: cycle and missing parent resolve to nothing
    config.mechanisms[0].attachTo("arm");
    CHECK_FALSE(iface.linkPoseInChassis("arm", "root").has_value());
    config.mechanisms[0].attachTo("ghost");
    CHECK_FALSE(iface.linkPoseInChassis("lift", "root").has_value());
}

TEST_CASE("a mechanism without a directory binding still ticks against the extended SimContext") {
    testing::FakeSimEnvironment env;
    LinkedMechanism m("solo", "roller");
    CHECK(m.linkPose("root").has_value());
    CHECK_FALSE(m.linkPose("tip").has_value());
    m.links = {LinkPose{"tip", Pose3{Vec3{0.f, 0.f, 1.f}, Quat{}}}};
    CHECK(m.linkPose("tip")->position.z == 1.f);
    CHECK(env.ctx.mechanisms() == nullptr);
}
