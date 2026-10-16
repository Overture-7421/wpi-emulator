#include <doctest/doctest.h>

#include <algorithm>
#include <string>

#include "test_support.h"
#include "wpiemu/plugins/game_2026/config.h"
#include "wpiemu/plugins/game_2026/fuel_intake.h"
#include "wpiemu/plugins/game_2026/fuel_shooter.h"
#include "wpiemu/sdk/game/builtin_plugins.h"

using namespace wpiemu;
using namespace wpiemu::plugins::game_2026;

TEST_CASE("module metadata identifies REBUILT and registers through the aggregation seam") {
    sdk::GamePluginRegistry registry;
    REQUIRE(sdk::registerBuiltinGamePlugins(registry).ok());
    sdk::IGameModule* module = registry.findByYear(2026);
    REQUIRE(module != nullptr);
    const sdk::GameMetadata m = module->metadata();
    CHECK(m.id == "game_2026");
    CHECK(m.year == 2026);
    CHECK(m.displayName.find("REBUILT") != std::string::npos);
    CHECK(m.minSdkMajor == 1);
    CHECK(m.minSdkMinor == 1);
    CHECK(module->contributeConfigSchema().schemaVersion == kConfigSchemaVersion);
}

TEST_CASE("2026 mechanism templates are registered under the plugin id and describe themselves for the editor") {
    g26test::Session s;
    REQUIRE(s.module().registerMechanisms(s.factory).ok());
    for (const char* type : {kIntakeType, kShooterType}) {
        const sdk::MechanismTemplateInfo* info = s.factory.templateInfo(type);
        REQUIRE(info != nullptr);
        CHECK(info->origin == "game_2026");
        CHECK(sdk::ownsMechanismType("game_2026", info->type));
        CHECK(info->slotSpecs.empty());  // they bind to OTHER mechanisms, never to devices directly
        CHECK(sdk::templateLinkIssues(*info).empty());
        CHECK_FALSE(info->description.empty());
        CHECK_FALSE(info->defaultGeometry.empty());
        // defaults validate and every property is documented for the generic inspector
        CHECK(sdk::validateProperties(info->properties, sdk::defaultsOf(info->properties)).empty());
        for (const sdk::PropertySpec& p : info->properties.properties) {
            CHECK_MESSAGE(!p.label.empty(), p.key);
            CHECK_MESSAGE(!p.group.empty(), p.key);
            CHECK_MESSAGE(!p.description.empty(), p.key);
        }
    }
    CHECK(s.factory.templateInfo(kShooterType)->category == "Shooter");
    CHECK(sdk::hasLink(*s.factory.templateInfo(kShooterType), "muzzle"));
    CHECK(s.factory.templateInfo(kIntakeType)->category == "Intake");

    // registering twice is an error, never a silent override
    CHECK(s.module().registerMechanisms(s.factory).error().code == sdk::ErrorCode::AlreadyExists);
}

TEST_CASE("mechanism parameters have sane defaults and survive invalid persisted values") {
    const IntakeParams intake = parseIntakeParams(sdk::Value::makeObject());
    CHECK(intake.activation == IntakeActivation::Roller);
    CHECK(intake.rollerValue == "velocity");
    CHECK(intake.intakeRateHz == doctest::Approx(15.0));
    CHECK(intake.pickupZone.sizeM.x > 0.f);

    sdk::Value bad = sdk::Value::makeObject();
    bad["pickupZone"] = "not a box";
    bad["activation"] = "sometimes";
    bad["rollerThreshold"] = -4.0;
    bad["intakeRateHz"] = -1.0;
    const IntakeParams fixed = parseIntakeParams(bad);
    CHECK(fixed.pickupZone.sizeM.x == doctest::Approx(0.30f));  // default zone
    CHECK(fixed.activation == IntakeActivation::Roller);
    CHECK(fixed.rollerThreshold == 0.0);
    CHECK(fixed.intakeRateHz == 0.0);

    const ShooterParams shooter = parseShooterParams(sdk::Value::makeObject());
    CHECK(shooter.fireRateHz == doctest::Approx(10.0));
    CHECK(shooter.efficiency == doctest::Approx(0.6));
    CHECK(shooter.hoodAngleRad == doctest::Approx(1.0472).epsilon(0.001));
    CHECK(shooter.inheritRobotVelocity);

    sdk::Value badShooter = sdk::Value::makeObject();
    badShooter["fireRateHz"] = 0.0;
    badShooter["flywheelRadiusM"] = -1.0;
    badShooter["muzzleOffset"] = 7;
    badShooter["exitSpeedJitter"] = 9.0;
    const ShooterParams fixedShooter = parseShooterParams(badShooter);
    CHECK(fixedShooter.fireRateHz >= 0.1);
    CHECK(fixedShooter.flywheelRadiusM > 0.0);
    CHECK(fixedShooter.muzzleOffset.z == doctest::Approx(0.55f));
    CHECK(fixedShooter.exitSpeedJitter == doctest::Approx(0.5));
}

TEST_CASE("HUD widgets are namespaced, unique, and well-formed") {
    sdk::GamePluginRegistry registry;
    REQUIRE(registerPlugin(registry).ok());
    const auto widgets = registry.find("game_2026")->contributeHud();
    CHECK(widgets.size() >= 8);
    std::vector<std::string> ids;
    bool hasMeter = false, hasTimer = false, hasBadge = false;
    for (const sdk::HudWidgetDescriptor& w : widgets) {
        CHECK(w.id.rfind("game_2026.", 0) == 0);
        CHECK(w.valueKey.rfind("game_2026.", 0) == 0);
        CHECK(std::find(ids.begin(), ids.end(), w.id) == ids.end());
        ids.push_back(w.id);
        hasMeter |= w.kind == sdk::HudWidgetKind::Meter && !w.maxKey.empty();
        hasTimer |= w.kind == sdk::HudWidgetKind::Timer;
        hasBadge |= w.kind == sdk::HudWidgetKind::Badge;
    }
    CHECK(hasMeter);
    CHECK(hasTimer);
    CHECK(hasBadge);
}

TEST_CASE("editor panels only reference properties that exist in their target schema") {
    g26test::Session s;
    REQUIRE(s.module().registerMechanisms(s.factory).ok());
    const sdk::PluginConfigSchema config = s.module().contributeConfigSchema();
    const auto panels = s.module().contributeEditorPanels();
    CHECK(panels.size() >= 4);
    int mechanismPanels = 0;
    for (const sdk::EditorPanelDescriptor& panel : panels) {
        CHECK(panel.id.rfind("game_2026.", 0) == 0);
        CHECK_FALSE(panel.properties.properties.empty());
        const sdk::PropertySchema* target = nullptr;
        if (panel.scope == sdk::PanelScope::PluginConfig) {
            target = &config.properties;
        } else {
            ++mechanismPanels;
            REQUIRE(panel.mechanismTypes.size() == 1);
            const sdk::MechanismTemplateInfo* info = s.factory.templateInfo(panel.mechanismTypes.front());
            REQUIRE(info != nullptr);
            target = &info->properties;
        }
        for (const sdk::PropertySpec& p : panel.properties.properties) CHECK_MESSAGE(target->find(p.key) != nullptr, p.key);
        for (const sdk::ViewportOverlayDescriptor& o : panel.overlays) {
            const sdk::PropertySpec* spec = panel.properties.find(o.propertyKey);
            REQUIRE(spec != nullptr);
            CHECK((spec->type == sdk::PropertyType::Box3 || spec->type == sdk::PropertyType::Vec3));
        }
    }
    CHECK(mechanismPanels == 2);
}

TEST_CASE("module hooks are safe with default contexts and an absent plugin section") {
    g26test::Session s;
    s.start();  // no "game_2026" section in the config: defaults apply
    CHECK(s.game->config().capacity == 30);
    s.module().onTick(s.env.ctx, 0.005);  // game logic lives in the game state; the hook is a no-op
    CHECK(s.stateOwner->pluginId() == "game_2026");
    CHECK(s.fieldOwner->description().lengthM == doctest::Approx(16.541f));
}

TEST_CASE("plugin section values reach the game state") {
    g26test::Session s;
    s.section()["capacity"] = 12;
    s.section()["preloadFuel"] = 5;
    s.section()["neutralFuel"] = 12;
    s.start();
    CHECK(s.game->config().capacity == 12);
    CHECK(s.game->fuelInRobot() == 5);
    CHECK(s.game->fuelOnField() == 12 + 48);
}
