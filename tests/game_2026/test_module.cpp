#include <doctest/doctest.h>

#include <algorithm>
#include <string>

#include "test_support.h"
#include "wpiemu/plugins/game_2026/config.h"
#include "wpiemu/plugins/game_2026/fuel_zones.h"
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
    CHECK(m.minSdkMinor == 0);
    CHECK(module->contributeConfigSchema().schemaVersion == kConfigSchemaVersion);
}

TEST_CASE("zone kinds are owned by the plugin, registered no mechanism templates and describe themselves for the editor") {
    g26test::Session s;
    REQUIRE(s.module().registerMechanisms(s.factory).ok());
    CHECK(s.factory.empty());  // the pickup and the shooter are zones, not mechanisms
    CHECK(s.module().registerMechanisms(s.factory).ok());

    const std::vector<sdk::ZoneKindInfo> kinds = s.module().contributeZoneKinds();
    REQUIRE(kinds.size() == 2);
    const sdk::ZoneKindInfo* pickup = nullptr;
    const sdk::ZoneKindInfo* shooter = nullptr;
    for (const sdk::ZoneKindInfo& k : kinds) {
        CHECK(sdk::ownsMechanismType("game_2026", k.kind));  // "<pluginId>." prefix
        CHECK_FALSE(k.displayName.empty());
        CHECK_FALSE(k.description.empty());
        // defaults validate and every property is documented for the generic inspector
        CHECK(sdk::validateProperties(k.properties, sdk::defaultsOf(k.properties)).empty());
        for (const sdk::PropertySpec& p : k.properties.properties) {
            CHECK_MESSAGE(!p.label.empty(), p.key);
            CHECK_MESSAGE(!p.group.empty(), p.key);
            CHECK_MESSAGE(!p.description.empty(), p.key);
        }
        if (k.kind == kPickupKind) pickup = &k;
        if (k.kind == kShooterKind) shooter = &k;
    }
    REQUIRE(pickup != nullptr);
    REQUIRE(shooter != nullptr);
    CHECK(std::string(kPickupKind) == "game_2026.pickup");
    CHECK(std::string(kShooterKind) == "game_2026.shooter");

    CHECK(pickup->shape == sdk::ZoneShape::Volume);
    CHECK(pickup->defaultSizeM == sdk::Vec3{0.30f, 0.60f, 0.30f});
    CHECK(pickup->defaultLocal.position == sdk::Vec3{0.45f, 0.f, 0.15f});
    CHECK(pickup->properties.properties.size() == 1);
    CHECK(pickup->properties.find("intakeRateHz") != nullptr);

    CHECK(shooter->shape == sdk::ZoneShape::Muzzle);
    CHECK(shooter->defaultLocal.position == sdk::Vec3{0.25f, 0.f, 0.55f});
    for (const char* key : {"flywheelMechanism", "flywheelValue", "flywheelRadiusM", "efficiency", "fixedExitSpeedMps",
                            "minExitSpeedMps", "hoodAngleRad", "hoodMechanism", "hoodValue", "hoodGain", "turretMechanism",
                            "turretValue", "turretGain", "turretYawOffsetRad", "fireRateHz", "exitSpeedJitter",
                            "aimJitterRad", "inheritRobotVelocity"})
        CHECK_MESSAGE(shooter->properties.find(key) != nullptr, key);
    CHECK(shooter->properties.properties.size() == 18);  // no muzzleOffset (the zone pose) and no feed*
}

TEST_CASE("zone parameters have sane defaults and survive invalid persisted values") {
    const PickupParams pickup = parsePickupParams(sdk::Value::makeObject());
    CHECK(pickup.intakeRateHz == doctest::Approx(15.0));
    sdk::Value bad = sdk::Value::makeObject();
    bad["intakeRateHz"] = -1.0;
    CHECK(parsePickupParams(bad).intakeRateHz == 0.0);
    bad["intakeRateHz"] = "fast";
    CHECK(parsePickupParams(bad).intakeRateHz == doctest::Approx(15.0));

    const ShooterParams shooter = parseShooterParams(sdk::Value::makeObject());
    CHECK(shooter.fireRateHz == doctest::Approx(10.0));
    CHECK(shooter.efficiency == doctest::Approx(0.6));
    CHECK(shooter.hoodAngleRad == doctest::Approx(1.0472).epsilon(0.001));
    CHECK(shooter.inheritRobotVelocity);

    sdk::Value badShooter = sdk::Value::makeObject();
    badShooter["fireRateHz"] = 0.0;
    badShooter["flywheelRadiusM"] = -1.0;
    badShooter["exitSpeedJitter"] = 9.0;
    const ShooterParams fixedShooter = parseShooterParams(badShooter);
    CHECK(fixedShooter.fireRateHz >= 0.1);
    CHECK(fixedShooter.flywheelRadiusM > 0.0);
    CHECK(fixedShooter.exitSpeedJitter == doctest::Approx(0.5));
}

TEST_CASE("HUD widgets are namespaced, unique, and well-formed") {
    sdk::GamePluginRegistry registry;
    REQUIRE(registerPlugin(registry).ok());
    const auto widgets = registry.find("game_2026")->contributeHud();
    CHECK(widgets.size() >= 7);
    std::vector<std::string> ids;
    bool hasMeter = false, hasBadge = false;
    for (const sdk::HudWidgetDescriptor& w : widgets) {
        CHECK(w.id.rfind("game_2026.", 0) == 0);
        CHECK(w.valueKey.rfind("game_2026.", 0) == 0);
        CHECK(std::find(ids.begin(), ids.end(), w.id) == ids.end());
        ids.push_back(w.id);
        hasMeter |= w.kind == sdk::HudWidgetKind::Meter && !w.maxKey.empty();
        hasBadge |= w.kind == sdk::HudWidgetKind::Badge;
    }
    CHECK(hasMeter);
    CHECK(hasBadge);
}

TEST_CASE("editor panels only configure the plugin section and reference properties that exist") {
    g26test::Session s;
    const sdk::PluginConfigSchema config = s.module().contributeConfigSchema();
    const auto panels = s.module().contributeEditorPanels();
    CHECK(panels.size() >= 3);
    for (const sdk::EditorPanelDescriptor& panel : panels) {
        CHECK(panel.id.rfind("game_2026.", 0) == 0);
        CHECK_FALSE(panel.properties.properties.empty());
        CHECK(panel.scope == sdk::PanelScope::PluginConfig);  // pickup / muzzle editing is the zones' job
        for (const sdk::PropertySpec& p : panel.properties.properties)
            CHECK_MESSAGE(config.properties.find(p.key) != nullptr, p.key);
        for (const sdk::ViewportOverlayDescriptor& o : panel.overlays) {
            const sdk::PropertySpec* spec = panel.properties.find(o.propertyKey);
            REQUIRE(spec != nullptr);
            CHECK((spec->type == sdk::PropertyType::Box3 || spec->type == sdk::PropertyType::Vec3));
        }
    }
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
