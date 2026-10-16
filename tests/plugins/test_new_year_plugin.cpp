// Proof that a new season is "copy the template, register one line": a complete year plugin (field, game state,
// mechanism template, config schema with a migration, HUD) written exactly like plugins/_template, registered next to
// the built-in 2026 plugin, and driven by the REAL core config pipeline and simulation session. Neither core nor
// any other plugin is touched. (tools/check_boundaries.py, run by CTest, separately proves core contains no
// season-specific code and plugins link only the SDK.)
#include <doctest/doctest.h>

#include <memory>
#include <string>

#include "../runtime/runtime_fixtures.h"
#include "wpiemu/core/config/config_pipeline.h"
#include "wpiemu/core/config/json_codec.h"
#include "wpiemu/core/runtime/simulation_session.h"
#include "wpiemu/sdk/game/builtin_plugins.h"
#include "wpiemu/sdk/game/null_objects.h"
#include "wpiemu/sdk/game/plugin_registry.h"
#include "wpiemu/sdk/telemetry/snapshot.h"

using namespace wpiemu::sdk;
using wpiemu::test_runtime::Registries;
using wpiemu::test_runtime::sampleConfig;

namespace wpiemu::tests::new_year_plugin {

// ---- the "2998 season" plugin, structured like the template ----------------------------------------------------

class CounterMechanism final : public IMechanism {
public:
    explicit CounterMechanism(const MechanismBuildContext& ctx) : id_(ctx.config.id) {}
    const std::string& id() const override { return id_; }
    std::string_view type() const override { return "game_2998.counter"; }
    void tick(SimContext&, double) override { ++ticks; }
    void reset() override { ticks = 0; }
    MechanismState state() const override {
        MechanismState s;
        s.id = id_;
        s.type = "game_2998.counter";
        s.values.push_back({"ticks", static_cast<double>(ticks), ""});
        return s;
    }
    int ticks = 0;

private:
    std::string id_;
};

class Field final : public IField {
public:
    explicit Field(IPhysicsWorld& world) : world_(world) {
        BodyDesc d;
        d.type = BodyType::Static;
        body_ = world.createBody(d);
        ShapeDesc s;
        s.geometry = BoxShape{{5.f, 5.f, 0.1f}};
        world.addShape(body_, s);
        desc_.name = "Season 2998";
        desc_.lengthM = 10.f;
        desc_.widthM = 10.f;
    }
    ~Field() override { world_.destroyBody(body_); }
    const FieldDescription& description() const override { return desc_; }

private:
    IPhysicsWorld& world_;
    BodyId body_;
    FieldDescription desc_;
};

class State final : public IGameState {
public:
    std::string_view pluginId() const override { return "game_2998"; }
    void reset(SimContext&) override { steps = 0; }
    void step(SimContext&, double) override { ++steps; }
    void publish(SnapshotBuilder& out) const override { out.setHud("game_2998.steps", steps); }
    int steps = 0;
};

class RenameCapMigrator final : public IConfigMigrator {
public:
    int fromVersion() const override { return 1; }
    int toVersion() const override { return 2; }
    Status migrate(Value& data) const override {
        if (const Value* old = data.find("limit")) {
            const Value moved = *old;
            data.object().erase("limit");
            data["capacity"] = moved;
        }
        return okStatus();
    }
};

class Module final : public IGameModule {
public:
    GameMetadata metadata() const override {
        GameMetadata m;
        m.id = "game_2998";
        m.displayName = "Season 2998";
        m.year = 2998;
        return m;
    }
    Status registerMechanisms(MechanismFactory& f) override {
        MechanismTemplateInfo info;
        info.type = "game_2998.counter";
        info.displayName = "Counter";
        info.origin = "game_2998";
        info.slotSpecs.push_back({"motor", "Motor", SlotType::Motor, false, ""});
        return f.registerTemplate(std::move(info), [](const MechanismBuildContext& c) { return std::make_unique<CounterMechanism>(c); });
    }
    Result<std::unique_ptr<IField>> createField(IPhysicsWorld& w) override {
        return std::unique_ptr<IField>(std::make_unique<Field>(w));
    }
    Result<std::unique_ptr<IGameState>> createGameState(SimContext&) override {
        return std::unique_ptr<IGameState>(std::make_unique<State>());
    }
    PluginConfigSchema contributeConfigSchema() const override {
        PluginConfigSchema s;
        s.schemaVersion = 2;
        PropertySpec p;
        p.key = "capacity";
        p.label = "Capacity";
        p.type = PropertyType::Int;
        p.defaultValue = 7;
        p.min = 0;
        p.max = 99;
        s.properties.properties.push_back(p);
        return s;
    }
    std::vector<std::unique_ptr<IConfigMigrator>> configMigrators() const override {
        std::vector<std::unique_ptr<IConfigMigrator>> m;
        m.push_back(std::make_unique<RenameCapMigrator>());
        return m;
    }
    std::vector<HudWidgetDescriptor> contributeHud() const override {
        HudWidgetDescriptor w;
        w.id = "game_2998.steps";
        w.title = "Steps";
        w.valueKey = "game_2998.steps";
        return {w};
    }
};

WPIEMU_REGISTER_GAME_PLUGIN(Module)

}  // namespace wpiemu::tests::new_year_plugin

namespace {

struct World {
    Registries reg{false};
    wpiemu::core::config::RobotConfigCodec codec;
    wpiemu::core::config::ConfigValidator validator;
    wpiemu::core::config::ConfigPipeline pipeline{codec, validator, reg.registrar, reg.drivetrains, reg.plugins};

    World() {
        REQUIRE(registerBuiltinGamePlugins(reg.plugins).ok());                   // the shipped seasons
        REQUIRE(wpiemu::tests::new_year_plugin::registerPlugin(reg.plugins).ok());  // + the new one: ONE call
    }
};

}  // namespace

TEST_CASE("a new season registers beside the built-ins without touching either") {
    World w;
    REQUIRE(w.reg.plugins.size() == 2);
    REQUIRE(w.reg.plugins.findByYear(2026) != nullptr);
    REQUIRE(w.reg.plugins.findByYear(2998) != nullptr);
    CHECK(w.reg.plugins.findByYear(2026)->metadata().id == "game_2026");
    CHECK(w.reg.plugins.findByYear(2998)->metadata().id == "game_2998");
    // duplicates are refused, not shadowed
    CHECK(wpiemu::tests::new_year_plugin::registerPlugin(w.reg.plugins).error().code == ErrorCode::AlreadyExists);
}

TEST_CASE("the core config pipeline selects the new plugin by year and migrates its section") {
    World w;
    RobotConfig c = sampleConfig();
    c.project.wpilibYear = 2998;
    c.plugins["game_2998"] = PluginSection{1, Value::makeObject()};
    c.plugins["game_2998"].data["limit"] = 12;  // v1 key

    const auto loaded = w.pipeline.load(w.codec.toValue(c));
    REQUIRE_MESSAGE(loaded.ok(), (loaded.ok() ? "" : loaded.error().message));
    CHECK(loaded.value().pluginId == "game_2998");
    CHECK(loaded.value().pluginMigrated);
    CHECK(loaded.value().config.findPlugin("game_2998")->schemaVersion == 2);
    CHECK(loaded.value().config.findPlugin("game_2998")->data.find("capacity")->asInt() == 12);
    CHECK(loaded.value().valid());
    CHECK(w.reg.mechanisms.contains("game_2998.counter"));

    // the same pipeline still selects 2026 for a 2026 project
    RobotConfig c26 = sampleConfig();
    c26.project.wpilibYear = 2026;
    const auto loaded26 = w.pipeline.load(w.codec.toValue(c26));
    REQUIRE(loaded26.ok());
    CHECK(loaded26.value().pluginId == "game_2026");
    CHECK(loaded26.value().config.findPlugin("game_2026")->schemaVersion >= 2);
    CHECK(loaded26.value().valid());
}

TEST_CASE("the core session runs the new plugin's field, game state and mechanism") {
    World w;
    RobotConfig c = sampleConfig();
    c.project.wpilibYear = 2998;
    MechanismConfig m;
    m.id = "counter";
    m.type = "game_2998.counter";
    c.mechanisms.push_back(m);
    const auto loaded = w.pipeline.load(w.codec.toValue(c));
    REQUIRE_MESSAGE(loaded.ok(), (loaded.ok() ? "" : loaded.error().message));
    REQUIRE(loaded.value().valid());

    wpiemu::core::runtime::SessionParts parts;
    parts.config = loaded.value().config;
    auto world = std::make_unique<testing::FakePhysicsWorld>(parts.config.sim.physics);
    testing::FakePhysicsWorld* worldPtr = world.get();
    parts.world = std::move(world);
    parts.mechanisms = &w.reg.mechanisms;
    parts.drivetrains = &w.reg.drivetrains;
    parts.tireModels = &w.reg.tires;
    parts.plugins = &w.reg.plugins;

    auto built = wpiemu::core::runtime::SessionBuilder::build(std::move(parts));
    REQUIRE_MESSAGE(built.ok(), (built.ok() ? "" : built.error().message));
    auto& session = *built.value();
    CHECK(session.gameModule() != nullptr);
    CHECK(session.gameModule()->metadata().id == "game_2998");
    CHECK(worldPtr->shapeCount() >= 1);  // the plugin's field box exists next to the chassis

    proto::SensorFrame frame;
    for (int i = 0; i < 5; ++i) session.tick(frame, false);
    const auto snapshot = session.buildSnapshot(false, true);
    REQUIRE(snapshot->hudValue("game_2998.steps") != nullptr);
    CHECK(snapshot->hudValue("game_2998.steps")->asInt() == 5);
    bool sawCounter = false;
    for (const MechanismState& s : snapshot->mechanisms())
        if (s.id == "counter") {
            sawCounter = true;
            CHECK(s.values.front().value == 5.0);
        }
    CHECK(sawCounter);
}

TEST_CASE("the template plugin never ships in the built-in set") {
    GamePluginRegistry registry;
    REQUIRE(registerBuiltinGamePlugins(registry).ok());
    CHECK(registry.find("_template") == nullptr);
    for (const IGameModule* m : registry.list()) CHECK(m->metadata().year > 0);
}
