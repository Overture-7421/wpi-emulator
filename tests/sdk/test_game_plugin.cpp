#include <doctest/doctest.h>

#include <cmath>
#include <memory>
#include <string>

#include "wpiemu/plugins/_template/plugin.h"
#include "wpiemu/plugins/game_2026/plugin.h"
#include "wpiemu/sdk/game/builtin_plugins.h"
#include "wpiemu/sdk/game/plugin_registry.h"
#include "wpiemu/sdk/testing/fake_game_module.h"
#include "wpiemu/sdk/testing/fake_sim_environment.h"

using namespace wpiemu::sdk;

namespace wpiemu::tests::dummy_plugin {

// A complete (if useless) year plugin registered with the same macro seasons/_template/game uses.
class DummyModule final : public IGameModule {
public:
    GameMetadata metadata() const override {
        GameMetadata m;
        m.id = "game_dummy";
        m.displayName = "Dummy";
        m.year = 2999;
        return m;
    }
    Status registerMechanisms(MechanismFactory&) override { return okStatus(); }
    Result<std::unique_ptr<IField>> createField(IPhysicsWorld&) override {
        return std::unique_ptr<IField>(std::make_unique<EmptyField>());
    }
    Result<std::unique_ptr<IGameState>> createGameState(SimContext&) override {
        return std::unique_ptr<IGameState>(std::make_unique<NullGameState>("game_dummy"));
    }
    PluginConfigSchema contributeConfigSchema() const override { return {}; }
};

WPIEMU_REGISTER_GAME_PLUGIN(DummyModule)

}  // namespace wpiemu::tests::dummy_plugin

TEST_CASE("a plugin registers through the macro with zero core changes") {
    GamePluginRegistry registry;
    CHECK(wpiemu::tests::dummy_plugin::registerPlugin(registry).ok());
    CHECK(registry.size() == 1);
    REQUIRE(registry.find("game_dummy") != nullptr);
    CHECK(registry.findByYear(2999) == registry.find("game_dummy"));
    CHECK(registry.findByYear(1999) == nullptr);  // unsupported year => core features only

    // duplicate registration is an error, not an override
    CHECK(wpiemu::tests::dummy_plugin::registerPlugin(registry).error().code == ErrorCode::AlreadyExists);
}

TEST_CASE("registry validates module metadata") {
    GamePluginRegistry registry;
    CHECK(registry.add(nullptr).error().code == ErrorCode::InvalidArgument);
    CHECK(registry.add(std::make_unique<testing::FakeGameModule>("", 2030)).error().code == ErrorCode::InvalidArgument);
    CHECK(registry.add(std::make_unique<testing::FakeGameModule>("no_year", 0)).error().code == ErrorCode::InvalidArgument);
    CHECK(registry.add(std::make_unique<testing::FakeGameModule>("_template_like", 0)).ok());
}

TEST_CASE("built-in plugins aggregate through the seam") {
    GamePluginRegistry registry;
    CHECK(registerBuiltinGamePlugins(registry).ok());
    const IGameModule* g = registry.findByYear(2026);
    REQUIRE(g != nullptr);
    CHECK(g->metadata().id == "game_2026");
    CHECK(registry.find("_template") == nullptr);  // the template is never built in
}

TEST_CASE("template and game_2026 stubs implement the whole contract") {
    for (auto reg : {&wpiemu::plugins::game_2026::registerPlugin, &wpiemu::plugins::_template::registerPlugin}) {
        GamePluginRegistry registry;
        REQUIRE(reg(registry).ok());
        IGameModule* module = registry.list().front();
        testing::FakeSimEnvironment env;
        MechanismFactory factory;
        CHECK(module->registerMechanisms(factory).ok());
        auto field = module->createField(env.physics);
        REQUIRE(field.ok());
        auto state = module->createGameState(env.ctx);
        REQUIRE(state.ok());
        state.value()->reset(env.ctx);
        state.value()->step(env.ctx, 0.005);
        module->onTick(env.ctx, 0.005);
        CHECK(module->contributeConfigSchema().schemaVersion >= 1);
    }
}

TEST_CASE("fake game module drives a full plugin lifecycle through the SDK only") {
    GamePluginRegistry registry;
    auto fake = std::make_unique<testing::FakeGameModule>("game_fake", 2998);
    const auto counters = fake->counters();
    REQUIRE(registry.add(std::move(fake)).ok());
    IGameModule* module = registry.find("game_fake");

    testing::FakeSimEnvironment env;
    MechanismFactory mechanisms;
    REQUIRE(module->registerMechanisms(mechanisms).ok());
    CHECK(mechanisms.contains("game_fake.fake_mechanism"));
    CHECK(mechanisms.templateInfo("game_fake.fake_mechanism")->origin == "game_fake");

    auto field = module->createField(env.physics);
    REQUIRE(field.ok());
    CHECK(field.value()->description().lengthM == 16.f);

    auto state = module->createGameState(env.ctx);
    REQUIRE(state.ok());
    env.ctx.bindGameState(state.value().get());
    CHECK(env.ctx.gameState() == state.value().get());

    // a mechanism built from the plugin template ticks
    MechanismConfig cfg;
    cfg.id = "m1";
    cfg.type = "game_fake.fake_mechanism";
    auto mech = mechanisms.create(cfg.type, MechanismBuildContext{cfg, env.ctx});
    REQUIRE(mech.ok());

    for (int i = 0; i < 3; ++i) {
        mech.value()->tick(env.ctx, 0.005);
        env.advance();
        state.value()->step(env.ctx, 0.005);
        module->onTick(env.ctx, 0.005);
    }
    CHECK(counters->gameStateSteps == 3);
    CHECK(counters->onTickCalls == 3);
    CHECK(mech.value()->state().values.front().value == 3.0);

    SnapshotBuilder builder;
    state.value()->publish(builder);
    mech.value().get()->reset();
    auto snap = builder.build();
    REQUIRE(snap->hudValue("game_fake.steps") != nullptr);
    CHECK(snap->hudValue("game_fake.steps")->asInt() == 3);
    CHECK(module->contributeHud().front().valueKey == "game_fake.steps");
    CHECK(module->contributeEditorPanels().size() == 1);
}

TEST_CASE("MechanismFactory publishes template metadata for the editor") {
    MechanismFactory factory;
    MechanismTemplateInfo info;
    info.type = "roller";
    info.displayName = "Roller";
    info.slotSpecs = {{"motor", "Motor", SlotType::Motor, true, ""},
                  {"beam", "Beam break", SlotType::BeamBreak, false, ""}};
    PropertySpec p;
    p.key = "momentOfInertia";
    p.defaultValue = 0.01;
    info.properties.properties.push_back(p);
    CHECK(factory.registerTemplate(info, [](const MechanismBuildContext& ctx) {
                     return std::make_unique<testing::FakeMechanism>(ctx);
                 }).ok());
    CHECK(factory.registerTemplate(info, nullptr).error().code == ErrorCode::InvalidArgument);

    std::vector<std::string> palette;
    factory.forEach([&](const std::string& key, const MechanismTemplateInfo& i) { palette.push_back(key + ":" + i.displayName); });
    CHECK(palette == std::vector<std::string>{"roller:Roller"});
    CHECK(factory.templateInfo("roller")->slotSpecs.size() == 2);
    CHECK(defaultsOf(factory.templateInfo("roller")->properties).find("momentOfInertia")->asDouble() == 0.01);
}

TEST_CASE("SimContext bundles services without globals") {
    testing::FakeSimEnvironment env;
    env.driverStation.enabled = true;
    env.config.name = "Bot";
    env.ctx.bindChassisBody(BodyId{9});
    CHECK(env.ctx.driverStation().enabled);
    CHECK(env.ctx.config().name == "Bot");
    CHECK(env.ctx.chassisBody() == BodyId{9});
    CHECK(env.ctx.gameState() == nullptr);
    CHECK(env.ctx.mechanisms() == nullptr);
    env.advance();
    CHECK(env.ctx.clock().tickIndex() == 1);
    CHECK(env.ctx.clock().simTimeUs() == 5000);
    env.ctx.logger().warn("test", "hello");
    CHECK(env.logger.count(LogLevel::Warn) == 1);
}

TEST_CASE("field frame: flipping a start pose to the other alliance round-trips for both symmetries") {
    const RobotStartPose blue{4.51f, 0.527f, 0.f};
    FieldFrame frame{16.541f, 8.069f, FieldSymmetry::Rotational};
    const auto red = flipToOtherAlliance(blue, frame);
    REQUIRE(red.has_value());
    CHECK(red->xM == doctest::Approx(16.541f - 4.51f));
    CHECK(red->yM == doctest::Approx(8.069f - 0.527f));
    CHECK(std::fabs(red->headingRad) == doctest::Approx(3.14159265f));
    const auto back = flipToOtherAlliance(*red, frame);
    REQUIRE(back.has_value());
    CHECK(back->xM == doctest::Approx(blue.xM));
    CHECK(back->yM == doctest::Approx(blue.yM));
    CHECK(back->headingRad == doctest::Approx(0.f).epsilon(1e-4));

    frame.symmetry = FieldSymmetry::Mirrored;
    const auto mirrored = flipToOtherAlliance(RobotStartPose{4.51f, 0.527f, 0.5f}, frame);
    REQUIRE(mirrored.has_value());
    CHECK(mirrored->yM == doctest::Approx(0.527f));
    CHECK(mirrored->headingRad == doctest::Approx(3.14159265f - 0.5f));

    frame.symmetry = FieldSymmetry::None;
    CHECK_FALSE(flipToOtherAlliance(blue, frame).has_value());
}
