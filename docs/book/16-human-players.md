# 16. Human players and the exchange

## 16.1 Objectives

After this chapter the reader can implement a deterministic external agent,
model an exchange as custody transfer, and test that human actions occur only
in teleoperation. Chapter 15 is the prerequisite. The human-player module
addresses R4 and F-11 through F-12.

## 16.2 Problem statement

Portal reserves are visible but inert in the core. A useful match needs a way
to introduce those cubes and handle a robot's exchange delivery. A counter
increment alone would lose the identity of the physical cube being delivered.

## 16.3 Background and definitions

The manual separates the lower robot exchange opening from the upper human
return. Human feeding belongs to teleoperation [1, sections 3.6 and 7].
The lower opening is 21 in wide and 16.25 in high. It is not a return chute.

**Definition 16.1 (exchange custody).** Exchange custody begins when a field
cube's centre enters the exterior collection volume. Its physical body is then
disabled, while its pool identity remains available for return or vault transfer.

The case study senses at the exterior endpoint rather than simulating a human
hand and chute. Portal and upper-return endpoints are approximate poses; their
status is recorded in F-11 and F-12.

## 16.4 Design

An interactive human-player UI would require input routing outside the plugin.
A timed agent instead uses configured teleoperation time and a selected policy:
hold, return, or later deposit into the vault. Feeding is separately enabled
and is off by default.

**Invariant 16.1 (human custody).** An action transfers an existing identity
from reserve or exchange; it cannot manufacture a cube. Disabled and autonomous
intervals do not advance the human-action clock or collect exchange cubes.

One action opportunity occurs per configured period. At the book's 5 ms step,
even the minimum 0.1 s period spans several updates.

## 16.5 Implementation

### 16.5.1 Agent and return path

The agent observes the exterior exchange volumes and advances a teleoperation
timer. A return moves a cube to the upper endpoint. Portal feeding consumes a
reserve from the corresponding allocation.

Listing 16.1. Human-player timing and custody interface. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\human_player.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\human_player.h -->
```cpp
#pragma once

#include "wpiemu/plugins/game_2027/config.h"
#include "wpiemu/plugins/game_2027/cube_pool.h"
#include "wpiemu/plugins/game_2027/rules_types.h"

namespace wpiemu::plugins::game_2027 {

class HumanPlayer {
public:
    void reset() {
        elapsedUs_ = 0;
        nextActionUs_ = 0;
    }
    void observe(CubePool& pool);
    bool advance(CubePool& pool, RulesTimeUs elapsedUs, const GameConfig& config);

private:
    RulesTimeUs elapsedUs_ = 0;
    RulesTimeUs nextActionUs_ = 0;
};

}  // namespace wpiemu::plugins::game_2027
```

Listing 16.2. Exchange observation, feeding and upper return. <code>seasons\2027\game\src\human_player.cpp</code>.

<!-- book-file: seasons\2027\game\src\human_player.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/human_player.h"

namespace wpiemu::plugins::game_2027 {

void HumanPlayer::observe(CubePool& pool) {
    for (std::size_t a = 0; a < 2; ++a) {
        const s::Vec3 p{a == 0 ? -0.35f : kLengthM + 0.35f, a == 0 ? 2.4f : kWidthM - 2.4f,
                        0.206375f};
        pool.exchange(a, p,
                      {0.15f, 0.2667f, 0.206375f});  // [GM 3.6] aperture; [approx] sensing depth.
    }
}

bool HumanPlayer::advance(CubePool& pool, RulesTimeUs elapsedUs, const GameConfig& config) {
    const auto periodUs =
        static_cast<RulesTimeUs>(std::llround(config.humanPeriodS * kRulesSecondUs));
    if (nextActionUs_ == 0) nextActionUs_ = periodUs;
    elapsedUs_ += elapsedUs;
    if (elapsedUs_ < nextActionUs_) return false;
    nextActionUs_ += periodUs;
    for (std::size_t a = 0; a < 2; ++a) {
        const s::Pose3 portal{
            {a == 0 ? kLengthM - 0.4f : 0.4f, a == 0 ? 0.8f : kWidthM - 0.8f, 0.70f}, {}};
        if (config.timedFeed) pool.feed(a, portal);
        if (config.exchangeMode == "return") {
            const s::Pose3 opening{
                {a == 0 ? 0.3f : kLengthM - 0.3f, a == 0 ? 2.4f : kWidthM - 2.4f, 0.85f}, {}};
            pool.returnCube(a, opening);
        }
    }
    return true;
}

}  // namespace wpiemu::plugins::game_2027
```

The pool gains a checked exchange-to-field operation. The body is enabled
before velocity is assigned: the backend ignores velocity changes on a disabled
body. This ordering also applies to ordinary release and feed.

Listing 16.3. Pool interface with return custody. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\cube_pool.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\cube_pool.h -->
```cpp
#pragma once

#include "wpiemu/plugins/game_2027/body_set.h"
#include "wpiemu/plugins/game_2027/cube_layout.h"

namespace wpiemu::plugins::game_2027 {

enum class CubeLocation { Field, Held, Reserve, Exchange, Vault, Removed };

class CubePool {
public:
    explicit CubePool(s::IPhysicsWorld& world) : world_(world), bodies_(world) {}
    s::Status build();
    void reset(bool preload, std::size_t alliance);
    std::size_t count(CubeLocation location, std::size_t alliance) const;
    std::vector<s::Vec3> positions(CubeLocation location) const;
    bool pickup(const s::Pose3& zone, const s::Vec3& half);
    bool release(const s::Pose3& pose, const s::Vec3& velocity);
    bool feed(std::size_t alliance, const s::Pose3& pose);
    bool exchange(std::size_t alliance, const s::Vec3& center, const s::Vec3& half);
    bool deposit(std::size_t alliance);
    bool returnCube(std::size_t alliance, const s::Pose3& pose);
    int inventory() const { return held_ ? 1 : 0; }
    int released() const { return released_; }

private:
    bool contains(std::size_t i, const s::Pose3& pose, const s::Vec3& half) const;
    void move(std::size_t i, CubeLocation location);
    s::IPhysicsWorld& world_;
    Bodies bodies_;
    std::array<s::BodyId, kCubeCount> cubes_{};
    std::array<CubeLocation, kCubeCount> locations_{};
    std::array<std::size_t, kCubeCount> custodians_{};
    std::optional<std::size_t> held_;
    int released_ = 0;
};

}  // namespace wpiemu::plugins::game_2027
```

Listing 16.4. Return of one existing exchange cube. <code>seasons\2027\game\src\cube_return.cpp</code>.

<!-- book-file: seasons\2027\game\src\cube_return.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/cube_pool.h"

namespace wpiemu::plugins::game_2027 {

bool CubePool::returnCube(std::size_t alliance, const s::Pose3& pose) {
    for (std::size_t i = 0; i < cubes_.size(); ++i) {
        if (custodians_[i] != alliance || locations_[i] != CubeLocation::Exchange) continue;
        move(i, CubeLocation::Field);
        world_.setPose(cubes_[i], pose);
        world_.setLinearVelocity(cubes_[i], {});
        world_.setAngularVelocity(cubes_[i], {});
        return true;
    }
    return false;
}

}  // namespace wpiemu::plugins::game_2027
```

### 16.5.2 Aperture and configuration

The alliance wall now has a lower opening matching the nominal aperture.
An exterior collection volume without this physical gap would allow a test
to count a cube that the wall could never admit.

Listing 16.5. Wall construction with the lower exchange aperture. <code>seasons\2027\game\src\field_layout.cpp</code>.

<!-- book-file: seasons\2027\game\src\field_layout.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/field_layout.h"

namespace wpiemu::plugins::game_2027 {
Field::Field(s::IPhysicsWorld& world) : bodies_(world) {}

s::Status Field::build() {
    description_.name = "FIRST POWER UP (2027 ABI case study)";
    description_.lengthM = kLengthM;
    description_.widthM = kWidthM;
    description_.driverStations = s::genericDriverStations(kLengthM, kWidthM);
    std::vector<Box> boxes{
        {"carpet",
         {{kLengthM / 2.f, kWidthM / 2.f, -0.10f}, {}},
         {kLengthM, kWidthM, 0.20f},
         "#555962"},
        {"left_rail", {{kLengthM / 2.f, -0.05f, 0.25f}, {}}, {kLengthM, 0.10f, 0.50f}, "#9199a0"},
        {"right_rail",
         {{kLengthM / 2.f, kWidthM + 0.05f, 0.25f}, {}},
         {kLengthM, 0.10f, 0.50f},
         "#9199a0"},
    };

    for (int a = 0; a < 2; ++a) {
        const float x = a == 0 ? -0.05f : kLengthM + 0.05f;
        const float y = a == 0 ? 2.4f : kWidthM - 2.4f;  // [approx] exchange centre.
        const std::string color = a == 0 ? "#2457c5" : "#c52c30";
        const float low = y - 0.2667f;
        const float high = y + 0.2667f;  // [GM 3.6] 21 in opening.
        boxes.push_back({"wall_side", {{x, low / 2.f, 0.6f}, {}}, {0.1f, low, 1.2f}, color});
        boxes.push_back({"wall_side",
                         {{x, (high + kWidthM) / 2.f, 0.6f}, {}},
                         {0.1f, kWidthM - high, 1.2f},
                         color});
        boxes.push_back({"exchange_lintel",
                         {{x, y, (0.41275f + 1.2f) / 2.f}, {}},
                         {0.1f, 0.5334f, 1.2f - 0.41275f},
                         color});  // [GM 3.6] 16.25 in opening.
    }

    for (const float x : {kSwitchXM, kLengthM - kSwitchXM}) {
        for (const float dx : {-0.7112f, 0.7112f})
            boxes.push_back({"switch_fence",
                             {{x + dx, kWidthM / 2.f, 0.24f}, {}},
                             {0.05f, 3.90f, 0.48f},
                             "#a0a9b0"});
        for (const float dy : {-1.95f, 1.95f})
            boxes.push_back({"switch_end",
                             {{x, kWidthM / 2.f + dy, 0.24f}, {}},
                             {1.47f, 0.05f, 0.48f},
                             "#a0a9b0"});
    }
    boxes.push_back({"scale_tower",
                     {{kLengthM / 2.f, kWidthM / 2.f, 1.05f}, {}},
                     {0.30f, 0.30f, 2.10f},
                     "#a0a9b0"});
    for (const float x : {6.90f, kLengthM - 6.90f})
        boxes.push_back(
            {"platform", {{x, kWidthM / 2.f, 0.045f}, {}}, {1.05f, 2.64f, 0.09f}, "#9b9b9b"});

    for (const Box& box : boxes) {
        auto body = bodies_.box(
            box.pose, box.sizeM * 0.5f, s::BodyType::Static, s::layer::Field, 0,
            box.name == "carpet" ? s::materials::kCarpet : s::materials::kFieldStructure);
        if (!body) return body.error();
        s::FieldVisual visual;
        visual.name = box.name;
        visual.pose = box.pose;
        visual.sizeM = box.sizeM;
        visual.color = box.color;
        description_.visuals.push_back(std::move(visual));
    }

    for (const float x : {3.048f, kLengthM - 3.048f}) {
        s::FieldVisual line;
        line.name = "auto_line";
        line.pose.position = {x, kWidthM / 2.f, 0.002f};
        line.sizeM = {0.05f, kWidthM, 0.004f};
        line.color = "#ffffff";
        description_.visuals.push_back(std::move(line));
    }
    for (const float x : {-0.6f, kLengthM + 0.6f})
        for (const float y : {0.775f, kWidthM - 0.775f}) {
            s::FieldVisual portal;
            portal.name = "off_field_portal_supply";
            portal.pose.position = {x, y, -0.015f};
            portal.sizeM = {1.f, 1.5f, 0.03f};
            portal.color = x < 0.f ? "#c52c30" : "#2457c5";
            description_.visuals.push_back(std::move(portal));
        }
    return s::okStatus();
}

const s::FieldDescription& Field::description() const {
    return description_;
}

}  // namespace wpiemu::plugins::game_2027
```

The new settings select feeding, action period and exchange destination.
The vault option is declared here; its processing is supplied in Chapter 17.
Hold remains the default.

Listing 16.6. Human-player settings and defaults. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\config.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\config.h -->
```cpp
#pragma once

#include "wpiemu/plugins/game_2027/constants.h"

namespace wpiemu::plugins::game_2027 {
struct GameConfig {
    std::string assignment = "LRL";
    bool preload = true;
    bool timedFeed = false;
    double humanPeriodS = 2.0;
    std::string exchangeMode = "hold";
};

bool validAssignment(const std::string& value);
s::PluginConfigSchema configSchema();
GameConfig parseConfig(const s::Value& data);

}  // namespace wpiemu::plugins::game_2027
```

Listing 16.7. Shared bindings for human-player controls. <code>seasons\2027\game\src\config.cpp</code>.

<!-- book-file: seasons\2027\game\src\config.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/config.h"

#include "wpiemu/sdk/config/config_bindings.h"

namespace wpiemu::plugins::game_2027 {
namespace {

s::ConfigBindings<GameConfig> bindings() {
    s::ConfigBindings<GameConfig> b;
    b.add(s::props::choice("assignment", "Field assignment", "Scenario", "LRL",
                           {{"LLL", "LLL"}, {"LRL", "LRL"}, {"RLR", "RLR"}, {"RRR", "RRR"}}),
          &GameConfig::assignment);
    b.add(s::props::flag("preload", "One preloaded cube", "Scenario", true), &GameConfig::preload);
    b.add(s::props::flag("timedFeed", "Timed portal feeding", "Human player", false),
          &GameConfig::timedFeed);
    b.add(s::props::real("humanPeriodS", "Action period", "Human player", 2.0, 0.1, 60.0,
                         s::PropertyUnit::Seconds),
          &GameConfig::humanPeriodS);
    b.add(s::props::choice("exchangeMode", "Exchange destination", "Human player", "hold",
                           {{"hold", "Retain"}, {"return", "Upper return"}, {"vault", "Vault"}}),
          &GameConfig::exchangeMode);
    return b;
}

}  // namespace

bool validAssignment(const std::string& value) {
    return value == "LLL" || value == "LRL" || value == "RLR" || value == "RRR";
}

s::PluginConfigSchema configSchema() {
    s::PluginConfigSchema schema;
    schema.schemaVersion = 1;
    schema.properties = bindings().schema();
    return schema;
}

GameConfig parseConfig(const s::Value& data) {
    return bindings().decode(data);
}

}  // namespace wpiemu::plugins::game_2027
```

### 16.5.3 State integration

Construction passes the typed configuration to the state. Reset clears the
agent's timer. Tick order is physics, zone release, plate observation, then
teleoperation-only human collection and action.

Listing 16.8. State ownership of the human agent. <code>seasons\2027\game\include\wpiemu\plugins\game_2027\game_state.h</code>.

<!-- book-file: seasons\2027\game\include\wpiemu\plugins\game_2027\game_state.h -->
```cpp
#pragma once

#include "wpiemu/plugins/game_2027/cube_layout.h"
#include "wpiemu/plugins/game_2027/cube_pool.h"
#include "wpiemu/plugins/game_2027/cube_zones.h"
#include "wpiemu/plugins/game_2027/human_player.h"
#include "wpiemu/plugins/game_2027/ownership_scoring.h"
#include "wpiemu/plugins/game_2027/plate.h"
#include "wpiemu/plugins/game_2027/plates.h"

namespace wpiemu::plugins::game_2027 {
class State final : public s::IGameState {
public:
    State(s::SimContext& ctx, GameConfig config);

    s::Status build();

    std::string_view pluginId() const override;

    void reset(s::SimContext& ctx) override;

    void step(s::SimContext& ctx, double dt) override;

    void publish(s::SnapshotBuilder& out) const override;

    std::optional<std::string> gameSpecificMessage() const override;

    std::optional<double> gameValue(std::string_view id) const override;

private:
    PlateOwnership observedOwnership() const;
    void score(s::SimContext& ctx, double dt, bool autonomous);
    s::IPhysicsWorld& world_;
    CubePool pool_;
    Plates plates_;
    CubeZones zones_;
    OwnershipScoring scoring_;
    HumanPlayer human_;
    GameConfig config_;
    bool autoSeen_ = false;
    std::string assignment_;
    bool preload_;
    std::array<double, 2> scores_{};
    std::array<bool, 2> autoRuns_{};
    double autoSeconds_ = 0.0;
    double teleopSeconds_ = 0.0;
    double endgameSeconds_ = 0.0;
    double remainingS_ = -1.0;
    std::string phase_ = "Disabled";
};

}  // namespace wpiemu::plugins::game_2027
```

Listing 16.9. Construction with typed settings. <code>seasons\2027\game\src\game_state.cpp</code>.

<!-- book-file: seasons\2027\game\src\game_state.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/game_state.h"

namespace wpiemu::plugins::game_2027 {
State::State(s::SimContext& ctx, GameConfig config)
    : world_(ctx.physics()),
      pool_(world_),
      plates_(world_),
      zones_(pool_, world_),
      config_(std::move(config)),
      assignment_(config_.assignment),
      preload_(config_.preload) {}

s::Status State::build() {
    auto status = pool_.build();
    if (!status) return status;
    return plates_.build();
}

std::string_view State::pluginId() const {
    return kId;
}

std::optional<std::string> State::gameSpecificMessage() const {
    return assignment_;
}

std::optional<double> State::gameValue(std::string_view id) const {
    if (id == "game_2027.inventory") return pool_.inventory();
    return std::nullopt;
}

}  // namespace wpiemu::plugins::game_2027
```

Listing 16.10. Factory with the revised state constructor. <code>seasons\2027\game\src\game_module.cpp</code>.

<!-- book-file: seasons\2027\game\src\game_module.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/game_module.h"

#include "wpiemu/plugins/game_2027/game_state.h"

namespace wpiemu::plugins::game_2027 {
s::Status Module::registerMechanisms(s::MechanismFactory&) {
    return s::okStatus();
}

s::Result<std::unique_ptr<s::IField>> Module::createField(s::IPhysicsWorld& world) {
    auto field = std::make_unique<Field>(world);
    const auto status = field->build();
    if (!status) return status.error();
    return std::unique_ptr<s::IField>(std::move(field));
}

s::Result<std::unique_ptr<s::IGameState>> Module::createGameState(s::SimContext& ctx) {
    s::Value config =
        ctx.config().findPlugin(kId) ? ctx.config().findPlugin(kId)->data : s::Value::makeObject();
    const auto parsed = parseConfig(config);
    if (!validAssignment(parsed.assignment))
        return s::makeError(s::ErrorCode::InvalidArgument,
                            "Power UP assignment must be LLL, LRL, RLR or RRR");
    auto state = std::make_unique<State>(ctx, parsed);
    const auto status = state->build();
    if (!status) return status.error();
    return std::unique_ptr<s::IGameState>(std::move(state));
}

s::PluginConfigSchema Module::contributeConfigSchema() const {
    return configSchema();
}

}  // namespace wpiemu::plugins::game_2027

namespace wpiemu::plugins::game_2027 {
WPIEMU_REGISTER_GAME_PLUGIN(Module)

}  // namespace wpiemu::plugins::game_2027
```

Listing 16.11. Reset of human-action timing. <code>seasons\2027\game\src\game_reset.cpp</code>.

<!-- book-file: seasons\2027\game\src\game_reset.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/game_state.h"

namespace wpiemu::plugins::game_2027 {
void State::reset(s::SimContext& ctx) {
    scores_ = {};
    scoring_.reset();
    human_.reset();
    autoSeen_ = false;
    autoRuns_ = {};
    const auto alliance = s::proto::allianceOf(ctx.driverStation().station);
    pool_.reset(preload_, alliance == s::Alliance::Red ? 1 : 0);
    plates_.reset();
    autoSeconds_ = teleopSeconds_ = endgameSeconds_ = 0.0;
    zones_.reset();
    phase_ = "Disabled";
    remainingS_ = -1.0;
}

}  // namespace wpiemu::plugins::game_2027
```

Listing 16.12. Teleoperation gates and human actions. <code>seasons\2027\game\src\game_tick.cpp</code>.

<!-- book-file: seasons\2027\game\src\game_tick.cpp -->
```cpp
#include "wpiemu/plugins/game_2027/game_state.h"
#include "wpiemu/plugins/game_2027/match_clock.h"

namespace wpiemu::plugins::game_2027 {
void State::step(s::SimContext& ctx, double dt) {
    const auto& ds = ctx.driverStation();
    remainingS_ = ds.matchTimeS;
    const auto phase = phaseOf(ds);
    const bool running = phase != MatchPhase::Disabled;
    const bool autonomous = phase == MatchPhase::Autonomous;
    const bool endgame = phase == MatchPhase::Endgame;
    const bool teleop = phase == MatchPhase::Teleop || endgame;
    phase_ = phaseName(phase);
    if (autoSeen_ && !autonomous && !scoring_.snapshot().autoEndCaptured) scoring_.captureAutoEnd();
    autoSeen_ |= autonomous;
    if (running) {
        if (autonomous) autoSeconds_ += dt;
        if (teleop) teleopSeconds_ += dt;
        if (endgame) endgameSeconds_ += dt;
        zones_.update(ctx);
    }

    plates_.update(dt);
    if (running && teleop) human_.observe(pool_);
    if (running && teleop)
        human_.advance(pool_, static_cast<RulesTimeUs>(std::llround(dt * kRulesSecondUs)), config_);
    if (running && (autonomous || teleop)) {
        score(ctx, dt, autonomous);
    }
}

}  // namespace wpiemu::plugins::game_2027
```

Listing 16.13. Library sources through Chapter 16. <code>seasons\2027\game\CMakeLists.txt</code>.

<!-- book-file: seasons\2027\game\CMakeLists.txt -->
```cmake
add_library(wpiemu_plugin_game_2027 STATIC
    src/game_module.cpp
    src/field_layout.cpp
    src/cube_layout.cpp
    src/cube_pool.cpp
    src/plate_build.cpp
    src/plate_rules.cpp
    src/plates.cpp
    src/match_clock.cpp
    src/cube_zones.cpp
    src/config.cpp
    src/plugin_info.cpp
    src/game_ui.cpp
    src/game_values.cpp
    src/game_state.cpp
    src/game_reset.cpp
    src/game_tick.cpp
    src/game_snapshot.cpp
    src/ownership_scoring.cpp
    src/game_scoring.cpp
    src/human_player.cpp
    src/cube_return.cpp)
add_library(wpiemu::plugin_game_2027 ALIAS wpiemu_plugin_game_2027)
target_include_directories(wpiemu_plugin_game_2027 PUBLIC include)
target_link_libraries(wpiemu_plugin_game_2027 PUBLIC wpiemu_sdk)
wpiemu_target_defaults(wpiemu_plugin_game_2027)
```

## 16.6 Verification

The exchange test releases a real cube toward the aperture, waits for physical
delivery, and returns it exactly once. Feeding tests compare reserve and
on-field counts through autonomous, teleoperation, disable and reset.

Listing 16.14. Physical lower-exchange delivery and one return. <code>tests\game_2027\test_exchange.cpp</code>.

<!-- book-file: tests\game_2027\test_exchange.cpp -->
```cpp
#include <doctest/doctest.h>

#include "wpiemu/core/physics/box3d_world.h"
#include "wpiemu/plugins/game_2027/field_layout.h"
#include "wpiemu/plugins/game_2027/human_player.h"

namespace pu = wpiemu::plugins::game_2027;
namespace s = wpiemu::sdk;

TEST_CASE("Power UP: a physical delivery crosses the lower exchange and returns only once") {
    auto world = wpiemu::core::physics::createBox3dWorld();
    pu::Field field(*world);
    REQUIRE(field.build());
    pu::CubePool pool(*world);
    REQUIRE(pool.build());
    pool.reset(true, 0);
    REQUIRE(pool.release({{0.3f, 2.4f, 0.15f}, {}}, {-3.f, 0.f, 0.f}));
    pu::HumanPlayer human;
    for (int i = 0; i < 200; ++i) {
        world->step(0.005f);
        human.observe(pool);
    }
    const auto p = pool.positions(pu::CubeLocation::Field)[16];
    CAPTURE(p.x);
    CAPTURE(p.y);
    CAPTURE(p.z);
    CHECK(pool.count(pu::CubeLocation::Exchange, 0) == 1);
    CHECK(pool.positions(pu::CubeLocation::Field).size() == 32);
    pu::GameConfig config;
    config.exchangeMode = "return";
    CHECK_FALSE(human.advance(pool, 999'999, config));
    CHECK(human.advance(pool, 1'000'001, config));
    CHECK(pool.count(pu::CubeLocation::Exchange, 0) == 0);
    CHECK(pool.positions(pu::CubeLocation::Field).size() == 33);
    CHECK_FALSE(pool.returnCube(0, {}));
    CHECK(pool.released() == 1);
}
```

Listing 16.15. Phase restrictions and reserve conservation. <code>tests\game_2027\test_human_player.cpp</code>.

<!-- book-file: tests\game_2027\test_human_player.cpp -->
```cpp
#include "match_fixture.h"

using namespace power_up_test;

TEST_CASE("Power UP: timed human feeding occurs only in teleop and conserves identity") {
    Match match("LRL");
    match.game.reset();
    auto& data = match.services.config.plugins["game_2027"].data;
    data["timedFeed"] = true;
    data["humanPeriodS"] = 1.0;
    auto built = match.registry.find("game_2027")->createGameState(match.ctx);
    REQUIRE(built);
    match.game = std::move(built.value());
    match.game->reset(match.ctx);
    match.services.config.zones.clear();
    match.services.driverStation.enabled = true;
    match.services.driverStation.mode = s::proto::RobotMode::Autonomous;
    match.services.driverStation.matchTimeS = 15.0;
    match.advance(1000);
    CHECK(match.hud("game_2027.portal_blue") == 13);
    match.services.driverStation.mode = s::proto::RobotMode::Teleop;
    match.services.driverStation.matchTimeS = 135.0;
    match.advance(200);
    CHECK(match.hud("game_2027.portal_blue") == 12);
    CHECK(match.hud("game_2027.portal_red") == 13);
    CHECK(match.snapshot()->entityGroups()[0].positions.size() == 34);
    match.services.driverStation.enabled = false;
    match.advance(400);
    CHECK(match.hud("game_2027.portal_blue") == 12);
    match.game->reset(match.ctx);
    CHECK(match.hud("game_2027.portal_blue") == 13);
}

TEST_CASE("Power UP: exchange custody waits for teleop") {
    Match match("LRL");
    match.services.config.zones[0].local.position = {-3.52f, 2.4f - 5.4864f, -0.10f};
    match.services.driverStation.enabled = true;
    match.services.driverStation.mode = s::proto::RobotMode::Autonomous;
    match.services.driverStation.matchTimeS = 15.0;
    match.advance(1);
    CHECK(match.hud("game_2027.released") == 1);
    CHECK(match.snapshot()->entityGroups()[0].positions.size() == 33);
    match.services.driverStation.mode = s::proto::RobotMode::Teleop;
    match.services.driverStation.matchTimeS = 135.0;
    match.advance(1);
    CHECK(match.snapshot()->entityGroups()[0].positions.size() == 32);
}
```

## 16.7 Reproduction

Extract through Chapter 16, rebuild and run:

```powershell
ctest --test-dir build\book -R "Power UP:.*(exchange|feeding)" --no-tests=error --output-on-failure
```

In the saved model's plugin settings, enable timed feeding to observe portal
delivery. Retain the default hold policy while inspecting exchange collection.

## 16.8 Discussion

The initial single preload comes from one portal's seven cubes, leaving six
there and seven at the other portal. The model does not offer six independent
preload controls. F-04 records this single-robot setup interpretation.

## 16.9 Summary

Human actions now have a phase gate, a deterministic schedule and physical
custody inputs. Lower delivery and upper return remain distinct operations.
The vault can build on exchange custody without creating an alternate cube supply.

## 16.10 Exercises

1. Count all custody categories before delivery, after collection and after return.
2. Explain why returning through the lower aperture would violate the model.
3. Predict the first feeding time after a disabled interval interrupts the
   human-action period.

[Chapter 17](17-power-ups.md)
