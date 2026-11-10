// The 2026 plugin and the robot tags of practice matches (sdk/physics/robot_tags.h) share the physics user tag space. The
// plugin must stay oblivious to robot-tagged shapes without being changed.
#include <doctest/doctest.h>

#include "test_support.h"
#include "wpiemu/plugins/game_2026/field_dimensions.h"
#include "wpiemu/plugins/game_2026/game_state.h"
#include "wpiemu/sdk/physics/robot_tags.h"

using namespace wpiemu;
using namespace wpiemu::plugins::game_2026;
using g26test::emptyField;
using g26test::Session;

namespace {

sdk::Vec3 hubSensorCentre(Alliance a) {
    const FieldLayout layout = buildFieldLayout();
    return {layout.hub(a).center.x, layout.hub(a).center.y, dim::kHubSensorCenterZM};
}

void autoEnabled(Session& s) {
    s.env.driverStation.enabled = true;
    s.env.driverStation.mode = sdk::proto::RobotMode::Autonomous;
    s.env.driverStation.matchTimeS = 15.0;
}

// A body parked in the middle of a hub sensor. Hub sensors only see the Fuel layer, so this shape is on it (the worst case)
// to prove the plugin judges the visitor by its tag, not by what it happens to touch.
sdk::ShapeId parkRobotShapeInHub(Session& s, std::uint64_t tag, Alliance hub) {
    sdk::BodyDesc body;
    body.type = sdk::BodyType::Kinematic;
    body.pose.position = hubSensorCentre(hub);
    const sdk::BodyId id = s.env.physics.createBody(body);
    sdk::ShapeDesc shape;
    shape.geometry = sdk::SphereShape{0.1f};
    shape.category = sdk::layer::Fuel;
    shape.mask = sdk::layer::All;
    shape.sensorEvents = true;
    shape.userTag = tag;
    return s.env.physics.addShape(id, shape);
}

}  // namespace

TEST_CASE("robot tags never collide with a game_2026 tag kind") {
    for (const EntityKind kind : {EntityKind::None, EntityKind::Fuel, EntityKind::HubSensor, EntityKind::CorralSensor})
        for (const Alliance alliance : kAlliances)
            for (const std::uint32_t id : {0u, 1u, 7u, 0xFFFFu, 0xFFFFFFFFu}) {
                const std::uint64_t tag = makeTag(kind, alliance, id);
                CHECK_FALSE(sdk::isRobotTag(tag));
                CHECK(sdk::robotTagParticipant(tag) == sdk::match::kNoParticipant);
            }
    for (std::uint32_t participant = 0; participant <= 0xFFFFu; participant += 257) {
        const std::uint64_t tag = sdk::makeRobotTag(static_cast<sdk::match::ParticipantId>(participant));
        const EntityKind kind = tagKind(tag);
        CHECK(kind != EntityKind::None);
        CHECK(kind != EntityKind::Fuel);
        CHECK(kind != EntityKind::HubSensor);
        CHECK(kind != EntityKind::CorralSensor);
    }
}

TEST_CASE("the plugin ignores robot-tagged shapes in its sensors and still scores fuel") {
    Session s;
    emptyField(s);
    s.start();
    autoEnabled(s);

    const sdk::ShapeId hostShape = parkRobotShapeInHub(s, sdk::makeRobotTag(sdk::match::kHostParticipantId), Alliance::Blue);
    const sdk::ShapeId remoteShape = parkRobotShapeInHub(s, sdk::makeRobotTag(3), Alliance::Red);
    REQUIRE(hostShape.valid());
    REQUIRE(remoteShape.valid());

    // The robots do enter the sensors: the events exist and carry the robot tags, the plugin just does not act on them.
    bool sawRobotVisitor = false;
    for (int i = 0; i < 5; ++i) {
        s.tick();
        for (const sdk::SensorEvent& e : s.env.physics.sensorEvents())
            if (e.begin && sdk::isRobotTag(e.visitorTag) && tagKind(e.sensorTag) == EntityKind::HubSensor) sawRobotVisitor = true;
    }
    CHECK(sawRobotVisitor);
    CHECK(s.game->score(Alliance::Blue).points() == 0);
    CHECK(s.game->score(Alliance::Red).points() == 0);
    CHECK(s.game->fuelOnField() == 0);

    s.game->pool().spawn(hubSensorCentre(Alliance::Blue), {});
    s.tick();
    CHECK(s.game->score(Alliance::Blue).points() == 1);
    CHECK(s.game->score(Alliance::Red).points() == 0);
}
