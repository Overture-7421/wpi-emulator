#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <set>

#include "wpiemu/plugins/game_2026/april_tags.h"
#include "wpiemu/plugins/game_2026/field_dimensions.h"
#include "wpiemu/plugins/game_2026/field_layout.h"
#include "wpiemu/sdk/game/builtin_plugins.h"
#include "wpiemu/sdk/game/plugin_registry.h"

using namespace wpiemu;
using namespace wpiemu::plugins::game_2026;

namespace {

sdk::AprilTagLayout moduleLayout() {
    static sdk::GamePluginRegistry registry;
    static const bool registered = sdk::registerBuiltinGamePlugins(registry).ok();
    REQUIRE(registered);
    sdk::IGameModule* module = registry.findByYear(2026);
    REQUIRE(module != nullptr);
    return module->aprilTagLayout();
}

// Hub faces (the trench tags 1, 6, 7, 12, 17, 22, 23, 28 sit on the trench posts, 13-16 / 29-32 on the walls).
constexpr int kHubTagIds[] = {2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27};

}  // namespace

TEST_CASE("april tags: the module publishes the official 2026 welded layout") {
    const sdk::AprilTagLayout layout = moduleLayout();
    CHECK(layout.name == "2026-rebuilt-welded");
    CHECK(layout.fieldLengthM == doctest::Approx(16.541f));
    CHECK(layout.fieldWidthM == doctest::Approx(8.069f));
    CHECK(layout.tags.size() == 32);
    CHECK_FALSE(layout.empty());

    std::set<int> ids;
    for (const sdk::AprilTag& t : layout.tags) {
        ids.insert(t.id);
        CHECK(t.sizeM == doctest::Approx(0.1651f));
        const sdk::Quat& q = t.pose.rotation;
        CHECK(std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) == doctest::Approx(1.0).epsilon(1e-6));
        CHECK(t.pose.position.x >= 0.f);
        CHECK(t.pose.position.x <= layout.fieldLengthM);
        CHECK(t.pose.position.y >= 0.f);
        CHECK(t.pose.position.y <= layout.fieldWidthM);
    }
    CHECK(ids.size() == layout.tags.size());
    CHECK(*ids.begin() == 1);
    CHECK(*ids.rbegin() == 32);
}

TEST_CASE("april tags: known tags match 2026-rebuilt-welded.json and use the SDK quaternion order") {
    const sdk::AprilTagLayout layout = game2026AprilTagLayout();

    // JSON: ID 18, translation (4.6256194, 3.4312352, 1.12395), quaternion W=-0.7071068 X=0 Y=0 Z=0.7071068
    const sdk::AprilTag* t18 = layout.find(18);
    REQUIRE(t18 != nullptr);
    CHECK(t18->pose.position.x == doctest::Approx(4.6256194).epsilon(1e-4));
    CHECK(t18->pose.position.y == doctest::Approx(3.4312352).epsilon(1e-4));
    CHECK(t18->pose.position.z == doctest::Approx(1.12395).epsilon(1e-4));
    CHECK(t18->pose.rotation.x == doctest::Approx(0.0).epsilon(1e-4));
    CHECK(t18->pose.rotation.y == doctest::Approx(0.0).epsilon(1e-4));
    CHECK(t18->pose.rotation.z == doctest::Approx(0.7071068).epsilon(1e-4));
    CHECK(t18->pose.rotation.w == doctest::Approx(-0.7071068).epsilon(1e-4));

    // JSON: ID 26, translation (4.0218614, 4.0346376, 1.12395), quaternion W=0 X=0 Y=0 Z=1
    const sdk::AprilTag* t26 = layout.find(26);
    REQUIRE(t26 != nullptr);
    CHECK(t26->pose.position.x == doctest::Approx(4.0218614).epsilon(1e-4));
    CHECK(t26->pose.position.y == doctest::Approx(4.0346376).epsilon(1e-4));
    CHECK(t26->pose.rotation.z == doctest::Approx(1.0).epsilon(1e-4));
    CHECK(std::abs(t26->pose.rotation.w) < 1e-4f);

    // JSON: ID 32, translation (0.0080772, 4.1775126, 0.55245), identity rotation
    const sdk::AprilTag* t32 = layout.find(32);
    REQUIRE(t32 != nullptr);
    CHECK(t32->pose.position.x == doctest::Approx(0.0080772).epsilon(1e-4));
    CHECK(t32->pose.position.y == doctest::Approx(4.1775126).epsilon(1e-4));
    CHECK(t32->pose.position.z == doctest::Approx(0.55245).epsilon(1e-4));
    CHECK(t32->pose.rotation.w == doctest::Approx(1.0).epsilon(1e-4));

    CHECK(layout.find(0) == nullptr);
    CHECK(layout.find(33) == nullptr);
}

TEST_CASE("april tags: hub faces point away from their hub and the tags share the plugin's field frame") {
    const sdk::AprilTagLayout layout = game2026AprilTagLayout();
    const FieldLayout field = buildFieldLayout();

    for (const int id : kHubTagIds) {
        const sdk::AprilTag* tag = layout.find(id);
        REQUIRE(tag != nullptr);
        const HubSpec* nearest = &field.hubs[0];
        for (const HubSpec& h : field.hubs) {
            const auto d = [&](const HubSpec& s) {
                return std::hypot(tag->pose.position.x - s.center.x, tag->pose.position.y - s.center.y);
            };
            if (d(h) < d(*nearest)) nearest = &h;
        }
        const sdk::Vec3 out = tag->pose.transformVector({1.f, 0.f, 0.f});
        const float dx = tag->pose.position.x - nearest->center.x;
        const float dy = tag->pose.position.y - nearest->center.y;
        INFO("tag " << id);
        CHECK(std::abs(out.z) < 1e-5f);          // faces are vertical
        CHECK(out.x * dx + out.y * dy > 0.5f * dim::kHubSizeM - 0.05f);  // pointing out, at the face distance
        CHECK(std::hypot(dx, dy) < 0.5f * dim::kHubSizeM + 0.15f);   // and on the hub footprint
    }
}

TEST_CASE("april tags: wall tags face into the field") {
    const sdk::AprilTagLayout layout = game2026AprilTagLayout();
    for (const int id : {29, 30, 31, 32}) {  // blue wall: outpost and tower
        const sdk::Vec3 out = layout.find(id)->pose.transformVector({1.f, 0.f, 0.f});
        CHECK(out.x == doctest::Approx(1.0).epsilon(1e-5));
    }
    for (const int id : {13, 14, 15, 16}) {  // red wall
        const sdk::Vec3 out = layout.find(id)->pose.transformVector({1.f, 0.f, 0.f});
        CHECK(out.x == doctest::Approx(-1.0).epsilon(1e-5));
    }
    // The plugin's tower centre line is tag 31's y; the red wall tags are the 180 degree rotated copies.
    CHECK(layout.find(31)->pose.position.y == doctest::Approx(dim::kTowerCenterYM).epsilon(1e-3));
    CHECK(layout.find(15)->pose.position.y == doctest::Approx(dim::kFieldWidthM - dim::kTowerCenterYM).epsilon(1e-3));
}