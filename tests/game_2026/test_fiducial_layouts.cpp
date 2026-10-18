// The 2026 AprilTag layouts (src/fiducial_layouts.cpp, generated from WPILib's JSON by
// tools/gen_fiducial_layout.py). The spot checks quote the JSON digit for digit, so a mixed-up
// column in the generated tables fails here; the other tests hold the poses against the plugin's
// field model.
#include <doctest/doctest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <set>
#include <string>
#include <vector>

#include "wpiemu/plugins/game_2026/fiducial_layouts.h"
#include "wpiemu/plugins/game_2026/field_dimensions.h"
#include "wpiemu/plugins/game_2026/field_layout.h"
#include "wpiemu/plugins/game_2026/plugin_info.h"
#include "wpiemu/sdk/game/builtin_plugins.h"
#include "wpiemu/sdk/game/plugin_registry.h"

using namespace wpiemu;
using namespace wpiemu::plugins::game_2026;

namespace {

// Both WPILib resources hold 32 tags (tools/gen_fiducial_layout.py prints the count).
constexpr std::size_t kTagCount = 32;

// WPILib numbers the red alliance's tags 1-16 and the blue alliance's 17-32.
struct AllianceTags {
    std::vector<int> hub;     // two tags per hub face, outward normals along +-X and +-Y
    std::vector<int> trench;  // next to each guardrail, back to back along X
    std::vector<int> wall;    // outpost and tower tags on the alliance wall
};
const AllianceTags kRedTags{{2, 3, 4, 5, 8, 9, 10, 11}, {1, 6, 7, 12}, {13, 14, 15, 16}};
const AllianceTags kBlueTags{{18, 19, 20, 21, 24, 25, 26, 27}, {17, 22, 23, 28}, {29, 30, 31, 32}};

// The tags whose outward normal points AWAY from the field centre. All of them are on a hub or a
// trench, which stand free of the walls: a hub looks away on its alliance-wall face and on both
// sideways (+-Y) faces (it is centred on the field's long axis), a trench on its alliance-wall
// face. Every other tag, including every wall tag, looks into the field.
const std::set<int> kFacesAwayFromCentre = {2,  5,  7,  8,  9,  10, 11, 12,
                                            18, 21, 23, 24, 25, 26, 27, 28};

std::vector<sdk::FiducialLayout> layouts() {
    std::vector<sdk::FiducialLayout> all = buildFiducialLayouts();
    REQUIRE(all.size() == 2);
    return all;
}

const sdk::FiducialTag& tagAt(const sdk::FiducialLayout& layout, int id) {
    const sdk::FiducialTag* tag = layout.find(id);
    REQUIRE_MESSAGE(tag != nullptr, "tag " << id << " in " << layout.id);
    return *tag;
}

sdk::Vec3 outwardNormal(const sdk::FiducialTag& tag) {
    return tag.pose.rotation.rotate({1.f, 0.f, 0.f});
}

// Dot product, in the field plane, of the tag's outward normal with the direction to (x, y).
float facing(const sdk::FiducialTag& tag, float x, float y) {
    const sdk::Vec3 n = outwardNormal(tag);
    return n.x * (x - tag.pose.position.x) + n.y * (y - tag.pose.position.y);
}

const AllianceTags& tagsOf(Alliance alliance) {
    return alliance == Alliance::Red ? kRedTags : kBlueTags;
}

// The four planes a hub's tags lie in.
struct HubPlanes {
    float xPlus = 0.f, xMinus = 0.f, yPlus = 0.f, yMinus = 0.f;
    float centerX() const { return (xPlus + xMinus) / 2; }
    float centerY() const { return (yPlus + yMinus) / 2; }
    float halfX() const { return (xPlus - xMinus) / 2; }
    float halfY() const { return (yPlus - yMinus) / 2; }
};

HubPlanes hubPlanes(const sdk::FiducialLayout& layout, const std::vector<int>& hubIds) {
    std::array<std::vector<float>, 4> coords;  // +X, -X, +Y, -Y faces
    for (const int id : hubIds) {
        const sdk::FiducialTag& tag = tagAt(layout, id);
        const sdk::Vec3 n = outwardNormal(tag);
        const sdk::Vec3& p = tag.pose.position;
        if (n.x > 0.5f) {
            coords[0].push_back(p.x);
        } else if (n.x < -0.5f) {
            coords[1].push_back(p.x);
        } else if (n.y > 0.5f) {
            coords[2].push_back(p.y);
        } else {
            coords[3].push_back(p.y);
        }
    }
    std::array<float, 4> plane{};
    for (std::size_t i = 0; i < coords.size(); ++i) {
        REQUIRE_FALSE(coords[i].empty());
        const float sum = std::accumulate(coords[i].begin(), coords[i].end(), 0.f);
        plane[i] = sum / static_cast<float>(coords[i].size());
        for (const float c : coords[i])
            CHECK(std::abs(c - plane[i]) < 1e-4f);  // a face is one plane
    }
    return {plane[0], plane[1], plane[2], plane[3]};
}

struct Expected {
    int id;
    float x, y, z;
    float qx, qy, qz, qw;
};

// Quoted from the JSON resources: translation x y z, quaternion X Y Z W.
// clang-format off
const Expected kWeldedSpot[] = {
    {1, 11.8779798f, 7.4247756f, 0.889f,
     0.0f, 0.0f, 1.0f, 6.123233995736766e-17f},
    {2, 11.9154194f, 4.638039999999999f, 1.12395f,
     0.0f, 0.0f, 0.7071067811865476f, 0.7071067811865476f},
    {5, 11.9154194f, 3.4312351999999997f, 1.12395f,
     -0.0f, 0.0f, 0.7071067811865476f, -0.7071067811865475f},
    {7, 11.9528844f, 0.6444996f, 0.889f,
     0.0f, 0.0f, 0.0f, 1.0f},
    {13, 16.5333172f, 7.4033126f, 0.55245f,
     0.0f, 0.0f, 1.0f, 6.123233995736766e-17f},
    {20, 5.229174199999999f, 4.0346376f, 1.12395f,
     0.0f, 0.0f, 0.0f, 1.0f},
    {29, 0.0077469999999999995f, 0.6659626f, 0.55245f,
     0.0f, 0.0f, 0.0f, 1.0f},
    {31, 0.0080772f, 3.7457125999999996f, 0.55245f,
     0.0f, 0.0f, 0.0f, 1.0f},
};
const Expected kAndyMarkSpot[] = {
    {1, 11.863959f, 7.411491399999999f, 0.889f,
     0.0f, 0.0f, 1.0f, 6.123233995736766e-17f},
    {2, 11.9013986f, 4.6247558f, 1.12395f,
     0.0f, 0.0f, 0.7071067811865476f, 0.7071067811865476f},
    {5, 11.9013986f, 3.417951f, 1.12395f,
     -0.0f, 0.0f, 0.7071067811865476f, -0.7071067811865475f},
    {12, 11.9388636f, 7.411491399999999f, 0.889f,
     0.0f, 0.0f, 0.0f, 1.0f},
    {16, 16.4989764f, 3.8806881999999994f, 0.55245f,
     0.0f, 0.0f, 1.0f, 6.123233995736766e-17f},
    {19, 5.2151534f, 3.6657534f, 1.12395f,
     0.0f, 0.0f, 0.0f, 1.0f},
    {29, 0.0136906f, 0.6507734f, 0.55245f,
     0.0f, 0.0f, 0.0f, 1.0f},
    {32, 0.0140462f, 4.1619931999999995f, 0.55245f,
     0.0f, 0.0f, 0.0f, 1.0f},
};
// clang-format on

template <std::size_t N>
void checkSpot(const sdk::FiducialLayout& layout, const Expected (&rows)[N]) {
    CAPTURE(layout.id);
    for (const Expected& e : rows) {
        CAPTURE(e.id);
        const sdk::FiducialTag& tag = tagAt(layout, e.id);
        CHECK(tag.pose.position.x == e.x);
        CHECK(tag.pose.position.y == e.y);
        CHECK(tag.pose.position.z == e.z);
        CHECK(tag.pose.rotation.x == e.qx);
        CHECK(tag.pose.rotation.y == e.qy);
        CHECK(tag.pose.rotation.z == e.qz);
        CHECK(tag.pose.rotation.w == e.qw);
    }
}

}  // namespace

TEST_CASE("the 2026 module contributes the welded layout first and the AndyMark layout second") {
    sdk::GamePluginRegistry registry;
    REQUIRE(sdk::registerBuiltinGamePlugins(registry).ok());
    sdk::IGameModule* module = registry.findByYear(2026);
    REQUIRE(module != nullptr);

    const std::vector<sdk::FiducialLayout> contributed = module->contributeFiducialLayouts();
    REQUIRE(contributed.size() == 2);
    // The first layout is the plugin default (an empty VisionConfig::layoutId resolves to it).
    CHECK(contributed[0].id == "game_2026.welded");
    CHECK(contributed[0].displayName == "REBUILT 2026 (welded)");
    CHECK(contributed[1].id == "game_2026.andymark");
    CHECK(contributed[1].displayName == "REBUILT 2026 (AndyMark)");
    CHECK(contributed == buildFiducialLayouts());

    for (const sdk::FiducialLayout& layout : contributed) {
        CAPTURE(layout.id);
        CHECK(layout.id.rfind(std::string(pluginId()) + ".", 0) == 0);
        CHECK(layout.tagSizeM == 0.1651);  // 6.5 in
        CHECK(layout.tags.size() == kTagCount);
        for (const sdk::FiducialTag& tag : layout.tags)
            CHECK(tag.sizeM == 0.0);  // the layout's size
    }
}

TEST_CASE("AprilTag field sizes: welded equals the plugin field, AndyMark is the smaller one") {
    const std::vector<sdk::FiducialLayout> all = layouts();
    CHECK(all[0].fieldLengthM == dim::kFieldLengthM);
    CHECK(all[0].fieldWidthM == dim::kFieldWidthM);
    const FieldLayout field = buildFieldLayout();
    CHECK(all[0].fieldLengthM == field.description.lengthM);
    CHECK(all[0].fieldWidthM == field.description.widthM);
    // The AndyMark field is 23 mm shorter and 26 mm narrower; its tags are measured on that field.
    CHECK(all[1].fieldLengthM == 16.518f);
    CHECK(all[1].fieldWidthM == 8.043f);
}

TEST_CASE("tags 1-32 appear once per layout, red 1-16 on the red half and blue 17-32 on blue") {
    for (const sdk::FiducialLayout& layout : layouts()) {
        CAPTURE(layout.id);
        std::set<int> ids;
        for (const sdk::FiducialTag& tag : layout.tags) {
            CAPTURE(tag.id);
            CHECK(ids.insert(tag.id).second);
            const bool red = tag.id <= 16;
            const float x = tag.pose.position.x;
            CHECK((red ? x > layout.fieldLengthM / 2 : x < layout.fieldLengthM / 2));
            // inside the field, the whole tag above the carpet and below the alliance wall's top
            CHECK(x > 0.f);
            CHECK(x < layout.fieldLengthM);
            CHECK(tag.pose.position.y > 0.f);
            CHECK(tag.pose.position.y < layout.fieldWidthM);
            CHECK(tag.pose.position.z - static_cast<float>(layout.tagSizeM) / 2 > 0.f);
            CHECK(tag.pose.position.z + static_cast<float>(layout.tagSizeM) / 2 <
                  dim::kAllianceWallHeightM);
        }
        CHECK(ids.size() == kTagCount);
        CHECK(*ids.begin() == 1);
        CHECK(*ids.rbegin() == static_cast<int>(kTagCount));
    }
}

TEST_CASE("AprilTag poses match the WPILib JSON digit for digit") {
    const std::vector<sdk::FiducialLayout> all = layouts();
    checkSpot(all[0], kWeldedSpot);
    checkSpot(all[1], kAndyMarkSpot);
}

TEST_CASE("every AprilTag is upright with a unit quaternion and a horizontal axis-aligned normal") {
    for (const sdk::FiducialLayout& layout : layouts()) {
        CAPTURE(layout.id);
        for (const sdk::FiducialTag& tag : layout.tags) {
            CAPTURE(tag.id);
            // All 2026 tags are yaw-only: vertical faces looking along +-X or +-Y, tag +Z is up.
            const sdk::Quat& q = tag.pose.rotation;
            CHECK(std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) ==
                  doctest::Approx(1.0).epsilon(1e-6));
            const sdk::Vec3 normal = outwardNormal(tag);
            CHECK(std::abs(normal.z) < 1e-6f);
            CHECK(std::max(std::abs(normal.x), std::abs(normal.y)) ==
                  doctest::Approx(1.0).epsilon(1e-6));
            CHECK(tag.pose.rotation.rotate({0.f, 0.f, 1.f}).z ==
                  doctest::Approx(1.0).epsilon(1e-6));
        }
    }
}

TEST_CASE("wall tags face into the field, hub and trench tags face away from their element") {
    const FieldLayout field = buildFieldLayout();
    for (const sdk::FiducialLayout& layout : layouts()) {
        CAPTURE(layout.id);
        const float centerX = layout.fieldLengthM / 2, centerY = layout.fieldWidthM / 2;
        for (const Alliance alliance : kAlliances) {
            const AllianceTags& tags = tagsOf(alliance);
            const sdk::Vec3 hub = field.hub(alliance).center;
            const float wallX = alliance == Alliance::Red ? layout.fieldLengthM : 0.f;
            for (const int id : tags.wall) {
                CAPTURE(id);
                const sdk::FiducialTag& tag = tagAt(layout, id);
                // on the wall, a few mm proud of it
                CHECK(std::abs(tag.pose.position.x - wallX) < 0.025f);
                CHECK(facing(tag, centerX, centerY) > 0.f);
                CHECK(std::abs(outwardNormal(tag).x) == doctest::Approx(1.0).epsilon(1e-6));
            }
            for (const int id : tags.hub) {
                CAPTURE(id);
                CHECK(facing(tagAt(layout, id), hub.x, hub.y) < 0.f);
            }
            for (const int id : tags.trench) {
                CAPTURE(id);
                const sdk::FiducialTag& tag = tagAt(layout, id);
                // back to back about the trench's centre line (the hub's x), next to a guardrail
                CHECK(facing(tag, hub.x, tag.pose.position.y) < 0.f);
                CHECK(std::abs(tag.pose.position.x - hub.x) < 0.06f);
                CHECK(std::min(tag.pose.position.y, layout.fieldWidthM - tag.pose.position.y) < 1.f);
            }
        }
    }
}

TEST_CASE("only far hub and trench faces look away from the field centre") {
    for (const sdk::FiducialLayout& layout : layouts()) {
        CAPTURE(layout.id);
        std::set<int> lookingAway;
        for (const sdk::FiducialTag& tag : layout.tags)
            if (facing(tag, layout.fieldLengthM / 2, layout.fieldWidthM / 2) < 0.f)
                lookingAway.insert(tag.id);
        CHECK(lookingAway == kFacesAwayFromCentre);
    }
}

TEST_CASE("hub tags stand on the 47 in hub faces around the centre field_dimensions.h states") {
    const FieldLayout field = buildFieldLayout();
    const std::vector<sdk::FiducialLayout> all = layouts();
    for (const sdk::FiducialLayout& layout : all) {
        CAPTURE(layout.id);
        // The AndyMark field is a few cm smaller than the plugin's: its hubs are centred on its own
        // long axis (y) but 14 mm closer to the blue wall (x) than the physical hub.
        const bool welded = &layout == &all[0];
        const float toleranceX = welded ? 0.001f : 0.02f;
        for (const Alliance alliance : kAlliances) {
            CAPTURE(allianceName(alliance));
            const HubPlanes hub = hubPlanes(layout, tagsOf(alliance).hub);
            // each face stands 5-8 mm proud of the 47 in footprint ([AT] faces 4.022 / 5.229 m)
            CHECK(hub.halfX() - dim::kHubSizeM / 2 > 0.005f);
            CHECK(hub.halfX() - dim::kHubSizeM / 2 < 0.008f);
            CHECK(hub.halfY() - dim::kHubSizeM / 2 > 0.005f);
            CHECK(hub.halfY() - dim::kHubSizeM / 2 < 0.008f);
            const float expectedX = alliance == Alliance::Red
                                        ? dim::kFieldLengthM - dim::kHubCenterXM
                                        : dim::kHubCenterXM;
            CHECK(std::abs(hub.centerX() - expectedX) < toleranceX);
            CHECK(std::abs(hub.centerX() - field.hub(alliance).center.x) < toleranceX);
            CHECK(std::abs(hub.centerY() - layout.fieldWidthM / 2) < 0.001f);
            if (welded) CHECK(std::abs(hub.centerY() - dim::kHubCenterYM) < 0.001f);
        }
    }
}

TEST_CASE("welded outpost and tower tags agree with the AprilTag centres in field_dimensions.h") {
    const sdk::FiducialLayout welded = layouts()[0];
    const float width = dim::kFieldWidthM;
    // tag 29 is centred on the blue outpost openings, tag 31 on the blue tower ([AT] constants)
    CHECK(std::abs(tagAt(welded, 29).pose.position.y - dim::kOutpostCenterYM) < 0.001f);
    CHECK(std::abs(tagAt(welded, 13).pose.position.y - (width - dim::kOutpostCenterYM)) < 0.001f);
    CHECK(std::abs(tagAt(welded, 31).pose.position.y - dim::kTowerCenterYM) < 0.003f);
    CHECK(std::abs(tagAt(welded, 15).pose.position.y - (width - dim::kTowerCenterYM)) < 0.003f);
    // the wall tags are a few mm in front of the wall planes at x = 0 and x = length
    for (const int id : {29, 30, 31, 32})
        CHECK(std::abs(tagAt(welded, id).pose.position.x) < 0.01f);
    for (const int id : {13, 14, 15, 16})
        CHECK(std::abs(dim::kFieldLengthM - tagAt(welded, id).pose.position.x) < 0.01f);
}

TEST_CASE("red AprilTags are the blue ones rotated 180 degrees about the field centre") {
    const std::vector<sdk::FiducialLayout> all = layouts();
    // WPILib's AndyMark data is not exactly symmetric: every red tag is 5 mm closer to the blue wall
    // than the mirror image of its blue partner. The welded data mirrors to 0.3 mm.
    const std::array<float, 2> tolerance{0.0005f, 0.006f};
    for (std::size_t i = 0; i < all.size(); ++i) {
        CAPTURE(all[i].id);
        for (int red = 1; red <= 16; ++red) {
            CAPTURE(red);
            const sdk::FiducialTag& r = tagAt(all[i], red);
            const sdk::FiducialTag& b = tagAt(all[i], red + 16);
            CHECK(std::abs(r.pose.position.x - (all[i].fieldLengthM - b.pose.position.x)) <
                  tolerance[i]);
            CHECK(std::abs(r.pose.position.y - (all[i].fieldWidthM - b.pose.position.y)) <
                  tolerance[i]);
            CHECK(r.pose.position.z == b.pose.position.z);
            const sdk::Vec3 rn = outwardNormal(r), bn = outwardNormal(b);
            CHECK(std::abs(rn.x + bn.x) < 1e-6f);
            CHECK(std::abs(rn.y + bn.y) < 1e-6f);
        }
    }
}
