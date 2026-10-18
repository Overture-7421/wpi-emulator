// The vision engine against the REAL 2026 plugin: the Box3D world holding the plugin's field colliders
// and its AprilTag layouts (tests may link anything; the engine itself only sees the SDK).
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <set>

#include "vision_rig.h"
#include "wpiemu/core/vision/visibility.h"
#include "wpiemu/plugins/game_2026/field_layout.h"
#include "wpiemu/plugins/game_2026/fiducial_layouts.h"

using namespace vision_test;
using doctest::Approx;
namespace g26 = wpiemu::plugins::game_2026;

namespace {

// The ids of the 8 trench tags of a 2026 layout: they sit at the mid-plane of the trench-arm boxes.
const std::set<int> kTrenchTags = {1, 6, 7, 12, 17, 22, 23, 28};

struct FieldRig {
    BoxWorld world;
    std::unique_ptr<g26::Game2026Field> field;
    sdk::FiducialLayout layout;

    explicit FieldRig(std::size_t layoutIndex = 0) {
        auto created = g26::Game2026Field::create(*world.world, g26::buildFieldLayout());
        REQUIRE(created.ok());
        field = std::move(created).value();
        const auto layouts = g26::buildFiducialLayouts();
        REQUIRE(layouts.size() > layoutIndex);
        layout = layouts[layoutIndex];
    }

    // The tag ids the camera sees in its first capture from `position` looking along `yawRad`.
    std::vector<int> idsFrom(const Vec3d& position, double yawRad, bool occlusion = true) {
        sdk::CameraConfig cam = makeCamera();
        cam.occlusion = occlusion;
        cam.frameRateHz = 200.0;
        cam.latencyUs = 0;
        auto created = vis::VisionSystem::create(visionOf({cam}), layout, 5000);
        REQUIRE(created.ok());
        auto system = std::move(created).value();
        world.world->setPose(world.chassis, vis::toSdk(poseYpr(position, yawRad)));
        sdk::Rng rng(1);
        system->step(viewOf(world), rng, 5000);
        std::vector<sdk::VisionFrame> out;
        system->collectDue(10000, out);
        REQUIRE(out.size() == 1);
        std::vector<int> ids;
        for (const auto& o : out[0].observations) ids.push_back(o.id);
        std::sort(ids.begin(), ids.end());
        return ids;
    }

    vis::TagRejection whyNot(int tagId, const Vec3d& position, double yawRad) {
        const sdk::FiducialTag* tag = layout.find(tagId);
        REQUIRE(tag != nullptr);
        const vis::CameraModel model(makeCamera().intrinsics);
        return vis::evaluateTagVisibility(model, vis::VisibilityLimits{12.0, 1.309, true}, poseYpr(position, yawRad),
                                          vis::fromSdk(tag->pose), layout.tagSizeM, world.world.get())
            .rejection;
    }
};

bool contains(const std::vector<int>& ids, int id) { return std::find(ids.begin(), ids.end(), id) != ids.end(); }

}  // namespace

TEST_CASE("vision game_2026: the plugin field is where the AprilTag layouts expect it") {
    FieldRig rig;
    const auto layouts = g26::buildFiducialLayouts();
    REQUIRE(layouts.size() == 2);
    for (std::size_t li = 0; li < layouts.size(); ++li) {
        const sdk::FiducialLayout& layout = layouts[li];
        const bool welded = li == 0;  // the colliders are built from the welded field
        std::set<int> embedded;
        for (const sdk::FiducialTag& tag : layout.tags) {
            const Pose3d pose = vis::fromSdk(tag.pose);
            const Vec3d normal = vis::tagNormal(pose);
            // From 0.6 m in front of the tag straight at it: where is the face of the collider it hangs on?
            const sdk::RayHit hit = rig.world.world->rayCastClosest(vis::toSdk(pose.t + normal * 0.6), vis::toSdk(-normal),
                                                                    1.2f, sdk::layer::Field);
            INFO("layout " << layout.id << ", tag " << tag.id);
            REQUIRE(hit.hit);  // every tag has a collider behind it, none floats in free space
            const double faceToCentreM = 0.6 - static_cast<double>(hit.fraction) * 1.2;  // > 0: the centre is inside
            if (faceToCentreM > vis::kOcclusionStandoffM) {
                embedded.insert(tag.id);
                CHECK(faceToCentreM > 0.5);  // the mid-plane of the 1.19 m deep trench-arm box
                CHECK(faceToCentreM < 0.6);
            } else {
                // hub and alliance wall tags: on the collider face to within the modelling tolerance
                CHECK(std::abs(faceToCentreM) < (welded ? 0.01 : 0.05));
            }
        }
        CHECK(embedded == kTrenchTags);
    }
}

TEST_CASE("vision game_2026: a trench tag and a hub tag are visible from in front and hidden from behind the structure") {
    FieldRig rig;
    const double kPiD = kPi;

    // Red low-Y trench: tag 7 faces +X, tag 6 faces -X (they are back to back on the arm).
    {
        const auto front = rig.idsFrom({14.0, 0.644, 0.889}, kPiD);  // in the red zone, looking at the arm
        CHECK(contains(front, 7));
        CHECK_FALSE(contains(front, 6));
        const auto behind = rig.idsFrom({10.0, 0.644, 0.889}, 0.0);  // the neutral-zone side
        CHECK(contains(behind, 6));
        CHECK_FALSE(contains(behind, 7));
        // seen from behind, the reason is the tag facing away, never the arm that hides it
        CHECK(rig.whyNot(7, {10.0, 0.644, 0.889}, 0.0) == vis::TagRejection::Facing);
        CHECK(rig.whyNot(7, {14.0, 0.644, 0.889}, kPiD) == vis::TagRejection::None);
    }
    // Blue high-Y trench (ids 22 / 23 at y = 7.425): 22 faces +X, 23 faces -X.
    {
        const auto front = rig.idsFrom({7.0, 7.425, 0.889}, kPiD);
        CHECK(contains(front, 22));
        CHECK_FALSE(contains(front, 23));
        const auto behind = rig.idsFrom({2.5, 7.425, 0.889}, 0.0);
        CHECK(contains(behind, 23));
        CHECK_FALSE(contains(behind, 22));
    }
    // Blue hub, +X face: tags 19 / 20 face the neutral zone, 25 / 26 (x = 4.022) face the alliance wall.
    {
        const auto front = rig.idsFrom({7.5, 3.857, 1.124}, kPiD);
        CHECK(contains(front, 19));
        CHECK(contains(front, 20));
        CHECK_FALSE(contains(front, 25));
        const auto behind = rig.idsFrom({2.5, 3.857, 1.124}, 0.0);
        CHECK(contains(behind, 25));
        CHECK(contains(behind, 26));
        CHECK_FALSE(contains(behind, 19));
        // the hub body is between this camera and tag 19, and the tag faces away from it
        CHECK(rig.whyNot(19, {2.5, 3.857, 1.124}, 0.0) == vis::TagRejection::Facing);
    }
}

TEST_CASE("vision game_2026: a tower upright hides the wall tag behind it and not from the side") {
    FieldRig rig;
    // The uprights stand at y = 3.315 and 4.172 in front of the blue wall tags 31 (y 3.746) and 32 (y 4.178).
    const auto lined = rig.idsFrom({3.0, 4.178, 0.552}, kPi);  // tag 32 straight behind the upright
    CHECK(contains(lined, 31));
    CHECK_FALSE(contains(lined, 32));
    CHECK(rig.whyNot(32, {3.0, 4.178, 0.552}, kPi) == vis::TagRejection::Occluded);

    const auto between = rig.idsFrom({3.0, 3.75, 0.552}, kPi);  // between the uprights
    CHECK(contains(between, 31));
    CHECK(contains(between, 32));

    // without the occlusion switch the camera ignores the structure
    CHECK(contains(rig.idsFrom({3.0, 4.178, 0.552}, kPi, false), 32));
}
