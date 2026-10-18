#include <doctest/doctest.h>

#include <cmath>

#include "wpiemu/core/runtime/vision_system.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/mechanisms/mechanism.h"

using namespace wpiemu::sdk;
using namespace wpiemu::core::runtime;

namespace {

constexpr SimTimeUs kStepUs = 5000;
constexpr float kPif = static_cast<float>(kPi);

class FakeDirectory final : public IMechanismDirectory {
public:
    IMechanism* find(std::string_view) override { return nullptr; }
    std::vector<IMechanism*> all() override { return {}; }
    std::optional<Pose3> linkPoseInChassis(std::string_view mechanismId, std::string_view link) const override {
        if (mechanismId == "turret" && link == "tip") return linkPose;
        return std::nullopt;
    }
    Pose3 linkPose;
};

// A camera 1 m above the floor looking along +X from the chassis origin (the COM, 0.2 m above the floor).
CameraConfig cameraAt(const std::string& id) {
    CameraConfig c;
    c.id = id;
    c.name = id;
    c.local = Pose3{Vec3{0.f, 0.f, 0.8f}, Quat{}};
    c.hFovDeg = 60.0;
    c.vFovDeg = 40.0;
    c.maxRangeM = 6.0;
    c.maxIncidenceDeg = 75.0;
    c.fps = 200.0;  // every step
    c.latencyMs = 0.0;
    return c;
}

// A tag at `position` whose face points back at the origin when yawOffsetRad == 0.
AprilTag tagAt(int id, Vec3 position, float yawOffsetRad = 0.f) {
    AprilTag t;
    t.id = id;
    t.pose = Pose3{position, Quat::fromYaw(kPif + yawOffsetRad)};
    return t;
}

RobotConfig configWith(std::vector<CameraConfig> cameras) {
    RobotConfig c;
    c.cameras = std::move(cameras);
    return c;
}

AprilTagLayout layoutOf(std::vector<AprilTag> tags) {
    AprilTagLayout l;
    l.name = "test";
    l.tags = std::move(tags);
    return l;
}

const Pose3 kSpawn{Vec3{0.f, 0.f, 0.2f}, Quat{}};

// One step at the first step boundary; returns the frames published at that time.
std::vector<CameraFrame> firstFrames(VisionSystem& vision, Rng& rng, const Pose3& chassis = kSpawn,
                                     IMechanismDirectory* dir = nullptr) {
    vision.step(kStepUs, chassis, dir, rng);
    return vision.collectDue(kStepUs);
}

bool sees(const RobotConfig& config, const AprilTagLayout& layout, const Pose3& chassis = kSpawn) {
    VisionSystem vision(config, layout);
    Rng rng;
    const auto frames = firstFrames(vision, rng, chassis);
    REQUIRE(frames.size() == 1);
    return !frames[0].tags.empty();
}

}  // namespace

TEST_CASE("vision: a tag straight ahead is detected with consistent geometry") {
    VisionSystem vision(configWith({cameraAt("cam")}), layoutOf({tagAt(7, Vec3{3.f, 0.f, 1.f})}));
    Rng rng;
    const auto frames = firstFrames(vision, rng);
    REQUIRE(frames.size() == 1);
    REQUIRE(frames[0].tags.size() == 1);
    const TagDetection& t = frames[0].tags[0];
    CHECK(t.id == 7);
    CHECK(t.distanceM == doctest::Approx(3.0).epsilon(1e-4));
    CHECK(t.cameraToTag.position.x == doctest::Approx(3.f));
    CHECK(t.cameraToTag.position.y == doctest::Approx(0.f).epsilon(1e-4));
    CHECK(t.yawDeg == doctest::Approx(0.0).epsilon(1e-3));
    CHECK(t.pitchDeg == doctest::Approx(0.0).epsilon(1e-3));
    CHECK(t.ambiguity == 0.0);

    // 1280 px over a 60 degree FOV: f = 640 / tan(30 deg); the 0.1651 m tag spans f * 0.1651 / 3 px
    const double sidePx = 640.0 / std::tan(degToRad(30.0)) * 0.1651 / 3.0;
    const double sidePy = 400.0 / std::tan(degToRad(20.0)) * 0.1651 / 3.0;
    CHECK(t.areaPercent == doctest::Approx(100.0 * sidePx * sidePy / (1280.0 * 800.0)).epsilon(1e-3));
    // corners: bottom-left, bottom-right, top-right, top-left as seen from the tag front (the image is seen from the other side)
    CHECK(t.cornersPx[0] == doctest::Approx(640.0 - sidePx / 2).epsilon(1e-3));
    CHECK(t.cornersPx[1] == doctest::Approx(400.0 + sidePy / 2).epsilon(1e-3));
    CHECK(t.cornersPx[2] == doctest::Approx(640.0 + sidePx / 2).epsilon(1e-3));
    CHECK(t.cornersPx[3] == doctest::Approx(400.0 + sidePy / 2).epsilon(1e-3));
    CHECK(t.cornersPx[4] == doctest::Approx(640.0 + sidePx / 2).epsilon(1e-3));
    CHECK(t.cornersPx[5] == doctest::Approx(400.0 - sidePy / 2).epsilon(1e-3));
    CHECK(t.cornersPx[7] == doctest::Approx(400.0 - sidePy / 2).epsilon(1e-3));

    CHECK(frames[0].hasRobotPose);
    CHECK(frames[0].avgTagDistM == doctest::Approx(3.0).epsilon(1e-4));
    CHECK(frames[0].avgTagAreaPercent == doctest::Approx(t.areaPercent));
    CHECK(frames[0].tagSpanM == 0.0);
}

TEST_CASE("vision: bearing signs are + right and + up") {
    VisionSystem vision(configWith({cameraAt("cam")}),
                        layoutOf({tagAt(1, Vec3{3.f, 1.f, 1.f}), tagAt(2, Vec3{3.f, -1.f, 1.5f})}));
    Rng rng;
    const auto frames = firstFrames(vision, rng);
    REQUIRE(frames[0].tags.size() == 2);
    const double leftDeg = radToDeg(std::atan2(1.0, 3.0));
    CHECK(frames[0].tags[0].yawDeg == doctest::Approx(-leftDeg).epsilon(1e-3));  // +Y is left: negative yaw
    CHECK(frames[0].tags[1].yawDeg == doctest::Approx(leftDeg).epsilon(1e-3));
    CHECK(frames[0].tags[1].pitchDeg == doctest::Approx(radToDeg(std::atan2(0.5, 3.0))).epsilon(1e-3));
    CHECK(frames[0].tagSpanM == doctest::Approx(std::sqrt(4.0 + 0.25)).epsilon(1e-4));
}

TEST_CASE("vision: range limit") {
    CameraConfig cam = cameraAt("cam");
    const AprilTagLayout layout = layoutOf({tagAt(1, Vec3{3.f, 0.f, 1.f})});
    cam.maxRangeM = 3.01;
    CHECK(sees(configWith({cam}), layout));
    cam.maxRangeM = 2.99;
    CHECK_FALSE(sees(configWith({cam}), layout));
}

TEST_CASE("vision: horizontal and vertical field of view") {
    CameraConfig cam = cameraAt("cam");  // +-30 deg horizontally, +-20 deg vertically
    CHECK(sees(configWith({cam}), layoutOf({tagAt(1, Vec3{3.f, 1.5f, 1.f})})));    // 26.6 deg
    CHECK_FALSE(sees(configWith({cam}), layoutOf({tagAt(1, Vec3{3.f, 2.f, 1.f})})));  // 33.7 deg
    CHECK(sees(configWith({cam}), layoutOf({tagAt(1, Vec3{3.f, 0.f, 2.0f})})));    // 18.4 deg up
    CHECK_FALSE(sees(configWith({cam}), layoutOf({tagAt(1, Vec3{3.f, 0.f, 2.3f})})));  // 23.4 deg up
    CHECK_FALSE(sees(configWith({cam}), layoutOf({tagAt(1, Vec3{-3.f, 0.f, 1.f})})));  // behind the camera
}

TEST_CASE("vision: a tag facing away is not seen and the incidence angle is limited") {
    CameraConfig cam = cameraAt("cam");
    cam.maxIncidenceDeg = 60.0;
    const auto at = [](float incidenceDeg) { return layoutOf({tagAt(1, Vec3{3.f, 0.f, 1.f}, static_cast<float>(degToRad(incidenceDeg)))}); };
    CHECK(sees(configWith({cam}), at(0.f)));
    CHECK(sees(configWith({cam}), at(55.f)));
    CHECK_FALSE(sees(configWith({cam}), at(65.f)));
    CHECK_FALSE(sees(configWith({cam}), at(-65.f)));
    CHECK_FALSE(sees(configWith({cam}), at(100.f)));  // the back of the tag
    AprilTag away = tagAt(1, Vec3{3.f, 0.f, 1.f});
    away.pose.rotation = Quat{};
    CHECK_FALSE(sees(configWith({cam}), layoutOf({away})));
}

TEST_CASE("vision: the camera follows the chassis pose") {
    const CameraConfig cam = cameraAt("cam");
    const AprilTagLayout layout = layoutOf({tagAt(1, Vec3{3.f, 0.f, 1.f})});
    CHECK(sees(configWith({cam}), layout, Pose3{Vec3{-1.f, 0.f, 0.2f}, Quat{}}));
    CHECK_FALSE(sees(configWith({cam}), layout, Pose3{Vec3{-4.f, 0.f, 0.2f}, Quat{}}));  // out of range
    CHECK_FALSE(sees(configWith({cam}), layout, Pose3{Vec3{0.f, 0.f, 0.2f}, Quat::fromYaw(kPif)}));  // looking away
}

TEST_CASE("vision: an empty layout never sees tags but frames are still published") {
    VisionSystem vision(configWith({cameraAt("cam")}), AprilTagLayout{});
    Rng rng;
    const auto frames = firstFrames(vision, rng);
    REQUIRE(frames.size() == 1);
    CHECK(frames[0].tags.empty());
    CHECK_FALSE(frames[0].hasRobotPose);
    CHECK(frames[0].sequence == 0);
    CHECK(frames[0].captureTimeUs == kStepUs);
}

TEST_CASE("vision: the robot frame is the floor below the hull centre with the chassis yaw") {
    RobotConfig config = configWith({cameraAt("cam")});
    config.chassis.comOffsetM = Vec3{0.1f, 0.f, 0.f};  // the hull centre is 0.1 m behind the COM
    config.chassis.comHeightM = 0.2f;
    Rng rng;
    {
        VisionSystem vision(config, layoutOf({tagAt(1, Vec3{4.f, 2.f, 1.f})}));
        const Pose3 chassis{Vec3{1.f, 2.f, 0.2f}, Quat{}};
        const auto frames = firstFrames(vision, rng, chassis);
        REQUIRE(frames[0].hasRobotPose);
        CHECK(frames[0].robotPoseField.position.x == doctest::Approx(0.9f));
        CHECK(frames[0].robotPoseField.position.y == doctest::Approx(2.f));
        CHECK(frames[0].robotPoseField.position.z == doctest::Approx(0.f).epsilon(1e-4));
        CHECK(frames[0].robotPoseField.rotation.yaw() == doctest::Approx(0.f).epsilon(1e-4));
        // camera 0.8 m above the COM, which is 0.2 m above the floor; the robot frame origin is 0.1 m behind the COM
        CHECK(frames[0].robotToCamera.position.x == doctest::Approx(0.1f));
        CHECK(frames[0].robotToCamera.position.z == doctest::Approx(1.0f));
    }
    {
        // yaw rotates the offset: facing +Y, the hull centre is 0.1 m toward -Y of the COM
        AprilTag tag;
        tag.id = 1;
        tag.pose = Pose3{Vec3{0.f, 3.f, 1.f}, Quat::fromYaw(-kPif / 2.f)};
        VisionSystem vision(config, layoutOf({tag}));
        const auto frames = firstFrames(vision, rng, Pose3{Vec3{0.f, 0.f, 0.2f}, Quat::fromYaw(kPif / 2.f)});
        REQUIRE(frames[0].hasRobotPose);
        CHECK(frames[0].robotPoseField.position.x == doctest::Approx(0.f).epsilon(1e-4));
        CHECK(frames[0].robotPoseField.position.y == doctest::Approx(-0.1f));
        CHECK(frames[0].robotPoseField.rotation.yaw() == doctest::Approx(kPif / 2.f).epsilon(1e-4));
    }
}

TEST_CASE("vision: a pitched chassis does not shift the floor-referenced robot pose") {
    RobotConfig config = configWith({cameraAt("cam")});
    config.chassis.comHeightM = 0.2f;
    Rng rng;
    VisionSystem vision(config, layoutOf({tagAt(1, Vec3{4.f, 2.f, 1.f})}));
    const Pose3 chassis{Vec3{1.f, 2.f, 0.2f}, Quat::fromEuler(0.f, 0.15f, 0.f)};  // ~8.6 deg nose down
    const auto frames = firstFrames(vision, rng, chassis);
    REQUIRE(frames[0].hasRobotPose);
    CHECK(frames[0].robotPoseField.position.x == doctest::Approx(1.f));
    CHECK(frames[0].robotPoseField.position.y == doctest::Approx(2.f));
    CHECK(frames[0].robotPoseField.position.z == doctest::Approx(0.f).epsilon(1e-4));
}
TEST_CASE("vision: capture cadence follows the frame rate and starts on the first step") {
    CameraConfig cam = cameraAt("cam");
    cam.fps = 20.0;  // 50 ms
    VisionSystem vision(configWith({cam}), AprilTagLayout{});
    Rng rng;
    std::vector<CameraFrame> all;
    for (SimTimeUs t = kStepUs; t <= 1'000'000; t += kStepUs) {
        vision.step(t, kSpawn, nullptr, rng);
        for (CameraFrame& f : vision.collectDue(t)) all.push_back(std::move(f));
    }
    REQUIRE(all.size() == 21);
    CHECK(all[0].captureTimeUs == kStepUs);
    for (std::size_t i = 1; i < all.size(); ++i) {
        CHECK(all[i].captureTimeUs == static_cast<SimTimeUs>(i) * 50'000);
        CHECK(all[i].sequence == i);
    }

    // a rate faster than the step captures once per step
    cam.fps = 240.0;
    VisionSystem fast(configWith({cam}), AprilTagLayout{});
    int frames = 0;
    for (SimTimeUs t = kStepUs; t <= 100'000; t += kStepUs) {
        fast.step(t, kSpawn, nullptr, rng);
        frames += static_cast<int>(fast.collectDue(t).size());
    }
    CHECK(frames == 20);
}

TEST_CASE("vision: frames are delivered after the latency in publish order") {
    CameraConfig slow = cameraAt("slow");
    slow.latencyMs = 100.0;
    CameraConfig quick = cameraAt("quick");
    quick.latencyMs = 12.0;
    VisionSystem vision(configWith({slow, quick}), AprilTagLayout{});
    Rng rng;
    vision.step(kStepUs, kSpawn, nullptr, rng);
    CHECK(vision.collectDue(kStepUs).empty());
    CHECK(vision.collectDue(16'000).empty());

    const auto first = vision.collectDue(17'000);
    REQUIRE(first.size() == 1);
    CHECK(first[0].cameraIndex == 1);
    CHECK(first[0].captureTimeUs == kStepUs);
    CHECK(first[0].publishTimeUs == kStepUs + 12'000);

    vision.step(10'000, kSpawn, nullptr, rng);
    vision.step(15'000, kSpawn, nullptr, rng);
    // the quick camera's next two frames come before the slow camera's three
    const auto rest = vision.collectDue(200'000);
    REQUIRE(rest.size() == 5);
    CHECK(rest[0].cameraIndex == 1);
    CHECK(rest[1].cameraIndex == 1);
    CHECK(rest[2].cameraIndex == 0);
    CHECK(rest[2].publishTimeUs == kStepUs + 100'000);
    CHECK(rest[3].cameraIndex == 0);
    CHECK(rest[4].cameraIndex == 0);
    for (std::size_t i = 1; i < rest.size(); ++i) CHECK(rest[i - 1].publishTimeUs <= rest[i].publishTimeUs);
    CHECK(vision.collectDue(500'000).empty());
}

TEST_CASE("vision: noise is deterministic for a seed and absent without noise") {
    CameraConfig noisy = cameraAt("cam");
    noisy.positionNoiseM = 0.05;
    noisy.yawNoiseDeg = 2.0;
    const AprilTagLayout layout = layoutOf({tagAt(1, Vec3{3.f, 0.f, 1.f})});
    const auto run = [&](std::uint64_t seed, const CameraConfig& cam) {
        VisionSystem vision(configWith({cam}), layout);
        Rng rng(seed);
        std::vector<Pose3> poses;
        for (SimTimeUs t = kStepUs; t <= 50'000; t += kStepUs) {
            vision.step(t, kSpawn, nullptr, rng);
            for (const CameraFrame& f : vision.collectDue(t)) poses.push_back(f.robotPoseField);
        }
        return poses;
    };
    const auto a = run(42, noisy);
    const auto b = run(42, noisy);
    const auto c = run(43, noisy);
    REQUIRE(a.size() == 10);
    CHECK(a == b);
    CHECK_FALSE(a == c);
    CHECK_FALSE(a[0] == a[1]);

    double sumX = 0.0;
    for (const Pose3& p : a) sumX += p.position.x;
    CHECK(std::abs(sumX / static_cast<double>(a.size())) < 0.05);  // noise is centred on the true pose

    const auto clean = run(42, cameraAt("cam"));
    for (const Pose3& p : clean) {
        CHECK(p.position.x == doctest::Approx(0.f).epsilon(1e-4));
        CHECK(p.rotation.yaw() == doctest::Approx(0.f).epsilon(1e-4));
    }
}

TEST_CASE("vision: a camera on a mechanism link moves with the link and falls back to the chassis when unresolved") {
    CameraConfig cam = cameraAt("turretCam");
    cam.local = Pose3{};
    cam.attachment.parentMechanismId = "turret";
    cam.attachment.parentLink = "tip";
    const AprilTagLayout layout = layoutOf({tagAt(1, Vec3{3.f, 0.f, 1.f})});
    VisionSystem vision(configWith({cam}), layout);
    Rng rng;
    FakeDirectory dir;

    dir.linkPose = Pose3{Vec3{0.f, 0.f, 0.8f}, Quat{}};
    auto frames = firstFrames(vision, rng, kSpawn, &dir);
    REQUIRE(frames.size() == 1);
    CHECK(frames[0].tags.size() == 1);
    CHECK(frames[0].robotToCamera.position.z == doctest::Approx(1.0f));

    // the turret rotates away: the tag leaves the field of view
    dir.linkPose = Pose3{Vec3{0.f, 0.f, 0.8f}, Quat::fromYaw(kPif / 2.f)};
    vision.step(10'000, kSpawn, &dir, rng);
    frames = vision.collectDue(10'000);
    REQUIRE(frames.size() == 1);
    CHECK(frames[0].tags.empty());
    SnapshotBuilder builder;
    vision.publish(builder);
    const auto snapshot = builder.build();
    REQUIRE(snapshot->cameras().size() == 1);
    CHECK(snapshot->cameras()[0].id == "turretCam");
    CHECK_FALSE(snapshot->cameras()[0].tagsVisible);
    CHECK(snapshot->cameras()[0].worldPose.rotation.yaw() == doctest::Approx(kPif / 2.f).epsilon(1e-4));

    // an unknown link (or no directory) uses the chassis frame, like zones
    cam.attachment.parentLink = "nolink";
    VisionSystem unresolved(configWith({cam}), layout);
    unresolved.step(kStepUs, Pose3{Vec3{0.f, 0.f, 0.2f}, Quat{}}, &dir, rng);
    CHECK(unresolved.collectDue(kStepUs)[0].robotToCamera.position.z == doctest::Approx(0.2f));
}

TEST_CASE("vision: the snapshot reports the last captured frame of each camera") {
    VisionSystem vision(configWith({cameraAt("a"), cameraAt("b")}), layoutOf({tagAt(5, Vec3{3.f, 0.f, 1.f}), tagAt(9, Vec3{3.f, 0.5f, 1.f})}));
    Rng rng;
    {
        SnapshotBuilder before;
        vision.publish(before);
        CHECK(before.build()->cameras().empty());  // nothing captured yet
    }
    vision.step(kStepUs, kSpawn, nullptr, rng);
    SnapshotBuilder builder;
    vision.publish(builder);
    const auto snapshot = builder.build();
    REQUIRE(snapshot->cameras().size() == 2);
    CHECK(snapshot->cameras()[0].id == "a");
    CHECK(snapshot->cameras()[0].tagsVisible);
    CHECK(snapshot->cameras()[0].visibleTagIds == std::vector<int>{5, 9});
    CHECK(snapshot->cameras()[1].id == "b");
    CHECK(snapshot->cameras()[0].worldPose.position.z == doctest::Approx(1.0f));
}

TEST_CASE("vision: reset starts a new epoch") {
    CameraConfig cam = cameraAt("cam");
    cam.latencyMs = 50.0;
    cam.fps = 20.0;
    VisionSystem vision(configWith({cam}), layoutOf({tagAt(1, Vec3{3.f, 0.f, 1.f})}));
    Rng rng;
    for (SimTimeUs t = kStepUs; t <= 60'000; t += kStepUs) vision.step(t, kSpawn, nullptr, rng);
    vision.reset();
    CHECK(vision.collectDue(1'000'000).empty());  // queued frames are gone
    SnapshotBuilder builder;
    vision.publish(builder);
    CHECK(builder.build()->cameras().empty());

    vision.step(kStepUs, kSpawn, nullptr, rng);
    const auto frames = vision.collectDue(1'000'000);
    REQUIRE(frames.size() == 1);
    CHECK(frames[0].sequence == 0);
    CHECK(frames[0].captureTimeUs == kStepUs);
    CHECK(frames[0].publishTimeUs == kStepUs + 50'000);
}

TEST_CASE("vision: setup describes the configured cameras and the layout") {
    VisionSystem none(RobotConfig{}, AprilTagLayout{});
    CHECK(none.empty());
    CHECK(none.setup().cameras.empty());

    CameraConfig cam = cameraAt("cam");
    cam.kind = CameraKind::PhotonVision;
    VisionSystem vision(configWith({cam}), layoutOf({}));
    const VisionSetup setup = vision.setup();
    REQUIRE(setup.cameras.size() == 1);
    CHECK(setup.cameras[0].config == cam);
    CHECK(setup.layoutName == "test");
    CHECK(setup.photonVersion.empty());
}
