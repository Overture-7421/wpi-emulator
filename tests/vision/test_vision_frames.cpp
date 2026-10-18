#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>

#include "vision_rig.h"
#include "wpiemu/core/vision/visibility.h"

using namespace vision_test;
using doctest::Approx;

namespace {

// A directory with one movable mechanism link, "turret" / "root", given in the chassis frame.
class TurretDirectory final : public sdk::IMechanismDirectory {
public:
    sdk::IMechanism* find(std::string_view) override { return nullptr; }
    std::vector<sdk::IMechanism*> all() override { return {}; }
    std::optional<sdk::Pose3> linkPoseInChassis(std::string_view mechanism, std::string_view link) const override {
        if (!known || mechanism != "turret" || link != "root") return std::nullopt;
        return pose;
    }
    sdk::Pose3 pose;
    bool known = true;
};

sdk::CameraConfig turretCamera() {
    sdk::CameraConfig c = makeCamera("turret_cam");
    c.attachment.parentMechanismId = "turret";
    c.attachment.parentLink = "root";
    c.local = sdkPose(0.1f, 0.f, 0.3f);
    c.frameRateHz = 200.0;
    c.latencyUs = 0;
    return c;
}

vis::CameraModel modelOf(const sdk::CameraConfig& c) { return vis::CameraModel(c.intrinsics); }

const sdk::FiducialObservation* observationOf(const sdk::VisionFrame& frame, int id) {
    for (const sdk::FiducialObservation& o : frame.observations)
        if (o.id == id) return &o;
    return nullptr;
}

}  // namespace

TEST_CASE("vision frames: an ideal frame carries the exact geometry of every visible tag") {
    sdk::CameraConfig cam = frontCamera("front");
    cam.ntName = "photon-front";
    cam.frameRateHz = 200.0;
    cam.latencyUs = 0;
    Rig rig(visionOf({cam}));
    const auto frames = rig.run(2);
    REQUIRE(frames.size() == 1);
    const sdk::VisionFrame& f = frames[0].frame;
    CHECK(f.cameraId == "front");
    CHECK(f.vendor == sdk::CameraVendor::PhotonVision);
    CHECK(f.ntName == "photon-front");
    CHECK(f.sequence == 1);
    CHECK(f.captureTimeUs == 5000);
    CHECK(f.latencyUs == 0);
    CHECK(f.robotToCamera == sdkPose(0.f, 0.f, 0.5f));
    REQUIRE(f.observations.size() == 3);

    const vis::CameraModel model = modelOf(cam);
    const Pose3d cameraPose = poseYpr({0.0, 0.0, 0.5}, 0.0);
    for (const sdk::FiducialTag& tag : wallLayout().tags) {
        const sdk::FiducialObservation* o = observationOf(f, tag.id);
        REQUIRE(o != nullptr);
        const Pose3d cameraToTag = cameraPose.inverse() * vis::fromSdk(tag.pose);
        const auto corners = vis::projectTagCorners(model, cameraToTag, 0.1651);
        REQUIRE(corners);
        for (std::size_t k = 0; k < 4; ++k) {
            CHECK(o->cornersPx[k].x == Approx((*corners)[k].x).epsilon(1e-12));
            CHECK(o->cornersPx[k].y == Approx((*corners)[k].y).epsilon(1e-12));
        }
        const sdk::PixelPoint centre = vis::quadCenterPx(*corners);
        CHECK(o->centerPx.x == Approx(centre.x).epsilon(1e-12));
        CHECK(o->centerPx.y == Approx(centre.y).epsilon(1e-12));
        CHECK(o->yawRad == Approx(model.yawRad(centre)));
        CHECK(o->pitchRad == Approx(model.pitchRad(centre)));
        CHECK(o->areaPct == Approx(100.0 * vis::polygonAreaPx(*corners) / (1280.0 * 800.0)));
        CHECK(o->distanceM == Approx(vis::norm(cameraToTag.t)).epsilon(1e-9));
        CHECK(o->fieldToTag == tag.pose);
        CHECK(positionError(o->cameraToTagBest, cameraToTag) < 1e-5);
        CHECK(rotationError(o->cameraToTagBest, cameraToTag) < 1e-6);
        CHECK(o->ambiguity == 0.0);
        CHECK(o->reprojErrBestPx < 1e-9);
        CHECK(o->reprojErrAltPx >= o->reprojErrBestPx);
    }
    // the camera is 0.5 m up: a tag at the same height is level, one 0.5 m higher is above, left is left
    const auto* level = observationOf(f, 1);
    const auto* above = observationOf(f, 3);
    REQUIRE((level != nullptr && above != nullptr));
    CHECK(level->pitchRad == Approx(0.0).epsilon(1e-9));
    CHECK(above->pitchRad > 0.05);
    CHECK(level->yawRad < 0.0);  // tag 1 is at +Y (left), yaw is positive to the right
    CHECK(observationOf(f, 2)->yawRad > 0.0);
    CHECK(level->centerPx.x < 640.0);
}

TEST_CASE("vision frames: the multi-tag solution is the exact camera pose in the field frame") {
    sdk::CameraConfig cam = frontCamera();
    cam.frameRateHz = 200.0;
    cam.latencyUs = 0;
    Rig rig(visionOf({cam}));
    rig.world.place(sdkPose(1.f, -0.5f, 0.f, 0.2f));  // some chassis pose with a heading
    const auto frames = rig.run(2);
    REQUIRE(frames.size() == 1);
    const sdk::VisionFrame& f = frames[0].frame;
    REQUIRE(f.observations.size() == 3);
    REQUIRE(f.multiTag.valid);
    const Pose3d camera = vis::fromSdk(rig.world.world.pose(rig.world.chassis)) * vis::fromSdk(cam.local);
    CHECK(positionError(f.multiTag.fieldToCamera, camera) < 1e-5);
    CHECK(rotationError(f.multiTag.fieldToCamera, camera) < 1e-6);
    CHECK(f.multiTag.reprojErrPx < 1e-6);
    REQUIRE(f.multiTag.tagIds.size() == 3);
    for (std::size_t i = 0; i < 3; ++i) CHECK(f.multiTag.tagIds[i] == f.observations[i].id);  // same order
}

TEST_CASE("vision frames: multi-tag needs the option and at least two tags") {
    sdk::CameraConfig off = frontCamera();
    off.frameRateHz = 200.0;
    off.latencyUs = 0;
    off.multiTag = false;
    Rig noMulti(visionOf({off}));
    const auto a = noMulti.run(2);
    REQUIRE(a.size() == 1);
    CHECK(a[0].frame.observations.size() == 3);
    CHECK_FALSE(a[0].frame.multiTag.valid);

    sdk::CameraConfig on = off;
    on.multiTag = true;
    Rig single(visionOf({on}), layoutOf({tagAt(1, 4.f, 0.5f, 0.5f)}));
    const auto b = single.run(2);
    REQUIRE(b.size() == 1);
    CHECK(b[0].frame.observations.size() == 1);
    CHECK_FALSE(b[0].frame.multiTag.valid);
    CHECK(b[0].frame.multiTag.tagIds.empty());
}

TEST_CASE("vision frames: observations are sorted by area, largest first") {
    sdk::CameraConfig cam = frontCamera();
    cam.frameRateHz = 200.0;
    cam.latencyUs = 0;
    const auto layout = layoutOf({tagAt(5, 6.f, 0.f, 0.5f), tagAt(3, 3.f, 0.3f, 0.5f), tagAt(9, 4.5f, -0.3f, 0.5f)});
    Rig rig(visionOf({cam}), layout);
    const auto frames = rig.run(2);
    REQUIRE(frames.size() == 1);
    const auto& obs = frames[0].frame.observations;
    REQUIRE(obs.size() == 3);
    CHECK(obs[0].id == 3);  // 3 m
    CHECK(obs[1].id == 9);  // 4.5 m
    CHECK(obs[2].id == 5);  // 6 m
    CHECK(obs[0].areaPct > obs[1].areaPct);
    CHECK(obs[1].areaPct > obs[2].areaPct);
}

TEST_CASE("vision frames: the tag size is the camera override, else the tag's own, else the layout default") {
    sdk::CameraConfig cam = frontCamera();
    cam.frameRateHz = 200.0;
    cam.latencyUs = 0;
    const vis::CameraModel model = modelOf(cam);
    sdk::FiducialLayout layout = layoutOf({tagAt(1, 4.f, 0.f, 0.5f), tagAt(2, 4.f, 1.f, 0.5f)});
    layout.tagSizeM = 0.30;
    layout.tags[1].sizeM = 0.25;

    auto widthPx = [&](const sdk::CameraConfig& c, int id) {
        Rig rig(visionOf({c}), layout);
        const auto frames = rig.run(2);
        REQUIRE(frames.size() == 1);
        const auto* o = observationOf(frames[0].frame, id);
        REQUIRE(o != nullptr);
        return o->cornersPx[1].x - o->cornersPx[0].x;
    };
    auto expectedWidthPx = [&](double sizeM, int id) {
        const Pose3d cameraToTag = poseYpr({0.0, 0.0, 0.5}, 0.0).inverse() * vis::fromSdk(layout.tags[static_cast<std::size_t>(id - 1)].pose);
        return (*vis::projectTagCorners(model, cameraToTag, sizeM))[1].x -
               (*vis::projectTagCorners(model, cameraToTag, sizeM))[0].x;
    };

    cam.tagSizeM = 0.0;  // the layout decides
    CHECK(widthPx(cam, 1) == Approx(expectedWidthPx(0.30, 1)));  // layout default
    CHECK(widthPx(cam, 2) == Approx(expectedWidthPx(0.25, 2)));  // the tag's own size
    cam.tagSizeM = 0.2;                                          // the camera overrides both
    CHECK(widthPx(cam, 1) == Approx(expectedWidthPx(0.2, 1)));
    CHECK(widthPx(cam, 2) == Approx(expectedWidthPx(0.2, 2)));
}

TEST_CASE("vision frames: a field shape hides only the tags it stands in front of") {
    BoxWorld world;
    // a wall segment on the way to tag 1 (the camera is at (0,0,0.5), tag 1 at (4, 0.5, 0.5))
    world.addBox({2.f, 0.25f, 0.5f}, {0.05f, 0.2f, 1.f});

    sdk::CameraConfig cam = frontCamera();
    cam.frameRateHz = 200.0;
    cam.latencyUs = 0;
    auto idsSeen = [&](bool occlusion) {
        cam.occlusion = occlusion;
        auto created = vis::VisionSystem::create(visionOf({cam}), wallLayout(), 5000);
        REQUIRE(created.ok());
        auto system = std::move(created).value();
        sdk::Rng rng(1);
        system->step(viewOf(world), rng, 5000);
        std::vector<sdk::VisionFrame> out;
        system->collectDue(10000, out);
        REQUIRE(out.size() == 1);
        std::vector<int> ids;
        for (const auto& o : out[0].observations) ids.push_back(o.id);
        std::sort(ids.begin(), ids.end());
        return ids;
    };
    CHECK(idsSeen(false) == std::vector<int>{1, 2, 3});
    CHECK(idsSeen(true) == std::vector<int>{2, 3});
}

TEST_CASE("vision frames: a camera on a mechanism link follows the link") {
    TurretDirectory turret;
    turret.pose = sdkPose(0.2f, 0.f, 0.1f);  // the link, in the chassis frame
    const sdk::CameraConfig cam = turretCamera();
    Rig rig(visionOf({cam}), wallLayout(), 5000, &turret);

    // link * local = (0.3, 0, 0.4)
    auto first = rig.run(2);
    REQUIRE(first.size() == 1);
    CHECK(positionError(first[0].frame.robotToCamera, Pose3d{Mat3d{}, {0.3, 0.0, 0.4}}) < 1e-6);
    REQUIRE(first[0].frame.multiTag.valid);
    CHECK(positionError(first[0].frame.multiTag.fieldToCamera, Pose3d{Mat3d{}, {0.3, 0.0, 0.4}}) < 1e-5);

    // turn the turret 0.3 rad to the left
    turret.pose = {sdk::Vec3{0.2f, 0.f, 0.1f}, sdk::Quat::fromYaw(0.3f)};
    const auto later = rig.run(2);
    REQUIRE(!later.empty());
    const sdk::VisionFrame& f = later.back().frame;
    const Pose3d expected = vis::fromSdk(turret.pose) * vis::fromSdk(cam.local);
    CHECK(positionError(f.robotToCamera, expected) < 1e-6);
    CHECK(rotationError(f.robotToCamera, expected) < 1e-6);
    REQUIRE(f.multiTag.valid);
    CHECK(positionError(f.multiTag.fieldToCamera, expected) < 1e-5);  // the chassis is at the origin
    CHECK(rotationError(f.multiTag.fieldToCamera, expected) < 1e-6);
    CHECK(f.observations.front().yawRad > 0.1);  // the tags moved to the right in the image

    // the snapshot places the camera from the live link too
    CHECK(positionError(rig.cameras()[0].pose, expected) < 1e-6);

    // the default assumed mount of the handshake is the pose at session start, not the current one
    const sdk::VisionSetup setup = rig.system->setup("");
    REQUIRE(setup.cameras.size() == 1);
    CHECK(positionError(setup.cameras[0].defaultAssumedRobotToCamera, Pose3d{Mat3d{}, {0.3, 0.0, 0.4}}) < 1e-6);
}

TEST_CASE("vision frames: an unresolvable attachment falls back to the chassis root and is reported") {
    TurretDirectory turret;
    turret.known = false;
    const sdk::CameraConfig cam = turretCamera();
    Rig rig(visionOf({cam}), wallLayout(), 5000, &turret);
    const auto frames = rig.run(2);
    REQUIRE(frames.size() == 1);
    CHECK(positionError(frames[0].frame.robotToCamera, Pose3d{Mat3d{}, {0.1, 0.0, 0.3}}) < 1e-6);
    CHECK(rig.cameras()[0].status == "attachment_unresolved");

    // with no directory at all, a chassis camera is fine and a mechanism camera falls back
    Rig bare(visionOf({cam}));
    CHECK(bare.cameras()[0].status == "attachment_unresolved");
    Rig chassis(visionOf({frontCamera()}));
    CHECK(chassis.cameras()[0].status.empty());
}

TEST_CASE("vision frames: the camera names follow the vendor defaults") {
    sdk::CameraConfig photon = frontCamera("front");
    sdk::CameraConfig limelight = frontCamera("rear");
    limelight.vendor = sdk::CameraVendor::Limelight;
    sdk::CameraConfig named = frontCamera("left");
    named.vendor = sdk::CameraVendor::Limelight;
    named.ntName = "limelight-left";
    for (sdk::CameraConfig* c : {&photon, &limelight, &named}) {
        c->frameRateHz = 200.0;
        c->latencyUs = 0;
    }
    Rig rig(visionOf({photon, limelight, named}));
    const auto frames = rig.run(2);
    REQUIRE(frames.size() == 3);
    CHECK(frames[0].frame.ntName == "front");  // PhotonVision: the id
    CHECK(frames[1].frame.ntName == "limelight");
    CHECK(frames[2].frame.ntName == "limelight-left");
    CHECK(frames[1].frame.vendor == sdk::CameraVendor::Limelight);
}

TEST_CASE("vision setup: one entry per enabled camera with intrinsics, rate, latency and the default mount") {
    sdk::CameraConfig photon = frontCamera("front");
    photon.ntName = "photon-front";
    photon.frameRateHz = 30.0;
    photon.latencyUs = 25000;
    photon.intrinsics.distortion = {0.1, -0.2, 0.001, 0.002, 0.03};
    photon.local = sdkPose(0.3f, -0.2f, 0.5f, 0.5f);

    sdk::CameraConfig limelight = frontCamera("rear");
    limelight.vendor = sdk::CameraVendor::Limelight;
    limelight.ntName = "limelight-rear";
    limelight.frameRateHz = 90.0;
    limelight.latencyUs = 20000;
    limelight.intrinsics.widthPx = 640;
    limelight.intrinsics.heightPx = 480;
    limelight.intrinsics.hfovRad = 1.0;
    limelight.intrinsics.fxPx = 500.0;
    limelight.intrinsics.cxPx = 330.0;
    limelight.assumedPose = sdkPose(0.31f, -0.19f, 0.52f, 0.45f);  // what the device believes

    sdk::CameraConfig off = frontCamera("off");
    off.enabled = false;

    Rig rig(visionOf({photon, off, limelight}));
    const sdk::VisionSetup setup = rig.system->setup("");
    REQUIRE(setup.cameras.size() == 2);  // the disabled camera is not announced
    const sdk::CameraSetup& p = setup.cameras[0];
    CHECK(p.cameraId == "front");
    CHECK(p.vendor == sdk::CameraVendor::PhotonVision);
    CHECK(p.ntName == "photon-front");
    CHECK(p.widthPx == 1280);
    CHECK(p.heightPx == 800);
    CHECK(p.fxPx == Approx(640.0));
    CHECK(p.fyPx == Approx(640.0));
    CHECK(p.cxPx == Approx(640.0));
    CHECK(p.cyPx == Approx(400.0));
    CHECK(p.distortion == photon.intrinsics.distortion);
    CHECK(p.frameRateHz == 30.0);
    CHECK(p.latencyUs == 25000);
    CHECK(p.defaultAssumedRobotToCamera.position == photon.local.position);  // no assumed pose: the physical mount
    CHECK(rotationError(p.defaultAssumedRobotToCamera, vis::fromSdk(photon.local)) < 1e-6);

    const sdk::CameraSetup& l = setup.cameras[1];
    CHECK(l.cameraId == "rear");
    CHECK(l.vendor == sdk::CameraVendor::Limelight);
    CHECK(l.ntName == "limelight-rear");
    CHECK(l.widthPx == 640);
    CHECK(l.fxPx == 500.0);
    CHECK(l.fyPx == 500.0);  // fy follows fx
    CHECK(l.cxPx == 330.0);
    CHECK(l.cyPx == 240.0);
    CHECK(l.frameRateHz == 90.0);
    CHECK(l.latencyUs == 20000);
    CHECK(l.defaultAssumedRobotToCamera == *limelight.assumedPose);  // the device's belief, not the truth
}

TEST_CASE("vision setup: the PhotonVision message UUID is the config override, else the argument, else empty") {
    sdk::CameraConfig cam = frontCamera();
    sdk::VisionConfig config = visionOf({cam});
    {
        Rig rig(config);
        CHECK(rig.system->setup("from-argument").cameras[0].photonMessageUuid == "from-argument");
        CHECK(rig.system->setup("").cameras[0].photonMessageUuid.empty());
    }
    config.photonMessageUuid = "from-config";
    Rig rig(config);
    CHECK(rig.system->setup("from-argument").cameras[0].photonMessageUuid == "from-config");
    CHECK(rig.system->setup("").cameras[0].photonMessageUuid == "from-config");
}

TEST_CASE("vision snapshot: cameras and tags are contributed from the live world") {
    sdk::CameraConfig cam = frontCamera("front");
    cam.label = "Front camera";
    cam.frameRateHz = 30.0;
    cam.latencyUs = 25000;
    sdk::CameraConfig off = frontCamera("off");
    off.enabled = false;
    off.vendor = sdk::CameraVendor::Limelight;
    sdk::FiducialLayout layout = wallLayout();
    layout.tagSizeM = 0.2;
    layout.tags[2].sizeM = 0.3;
    Rig rig(visionOf({cam, off}), layout);

    // nothing yet: no capture, no latency
    sdk::SimSnapshotData data = rig.snapshot();
    REQUIRE(data.cameras.size() == 2);
    CHECK(data.cameras[0].id == "front");
    CHECK(data.cameras[0].label == "Front camera");
    CHECK(data.cameras[0].enabled);
    CHECK(data.cameras[0].widthPx == 1280);
    CHECK(data.cameras[0].heightPx == 800);
    CHECK(data.cameras[0].hfovRad == Approx(sdk::kPi / 2.0).epsilon(1e-4));  // the default 1.5708 rad
    CHECK(data.cameras[0].vfovRad == Approx(2.0 * std::atan(400.0 / (640.0 / std::tan(0.5 * 1.5708)))));
    CHECK(data.cameras[0].framesCaptured == 0);
    CHECK(data.cameras[0].visibleTagIds.empty());
    CHECK(data.cameras[0].lastLatencyMs == 0.0);
    CHECK_FALSE(data.cameras[0].robotConnected);
    CHECK(data.cameras[0].status.empty());
    CHECK(data.cameras[1].id == "off");
    CHECK(data.cameras[1].vendor == sdk::CameraVendor::Limelight);
    CHECK_FALSE(data.cameras[1].enabled);
    CHECK(data.cameras[1].status == "disabled");
    CHECK(data.detectedCameras.empty());

    REQUIRE(data.fiducials.size() == 3);
    CHECK(data.fiducials[0].id == 1);
    CHECK(data.fiducials[0].pose == layout.tags[0].pose);
    CHECK(data.fiducials[0].sizeM == Approx(0.2f));  // layout default
    CHECK(data.fiducials[2].sizeM == Approx(0.3f));  // the tag's own size

    // the camera is placed from the world as it is now, with no tick in between
    rig.world.place(sdkPose(2.f, 1.f, 0.f, 0.5f));
    const sdk::Pose3 placed = rig.cameras()[0].pose;
    CHECK(positionError(placed, vis::fromSdk(rig.world.world.pose(rig.world.chassis)) * vis::fromSdk(cam.local)) < 1e-6);
    CHECK(placed.position.x == Approx(2.f));
    CHECK(placed.position.z == Approx(0.5f));
    rig.world.place(sdk::Pose3{});

    // the visible set of the latest capture shows at once, the frame itself publishes later
    rig.run(7);  // first capture at 35 ms
    data = rig.snapshot();
    CHECK(data.cameras[0].framesCaptured == 1);
    CHECK(data.cameras[0].framesPublished == 0);
    CHECK(data.cameras[0].visibleTagIds.size() == 3);
    CHECK(data.cameras[0].lastLatencyMs == 0.0);
    rig.run(6);  // published at tick 13 (35 ms + 25 ms latency + one step)
    data = rig.snapshot();
    CHECK(data.cameras[0].framesPublished == 1);
    CHECK(data.cameras[0].lastLatencyMs == Approx(25.0));
}

TEST_CASE("vision snapshot: what the robot reported is shown next to the camera, never used for measuring") {
    sdk::CameraConfig cam = frontCamera("front");
    cam.ntName = "photon-front";
    cam.frameRateHz = 200.0;
    cam.latencyUs = 0;
    sdk::CameraConfig off = frontCamera("off");
    off.ntName = "photon-off";
    off.enabled = false;
    Rig rig(visionOf({cam, off}));
    const auto before = rig.run(4);

    sdk::CameraRobotState state;
    state.ntName = "photon-front";
    state.cameraId = "front";
    state.publishing = true;
    state.framesPublished = 12;
    state.driverMode = true;  // a robot request: it must not change what the engine measures
    state.throttle = 0.9;
    state.pipelineIndex = 3;
    sdk::CameraRobotState stray;
    stray.ntName = "mystery";
    stray.vendor = sdk::CameraVendor::Limelight;
    stray.cameraId = "";
    stray.status = "foreign_publisher";
    sdk::CameraRobotState offState;
    offState.ntName = "photon-off";
    offState.cameraId = "off";
    offState.publishing = true;
    const sdk::CameraRobotState states[] = {state, stray, offState};
    rig.robotStates.upsert(states);

    sdk::SimSnapshotData data = rig.snapshot();
    REQUIRE(data.cameras.size() == 2);
    CHECK(data.cameras[0].robotConnected);
    CHECK(data.cameras[0].status.empty());
    CHECK(data.cameras[1].status == "disabled");  // the configuration wins over the robot's report
    // the detected cameras are the session's to list: they exist without an engine
    CHECK(data.detectedCameras.empty());

    // a changed entry replaces its predecessor (matched by vendor and NT name)
    sdk::CameraRobotState refused = state;
    refused.publishing = false;
    refused.status = "photon_uuid_unsupported";
    rig.robotStates.upsert(std::span<const sdk::CameraRobotState>(&refused, 1));
    data = rig.snapshot();
    CHECK_FALSE(data.cameras[0].robotConnected);
    CHECK(data.cameras[0].status == "photon_uuid_unsupported");

    // the measurements are exactly those of a run that never heard from the robot
    Rig reference(visionOf({cam, off}));
    reference.run(4);
    const auto after = rig.run(40);
    const auto expected = reference.run(40);
    REQUIRE(after.size() == expected.size());
    for (std::size_t i = 0; i < after.size(); ++i) CHECK(after[i].frame == expected[i].frame);
    CHECK(before.size() == 3);  // 4 ticks at 200 Hz, zero latency: the captures of ticks 1..3

    // a lost connection: nothing of what the robot reported is shown any more
    CHECK(rig.cameras()[0].status == "photon_uuid_unsupported");
    rig.robotStates.clear();
    CHECK(rig.cameras()[0].status.empty());
    CHECK_FALSE(rig.cameras()[0].robotConnected);
}
