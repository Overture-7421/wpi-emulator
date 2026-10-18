#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

#include "editor_page.h"
#include "model/editor_document.h"
#include "model/property_groups.h"
#include "model/slot_status.h"
#include "model/transform_access.h"
#include "model/vision_specs.h"
#include "test_support.h"
#include "viewmodel/editor_controller.h"
#include "wpiemu/sdk/config/value_codec.h"

using namespace wpiemu::app::editor;
using namespace wpiemu::app::editor::testing;
namespace sdk = wpiemu::sdk;

namespace {

constexpr sdk::CameraVendor kPhoton = sdk::CameraVendor::PhotonVision;
constexpr sdk::CameraVendor kLimelight = sdk::CameraVendor::Limelight;

sdk::Vec3 cameraWorldPos(const sdk::RobotConfig& c, const EditorCatalog& cat, const std::string& id) {
    return ops::cameraWorld(c, cat, *c.findCamera(id)).position;
}

// Where the camera looks (its +X axis) in the display frame.
sdk::Vec3 cameraAxis(const sdk::RobotConfig& c, const EditorCatalog& cat, const std::string& id) {
    return ops::cameraWorld(c, cat, *c.findCamera(id)).rotation.rotate({1.f, 0.f, 0.f});
}

sdk::CameraRobotState detection(const char* name, sdk::CameraVendor vendor) {
    sdk::CameraRobotState s;
    s.ntName = name;
    s.vendor = vendor;
    return s;
}

}  // namespace

// ---- ops: adding, naming, defaults -------------------------------------------------------------------

TEST_CASE("camera ops: the sample robot carries a PhotonVision camera at the front and a Limelight at the back and validates") {
    const auto c = robot();
    REQUIRE(c.vision.cameras.size() == 2);
    const sdk::CameraConfig* photon = c.findCamera("photon_front");
    const sdk::CameraConfig* limelight = c.findCamera("limelight_rear");
    REQUIRE((photon != nullptr && limelight != nullptr));
    CHECK(photon->vendor == kPhoton);
    CHECK(photon->networkName() == "photon-front");
    CHECK(limelight->vendor == kLimelight);
    CHECK(limelight->networkName() == "limelight-rear");
    CHECK(cameraAxis(c, catalog(), "photon_front").x > 0.9f);
    CHECK(cameraAxis(c, catalog(), "limelight_rear").x < -0.9f);   // looks backwards
    FakeEditorBackend backend;
    for (const auto& issue : backend.validate(c)) CHECK(issue.severity != sdk::Severity::Error);
}

TEST_CASE("camera ops: adding gives each vendor its defaults, a front mount looking forward and up, and an ideal camera") {
    const auto cat = catalog();
    auto c = robot();
    c.vision.cameras.clear();

    const std::string photonId = ops::addCamera(c, cat, kPhoton, {});
    CHECK(photonId == "photon");
    const sdk::CameraConfig* photon = c.findCamera(photonId);
    REQUIRE(photon != nullptr);
    CHECK(photon->vendor == kPhoton);
    CHECK(photon->ntName == "photon");   // the nickname defaults to the id
    CHECK(photon->label == "PhotonVision camera");
    CHECK(photon->enabled);
    CHECK(photon->attachment.onChassis());
    CHECK(photon->intrinsics.widthPx == 1280);
    CHECK(photon->intrinsics.heightPx == 800);
    CHECK(photon->intrinsics.hfovRad == doctest::Approx(1.5708));
    CHECK(photon->frameRateHz == doctest::Approx(30.0));
    CHECK(photon->realism.ideal());
    CHECK_FALSE(photon->assumedPose.has_value());
    // On the front edge of the hull (0.38 m from its centre, 5 cm in), 20 cm above the frame, looking forward and 20 degrees up.
    CHECK(approx(photon->local.position, {0.33f, 0.f, 0.03f + 0.1f + 0.2f - 0.22f}));
    const sdk::Vec3 axis = cameraAxis(c, cat, photonId);
    CHECK(axis.x == doctest::Approx(std::cos(sdk::degToRad(20.0))).epsilon(0.01));
    CHECK(axis.z == doctest::Approx(std::sin(sdk::degToRad(20.0))).epsilon(0.01));

    const std::string limelightId = ops::addCamera(c, cat, kLimelight, {});
    CHECK(limelightId == "limelight");
    const sdk::CameraConfig* limelight = c.findCamera(limelightId);
    REQUIRE(limelight != nullptr);
    CHECK(limelight->ntName == "limelight");   // the first Limelight keeps the default table name
    CHECK(limelight->label == "Limelight camera");
    CHECK(limelight->frameRateHz == doctest::Approx(90.0));
    CHECK(limelight->local.position.y < photon->local.position.y);   // the second camera sits beside the first one
}

TEST_CASE("camera ops: ids are unique against cameras, zones and mechanisms, network names against cameras of the same vendor") {
    const auto cat = catalog();
    auto c = robot();
    c.vision.cameras.clear();

    CHECK(ops::addCamera(c, cat, kLimelight, {}) == "limelight");
    CHECK(ops::addCamera(c, cat, kLimelight, {}) == "limelight_2");
    CHECK(c.findCamera("limelight_2")->ntName == "limelight-2");   // two Limelights never share a table name
    CHECK(c.findCamera("limelight_2")->label == "Limelight camera 2");
    CHECK(ops::addCamera(c, cat, kPhoton, {}) == "photon");
    CHECK(ops::addCamera(c, cat, kPhoton, {}) == "photon_2");
    CHECK(c.findCamera("photon_2")->ntName == "photon_2");

    // Mechanisms and zones keep clear of camera ids as well (the validator rejects clashes), and cameras of theirs.
    CHECK(ops::uniqueMechanismId(c, "photon") == "photon_3");
    CHECK(ops::uniqueZoneId(c, "photon") == "photon_3");
    c.zones.push_back({});
    c.zones.back().id = "photon_3";
    c.zones.back().kind = "game_2026.pickup";
    sdk::MechanismConfig m;
    m.id = "photon_4";
    m.type = "mechanism";
    c.mechanisms.push_back(m);
    CHECK(ops::uniqueCameraId(c, "photon") == "photon_5");
    CHECK(ops::uniqueCameraId(c, "chassis") == "chassis_2");   // the reserved names are never handed out
    CHECK(ops::uniqueCameraId(c, "root") == "root_2");
    CHECK(ops::uniqueCameraId(c, "").size() > 0);
    CHECK(ops::uniqueNetworkName(c, kPhoton, "photon_2") == "photon_2-2");
    CHECK(ops::uniqueNetworkName(c, kPhoton, "photon_2", "photon_2") == "photon_2");   // the camera itself does not count
    CHECK(ops::uniqueNetworkName(c, kLimelight, "photon_2") == "photon_2");            // another vendor
    CHECK(ops::duplicateMechanism(c, "shooter") == "shooter_2");

    FakeEditorBackend backend;
    for (const auto& issue : backend.validate(c)) CHECK(issue.severity != sdk::Severity::Error);
}

TEST_CASE("camera ops: a mount on a mechanism link is kept, an unknown mechanism or link falls back to the chassis") {
    const auto cat = catalog();
    auto c = robot();
    const std::string onTurret = ops::addCamera(c, cat, kPhoton, {"turret", "mover"});
    CHECK(c.findCamera(onTurret)->attachment.parentMechanismId == "turret");
    CHECK(c.findCamera(onTurret)->attachment.parentLink == "mover");
    CHECK(ops::addCamera(c, cat, kPhoton, {"ghost", "root"}).size() > 0);
    CHECK(c.vision.cameras.back().attachment.onChassis());
    CHECK(ops::addCamera(c, cat, kPhoton, {"turret", "no_such_link"}).size() > 0);
    CHECK(c.vision.cameras.back().attachment.onChassis());
    CHECK(ops::addCamera(c, cat, kPhoton, {"intake", ""}).size() > 0);
    CHECK(c.vision.cameras.back().attachment.parentMechanismId == "intake");
    CHECK(c.vision.cameras.back().attachment.parentLink == "root");
    FakeEditorBackend backend;
    for (const auto& issue : backend.validate(c)) CHECK(issue.severity != sdk::Severity::Error);
}

TEST_CASE("camera ops: a camera for a detected name takes the vendor and the name, and a Limelight's own pose places it") {
    auto c = robot();
    c.vision.cameras.clear();

    const std::string photon = ops::addCameraFromDetected(c, detection("photon-arm", kPhoton));
    CHECK(photon == "photon_arm");
    const sdk::CameraConfig* cam = c.findCamera(photon);
    REQUIRE(cam != nullptr);
    CHECK(cam->vendor == kPhoton);
    CHECK(cam->ntName == "photon-arm");
    CHECK(cam->label == "photon-arm");
    CHECK(cam->attachment.onChassis());
    CHECK(cam->local.position.x > 0.f);   // the default front mount

    CHECK(ops::addCameraFromDetected(c, detection("photon-arm", kPhoton)).empty());   // already configured
    CHECK(ops::addCameraFromDetected(c, detection("", kPhoton)).empty());
    CHECK(c.vision.cameras.size() == 1);
    CHECK(ops::addCameraFromDetected(c, detection("photon-arm", kLimelight)) == "photon_arm_2");   // the id stays unique, the name is the vendor's own
    CHECK(c.findCamera("photon_arm_2")->ntName == "photon-arm");
    CHECK(ops::addCameraFromDetected(c, detection("photon.arm!", kPhoton)) == "photon_arm_3");     // ids are made of letters, digits and '_'

    // setCameraPose_RobotSpace(forward -0.30, side 0, up 0.45, roll 0, pitch 25, yaw -180): Limelight robot space has +Y to the right and
    // angles with the opposite sign of WPILib's, so this is the sample robot's rear camera (-0.30, 0, 0.45, pitch -25, yaw 180).
    sdk::CameraRobotState rear = detection("limelight-rear", kLimelight);
    rear.cameraPoseOverride = std::array<double, 6>{-0.30, 0.0, 0.45, 0.0, 25.0, -180.0};
    const std::string rearId = ops::addCameraFromDetected(c, rear);
    CHECK(rearId == "limelight_rear");
    const sdk::CameraConfig* ll = c.findCamera(rearId);
    REQUIRE(ll != nullptr);
    CHECK(approx(ll->local.position, {-0.30f, 0.f, 0.45f - c.chassis.comHeightM}));
    const sdk::Vec3 axis = cameraAxis(c, catalog(), rearId);
    CHECK(approx(axis, {static_cast<float>(-std::cos(sdk::degToRad(25.0))), 0.f, static_cast<float>(std::sin(sdk::degToRad(25.0)))}, 1e-3f));

    // The side is positive to the right: 0.2 m right of the centre line is -0.2 m on our left-positive Y axis.
    sdk::CameraRobotState right = detection("limelight-right", kLimelight);
    right.cameraPoseOverride = std::array<double, 6>{0.1, 0.2, 0.3, 0.0, 0.0, 0.0};
    const sdk::CameraConfig* rc = c.findCamera(ops::addCameraFromDetected(c, right));
    REQUIRE(rc != nullptr);
    CHECK(rc->local.position.y == doctest::Approx(-0.2));
    // A PhotonVision camera has no pose to read, and a non-finite one is ignored.
    sdk::CameraRobotState odd = detection("limelight-odd", kLimelight);
    odd.cameraPoseOverride = std::array<double, 6>{std::nan(""), 0.0, 0.0, 0.0, 0.0, 0.0};
    const sdk::CameraConfig* oc = c.findCamera(ops::addCameraFromDetected(c, odd));
    REQUIRE(oc != nullptr);
    CHECK(std::isfinite(oc->local.position.x));
}

// ---- ops: duplicate, remove, re-attach ---------------------------------------------------------------

TEST_CASE("camera ops: duplicate copies the camera next to the original with its own id and network name, remove deletes") {
    auto c = robot();
    const sdk::CameraConfig original = *c.findCamera("photon_front");
    const std::string copy = ops::duplicateCamera(c, "photon_front");
    CHECK(copy == "photon_front_2");
    const sdk::CameraConfig* z = c.findCamera(copy);
    REQUIRE(z != nullptr);
    CHECK(z->label == "Front camera (copy)");
    CHECK(z->ntName == "photon-front-2");   // an enabled copy with the same name would be a validation error
    CHECK(z->attachment == original.attachment);
    CHECK(z->vendor == original.vendor);
    CHECK(z->intrinsics == original.intrinsics);
    CHECK(approx(z->local.position.y, original.local.position.y + 0.1f));
    CHECK(ops::duplicateCamera(c, "nobody").empty());
    FakeEditorBackend backend;
    for (const auto& issue : backend.validate(c)) CHECK(issue.severity != sdk::Severity::Error);

    CHECK(ops::removeCamera(c, copy));
    CHECK_FALSE(ops::removeCamera(c, copy));
    CHECK(c.vision.cameras.size() == 2);
}

TEST_CASE("camera ops: re-attaching keeps the world pose and refuses unknown parents or links") {
    const auto cat = catalog();
    auto c = robot();
    const sdk::Vec3 before = cameraWorldPos(c, cat, "photon_front");
    const sdk::Vec3 lookingBefore = cameraAxis(c, cat, "photon_front");

    REQUIRE(ops::reattachCamera(c, cat, "photon_front", "turret", "mover"));
    CHECK(c.findCamera("photon_front")->attachment.parentMechanismId == "turret");
    CHECK(c.findCamera("photon_front")->attachment.parentLink == "mover");
    CHECK(approx(cameraWorldPos(c, cat, "photon_front"), before));
    CHECK(approx(cameraAxis(c, cat, "photon_front"), lookingBefore, 1e-3f));
    REQUIRE(ops::reattachCamera(c, cat, "photon_front", "chassis", "root"));
    CHECK(c.findCamera("photon_front")->attachment.onChassis());
    CHECK(approx(cameraWorldPos(c, cat, "photon_front"), before));

    CHECK_FALSE(ops::reattachCamera(c, cat, "photon_front", "ghost", "root"));
    CHECK_FALSE(ops::reattachCamera(c, cat, "photon_front", "turret", "no_such_link"));
    CHECK_FALSE(ops::reattachCamera(c, cat, "photon_front", "chassis", "mover"));
    CHECK_FALSE(ops::reattachCamera(c, cat, "ghost", "chassis", "root"));
    CHECK(approx(cameraWorldPos(c, cat, "photon_front"), before));

    // Without keeping the world pose the local pose is untouched and the camera jumps.
    const sdk::Pose3 local = c.findCamera("photon_front")->local;
    REQUIRE(ops::reattachCamera(c, cat, "photon_front", "turret", "mover", false));
    CHECK(approx(c.findCamera("photon_front")->local.position, local.position));
}

TEST_CASE("camera ops: deleting a mechanism moves its cameras to its parent in place, moving the COM keeps chassis cameras where they are") {
    const auto cat = catalog();
    auto c = robot();
    REQUIRE(ops::reattachCamera(c, cat, "photon_front", "turret", "mover"));
    const sdk::Vec3 photonBefore = cameraWorldPos(c, cat, "photon_front");
    const sdk::Vec3 lookingBefore = cameraAxis(c, cat, "photon_front");

    REQUIRE(ops::removeMechanism(c, cat, "turret"));
    CHECK(c.findCamera("photon_front")->attachment.onChassis());   // the turret hung on the chassis
    CHECK(approx(cameraWorldPos(c, cat, "photon_front"), photonBefore));
    CHECK(approx(cameraAxis(c, cat, "photon_front"), lookingBefore, 1e-3f));

    // A camera on a mechanism below another one moves up to that parent, not to the chassis.
    auto d = robot();
    REQUIRE(ops::reattachCamera(d, cat, "limelight_rear", "shooter", "root"));
    const sdk::Vec3 rearBefore = cameraWorldPos(d, cat, "limelight_rear");
    REQUIRE(ops::removeMechanism(d, cat, "shooter"));
    CHECK(d.findCamera("limelight_rear")->attachment.parentMechanismId == "turret");
    CHECK(d.findCamera("limelight_rear")->attachment.parentLink == "mover");
    CHECK(approx(cameraWorldPos(d, cat, "limelight_rear"), rearBefore));

    auto e = robot();
    const sdk::Vec3 frontBefore = cameraWorldPos(e, cat, "photon_front");
    ops::setCom(e, 0.05f, -0.03f, 0.30f, true, &cat);
    CHECK(approx(cameraWorldPos(e, cat, "photon_front"), frontBefore));
    CHECK(approx(cameraWorldPos(e, cat, "limelight_rear"), cameraWorldPos(robot(), cat, "limelight_rear")));
}

// ---- ops: editing fields -----------------------------------------------------------------------------

TEST_CASE("camera ops: every field of a camera can be set by its persisted name and unchanged or invalid values change nothing") {
    const auto cat = catalog();
    auto c = robot();
    const auto cam = [&]() -> const sdk::CameraConfig& { return *c.findCamera("photon_front"); };
    const auto set = [&](const char* key, sdk::Value v) { return ops::setCameraProperty(c, cat, "photon_front", key, v); };

    CHECK(set("label", "Left camera"));
    CHECK(cam().label == "Left camera");
    CHECK_FALSE(set("label", "Left camera"));   // unchanged
    CHECK(set("ntName", "  arducam  "));
    CHECK(cam().ntName == "arducam");           // typed names are trimmed
    CHECK(set("enabled", false));
    CHECK_FALSE(cam().enabled);

    CHECK(set("widthPx", 640));
    CHECK(set("heightPx", 480));
    CHECK(set("hfovRad", 1.2));
    CHECK(set("fxPx", 500.5));
    CHECK(set("fyPx", 501.5));
    CHECK(set("cxPx", 320.25));
    CHECK(set("cyPx", 239.75));
    sdk::Value::Array coefficients;
    for (const double v : {0.1, -0.2, 0.001, 0.002, 0.3}) coefficients.push_back(sdk::Value(v));
    CHECK(set("distortion", sdk::Value(coefficients)));
    CHECK(cam().intrinsics.widthPx == 640);
    CHECK(cam().intrinsics.heightPx == 480);
    CHECK(cam().intrinsics.hfovRad == doctest::Approx(1.2));
    CHECK(cam().intrinsics.fxPx == doctest::Approx(500.5));
    CHECK(cam().intrinsics.fyPx == doctest::Approx(501.5));
    CHECK(cam().intrinsics.cxPx == doctest::Approx(320.25));
    CHECK(cam().intrinsics.cyPx == doctest::Approx(239.75));
    CHECK(cam().intrinsics.distortion[4] == doctest::Approx(0.3));

    CHECK(set("frameRateHz", 60.0));
    CHECK(set("latencyUs", 40000));
    CHECK(set("captureLatencyUs", 12000));
    CHECK(cam().frameRateHz == doctest::Approx(60.0));
    CHECK(cam().latencyUs == 40000);
    CHECK(cam().captureLatencyUs == 12000);

    CHECK(set("tagSizeM", 0.2));
    CHECK(set("minTagPixels", 8.0));
    CHECK(set("maxViewAngleRad", 1.0));
    CHECK(set("occlusion", false));
    CHECK(set("multiTag", false));
    CHECK(cam().tagSizeM == doctest::Approx(0.2));
    CHECK(cam().minTagPixels == doctest::Approx(8.0));
    CHECK(cam().maxViewAngleRad == doctest::Approx(1.0));
    CHECK_FALSE(cam().occlusion);
    CHECK_FALSE(cam().multiTag);

    CHECK(set("cornerNoisePx", 0.5));
    CHECK(set("latencyJitterUs", 5000));
    CHECK(set("dropoutProbability", 0.25));
    CHECK_FALSE(cam().realism.ideal());
    CHECK(cam().realism.cornerNoisePx == doctest::Approx(0.5));
    CHECK(cam().realism.latencyJitterUs == 5000);
    CHECK(cam().realism.dropoutProbability == doctest::Approx(0.25));

    sdk::Pose3 pose;
    pose.position = {0.1f, 0.2f, 0.3f};
    pose.rotation = sdk::Quat{0.f, 0.f, 0.f, 2.f};   // not normalised: stored as a rotation
    CHECK(set("local", sdk::toValue(pose)));
    CHECK(approx(cam().local.position, pose.position));
    CHECK(cam().local.rotation == sdk::Quat{});
    CHECK_FALSE(set("local", sdk::toValue(cam().local)));

    // The pose the Limelight believes: unset until overridden; the override starts at the physical pose.
    CHECK_FALSE(cam().assumedPose.has_value());
    CHECK(set("assumedOverride", true));
    REQUIRE(cam().assumedPose.has_value());
    CHECK(approx(cam().assumedPose->position, ops::cameraInChassis(c, cat, cam()).position));
    CHECK_FALSE(set("assumedOverride", true));
    pose.position = {-0.3f, 0.f, 0.2f};
    CHECK(set("assumedPose", sdk::toValue(pose)));
    CHECK(approx(cam().assumedPose->position, pose.position));
    CHECK(set("assumedPose", sdk::Value()));   // null: back to the physical pose
    CHECK_FALSE(cam().assumedPose.has_value());
    CHECK_FALSE(set("assumedPose", sdk::Value()));
    CHECK(set("assumedPose", sdk::toValue(sdk::Pose3{})));   // an identity pose is a pose too
    CHECK(cam().assumedPose.has_value());
    CHECK(set("assumedOverride", false));
    CHECK_FALSE(cam().assumedPose.has_value());

    // The wrong type or a number that is not finite changes nothing; the id and the vendor are fixed.
    const sdk::CameraConfig snapshot = cam();
    CHECK_FALSE(set("widthPx", "wide"));
    CHECK_FALSE(set("widthPx", std::nan("")));
    CHECK_FALSE(set("frameRateHz", std::numeric_limits<double>::infinity()));
    CHECK_FALSE(set("enabled", 1));
    CHECK_FALSE(set("label", 3));
    CHECK_FALSE(set("local", sdk::Value(3)));
    CHECK_FALSE(set("distortion", sdk::Value::makeArray()));
    CHECK_FALSE(set("assumedPose", sdk::Value(true)));
    CHECK_FALSE(set("id", "other"));
    CHECK_FALSE(set("vendor", "limelight"));
    CHECK_FALSE(set("nope", 1));
    CHECK_FALSE(ops::setCameraProperty(c, cat, "ghost", "label", sdk::Value("x")));
    CHECK(cam() == snapshot);

    // The attachment goes through the keep-the-world-pose path.
    const sdk::Vec3 before = cameraWorldPos(c, cat, "photon_front");
    CHECK(set("parent", "turret"));
    CHECK(c.findCamera("photon_front")->attachment.parentMechanismId == "turret");
    CHECK(approx(cameraWorldPos(c, cat, "photon_front"), before));
    CHECK(set("parentLink", "mover"));
    CHECK(approx(cameraWorldPos(c, cat, "photon_front"), before));
    CHECK_FALSE(set("parentLink", "nowhere"));
}

TEST_CASE("camera ops: the camera matrix is frozen from the field of view or derived again") {
    auto c = robot();
    const auto in = [&]() -> const sdk::CameraIntrinsics& { return c.findCamera("photon_front")->intrinsics; };
    REQUIRE_FALSE(ops::hasExplicitIntrinsics(in()));
    CHECK(ops::effectiveHfovRad(in()) == doctest::Approx(in().hfovRad).epsilon(1e-4));
    const double fx = in().fx();

    CHECK(ops::setCameraExplicitIntrinsics(c, "photon_front", true));
    CHECK(ops::hasExplicitIntrinsics(in()));
    CHECK(in().fxPx == doctest::Approx(fx));
    CHECK(in().fyPx == doctest::Approx(fx));
    CHECK(in().cxPx == doctest::Approx(640.0));
    CHECK(in().cyPx == doctest::Approx(400.0));
    CHECK_FALSE(ops::setCameraExplicitIntrinsics(c, "photon_front", true));   // already explicit and equal
    CHECK(ops::effectiveHfovRad(in()) == doctest::Approx(1.5708).epsilon(1e-3));

    // The field of view is not used while the focal length is set.
    ops::findCamera(c, "photon_front")->intrinsics.fxPx = 320.0;
    CHECK(ops::effectiveHfovRad(in()) == doctest::Approx(2.0 * std::atan(2.0)));

    CHECK(ops::setCameraExplicitIntrinsics(c, "photon_front", false));
    CHECK_FALSE(ops::hasExplicitIntrinsics(in()));
    CHECK_FALSE(ops::setCameraExplicitIntrinsics(c, "photon_front", false));
    CHECK_FALSE(ops::setCameraExplicitIntrinsics(c, "ghost", true));

    ops::findCamera(c, "photon_front")->intrinsics.hfovRad = 0.0;   // nothing to freeze
    CHECK_FALSE(ops::setCameraExplicitIntrinsics(c, "photon_front", true));
    CHECK(ops::effectiveHfovRad(in()) == 0.0);
}

TEST_CASE("camera ops: the vision settings are the layout, a custom file and the PhotonLib UUID, cleaned up as they are typed") {
    auto c = robot();
    CHECK(ops::setVisionProperty(c, "layoutId", sdk::Value("game_2026.andymark")));
    CHECK(c.vision.layoutId == "game_2026.andymark");
    CHECK_FALSE(ops::setVisionProperty(c, "layoutId", sdk::Value("game_2026.andymark")));
    CHECK(ops::setVisionProperty(c, "layoutFile", sdk::Value("  tags/field.json ")));
    CHECK(c.vision.layoutFile == "tags/field.json");
    CHECK(ops::setVisionProperty(c, "photonMessageUuid", sdk::Value(" 4B2FF16A964B5E2BF04BE0C1454D91C4\n")));
    CHECK(c.vision.photonMessageUuid == "4b2ff16a964b5e2bf04be0c1454d91c4");
    CHECK(ops::setVisionProperty(c, "photonMessageUuid", sdk::Value("")));
    CHECK(c.vision.photonMessageUuid.empty());
    CHECK_FALSE(ops::setVisionProperty(c, "layoutId", sdk::Value(3)));
    CHECK_FALSE(ops::setVisionProperty(c, "cameras", sdk::Value("x")));
    CHECK(c.vision.cameras.size() == 2);   // the cameras are not a vision setting

    CHECK(spec::isPhotonMessageUuid(""));
    CHECK(spec::isPhotonMessageUuid("4b2ff16a964b5e2bf04be0c1454d91c4"));
    CHECK(spec::isPhotonMessageUuid("4B2FF16A964B5E2BF04BE0C1454D91C4"));
    CHECK_FALSE(spec::isPhotonMessageUuid("4b2ff16a-964b-5e2b-f04b-e0c1454d91c4"));
    CHECK_FALSE(spec::isPhotonMessageUuid("4b2ff16a964b5e2bf04be0c1454d91c"));
    CHECK_FALSE(spec::isPhotonMessageUuid("zb2ff16a964b5e2bf04be0c1454d91c4"));
}

// ---- document and controller -------------------------------------------------------------------------

TEST_CASE("camera document: add, duplicate, re-attach and remove are undoable") {
    EditorDocument doc;
    doc.setCatalog(catalog());
    doc.load(robot());

    const QString id = doc.addCamera(kLimelight, {"turret", "mover"});
    REQUIRE_FALSE(id.isEmpty());
    CHECK(doc.config().vision.cameras.size() == 3);
    CHECK(doc.undoStack()->undoText() == "Add Limelight camera");
    doc.undoStack()->undo();
    CHECK(doc.config().vision.cameras.size() == 2);
    doc.undoStack()->redo();
    CHECK(doc.config().findCamera(id.toStdString()) != nullptr);

    const sdk::Vec3 before = cameraWorldPos(doc.config(), doc.catalog(), "photon_front");
    REQUIRE(doc.reattachCamera("photon_front", "turret", "mover"));
    CHECK(approx(cameraWorldPos(doc.config(), doc.catalog(), "photon_front"), before));
    CHECK_FALSE(doc.reattachCamera("photon_front", "ghost"));
    doc.undoStack()->undo();
    CHECK(doc.config().findCamera("photon_front")->attachment.onChassis());

    const QString copy = doc.duplicateCamera("photon_front");
    CHECK(copy == "photon_front_2");
    CHECK(doc.duplicateCamera("ghost").isEmpty());
    doc.undoStack()->undo();
    CHECK(doc.config().findCamera("photon_front_2") == nullptr);

    REQUIRE(doc.removeCamera("limelight_rear"));
    CHECK_FALSE(doc.removeCamera("limelight_rear"));
    CHECK(doc.config().findCamera("limelight_rear") == nullptr);
    doc.undoStack()->undo();
    CHECK(doc.config().findCamera("limelight_rear") != nullptr);
}

TEST_CASE("camera document: adding for a detected name is undoable and an already configured name adds nothing") {
    EditorDocument doc;
    doc.setCatalog(catalog());
    doc.load(robot());
    const int steps = doc.undoStack()->count();
    const QString id = doc.addCameraFromDetected(detection("photon-arm", kPhoton));
    CHECK(id == "photon_arm");
    CHECK(doc.undoStack()->count() == steps + 1);
    CHECK(doc.undoStack()->undoText().contains("photon-arm"));
    CHECK(doc.addCameraFromDetected(detection("photon-arm", kPhoton)).isEmpty());
    CHECK(doc.addCameraFromDetected(detection("photon-front", kPhoton)).isEmpty());   // the sample robot's own camera
    CHECK(doc.undoStack()->count() == steps + 1);
    doc.undoStack()->undo();
    CHECK(doc.config().findCamera("photon_arm") == nullptr);
}

TEST_CASE("camera document: typing in one field is one undo step, a gizmo drag is one step, a different field starts a new one") {
    EditorDocument doc;
    doc.setCatalog(catalog());
    doc.load(robot());
    const SelectionRef sel = SelectionRef::camera("photon_front");
    const int steps = doc.undoStack()->count();

    REQUIRE(doc.setProperty(sel, "camera.optics", "frameRateHz", sdk::Value(45.0)));
    REQUIRE(doc.setProperty(sel, "camera.optics", "frameRateHz", sdk::Value(50.0)));
    CHECK(doc.undoStack()->count() == steps + 1);
    REQUIRE(doc.setProperty(sel, "camera.optics", "latencyMs", sdk::Value(40.0)));
    CHECK(doc.undoStack()->count() == steps + 2);
    CHECK_FALSE(doc.setProperty(sel, "camera.optics", "latencyMs", sdk::Value(40.0)));   // no change, no step
    CHECK(doc.undoStack()->count() == steps + 2);
    CHECK(doc.config().findCamera("photon_front")->latencyUs == 40000);
    doc.undoStack()->undo();
    CHECK(doc.config().findCamera("photon_front")->latencyUs == 25000);
    doc.undoStack()->undo();
    CHECK(doc.config().findCamera("photon_front")->frameRateHz == doctest::Approx(30.0));

    const auto start = readTransform(doc.config(), doc.catalog(), sel);
    REQUIRE(start);
    const int before = doc.undoStack()->index();   // the undone steps above are dropped by the first new edit
    for (int i = 1; i <= 3; ++i) {
        EditableTransform t = start->transform;
        t.local.position.y += 0.05f * static_cast<float>(i);
        REQUIRE(doc.setTransform(sel, t, "g1"));
    }
    CHECK(doc.undoStack()->index() == before + 1);
    CHECK(approx(doc.config().findCamera("photon_front")->local.position.y, start->transform.local.position.y + 0.15f));
    doc.undoStack()->undo();
    CHECK(approx(doc.config().findCamera("photon_front")->local.position.y, start->transform.local.position.y));
}

TEST_CASE("camera document: the vision settings are undoable edits of the project") {
    EditorDocument doc;
    doc.setCatalog(catalog());
    doc.load(robot());
    REQUIRE(doc.setProperty(SelectionRef::vision(), "robot.vision", "layoutId", sdk::Value("game_2026.andymark")));
    CHECK(doc.config().vision.layoutId == "game_2026.andymark");
    REQUIRE(doc.setProperty(SelectionRef::robot(), "robot.vision", "photonMessageUuid", sdk::Value("4B2FF16A964B5E2BF04BE0C1454D91C4")));
    CHECK(doc.config().vision.photonMessageUuid == "4b2ff16a964b5e2bf04be0c1454d91c4");
    doc.undoStack()->undo();
    CHECK(doc.config().vision.photonMessageUuid.empty());
    doc.undoStack()->undo();
    CHECK(doc.config().vision.layoutId.empty());
    CHECK(doc.config().vision.cameras.size() == 2);
}

TEST_CASE("camera controller: adding selects the camera and delete, duplicate and the detected list act on it") {
    FakeEditorBackend backend;
    EditorController c(backend);
    c.setAutosaveDelayMs(0);
    const QString id = c.addCamera(kLimelight, {"intake", "root"});
    REQUIRE_FALSE(id.isEmpty());
    CHECK(c.selection().current() == SelectionRef::camera(id.toStdString()));
    CHECK(c.selectionExists(SelectionRef::camera(id.toStdString())));
    CHECK_FALSE(c.selectionExists(SelectionRef::camera("ghost")));

    REQUIRE(c.duplicateSelection());
    const SelectionRef copy = c.selection().current();
    CHECK(copy.kind == SelKind::Camera);
    CHECK(copy.owner != id.toStdString());
    CHECK(c.document().config().vision.cameras.size() == 4);

    REQUIRE(c.deleteSelection());
    CHECK(c.document().config().vision.cameras.size() == 3);
    CHECK(c.selection().current() == SelectionRef::robot());   // the selected camera is gone
}

TEST_CASE("camera controller: the cameras the robot uses come from the backend, adding one for a detected name selects it") {
    FakeEditorBackend backend;
    EditorController c(backend);
    c.setAutosaveDelayMs(0);
    CHECK(c.detectedCameras().empty());
    int updates = 0;
    QObject::connect(&c, &EditorController::detectedCamerasUpdated, &c, [&] { ++updates; });

    backend.setDetectedCameras(FakeEditorBackend::sampleDetectedCameras());
    CHECK(updates == 1);
    REQUIRE(c.detectedCameras().size() == 3);
    CHECK(c.detectedCameras()[1].ntName == "photon-arm");

    sdk::CameraRobotState update = detection("photon-arm", kPhoton);
    update.pipelineIndex = 2;
    backend.announceCamera(update);   // the same camera again: updated, not duplicated
    CHECK(updates == 2);
    REQUIRE(c.detectedCameras().size() == 3);
    CHECK(c.detectedCameras()[1].pipelineIndex == 2);

    const QString id = c.addCameraFromDetected(c.detectedCameras()[1]);
    CHECK(id == "photon_arm");
    CHECK(c.selection().current() == SelectionRef::camera("photon_arm"));
    CHECK(c.addCameraFromDetected(c.detectedCameras()[1]).isEmpty());   // now it is configured
    c.document().undoStack()->undo();
    CHECK(c.document().config().findCamera("photon_arm") == nullptr);

    backend.setDetectedCameras({});
    CHECK(c.detectedCameras().empty());
}

// ---- property groups ---------------------------------------------------------------------------------

TEST_CASE("camera property groups: a PhotonVision camera gets five cards, a Limelight a sixth with the pose it believes") {
    const auto cat = catalog();
    const auto c = robot();

    const auto ids = [&](const char* camera) {
        std::vector<std::string> out;
        for (const auto& g : propertyGroupsFor(c, cat, SelectionRef::camera(camera))) out.push_back(g.id);
        return out;
    };
    CHECK(ids("photon_front") == std::vector<std::string>{"camera.general", "camera.placement", "camera.optics", "camera.detection", "camera.realism"});
    CHECK(ids("limelight_rear") ==
          std::vector<std::string>{"camera.general", "camera.placement", "camera.optics", "camera.detection", "camera.realism", "camera.assumed"});
    CHECK(propertyGroupsFor(c, cat, SelectionRef::camera("ghost")).empty());
    CHECK(describeSelection(c, cat, SelectionRef::camera("photon_front")) == "Front camera");
    CHECK(describeSelection(c, cat, SelectionRef::camera("ghost")) == "ghost");
    CHECK(describeSelection(c, cat, SelectionRef::vision()) == "Vision");

    const auto photon = propertyGroupsFor(c, cat, SelectionRef::camera("photon_front"));
    CHECK(photon[0].schema.find("label") != nullptr);
    CHECK(photon[0].schema.find("ntName") != nullptr);
    CHECK(photon[0].schema.find("enabled") != nullptr);
    CHECK(photon[0].values.find("ntName")->asString() == "photon-front");
    CHECK(photon[1].section == sdk::PropertySection::Placement);
    CHECK(photon[1].schema.find("parent") != nullptr);
    CHECK(photon[1].schema.find("parentLink") != nullptr);
    CHECK(photon[1].schema.find("local") != nullptr);
    CHECK(photon[2].schema.find("captureLatencyMs") == nullptr);   // only a Limelight splits its latency
    for (const char* key : {"widthPx", "heightPx", "hfovRad", "advancedIntrinsics", "fxPx", "fyPx", "cxPx", "cyPx", "frameRateHz", "latencyMs"})
        CHECK(photon[2].schema.find(key) != nullptr);
    for (const char* key : {"tagSizeM", "minTagPixels", "maxViewAngleRad", "occlusion", "multiTag"}) CHECK(photon[3].schema.find(key) != nullptr);
    for (const char* key : {"cornerNoisePx", "latencyJitterMs", "dropoutPercent"}) CHECK(photon[4].schema.find(key) != nullptr);

    // Angles are shown in degrees (stored in radians), lengths in the display unit, latency in milliseconds.
    CHECK(photon[2].schema.find("hfovRad")->unit == sdk::PropertyUnit::Radians);
    CHECK(photon[3].schema.find("maxViewAngleRad")->unit == sdk::PropertyUnit::Radians);
    CHECK(photon[3].schema.find("tagSizeM")->unit == sdk::PropertyUnit::Meters);
    CHECK(photon[1].schema.find("local")->unit == sdk::PropertyUnit::Meters);
    CHECK(photon[2].values.find("latencyMs")->asDouble() == doctest::Approx(25.0));
    CHECK(photon[2].values.find("frameRateHz")->asDouble() == doctest::Approx(30.0));
    CHECK(photon[4].values.find("dropoutPercent")->asDouble() == doctest::Approx(0.0));
    for (const auto& g : photon) CHECK(sdk::validateProperties(g.schema, g.values).empty());

    const auto limelight = propertyGroupsFor(c, cat, SelectionRef::camera("limelight_rear"));
    CHECK(limelight[2].schema.find("captureLatencyMs") != nullptr);
    CHECK(limelight[2].values.find("captureLatencyMs")->asDouble() == doctest::Approx(10.0));
    REQUIRE(limelight[5].schema.find("assumedOverride") != nullptr);
    CHECK_FALSE(limelight[5].values.find("assumedOverride")->asBool());
    const sdk::PropertySpec* pose = limelight[5].schema.find("assumedPose");
    REQUIRE(pose != nullptr);
    CHECK(pose->visibleWhenKey == "assumedOverride");   // the pose only shows while the override is on
    CHECK(pose->visibleWhenEquals == sdk::Value(true));
    for (const auto& g : limelight) CHECK(sdk::validateProperties(g.schema, g.values).empty());

    // The camera matrix fields only show with "Use a camera matrix"; the field of view only without it.
    CHECK(photon[2].schema.find("hfovRad")->visibleWhenKey == "advancedIntrinsics");
    CHECK(photon[2].schema.find("fxPx")->visibleWhenKey == "advancedIntrinsics");
    CHECK(photon[2].values.find("fxPx")->asDouble() == doctest::Approx(640.0).epsilon(1e-3));   // derived while unset
}

TEST_CASE("camera property groups: edits are applied in the units the forms show") {
    const auto cat = catalog();
    auto c = robot();
    const SelectionRef photon = SelectionRef::camera("photon_front");
    const SelectionRef limelight = SelectionRef::camera("limelight_rear");
    const auto cam = [&](const char* id) -> const sdk::CameraConfig& { return *c.findCamera(id); };

    CHECK(applyProperty(c, cat, photon, "camera.general", "label", sdk::Value("Arm camera")));
    CHECK(cam("photon_front").label == "Arm camera");
    CHECK(applyProperty(c, cat, photon, "camera.general", "ntName", sdk::Value("photon-arm")));
    CHECK(cam("photon_front").networkName() == "photon-arm");
    CHECK(applyProperty(c, cat, photon, "camera.general", "enabled", sdk::Value(false)));
    CHECK_FALSE(cam("photon_front").enabled);

    CHECK(applyProperty(c, cat, photon, "camera.optics", "latencyMs", sdk::Value(40.0)));
    CHECK(cam("photon_front").latencyUs == 40000);
    CHECK(applyProperty(c, cat, photon, "camera.optics", "latencyMs", sdk::Value(12.5)));
    CHECK(cam("photon_front").latencyUs == 12500);
    CHECK(applyProperty(c, cat, limelight, "camera.optics", "captureLatencyMs", sdk::Value(7.0)));
    CHECK(cam("limelight_rear").captureLatencyUs == 7000);
    CHECK(applyProperty(c, cat, photon, "camera.optics", "frameRateHz", sdk::Value(60.0)));
    CHECK(cam("photon_front").frameRateHz == doctest::Approx(60.0));
    CHECK(applyProperty(c, cat, photon, "camera.optics", "widthPx", sdk::Value(1920)));
    CHECK(cam("photon_front").intrinsics.widthPx == 1920);
    CHECK(applyProperty(c, cat, photon, "camera.optics", "hfovRad", sdk::Value(1.0)));
    CHECK(cam("photon_front").intrinsics.hfovRad == doctest::Approx(1.0));

    CHECK(applyProperty(c, cat, photon, "camera.optics", "advancedIntrinsics", sdk::Value(true)));
    CHECK(ops::hasExplicitIntrinsics(cam("photon_front").intrinsics));
    CHECK(cam("photon_front").intrinsics.cxPx == doctest::Approx(960.0));
    CHECK(applyProperty(c, cat, photon, "camera.optics", "fxPx", sdk::Value(700.0)));
    CHECK(cam("photon_front").intrinsics.fxPx == doctest::Approx(700.0));
    CHECK(applyProperty(c, cat, photon, "camera.optics", "advancedIntrinsics", sdk::Value(false)));
    CHECK_FALSE(ops::hasExplicitIntrinsics(cam("photon_front").intrinsics));

    CHECK(applyProperty(c, cat, photon, "camera.detection", "tagSizeM", sdk::Value(0.2)));
    CHECK(applyProperty(c, cat, photon, "camera.detection", "minTagPixels", sdk::Value(20.0)));
    CHECK(applyProperty(c, cat, photon, "camera.detection", "maxViewAngleRad", sdk::Value(1.0)));
    CHECK(applyProperty(c, cat, photon, "camera.detection", "occlusion", sdk::Value(false)));
    CHECK(applyProperty(c, cat, photon, "camera.detection", "multiTag", sdk::Value(false)));
    CHECK(cam("photon_front").tagSizeM == doctest::Approx(0.2));
    CHECK(cam("photon_front").minTagPixels == doctest::Approx(20.0));
    CHECK(cam("photon_front").maxViewAngleRad == doctest::Approx(1.0));
    CHECK_FALSE(cam("photon_front").occlusion);
    CHECK_FALSE(cam("photon_front").multiTag);

    CHECK(applyProperty(c, cat, photon, "camera.realism", "cornerNoisePx", sdk::Value(0.4)));
    CHECK(applyProperty(c, cat, photon, "camera.realism", "latencyJitterMs", sdk::Value(5.0)));
    CHECK(applyProperty(c, cat, photon, "camera.realism", "dropoutPercent", sdk::Value(25.0)));
    CHECK(cam("photon_front").realism.latencyJitterUs == 5000);
    CHECK(cam("photon_front").realism.dropoutProbability == doctest::Approx(0.25));
    CHECK(cam("photon_front").realism.cornerNoisePx == doctest::Approx(0.4));

    CHECK(applyProperty(c, cat, limelight, "camera.assumed", "assumedOverride", sdk::Value(true)));
    REQUIRE(cam("limelight_rear").assumedPose.has_value());
    sdk::Pose3 believed = *cam("limelight_rear").assumedPose;
    believed.position.x += 0.1f;
    CHECK(applyProperty(c, cat, limelight, "camera.assumed", "assumedPose", sdk::toValue(believed)));
    CHECK(approx(cam("limelight_rear").assumedPose->position, believed.position));
    const auto groups = propertyGroupsFor(c, cat, limelight);
    CHECK(groups.back().values.find("assumedOverride")->asBool());
    CHECK(approx(static_cast<float>(groups.back().values.find("assumedPose")->get("position").get("x").asDouble()), believed.position.x));

    // Placement: re-attaching keeps the camera where it is.
    const sdk::Vec3 before = cameraWorldPos(c, cat, "photon_front");
    CHECK(applyProperty(c, cat, photon, "camera.placement", "parent", sdk::Value("turret")));
    CHECK(applyProperty(c, cat, photon, "camera.placement", "parentLink", sdk::Value("mover")));
    CHECK(approx(cameraWorldPos(c, cat, "photon_front"), before));
    sdk::Pose3 mount;
    mount.position = {0.1f, 0.2f, 0.3f};
    CHECK(applyProperty(c, cat, photon, "camera.placement", "local", sdk::toValue(mount)));
    CHECK(approx(cam("photon_front").local.position, mount.position));
    CHECK_FALSE(applyProperty(c, cat, photon, "camera.placement", "parentLink", sdk::Value("nowhere")));

    CHECK_FALSE(applyProperty(c, cat, photon, "camera.optics", "frameRateHz", sdk::Value(60.0)));   // unchanged
    CHECK_FALSE(applyProperty(c, cat, photon, "zone.general", "label", sdk::Value("x")));            // not a camera group
    CHECK_FALSE(applyProperty(c, cat, SelectionRef::camera("ghost"), "camera.general", "label", sdk::Value("x")));
}

TEST_CASE("vision property group: the Robot page and the Vision selection offer the layout, a custom file and the PhotonLib UUID") {
    const auto cat = catalog();
    auto c = robot();

    std::vector<std::string> ids;
    for (const auto& g : propertyGroupsFor(c, cat, SelectionRef::robot())) ids.push_back(g.id);
    CHECK(ids == std::vector<std::string>{"robot", "robot.start", "robot.vision"});
    const auto own = propertyGroupsFor(c, cat, SelectionRef::vision());
    REQUIRE(own.size() == 1);
    CHECK(own[0].id == "robot.vision");
    CHECK(sdk::validateProperties(own[0].schema, own[0].values).empty());

    const sdk::PropertySpec* layout = own[0].schema.find("layoutId");
    REQUIRE(layout != nullptr);
    REQUIRE(layout->options.size() == 3);   // the plugin default, then every layout the plugin ships
    CHECK(layout->options[0].value.empty());
    CHECK(layout->options[0].label == "Game plugin default (REBUILT (welded))");
    CHECK(layout->options[1].value == "game_2026.welded");
    CHECK(layout->options[2].label == "REBUILT (AndyMark)");
    CHECK(own[0].schema.find("layoutFile") != nullptr);
    CHECK(own[0].schema.find("layoutFile")->description.find("relative path") != std::string::npos);
    CHECK(own[0].schema.find("photonMessageUuid") != nullptr);

    CHECK(applyProperty(c, cat, SelectionRef::vision(), "robot.vision", "layoutId", sdk::Value("game_2026.andymark")));
    CHECK(c.vision.layoutId == "game_2026.andymark");
    CHECK(propertyGroupsFor(c, cat, SelectionRef::vision())[0].values.find("layoutId")->asString() == "game_2026.andymark");
    CHECK(applyProperty(c, cat, SelectionRef::robot(), "robot.vision", "layoutFile", sdk::Value("tags.json")));
    CHECK(c.vision.layoutFile == "tags.json");
    CHECK(applyProperty(c, cat, SelectionRef::robot(), "robot.vision", "photonMessageUuid", sdk::Value("4b2ff16a964b5e2bf04be0c1454d91c4")));
    CHECK_FALSE(applyProperty(c, cat, SelectionRef::robot(), "robot.vision", "photonMessageUuid", sdk::Value("4b2ff16a964b5e2bf04be0c1454d91c4")));

    EditorCatalog noPlugin = cat;
    noPlugin.fiducialLayouts.clear();
    const auto bare = propertyGroupsFor(c, noPlugin, SelectionRef::vision());
    CHECK(bare[0].schema.find("layoutId")->options.size() == 1);   // just "Game plugin default"
}

TEST_CASE("camera catalog: the active layout follows the vision settings, a custom file is not known to the editor") {
    const auto cat = catalog();
    REQUIRE(cat.fiducialLayouts.size() == 2);
    sdk::VisionConfig v;
    CHECK(cat.activeFiducialLayout(v) == &cat.fiducialLayouts.front());   // "" = the plugin's first layout
    v.layoutId = "game_2026.andymark";
    CHECK(cat.activeFiducialLayout(v)->displayName == "REBUILT (AndyMark)");
    v.layoutId = "game_2026.gone";
    CHECK(cat.activeFiducialLayout(v) == nullptr);
    v.layoutId = "game_2026.welded";
    v.layoutFile = "custom.json";
    CHECK(cat.activeFiducialLayout(v) == nullptr);
    CHECK(EditorCatalog{}.activeFiducialLayout(sdk::VisionConfig{}) == nullptr);
    CHECK(cat.findFiducialLayout("game_2026.welded")->tags.size() == 9);
    CHECK(cat.findFiducialLayout("nope") == nullptr);
}

// ---- validation --------------------------------------------------------------------------------------

TEST_CASE("camera validation: issue paths are located to their camera and the form field that shows them") {
    const auto cat = catalog();
    const auto c = robot();
    const auto at = [&](const std::string& path) { return locateIssue(c, cat, {sdk::Severity::Error, path, "x", {}}); };
    CHECK(at("vision.cameras[1].id").selection == SelectionRef::camera("limelight_rear"));
    CHECK(at("vision.cameras[1].id").fieldKey == "id");
    CHECK(at("vision.cameras[0].ntName").fieldKey == "ntName");
    CHECK(at("vision.cameras[0].attachment").fieldKey == "parent");
    CHECK(at("vision.cameras[0].local").fieldKey == "local");
    CHECK(at("vision.cameras[0].assumedPose").fieldKey == "assumedPose");
    CHECK(at("vision.cameras[0].intrinsics.hfovRad").fieldKey == "hfovRad");
    CHECK(at("vision.cameras[0].intrinsics.widthPx").fieldKey == "widthPx");
    CHECK(at("vision.cameras[0].frameRateHz").fieldKey == "frameRateHz");
    CHECK(at("vision.cameras[0].latencyUs").fieldKey == "latencyMs");   // the form shows milliseconds
    CHECK(at("vision.cameras[0].captureLatencyUs").fieldKey == "captureLatencyMs");
    CHECK(at("vision.cameras[0].tagSizeM").fieldKey == "tagSizeM");
    CHECK(at("vision.cameras[0].minTagPixels").fieldKey == "minTagPixels");
    CHECK(at("vision.cameras[0].maxViewAngleRad").fieldKey == "maxViewAngleRad");
    CHECK(at("vision.cameras[0].realism.cornerNoisePx").fieldKey == "cornerNoisePx");
    CHECK(at("vision.cameras[0].realism.latencyJitterUs").fieldKey == "latencyJitterMs");
    CHECK(at("vision.cameras[0].realism.dropoutProbability").fieldKey == "dropoutPercent");
    CHECK(at("vision.cameras[9].id").selection.isNone());
    CHECK(at("vision.layoutId").selection == SelectionRef::vision());
    CHECK(at("vision.layoutId").fieldKey == "layoutId");
    CHECK(at("vision.photonMessageUuid").selection == SelectionRef::vision());
    CHECK(at("vision.photonMessageUuid").fieldKey == "photonMessageUuid");
}

TEST_CASE("camera validation: the editor checks the UUID format itself and the fake validator reports what the real one does") {
    const auto cat = catalog();
    auto c = robot();
    DeviceDirectory devices;
    const auto malformed = [&] {
        for (const auto& issue : localIssues(c, cat, devices))
            if (issue.code == "photon_uuid_malformed") return issue.path == "vision.photonMessageUuid";
        return false;
    };
    CHECK_FALSE(malformed());
    c.vision.photonMessageUuid = "4b2ff16a964b5e2bf04be0c1454d91c4";
    CHECK_FALSE(malformed());
    c.vision.photonMessageUuid = "not a uuid";
    CHECK(malformed());

    FakeEditorBackend backend;
    const auto codes = [&](const sdk::RobotConfig& config) {
        std::vector<std::string> out;
        for (const auto& issue : backend.validate(config)) out.push_back(issue.code);
        return out;
    };
    const auto has = [&](const sdk::RobotConfig& config, const char* code) {
        const auto all = codes(config);
        return std::find(all.begin(), all.end(), code) != all.end();
    };
    auto bad = robot();
    bad.vision.cameras[1].ntName = "photon-front";
    bad.vision.cameras[1].vendor = kPhoton;
    CHECK(has(bad, "camera_name_duplicate"));
    bad = robot();
    bad.vision.cameras[1].enabled = true;
    bad.vision.cameras[1].id = "turret";
    CHECK(has(bad, "camera_id_duplicate"));
    bad = robot();
    bad.vision.cameras[0].id = "chassis";
    CHECK(has(bad, "camera_id_reserved"));
    bad = robot();
    bad.vision.cameras[0].intrinsics.hfovRad = 4.0;
    CHECK(has(bad, "camera_intrinsics_invalid"));
    bad = robot();
    bad.vision.cameras[0].frameRateHz = 500.0;
    CHECK(has(bad, "camera_frame_rate_invalid"));
    bad = robot();
    bad.vision.cameras[0].latencyUs = -1;
    CHECK(has(bad, "camera_latency_invalid"));
    bad = robot();
    bad.vision.cameras[0].maxViewAngleRad = 0.0;
    CHECK(has(bad, "camera_tag_params_invalid"));
    bad = robot();
    bad.vision.cameras[0].realism.dropoutProbability = 1.5;
    CHECK(has(bad, "camera_realism_invalid"));
    bad = robot();
    bad.vision.cameras[0].attachment.parentMechanismId = "ghost";
    CHECK(has(bad, "attachment_parent_missing"));
    bad = robot();
    bad.vision.layoutId = "game_2026.gone";
    CHECK(has(bad, "camera_layout_unknown"));
    bad.vision.cameras.clear();
    CHECK_FALSE(has(bad, "camera_layout_unknown"));   // without cameras the layout does not matter
}

// ---- viewport: transforms ----------------------------------------------------------------------------

TEST_CASE("camera transform: the pose is editable in the parent link's frame, there is no size") {
    const auto cat = catalog();
    auto c = robot();
    const auto onChassis = readTransform(c, cat, SelectionRef::camera("photon_front"));
    REQUIRE(onChassis);
    CHECK(onChassis->transform.canMove);
    CHECK(onChassis->transform.canRotate);
    CHECK_FALSE(onChassis->transform.canScale);
    CHECK(onChassis->transform.sizeKind == SizeKind::None);
    CHECK(approx(onChassis->world().position, cameraWorldPos(c, cat, "photon_front")));

    REQUIRE(ops::reattachCamera(c, cat, "photon_front", "turret", "mover"));
    const auto onTurret = readTransform(c, cat, SelectionRef::camera("photon_front"));
    REQUIRE(onTurret);
    CHECK(approx(onTurret->world().position, cameraWorldPos(c, cat, "photon_front")));
    CHECK(approx(onTurret->parentWorld.position, ops::mechanismRootWorld(c, cat, "turret").position + sdk::Vec3{0.f, 0.f, 0.05f}, 0.2f));

    EditableTransform t = onTurret->transform;
    t.local.position = {0.5f, 0.1f, 0.2f};
    t.local.rotation = sdk::Quat{0.f, 0.f, 0.f, 3.f};   // not normalised
    REQUIRE(writeTransform(c, cat, SelectionRef::camera("photon_front"), t));
    CHECK(approx(c.findCamera("photon_front")->local.position, {0.5f, 0.1f, 0.2f}));
    CHECK(c.findCamera("photon_front")->local.rotation == sdk::Quat{});
    CHECK_FALSE(readTransform(c, cat, SelectionRef::camera("ghost")));
    CHECK_FALSE(writeTransform(c, cat, SelectionRef::camera("ghost"), t));
}

// ---- backend -----------------------------------------------------------------------------------------

TEST_CASE("camera backend: the scripted detection run announces the cameras the robot uses, and a new run starts from nothing") {
    FakeEditorBackend backend;
    backend.setDetectionStepMs(5);
    CHECK(backend.detectedCameras().empty());
    int announcements = 0;
    QObject::connect(&backend, &IEditorBackend::detectedCamerasChanged, &backend, [&] { ++announcements; });
    backend.startDetection();
    pumpEvents(150);
    CHECK_FALSE(backend.detecting());
    REQUIRE(backend.detectedCameras().size() == 3);
    CHECK(backend.detectedCameras()[0].cameraId == "photon_front");   // the configured camera
    CHECK(backend.detectedCameras()[1].cameraId.empty());              // two the model does not list
    CHECK(backend.detectedCameras()[2].vendor == kLimelight);
    CHECK(announcements == 3);

    backend.startDetection();
    CHECK(backend.detectedCameras().empty());
    backend.stopDetection();
}
