#include <QAbstractButton>
#include <QComboBox>
#include <QFrame>
#include <QLabel>
#include <QLineEdit>
#include <QMimeData>
#include <QToolButton>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <memory>

#include "editor_page.h"
#include "model/detected_cameras.h"
#include "model/editor_document.h"
#include "model/property_groups.h"
#include "model/scene_builder.h"
#include "model/slot_status.h"
#include "model/transform_access.h"
#include "model/vision_specs.h"
#include "test_support.h"
#include "viewmodel/editor_controller.h"
#include "viewmodel/project_tree_model.h"
#include "viewmodel/scene_model.h"
#include "viewmodel/viewport_controller.h"
#include "views/detected_cameras_section.h"
#include "views/device_panel.h"
#include "views/inspector_panel.h"
#include "views/project_tree_view.h"
#include "views/property_form.h"
#include "views/viewport_widget.h"
#include "wpiemu/sdk/config/value_codec.h"

using namespace wpiemu::app::editor;
using namespace wpiemu::app::editor::testing;
namespace sdk = wpiemu::sdk;

namespace {

constexpr sdk::CameraVendor kPhoton = sdk::CameraVendor::PhotonVision;
constexpr sdk::CameraVendor kLimelight = sdk::CameraVendor::Limelight;

struct CameraPage {
    FakeEditorBackend backend;
    EditorPage page{backend};
    CameraPage() {
        page.setAutosaveDelayMs(0);
        page.resize(1400, 900);
        page.show();
        pumpEvents(60);
    }
    EditorController& c() { return page.controller(); }
    const sdk::RobotConfig& config() { return c().document().config(); }
    const sdk::CameraConfig& camera(const char* id) { return *config().findCamera(id); }
    void select(const SelectionRef& ref) {
        c().selection().set(ref);
        pumpEvents(30);
    }
};
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
    CHECK(layout->options[0].label == "Default (REBUILT (welded))");
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
    CHECK(bare[0].schema.find("layoutId")->options.size() == 1);   // just "Plugin default"
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

TEST_CASE("camera validation: the runtime warnings of the vision subsystem have no path, their code and message place them") {
    const auto cat = catalog();
    auto c = robot();
    const auto at = [&](const char* code, const char* message, const std::string& path = {}) {
        return locateIssue(c, cat, {sdk::Severity::Warning, path, message, code});
    };
    CHECK(at("vision_layout_missing", "no layout could be resolved").selection == SelectionRef::vision());
    CHECK(at("vision_layout_missing", "no layout could be resolved").fieldKey == "layoutId");
    c.vision.layoutFile = "tags.json";
    CHECK(at("vision_layout_missing", "cannot read tags.json").fieldKey == "layoutFile");

    // A configured camera with a problem: the message starts with its id.
    CHECK(at("vision_camera_status", "camera 'limelight_rear' (limelight-rear): NetworkTables is not available").selection ==
          SelectionRef::camera("limelight_rear"));
    CHECK(at("vision_camera_status", "camera 'ghost' (x): the camera is disabled").selection == SelectionRef::vision());
    // A camera the robot uses that is not simulated: by its NetworkTables name when the project has one, else the Vision page with the list.
    CHECK(at("vision_camera_unconfigured", "the robot uses the Limelight camera 'limelight-rear', which is disabled in the vision configuration").selection ==
          SelectionRef::camera("limelight_rear"));
    CHECK(at("vision_camera_unconfigured", "the robot uses the PhotonVision camera 'photon-arm', which is not in the vision configuration").selection ==
          SelectionRef::vision());
    CHECK(at("vision_camera_unconfigured", "nothing quoted here").selection == SelectionRef::vision());

    // Other issues are left to the path, and an empty path with another code locates nothing.
    CHECK(at("slot_unfilled", "camera 'photon_front'").selection.isNone());
    CHECK(at("vision_camera_status", "camera 'photon_front' (photon-front): x", "devices[2]").selection.isNone());
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

// ---- project tree ------------------------------------------------------------------------------------

TEST_CASE("camera tree: the Cameras group lists every camera with its name, vendor and attachment and selects the vision settings") {
    EditorDocument doc;
    doc.setCatalog(catalog());
    doc.load(robot());
    DeviceDirectory devices;
    ProjectTreeModel tree(doc, devices);

    const QModelIndex group = tree.indexForKey("cameras");
    REQUIRE(group.isValid());
    CHECK(tree.parent(group) == tree.index(0, 0));   // under the robot
    CHECK(tree.data(group).toString() == "Cameras");
    CHECK(tree.data(group, ProjectTreeModel::DetailRole).toString() == "2");
    CHECK(tree.refAt(group) == SelectionRef::vision());
    CHECK(tree.indexFor(SelectionRef::vision()) == group);
    CHECK((tree.flags(group) & Qt::ItemIsSelectable));
    REQUIRE(tree.rowCount(group) == 2);

    const QModelIndex photon = tree.indexFor(SelectionRef::camera("photon_front"));
    const QModelIndex limelight = tree.indexFor(SelectionRef::camera("limelight_rear"));
    REQUIRE((photon.isValid() && limelight.isValid()));
    CHECK(tree.parent(photon) == group);
    CHECK(tree.data(photon).toString() == "Front camera");
    CHECK(tree.data(photon, ProjectTreeModel::KindRole).toInt() == static_cast<int>(NodeKind::Camera));
    CHECK(tree.data(photon, ProjectTreeModel::DetailRole).toString() == QString::fromUtf8("photon-front \xC2\xB7 PhotonVision"));
    CHECK(tree.data(limelight, ProjectTreeModel::DetailRole).toString() == QString::fromUtf8("limelight-rear \xC2\xB7 Limelight"));
    CHECK_FALSE(tree.data(photon, ProjectTreeModel::DisabledRole).toBool());
    CHECK(tree.data(photon, Qt::ToolTipRole).toString().contains("PhotonVision"));

    REQUIRE(doc.reattachCamera("photon_front", "turret", "mover"));
    tree.refresh();
    CHECK(tree.data(tree.indexFor(SelectionRef::camera("photon_front")), ProjectTreeModel::DetailRole).toString().endsWith("@ turret.mover"));
    REQUIRE(doc.setProperty(SelectionRef::camera("limelight_rear"), "camera.general", "enabled", sdk::Value(false)));
    tree.refresh();
    const QModelIndex disabled = tree.indexFor(SelectionRef::camera("limelight_rear"));
    CHECK(tree.data(disabled, ProjectTreeModel::DisabledRole).toBool());
    CHECK(tree.data(disabled, ProjectTreeModel::DetailRole).toString().startsWith("disabled"));

    CHECK(doc.addCamera(kLimelight).size() > 0);
    tree.refresh();
    CHECK(tree.rowCount(tree.indexForKey("cameras")) == 3);
    CHECK(tree.data(tree.indexForKey("cameras"), ProjectTreeModel::DetailRole).toString() == "3");

    // A project without cameras keeps the (empty) group, so the menu and the vision settings stay reachable.
    auto bare = robot();
    bare.vision.cameras.clear();
    doc.load(bare);
    tree.refresh();
    REQUIRE(tree.indexForKey("cameras").isValid());
    CHECK(tree.rowCount(tree.indexForKey("cameras")) == 0);
}

TEST_CASE("camera tree: validator issues put a warning on the camera or the group they point at") {
    EditorDocument doc;
    doc.setCatalog(catalog());
    doc.load(robot());
    DeviceDirectory devices;
    ProjectTreeModel tree(doc, devices);
    const auto warning = [&](const SelectionRef& ref) { return tree.data(tree.indexFor(ref), ProjectTreeModel::WarningRole).toBool(); };
    const auto groupWarning = [&] { return tree.data(tree.indexForKey("cameras"), ProjectTreeModel::WarningRole).toBool(); };
    CHECK_FALSE(warning(SelectionRef::camera("photon_front")));
    CHECK_FALSE(groupWarning());

    tree.setIssues({{sdk::Severity::Error, "vision.cameras[1].frameRateHz", "bad rate", "camera_frame_rate_invalid"}});
    CHECK(warning(SelectionRef::camera("limelight_rear")));
    CHECK_FALSE(warning(SelectionRef::camera("photon_front")));
    CHECK(groupWarning());   // visible while the group is collapsed too
    tree.setIssues({{sdk::Severity::Info, "vision.cameras[0].id", "just so you know", {}}});
    CHECK_FALSE(warning(SelectionRef::camera("photon_front")));   // information is not a warning
    CHECK_FALSE(groupWarning());

    tree.setIssues({{sdk::Severity::Warning, "vision.layoutId", "no such layout", "camera_layout_unknown"}});
    CHECK(groupWarning());
    CHECK_FALSE(warning(SelectionRef::camera("limelight_rear")));

    // The runtime warnings of the vision subsystem have no path: the code and the message place them.
    tree.setIssues({{sdk::Severity::Warning, {}, "camera 'photon_front' (photon-front): the camera is disabled", "vision_camera_status"}});
    CHECK(warning(SelectionRef::camera("photon_front")));
    tree.setIssues({{sdk::Severity::Warning, {}, "the robot uses the PhotonVision camera 'photon-arm', which is not in the vision configuration", "vision_camera_unconfigured"}});
    CHECK(groupWarning());
    CHECK_FALSE(warning(SelectionRef::camera("photon_front")));
    tree.setIssues({});
    CHECK_FALSE(groupWarning());
}

TEST_CASE("camera tree: dragging a camera onto a mechanism or the chassis offers re-attaching it") {
    EditorDocument doc;
    doc.setCatalog(catalog());
    doc.load(robot());
    DeviceDirectory devices;
    ProjectTreeModel tree(doc, devices);

    const QModelIndex photon = tree.indexFor(SelectionRef::camera("photon_front"));
    CHECK((tree.flags(photon) & Qt::ItemIsDragEnabled));
    std::unique_ptr<QMimeData> mime(tree.mimeData({photon}));
    REQUIRE(mime != nullptr);
    CHECK(mime->hasFormat(kCameraMime));

    const QModelIndex turret = tree.indexFor(SelectionRef::mechanism("turret"));
    const QModelIndex chassis = tree.indexFor(SelectionRef::chassis());
    CHECK(tree.canDropMimeData(mime.get(), Qt::MoveAction, -1, -1, turret));
    CHECK_FALSE(tree.canDropMimeData(mime.get(), Qt::MoveAction, -1, -1, chassis));   // already there
    CHECK_FALSE(tree.canDropMimeData(mime.get(), Qt::MoveAction, -1, -1, tree.indexForKey("cameras")));
    CHECK_FALSE(tree.canDropMimeData(mime.get(), Qt::MoveAction, -1, -1, tree.indexFor(SelectionRef::camera("limelight_rear"))));
    CHECK_FALSE(tree.canDropMimeData(mime.get(), Qt::MoveAction, -1, -1, tree.indexFor(SelectionRef::slot("turret", "motor"))));

    QString droppedId;
    sdk::MechanismAttachment droppedOn;
    QObject::connect(&tree, &ProjectTreeModel::cameraDropped, &tree, [&](const QString& id, const sdk::MechanismAttachment& att) {
        droppedId = id;
        droppedOn = att;
    });
    REQUIRE(tree.dropMimeData(mime.get(), Qt::MoveAction, -1, -1, turret));
    CHECK(droppedId == "photon_front");
    CHECK(droppedOn.parentMechanismId == "turret");

    // Once it sits on the turret, the chassis is a valid target again.
    REQUIRE(doc.reattachCamera("photon_front", "turret", "mover"));
    tree.refresh();
    std::unique_ptr<QMimeData> moved(tree.mimeData({tree.indexFor(SelectionRef::camera("photon_front"))}));
    CHECK(tree.canDropMimeData(moved.get(), Qt::MoveAction, -1, -1, tree.indexFor(SelectionRef::chassis())));
    CHECK(tree.canDropMimeData(moved.get(), Qt::MoveAction, -1, -1, tree.indexFor(SelectionRef::mechanism("turret"))));   // to its origin
    REQUIRE(doc.reattachCamera("photon_front", "turret", "root"));
    tree.refresh();
    std::unique_ptr<QMimeData> atOrigin(tree.mimeData({tree.indexFor(SelectionRef::camera("photon_front"))}));
    CHECK_FALSE(tree.canDropMimeData(atOrigin.get(), Qt::MoveAction, -1, -1, tree.indexFor(SelectionRef::mechanism("turret"))));   // already there
}

// ---- the inspector -----------------------------------------------------------------------------------

TEST_CASE("camera inspector: a PhotonVision camera shows General, Placement, Optics, Detection and a collapsed Realism card") {
    CameraPage f;
    f.select(SelectionRef::camera("photon_front"));
    auto* insp = f.page.inspector();
    CHECK(insp->titleText() == "Front camera");
    for (const char* group : {"camera.general", "camera.placement", "camera.optics", "camera.detection", "camera.realism"})
        CHECK(insp->form(group) != nullptr);
    CHECK(insp->form("camera.assumed") == nullptr);   // a Limelight-only card
    CHECK(insp->form("camera.general")->hasField("label"));
    CHECK(insp->form("camera.general")->hasField("ntName"));
    CHECK(insp->form("camera.placement")->hasField("local"));
    CHECK(insp->form("camera.optics")->hasField("hfovRad"));
    CHECK_FALSE(insp->form("camera.optics")->hasField("captureLatencyMs"));
    CHECK(insp->findChild<QFrame*>("card.info") != nullptr);
    CHECK(insp->findChild<QFrame*>("card.placement") != nullptr);

    CHECK(insp->findChild<QLabel*>("camera.id")->text().contains("photon_front"));
    CHECK(insp->findChild<QLabel*>("camera.id")->text().contains("PhotonVision"));
    CHECK(insp->findChild<QLabel*>("camera.endpoint")->text().contains("/photonvision/photon-front"));
    CHECK(insp->cameraRobotStatus()->text().contains("Not seen"));
    CHECK(insp->cameraRealismHint()->text().contains("Ideal"));
    CHECK_FALSE(insp->form("camera.realism")->editorFor("cornerNoisePx")->isVisibleTo(insp));   // ideal: the card starts collapsed
    CHECK(insp->form("camera.optics")->editorFor("hfovRad")->isVisibleTo(insp));

    // Editing goes through the document: undoable, and the title follows the label.
    emit insp->form("camera.general")->editorFor("label")->edited(sdk::Value("Arm camera"));
    CHECK(f.camera("photon_front").label == "Arm camera");
    pumpEvents(30);
    CHECK(insp->titleText() == "Arm camera");
    f.c().document().undoStack()->undo();
    CHECK(f.camera("photon_front").label == "Front camera");
    pumpEvents(30);
    CHECK(insp->titleText() == "Front camera");

    emit insp->form("camera.optics")->editorFor("latencyMs")->edited(sdk::Value(40.0));
    CHECK(f.camera("photon_front").latencyUs == 40000);
    emit insp->form("camera.realism")->editorFor("dropoutPercent")->edited(sdk::Value(20.0));
    CHECK(f.camera("photon_front").realism.dropoutProbability == doctest::Approx(0.2));
    CHECK(insp->cameraRealismHint()->text().contains("dropped frames"));
    CHECK_FALSE(insp->cameraRealismHint()->text().contains("Ideal"));
}

TEST_CASE("camera inspector: the camera matrix replaces the field of view, a Limelight adds its capture share and the pose it believes") {
    CameraPage f;
    f.select(SelectionRef::camera("limelight_rear"));
    auto* insp = f.page.inspector();
    REQUIRE(insp->form("camera.assumed") != nullptr);
    CHECK(insp->form("camera.optics")->hasField("captureLatencyMs"));
    CHECK(insp->findChild<QLabel*>("camera.endpoint")->text().contains("/limelight-rear"));
    CHECK_FALSE(insp->findChild<QLabel*>("camera.endpoint")->text().contains("photonvision"));

    auto* optics = insp->form("camera.optics");
    CHECK(optics->editorFor("hfovRad")->isVisibleTo(insp));
    CHECK_FALSE(optics->editorFor("fxPx")->isVisibleTo(insp));
    emit optics->editorFor("advancedIntrinsics")->edited(sdk::Value(true));
    CHECK(ops::hasExplicitIntrinsics(f.camera("limelight_rear").intrinsics));
    CHECK_FALSE(optics->editorFor("hfovRad")->isVisibleTo(insp));
    CHECK(optics->editorFor("fxPx")->isVisibleTo(insp));
    emit optics->editorFor("advancedIntrinsics")->edited(sdk::Value(false));
    CHECK_FALSE(ops::hasExplicitIntrinsics(f.camera("limelight_rear").intrinsics));
    CHECK(optics->editorFor("hfovRad")->isVisibleTo(insp));

    auto* assumed = insp->form("camera.assumed");
    CHECK_FALSE(assumed->editorFor("assumedPose")->isVisibleTo(insp));   // the pose only shows while the override is on
    emit assumed->editorFor("assumedOverride")->edited(sdk::Value(true));
    REQUIRE(f.camera("limelight_rear").assumedPose.has_value());
    CHECK(assumed->editorFor("assumedPose")->isVisibleTo(insp));
    f.c().document().undoStack()->undo();
    CHECK_FALSE(f.camera("limelight_rear").assumedPose.has_value());
    CHECK_FALSE(assumed->editorFor("assumedPose")->isVisibleTo(insp));
}

TEST_CASE("camera inspector: what the robot reported about the camera shows in the General card") {
    CameraPage f;
    f.select(SelectionRef::camera("photon_front"));
    auto* status = f.page.inspector()->cameraRobotStatus();
    CHECK(status->text().contains("Not seen"));

    sdk::CameraRobotState seen = detection("photon-front", kPhoton);
    seen.publishing = true;
    seen.framesPublished = 42;
    f.backend.announceCamera(seen);
    CHECK(status->text().contains("uses this camera"));
    CHECK(status->text().contains("42"));
    seen.status = "photon_uuid_unsupported";
    f.backend.announceCamera(seen);
    CHECK(status->text().contains("not supported"));
    seen.status = "foreign_publisher";
    f.backend.announceCamera(seen);
    CHECK(status->text().contains("PhotonCameraSim"));
    seen.status = "ntcore_unavailable";
    f.backend.announceCamera(seen);
    CHECK(status->text().contains("NetworkTables is not available"));
    f.backend.setDetectedCameras({});
    CHECK(status->text().contains("Not seen"));

    // A camera of the robot process the code has not used (yet) is not "used"; one the code uses under another name is named.
    sdk::CameraRobotState waiting = detection("photon-front", kPhoton);
    waiting.cameraId = "photon_front";
    waiting.present = false;
    f.backend.announceCamera(waiting);
    CHECK(status->text().contains("Not seen"));
    f.backend.announceCamera(detection("photon-arm", kPhoton));
    CHECK(status->text() == "Not seen from the robot code under this name. It uses: photon-arm.");
    f.backend.announceCamera(detection("limelight-left", kLimelight));   // another vendor: not an alternative
    CHECK(status->text() == "Not seen from the robot code under this name. It uses: photon-arm.");
    sdk::CameraRobotState used = detection("photon-front", kPhoton);
    f.backend.announceCamera(used);
    CHECK(status->text() == "The robot code uses this camera.");
    f.c().document().setProperty(SelectionRef::camera("photon_front"), "camera.general", "enabled", sdk::Value(false));
    pumpEvents(30);
    CHECK(f.page.inspector()->cameraRobotStatus()->text().contains("disabled here"));
    f.c().document().undoStack()->undo();
    pumpEvents(30);

    // A name change makes the detection stop matching.
    f.backend.setDetectedCameras({});
    f.backend.announceCamera(seen);
    f.c().document().setProperty(SelectionRef::camera("photon_front"), "camera.general", "ntName", sdk::Value("other"));
    pumpEvents(30);
    CHECK(f.page.inspector()->cameraRobotStatus()->text() == "Not seen from the robot code under this name. It uses: photon-front.");
}

TEST_CASE("camera inspector: a field issue shows a badge and opens the collapsed card that holds the field") {
    CameraPage f;
    f.select(SelectionRef::camera("photon_front"));
    REQUIRE(f.c().document().setProperty(SelectionRef::camera("photon_front"), "camera.optics", "frameRateHz", sdk::Value(500.0)));
    pumpEvents(300);   // validation is debounced
    auto* insp = f.page.inspector();
    bool badge = false;
    for (auto* l : insp->form("camera.optics")->findChildren<QLabel*>())
        badge = badge || (l->toolTip().contains("frame rate") && l->isVisibleTo(insp->form("camera.optics")));
    CHECK(badge);

    // Focusing an issue of a field in the (collapsed) Realism card opens it.
    CHECK_FALSE(insp->form("camera.realism")->editorFor("cornerNoisePx")->isVisibleTo(insp));
    f.c().focusIssue({sdk::Severity::Error, "vision.cameras[0].realism.cornerNoisePx", "bad noise", "camera_realism_invalid"});
    pumpEvents(60);
    CHECK(f.c().selection().current() == SelectionRef::camera("photon_front"));
    CHECK(f.page.inspector()->form("camera.realism")->editorFor("cornerNoisePx")->isVisibleTo(f.page.inspector()));
}

TEST_CASE("vision inspector: the Robot page carries the Vision card, the Cameras group has its own page with the camera list") {
    CameraPage f;
    f.select(SelectionRef::robot());
    auto* insp = f.page.inspector();
    REQUIRE(insp->form("robot.vision") != nullptr);
    CHECK(insp->form("robot.vision")->hasField("layoutId"));
    CHECK(insp->form("robot.vision")->hasField("layoutFile"));
    CHECK(insp->form("robot.vision")->hasField("photonMessageUuid"));
    CHECK(insp->form("robot")->hasField("name"));   // the other cards are still there
    REQUIRE(insp->form("robot.start") != nullptr);
    REQUIRE(insp->visionHint() != nullptr);
    CHECK(insp->visionHint()->text().contains("REBUILT (welded)"));
    CHECK(insp->visionHint()->text().contains("9 tag(s)"));
    CHECK(insp->findChild<QLabel*>("vision.uuidHint")->text().contains("PhotonLib version"));

    // Picking the other layout is an undoable edit, and the summary follows.
    auto* layout = insp->form("robot.vision")->editorFor("layoutId");
    auto* combo = layout->findChild<QComboBox*>();
    REQUIRE(combo != nullptr);
    CHECK(combo->count() == 3);
    combo->setCurrentIndex(combo->findData("game_2026.andymark"));
    emit layout->edited(sdk::Value("game_2026.andymark"));
    CHECK(f.config().vision.layoutId == "game_2026.andymark");
    pumpEvents(30);
    CHECK(f.page.inspector()->visionHint()->text().contains("AndyMark"));
    f.c().document().undoStack()->undo();
    CHECK(f.config().vision.layoutId.empty());

    emit insp->form("robot.vision")->editorFor("layoutFile")->edited(sdk::Value("tags/off_season.json"));
    pumpEvents(30);
    CHECK(f.page.inspector()->visionHint()->text().contains("tags/off_season.json"));
    CHECK(f.page.inspector()->visionHint()->text().contains("project folder"));
    emit f.page.inspector()->form("robot.vision")->editorFor("photonMessageUuid")->edited(sdk::Value("4B2FF16A964B5E2BF04BE0C1454D91C4"));
    CHECK(f.config().vision.photonMessageUuid == "4b2ff16a964b5e2bf04be0c1454d91c4");

    f.select(SelectionRef::vision());
    insp = f.page.inspector();
    CHECK(insp->titleText() == "Vision");
    CHECK(insp->form("robot.vision") != nullptr);
    CHECK(insp->form("robot.start") == nullptr);   // only the vision settings and the camera list
    CHECK(insp->findChild<QToolButton*>("vision.camera:photon_front") != nullptr);
    CHECK(insp->findChild<QToolButton*>("vision.camera:limelight_rear") != nullptr);
    insp->findChild<QToolButton*>("vision.camera:limelight_rear")->click();
    CHECK(f.c().selection().current() == SelectionRef::camera("limelight_rear"));

    f.select(SelectionRef::vision());
    f.page.inspector()->findChild<QToolButton*>("vision.addLimelight")->click();
    REQUIRE(f.config().vision.cameras.size() == 3);
    CHECK(f.c().selection().current() == SelectionRef::camera(f.config().vision.cameras.back().id));
    CHECK(f.config().vision.cameras.back().vendor == kLimelight);
    f.select(SelectionRef::vision());
    f.page.inspector()->findChild<QToolButton*>("vision.addPhoton")->click();
    CHECK(f.config().vision.cameras.size() == 4);
    CHECK(f.config().vision.cameras.back().vendor == kPhoton);
}

TEST_CASE("vision inspector: an issue about the layout or the UUID is flagged on the Robot page and the Vision page") {
    CameraPage f;
    auto cfg = robot();
    cfg.vision.layoutId = "game_2026.gone";
    cfg.vision.photonMessageUuid = "not a uuid";
    f.backend.setConfig(cfg);
    pumpEvents(300);
    const auto badges = [&](const SelectionRef& ref) {
        f.select(ref);
        pumpEvents(300);
        QStringList tips;
        auto* form = f.page.inspector()->form("robot.vision");
        for (auto* l : form->findChildren<QLabel*>())
            if (!l->toolTip().isEmpty() && l->isVisibleTo(form)) tips << l->toolTip();
        return tips;
    };
    for (const SelectionRef& page : {SelectionRef::robot(), SelectionRef::vision()}) {
        const QStringList tips = badges(page);
        CHECK(std::any_of(tips.begin(), tips.end(), [](const QString& t) { return t.contains("fiducial layout"); }));
        CHECK(std::any_of(tips.begin(), tips.end(), [](const QString& t) { return t.contains("32 hexadecimal"); }));
    }
    CHECK(f.page.inspector()->visionHint()->text().contains("game_2026.gone"));

    // Clicking such an issue opens the Vision page with the field focused.
    QString focused;
    QObject::connect(&f.c().selection(), &SelectionState::fieldFocusRequested, &f.c(), [&](const QString& key) { focused = key; });
    f.c().focusIssue({sdk::Severity::Warning, "vision.photonMessageUuid", "bad uuid", "photon_uuid_malformed"});
    CHECK(f.c().selection().current() == SelectionRef::vision());
    CHECK(focused == "photonMessageUuid");
}

TEST_CASE("camera page: dropping a camera onto the chassis or a mechanism without links re-attaches it and is undoable") {
    CameraPage f;
    REQUIRE(f.c().document().reattachCamera("photon_front", "turret", "mover"));
    const sdk::Vec3 before = cameraWorldPos(f.config(), f.c().document().catalog(), "photon_front");
    auto* tree = f.page.tree();
    auto* model = static_cast<ProjectTreeModel*>(tree->model());
    std::unique_ptr<QMimeData> mime(model->mimeData({model->indexFor(SelectionRef::camera("photon_front"))}));
    REQUIRE(mime != nullptr);

    REQUIRE(model->dropMimeData(mime.get(), Qt::MoveAction, -1, -1, model->indexFor(SelectionRef::chassis())));
    CHECK(f.camera("photon_front").attachment.onChassis());
    CHECK(approx(cameraWorldPos(f.config(), f.c().document().catalog(), "photon_front"), before));
    f.c().document().undoStack()->undo();
    CHECK(f.camera("photon_front").attachment.parentMechanismId == "turret");

    // The intake has no named links: the drop attaches to its origin without asking.
    REQUIRE(model->dropMimeData(mime.get(), Qt::MoveAction, -1, -1, model->indexFor(SelectionRef::mechanism("intake"))));
    CHECK(f.camera("photon_front").attachment.parentMechanismId == "intake");
    CHECK(f.camera("photon_front").attachment.parentLink == "root");
    CHECK(approx(cameraWorldPos(f.config(), f.c().document().catalog(), "photon_front"), before));
}

TEST_CASE("camera page: the tree row selects the camera and Delete and Ctrl+D act on it") {
    CameraPage f;
    auto* tree = f.page.tree();
    auto* model = static_cast<ProjectTreeModel*>(tree->model());
    tree->setCurrentIndex(model->indexFor(SelectionRef::camera("limelight_rear")));
    pumpEvents(30);
    CHECK(f.c().selection().current() == SelectionRef::camera("limelight_rear"));
    tree->setCurrentIndex(model->indexForKey("cameras"));
    pumpEvents(30);
    CHECK(f.c().selection().current() == SelectionRef::vision());

    f.select(SelectionRef::camera("limelight_rear"));
    f.page.duplicateAction()->trigger();
    CHECK(f.config().vision.cameras.size() == 3);
    CHECK(f.c().selection().current().kind == SelKind::Camera);
    f.page.deleteAction()->trigger();
    CHECK(f.config().vision.cameras.size() == 2);
    CHECK(f.c().selection().current() == SelectionRef::robot());
}

// ---- the viewport scene ------------------------------------------------------------------------------

TEST_CASE("camera scene: a camera is a body behind a lens with a view pyramid, and the body and the far plane select it") {
    const auto cat = catalog();
    const auto c = robot();
    const SceneTheme theme = SceneTheme::dark();
    const auto scene = buildScene(c, cat, theme);
    const SelectionRef ref = SelectionRef::camera("photon_front");
    const sdk::Pose3 world = ops::cameraWorld(c, cat, *c.findCamera("photon_front"));
    const sdk::Vec3 axis = world.rotation.rotate({1.f, 0.f, 0.f});

    const auto* body = scene.find("camera:photon_front");
    const auto* lens = scene.find("camera:photon_front:lens");
    const auto* plane = scene.find("camera:photon_front:view");
    REQUIRE((body && lens && plane));
    CHECK(body->ref == ref);
    CHECK(body->group == ref);
    CHECK(body->pickable);
    CHECK(body->color == theme.camera);
    CHECK(approx(body->world.position, world.position - axis * 0.025f, 1e-4f));   // the camera origin is the lens: the body sits behind it
    CHECK(lens->shape == ItemShape::Cylinder);
    CHECK(lens->pickable);
    CHECK(approx(lens->world.rotation.rotate({0.f, 0.f, 1.f}), axis, 1e-4f));   // a cylinder's axis is its local Z: it looks along the camera

    // The pyramid: the far plane is one depth away along the optical axis and as wide as the field of view says.
    const float depth = cameraFrustumDepthM(c, cat);
    CHECK(depth == doctest::Approx(1.0f));   // 10 % of the 6 m demo field, at least 1 m
    CHECK(plane->role == ItemRole::Frustum);
    CHECK(plane->pickable);
    CHECK(plane->opacity < 0.2f);
    CHECK(approx(plane->world.position, world.position + axis * depth, 1e-3f));
    CHECK(plane->size.y == doctest::Approx(2.0 * depth * std::tan(c.findCamera("photon_front")->intrinsics.hfovRad / 2.0)).epsilon(1e-3));
    CHECK(plane->size.z == doctest::Approx(2.0 * depth * std::tan(c.findCamera("photon_front")->intrinsics.vfovRad() / 2.0)).epsilon(1e-3));
    for (int i = 0; i < 4; ++i) {
        const auto* ray = scene.find("camera:photon_front:ray" + std::to_string(i));
        const auto* rim = scene.find("camera:photon_front:rim" + std::to_string(i));
        REQUIRE((ray && rim));
        CHECK(ray->role == ItemRole::Frustum);
        CHECK_FALSE(ray->pickable);
        CHECK(ray->group == ref);
        // A ray runs from the lens to a far corner: its centre is half way, its length the distance to the corner.
        CHECK(sdk::length(ray->world.position - world.position) == doctest::Approx(0.5 * ray->size.x).epsilon(1e-3));
        CHECK(sdk::dot(ray->world.position - world.position, axis) == doctest::Approx(0.5 * depth).epsilon(1e-3));
    }

    std::vector<std::string> ids;
    for (const auto& i : scene.items) ids.push_back(i.id);
    std::sort(ids.begin(), ids.end());
    CHECK(std::adjacent_find(ids.begin(), ids.end()) == ids.end());
    CHECK(scene.find("camera:limelight_rear:view") != nullptr);
    CHECK(approx(scene.find("camera:limelight_rear:lens")->world.rotation.rotate({0.f, 0.f, 1.f}), cameraAxis(c, cat, "limelight_rear"), 1e-4f));

    // The depth follows the layout: 1.5 m without one (no plugin, a custom file or an unknown id).
    auto custom = c;
    custom.vision.layoutFile = "tags.json";
    CHECK(cameraFrustumDepthM(custom, cat) == doctest::Approx(1.5f));
    EditorCatalog bare = cat;
    bare.fiducialLayouts.clear();
    CHECK(cameraFrustumDepthM(c, bare) == doctest::Approx(1.5f));
    bare.fiducialLayouts = cat.fiducialLayouts;
    bare.fiducialLayouts[0].fieldLengthM = 16.5f;
    CHECK(cameraFrustumDepthM(c, bare) == doctest::Approx(1.65f));
    bare.fiducialLayouts[0].fieldLengthM = 60.f;
    CHECK(cameraFrustumDepthM(c, bare) == doctest::Approx(3.0f));   // clamped
}

TEST_CASE("camera scene: a camera that is switched off is dimmed, an unusable lens draws no pyramid, a camera on a mechanism rides with it") {
    const auto cat = catalog();
    auto c = robot();
    const SceneTheme theme = SceneTheme::dark();
    const auto before = buildScene(c, cat, theme);

    ops::findCamera(c, "photon_front")->enabled = false;
    const auto off = buildScene(c, cat, theme);
    CHECK(off.find("camera:photon_front")->color == theme.cameraOff);
    CHECK(off.find("camera:photon_front")->opacity < before.find("camera:photon_front")->opacity);
    CHECK(off.find("camera:photon_front:ray0")->opacity < before.find("camera:photon_front:ray0")->opacity);
    CHECK(off.find("camera:limelight_rear")->color == theme.camera);   // the other one is untouched

    ops::findCamera(c, "photon_front")->enabled = true;
    ops::findCamera(c, "photon_front")->intrinsics.hfovRad = 0.0;
    const auto noLens = buildScene(c, cat, theme);
    CHECK(noLens.find("camera:photon_front") != nullptr);
    CHECK(noLens.find("camera:photon_front:lens") != nullptr);
    CHECK(noLens.find("camera:photon_front:view") == nullptr);
    CHECK(noLens.find("camera:photon_front:ray0") == nullptr);

    ops::findCamera(c, "photon_front")->intrinsics.hfovRad = 1.5708;
    ops::findCamera(c, "photon_front")->intrinsics.fxPx = 320.0;   // the camera matrix wins over the field of view: 2 atan(640 / 320)
    const auto narrow = buildScene(c, cat, theme);
    CHECK(narrow.find("camera:photon_front:view")->size.y == doctest::Approx(2.0 * 2.0 * 1.0).epsilon(1e-3));   // depth 1 m, half width = depth * 640 / 320

    auto d = robot();
    REQUIRE(ops::reattachCamera(d, cat, "limelight_rear", "turret", "mover"));
    const auto start = buildScene(d, cat, theme);
    auto moved = d;
    for (auto& m : moved.mechanisms)
        if (m.id == "turret") m.mount.position.x += 0.2f;
    const auto after = buildScene(moved, cat, theme);
    CHECK(approx(after.find("camera:limelight_rear")->world.position.x, start.find("camera:limelight_rear")->world.position.x + 0.2f, 1e-3f));
    CHECK(approx(after.find("camera:limelight_rear:view")->world.position.x, start.find("camera:limelight_rear:view")->world.position.x + 0.2f, 1e-3f));
}

TEST_CASE("camera scene: the layout's tags are thin plates as the robot sees the field from its start pose, only while asked for") {
    const auto cat = catalog();
    auto c = robot();
    const SceneTheme theme = SceneTheme::dark();
    CHECK(buildScene(c, cat, theme).find("tag:4") == nullptr);   // not asked for
    SceneOptions on;
    on.showTags = true;
    const auto scene = buildScene(c, cat, theme, on);
    std::size_t plates = 0;
    for (const auto& item : scene.items)
        if (item.role == ItemRole::Tag) {
            ++plates;
            CHECK_FALSE(item.pickable);
            CHECK(item.ref.isNone());
            CHECK(item.color == theme.tag);
            CHECK(item.text == QString::number(std::stoi(item.id.substr(4))));
        }
    CHECK(plates == 9);

    // Tag 4 stands at (3.0, 0.8, 0.7) on the field facing -X; the robot starts at (1.4, 1.0) facing +X: 1.6 m ahead, 0.2 m to the right.
    const auto* tag = scene.find("tag:4");
    REQUIRE(tag != nullptr);
    CHECK(tag->text == "4");
    CHECK(approx(tag->size, {0.012f, 0.1651f, 0.1651f}, 1e-4f));
    const sdk::Vec3 normal = tag->world.rotation.rotate({1.f, 0.f, 0.f});
    CHECK(approx(normal, {-1.f, 0.f, 0.f}, 1e-4f));
    CHECK(approx(tag->world.position + normal * 0.006f, {1.6f, -0.2f, 0.7f}, 1e-3f));   // the pose is the face, the plate is behind it

    // Moving or turning the start pose moves the field around the robot.
    c.sim.startPose = sdk::StartPose{2.4f, 1.0f, 0.f};
    CHECK(approx(buildScene(c, cat, theme, on).find("tag:4")->world.position.x, 0.6f + 0.006f, 1e-3f));
    c.sim.startPose = sdk::StartPose{1.4f, 2.0f, static_cast<float>(sdk::kPi) / 2.f};   // facing +Y: tag 5 (3.0, 1.6) is 0.4 m behind it, on its right
    const auto turned = buildScene(c, cat, theme, on);
    CHECK(approx(turned.find("tag:5")->world.position + turned.find("tag:5")->world.rotation.rotate({1.f, 0.f, 0.f}) * 0.006f, {-0.4f, -1.6f, 0.7f}, 1e-3f));

    // Far from the robot the floor patch ends: nothing is drawn.
    c.sim.startPose = sdk::StartPose{30.f, 0.f, 0.f};
    std::size_t far = 0;
    for (const auto& item : buildScene(c, cat, theme, on).items) far += item.role == ItemRole::Tag ? 1 : 0;
    CHECK(far == 0);

    // A custom layout file or an unknown layout id leaves nothing to draw (the editor does not read the file).
    auto file = robot();
    file.vision.layoutFile = "tags.json";
    CHECK(buildScene(file, cat, theme, on).find("tag:4") == nullptr);
    auto unknown = robot();
    unknown.vision.layoutId = "game_2026.gone";
    CHECK(buildScene(unknown, cat, theme, on).find("tag:4") == nullptr);
    unknown.vision.layoutId = "game_2026.andymark";
    CHECK(buildScene(unknown, cat, theme, on).find("tag:9")->world.position.x == doctest::Approx(4.5f + 0.006f).epsilon(1e-3));   // the AndyMark far wall is 5.9 m out
}

TEST_CASE("camera scene: the tags the selected camera would see are tinted, by the rules of the vision engine without occlusion") {
    const auto cat = catalog();
    const auto c = robot();
    const SceneTheme theme = SceneTheme::dark();
    SceneOptions options;
    options.showTags = true;
    options.cameraId = "photon_front";
    const auto scene = buildScene(c, cat, theme, options);
    std::vector<std::string> seen;
    for (const auto& item : scene.items)
        if (item.role == ItemRole::Tag && item.color == theme.tagSeen) seen.push_back(item.text.toStdString());
    std::sort(seen.begin(), seen.end());
    CHECK(seen == std::vector<std::string>{"4", "5", "8", "9"});   // ahead of the camera and facing it; the wall behind and the side walls are not

    options.cameraId = "limelight_rear";   // looks back at the wall behind the robot
    const auto rear = buildScene(c, cat, theme, options);
    CHECK(rear.find("tag:1")->color == theme.tagSeen);
    CHECK(rear.find("tag:4")->color == theme.tag);
    options.cameraId = "ghost";
    CHECK(buildScene(c, cat, theme, options).find("tag:4")->color == theme.tag);
    options.cameraId.clear();
    CHECK(buildScene(c, cat, theme, options).find("tag:4")->color == theme.tag);

    // The rule itself: facing, the image, the minimum size and the distance.
    const sdk::CameraConfig& photon = *c.findCamera("photon_front");
    const sdk::Pose3 camera = ops::cameraWorld(c, cat, photon);
    const auto ahead = [&](float distance, float yawRad) {   // a tag on the optical axis facing the camera (yaw 0 faces away)
        const sdk::Vec3 axis = camera.rotation.rotate({1.f, 0.f, 0.f});
        return sdk::Pose3(camera.position + axis * distance, camera.rotation * sdk::Quat::fromYaw(static_cast<float>(sdk::kPi) + yawRad));
    };
    CHECK(cameraSeesTag(photon, camera, ahead(2.f, 0.f), 0.1651));
    CHECK_FALSE(cameraSeesTag(photon, camera, ahead(2.f, static_cast<float>(sdk::kPi)), 0.1651));        // faces away
    CHECK_FALSE(cameraSeesTag(photon, camera, ahead(2.f, 1.45f), 0.1651));                                  // 83 degrees off the line of sight: past the 75 limit
    CHECK(cameraSeesTag(photon, camera, ahead(2.f, 1.2f), 0.1651));                                         // 69 degrees: inside it
    CHECK_FALSE(cameraSeesTag(photon, camera, ahead(15.f, 0.f), 0.1651));                                   // 640 * 0.1651 / 15 = 7 px < 12
    CHECK(cameraSeesTag(photon, camera, ahead(15.f, 0.f), 0.5));                                            // a bigger tag is long enough
    CHECK_FALSE(cameraSeesTag(photon, camera, ahead(0.0005f, 0.f), 0.1651));                                // not in front of the lens
    CHECK_FALSE(cameraSeesTag(photon, camera, ahead(-2.f, 0.f), 0.1651));                                   // behind it
    CHECK_FALSE(cameraSeesTag(photon, camera, ahead(0.1f, 0.f), 0.1651));                                   // so close that its corners leave the image
    sdk::CameraConfig blind = photon;
    blind.intrinsics.hfovRad = 0.0;
    CHECK_FALSE(cameraSeesTag(blind, camera, ahead(2.f, 0.f), 0.1651));
    sdk::CameraConfig picky = photon;
    picky.minTagPixels = 200.0;
    CHECK_FALSE(cameraSeesTag(picky, camera, ahead(2.f, 0.f), 0.1651));
}

// ---- the viewport controller -------------------------------------------------------------------------

namespace {

struct CameraViewFixture {
    FakeEditorBackend backend;
    EditorController editor{backend};
    ViewportController view{editor};
    CameraViewFixture() {
        editor.setAutosaveDelayMs(0);
        view.setViewSize(800, 600);
        editor.settings().setSnapEnabled(false);
    }
    QPointF screen(const sdk::Vec3& world) {
        const auto px = view.camera().project(world, 800.f, 600.f);
        REQUIRE(px);
        return {static_cast<qreal>(px->first), static_cast<qreal>(px->second)};
    }
    std::size_t plates() {
        std::size_t n = 0;
        for (const auto& item : view.scene().items()) n += item.role == ItemRole::Tag ? 1 : 0;
        return n;
    }
};

}  // namespace

TEST_CASE("camera viewport: the tags follow the Tags toggle, a selected camera and the vision settings") {
    CameraViewFixture f;
    CHECK_FALSE(f.view.sceneOptions().showTags);
    CHECK(f.plates() == 0);
    CHECK(f.view.tagLabels().isEmpty());

    f.editor.selection().set(SelectionRef::camera("photon_front"));
    CHECK(f.view.sceneOptions().showTags);
    CHECK(f.view.sceneOptions().cameraId == "photon_front");
    CHECK(f.plates() == 9);
    f.editor.selection().set(SelectionRef::camera("limelight_rear"));
    CHECK(f.view.sceneOptions().cameraId == "limelight_rear");
    CHECK(f.view.scene().itemById("tag:1")->color == f.view.theme().tagSeen);   // the other camera sees the wall behind the robot
    f.editor.selection().set(SelectionRef::vision());
    CHECK(f.view.sceneOptions().showTags);
    CHECK(f.view.sceneOptions().cameraId.empty());
    CHECK(f.plates() == 9);
    f.editor.selection().set(SelectionRef::robot());
    CHECK(f.plates() == 0);

    f.editor.settings().setShowTags(true);
    CHECK(f.plates() == 9);
    f.editor.selection().set(SelectionRef::mechanism("turret"));
    CHECK(f.plates() == 9);   // the toggle keeps them
    f.editor.settings().setShowTags(false);
    CHECK(f.plates() == 0);

    // Editing the vision settings redraws them: another layout, or none.
    f.editor.selection().set(SelectionRef::vision());
    REQUIRE(f.editor.document().setProperty(SelectionRef::vision(), "robot.vision", "layoutFile", sdk::Value("tags.json")));
    CHECK(f.plates() == 0);
    REQUIRE(f.editor.document().setProperty(SelectionRef::vision(), "robot.vision", "layoutFile", sdk::Value("")));
    CHECK(f.plates() == 9);
}

TEST_CASE("camera viewport: the ids of the tags in view are projected to the screen and follow the viewport camera") {
    CameraViewFixture f;
    f.editor.selection().set(SelectionRef::camera("photon_front"));
    f.view.setView("top");
    f.view.frameSelection();   // the camera and its pyramid fill the view

    // A label per plate on screen, at the plate's screen position and in its colour; none for the plates outside the view.
    const auto labelsByText = [&] {
        std::map<QString, QVariantMap> byText;
        for (const QVariant& v : f.view.tagLabels()) byText[v.toMap().value("text").toString()] = v.toMap();
        return byText;
    };
    const auto check = [&] {
        const auto byText = labelsByText();
        std::size_t inView = 0;
        for (const SceneItem& item : f.view.scene().items()) {
            if (item.role != ItemRole::Tag) continue;
            const auto px = f.view.camera().project(item.world.position, 800.f, 600.f);
            const bool visible = px && px->first >= 0.f && px->first <= 800.f && px->second >= 0.f && px->second <= 600.f;
            const auto it = byText.find(item.text);
            if (!visible) {
                CHECK(it == byText.end());
                continue;
            }
            ++inView;
            REQUIRE(it != byText.end());
            CHECK(it->second.value("x").toDouble() == doctest::Approx(static_cast<double>(px->first)).epsilon(1e-3));
            CHECK(it->second.value("y").toDouble() == doctest::Approx(static_cast<double>(px->second)).epsilon(1e-3));
            CHECK(it->second.value("color").value<QColor>() == item.color);
        }
        return inView;
    };
    const std::size_t top = check();
    CHECK(top >= 3);
    CHECK(top == f.view.tagLabels().size());
    CHECK(labelsByText().at("4").value("color").value<QColor>() == f.view.theme().tagSeen);   // a tag the camera sees

    int changes = 0;
    QObject::connect(&f.view, &ViewportController::tagLabelsChanged, &f.view, [&] { ++changes; });
    f.view.setView("front");
    CHECK(changes >= 1);   // the labels follow the viewport camera
    check();
    f.view.camera().target = {50.f, 50.f, 0.f};   // looking far away from the field: nothing is in view
    f.view.wheel(0);
    CHECK(f.view.tagLabels().isEmpty());

    f.editor.selection().set(SelectionRef::robot());   // the tags are gone with the selection that asked for them
    f.view.resetCamera();
    CHECK(f.view.tagLabels().isEmpty());
}

TEST_CASE("camera viewport: picking, gizmo modes and a drag that is one undo step") {
    CameraViewFixture f;
    CHECK(f.view.chooseItem({"i:camera:photon_front", "i:hull"}) == SelectionRef::camera("photon_front"));   // the body is solid
    CHECK(f.view.chooseItem({"i:camera:photon_front:lens"}) == SelectionRef::camera("photon_front"));
    CHECK(f.view.chooseItem({"i:camera:photon_front:view"}) == SelectionRef::camera("photon_front"));        // the plane alone selects it
    CHECK(f.view.chooseItem({"i:camera:photon_front:view", "i:hull"}) == SelectionRef::chassis());            // solid parts win over the plane

    f.editor.selection().set(SelectionRef::camera("photon_front"));
    REQUIRE(f.view.effectiveMode());
    f.view.setGizmoMode(GizmoMode::Scale);
    CHECK(*f.view.effectiveMode() == GizmoMode::Move);   // a camera has no size
    f.view.setGizmoMode(GizmoMode::Rotate);
    CHECK(*f.view.effectiveMode() == GizmoMode::Rotate);
    CHECK(f.view.selectionEdges().size() == 24);   // the body and the lens are outlined, the pyramid is not
    CHECK(f.view.transformFields().targetName == "Front camera");

    // Highlighting the camera brightens its pyramid.
    const auto opacityOf = [&](const char* id) {
        const auto& items = f.view.scene().items();
        for (std::size_t i = 0; i < items.size(); ++i)
            if (items[i].id == id) return f.view.scene().data(f.view.scene().index(static_cast<int>(i)), SceneModel::OpacityRole).toDouble();
        return -1.0;
    };
    const double selected = opacityOf("camera:photon_front:ray0");
    CHECK(selected > opacityOf("camera:limelight_rear:ray0"));
    f.editor.selection().set(SelectionRef::robot());
    CHECK(opacityOf("camera:photon_front:ray0") < selected);
    f.editor.selection().set(SelectionRef::camera("photon_front"));

    f.view.setGizmoMode(GizmoMode::Move);
    const auto start = readTransform(f.editor.document().config(), f.editor.document().catalog(), SelectionRef::camera("photon_front"));
    REQUIRE(start);
    const sdk::Vec3 origin = start->world().position;
    const sdk::Vec3 axis = start->parentWorld.rotation.rotate({1.f, 0.f, 0.f});
    const QPointF p0 = f.screen(origin + axis * 0.2f);
    REQUIRE(f.view.beginGizmoDrag("h:m:0", p0.x(), p0.y()));
    for (int i = 1; i <= 4; ++i) {
        const QPointF p = f.screen(origin + axis * (0.2f + 0.05f * static_cast<float>(i)));
        f.view.updateGizmoDrag(p.x(), p.y(), false);
    }
    f.view.endGizmoDrag();
    CHECK(approx(f.editor.document().config().findCamera("photon_front")->local.position.x, start->transform.local.position.x + 0.2f, 2e-3f));
    CHECK(f.editor.document().undoStack()->count() == 1);
    f.editor.document().undoStack()->undo();
    CHECK(approx(f.editor.document().config().findCamera("photon_front")->local.position.x, start->transform.local.position.x));

    // The numeric entry of the toolbar edits the camera too (position in metres).
    REQUIRE(f.view.setTransformComponent(1, 0.2));
    CHECK(approx(f.editor.document().config().findCamera("photon_front")->local.position.y, 0.2f));
}

TEST_CASE("camera viewport: framing a camera includes its view, framing the robot does not") {
    CameraViewFixture f;
    f.editor.selection().set(SelectionRef::camera("photon_front"));
    f.view.frameSelection();
    CHECK(f.view.camera().distance > 2.0f);   // the pyramid reaches a metre ahead: farther than the body alone

    f.editor.selection().set(SelectionRef::robot());
    f.view.frameSelection();
    const float withCameras = f.view.camera().distance;
    auto bare = robot();
    bare.vision.cameras.clear();
    f.backend.setConfig(bare);
    f.editor.selection().set(SelectionRef::robot());
    f.view.frameSelection();
    CHECK(f.view.camera().distance == doctest::Approx(withCameras).epsilon(0.1));   // the bodies sit inside the hull's reach, the pyramids are ignored
}

TEST_CASE("camera page: the Tags button toggles the AprilTags and the viewport shows them") {
    CameraPage f;
    auto* button = f.page.viewport()->findChild<QToolButton*>("viewport.tags");
    REQUIRE(button != nullptr);
    CHECK_FALSE(button->isChecked());
    CHECK_FALSE(f.c().settings().showTags());
    button->click();
    CHECK(f.c().settings().showTags());
    CHECK(button->isChecked());
    f.c().settings().setShowTags(false);   // the button follows the setting
    CHECK_FALSE(button->isChecked());
}

// ---- detected cameras (the left panel) ---------------------------------------------------------------

TEST_CASE("detected cameras: what the robot uses and the project lacks is matched by vendor and name, never by id") {
    auto c = robot();
    auto seen = FakeEditorBackend::sampleDetectedCameras();   // photon-front (configured), photon-arm and limelight-left (not)
    REQUIRE(seen.size() == 3);

    auto rows = unconfiguredCameras(c, seen);
    REQUIRE(rows.size() == 2);
    CHECK(rows[0].seen.ntName == "photon-arm");
    CHECK(rows[0].renameCandidates.empty());   // the project's only PhotonVision camera is used under its name
    CHECK(rows[1].seen.ntName == "limelight-left");
    CHECK(rows[1].renameCandidates == std::vector<std::string>{"limelight_rear"});   // the robot code does not use that one's name

    seen[0].cameraId = "from_an_older_run";   // ids mean something only to the run that reported them
    CHECK(unconfiguredCameras(c, seen).size() == 2);
    seen[1].present = false;                  // the code stopped using it
    CHECK(unconfiguredCameras(c, seen).size() == 1);
    seen[1].present = true;

    seen.push_back(detection("photon-front", kLimelight));   // the same name on another vendor is another camera
    rows = unconfiguredCameras(c, seen);
    REQUIRE(rows.size() == 3);
    CHECK(rows[2].seen.vendor == kLimelight);
    CHECK(rows[2].renameCandidates == std::vector<std::string>{"limelight_rear"});

    // The name of a camera without one is the vendor's default: "limelight" for a Limelight, the id for PhotonVision.
    ops::findCamera(c, "limelight_rear")->ntName.clear();
    ops::findCamera(c, "photon_front")->ntName.clear();
    CHECK(unconfiguredCameras(c, {detection("limelight", kLimelight), detection("photon_front", kPhoton)}).empty());
    CHECK(unconfiguredCameras(c, {}).empty());   // nothing reported: nothing to add

    CHECK(detectedCameraStatusText(detection("x", kPhoton)) == "not in the project");
    sdk::CameraRobotState foreign = detection("x", kPhoton);
    foreign.status = "foreign_publisher";
    CHECK(detectedCameraStatusText(foreign) == "published by the robot code itself");
    foreign.status = "ntcore_unavailable";
    CHECK(detectedCameraStatusText(foreign) == "NetworkTables is not available in the robot process");
    foreign.status = "something_new";
    CHECK(detectedCameraStatusText(foreign) == "status: something_new");
}

TEST_CASE("detected cameras: the panel lists the cameras the project lacks, adds one and follows the live reports") {
    CameraPage f;
    DetectedCamerasSection* section = f.page.devicePanel()->cameras();
    REQUIRE(section != nullptr);
    const auto button = [&](const QString& name) { return section->findChild<QPushButton*>(name); };
    const auto label = [&](const QString& name) { return section->findChild<QLabel*>(name); };
    CHECK_FALSE(section->isVisible());   // nothing reported: no section
    CHECK(section->rowCount() == 0);

    f.backend.setDetectedCameras(FakeEditorBackend::sampleDetectedCameras());
    pumpEvents(30);
    REQUIRE(section->isVisible());
    CHECK(section->rowCount() == 2);   // photon-front is in the project
    CHECK(label("cameras.title")->text() == "Cameras (2)");
    REQUIRE(label("cameras.name:PhotonVision:photon-arm") != nullptr);
    CHECK(label("cameras.status:PhotonVision:photon-arm")->text() == "PhotonVision \u00b7 not in the project");
    CHECK(label("cameras.status:Limelight:limelight-left")->text().startsWith("Limelight"));
    CHECK(label("cameras.name:PhotonVision:photon-front") == nullptr);
    REQUIRE(button("cameras.add:PhotonVision:photon-arm") != nullptr);
    REQUIRE(button("cameras.add:Limelight:limelight-left") != nullptr);
    CHECK(button("cameras.add:Limelight:limelight-left")->text() == "Add camera");

    // The Limelight of the project is not used under its name: the hint says so and offers the robot's name.
    REQUIRE(label("cameras.hint:Limelight:limelight-left") != nullptr);
    CHECK(label("cameras.hint:Limelight:limelight-left")->text() == "Not used by the robot code: Rear Limelight (limelight-rear). The same camera?");
    CHECK(label("cameras.hint:PhotonVision:photon-arm") == nullptr);
    CHECK(button("cameras.rename:PhotonVision:photon-arm") == nullptr);

    // Adding one configures it under the robot's name and selects it; the row goes, the other stays.
    button("cameras.add:PhotonVision:photon-arm")->click();
    pumpEvents(30);
    const sdk::CameraConfig* added = f.config().findCamera("photon_arm");
    REQUIRE(added != nullptr);
    CHECK(added->vendor == kPhoton);
    CHECK(added->networkName() == "photon-arm");
    CHECK(f.c().selection().current() == SelectionRef::camera("photon_arm"));
    CHECK(section->rowCount() == 1);
    CHECK(label("cameras.title")->text() == "Cameras (1)");
    CHECK(button("cameras.add:PhotonVision:photon-arm") == nullptr);
    f.c().document().undoStack()->undo();   // one undo step
    pumpEvents(30);
    CHECK(f.config().findCamera("photon_arm") == nullptr);
    CHECK(section->rowCount() == 2);

    // "Use this name" renames the project's camera instead (one undo step) and selects it.
    REQUIRE(button("cameras.rename:Limelight:limelight-left") != nullptr);
    button("cameras.rename:Limelight:limelight-left")->click();
    pumpEvents(30);
    CHECK(f.camera("limelight_rear").networkName() == "limelight-left");
    CHECK(f.c().selection().current() == SelectionRef::camera("limelight_rear"));
    CHECK(section->rowCount() == 1);
    CHECK(button("cameras.add:Limelight:limelight-left") == nullptr);
    f.c().document().undoStack()->undo();
    pumpEvents(30);
    CHECK(f.camera("limelight_rear").networkName() == "limelight-rear");
    CHECK(section->rowCount() == 2);

    // Live: a new camera appears, the code stops using another, the project's own camera shows up under its name.
    f.backend.announceCamera(detection("photon-new", kPhoton));
    pumpEvents(30);
    CHECK(section->rowCount() == 3);
    CHECK(button("cameras.add:PhotonVision:photon-new") != nullptr);
    sdk::CameraRobotState gone = detection("photon-new", kPhoton);
    gone.present = false;
    f.backend.announceCamera(gone);
    pumpEvents(30);
    CHECK(section->rowCount() == 2);
    f.backend.setDetectedCameras({});
    pumpEvents(30);
    CHECK_FALSE(section->isVisible());
}

TEST_CASE("detected cameras: with several unused cameras of the vendor the hint lists them and leaves the choice to the user") {
    CameraPage f;
    DetectedCamerasSection* section = f.page.devicePanel()->cameras();
    const QString second = f.c().addCamera(kLimelight);   // another Limelight the robot code does not use
    REQUIRE_FALSE(second.isEmpty());
    f.backend.setDetectedCameras({detection("limelight-left", kLimelight)});
    pumpEvents(30);
    REQUIRE(section->rowCount() == 1);
    auto* hint = section->findChild<QLabel*>("cameras.hint:Limelight:limelight-left");
    REQUIRE(hint != nullptr);
    CHECK(hint->text().startsWith("Not used by the robot code: Rear Limelight (limelight-rear), "));
    CHECK(hint->text().endsWith("set its NetworkTables name to \"limelight-left\"."));
    CHECK(section->findChild<QPushButton*>("cameras.rename:Limelight:limelight-left") == nullptr);   // ambiguous: no one-click rename
    CHECK(section->findChild<QPushButton*>("cameras.add:Limelight:limelight-left") != nullptr);
}

TEST_CASE("detected cameras: the panel is rebuilt only when what it shows changes") {
    CameraPage f;
    DetectedCamerasSection* section = f.page.devicePanel()->cameras();
    f.backend.setDetectedCameras(FakeEditorBackend::sampleDetectedCameras());
    pumpEvents(30);
    auto* add = section->findChild<QPushButton*>("cameras.add:PhotonVision:photon-arm");
    REQUIRE(add != nullptr);
    add->setProperty("kept", true);

    REQUIRE(f.c().document().setProperty(SelectionRef::camera("photon_front"), "camera.optics", "frameRateHz", sdk::Value(25.0)));
    pumpEvents(30);
    auto* again = section->findChild<QPushButton*>("cameras.add:PhotonVision:photon-arm");
    REQUIRE(again != nullptr);
    CHECK(again->property("kept").toBool());   // an unrelated edit keeps the widgets

    REQUIRE(f.c().document().setProperty(SelectionRef::camera("limelight_rear"), "camera.general", "label", sdk::Value("Back camera")));
    pumpEvents(30);
    auto* hint = section->findChild<QLabel*>("cameras.hint:Limelight:limelight-left");
    REQUIRE(hint != nullptr);
    CHECK(hint->text() == "Not used by the robot code: Back camera (limelight-rear). The same camera?");   // the hint follows the label
}
