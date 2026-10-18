#include <doctest/doctest.h>

#include <cmath>
#include <string>
#include <vector>

#include "photon_test_support.h"
#include "vision_frame_support.h"
#include "wpiemu/bridges/vision/photon_codec.h"
#include "wpiemu/bridges/vision/photon_mapper.h"
#include "wpiemu/bridges/vision/photon_profile.h"

namespace {

namespace sdk = wpiemu::sdk;
using namespace wpiemu::bridges::vision;
using wpiemu::sdk::ErrorCode;
using wpiemu::tests::photon::MessageSpec;
using wpiemu::tests::photon::OwnedProfile;
using wpiemu::tests::photon::realMessageSpecs;
namespace support = wpiemu::tests::vision;

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

sdk::VisionFrame sampleFrame(std::vector<int> tags = {1, 2, 3, 4}) {
    support::FrameSpec spec;
    spec.fieldToRobot = support::sampleRobotPose();
    spec.robotToCamera = support::sampleFrontMount();
    spec.tagIds = std::move(tags);
    spec.sequence = 17;
    return support::makeFrame(spec, support::sampleLayout());
}

// value.<member>.<member>... for a tree of structs.
const PhotonValue& at(const PhotonValue& value, std::initializer_list<std::string_view> path) {
    const PhotonValue* current = &value;
    for (const std::string_view name : path) {
        current = current->find(name);
        REQUIRE_MESSAGE(current != nullptr, "no member " << name);
    }
    return *current;
}

double number(const PhotonValue& value, std::initializer_list<std::string_view> path) {
    return at(value, path).realValue();
}

double quaternionNorm(const PhotonValue& transform) {
    const PhotonValue& q = at(transform, {"rotation", "q"});
    return std::sqrt(number(q, {"w"}) * number(q, {"w"}) + number(q, {"x"}) * number(q, {"x"}) +
                     number(q, {"y"}) * number(q, {"y"}) + number(q, {"z"}) * number(q, {"z"}));
}

TEST_CASE("photon mapper: a frame becomes the targets PhotonVision reports, in PhotonVision's units and frames") {
    const sdk::VisionFrame frame = sampleFrame();
    REQUIRE(frame.observations.size() == 4);
    const PhotonValue result = mapPhotonResult(frame, PhotonResultTiming{975000, 1000000, 10000});

    const PhotonValue& metadata = at(result, {"metadata"});
    CHECK(at(metadata, {"sequenceID"}).intValue() == 17);
    CHECK(at(metadata, {"captureTimestampMicros"}).intValue() == 975000);
    CHECK(at(metadata, {"publishTimestampMicros"}).intValue() == 1000000);
    CHECK(at(metadata, {"timeSinceLastPong"}).intValue() == 10000);

    const PhotonValue& targets = at(result, {"targets"});
    REQUIRE(targets.children().size() == 4);
    for (std::size_t i = 0; i < 4; ++i) {
        CAPTURE(i);
        const sdk::FiducialObservation& o = frame.observations[i];
        const PhotonValue& t = targets.children()[i];
        CHECK(number(t, {"yaw"}) == doctest::Approx(o.yawRad * kRadToDeg).epsilon(1e-12));
        CHECK(number(t, {"pitch"}) == doctest::Approx(o.pitchRad * kRadToDeg).epsilon(1e-12));
        CHECK(number(t, {"area"}) == o.areaPct);
        CHECK(number(t, {"skew"}) == 0.0);  // PhotonVision reports skew 0 for AprilTags
        CHECK(at(t, {"fiducialId"}).intValue() == o.id);
        CHECK(at(t, {"objDetectId"}).intValue() == -1);
        CHECK(number(t, {"objDetectConf"}) == -1.0);
        CHECK(number(t, {"poseAmbiguity"}) == o.ambiguity);
        CHECK(number(t, {"bestCameraToTarget", "translation", "x"}) ==
              static_cast<double>(o.cameraToTagBest.position.x));
        CHECK(number(t, {"bestCameraToTarget", "translation", "y"}) ==
              static_cast<double>(o.cameraToTagBest.position.y));
        CHECK(number(t, {"bestCameraToTarget", "translation", "z"}) ==
              static_cast<double>(o.cameraToTagBest.position.z));
        CHECK(quaternionNorm(at(t, {"bestCameraToTarget"})) == doctest::Approx(1.0).epsilon(1e-15));
        CHECK(quaternionNorm(at(t, {"altCameraToTarget"})) == doctest::Approx(1.0).epsilon(1e-15));
        // Both lists carry the corners in the order BL, BR, TR, TL.
        for (const char* list : {"detectedCorners", "minAreaRectCorners"}) {
            REQUIRE(at(t, {list}).children().size() == 4);
            for (std::size_t c = 0; c < 4; ++c) {
                CHECK(number(at(t, {list}).children()[c], {"x"}) == o.cornersPx[c].x);
                CHECK(number(at(t, {list}).children()[c], {"y"}) == o.cornersPx[c].y);
            }
        }
    }
    // The tag in front of the camera and to its right (tag 1, y = 3.6 < 4.0): positive yaw, below the axis: negative
    // pitch.
    const auto tag1 = std::find_if(targets.children().begin(), targets.children().end(),
                                   [](const PhotonValue& t) { return at(t, {"fiducialId"}).intValue() == 1; });
    REQUIRE(tag1 != targets.children().end());
    CHECK(number(*tag1, {"yaw"}) > 0.0);
    CHECK(number(*tag1, {"pitch"}) < 0.0);
}

TEST_CASE("photon mapper: the multi-tag result is the field-to-camera transform with no alternate solution") {
    sdk::VisionFrame frame = sampleFrame({1, 2});
    REQUIRE(frame.multiTag.valid);
    frame.multiTag.reprojErrPx = 0.375;
    const PhotonValue result = mapPhotonResult(frame, PhotonResultTiming{});
    const PhotonValue& multi = at(result, {"multitagResult"});
    REQUIRE(multi.kind() == PhotonValue::Kind::Struct);
    const PhotonValue& pnp = at(multi, {"estimatedPose"});
    CHECK(number(pnp, {"best", "translation", "x"}) == doctest::Approx(2.30).epsilon(1e-6));
    CHECK(number(pnp, {"best", "translation", "y"}) == doctest::Approx(4.0).epsilon(1e-6));
    CHECK(number(pnp, {"best", "translation", "z"}) == doctest::Approx(0.5).epsilon(1e-6));
    CHECK(at(pnp, {"alt"}) == at(pnp, {"best"}));
    CHECK(number(pnp, {"bestReprojErr"}) == 0.375);
    CHECK(number(pnp, {"altReprojErr"}) == 0.375);
    CHECK(number(pnp, {"ambiguity"}) == 0.0);
    const PhotonValue& ids = at(multi, {"fiducialIDsUsed"});
    REQUIRE(ids.children().size() == frame.multiTag.tagIds.size());
    for (std::size_t i = 0; i < ids.children().size(); ++i)
        CHECK(ids.children()[i].intValue() == frame.multiTag.tagIds[i]);

    frame.multiTag.valid = false;
    CHECK(at(mapPhotonResult(frame, PhotonResultTiming{}), {"multitagResult"}).isAbsent());
}

TEST_CASE("photon mapper: a frame without observations is an empty result, so is any frame in driver mode") {
    sdk::VisionFrame none;
    none.sequence = 3;
    const PhotonValue empty = mapPhotonResult(none, PhotonResultTiming{});
    CHECK(at(empty, {"targets"}).kind() == PhotonValue::Kind::List);
    CHECK(at(empty, {"targets"}).children().empty());
    CHECK(at(empty, {"multitagResult"}).isAbsent());

    const PhotonValue driver = mapPhotonResult(sampleFrame({1, 2, 3}), PhotonResultTiming{}, false);
    CHECK(at(driver, {"targets"}).children().empty());
    CHECK(at(driver, {"multitagResult"}).isAbsent());
    CHECK(at(driver, {"metadata", "sequenceID"}).intValue() == 17);  // still a result of this frame
}

TEST_CASE("photon mapper: lists are cut at 127 items, keeping the largest targets and the first ids") {
    sdk::VisionFrame frame = sampleFrame({1, 2});
    const sdk::FiducialObservation first = frame.observations.front();
    frame.observations.clear();
    for (int i = 0; i < 130; ++i) {
        frame.observations.push_back(first);
        frame.observations.back().id = i;
    }
    frame.multiTag.tagIds.clear();
    for (int i = 0; i < 130; ++i) frame.multiTag.tagIds.push_back(1000 + i);
    const PhotonValue result = mapPhotonResult(frame, PhotonResultTiming{});
    const PhotonValue& targets = at(result, {"targets"});
    REQUIRE(targets.children().size() == 127);
    CHECK(at(targets.children().front(), {"fiducialId"}).intValue() == 0);
    CHECK(at(targets.children().back(), {"fiducialId"}).intValue() == 126);
    const PhotonValue& ids = at(result, {"multitagResult", "fiducialIDsUsed"});
    REQUIRE(ids.children().size() == 127);
    CHECK(ids.children().back().intValue() == 1126);
}

TEST_CASE("photon mapper: a pose is written with a unit quaternion and an exact translation") {
    const PhotonValue scaled = photonTransform3d(sdk::Pose3({1.5f, -2.25f, 0.125f}, sdk::Quat(0.f, 0.f, 0.f, 2.f)));
    CHECK(number(scaled, {"translation", "x"}) == 1.5);
    CHECK(number(scaled, {"translation", "y"}) == -2.25);
    CHECK(number(scaled, {"translation", "z"}) == 0.125);
    CHECK(number(scaled, {"rotation", "q", "w"}) == 1.0);
    CHECK(number(scaled, {"rotation", "q", "x"}) == 0.0);

    const PhotonValue zero = photonTransform3d(sdk::Pose3({0.f, 0.f, 0.f}, sdk::Quat(0.f, 0.f, 0.f, 0.f)));
    CHECK(number(zero, {"rotation", "q", "w"}) == 1.0);  // a zero quaternion is not a rotation: identity

    const sdk::Pose3 tilted({0.f, 0.f, 0.f}, sdk::Quat::fromEuler(0.1f, -0.35f, 2.9f));
    CHECK(quaternionNorm(photonTransform3d(tilted)) == doctest::Approx(1.0).epsilon(1e-15));

    // The numbers form writes exactly what it is given.
    const PhotonValue raw = photonTransform3d(1, 2, 3, 0.5, 0.25, 0.125, 0.0625);
    CHECK(number(raw, {"rotation", "q", "w"}) == 0.5);
    CHECK(number(raw, {"rotation", "q", "z"}) == 0.0625);
}

// ---- the profile / mapper contract ---------------------------------------------------------------------------

TEST_CASE("photon mapper: the real profile is fully covered by the mapper") {
    const auto codec = PhotonCodec::create(newestPhotonProfile());
    REQUIRE(codec.ok());
    const sdk::Status status = verifyPhotonMapping(codec.value());
    CHECK_MESSAGE(status.ok(), wpiemu::tests::photon::why(status));
}

TEST_CASE("photon mapper: a field the mapper cannot fill makes the profile unsupported, naming the field") {
    SUBCASE("a new field of the target") {
        std::vector<MessageSpec> specs = realMessageSpecs();
        for (MessageSpec& spec : specs) {
            if (spec.name == "PhotonTrackedTarget") spec.schema += "float64 shinyNewField;";
        }
        const OwnedProfile owned(specs);
        const auto codec = PhotonCodec::create(owned.profile());
        REQUIRE_MESSAGE(codec.ok(), wpiemu::tests::photon::why(codec));  // a well formed profile ...
        CHECK(codec.value().uuid() != newestPhotonProfile().uuid);       // ... that is a new message version
        const sdk::Status status = verifyPhotonMapping(codec.value());
        REQUIRE_FALSE(status.ok());
        CHECK(status.error().code == ErrorCode::Unsupported);
        CHECK(status.error().message.find("shinyNewField") != std::string::npos);
        CHECK(status.error().message.find(codec.value().uuid()) != std::string::npos);
    }
    SUBCASE("a new field inside the optional multi-tag result") {
        std::vector<MessageSpec> specs = realMessageSpecs();
        for (MessageSpec& spec : specs) {
            if (spec.name == "PnpResult") spec.schema += "float64 newError;";
        }
        const OwnedProfile owned(specs);
        const auto codec = PhotonCodec::create(owned.profile());
        REQUIRE(codec.ok());
        const sdk::Status status = verifyPhotonMapping(codec.value());
        REQUIRE_FALSE(status.ok());
        CHECK(status.error().message.find("multitagResult.estimatedPose.newError") != std::string::npos);
    }
    SUBCASE("a message that lost a field is fine: the mapper's extra values are ignored") {
        std::vector<MessageSpec> specs = realMessageSpecs();
        for (MessageSpec& spec : specs) {
            if (spec.name == "PhotonTrackedTarget") {
                const std::string drop = "float32 objDetectConf;";
                spec.schema.erase(spec.schema.find(drop), drop.size());
            }
        }
        const OwnedProfile owned(specs);
        const auto codec = PhotonCodec::create(owned.profile());
        REQUIRE(codec.ok());
        CHECK(verifyPhotonMapping(codec.value()).ok());
    }
}

}  // namespace
