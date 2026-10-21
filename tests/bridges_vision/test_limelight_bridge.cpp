#include <cmath>
#include <limits>
#include <string>

#include "vision_test_rig.h"

using namespace vision_test;
using doctest::Approx;
using vis::ntDouble;
using vis::ntInt;

namespace {

const std::string kLL = "/limelight";

std::vector<sdk::CameraFrame> oneTagFrame() {
    return {makeFrame(0, {makeTag(7, 2.0, 5.0, -3.0, {2.f, 0.2f, 0.f})})};
}

void configureLimelight(Rig& rig, const std::string& name = "") {
    rig.configure(makeSetup({makeCamera(sdk::CameraKind::Limelight, name)}));
}

// Angles are compared modulo 360 degrees (a yaw of +-180 is the same pose).
void checkPose(const std::vector<double>& a, double x, double y, double z, double roll, double pitch, double yaw) {
    REQUIRE(a.size() >= 6);
    CHECK(a[0] == Approx(x).epsilon(1e-4).scale(1.0));
    CHECK(a[1] == Approx(y).epsilon(1e-4).scale(1.0));
    CHECK(a[2] == Approx(z).epsilon(1e-4).scale(1.0));
    CHECK(std::abs(std::remainder(a[3] - roll, 360.0)) < 1e-3);
    CHECK(std::abs(std::remainder(a[4] - pitch, 360.0)) < 1e-3);
    CHECK(std::abs(std::remainder(a[5] - yaw, 360.0)) < 1e-3);
}

}  // namespace

TEST_CASE("vision limelight: the table is limelight or limelight-name") {
    Rig rig;
    rig.configure(makeSetup({makeCamera(sdk::CameraKind::Limelight, ""), makeCamera(sdk::CameraKind::Limelight, "front")}));
    rig.poll(0);
    CHECK(rig.nt->has("/limelight/tv"));
    CHECK(rig.nt->has("/limelight/getpipe"));
    CHECK(rig.nt->has("/limelight/json"));
    CHECK(rig.nt->has("/limelight-front/tv"));
    CHECK(rig.nt->has("/limelight-front/hb"));
    CHECK_FALSE(rig.nt->has("/limelight-/tv"));
}

TEST_CASE("vision limelight: no tag publishes tv 0 and empty arrays") {
    Rig rig;
    configureLimelight(rig);
    rig.apply({makeFrame(0, {})}, 1'040'000);
    CHECK(rig.nt->lastDouble(kLL + "/tv") == 0.0);
    CHECK(rig.nt->lastDouble(kLL + "/tid") == -1.0);
    CHECK(rig.nt->lastDouble(kLL + "/tx") == 0.0);
    CHECK(rig.nt->lastDouble(kLL + "/ta") == 0.0);
    CHECK(rig.nt->lastDoubles(kLL + "/botpose_wpiblue").empty());
    CHECK(rig.nt->lastDoubles(kLL + "/botpose_orb_wpired").empty());
    CHECK(rig.nt->lastDoubles(kLL + "/rawfiducials").empty());
    CHECK(rig.nt->lastDoubles(kLL + "/t2d").empty());
    CHECK(rig.nt->lastDoubles(kLL + "/stddevs").size() == 12);
    CHECK(rig.nt->lastDoubles(kLL + "/targetpose_cameraspace") == std::vector<double>(6, 0.0));
    const std::string* json = rig.nt->lastAs<std::string>(kLL + "/json");
    REQUIRE(json != nullptr);
    CHECK(json->find("\"v\":0") != std::string::npos);
    CHECK(json->find("\"Fiducial\":[]") != std::string::npos);
}

TEST_CASE("vision limelight: one tag fills the 2D targeting values") {
    Rig rig;
    configureLimelight(rig);
    rig.apply(oneTagFrame(), 1'040'000);
    CHECK(rig.nt->lastDouble(kLL + "/tv") == 1.0);
    CHECK(rig.nt->lastDouble(kLL + "/tid") == 7.0);
    CHECK(rig.nt->lastDouble(kLL + "/tx") == Approx(5.0));
    CHECK(rig.nt->lastDouble(kLL + "/ty") == Approx(-3.0));
    CHECK(rig.nt->lastDouble(kLL + "/ta") == Approx(2.0));
    CHECK(rig.nt->lastDouble(kLL + "/txnc") == Approx(5.0));
    CHECK(rig.nt->lastDouble(kLL + "/tync") == Approx(-3.0));

    const std::vector<double>& t2d = rig.nt->lastDoubles(kLL + "/t2d");
    REQUIRE(t2d.size() == 17);
    CHECK(t2d[0] == 1.0);
    CHECK(t2d[1] == 1.0);
    CHECK(t2d[2] == Approx(20.0));
    CHECK(t2d[3] == Approx(20.0));
    CHECK(t2d[9] == 7.0);
    CHECK(rig.nt->lastDoubles(kLL + "/tcornxy").size() == 8);
}

TEST_CASE("vision limelight: latency fields reproduce the capture time") {
    Rig rig;
    configureLimelight(rig);
    rig.nt->nowTimeUs = 9'000'000;
    rig.apply(oneTagFrame(), 1'040'000);
    const double tl = rig.nt->lastDouble(kLL + "/tl");
    const double cl = rig.nt->lastDouble(kLL + "/cl");
    CHECK(tl == Approx(20.0));
    CHECK(cl == Approx(20.0));
    // LimelightHelpers: FPGATimestamp(at publish) - (tl + cl) / 1000 == capture time.
    const auto* topic = rig.nt->topic(kLL + "/tv");
    REQUIRE(topic != nullptr);
    CHECK(topic->last().timestampUs == 9'000'000);
    CHECK(9.0 - (tl + cl) / 1000.0 == Approx(8.96));
    const std::vector<double>& botpose = rig.nt->lastDoubles(kLL + "/botpose_wpiblue");
    REQUIRE(botpose.size() == 18);
    CHECK(botpose[6] == Approx(40.0));
    CHECK(rig.nt->topic(kLL + "/botpose_wpiblue")->last().timestampUs == 9'000'000);
}

TEST_CASE("vision limelight: blue red and center pose arrays follow the field conventions") {
    Rig rig;
    configureLimelight(rig);
    rig.apply(oneTagFrame(), 1'040'000);

    const std::vector<double>& blue = rig.nt->lastDoubles(kLL + "/botpose_wpiblue");
    REQUIRE(blue.size() == 11 + 7);
    checkPose(blue, 2.0, 3.0, 0.0, 0.0, 0.0, 90.0);
    CHECK(blue[7] == 1.0);           // tag count
    CHECK(blue[8] == Approx(0.0));   // tag span (single tag)
    CHECK(blue[9] == Approx(2.5));   // average distance
    CHECK(blue[10] == Approx(1.5));  // average area

    checkPose(rig.nt->lastDoubles(kLL + "/botpose_wpired"), kTestFieldLengthM - 2.0, kTestFieldWidthM - 3.0, 0.0, 0.0, 0.0, -90.0);
    checkPose(rig.nt->lastDoubles(kLL + "/botpose"), 2.0 - kTestFieldLengthM / 2.0, 3.0 - kTestFieldWidthM / 2.0, 0.0, 0.0, 0.0, 90.0);

    // MegaTag2 arrays carry the same estimate.
    CHECK(rig.nt->lastDoubles(kLL + "/botpose_orb_wpiblue") == blue);
    CHECK(rig.nt->lastDoubles(kLL + "/botpose_orb_wpired") == rig.nt->lastDoubles(kLL + "/botpose_wpired"));
    CHECK(rig.nt->lastDoubles(kLL + "/botpose_orb") == rig.nt->lastDoubles(kLL + "/botpose"));
}

TEST_CASE("vision limelight: the red and center origins use the field size the setup carries") {
    Rig rig;
    sdk::VisionSetup setup = makeSetup({makeCamera(sdk::CameraKind::Limelight, "")});
    setup.fieldLengthM = 10.0;
    setup.fieldWidthM = 6.0;
    rig.configure(setup);
    rig.apply(oneTagFrame(), 1'040'000);
    checkPose(rig.nt->lastDoubles(kLL + "/botpose_wpired"), 8.0, 3.0, 0.0, 0.0, 0.0, -90.0);
    checkPose(rig.nt->lastDoubles(kLL + "/botpose"), -3.0, 0.0, 0.0, 0.0, 0.0, 90.0);
    CHECK(rig.logger.count(sdk::LogLevel::Warn) == 0);
}

TEST_CASE("vision limelight: without a field size only the blue-origin pose is published, with one warning") {
    for (const double bad : {0.0, -1.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
        INFO("field length " << bad);
        Rig rig;
        sdk::VisionSetup setup = makeSetup({makeCamera(sdk::CameraKind::Limelight, "")});
        setup.fieldLengthM = bad;
        CHECK_FALSE(setup.hasFieldSize());
        rig.configure(setup);
        rig.apply(oneTagFrame(), 1'040'000);
        rig.apply(oneTagFrame(), 1'060'000);

        checkPose(rig.nt->lastDoubles(kLL + "/botpose_wpiblue"), 2.0, 3.0, 0.0, 0.0, 0.0, 90.0);   // needs no field size
        CHECK(rig.nt->lastDoubles(kLL + "/botpose_wpired").empty());
        CHECK(rig.nt->lastDoubles(kLL + "/botpose").empty());
        CHECK(rig.nt->lastDoubles(kLL + "/botpose_orb_wpired").empty());
        CHECK(rig.nt->lastDoubles(kLL + "/botpose_orb").empty());
        CHECK(rig.nt->lastDouble(kLL + "/tv") == 1.0);                                              // the target itself is still seen
        const std::string* json = rig.nt->lastAs<std::string>(kLL + "/json");
        REQUIRE(json != nullptr);
        CHECK(json->find("nan") == std::string::npos);
        CHECK(rig.logger.count(sdk::LogLevel::Warn) == 1);                                          // once, not per frame
        CHECK(rig.logger.count(sdk::LogLevel::Error) == 0);
    }
}

TEST_CASE("vision limelight: rawfiducials use stride 7 and the largest tag is primary") {
    Rig rig;
    configureLimelight(rig);
    rig.apply(twoTagFrame(), 1'040'000);

    CHECK(rig.nt->lastDouble(kLL + "/tid") == 7.0);
    const std::vector<double>& raw = rig.nt->lastDoubles(kLL + "/rawfiducials");
    REQUIRE(raw.size() == 14);
    CHECK(raw[0] == 7.0);                            // id
    CHECK(raw[1] == Approx(5.0));                    // txnc
    CHECK(raw[2] == Approx(-3.0));                   // tync
    CHECK(raw[3] == Approx(2.0));                    // ta
    CHECK(raw[4] == Approx(2.00998).epsilon(1e-3));  // distance to camera
    CHECK(raw[5] > 0.0);                             // distance to robot
    CHECK(raw[6] == 0.0);                            // ambiguity
    CHECK(raw[7] == 3.0);

    const std::vector<double>& blue = rig.nt->lastDoubles(kLL + "/botpose_wpiblue");
    REQUIRE(blue.size() == 11 + 14);
    CHECK(blue[7] == 2.0);
    CHECK(blue[8] == Approx(1.25));
    CHECK(blue[11] == 7.0);
    CHECK(blue[18] == 3.0);
    CHECK(rig.nt->lastDoubles(kLL + "/t2d")[1] == 2.0);
}

TEST_CASE("vision limelight: camerapose_robotspace and the target space arrays") {
    Rig rig;
    configureLimelight(rig);
    rig.apply(oneTagFrame(), 1'040'000);
    checkPose(rig.nt->lastDoubles(kLL + "/camerapose_robotspace"), 0.3, 0.0, 0.5, 0.0, 0.0, 0.0);
    // The tag is 2 m ahead and 0.2 m left of the camera, which sits 0.3 m ahead of the robot origin.
    checkPose(rig.nt->lastDoubles(kLL + "/targetpose_cameraspace"), 2.0, 0.2, 0.0, 0.0, 0.0, 180.0);
    checkPose(rig.nt->lastDoubles(kLL + "/targetpose_robotspace"), 2.3, 0.2, 0.5, 0.0, 0.0, 180.0);
}

TEST_CASE("vision limelight: stddevs follow the configured noise") {
    Rig rig;
    sdk::CameraConfig c = makeCamera(sdk::CameraKind::Limelight, "");
    c.positionNoiseM = 0.02;
    c.yawNoiseDeg = 0.5;
    rig.configure(makeSetup({c}));
    rig.apply(oneTagFrame(), 1'040'000);
    const std::vector<double>& s = rig.nt->lastDoubles(kLL + "/stddevs");
    REQUIRE(s.size() == 12);
    CHECK(s[0] == Approx(0.02));
    CHECK(s[5] == Approx(0.5));
    CHECK(s[6] == Approx(0.02));
    CHECK(s[11] == Approx(0.5));
}

TEST_CASE("vision limelight: json carries the fiducial results LimelightHelpers parses") {
    Rig rig;
    configureLimelight(rig);
    rig.apply(twoTagFrame(), 1'040'000);
    const std::string* json = rig.nt->lastAs<std::string>(kLL + "/json");
    REQUIRE(json != nullptr);
    CHECK(json->front() == '{');
    CHECK(json->back() == '}');
    for (const char* key : {"\"pID\":0", "\"tl\":20", "\"cl\":20", "\"v\":1", "\"pTYPE\":\"pipe_fiducial\"",
                            "\"tx\":5", "\"botpose_wpiblue\":[2,3,0,0,0,", "\"botpose_tagcount\":2",
                            "\"Fiducial\":[{\"fID\":7", "{\"fID\":3", "\"t6c_ts\":[", "\"t6r_fs\":[", "\"t6r_ts\":[",
                            "\"t6t_cs\":[", "\"t6t_rs\":[", "\"t6c_rs\":[0.30000"}) {
        INFO(key);
        CHECK(json->find(key) != std::string::npos);
    }
}

TEST_CASE("vision limelight: robot-written pipeline is mirrored to getpipe and hb counts at 10 Hz") {
    Rig rig;
    configureLimelight(rig);
    rig.poll(0);
    CHECK(rig.nt->lastDouble(kLL + "/getpipe") == 0.0);
    CHECK(rig.nt->lastDouble(kLL + "/hb") == 1.0);

    rig.nt->setRobotValue(kLL + "/pipeline", ntDouble(3.0));
    rig.poll(20'000);
    CHECK(rig.nt->lastDouble(kLL + "/getpipe") == 3.0);
    CHECK(rig.nt->lastDouble(kLL + "/hb") == 1.0);

    rig.poll(100'000);
    CHECK(rig.nt->lastDouble(kLL + "/hb") == 2.0);
    rig.poll(150'000);
    CHECK(rig.nt->lastDouble(kLL + "/hb") == 2.0);
    rig.poll(200'000);
    CHECK(rig.nt->lastDouble(kLL + "/hb") == 3.0);

    // An integer written by the robot reads as a number too.
    rig.nt->setRobotValue(kLL + "/pipeline", ntInt(5));
    rig.poll(210'000);
    CHECK(rig.nt->lastDouble(kLL + "/getpipe") == 5.0);
    // The pipeline index is reported in the JSON as well.
    rig.apply(oneTagFrame(), 220'000);
    CHECK(rig.nt->lastAs<std::string>(kLL + "/json")->find("\"pID\":5") != std::string::npos);
}

TEST_CASE("vision limelight: an unavailable ntcore is skipped quietly and retried") {
    Rig rig;
    configureLimelight(rig);
    rig.nt->isAvailable = false;
    rig.poll(0);
    rig.apply(oneTagFrame(), 1'040'000);
    CHECK(rig.nt->topics.empty());
    CHECK(rig.logger.count(sdk::LogLevel::Warn) == 0);
    CHECK(rig.logger.count(sdk::LogLevel::Error) == 0);

    rig.nt->isAvailable = true;
    rig.poll(5'000);
    CHECK(rig.nt->has(kLL + "/tv"));
    CHECK(rig.nt->lastDouble(kLL + "/tv") == 0.0);
    rig.apply(oneTagFrame(), 1'040'000);
    CHECK(rig.nt->lastDouble(kLL + "/tv") == 1.0);
}

TEST_CASE("vision limelight: frames can arrive before the first poll") {
    Rig rig;
    configureLimelight(rig);
    rig.apply(oneTagFrame(), 1'040'000);
    CHECK(rig.nt->lastDouble(kLL + "/tv") == 1.0);
    CHECK(rig.nt->has(kLL + "/getpipetype"));
}

TEST_CASE("vision limelight: photonvision cameras and unknown camera indices are ignored") {
    Rig rig;
    rig.configure(makeSetup({makeCamera(sdk::CameraKind::PhotonVision, "photon"), makeCamera(sdk::CameraKind::Limelight, "ll")}));
    rig.poll(0);
    CHECK_FALSE(rig.nt->has("/limelight-photon/tv"));
    CHECK(rig.nt->has("/limelight-ll/tv"));
    rig.apply({makeFrame(0, {makeTag(7, 2.0, 5.0, -3.0, {2.f, 0.2f, 0.f})}), makeFrame(9, {})}, 1'040'000);
    CHECK(rig.nt->lastDouble("/limelight-ll/tv") == 0.0);  // the frame of camera 0 (PhotonVision) is not ours
    rig.apply({makeFrame(1, {makeTag(7, 2.0, 5.0, -3.0, {2.f, 0.2f, 0.f})})}, 1'040'000);
    CHECK(rig.nt->lastDouble("/limelight-ll/tv") == 1.0);
}

TEST_CASE("vision limelight: reset clears the published results") {
    Rig rig;
    configureLimelight(rig);
    rig.apply(twoTagFrame(), 1'040'000);
    REQUIRE(rig.nt->lastDouble(kLL + "/tv") == 1.0);
    rig.nt->nowTimeUs = 6'000'000;
    rig.limelight->reset();
    CHECK(rig.nt->lastDouble(kLL + "/tv") == 0.0);
    CHECK(rig.nt->lastDouble(kLL + "/tid") == -1.0);
    CHECK(rig.nt->lastDoubles(kLL + "/botpose_wpiblue").empty());
    CHECK(rig.nt->lastDoubles(kLL + "/rawfiducials").empty());
    CHECK(rig.nt->topic(kLL + "/tv")->last().timestampUs == 6'000'000);
}

TEST_CASE("vision limelight: reset while ntcore is unavailable clears on the next available tick") {
    Rig rig;
    configureLimelight(rig);
    rig.apply(oneTagFrame(), 1'040'000);
    rig.nt->isAvailable = false;
    rig.limelight->reset();
    CHECK(rig.nt->lastDouble(kLL + "/tv") == 1.0);
    rig.nt->isAvailable = true;
    rig.limelight->poll(10'000);
    CHECK(rig.nt->lastDouble(kLL + "/tv") == 0.0);
}

TEST_CASE("vision limelight: stop releases the camera topics") {
    Rig rig;
    configureLimelight(rig, "front");
    rig.poll(0);
    REQUIRE(rig.nt->has("/limelight-front/tv"));
    rig.limelight->stop();
    rig.limelight->stop();
    CHECK_FALSE(rig.nt->has("/limelight-front/tv"));
}
