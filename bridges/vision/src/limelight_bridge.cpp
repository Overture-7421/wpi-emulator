#include "limelight_bridge.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>

#include "vision_math.h"

namespace wpiemu::bridges::vision {

namespace {

using detail::FieldOrigin;
using detail::pose6;

constexpr std::int64_t kStddevCount = 12;
constexpr const char* kPipelineType = "pipe_fiducial";

// Locale-independent number formatting for the JSON dump (never NaN/inf, which JSON cannot carry).
class JsonOut {
public:
    JsonOut() {
        os_.imbue(std::locale::classic());
        os_ << std::setprecision(10);
    }
    JsonOut& raw(const char* s) {
        os_ << s;
        return *this;
    }
    JsonOut& key(const char* k) {
        os_ << '"' << k << "\":";
        return *this;
    }
    JsonOut& num(double v) {
        os_ << (std::isfinite(v) ? v : 0.0);
        return *this;
    }
    JsonOut& str(const char* s) {
        os_ << '"' << s << '"';
        return *this;
    }
    JsonOut& array(const std::vector<double>& v) {
        os_ << '[';
        for (std::size_t i = 0; i < v.size(); ++i) {
            if (i != 0) os_ << ',';
            num(v[i]);
        }
        os_ << ']';
        return *this;
    }
    std::string take() { return os_.str(); }

private:
    std::ostringstream os_;
};

std::vector<double> zeros(std::size_t n) { return std::vector<double>(n, 0.0); }

struct TagView {
    const sdk::TagDetection* tag = nullptr;
    sdk::Pose3 robotToTag;  // chassis frame -> tag
};

}  // namespace

LimelightBridge::LimelightBridge(std::shared_ptr<INtBackend> backend, VisionBridgeOptions options)
    : NtVisionBridgeBase(std::move(backend), "bridge.vision.limelight", options) {}

void LimelightBridge::configure(const sdk::VisionSetup& setup) {
    fieldLengthM_ = setup.fieldLengthM;
    fieldWidthM_ = setup.fieldWidthM;
    fieldSizeKnown_ = setup.hasFieldSize();
    fieldWarningLogged_ = false;
    configureCameras(setup, sdk::CameraKind::Limelight,
                     [](const sdk::CameraConfig& c) { return c.name.empty() ? "/limelight" : "/limelight-" + c.name; });
}

// The robot pose in the field convention `origin`. Blue is our own frame; red and center need the field size, which
// comes from the game plugin's AprilTag layout. Without it those two are not published (nullopt) rather than guessed.
std::optional<sdk::Pose3> LimelightBridge::poseInOrigin(const sdk::Pose3& blue, FieldOrigin origin) {
    if (origin != FieldOrigin::Blue && !fieldSizeKnown_) {
        if (!fieldWarningLogged_) {
            fieldWarningLogged_ = true;
            sink_.warn("the game's AprilTag layout has no field size; only the blue-origin Limelight pose is published");
        }
        return std::nullopt;
    }
    return detail::inFieldOrigin(blue, origin, fieldLengthM_, fieldWidthM_);
}

void LimelightBridge::onInitializeCamera(Cam& cam, std::int64_t ntNow) {
    cam.restartHeartbeat();
    publishFrame(cam, sdk::CameraFrame{}, ntNow, true);
    sink_.putDouble(cam.root + "/hb", static_cast<double>(cam.heartbeat), ntNow);
    sink_.putDouble(cam.root + "/getpipe", cam.pipeline, ntNow);
    sink_.putString(cam.root + "/getpipetype", kPipelineType, ntNow);
}

void LimelightBridge::onPollCamera(Cam& cam, sdk::SimTimeUs simTimeUs, std::int64_t ntNow) {
    const std::optional<double> pipeline = ntAsDouble(sink_.read(cam.root + "/pipeline"));
    if (pipeline && *pipeline != cam.pipeline) {
        cam.pipeline = *pipeline;
        sink_.putDouble(cam.root + "/getpipe", cam.pipeline, ntNow);
    }

    if (cam.beat(simTimeUs, options_.heartbeatPeriodUs))
        sink_.putDouble(cam.root + "/hb", static_cast<double>(cam.heartbeat), ntNow);
}

void LimelightBridge::onFrame(Cam& cam, const sdk::CameraFrame& frame, std::int64_t ntNow) {
    publishFrame(cam, frame, ntNow, false);
}

void LimelightBridge::publishFrame(Cam& cam, const sdk::CameraFrame& frame, std::int64_t ntNow, bool synthetic) {
    const std::string& root = cam.root;
    const double latencyMs = frame.latencyMs();
    const double clMs = 0.5 * latencyMs;
    const double tlMs = latencyMs - clMs;

    // Primary target first (largest area), like the pipeline's default sort.
    std::vector<TagView> tags;
    tags.reserve(frame.tags.size());
    for (const sdk::TagDetection& t : frame.tags) tags.push_back({&t, frame.robotToCamera * t.cameraToTag});
    std::stable_sort(tags.begin(), tags.end(),
                     [](const TagView& a, const TagView& b) { return a.tag->areaPercent > b.tag->areaPercent; });
    const bool valid = !tags.empty();
    const sdk::TagDetection* primary = valid ? tags.front().tag : nullptr;

    sink_.putDouble(root + "/tv", valid ? 1.0 : 0.0, ntNow);
    sink_.putDouble(root + "/tid", valid ? primary->id : -1.0, ntNow);
    sink_.putDouble(root + "/tx", valid ? primary->yawDeg : 0.0, ntNow);
    sink_.putDouble(root + "/ty", valid ? primary->pitchDeg : 0.0, ntNow);
    sink_.putDouble(root + "/ta", valid ? primary->areaPercent : 0.0, ntNow);
    sink_.putDouble(root + "/txnc", valid ? primary->yawDeg : 0.0, ntNow);
    sink_.putDouble(root + "/tync", valid ? primary->pitchDeg : 0.0, ntNow);
    sink_.putDouble(root + "/tl", tlMs, ntNow);
    sink_.putDouble(root + "/cl", clMs, ntNow);

    // ---- pose estimates ------------------------------------------------------------------------------------
    std::vector<double> rawFiducials;
    for (const TagView& t : tags) {
        const double block[7] = {static_cast<double>(t.tag->id), t.tag->yawDeg,        t.tag->pitchDeg,
                                 t.tag->areaPercent,             t.tag->distanceM,
                                 sdk::length(t.robotToTag.position),                    t.tag->ambiguity};
        rawFiducials.insert(rawFiducials.end(), block, block + 7);
    }
    const bool havePose = valid && frame.hasRobotPose;
    auto poseArray = [&](FieldOrigin origin) {
        if (!havePose) return std::vector<double>{};
        const std::optional<sdk::Pose3> pose = poseInOrigin(frame.robotPoseField, origin);
        if (!pose) return std::vector<double>{};
        std::vector<double> a = pose6(*pose);
        a.push_back(latencyMs);
        a.push_back(static_cast<double>(tags.size()));
        a.push_back(frame.tagSpanM);
        a.push_back(frame.avgTagDistM);
        a.push_back(frame.avgTagAreaPercent);
        a.insert(a.end(), rawFiducials.begin(), rawFiducials.end());
        return a;
    };
    // botpose / botpose_orb are the legacy center-origin arrays; the wpi* variants use the blue / red corner.
    const std::vector<double> center = poseArray(FieldOrigin::Center);
    const std::vector<double> blue = poseArray(FieldOrigin::Blue);
    const std::vector<double> red = poseArray(FieldOrigin::Red);
    sink_.putDoubles(root + "/botpose", center, ntNow);
    sink_.putDoubles(root + "/botpose_wpiblue", blue, ntNow);
    sink_.putDoubles(root + "/botpose_wpired", red, ntNow);
    sink_.putDoubles(root + "/botpose_orb", center, ntNow);
    sink_.putDoubles(root + "/botpose_orb_wpiblue", blue, ntNow);
    sink_.putDoubles(root + "/botpose_orb_wpired", red, ntNow);
    sink_.putDoubles(root + "/rawfiducials", rawFiducials, ntNow);

    // Pose noise reported to the robot as the pipeline's std-devs: x y z roll pitch yaw for MegaTag1 then MegaTag2.
    const double posSigma = cam.config.positionNoiseM;
    const std::vector<double> sigma = {posSigma, posSigma, posSigma, 0.0, 0.0, cam.config.yawNoiseDeg};
    std::vector<double> stddevs;
    stddevs.reserve(kStddevCount);
    if (havePose) {
        stddevs.insert(stddevs.end(), sigma.begin(), sigma.end());
        stddevs.insert(stddevs.end(), sigma.begin(), sigma.end());
    } else {
        stddevs = zeros(kStddevCount);
    }
    sink_.putDoubles(root + "/stddevs", stddevs, ntNow);

    // ---- geometry of the primary tag ----------------------------------------------------------------------
    if (!synthetic) {
        cam.robotToCamera = frame.robotToCamera;
        cam.hasRobotToCamera = true;
    }
    if (cam.hasRobotToCamera) sink_.putDoubles(root + "/camerapose_robotspace", pose6(cam.robotToCamera), ntNow);
    if (valid) {
        const sdk::Pose3& cameraToTag = primary->cameraToTag;
        const sdk::Pose3& robotToTag = tags.front().robotToTag;
        sink_.putDoubles(root + "/targetpose_cameraspace", pose6(cameraToTag), ntNow);
        sink_.putDoubles(root + "/targetpose_robotspace", pose6(robotToTag), ntNow);
        sink_.putDoubles(root + "/camerapose_targetspace", pose6(cameraToTag.inverse()), ntNow);
        sink_.putDoubles(root + "/botpose_targetspace", pose6(robotToTag.inverse()), ntNow);
    } else {
        for (const char* k : {"/targetpose_cameraspace", "/targetpose_robotspace", "/camerapose_targetspace",
                              "/botpose_targetspace"}) {
            sink_.putDoubles(root + k, zeros(6), ntNow);
        }
    }

    // ---- t2d / tcornxy --------------------------------------------------------------------------------------
    std::vector<double> t2d;
    std::vector<double> corners;
    if (valid) {
        const auto& c = primary->cornersPx;
        double minX = c[0], maxX = c[0], minY = c[1], maxY = c[1];
        for (int i = 0; i < 4; ++i) {
            minX = std::min(minX, c[2 * i]);
            maxX = std::max(maxX, c[2 * i]);
            minY = std::min(minY, c[2 * i + 1]);
            maxY = std::max(maxY, c[2 * i + 1]);
        }
        const double horizontal = maxX - minX;
        const double vertical = maxY - minY;
        t2d = {1.0,
               static_cast<double>(tags.size()),
               tlMs,
               clMs,
               primary->yawDeg,
               primary->pitchDeg,
               primary->yawDeg,
               primary->pitchDeg,
               primary->areaPercent,
               static_cast<double>(primary->id),
               0.0,
               0.0,
               std::max(horizontal, vertical),
               std::min(horizontal, vertical),
               horizontal,
               vertical,
               0.0};
        corners.assign(c.begin(), c.end());
    }
    sink_.putDoubles(root + "/t2d", std::move(t2d), ntNow);
    sink_.putDoubles(root + "/tcornxy", std::move(corners), ntNow);

    // ---- json (flat LimelightResults as parsed by LimelightHelpers.getLatestResults) ------------------------
    JsonOut j;
    const double publishS = sdk::usToSeconds(ntNow);
    const double captureS = publishS - latencyMs * 1e-3;
    j.raw("{").key("pID").num(cam.pipeline).raw(",").key("tl").num(tlMs).raw(",").key("cl").num(clMs);
    j.raw(",").key("ts").num(publishS).raw(",").key("ts_rio").num(captureS);
    j.raw(",").key("ts_nt").num(static_cast<double>(ntNow)).raw(",").key("ts_sys").num(static_cast<double>(ntNow));
    j.raw(",").key("ts_us").num(static_cast<double>(ntNow)).raw(",").key("v").num(valid ? 1.0 : 0.0);
    j.raw(",").key("pTYPE").str(kPipelineType);
    j.raw(",").key("tx").num(valid ? primary->yawDeg : 0.0).raw(",").key("ty").num(valid ? primary->pitchDeg : 0.0);
    j.raw(",").key("txnc").num(valid ? primary->yawDeg : 0.0).raw(",").key("tync").num(valid ? primary->pitchDeg : 0.0);
    j.raw(",").key("ta").num(valid ? primary->areaPercent : 0.0);
    auto pose6OrZeros = [&](FieldOrigin origin) {
        if (!havePose) return zeros(6);
        const std::optional<sdk::Pose3> pose = poseInOrigin(frame.robotPoseField, origin);
        return pose ? pose6(*pose) : zeros(6);
    };
    j.raw(",").key("botpose").array(pose6OrZeros(FieldOrigin::Center));
    j.raw(",").key("botpose_wpired").array(pose6OrZeros(FieldOrigin::Red));
    j.raw(",").key("botpose_wpiblue").array(pose6OrZeros(FieldOrigin::Blue));
    j.raw(",").key("botpose_tagcount").num(static_cast<double>(tags.size()));
    j.raw(",").key("botpose_span").num(frame.tagSpanM).raw(",").key("botpose_avgdist").num(frame.avgTagDistM);
    j.raw(",").key("botpose_avgarea").num(frame.avgTagAreaPercent);
    j.raw(",").key("botpose_orb").array(pose6OrZeros(FieldOrigin::Center));
    j.raw(",").key("botpose_orb_wpiblue").array(pose6OrZeros(FieldOrigin::Blue));
    j.raw(",").key("botpose_orb_wpired").array(pose6OrZeros(FieldOrigin::Red));
    j.raw(",").key("t6c_rs").array(pose6(frame.robotToCamera));
    j.raw(",").key("Fiducial").raw("[");
    for (std::size_t i = 0; i < tags.size(); ++i) {
        const sdk::TagDetection& t = *tags[i].tag;
        const double cx = 0.25 * (t.cornersPx[0] + t.cornersPx[2] + t.cornersPx[4] + t.cornersPx[6]);
        const double cy = 0.25 * (t.cornersPx[1] + t.cornersPx[3] + t.cornersPx[5] + t.cornersPx[7]);
        if (i != 0) j.raw(",");
        j.raw("{").key("fID").num(t.id).raw(",").key("fam").str("36h11");
        j.raw(",").key("t6c_ts").array(pose6(t.cameraToTag.inverse()));
        j.raw(",").key("t6r_fs").array(pose6OrZeros(FieldOrigin::Center));
        j.raw(",").key("t6r_ts").array(pose6(tags[i].robotToTag.inverse()));
        j.raw(",").key("t6t_cs").array(pose6(t.cameraToTag));
        j.raw(",").key("t6t_rs").array(pose6(tags[i].robotToTag));
        j.raw(",").key("ta").num(t.areaPercent).raw(",").key("tx").num(t.yawDeg).raw(",").key("ty").num(t.pitchDeg);
        j.raw(",").key("txp").num(cx).raw(",").key("typ").num(cy);
        j.raw(",").key("tx_nocross").num(t.yawDeg).raw(",").key("ty_nocross").num(t.pitchDeg);
        j.raw(",").key("ts").num(0.0).raw("}");
    }
    j.raw("],\"Retro\":[],\"Classifier\":[],\"Detector\":[],\"Barcode\":[],\"PythonOut\":[]}");
    sink_.putString(root + "/json", j.take(), ntNow);
}

}  // namespace wpiemu::bridges::vision
