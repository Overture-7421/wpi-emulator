#include "model/vision_specs.h"

#include <algorithm>
#include <cctype>
#include <cmath>

#include "model/config_ops.h"
#include "model/spec_values.h"
#include "model/tr.h"
#include "wpiemu/sdk/config/value_codec.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/vision/photon_profiles.h"

namespace wpiemu::app::editor::spec {

using sdk::PropertySchema;
using sdk::PropertySpec;
using sdk::PropertyType;
using sdk::PropertyUnit;
using sdk::Value;

namespace {

PropertySpec makeSpec(std::string key, const QString& label, PropertyType type, PropertyUnit unit, Value def, const QString& group,
                      const QString& description = {}) {
    PropertySpec s;
    s.key = std::move(key);
    s.label = label.toStdString();
    s.type = type;
    s.unit = unit;
    s.defaultValue = std::move(def);
    s.group = group.toStdString();
    s.description = description.toStdString();
    return s;
}

PropertySpec& withRange(PropertySpec& s, double min, double max, double step = 0.0) {
    s.min = min;
    s.max = max;
    s.step = step;
    return s;
}

PropertySpec& visibleWhen(PropertySpec& s, const std::string& key, bool equals) {
    s.visibleWhenKey = key;
    s.visibleWhenEquals = equals;
    return s;
}

constexpr double kOneDegreeRad = 0.017453292519943295;

// Latency and jitter are shown in whole milliseconds (PropertyUnit::Count has no decimals and no suffix; the label carries the unit).
std::int64_t microsFromMillis(const Value& v, bool& ok) {
    ok = v.isNumber() && std::isfinite(v.asDouble());
    return ok ? static_cast<std::int64_t>(std::llround(v.asDouble() * 1000.0)) : 0;
}

}  // namespace

// ---- one camera -------------------------------------------------------------------------------
PropertySchema cameraGeneralSchema(const sdk::CameraConfig& cam) {
    const bool limelight = cam.vendor == sdk::CameraVendor::Limelight;
    const QString group = ETR("General");
    PropertySchema s;
    s.properties.push_back(makeSpec("label", ETR("Label"), PropertyType::String, PropertyUnit::None, std::string(), group,
                                    ETR("Name shown in the project tree and the viewport.")));
    s.properties.push_back(makeSpec(
        "ntName", ETR("NetworkTables name"), PropertyType::String, PropertyUnit::None, std::string(), group,
        limelight ? ETR("The Limelight's NetworkTables table: \"limelight\", or \"limelight-<hostname>\" for a named device. It must match "
                        "the name your robot code passes to LimelightHelpers.")
                  : ETR("The camera nickname set in PhotonVision. It must match the name your robot code gives its PhotonCamera.")));
    s.properties.push_back(makeSpec("enabled", ETR("Enabled"), PropertyType::Bool, PropertyUnit::None, true, group,
                                    ETR("A disabled camera is not simulated and publishes nothing.")));
    return s;
}

PropertySchema cameraPlacementSchema(const sdk::RobotConfig& c, const EditorCatalog& catalog, const sdk::CameraConfig& cam) {
    return attachmentSchema(c, catalog, cam.attachment,
                            {ETR("A camera moves with the part it is attached to. The placement is kept when you change it."),
                             ETR("Named frame of the parent the camera hangs on (e.g. a turret's moving part)."),
                             ETR("Relative to the attachment link: x forward, y left, z up. The camera looks along its +X axis.")});
}

PropertySchema cameraOpticsSchema(const sdk::CameraConfig& cam) {
    const bool limelight = cam.vendor == sdk::CameraVendor::Limelight;
    PropertySchema s;
    const QString image = ETR("Image");
    auto width = makeSpec("widthPx", ETR("Width (px)"), PropertyType::Int, PropertyUnit::Count, 1280, image);
    s.properties.push_back(withRange(width, 1, 16384, 1));
    auto height = makeSpec("heightPx", ETR("Height (px)"), PropertyType::Int, PropertyUnit::Count, 800, image);
    s.properties.push_back(withRange(height, 1, 16384, 1));

    const QString lens = ETR("Lens");
    auto hfov = makeSpec("hfovRad", ETR("Horizontal field of view"), PropertyType::Double, PropertyUnit::Radians, 1.5708, lens,
                         ETR("Pinhole model; the vertical field of view follows from the image shape."));
    s.properties.push_back(visibleWhen(withRange(hfov, kOneDegreeRad, sdk::kPi - kOneDegreeRad, kOneDegreeRad), "advancedIntrinsics", false));
    s.properties.push_back(makeSpec("advancedIntrinsics", ETR("Use a camera matrix"), PropertyType::Bool, PropertyUnit::None, false, lens,
                                    ETR("Enter calibrated focal lengths and the principal point in pixels instead of deriving them from the "
                                        "field of view.")));
    const struct { const char* key; const char* label; } matrix[] = {{"fxPx", QT_TRANSLATE_NOOP("Editor", "Focal length x (px)")},
                                                                     {"fyPx", QT_TRANSLATE_NOOP("Editor", "Focal length y (px)")},
                                                                     {"cxPx", QT_TRANSLATE_NOOP("Editor", "Principal point x (px)")},
                                                                     {"cyPx", QT_TRANSLATE_NOOP("Editor", "Principal point y (px)")}};
    for (const auto& m : matrix) {
        auto p = makeSpec(m.key, QCoreApplication::translate("Editor", m.label), PropertyType::Double, PropertyUnit::None, 0.0, lens);
        s.properties.push_back(visibleWhen(withRange(p, 0.0, 100000.0, 0.5), "advancedIntrinsics", true));
    }

    const QString timing = ETR("Timing");
    auto rate = makeSpec("frameRateHz", ETR("Frame rate"), PropertyType::Double, PropertyUnit::Hertz, 30.0, timing,
                         ETR("Frames processed per second; at most one capture per simulation step."));
    s.properties.push_back(withRange(rate, 1.0, 240.0, 1.0));
    auto latency = makeSpec("latencyMs", ETR("Latency (ms)"), PropertyType::Double, PropertyUnit::Count, 25.0, timing,
                            ETR("From the moment of capture until the result is on NetworkTables; rounded up to whole simulation steps."));
    s.properties.push_back(withRange(latency, 0.0, 1000.0, 1.0));
    if (limelight) {
        auto capture = makeSpec("captureLatencyMs", ETR("Capture latency (ms)"), PropertyType::Double, PropertyUnit::Count, 10.0, timing,
                                ETR("The share of the latency the Limelight publishes as 'cl' (image capture); the rest is 'tl' "
                                    "(processing)."));
        s.properties.push_back(withRange(capture, 0.0, 1000.0, 1.0));
    }
    return s;
}

PropertySchema cameraDetectionSchema() {
    const QString group = ETR("Detection");
    PropertySchema s;
    auto size = makeSpec("tagSizeM", ETR("Tag size"), PropertyType::Double, PropertyUnit::Meters, 0.1651, group,
                         ETR("Edge length of the black square of a tag. 0 uses the size the AprilTag layout gives."));
    s.properties.push_back(withRange(size, 0.0, 2.0, 0.0));
    auto pixels = makeSpec("minTagPixels", ETR("Minimum tag size (px)"), PropertyType::Double, PropertyUnit::Count, 12.0, group,
                           ETR("The shortest projected tag edge that is still detected: farther tags are missed."));
    s.properties.push_back(withRange(pixels, 0.0, 1000.0, 1.0));
    auto angle = makeSpec("maxViewAngleRad", ETR("Maximum view angle"), PropertyType::Double, PropertyUnit::Radians, 1.309, group,
                          ETR("Tags seen at a steeper angle than this, between the line of sight and the tag's normal, are not detected."));
    s.properties.push_back(withRange(angle, kOneDegreeRad, sdk::kPi, kOneDegreeRad));
    s.properties.push_back(makeSpec("occlusion", ETR("Occlusion"), PropertyType::Bool, PropertyUnit::None, true, group,
                                    ETR("A tag hidden behind a field structure is not detected.")));
    s.properties.push_back(makeSpec("multiTag", ETR("Multi-tag pose"), PropertyType::Bool, PropertyUnit::None, true, group,
                                    ETR("Solve the pose from every visible tag together (PhotonVision multi-tag, Limelight MegaTag).")));
    return s;
}

PropertySchema cameraRealismSchema() {
    const QString group = ETR("Realism");
    PropertySchema s;
    auto noise = makeSpec("cornerNoisePx", ETR("Corner noise (px)"), PropertyType::Double, PropertyUnit::None, 0.0, group,
                          ETR("Standard deviation of the noise added to every detected corner coordinate."));
    s.properties.push_back(withRange(noise, 0.0, 20.0, 0.05));
    auto jitter = makeSpec("latencyJitterMs", ETR("Latency jitter (ms)"), PropertyType::Double, PropertyUnit::Count, 0.0, group,
                           ETR("Random variation of the latency, plus or minus this much. It cannot exceed the latency."));
    s.properties.push_back(withRange(jitter, 0.0, 1000.0, 1.0));
    auto dropout = makeSpec("dropoutPercent", ETR("Dropped frames"), PropertyType::Double, PropertyUnit::Percent, 0.0, group,
                            ETR("The chance that a frame detects nothing."));
    s.properties.push_back(withRange(dropout, 0.0, 100.0, 1.0));
    return s;
}

PropertySchema cameraAssumedSchema() {
    const QString group = ETR("Limelight pose");
    PropertySchema s;
    s.properties.push_back(makeSpec("assumedOverride", ETR("Override the pose the Limelight believes"), PropertyType::Bool, PropertyUnit::None, false,
                                    group,
                                    ETR("By default the Limelight knows where it really is. Override it to simulate a camera pose that was "
                                        "entered wrongly in its web UI (the robot code can still replace it with setCameraPose_RobotSpace).")));
    auto pose = makeSpec("assumedPose", ETR("Believed pose (chassis frame)"), PropertyType::Pose3, PropertyUnit::Meters, sdk::toValue(sdk::Pose3{}),
                         group, ETR("x forward, y left, z up from the chassis origin (the centre of mass)."));
    s.properties.push_back(visibleWhen(pose, "assumedOverride", true));
    return s;
}

Value cameraValues(const sdk::RobotConfig& c, const EditorCatalog& catalog, const sdk::CameraConfig& cam) {
    const sdk::CameraIntrinsics& in = cam.intrinsics;
    const auto finiteOr = [](double v, double fallback) { return std::isfinite(v) ? v : fallback; };
    Value o = Value::makeObject();
    o["label"] = cam.label;
    o["ntName"] = cam.networkName();
    o["enabled"] = cam.enabled;

    o["parent"] = cam.attachment.onChassis() ? std::string(sdk::kChassisParent) : cam.attachment.parentMechanismId;
    o["parentLink"] = cam.attachment.parentLink;
    o["local"] = sdk::toValue(cam.local);

    o["widthPx"] = in.widthPx;
    o["heightPx"] = in.heightPx;
    o["hfovRad"] = in.hfovRad;
    o["advancedIntrinsics"] = ops::hasExplicitIntrinsics(in);
    o["fxPx"] = finiteOr(in.fx(), 0.0);   // derived while unset, so switching the camera matrix on starts from the same view
    o["fyPx"] = finiteOr(in.fy(), 0.0);
    o["cxPx"] = finiteOr(in.cx(), 0.0);
    o["cyPx"] = finiteOr(in.cy(), 0.0);
    o["frameRateHz"] = cam.frameRateHz;
    o["latencyMs"] = static_cast<double>(cam.latencyUs) / 1000.0;
    o["captureLatencyMs"] = static_cast<double>(cam.captureLatencyUs) / 1000.0;

    o["tagSizeM"] = cam.tagSizeM;
    o["minTagPixels"] = cam.minTagPixels;
    o["maxViewAngleRad"] = cam.maxViewAngleRad;
    o["occlusion"] = cam.occlusion;
    o["multiTag"] = cam.multiTag;

    o["cornerNoisePx"] = cam.realism.cornerNoisePx;
    o["latencyJitterMs"] = static_cast<double>(cam.realism.latencyJitterUs) / 1000.0;
    o["dropoutPercent"] = cam.realism.dropoutProbability * 100.0;

    o["assumedOverride"] = cam.assumedPose.has_value();
    o["assumedPose"] = sdk::toValue(cam.assumedPose.value_or(ops::cameraInChassis(c, catalog, cam)));
    return o;
}

bool applyCamera(sdk::RobotConfig& c, const EditorCatalog& catalog, std::string_view id, const std::string& key, const Value& v) {
    bool ok = false;
    if (key == "latencyMs") {
        const std::int64_t us = microsFromMillis(v, ok);
        return ok && ops::setCameraProperty(c, catalog, id, "latencyUs", Value(us));
    }
    if (key == "captureLatencyMs") {
        const std::int64_t us = microsFromMillis(v, ok);
        return ok && ops::setCameraProperty(c, catalog, id, "captureLatencyUs", Value(us));
    }
    if (key == "latencyJitterMs") {
        const std::int64_t us = microsFromMillis(v, ok);
        return ok && ops::setCameraProperty(c, catalog, id, "latencyJitterUs", Value(us));
    }
    if (key == "dropoutPercent") {
        if (!v.isNumber() || !std::isfinite(v.asDouble())) return false;
        return ops::setCameraProperty(c, catalog, id, "dropoutProbability", Value(v.asDouble() / 100.0));
    }
    if (key == "advancedIntrinsics") return v.isBool() && ops::setCameraExplicitIntrinsics(c, id, v.asBool());
    return ops::setCameraProperty(c, catalog, id, key, v);
}

// ---- project-level settings -------------------------------------------------------------------
PropertySchema visionSchema(const EditorCatalog& catalog) {
    PropertySchema s;
    const QString layoutGroup = ETR("AprilTag layout");
    PropertySpec layout = makeSpec("layoutId", ETR("Layout"), PropertyType::Enum, PropertyUnit::None, std::string(), layoutGroup,
                                   ETR("Where the AprilTags are on the field. The cameras detect tags of this layout."));
    const sdk::FiducialLayout* first = catalog.fiducialLayouts.empty() ? nullptr : &catalog.fiducialLayouts.front();
    layout.options.push_back({"", first == nullptr ? ETR("Game plugin default").toStdString()
                                                   : ETR("Game plugin default (%1)").arg(QString::fromStdString(first->displayName.empty() ? first->id : first->displayName)).toStdString()});
    for (const auto& l : catalog.fiducialLayouts) layout.options.push_back({l.id, l.displayName.empty() ? l.id : l.displayName});
    s.properties.push_back(std::move(layout));
    s.properties.push_back(makeSpec("layoutFile", ETR("Custom layout file"), PropertyType::String, PropertyUnit::None, std::string(), layoutGroup,
                                    ETR("A WPILib AprilTagFieldLayout JSON file for a custom or off-season field. It replaces the layout above. "
                                        "A relative path is resolved against the robot project folder.")));

    s.properties.push_back(makeSpec("photonMessageUuid", ETR("Message UUID override"), PropertyType::String, PropertyUnit::None, std::string(),
                                    ETR("PhotonLib (advanced)"),
                                    ETR("Forces the PhotonVision message UUID the published results carry (32 hexadecimal digits). Leave empty: it is "
                                        "derived from the PhotonLib version of the robot project. The emulator publishes the format of PhotonLib "
                                        "2025.3 to 2026.3.4, UUID %1.")
                                        .arg(QString::fromLatin1(sdk::kPhotonMessageUuidV2025_3))));
    return s;
}

Value visionValues(const sdk::VisionConfig& vision) {
    Value o = Value::makeObject();
    o["layoutId"] = vision.layoutId;
    o["layoutFile"] = vision.layoutFile;
    o["photonMessageUuid"] = vision.photonMessageUuid;
    return o;
}

bool applyVision(sdk::RobotConfig& c, const std::string& key, const Value& v) { return ops::setVisionProperty(c, key, v); }

bool isPhotonMessageUuid(std::string_view uuid) {
    return uuid.empty() ||
           (uuid.size() == 32 && std::all_of(uuid.begin(), uuid.end(), [](char ch) { return std::isxdigit(static_cast<unsigned char>(ch)) != 0; }));
}

}  // namespace wpiemu::app::editor::spec
