#pragma once

// The rules for the start-position, zone, camera and sensor sections of a RobotConfig, in one place for
// core's ConfigValidator (the authority) and the editor's local validation (docs/reference/runtime.md).
// Each function appends its Issues to `out` in a fixed order; codes are sdk::validation_code::k{StartPosition,
// Zone,Camera,Sensor}* (config/validation_codes.h). What the rules need from the selected game plugin and the
// registered mechanism templates comes through IConfigCatalog, so neither core nor a plugin is required here.

#include <cmath>
#include <cstddef>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "wpiemu/sdk/config/plugin_config.h"
#include "wpiemu/sdk/config/property_schema.h"
#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/config/sensor.h"
#include "wpiemu/sdk/config/validation_codes.h"
#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/severity.h"
#include "wpiemu/sdk/devices/slot.h"
#include "wpiemu/sdk/game/game_value.h"
#include "wpiemu/sdk/game/zone_kind.h"
#include "wpiemu/sdk/mechanisms/mechanism.h"

namespace wpiemu::sdk {

class IConfigCatalog {
    WPIEMU_INTERFACE(IConfigCatalog)
public:
    virtual const MechanismTemplateInfo* mechanismTemplate(std::string_view type) const = 0;
    virtual std::string_view gamePluginId() const = 0;  // empty = no game plugin selected
    virtual const ZoneKindInfo* zoneKind(std::string_view kind) const = 0;
    virtual const GameValueInfo* gameValue(std::string_view id) const = 0;
};

// Attachment of a zone / camera: the chassis ('root') or an existing link of an existing mechanism.
inline void validateAttachment(const RobotConfig& config, const IConfigCatalog& catalog, const MechanismAttachment& att,
                               const std::string& path, std::vector<Issue>& out) {
    if (att.parentLink.empty()) {
        out.push_back({Severity::Error, path, "parent link must not be empty (use 'root')", validation_code::kAttachmentLinkMissing});
    } else if (att.onChassis()) {
        if (att.parentLink != kRootLink)
            out.push_back({Severity::Error, path, "the chassis only exposes the link 'root', not '" + att.parentLink + "'",
                           validation_code::kAttachmentLinkMissing});
    } else if (const MechanismConfig* parent = config.findMechanism(att.parentMechanismId); parent == nullptr) {
        out.push_back({Severity::Error, path, "parent mechanism '" + att.parentMechanismId + "' does not exist",
                       validation_code::kAttachmentParentMissing});
    } else if (const MechanismTemplateInfo* info = catalog.mechanismTemplate(parent->type);
               info != nullptr && !hasLink(*info, att.parentLink)) {
        out.push_back({Severity::Error, path, "mechanism '" + parent->id + "' (" + parent->type + ") has no link '" + att.parentLink + "'",
                       validation_code::kAttachmentLinkMissing});
    }
}

inline void validateStartPositions(const RobotConfig& config, std::vector<Issue>& out) {
    std::set<std::string> ids;
    for (std::size_t i = 0; i < config.startPositions.size(); ++i) {
        const StartPosition& p = config.startPositions[i];
        const std::string base = "startPositions[" + std::to_string(i) + "]";
        if (p.id.empty())
            out.push_back({Severity::Error, base + ".id", "start position id must not be empty", validation_code::kStartPositionIdInvalid});
        else if (!ids.insert(p.id).second)
            out.push_back({Severity::Error, base + ".id", "duplicate start position id '" + p.id + "'",
                           validation_code::kStartPositionIdDuplicate});
        if (!std::isfinite(p.pose.xM) || !std::isfinite(p.pose.yM) || !std::isfinite(p.pose.headingRad))
            out.push_back({Severity::Error, base + ".pose", "start position must have finite x, y and heading",
                           validation_code::kStartPositionInvalid});
    }
}

inline void validateZones(const RobotConfig& config, const IConfigCatalog& catalog, std::vector<Issue>& out) {
    if (config.zones.empty()) return;
    const std::string pluginId(catalog.gamePluginId());
    std::set<std::string> ids;
    for (std::size_t i = 0; i < config.zones.size(); ++i) {
        const ZoneConfig& z = config.zones[i];
        const std::string base = "zones[" + std::to_string(i) + "]";

        if (z.id.empty())
            out.push_back({Severity::Error, base + ".id", "zone id must not be empty", validation_code::kZoneIdInvalid});
        else if (config.findMechanism(z.id) != nullptr)
            out.push_back({Severity::Error, base + ".id", "zone id '" + z.id + "' is also a mechanism id", validation_code::kZoneIdDuplicate});
        else if (config.findCamera(z.id) != nullptr)
            out.push_back({Severity::Error, base + ".id", "zone id '" + z.id + "' is also a camera id", validation_code::kZoneIdDuplicate});
        else if (!ids.insert(z.id).second)
            out.push_back({Severity::Error, base + ".id", "duplicate zone id '" + z.id + "'", validation_code::kZoneIdDuplicate});

        validateAttachment(config, catalog, z.attachment, base + ".attachment", out);

        // ---- kind and properties ----
        const ZoneKindInfo* info = nullptr;
        if (!ownsPluginId(pluginId, z.kind)) {
            out.push_back({Severity::Error, base + ".kind",
                           pluginId.empty() ? "zone kind '" + z.kind + "' needs a game plugin (none is selected)"
                                            : "zone kind '" + z.kind + "' is not owned by plugin '" + pluginId + "'",
                           validation_code::kZoneKindNotOwned});
        } else {
            info = catalog.zoneKind(z.kind);
            if (info == nullptr)
                out.push_back({Severity::Warning, base + ".kind", "plugin '" + pluginId + "' does not declare zone kind '" + z.kind + "'",
                               validation_code::kZoneKindUnknown});
        }
        if (info != nullptr)
            for (Issue& issue : validateProperties(info->properties, z.properties, base + ".properties")) {
                if (issue.code.empty()) issue.code = validation_code::kZonePropertyInvalid;
                out.push_back(std::move(issue));
            }

        // ---- geometry ----
        if (!finite(z.local))
            out.push_back({Severity::Error, base + ".local", "zone pose must be finite", validation_code::kZoneGeometryInvalid});
        const bool volume = info == nullptr || info->shape == ZoneShape::Volume;
        if (volume && (!finite(z.sizeM) || z.sizeM.x <= 0.f || z.sizeM.y <= 0.f || z.sizeM.z <= 0.f))
            out.push_back({Severity::Error, base + ".sizeM", "zone size must be positive and finite", validation_code::kZoneGeometryInvalid});

        // ---- activation ----
        for (std::size_t j = 0; j < z.activation.conditions.size(); ++j) {
            const ZoneCondition& cond = z.activation.conditions[j];
            const std::string cp = base + ".activation.conditions[" + std::to_string(j) + "]";
            if (cond.value.empty())
                out.push_back({Severity::Error, cp + ".value", "condition value name must not be empty", validation_code::kZoneConditionInvalid});
            if (!std::isfinite(cond.threshold) || !std::isfinite(cond.tolerance) || cond.tolerance < 0.0)
                out.push_back({Severity::Error, cp, "condition threshold must be finite and tolerance >= 0",
                               validation_code::kZoneConditionInvalid});
            if (config.findMechanism(cond.mechanismId) == nullptr)
                out.push_back({Severity::Warning, cp + ".mechanism", "condition mechanism '" + cond.mechanismId + "' does not exist",
                               validation_code::kZoneConditionMechanismUnknown});
        }
    }
}

inline void validateCameras(const RobotConfig& config, const IConfigCatalog& catalog, std::vector<Issue>& out) {
    std::set<std::string> ids;
    std::set<std::pair<int, std::string>> names;
    const auto inRange = [](double v, double lo, double hi) { return std::isfinite(v) && v > lo && v <= hi; };
    for (std::size_t i = 0; i < config.cameras.size(); ++i) {
        const CameraConfig& cam = config.cameras[i];
        const std::string base = "cameras[" + std::to_string(i) + "]";

        if (cam.id.empty())
            out.push_back({Severity::Error, base + ".id", "camera id must not be empty", validation_code::kCameraIdInvalid});
        else if (config.findMechanism(cam.id) != nullptr)
            out.push_back({Severity::Error, base + ".id", "camera id '" + cam.id + "' is also a mechanism id", validation_code::kCameraIdDuplicate});
        else if (config.findZone(cam.id) != nullptr)
            out.push_back({Severity::Error, base + ".id", "camera id '" + cam.id + "' is also a zone id", validation_code::kCameraIdDuplicate});
        else if (!ids.insert(cam.id).second)
            out.push_back({Severity::Error, base + ".id", "duplicate camera id '" + cam.id + "'", validation_code::kCameraIdDuplicate});

        if (!names.emplace(static_cast<int>(cam.kind), cam.name).second)
            out.push_back({Severity::Error, base + ".name",
                           std::string("another ") + cameraKindKey(cam.kind) + " camera is already named '" + cam.name + "'",
                           validation_code::kCameraNameDuplicate});

        validateAttachment(config, catalog, cam.attachment, base + ".attachment", out);

        if (!finite(cam.local))
            out.push_back({Severity::Error, base + ".pose", "camera pose must be finite", validation_code::kCameraGeometryInvalid});

        const auto optics = [&](const char* field, bool ok, const char* why) {
            if (!ok) out.push_back({Severity::Error, base + "." + field, why, validation_code::kCameraOpticsInvalid});
        };
        optics("hFovDeg", std::isfinite(cam.hFovDeg) && cam.hFovDeg > 0.0 && cam.hFovDeg < 180.0, "horizontal FOV must be in (0, 180) degrees");
        optics("vFovDeg", std::isfinite(cam.vFovDeg) && cam.vFovDeg > 0.0 && cam.vFovDeg < 180.0, "vertical FOV must be in (0, 180) degrees");
        optics("maxRangeM", std::isfinite(cam.maxRangeM) && cam.maxRangeM > 0.0, "maximum range must be positive");
        optics("maxIncidenceDeg", inRange(cam.maxIncidenceDeg, 0.0, 90.0), "maximum incidence angle must be in (0, 90] degrees");
        optics("fps", inRange(cam.fps, 0.0, 240.0), "frame rate must be in (0, 240]");
        optics("latencyMs", std::isfinite(cam.latencyMs) && cam.latencyMs >= 0.0 && cam.latencyMs <= 2000.0, "latency must be in [0, 2000] ms");
        optics("widthPx", cam.widthPx > 0, "image width must be positive");
        optics("heightPx", cam.heightPx > 0, "image height must be positive");
        optics("positionNoiseM", std::isfinite(cam.positionNoiseM) && cam.positionNoiseM >= 0.0, "position noise must be >= 0");
        optics("yawNoiseDeg", std::isfinite(cam.yawNoiseDeg) && cam.yawNoiseDeg >= 0.0, "yaw noise must be >= 0");
    }
}

inline void validateSensors(const RobotConfig& config, const IConfigCatalog& catalog, std::vector<Issue>& out) {
    const std::string pluginId(catalog.gamePluginId());
    std::set<std::string> ids;
    for (std::size_t i = 0; i < config.sensors.size(); ++i) {
        const SensorConfig& s = config.sensors[i];
        const std::string base = "sensors[" + std::to_string(i) + "]";

        if (s.id.empty())
            out.push_back({Severity::Error, base + ".id", "sensor id must not be empty", validation_code::kSensorIdInvalid});
        else if (s.id == slot_owner::kDrivetrain || s.id == slot_owner::kRobot)
            out.push_back({Severity::Error, base + ".id", "'" + s.id + "' is a reserved slot owner", validation_code::kSensorIdInvalid});
        else if (s.id.find('.') != std::string::npos)
            out.push_back({Severity::Error, base + ".id", "sensor id must not contain '.'", validation_code::kSensorIdInvalid});
        else if (config.findMechanism(s.id) != nullptr)
            out.push_back({Severity::Error, base + ".id", "sensor id '" + s.id + "' is also a mechanism id", validation_code::kSensorIdDuplicate});
        else if (config.findZone(s.id) != nullptr)
            out.push_back({Severity::Error, base + ".id", "sensor id '" + s.id + "' is also a zone id", validation_code::kSensorIdDuplicate});
        else if (config.findCamera(s.id) != nullptr)
            out.push_back({Severity::Error, base + ".id", "sensor id '" + s.id + "' is also a camera id", validation_code::kSensorIdDuplicate});
        else if (!ids.insert(s.id).second)
            out.push_back({Severity::Error, base + ".id", "duplicate sensor id '" + s.id + "'", validation_code::kSensorIdDuplicate});

        if (s.source.empty()) {
            out.push_back({Severity::Warning, base + ".source", "sensor '" + s.id + "' has no game value selected; it will read as idle",
                           validation_code::kSensorSourceMissing});
        } else if (!ownsPluginId(pluginId, s.source)) {
            out.push_back({Severity::Error, base + ".source",
                           pluginId.empty() ? "sensor source '" + s.source + "' needs a game plugin (none is selected)"
                                            : "sensor source '" + s.source + "' is not owned by plugin '" + pluginId + "'",
                           validation_code::kSensorSourceNotOwned});
        } else if (catalog.gameValue(s.source) == nullptr) {
            out.push_back({Severity::Warning, base + ".source", "plugin '" + pluginId + "' does not declare game value '" + s.source + "'",
                           validation_code::kSensorSourceUnknown});
        }

        const PropertySchema schema = sensorResponseSchema(s.kind);
        for (Issue& issue : validateProperties(schema, s.properties, base + ".properties")) {
            if (issue.code.empty()) issue.code = validation_code::kSensorPropertyInvalid;
            out.push_back(std::move(issue));
        }
        if (s.kind == SensorKind::Distance && s.properties.isObject()) {
            const Value* mode = s.properties.find(sensor_prop::kMode);
            const Value* lo = s.properties.find(sensor_prop::kInputMin);
            const Value* hi = s.properties.find(sensor_prop::kInputMax);
            if (mode != nullptr && mode->isString() && mode->asString() == sensor_prop::kModeLinear && lo != nullptr && hi != nullptr &&
                lo->isNumber() && hi->isNumber() && lo->asDouble() == hi->asDouble())
                out.push_back({Severity::Error, base + ".properties." + sensor_prop::kInputMax, "linear input range must not be empty",
                               validation_code::kSensorPropertyInvalid});
        }
    }
}

}  // namespace wpiemu::sdk
