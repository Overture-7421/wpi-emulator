#include "model/slot_status.h"
#include "wpiemu/sdk/mechanisms/motion_config.h"

#include <algorithm>
#include <cstdlib>
#include <map>

#include "model/config_ops.h"
#include "model/motion_links.h"
#include "model/tr.h"
#include "model/vision_specs.h"

namespace wpiemu::app::editor {

const char* slotTypeName(sdk::SlotType type) {
    switch (type) {
        case sdk::SlotType::Motor: return "motor";
        case sdk::SlotType::PositionSensor: return "position sensor";
        case sdk::SlotType::VelocitySensor: return "velocity sensor";
        case sdk::SlotType::LimitSwitch: return "limit switch";
        case sdk::SlotType::BeamBreak: return "beam break";
        case sdk::SlotType::Imu: return "IMU";
        case sdk::SlotType::AnalogSensor: return "analog sensor";
        case sdk::SlotType::Solenoid: return "solenoid";
        case sdk::SlotType::DigitalOutput: return "digital output";
    }
    return "device";
}

QString slotTypeLabel(sdk::SlotType type) {
    switch (type) {
        case sdk::SlotType::Motor: return ETR("Motor controller");
        case sdk::SlotType::PositionSensor: return ETR("Position sensor");
        case sdk::SlotType::VelocitySensor: return ETR("Velocity sensor");
        case sdk::SlotType::LimitSwitch: return ETR("Limit switch");
        case sdk::SlotType::BeamBreak: return ETR("Beam break");
        case sdk::SlotType::Imu: return ETR("IMU");
        case sdk::SlotType::AnalogSensor: return ETR("Analog sensor");
        case sdk::SlotType::Solenoid: return ETR("Solenoid");
        case sdk::SlotType::DigitalOutput: return ETR("Digital output");
    }
    return ETR("Device");
}

QString mappedLabel(const SlotInfo& info) {
    if (!info.mapped) return {};
    return info.reversed ? ETR("%1 (reversed)").arg(deviceLabel(*info.mapped)) : deviceLabel(*info.mapped);
}

std::vector<SlotInfo> slotInfos(const sdk::RobotConfig& c, const EditorCatalog& catalog, const DeviceDirectory& devices,
                                std::string_view owner) {
    std::vector<SlotInfo> out;
    const auto conflicts = sdk::findAliasConflicts(c.allMappings());
    const MotionIndex motion(c, catalog);
    for (const sdk::SlotSpec& spec : ops::slotSpecsFor(c, catalog, owner)) {
        SlotInfo info;
        info.address = {std::string(owner), spec.name};
        info.spec = spec;
        const sdk::SlotMapping* mapping = ops::findMapping(c, info.address);
        if (motion.isFollowerMotor(info.address)) continue;
        if (mapping == nullptr) {
            info.state = spec.required ? SlotState::Unmapped : SlotState::OptionalEmpty;
            info.detail = spec.required ? ETR("Required: drop a %1 here or pick one from the list.").arg(slotTypeLabel(spec.type).toLower())
                          : spec.type == sdk::SlotType::Motor && spec.ordinal > 1
                              ? ETR("Not mapped: this motor mirrors motor 1 (same direction). Map its controller to simulate it separately.")
                              : ETR("Optional: unmapped slots read neutral values.");
            out.push_back(std::move(info));
            continue;
        }
        info.mapped = mapping->device;
        info.reversed = mapping->reversed && spec.type == sdk::SlotType::Motor;
        info.state = SlotState::Mapped;
        info.detail = ETR("Mapped to %1.").arg(mappedLabel(info));
        const sdk::DiscoveredDevice* dev = devices.find(mapping->device);
        const bool inConflict = std::any_of(conflicts.begin(), conflicts.end(), [&](const sdk::AliasConflict& cf) {
            return cf.physical == mapping->device.physical();
        });
        if (inConflict || (dev != nullptr && dev->aliasState == sdk::AliasState::Conflict)) {
            info.state = SlotState::AliasConflict;
            info.detail = ETR("This physical device is mapped through different addresses. The simulator cannot tell the buses apart.");
        } else if (dev != nullptr && !sdk::isCompatible(spec, dev->descriptor)) {
            info.state = SlotState::Incompatible;
            info.detail = ETR("%1 cannot act as a %2.").arg(deviceLabel(dev->descriptor), slotTypeLabel(spec.type).toLower());
        } else if (!devices.empty() && (dev == nullptr || !dev->live)) {
            info.state = SlotState::NotDetected;
            info.detail = ETR("%1 was not detected in the last discovery. Run \"Detect devices\" to refresh.").arg(deviceLabel(mapping->device));
        }
        out.push_back(std::move(info));
    }
    return out;
}

std::optional<sdk::SlotAddress> bestSlotFor(const sdk::RobotConfig& c, const EditorCatalog& catalog, std::string_view owner,
                                            sdk::CapabilitySet caps, std::string_view slotPrefix) {
    std::optional<sdk::SlotAddress> optionalFree;
    const auto axes = sdk::motionAxisInfos(c, [&](std::string_view type) { return catalog.findTemplate(type); });
    for (const sdk::SlotSpec& spec : ops::slotSpecsFor(c, catalog, owner)) {
        if (!slotPrefix.empty() && spec.name.rfind(slotPrefix, 0) != 0) continue;
        if (sdk::motionFollowerMotor(c, axes, {std::string(owner), spec.name})) continue;
        if (!slotAccepts(spec, caps)) continue;
        const sdk::SlotAddress address{std::string(owner), spec.name};
        if (ops::findMapping(c, address) != nullptr) continue;
        if (spec.required) return address;
        if (!optionalFree) optionalFree = address;
    }
    return optionalFree;
}

std::optional<ops::MotorGroup> growableMotorGroup(const sdk::RobotConfig& c, const EditorCatalog& catalog, std::string_view owner,
                                                  sdk::CapabilitySet caps, std::string_view slotPrefix) {
    if (!caps.containsAll(sdk::requiredCapabilities(sdk::SlotType::Motor))) return std::nullopt;
    const MotionIndex motion(c, catalog);
    for (auto& group : ops::motorGroups(c, catalog, owner)) {
        if (group.slotNames.empty() || group.count >= group.maxCount) continue;
        if (!slotPrefix.empty() && group.slotNames.front().rfind(slotPrefix, 0) != 0) continue;
        if (motion.isFollowerMotor({std::string(owner), group.slotNames.front()})) continue;
        return group;
    }
    return std::nullopt;
}

bool canAcceptDevice(const sdk::RobotConfig& c, const EditorCatalog& catalog, std::string_view owner, sdk::CapabilitySet caps,
                     std::string_view slotPrefix) {
    return bestSlotFor(c, catalog, owner, caps, slotPrefix).has_value() ||
           growableMotorGroup(c, catalog, owner, caps, slotPrefix).has_value();
}

DeviceStatus deviceStatus(const sdk::RobotConfig& c, const sdk::DiscoveredDevice& device) {
    DeviceStatus st;
    st.usedBy = ops::slotsUsing(c, device.descriptor.physicalId());
    st.alias = device.aliasState;
    for (const auto& cf : sdk::findAliasConflicts(c.allMappings()))
        if (cf.physical == device.descriptor.physicalId()) st.alias = sdk::AliasState::Conflict;
    return st;
}

std::vector<sdk::Issue> localIssues(const sdk::RobotConfig& c, const EditorCatalog& catalog, const DeviceDirectory& devices) {
    std::vector<sdk::Issue> out;
    const auto axes =
        sdk::motionAxisInfos(c, [&](std::string_view type) { return catalog.findTemplate(type); });
    for (auto& issue : sdk::motionLinkIssues(c, axes)) out.push_back(std::move(issue));
    for (const std::string& owner : ops::slotOwners(c)) {
        for (const SlotInfo& info : slotInfos(c, catalog, devices, owner)) {
            const std::string path = info.address.toString();
            const QString where = QString::fromStdString(ops::slotDisplayName(c, catalog, info.address));
            switch (info.state) {
                case SlotState::Unmapped:
                    out.push_back({sdk::Severity::Warning, path,
                                   ETR("%1 needs a %2. Unmapped required slots read neutral values.").arg(where, slotTypeLabel(info.spec.type).toLower()).toStdString(),
                                   "slot_unfilled"});
                    break;
                case SlotState::Incompatible:
                    out.push_back({sdk::Severity::Error, path, (where + ": " + info.detail).toStdString(), "slot_incompatible"});
                    break;
                case SlotState::NotDetected:
                    out.push_back({sdk::Severity::Info, path, (where + ": " + info.detail).toStdString(), "device_not_detected"});
                    break;
                default: break;
            }
        }
    }
    for (auto& issue : sdk::validateDeviceAliases(c.allMappings())) out.push_back(std::move(issue));

    if (!spec::isPhotonMessageUuid(c.vision.photonMessageUuid))
        out.push_back({sdk::Severity::Warning, "vision.photonMessageUuid",
                       ETR("The PhotonLib message UUID must be 32 hexadecimal digits. Until it is fixed or cleared, the PhotonVision cameras "
                           "are not published.").toStdString(),
                       "photon_uuid_malformed"});

    for (const sdk::DiscoveredDevice& d : devices.all()) {
        if (!d.live || !sdk::isMappable(d.descriptor)) continue;
        const DeviceStatus st = deviceStatus(c, d);
        if (!st.mapped() || st.alias == sdk::AliasState::Conflict) {
            if (st.alias == sdk::AliasState::Conflict && st.mapped()) continue;   // reported as a conflict above
            out.push_back({sdk::Severity::Warning, "device:" + d.descriptor.physicalId().toString(),
                           ETR("%1 was detected but is not mapped to any slot, so it runs in loopback (outputs are echoed back).")
                               .arg(deviceLabel(d.descriptor)).toStdString(),
                           "unmapped_loopback"});
        }
    }

    std::map<std::pair<sdk::PhysicalDeviceId, int>, std::vector<sdk::SlotAddress>> byDeviceAndType;
    for (const auto& owner : ops::slotOwners(c))
        for (const auto& info : slotInfos(c, catalog, devices, owner))
            if (info.mapped) byDeviceAndType[{info.mapped->physical(), static_cast<int>(info.spec.type)}].push_back(info.address);
    for (const auto& [key, slots] : byDeviceAndType) {
        if (slots.size() < 2) continue;
        QString names;
        for (const auto& s : slots) names += (names.isEmpty() ? "" : ", ") + QString::fromStdString(ops::slotDisplayName(c, catalog, s));
        out.push_back({sdk::Severity::Warning, slots.front().toString(),
                       ETR("The same device drives several slots of the same kind: %1.").arg(names).toStdString(), "device_multi_mapped"});
    }
    return out;
}

std::vector<sdk::Issue> mergeIssues(const std::vector<std::vector<sdk::Issue>>& lists) {
    std::vector<sdk::Issue> out;
    for (const auto& list : lists)
        for (const auto& issue : list) {
            const bool dup = std::any_of(out.begin(), out.end(), [&](const sdk::Issue& o) {
                return o.severity == issue.severity && o.path == issue.path && o.code == issue.code &&
                       (!issue.code.empty() || o.message == issue.message);
            });
            if (!dup) out.push_back(issue);
        }
    return out;
}

namespace {

// "mechanisms[3]" -> 3, and the remainder after the closing bracket (without a leading dot).
bool parseIndexed(const std::string& path, const std::string& name, int& index, std::string& rest) {
    if (path.rfind(name + "[", 0) != 0) return false;
    const auto close = path.find(']');
    if (close == std::string::npos) return false;
    index = std::atoi(path.substr(name.size() + 1, close - name.size() - 1).c_str());
    rest = path.substr(close + 1);
    if (!rest.empty() && rest.front() == '.') rest.erase(0, 1);
    return true;
}

std::string firstSegment(const std::string& s) {
    const auto dot = s.find('.');
    return dot == std::string::npos ? s : s.substr(0, dot);
}

// The text between the first pair of single quotes ("camera 'photon_front' (photon-front): ..." -> photon_front).
std::string firstQuoted(const std::string& s) {
    const auto open = s.find('\'');
    if (open == std::string::npos) return {};
    const auto close = s.find('\'', open + 1);
    return close == std::string::npos ? std::string() : s.substr(open + 1, close - open - 1);
}

// The inspector field of a camera for the remainder of an issue path ("intrinsics.hfovRad", "realism.latencyJitterUs", ...). The
// form shows latency and dropout in friendlier units, so those fields have other keys than the persisted ones.
std::string cameraFieldKey(const std::string& rest) {
    const auto after = [&](const char* prefix) { return rest.rfind(prefix, 0) == 0 ? rest.substr(std::string(prefix).size()) : std::string(); };
    std::string key;
    if (rest.rfind("attachment", 0) == 0) return "parent";
    if (rest.rfind("local", 0) == 0) return "local";
    if (rest.rfind("assumedPose", 0) == 0) return "assumedPose";
    if (!(key = firstSegment(after("intrinsics."))).empty()) return key;
    if (!(key = firstSegment(after("realism."))).empty()) {
        if (key == "latencyJitterUs") return "latencyJitterMs";
        if (key == "dropoutProbability") return "dropoutPercent";
        return key;
    }
    key = firstSegment(rest);
    if (key == "latencyUs") return "latencyMs";
    if (key == "captureLatencyUs") return "captureLatencyMs";
    return key;
}

}  // namespace

IssueLocation locateIssue(const sdk::RobotConfig& c, const EditorCatalog& catalog, const sdk::Issue& issue) {
    IssueLocation loc;
    const std::string& path = issue.path;
    int index = 0;
    std::string rest;

    // The vision subsystem's runtime warnings carry no path: the code says where the fix is, and the message names the camera.
    if (path.empty()) {
        if (issue.code == sdk::validation_code::kVisionLayoutMissing) {
            loc.selection = SelectionRef::vision();
            loc.fieldKey = c.vision.layoutFile.empty() ? "layoutId" : "layoutFile";
            return loc;
        }
        if (issue.code == sdk::validation_code::kVisionCameraStatus || issue.code == sdk::validation_code::kVisionCameraUnconfigured) {
            loc.selection = SelectionRef::vision();
            const std::string name = firstQuoted(issue.message);   // the camera id (status) or its NetworkTables name (unconfigured)
            for (const sdk::CameraConfig& cam : c.vision.cameras)
                if (cam.id == name || (issue.code == sdk::validation_code::kVisionCameraUnconfigured && cam.networkName() == name))
                    loc.selection = SelectionRef::camera(cam.id);
            return loc;
        }
    }

    if (path.rfind("motionLinks", 0) == 0) {
        const std::string id = path.size() > 12 ? path.substr(12) : std::string();
        for (const auto& link : c.motionLinks)
            if (link.id == id) {
                loc.selection = MotionIndex::ownerOf(link.target);
                loc.fieldKey = "drivenBy";
                return loc;
            }
        loc.selection = SelectionRef::robot();
        loc.fieldKey = "drivenBy";
        return loc;
    }

    if (path.rfind("device:", 0) == 0) {
        loc.selection = SelectionRef::device(path.substr(7));
        return loc;
    }
    if (parseIndexed(path, "mechanisms", index, rest)) {
        if (index >= 0 && index < static_cast<int>(c.mechanisms.size())) {
            loc.selection = SelectionRef::mechanism(c.mechanisms[static_cast<std::size_t>(index)].id);
            if (rest.rfind("properties.", 0) == 0) loc.fieldKey = firstSegment(rest.substr(11));
            else if (rest.rfind("attachment", 0) == 0) loc.fieldKey = "parent";
            else if (rest.rfind("mount", 0) == 0) loc.fieldKey = "mount";
            else if (rest.rfind("id", 0) == 0) loc.fieldKey = "id";
        }
        return loc;
    }
    if (parseIndexed(path, "zones", index, rest)) {
        if (index >= 0 && index < static_cast<int>(c.zones.size())) {
            loc.selection = SelectionRef::zone(c.zones[static_cast<std::size_t>(index)].id);
            if (rest.rfind("properties.", 0) == 0) loc.fieldKey = firstSegment(rest.substr(11));
            else if (rest.rfind("attachment", 0) == 0) loc.fieldKey = "parent";
            else if (rest.rfind("local", 0) == 0) loc.fieldKey = "local";
            else if (rest.rfind("sizeM", 0) == 0) loc.fieldKey = "sizeM";
            else if (rest.rfind("kind", 0) == 0) loc.fieldKey = "kind";
            else if (rest.rfind("id", 0) == 0) loc.fieldKey = "id";
            else if (rest.rfind("activation.conditions[", 0) == 0) loc.fieldKey = "activation:" + std::to_string(std::atoi(rest.c_str() + 22));
        }
        return loc;
    }
    if (path.rfind("vision.", 0) == 0) {   // vision.cameras[i].<field>, vision.layoutId, ...
        if (parseIndexed(path.substr(7), "cameras", index, rest)) {
            if (index >= 0 && index < static_cast<int>(c.vision.cameras.size())) {
                loc.selection = SelectionRef::camera(c.vision.cameras[static_cast<std::size_t>(index)].id);
                loc.fieldKey = cameraFieldKey(rest);
            }
            return loc;
        }
        loc.selection = SelectionRef::vision();
        loc.fieldKey = firstSegment(path.substr(7));
        return loc;
    }
    if (path == "chassis" || path.rfind("chassis.", 0) == 0) {
        loc.selection = SelectionRef::chassis();
        loc.fieldKey = path.size() > 8 ? firstSegment(path.substr(8)) : std::string();
        return loc;
    }
    if (path == "sim.startPose") {   // the Robot page carries the Start pose card
        loc.selection = SelectionRef::robot();
        loc.fieldKey = "startX";
        return loc;
    }
    if (path == "drivetrain" || path.rfind("drivetrain", 0) == 0) {
        // "drivetrain.fl.drive" == slot address; "drivetrain" / "drivetrain.spec..." == the drivetrain.
        const auto dot = path.find('.');
        if (dot != std::string::npos) {
            const std::string slot = path.substr(dot + 1);
            for (const auto& spec : ops::drivetrainSlotSpecs(c.drivetrain.spec))
                if (spec.name == slot) {
                    loc.selection = SelectionRef::slot(sdk::slot_owner::kDrivetrain, slot);
                    return loc;
                }
        }
        loc.selection = SelectionRef::drivetrain();
        return loc;
    }
    if (path.rfind("plugins", 0) == 0) {
        loc.selection = SelectionRef::plugin();
        const auto dataPos = path.find("data.");
        if (dataPos != std::string::npos) loc.fieldKey = firstSegment(path.substr(dataPos + 5));
        return loc;
    }
    // "owner.slot": owner is a mechanism id, "robot" or "drivetrain".
    const auto dot = path.find('.');
    if (dot != std::string::npos) {
        const std::string owner = path.substr(0, dot);
        const std::string slot = path.substr(dot + 1);
        for (const auto& known : ops::slotOwners(c))
            if (known == owner)
                for (const auto& spec : ops::slotSpecsFor(c, catalog, owner))
                    if (spec.name == slot) {
                        loc.selection = SelectionRef::slot(owner, slot);
                        return loc;
                    }
    }
    if (c.findMechanism(path) != nullptr) loc.selection = SelectionRef::mechanism(path);
    return loc;
}

}  // namespace wpiemu::app::editor
