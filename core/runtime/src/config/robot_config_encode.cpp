#include <string>

#include "robot_config_codec_io.h"
#include "wpiemu/core/config/robot_config_codec.h"

namespace wpiemu::core::config {

using namespace wpiemu::sdk;
using namespace codec_io;

namespace {

Value materialValue(const Material& m) {
    Value o = Value::makeObject();
    o["friction"] = fv(m.friction);
    o["restitution"] = fv(m.restitution);
    o["rollingResistance"] = fv(m.rollingResistance);
    return o;
}

Value motorValue(const MotorSpec& m) {
    Value o = Value::makeObject();
    o["name"] = m.name;
    o["nominalVoltageV"] = m.nominalVoltageV;
    o["stallTorqueNm"] = m.stallTorqueNm;
    o["stallCurrentA"] = m.stallCurrentA;
    o["freeCurrentA"] = m.freeCurrentA;
    o["freeSpeedRadPerSec"] = m.freeSpeedRadPerSec;
    return o;
}

Value tireValue(const TireParams& t) {
    Value o = Value::makeObject();
    o["mu"] = fv(t.mu);
    o["slideRatio"] = fv(t.slideRatio);
    o["peakSlipLong"] = fv(t.peakSlipLong);
    o["peakSlipLat"] = fv(t.peakSlipLat);
    o["vRefMps"] = fv(t.vRefMps);
    o["lateralScale"] = fv(t.lateralScale);
    o["tractionAngleRad"] = fv(t.tractionAngleRad);
    return o;
}

Value primitiveValue(const PrimitiveSpec& p) {
    Value o = Value::makeObject();
    o["name"] = p.name;
    o["kind"] = p.kind == PrimitiveKind::Box ? "box" : "cylinder";
    o["local"] = poseValue(p.local);
    o["sizeM"] = vec3Value(p.sizeM);
    o["radiusM"] = fv(p.radiusM);
    o["heightM"] = fv(p.heightM);
    o["color"] = p.color;
    o["collides"] = p.collides;
    o["massKg"] = fv(p.massKg);
    return o;
}

Value primitivesValue(const std::vector<PrimitiveSpec>& ps) {
    Value a = Value::makeArray();
    for (const auto& p : ps) a.push_back(primitiveValue(p));
    return a;
}

Value mappingsValue(const std::vector<SlotMapping>& ms) {
    Value a = Value::makeArray();
    for (const SlotMapping& m : ms) {
        Value o = Value::makeObject();
        o["owner"] = m.slot.owner;
        o["slot"] = m.slot.slot;
        if (m.reversed) o["reversed"] = true;
        Value d = Value::makeObject();
        d["family"] = m.device.family;
        d["type"] = m.device.type;
        d["id"] = m.device.id;
        d["bus"] = m.device.bus;
        d["channel"] = m.device.channel;
        o["device"] = std::move(d);
        a.push_back(std::move(o));
    }
    return a;
}

Value wheelValue(const WheelSpec& w) {
    Value o = Value::makeObject();
    o["name"] = w.name;
    o["kind"] = wheelKindName(w.kind);
    o["positionM"] = vec3Value(w.positionM);
    o["radiusM"] = fv(w.radiusM);
    o["widthM"] = fv(w.widthM);
    o["inertiaKgM2"] = fv(w.inertiaKgM2);
    o["tire"] = tireValue(w.tire);
    o["steered"] = w.steered;
    o["driveMotor"] = motorValue(w.driveMotor);
    o["driveMotorCount"] = w.driveMotorCount;
    o["driveGearRatio"] = fv(w.driveGearRatio);
    o["driveEfficiency"] = fv(w.driveEfficiency);
    o["driveCurrentLimitA"] = fv(w.driveCurrentLimitA);
    o["steerMotor"] = motorValue(w.steerMotor);
    o["steerGearRatio"] = fv(w.steerGearRatio);
    o["steerInertiaKgM2"] = fv(w.steerInertiaKgM2);
    return o;
}

Value chassisValue(const ChassisSpec& c) {
    Value chassis = Value::makeObject();
    chassis["sizeM"] = vec3Value(c.sizeM);
    chassis["massKg"] = fv(c.massKg);
    chassis["comHeightM"] = fv(c.comHeightM);
    chassis["comOffsetM"] = vec3Value(c.comOffsetM);
    chassis["groundClearanceM"] = fv(c.groundClearanceM);
    chassis["inertiaDiagKgM2"] = vec3Value(c.inertiaDiagKgM2);
    chassis["material"] = materialValue(c.material);
    chassis["primitives"] = primitivesValue(c.primitives);
    return chassis;
}

Value drivetrainValue(const DrivetrainConfig& d) {
    Value dt = Value::makeObject();
    dt["kind"] = d.spec.kind;
    dt["tireModel"] = d.spec.tireModel;
    dt["loadTransferTauS"] = fv(d.spec.loadTransferTauS);
    Value wheels = Value::makeArray();
    for (const WheelSpec& w : d.spec.wheels) wheels.push_back(wheelValue(w));
    dt["wheels"] = std::move(wheels);
    dt["mappings"] = mappingsValue(d.mappings);
    return dt;
}

Value mechanismValue(const MechanismConfig& m) {
    Value o = Value::makeObject();
    o["id"] = m.id;
    o["type"] = m.type;
    o["label"] = m.label;
    Value a = Value::makeObject();
    a["parentMechanismId"] = m.attachment.onChassis() ? std::string(kChassisParent) : m.attachment.parentMechanismId;
    a["parentLink"] = m.attachment.parentLink;
    o["attachment"] = std::move(a);
    o["mount"] = poseValue(m.mount);
    o["geometry"] = primitivesValue(m.geometry);
    o["properties"] = m.properties.isObject() ? m.properties : Value::makeObject();
    o["mappings"] = mappingsValue(m.mappings);
    return o;
}

Value zoneValue(const ZoneConfig& z) {
    Value o = Value::makeObject();
    o["id"] = z.id;
    o["kind"] = z.kind;
    o["label"] = z.label;
    Value a = Value::makeObject();
    a["parentMechanismId"] = z.attachment.onChassis() ? std::string(kChassisParent) : z.attachment.parentMechanismId;
    a["parentLink"] = z.attachment.parentLink;
    o["attachment"] = std::move(a);
    o["pose"] = poseValue(z.local);
    o["sizeM"] = vec3Value(z.sizeM);
    o["properties"] = z.properties.isObject() ? z.properties : Value::makeObject();
    Value act = Value::makeObject();
    act["mode"] = z.activation.mode == ActivationMode::All ? "all" : "any";
    Value conds = Value::makeArray();
    for (const ZoneCondition& c : z.activation.conditions) {
        Value item = Value::makeObject();
        item["mechanism"] = c.mechanismId;
        item["value"] = c.value;
        item["op"] = conditionOpName(c.op);
        item["threshold"] = c.threshold;
        item["tolerance"] = c.tolerance;
        item["absolute"] = c.absolute;
        conds.push_back(std::move(item));
    }
    act["conditions"] = std::move(conds);
    o["activation"] = std::move(act);
    return o;
}

Value cameraValue(const CameraConfig& c) {
    Value o = Value::makeObject();
    o["id"] = c.id;
    o["label"] = c.label;
    o["name"] = c.name;
    o["kind"] = std::string(cameraKindKey(c.kind));
    Value a = Value::makeObject();
    a["parentMechanismId"] = c.attachment.onChassis() ? std::string(kChassisParent) : c.attachment.parentMechanismId;
    a["parentLink"] = c.attachment.parentLink;
    o["attachment"] = std::move(a);
    o["pose"] = poseValue(c.local);
    o["hFovDeg"] = c.hFovDeg;
    o["vFovDeg"] = c.vFovDeg;
    o["maxRangeM"] = c.maxRangeM;
    o["maxIncidenceDeg"] = c.maxIncidenceDeg;
    o["fps"] = c.fps;
    o["latencyMs"] = c.latencyMs;
    o["widthPx"] = c.widthPx;
    o["heightPx"] = c.heightPx;
    o["positionNoiseM"] = c.positionNoiseM;
    o["yawNoiseDeg"] = c.yawNoiseDeg;
    return o;
}

Value sensorValue(const SensorConfig& s) {
    Value o = Value::makeObject();
    o["id"] = s.id;
    o["label"] = s.label;
    o["kind"] = std::string(sensorKindKey(s.kind));
    o["source"] = s.source;
    o["properties"] = s.properties.isObject() ? s.properties : Value::makeObject();
    o["mappings"] = mappingsValue(s.mappings);
    return o;
}

Value startPositionValue(const StartPosition& p) {
    Value item = Value::makeObject();
    item["id"] = p.id;
    item["label"] = p.label;
    item["alliance"] = std::string(allianceKey(p.alliance));
    item["xM"] = fv(p.pose.xM);
    item["yM"] = fv(p.pose.yM);
    item["headingRad"] = fv(p.pose.headingRad);
    return item;
}

Value motionLinkValue(const MotionLink& link) {
    Value item = Value::makeObject();
    item["id"] = link.id;
    item["ratio"] = link.ratio;
    const auto endpoint = [&](const SlotAddress& address) {
        Value v = Value::makeObject();
        v["owner"] = address.owner;
        v["axis"] = address.slot;
        return v;
    };
    item["source"] = endpoint(link.source);
    item["target"] = endpoint(link.target);
    return item;
}

Value projectValue(const ProjectInfo& p) {
    Value project = Value::makeObject();
    project["path"] = p.path;
    project["language"] = p.language;
    project["wpilibYear"] = p.wpilibYear;
    project["gradleRioVersion"] = p.gradleRioVersion;
    return project;
}

Value startPoseValue(const RobotStartPose& p) {
    Value pose = Value::makeObject();
    pose["xM"] = fv(p.xM);
    pose["yM"] = fv(p.yM);
    pose["headingRad"] = fv(p.headingRad);
    return pose;
}

Value batteryValue(const BatteryConfig& b) {
    Value battery = Value::makeObject();
    battery["nominalVoltageV"] = b.nominalVoltageV;
    battery["internalResistanceOhm"] = b.internalResistanceOhm;
    battery["brownoutVoltageV"] = b.brownoutVoltageV;
    return battery;
}

Value simValue(const SimSettings& s) {
    Value sim = Value::makeObject();
    sim["stepUs"] = s.stepUs;
    sim["seed"] = static_cast<std::int64_t>(s.seed);
    Value physics = Value::makeObject();
    physics["workerCount"] = s.physics.workerCount;
    physics["subSteps"] = s.physics.subSteps;
    physics["sleeping"] = s.physics.sleeping;
    physics["gravity"] = vec3Value(s.physics.gravity);
    sim["physics"] = std::move(physics);
    return sim;
}

Value pluginsValue(const RobotConfig& cfg) {
    Value plugins = Value::makeObject();
    for (const auto& [id, section] : cfg.plugins) {
        Value s = Value::makeObject();
        s["schemaVersion"] = section.schemaVersion;
        s["data"] = section.data.isObject() ? section.data : Value::makeObject();
        plugins.object().set(id, std::move(s));
    }
    return plugins;
}

}  // namespace

Value RobotConfigCodec::toValue(const RobotConfig& cfg) const {
    Value root = Value::makeObject();
    root["schemaVersion"] = cfg.schemaVersion;
    root["name"] = cfg.name;
    root["project"] = projectValue(cfg.project);
    root["gamePluginId"] = cfg.gamePluginId;
    if (cfg.startPose) root["startPose"] = startPoseValue(*cfg.startPose);
    root["chassis"] = chassisValue(cfg.chassis);
    root["drivetrain"] = drivetrainValue(cfg.drivetrain);

    Value mechs = Value::makeArray();
    for (const MechanismConfig& m : cfg.mechanisms) mechs.push_back(mechanismValue(m));
    root["mechanisms"] = std::move(mechs);
    root["robotMappings"] = mappingsValue(cfg.robotMappings);
    Value zones = Value::makeArray();
    for (const ZoneConfig& z : cfg.zones) zones.push_back(zoneValue(z));
    root["zones"] = std::move(zones);
    Value cameras = Value::makeArray();
    for (const CameraConfig& c : cfg.cameras) cameras.push_back(cameraValue(c));
    root["cameras"] = std::move(cameras);
    Value sensors = Value::makeArray();
    for (const SensorConfig& s : cfg.sensors) sensors.push_back(sensorValue(s));
    root["sensors"] = std::move(sensors);
    Value startPositions = Value::makeArray();
    for (const StartPosition& p : cfg.startPositions) startPositions.push_back(startPositionValue(p));
    root["startPositions"] = std::move(startPositions);
    Value links = Value::makeArray();
    for (const auto& link : cfg.motionLinks) links.push_back(motionLinkValue(link));
    root["motionLinks"] = std::move(links);
    root["battery"] = batteryValue(cfg.battery);
    root["sim"] = simValue(cfg.sim);
    root["plugins"] = pluginsValue(cfg);

    return root;
}

}  // namespace wpiemu::core::config
