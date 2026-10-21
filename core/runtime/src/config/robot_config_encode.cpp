#include <string>

#include "robot_config_codec_io.h"
#include "wpiemu/core/config/robot_config_codec.h"

namespace wpiemu::core::config {

using namespace wpiemu::sdk;
using namespace codec_io;

namespace {

template <class T, class Fn>
Value encodeObject(const T& item, Fn fieldsFn) {
    ObjWriter w;
    fieldsFn(w, item);
    return w.take();
}

Value materialValue(const Material& m) { return encodeObject(m, materialFields<ObjWriter, const Material>); }

Value motorValue(const MotorSpec& m) { return encodeObject(m, motorFields<ObjWriter, const MotorSpec>); }

Value tireValue(const TireParams& t) { return encodeObject(t, tireFields<ObjWriter, const TireParams>); }

Value primitiveValue(const PrimitiveSpec& p) {
    ObjWriter w;
    w.str("name", p.name);
    w.str("kind", p.kind == PrimitiveKind::Box ? "box" : "cylinder");
    primitiveGeometryFields(w, p);
    return w.take();
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
    ObjWriter out;
    out.str("name", w.name);
    out.str("kind", wheelKindName(w.kind));
    out.vec3("positionM", w.positionM);
    out.flt("radiusM", w.radiusM);
    out.flt("widthM", w.widthM);
    out.flt("inertiaKgM2", w.inertiaKgM2);
    out.set("tire", tireValue(w.tire));
    out.boolean("steered", w.steered);
    out.set("driveMotor", motorValue(w.driveMotor));
    out.i32("driveMotorCount", w.driveMotorCount);
    out.flt("driveGearRatio", w.driveGearRatio);
    out.flt("driveEfficiency", w.driveEfficiency);
    out.flt("driveCurrentLimitA", w.driveCurrentLimitA);
    out.set("steerMotor", motorValue(w.steerMotor));
    out.flt("steerGearRatio", w.steerGearRatio);
    out.flt("steerInertiaKgM2", w.steerInertiaKgM2);
    return out.take();
}

Value chassisValue(const ChassisSpec& c) {
    ObjWriter out;
    chassisCoreFields(out, c);
    out.set("material", materialValue(c.material));
    out.set("primitives", primitivesValue(c.primitives));
    return out.take();
}

Value drivetrainValue(const DrivetrainConfig& d) {
    ObjWriter out;
    drivetrainCoreFields(out, d.spec);
    Value wheels = Value::makeArray();
    for (const WheelSpec& w : d.spec.wheels) wheels.push_back(wheelValue(w));
    out.set("wheels", std::move(wheels));
    out.set("mappings", mappingsValue(d.mappings));
    return out.take();
}

Value mechanismValue(const MechanismConfig& m) {
    ObjWriter out;
    out.str("id", m.id);
    out.str("type", m.type);
    out.str("label", m.label);
    out.attachment("attachment", m.attachment);
    out.pose("mount", m.mount);
    out.set("geometry", primitivesValue(m.geometry));
    out.set("properties", m.properties.isObject() ? m.properties : Value::makeObject());
    out.set("mappings", mappingsValue(m.mappings));
    return out.take();
}

Value zoneValue(const ZoneConfig& z) {
    ObjWriter out;
    out.str("id", z.id);
    out.str("kind", z.kind);
    out.str("label", z.label);
    out.attachment("attachment", z.attachment);
    out.pose("pose", z.local);
    out.vec3("sizeM", z.sizeM);
    out.set("properties", z.properties.isObject() ? z.properties : Value::makeObject());
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
    out.set("activation", std::move(act));
    return out.take();
}

Value cameraValue(const CameraConfig& c) {
    ObjWriter out;
    out.str("id", c.id);
    out.str("label", c.label);
    out.str("name", c.name);
    out.str("kind", std::string(cameraKindKey(c.kind)));
    out.attachment("attachment", c.attachment);
    cameraOpticsFields(out, c);
    return out.take();
}

Value sensorValue(const SensorConfig& s) {
    ObjWriter out;
    out.str("id", s.id);
    out.str("label", s.label);
    out.str("kind", std::string(sensorKindKey(s.kind)));
    out.str("source", s.source);
    out.set("properties", s.properties.isObject() ? s.properties : Value::makeObject());
    out.set("mappings", mappingsValue(s.mappings));
    return out.take();
}

Value startPositionValue(const StartPosition& p) {
    ObjWriter out;
    out.str("id", p.id);
    out.str("label", p.label);
    out.str("alliance", std::string(allianceKey(p.alliance)));
    out.flt("xM", p.pose.xM);
    out.flt("yM", p.pose.yM);
    out.flt("headingRad", p.pose.headingRad);
    return out.take();
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

Value projectValue(const ProjectInfo& p) { return encodeObject(p, projectFields<ObjWriter, const ProjectInfo>); }

Value startPoseValue(const RobotStartPose& p) {
    ObjWriter out;
    out.flt("xM", p.xM);
    out.flt("yM", p.yM);
    out.flt("headingRad", p.headingRad);
    return out.take();
}

Value batteryValue(const BatteryConfig& b) { return encodeObject(b, batteryFields<ObjWriter, const BatteryConfig>); }

Value simValue(const SimSettings& s) {
    ObjWriter out;
    out.i64("stepUs", s.stepUs);
    out.u64("seed", s.seed);
    out.set("physics", encodeObject(s.physics, physicsFields<ObjWriter, const PhysicsConfig>));
    return out.take();
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
