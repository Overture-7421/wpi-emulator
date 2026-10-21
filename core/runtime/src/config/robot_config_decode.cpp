#include <string>
#include <string_view>

#include "robot_config_codec_io.h"
#include "wpiemu/core/config/robot_config_codec.h"

namespace wpiemu::core::config {

using namespace wpiemu::sdk;
using namespace codec_io;

namespace {

void readMaterial(const Value* v, const std::string& path, Ctx& ctx, Material& m) {
    if (v == nullptr) return;
    ObjReader r(*v, path, ctx);
    materialFields(r, m);
}

void readMotor(const Value* v, const std::string& path, Ctx& ctx, MotorSpec& m) {
    if (v == nullptr) return;
    ObjReader r(*v, path, ctx);
    motorFields(r, m);
}

void readTire(const Value* v, const std::string& path, Ctx& ctx, TireParams& t) {
    if (v == nullptr) return;
    ObjReader r(*v, path, ctx);
    tireFields(r, t);
}

PrimitiveSpec readPrimitive(const Value& v, const std::string& path, Ctx& ctx) {
    PrimitiveSpec p;
    ObjReader r(v, path, ctx);
    r.str("name", p.name);
    std::string kind;
    if (r.str("kind", kind)) {
        if (kind == "box") p.kind = PrimitiveKind::Box;
        else if (kind == "cylinder") p.kind = PrimitiveKind::Cylinder;
        else ctx.err(join(path, "kind"), "unknown primitive kind '" + kind + "' (box|cylinder)");
    }
    primitiveGeometryFields(r, p);
    return p;
}

std::vector<PrimitiveSpec> readPrimitives(const Value* v, const std::string& path, Ctx& ctx) {
    std::vector<PrimitiveSpec> out;
    if (v == nullptr) return out;
    if (!v->isArray()) {
        ctx.err(path, "expected an array");
        return out;
    }
    for (std::size_t i = 0; i < v->array().size(); ++i)
        out.push_back(readPrimitive(v->array()[i], path + "[" + std::to_string(i) + "]", ctx));
    return out;
}

std::vector<SlotMapping> readMappings(const Value* v, const std::string& path, Ctx& ctx) {
    std::vector<SlotMapping> out;
    if (v == nullptr) return out;
    if (!v->isArray()) {
        ctx.err(path, "expected an array");
        return out;
    }
    for (std::size_t i = 0; i < v->array().size(); ++i) {
        const std::string p = path + "[" + std::to_string(i) + "]";
        ObjReader r(v->array()[i], p, ctx);
        SlotMapping m;
        r.str("owner", m.slot.owner);
        r.str("slot", m.slot.slot);
        r.boolean("reversed", m.reversed);
        if (const Value* d = r.get("device")) {
            ObjReader dr(*d, join(p, "device"), ctx);
            dr.str("family", m.device.family);
            dr.str("type", m.device.type);
            dr.i32("id", m.device.id);
            dr.str("bus", m.device.bus);
            dr.i32("channel", m.device.channel);
        } else {
            ctx.err(join(p, "device"), "missing device address");
        }
        if (r.valid() && (m.slot.owner.empty() || m.slot.slot.empty())) ctx.err(p, "mapping needs 'owner' and 'slot'");
        out.push_back(std::move(m));
    }
    return out;
}

void readWheel(const Value& v, const std::string& path, Ctx& ctx, WheelSpec& w) {
    ObjReader r(v, path, ctx);
    r.str("name", w.name);
    std::string kind;
    if (r.str("kind", kind)) {
        if (const std::optional<WheelKind> parsed = parseWheelKind(kind)) w.kind = *parsed;
        else ctx.err(join(path, "kind"), "unknown wheel kind '" + kind + "' (standard|omni|mecanum|swerve)");
    }
    r.vec3("positionM", w.positionM);
    r.flt("radiusM", w.radiusM);
    r.flt("widthM", w.widthM);
    r.flt("inertiaKgM2", w.inertiaKgM2);
    readTire(r.get("tire"), join(path, "tire"), ctx, w.tire);
    r.boolean("steered", w.steered);
    readMotor(r.get("driveMotor"), join(path, "driveMotor"), ctx, w.driveMotor);
    r.i32("driveMotorCount", w.driveMotorCount);
    r.flt("driveGearRatio", w.driveGearRatio);
    r.flt("driveEfficiency", w.driveEfficiency);
    r.flt("driveCurrentLimitA", w.driveCurrentLimitA);
    readMotor(r.get("steerMotor"), join(path, "steerMotor"), ctx, w.steerMotor);
    r.flt("steerGearRatio", w.steerGearRatio);
    r.flt("steerInertiaKgM2", w.steerInertiaKgM2);
}

void readChassis(const Value& v, Ctx& ctx, RobotConfig& cfg) {
    ObjReader r(v, "chassis", ctx);
    ChassisSpec& c = cfg.chassis;
    chassisCoreFields(r, c);
    readMaterial(r.get("material"), "chassis.material", ctx, c.material);
    c.primitives = readPrimitives(r.get("primitives"), "chassis.primitives", ctx);
}

void readDrivetrain(const Value& v, Ctx& ctx, RobotConfig& cfg) {
    ObjReader r(v, "drivetrain", ctx);
    DrivetrainConfig& d = cfg.drivetrain;
    drivetrainCoreFields(r, d.spec);
    if (const Value* wheels = r.get("wheels")) {
        if (!wheels->isArray()) {
            ctx.err("drivetrain.wheels", "expected an array");
        } else {
            for (std::size_t i = 0; i < wheels->array().size(); ++i) {
                WheelSpec w;
                readWheel(wheels->array()[i], "drivetrain.wheels[" + std::to_string(i) + "]", ctx, w);
                d.spec.wheels.push_back(std::move(w));
            }
        }
    }
    d.mappings = readMappings(r.get("mappings"), "drivetrain.mappings", ctx);
}

void readMechanism(const Value& v, const std::string& path, Ctx& ctx, RobotConfig& cfg) {
    MechanismConfig m;
    ObjReader r(v, path, ctx);
    r.str("id", m.id);
    r.str("type", m.type);
    r.str("label", m.label);
    r.attachment("attachment", m.attachment);
    r.pose("mount", m.mount);
    m.geometry = readPrimitives(r.get("geometry"), join(path, "geometry"), ctx);
    if (const Value* props = r.get("properties")) {
        if (props->isObject()) m.properties = *props;
        else ctx.err(join(path, "properties"), "expected an object");
    }
    m.mappings = readMappings(r.get("mappings"), join(path, "mappings"), ctx);
    cfg.mechanisms.push_back(std::move(m));
}

void readZone(const Value& v, const std::string& path, Ctx& ctx, RobotConfig& cfg) {
    ZoneConfig z;
    ObjReader r(v, path, ctx);
    r.str("id", z.id);
    r.str("kind", z.kind);
    r.str("label", z.label);
    r.attachment("attachment", z.attachment);
    r.pose("pose", z.local);
    r.vec3("sizeM", z.sizeM);
    if (const Value* props = r.get("properties")) {
        if (props->isObject()) z.properties = *props;
        else ctx.err(join(path, "properties"), "expected an object");
    }
    if (const Value* act = r.get("activation")) {
        const std::string actPath = join(path, "activation");
        ObjReader ar(*act, actPath, ctx);
        std::string mode;
        if (ar.str("mode", mode)) {
            if (mode == "all") z.activation.mode = ActivationMode::All;
            else if (mode == "any") z.activation.mode = ActivationMode::Any;
            else ctx.err(join(actPath, "mode"), "unknown activation mode '" + mode + "' (all|any)");
        }
        if (const Value* conds = ar.get("conditions")) {
            if (!conds->isArray()) {
                ctx.err(join(actPath, "conditions"), "expected an array");
            } else {
                for (std::size_t i = 0; i < conds->array().size(); ++i) {
                    const std::string cp = join(actPath, "conditions") + "[" + std::to_string(i) + "]";
                    ObjReader cr(conds->array()[i], cp, ctx);
                    ZoneCondition c;
                    cr.str("mechanism", c.mechanismId);
                    cr.str("value", c.value);
                    std::string op;
                    if (cr.str("op", op) && !conditionOpFrom(op, c.op))
                        ctx.err(join(cp, "op"), "unknown condition op '" + op + "' (above|below|within)");
                    cr.dbl("threshold", c.threshold);
                    cr.dbl("tolerance", c.tolerance);
                    cr.boolean("absolute", c.absolute);
                    z.activation.conditions.push_back(std::move(c));
                }
            }
        }
    }
    cfg.zones.push_back(std::move(z));
}

void readCamera(const Value& v, const std::string& path, Ctx& ctx, RobotConfig& cfg) {
    CameraConfig c;
    ObjReader r(v, path, ctx);
    r.str("id", c.id);
    r.str("label", c.label);
    r.str("name", c.name);
    std::string kind;
    if (r.str("kind", kind)) {
        if (kind == cameraKindKey(CameraKind::Limelight)) c.kind = CameraKind::Limelight;
        else if (kind == cameraKindKey(CameraKind::PhotonVision)) c.kind = CameraKind::PhotonVision;
        else ctx.err(join(path, "kind"), "unknown camera kind '" + kind + "' (limelight|photonvision)");
    }
    r.attachment("attachment", c.attachment);
    cameraOpticsFields(r, c);
    cfg.cameras.push_back(std::move(c));
}

void readSensor(const Value& v, const std::string& path, Ctx& ctx, RobotConfig& cfg) {
    SensorConfig s;
    ObjReader r(v, path, ctx);
    r.str("id", s.id);
    r.str("label", s.label);
    std::string kind;
    if (r.str("kind", kind) && !sensorKindFromKey(kind, s.kind))
        ctx.err(join(path, "kind"), "unknown sensor kind '" + kind + "' (distance|beam_break)");
    r.str("source", s.source);
    if (const Value* props = r.get("properties")) {
        if (props->isObject()) s.properties = *props;
        else ctx.err(join(path, "properties"), "expected an object");
    }
    s.mappings = readMappings(r.get("mappings"), join(path, "mappings"), ctx);
    cfg.sensors.push_back(std::move(s));
}

void readStartPosition(const Value& v, const std::string& path, Ctx& ctx, RobotConfig& cfg) {
    StartPosition p;
    ObjReader r(v, path, ctx);
    r.str("id", p.id);
    r.str("label", p.label);
    std::string alliance;
    if (r.str("alliance", alliance)) {
        if (alliance == allianceKey(Alliance::Blue)) p.alliance = Alliance::Blue;
        else if (alliance == allianceKey(Alliance::Red)) p.alliance = Alliance::Red;
        else ctx.err(join(path, "alliance"), "unknown alliance '" + alliance + "' (blue|red)");
    }
    if (!r.flt("xM", p.pose.xM)) ctx.err(join(path, "xM"), "required finite number");
    if (!r.flt("yM", p.pose.yM)) ctx.err(join(path, "yM"), "required finite number");
    if (!r.flt("headingRad", p.pose.headingRad)) ctx.err(join(path, "headingRad"), "required finite number");
    cfg.startPositions.push_back(std::move(p));
}

// ---- the members of the root object, in document order ----

void readStartPose(const Value& v, Ctx& ctx, RobotConfig& cfg) {
    ObjReader r(v, "startPose", ctx);
    RobotStartPose pose;
    if (!r.flt("xM", pose.xM)) ctx.err("startPose.xM", "required finite number");
    if (!r.flt("yM", pose.yM)) ctx.err("startPose.yM", "required finite number");
    if (!r.flt("headingRad", pose.headingRad))
        ctx.err("startPose.headingRad", "required finite number");
    cfg.startPose = pose;
}

void readProject(const Value& v, Ctx& ctx, RobotConfig& cfg) {
    ObjReader r(v, "project", ctx);
    projectFields(r, cfg.project);
}

void readMechanisms(const Value& ms, Ctx& ctx, RobotConfig& cfg) {
    if (!ms.isArray()) {
        ctx.err("mechanisms", "expected an array");
    } else {
        for (std::size_t i = 0; i < ms.array().size(); ++i)
            readMechanism(ms.array()[i], "mechanisms[" + std::to_string(i) + "]", ctx, cfg);
    }
}

void readZones(const Value& zs, Ctx& ctx, RobotConfig& cfg) {
    if (!zs.isArray()) {
        ctx.err("zones", "expected an array");
    } else {
        for (std::size_t i = 0; i < zs.array().size(); ++i)
            readZone(zs.array()[i], "zones[" + std::to_string(i) + "]", ctx, cfg);
    }
}

void readCameras(const Value& cs, Ctx& ctx, RobotConfig& cfg) {
    if (!cs.isArray()) {
        ctx.err("cameras", "expected an array");
    } else {
        for (std::size_t i = 0; i < cs.array().size(); ++i)
            readCamera(cs.array()[i], "cameras[" + std::to_string(i) + "]", ctx, cfg);
    }
}

void readSensors(const Value& ss, Ctx& ctx, RobotConfig& cfg) {
    if (!ss.isArray()) {
        ctx.err("sensors", "expected an array");
    } else {
        for (std::size_t i = 0; i < ss.array().size(); ++i)
            readSensor(ss.array()[i], "sensors[" + std::to_string(i) + "]", ctx, cfg);
    }
}

void readStartPositions(const Value& ps, Ctx& ctx, RobotConfig& cfg) {
    if (!ps.isArray()) {
        ctx.err("startPositions", "expected an array");
    } else {
        for (std::size_t i = 0; i < ps.array().size(); ++i)
            readStartPosition(ps.array()[i], "startPositions[" + std::to_string(i) + "]", ctx, cfg);
    }
}

void readMotionLinks(const Value& links, Ctx& ctx, RobotConfig& cfg) {
    if (!links.isArray()) {
        ctx.err("motionLinks", "expected an array");
    } else {
        for (std::size_t i = 0; i < links.array().size(); ++i) {
            const std::string path = "motionLinks[" + std::to_string(i) + "]";
            ObjReader r(links.array()[i], path, ctx);
            MotionLink link;
            r.str("id", link.id);
            r.dbl("ratio", link.ratio);
            const auto endpoint = [&](const char* key, SlotAddress& address) {
                const auto* v = r.get(key);
                if (v == nullptr) {
                    ctx.err(path + "." + key, "missing endpoint");
                    return;
                }
                ObjReader e(*v, path + "." + key, ctx);
                e.str("owner", address.owner);
                e.str("axis", address.slot);
            };
            endpoint("source", link.source);
            endpoint("target", link.target);
            cfg.motionLinks.push_back(std::move(link));
        }
    }
}

void readBattery(const Value& b, Ctx& ctx, RobotConfig& cfg) {
    ObjReader r(b, "battery", ctx);
    batteryFields(r, cfg.battery);
}

void readSim(const Value& s, Ctx& ctx, RobotConfig& cfg) {
    ObjReader r(s, "sim", ctx);
    r.i64("stepUs", cfg.sim.stepUs);
    r.u64("seed", cfg.sim.seed);
    if (const Value* ph = r.get("physics")) {
        ObjReader pr(*ph, "sim.physics", ctx);
        physicsFields(pr, cfg.sim.physics);
    }
}

void readPlugins(const Value& ps, Ctx& ctx, RobotConfig& cfg) {
    if (!ps.isObject()) {
        ctx.err("plugins", "expected an object");
    } else {
        for (std::size_t i = 0; i < ps.object().size(); ++i) {
            const std::string& id = ps.object().keyAt(i);
            ObjReader r(ps.object().valueAt(i), "plugins." + id, ctx);
            PluginSection section;
            r.i32("schemaVersion", section.schemaVersion);
            if (const Value* data = r.get("data")) {
                if (data->isObject()) section.data = *data;
                else ctx.err("plugins." + id + ".data", "expected an object");
            }
            cfg.plugins[id] = std::move(section);
        }
    }
}

}  // namespace

Result<RobotConfig> RobotConfigCodec::fromValue(const Value& value) const {
    if (!value.isObject()) return makeError(ErrorCode::ParseError, "robot config must be a JSON object");
    if (const Value* sv = value.find("schemaVersion"); sv != nullptr) {
        int version = 0;
        if (!readInteger(*sv, version))
            return makeError(ErrorCode::ParseError,
                             "schemaVersion: expected an integer in the int range");
        if (version != kRobotConfigSchemaVersion)
            return makeError(ErrorCode::VersionMismatch,
                             "robot config schemaVersion " + std::to_string(version) + " is not supported (expected " +
                                 std::to_string(kRobotConfigSchemaVersion) + ")");
    }

    const Value& work = value;
    Ctx ctx;
    RobotConfig cfg;
    cfg.schemaVersion = kRobotConfigSchemaVersion;
    ObjReader root(work, "", ctx);
    root.str("name", cfg.name);
    root.str("gamePluginId", cfg.gamePluginId);
    if (const Value* p = root.get("startPose")) readStartPose(*p, ctx, cfg);
    if (const Value* p = root.get("project")) readProject(*p, ctx, cfg);
    if (const Value* c = root.get("chassis")) readChassis(*c, ctx, cfg);
    if (const Value* d = root.get("drivetrain")) readDrivetrain(*d, ctx, cfg);
    if (const Value* ms = root.get("mechanisms")) readMechanisms(*ms, ctx, cfg);
    cfg.robotMappings = readMappings(root.get("robotMappings"), "robotMappings", ctx);
    if (const Value* zs = root.get("zones")) readZones(*zs, ctx, cfg);
    if (const Value* cs = root.get("cameras")) readCameras(*cs, ctx, cfg);
    if (const Value* ss = root.get("sensors")) readSensors(*ss, ctx, cfg);
    if (const Value* ps = root.get("startPositions")) readStartPositions(*ps, ctx, cfg);
    if (const Value* links = root.get("motionLinks")) readMotionLinks(*links, ctx, cfg);
    if (const Value* b = root.get("battery")) readBattery(*b, ctx, cfg);
    if (const Value* s = root.get("sim")) readSim(*s, ctx, cfg);
    if (const Value* ps = root.get("plugins")) readPlugins(*ps, ctx, cfg);

    if (!ctx.errors.empty()) {
        std::string msg = "invalid robot config: " + ctx.errors.front();
        for (std::size_t i = 1; i < ctx.errors.size() && i < 5; ++i) msg += "; " + ctx.errors[i];
        if (ctx.errors.size() > 5) msg += "; (+" + std::to_string(ctx.errors.size() - 5) + " more)";
        return makeError(ErrorCode::ParseError, std::move(msg));
    }
    return cfg;
}

}  // namespace wpiemu::core::config
