#include "wpiemu/core/config/robot_config_codec.h"

#include <cmath>
#include <limits>
#include <string>
#include <string_view>

#include "wpiemu/core/config/json_codec.h"

namespace wpiemu::core::config {

using namespace wpiemu::sdk;

namespace {

// ---- decode helpers ------------------------------------------------------------------------------

struct Ctx {
    std::vector<std::string> errors;
    void err(const std::string& path, const std::string& what) { errors.push_back((path.empty() ? "<root>" : path) + ": " + what); }
};

std::string join(const std::string& path, std::string_view key) {
    return path.empty() ? std::string(key) : path + "." + std::string(key);
}

template <typename T>
bool readInteger(const Value& value, T& out) {
    static_assert(std::numeric_limits<T>::is_integer && std::numeric_limits<T>::is_signed);
    if (value.isInt()) {
        const std::int64_t n = value.asInt();
        if (n < std::numeric_limits<T>::min() || n > std::numeric_limits<T>::max()) return false;
        out = static_cast<T>(n);
        return true;
    }
    if (!value.isDouble()) return false;
    const double d = value.asDouble();
    // The signed upper bound is exclusive: INT64_MAX rounds up to 2^63 as a double.
    const double limit = std::ldexp(1.0, std::numeric_limits<T>::digits);
    if (!std::isfinite(d) || std::trunc(d) != d || d < -limit || d >= limit) return false;
    out = static_cast<T>(d);
    return true;
}

// Typed access to one JSON object. Keys nobody asks for are ignored.
class ObjReader {
public:
    ObjReader(const Value& v, std::string path, Ctx& ctx) : path_(std::move(path)), ctx_(ctx) {
        if (v.isObject()) obj_ = &v;
        else ctx_.err(path_, "expected an object");
    }
    bool valid() const { return obj_ != nullptr; }
    const std::string& path() const { return path_; }

    const Value* get(std::string_view key) const {
        return obj_ != nullptr ? obj_->find(key) : nullptr;
    }

    bool str(std::string_view key, std::string& out) {
        const Value* v = get(key);
        if (v == nullptr) return false;
        if (!v->isString()) return typeErr(key, "a string");
        out = v->asString();
        return true;
    }
    bool boolean(std::string_view key, bool& out) {
        const Value* v = get(key);
        if (v == nullptr) return false;
        if (!v->isBool()) return typeErr(key, "a boolean");
        out = v->asBool();
        return true;
    }
    bool dbl(std::string_view key, double& out) {
        const Value* v = get(key);
        if (v == nullptr) return false;
        if (!v->isNumber()) return typeErr(key, "a number");
        if (!std::isfinite(v->asDouble())) return typeErr(key, "a finite number");
        out = v->asDouble();
        return true;
    }
    bool flt(std::string_view key, float& out) {
        double d = 0.0;
        if (!dbl(key, d)) return false;
        if (d < -std::numeric_limits<float>::max() || d > std::numeric_limits<float>::max())
            return typeErr(key, "a number in the float range");
        out = static_cast<float>(d);
        return true;
    }
    bool i64(std::string_view key, std::int64_t& out) {
        const Value* v = get(key);
        if (v == nullptr) return false;
        return readInteger(*v, out) || typeErr(key, "an integer in the int64 range");
    }
    bool i32(std::string_view key, int& out) {
        const Value* v = get(key);
        if (v == nullptr) return false;
        return readInteger(*v, out) || typeErr(key, "an integer in the int range");
    }
    bool u64(std::string_view key, std::uint64_t& out) {
        std::int64_t v = 0;
        if (!i64(key, v)) return false;
        // Seeds use the persisted signed int64 representation, including negative high-bit seeds.
        out = static_cast<std::uint64_t>(v);
        return true;
    }
    bool vec3(std::string_view key, Vec3& out) {
        const Value* v = get(key);
        if (v == nullptr) return false;
        ObjReader r(*v, join(path_, key), ctx_);
        Vec3 tmp;
        if (!r.flt("x", tmp.x) || !r.flt("y", tmp.y) || !r.flt("z", tmp.z))
            return typeErr(key, "an {x,y,z} object with finite float components");
        out = tmp;
        return true;
    }
    bool pose(std::string_view key, Pose3& out) {
        const Value* v = get(key);
        if (v == nullptr) return false;
        ObjReader p(*v, join(path_, key), ctx_);
        if (!p.valid()) return false;
        Pose3 tmp = out;
        p.vec3("position", tmp.position);
        p.quat("rotation", tmp.rotation);
        out = tmp;
        return true;
    }
    // Reads a quaternion stored under `key`; a zero-length quaternion is rejected.
    bool quat(std::string_view key, Quat& out) {
        const Value* v = get(key);
        if (v == nullptr) return false;
        ObjReader r(*v, join(path_, key), ctx_);
        Quat q;
        if (!r.flt("x", q.x) || !r.flt("y", q.y) || !r.flt("z", q.z) || !r.flt("w", q.w))
            return typeErr(key, "an {x,y,z,w} object with finite float components");
        const double n = std::sqrt(static_cast<double>(q.x) * q.x + static_cast<double>(q.y) * q.y +
                                   static_cast<double>(q.z) * q.z + static_cast<double>(q.w) * q.w);
        if (!(n > 1e-6)) return typeErr(key, "a non-zero quaternion");
        out = Quat{static_cast<float>(q.x / n), static_cast<float>(q.y / n),
                   static_cast<float>(q.z / n), static_cast<float>(q.w / n)};
        return true;
    }

private:
    bool typeErr(std::string_view key, const char* expected) {
        ctx_.err(join(path_, key), std::string("expected ") + expected);
        return false;
    }

    const Value* obj_ = nullptr;
    std::string path_;
    Ctx& ctx_;
};

bool wheelKindFrom(const std::string& s, WheelKind& out) {
    if (s == "standard") out = WheelKind::Standard;
    else if (s == "omni") out = WheelKind::Omni;
    else if (s == "mecanum") out = WheelKind::Mecanum;
    else if (s == "swerve") out = WheelKind::Swerve;
    else return false;
    return true;
}
const char* wheelKindName(WheelKind k) {
    switch (k) {
        case WheelKind::Standard: return "standard";
        case WheelKind::Omni: return "omni";
        case WheelKind::Mecanum: return "mecanum";
        case WheelKind::Swerve: return "swerve";
    }
    return "standard";
}

void readMaterial(const Value* v, const std::string& path, Ctx& ctx, Material& m) {
    if (v == nullptr) return;
    ObjReader r(*v, path, ctx);
    r.flt("friction", m.friction);
    r.flt("restitution", m.restitution);
    r.flt("rollingResistance", m.rollingResistance);
}

void readMotor(const Value* v, const std::string& path, Ctx& ctx, MotorSpec& m) {
    if (v == nullptr) return;
    ObjReader r(*v, path, ctx);
    r.str("name", m.name);
    r.dbl("nominalVoltageV", m.nominalVoltageV);
    r.dbl("stallTorqueNm", m.stallTorqueNm);
    r.dbl("stallCurrentA", m.stallCurrentA);
    r.dbl("freeCurrentA", m.freeCurrentA);
    r.dbl("freeSpeedRadPerSec", m.freeSpeedRadPerSec);
}

void readTire(const Value* v, const std::string& path, Ctx& ctx, TireParams& t) {
    if (v == nullptr) return;
    ObjReader r(*v, path, ctx);
    r.flt("mu", t.mu);
    r.flt("slideRatio", t.slideRatio);
    r.flt("peakSlipLong", t.peakSlipLong);
    r.flt("peakSlipLat", t.peakSlipLat);
    r.flt("vRefMps", t.vRefMps);
    r.flt("lateralScale", t.lateralScale);
    r.flt("tractionAngleRad", t.tractionAngleRad);
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
    r.pose("local", p.local);
    r.vec3("sizeM", p.sizeM);
    r.flt("radiusM", p.radiusM);
    r.flt("heightM", p.heightM);
    r.str("color", p.color);
    r.boolean("collides", p.collides);
    r.flt("massKg", p.massKg);
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
    if (r.str("kind", kind) && !wheelKindFrom(kind, w.kind))
        ctx.err(join(path, "kind"), "unknown wheel kind '" + kind + "' (standard|omni|mecanum|swerve)");
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
    r.vec3("sizeM", c.sizeM);
    r.flt("massKg", c.massKg);
    r.flt("comHeightM", c.comHeightM);
    r.vec3("comOffsetM", c.comOffsetM);
    r.flt("groundClearanceM", c.groundClearanceM);
    r.vec3("inertiaDiagKgM2", c.inertiaDiagKgM2);
    readMaterial(r.get("material"), "chassis.material", ctx, c.material);
    c.primitives = readPrimitives(r.get("primitives"), "chassis.primitives", ctx);
}

void readDrivetrain(const Value& v, Ctx& ctx, RobotConfig& cfg) {
    ObjReader r(v, "drivetrain", ctx);
    DrivetrainConfig& d = cfg.drivetrain;
    r.str("kind", d.spec.kind);
    r.str("tireModel", d.spec.tireModel);
    r.flt("loadTransferTauS", d.spec.loadTransferTauS);
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
    if (const Value* a = r.get("attachment")) {
        ObjReader ar(*a, join(path, "attachment"), ctx);
        ar.str("parentMechanismId", m.attachment.parentMechanismId);
        ar.str("parentLink", m.attachment.parentLink);
    }
    r.pose("mount", m.mount);
    m.geometry = readPrimitives(r.get("geometry"), join(path, "geometry"), ctx);
    if (const Value* props = r.get("properties")) {
        if (props->isObject()) m.properties = *props;
        else ctx.err(join(path, "properties"), "expected an object");
    }
    m.mappings = readMappings(r.get("mappings"), join(path, "mappings"), ctx);
    cfg.mechanisms.push_back(std::move(m));
}

bool conditionOpFrom(const std::string& s, ConditionOp& out) {
    if (s == "above") out = ConditionOp::Above;
    else if (s == "below") out = ConditionOp::Below;
    else if (s == "within") out = ConditionOp::Within;
    else return false;
    return true;
}
const char* conditionOpName(ConditionOp op) {
    switch (op) {
        case ConditionOp::Above: return "above";
        case ConditionOp::Below: return "below";
        case ConditionOp::Within: return "within";
    }
    return "above";
}

void readZone(const Value& v, const std::string& path, Ctx& ctx, RobotConfig& cfg) {
    ZoneConfig z;
    ObjReader r(v, path, ctx);
    r.str("id", z.id);
    r.str("kind", z.kind);
    r.str("label", z.label);
    if (const Value* a = r.get("attachment")) {
        ObjReader ar(*a, join(path, "attachment"), ctx);
        ar.str("parentMechanismId", z.attachment.parentMechanismId);
        ar.str("parentLink", z.attachment.parentLink);
    }
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

const char* cameraVendorName(CameraVendor v) {
    switch (v) {
        case CameraVendor::PhotonVision:
            return "photonvision";
        case CameraVendor::Limelight:
            return "limelight";
    }
    return "photonvision";
}
bool cameraVendorFrom(const std::string& s, CameraVendor& out) {
    if (s == "photonvision")
        out = CameraVendor::PhotonVision;
    else if (s == "limelight")
        out = CameraVendor::Limelight;
    else
        return false;
    return true;
}

void readIntrinsics(const Value& v, const std::string& path, Ctx& ctx, CameraIntrinsics& in) {
    ObjReader r(v, path, ctx);
    r.i32("widthPx", in.widthPx);
    r.i32("heightPx", in.heightPx);
    r.dbl("hfovRad", in.hfovRad);
    r.dbl("fxPx", in.fxPx);
    r.dbl("fyPx", in.fyPx);
    r.dbl("cxPx", in.cxPx);
    r.dbl("cyPx", in.cyPx);
    if (const Value* d = r.get("distortion")) {
        const std::string dp = join(path, "distortion");
        if (!d->isArray() || d->array().size() != in.distortion.size()) {
            ctx.err(dp,
                    "expected an array of " + std::to_string(in.distortion.size()) + " numbers");
        } else {
            std::array<double, 5> k{};
            bool ok = true;
            for (std::size_t i = 0; i < k.size(); ++i) {
                const Value& e = d->array()[i];
                if (e.isNumber() && std::isfinite(e.asDouble())) {
                    k[i] = e.asDouble();
                } else {
                    ctx.err(dp + "[" + std::to_string(i) + "]", "expected a finite number");
                    ok = false;
                }
            }
            if (ok) in.distortion = k;
        }
    }
}

void readCamera(const Value& v, const std::string& path, Ctx& ctx, VisionConfig& vision) {
    CameraConfig c;
    ObjReader r(v, path, ctx);
    r.str("id", c.id);
    r.str("label", c.label);
    std::string vendor;
    if (r.str("vendor", vendor) && !cameraVendorFrom(vendor, c.vendor))
        ctx.err(join(path, "vendor"),
                "unknown camera vendor '" + vendor + "' (photonvision|limelight)");
    r.str("ntName", c.ntName);
    r.boolean("enabled", c.enabled);
    if (const Value* a = r.get("attachment")) {
        ObjReader ar(*a, join(path, "attachment"), ctx);
        ar.str("parentMechanismId", c.attachment.parentMechanismId);
        ar.str("parentLink", c.attachment.parentLink);
    }
    r.pose("local", c.local);
    if (const Value* in = r.get("intrinsics"))
        readIntrinsics(*in, join(path, "intrinsics"), ctx, c.intrinsics);
    r.dbl("frameRateHz", c.frameRateHz);
    r.i64("latencyUs", c.latencyUs);
    r.i64("captureLatencyUs", c.captureLatencyUs);
    r.dbl("tagSizeM", c.tagSizeM);
    r.dbl("minTagPixels", c.minTagPixels);
    r.dbl("maxViewAngleRad", c.maxViewAngleRad);
    r.boolean("occlusion", c.occlusion);
    r.boolean("multiTag", c.multiTag);
    if (const Value* re = r.get("realism")) {
        ObjReader rr(*re, join(path, "realism"), ctx);
        rr.dbl("cornerNoisePx", c.realism.cornerNoisePx);
        rr.i64("latencyJitterUs", c.realism.latencyJitterUs);
        rr.dbl("dropoutProbability", c.realism.dropoutProbability);
    }
    if (const Value* ap = r.get("assumedPose"); ap != nullptr && !ap->isNull()) {
        Pose3 pose;
        if (r.pose("assumedPose", pose)) c.assumedPose = pose;
    }
    vision.cameras.push_back(std::move(c));
}

void readVision(const Value& v, Ctx& ctx, RobotConfig& cfg) {
    ObjReader r(v, "vision", ctx);
    r.str("layoutId", cfg.vision.layoutId);
    r.str("layoutFile", cfg.vision.layoutFile);
    r.str("photonMessageUuid", cfg.vision.photonMessageUuid);
    if (const Value* cams = r.get("cameras")) {
        if (!cams->isArray()) {
            ctx.err("vision.cameras", "expected an array");
        } else {
            for (std::size_t i = 0; i < cams->array().size(); ++i)
                readCamera(cams->array()[i], "vision.cameras[" + std::to_string(i) + "]", ctx,
                           cfg.vision);
        }
    }
}

// ---- encode helpers --------------------------------------------------------------------------------

Value fv(float f) { return Value(floatToDouble(f)); }

Value vec3Value(const Vec3& v) {
    Value o = Value::makeObject();
    o["x"] = fv(v.x);
    o["y"] = fv(v.y);
    o["z"] = fv(v.z);
    return o;
}
Value quatValue(const Quat& q) {
    Value o = Value::makeObject();
    o["x"] = fv(q.x);
    o["y"] = fv(q.y);
    o["z"] = fv(q.z);
    o["w"] = fv(q.w);
    return o;
}
Value poseValue(const Pose3& p) {
    Value o = Value::makeObject();
    o["position"] = vec3Value(p.position);
    o["rotation"] = quatValue(p.rotation);
    return o;
}

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
    o["vendor"] = cameraVendorName(c.vendor);
    o["ntName"] = c.ntName;
    o["enabled"] = c.enabled;
    Value a = Value::makeObject();
    a["parentMechanismId"] =
        c.attachment.onChassis() ? std::string(kChassisParent) : c.attachment.parentMechanismId;
    a["parentLink"] = c.attachment.parentLink;
    o["attachment"] = std::move(a);
    o["local"] = poseValue(c.local);
    Value in = Value::makeObject();
    in["widthPx"] = c.intrinsics.widthPx;
    in["heightPx"] = c.intrinsics.heightPx;
    in["hfovRad"] = c.intrinsics.hfovRad;
    in["fxPx"] = c.intrinsics.fxPx;
    in["fyPx"] = c.intrinsics.fyPx;
    in["cxPx"] = c.intrinsics.cxPx;
    in["cyPx"] = c.intrinsics.cyPx;
    Value distortion = Value::makeArray();
    for (double k : c.intrinsics.distortion) distortion.push_back(k);
    in["distortion"] = std::move(distortion);
    o["intrinsics"] = std::move(in);
    o["frameRateHz"] = c.frameRateHz;
    o["latencyUs"] = c.latencyUs;
    o["captureLatencyUs"] = c.captureLatencyUs;
    o["tagSizeM"] = c.tagSizeM;
    o["minTagPixels"] = c.minTagPixels;
    o["maxViewAngleRad"] = c.maxViewAngleRad;
    o["occlusion"] = c.occlusion;
    o["multiTag"] = c.multiTag;
    Value realism = Value::makeObject();
    realism["cornerNoisePx"] = c.realism.cornerNoisePx;
    realism["latencyJitterUs"] = c.realism.latencyJitterUs;
    realism["dropoutProbability"] = c.realism.dropoutProbability;
    o["realism"] = std::move(realism);
    if (c.assumedPose) o["assumedPose"] = poseValue(*c.assumedPose);
    return o;
}

Value visionValue(const VisionConfig& vision) {
    Value o = Value::makeObject();
    o["layoutId"] = vision.layoutId;
    o["layoutFile"] = vision.layoutFile;
    o["photonMessageUuid"] = vision.photonMessageUuid;
    Value cameras = Value::makeArray();
    for (const CameraConfig& c : vision.cameras) cameras.push_back(cameraValue(c));
    o["cameras"] = std::move(cameras);
    return o;
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

    const Value& work = value;    Ctx ctx;
    RobotConfig cfg;
    cfg.schemaVersion = kRobotConfigSchemaVersion;
    ObjReader root(work, "", ctx);
    root.str("name", cfg.name);
    root.str("gamePluginId", cfg.gamePluginId);

    if (const Value* p = root.get("project")) {
        ObjReader r(*p, "project", ctx);
        r.str("path", cfg.project.path);
        r.str("language", cfg.project.language);
        r.i32("wpilibYear", cfg.project.wpilibYear);
        r.str("gradleRioVersion", cfg.project.gradleRioVersion);
    }
    if (const Value* c = root.get("chassis")) readChassis(*c, ctx, cfg);
    if (const Value* d = root.get("drivetrain")) readDrivetrain(*d, ctx, cfg);
    if (const Value* ms = root.get("mechanisms")) {
        if (!ms->isArray()) {
            ctx.err("mechanisms", "expected an array");
        } else {
            for (std::size_t i = 0; i < ms->array().size(); ++i)
                readMechanism(ms->array()[i], "mechanisms[" + std::to_string(i) + "]", ctx, cfg);
        }
    }
    cfg.robotMappings = readMappings(root.get("robotMappings"), "robotMappings", ctx);
    if (const Value* zs = root.get("zones")) {
        if (!zs->isArray()) {
            ctx.err("zones", "expected an array");
        } else {
            for (std::size_t i = 0; i < zs->array().size(); ++i)
                readZone(zs->array()[i], "zones[" + std::to_string(i) + "]", ctx, cfg);
        }
    }
    if (const Value* vs = root.get("vision")) readVision(*vs, ctx, cfg);
    if (const Value* links = root.get("motionLinks")) {
        if (!links->isArray()) {
            ctx.err("motionLinks", "expected an array");
        } else {
            for (std::size_t i = 0; i < links->array().size(); ++i) {
                const std::string path = "motionLinks[" + std::to_string(i) + "]";
                ObjReader r(links->array()[i], path, ctx);
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
    if (const Value* b = root.get("battery")) {
        ObjReader r(*b, "battery", ctx);
        r.dbl("nominalVoltageV", cfg.battery.nominalVoltageV);
        r.dbl("internalResistanceOhm", cfg.battery.internalResistanceOhm);
        r.dbl("brownoutVoltageV", cfg.battery.brownoutVoltageV);
    }
    if (const Value* s = root.get("sim")) {
        ObjReader r(*s, "sim", ctx);
        r.i64("stepUs", cfg.sim.stepUs);
        r.u64("seed", cfg.sim.seed);
        if (const Value* ph = r.get("physics")) {
            ObjReader pr(*ph, "sim.physics", ctx);
            pr.i32("workerCount", cfg.sim.physics.workerCount);
            pr.i32("subSteps", cfg.sim.physics.subSteps);
            pr.boolean("sleeping", cfg.sim.physics.sleeping);
            pr.vec3("gravity", cfg.sim.physics.gravity);
        }
    }
    if (const Value* ps = root.get("plugins")) {
        if (!ps->isObject()) {
            ctx.err("plugins", "expected an object");
        } else {
            for (std::size_t i = 0; i < ps->object().size(); ++i) {
                const std::string& id = ps->object().keyAt(i);
                ObjReader r(ps->object().valueAt(i), "plugins." + id, ctx);
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

    if (!ctx.errors.empty()) {
        std::string msg = "invalid robot config: " + ctx.errors.front();
        for (std::size_t i = 1; i < ctx.errors.size() && i < 5; ++i) msg += "; " + ctx.errors[i];
        if (ctx.errors.size() > 5) msg += "; (+" + std::to_string(ctx.errors.size() - 5) + " more)";
        return makeError(ErrorCode::ParseError, std::move(msg));
    }
    return cfg;
}

Value RobotConfigCodec::toValue(const RobotConfig& cfg) const {
    Value root = Value::makeObject();
    root["schemaVersion"] = cfg.schemaVersion;
    root["name"] = cfg.name;

    Value project = Value::makeObject();
    project["path"] = cfg.project.path;
    project["language"] = cfg.project.language;
    project["wpilibYear"] = cfg.project.wpilibYear;
    project["gradleRioVersion"] = cfg.project.gradleRioVersion;
    root["project"] = std::move(project);
    root["gamePluginId"] = cfg.gamePluginId;

    Value chassis = Value::makeObject();
    chassis["sizeM"] = vec3Value(cfg.chassis.sizeM);
    chassis["massKg"] = fv(cfg.chassis.massKg);
    chassis["comHeightM"] = fv(cfg.chassis.comHeightM);
    chassis["comOffsetM"] = vec3Value(cfg.chassis.comOffsetM);
    chassis["groundClearanceM"] = fv(cfg.chassis.groundClearanceM);
    chassis["inertiaDiagKgM2"] = vec3Value(cfg.chassis.inertiaDiagKgM2);
    chassis["material"] = materialValue(cfg.chassis.material);
    chassis["primitives"] = primitivesValue(cfg.chassis.primitives);
    root["chassis"] = std::move(chassis);

    Value dt = Value::makeObject();
    dt["kind"] = cfg.drivetrain.spec.kind;
    dt["tireModel"] = cfg.drivetrain.spec.tireModel;
    dt["loadTransferTauS"] = fv(cfg.drivetrain.spec.loadTransferTauS);
    Value wheels = Value::makeArray();
    for (const WheelSpec& w : cfg.drivetrain.spec.wheels) wheels.push_back(wheelValue(w));
    dt["wheels"] = std::move(wheels);
    dt["mappings"] = mappingsValue(cfg.drivetrain.mappings);
    root["drivetrain"] = std::move(dt);

    Value mechs = Value::makeArray();
    for (const MechanismConfig& m : cfg.mechanisms) mechs.push_back(mechanismValue(m));
    root["mechanisms"] = std::move(mechs);
    root["robotMappings"] = mappingsValue(cfg.robotMappings);
    Value zones = Value::makeArray();
    for (const ZoneConfig& z : cfg.zones) zones.push_back(zoneValue(z));
    root["zones"] = std::move(zones);
    root["vision"] = visionValue(cfg.vision);
    Value links = Value::makeArray();
    for (const auto& link : cfg.motionLinks) {
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
        links.push_back(std::move(item));
    }
    root["motionLinks"] = std::move(links);

    Value battery = Value::makeObject();
    battery["nominalVoltageV"] = cfg.battery.nominalVoltageV;
    battery["internalResistanceOhm"] = cfg.battery.internalResistanceOhm;
    battery["brownoutVoltageV"] = cfg.battery.brownoutVoltageV;
    root["battery"] = std::move(battery);

    Value sim = Value::makeObject();
    sim["stepUs"] = cfg.sim.stepUs;
    sim["seed"] = static_cast<std::int64_t>(cfg.sim.seed);
    Value physics = Value::makeObject();
    physics["workerCount"] = cfg.sim.physics.workerCount;
    physics["subSteps"] = cfg.sim.physics.subSteps;
    physics["sleeping"] = cfg.sim.physics.sleeping;
    physics["gravity"] = vec3Value(cfg.sim.physics.gravity);
    sim["physics"] = std::move(physics);
    root["sim"] = std::move(sim);

    Value plugins = Value::makeObject();
    for (const auto& [id, section] : cfg.plugins) {
        Value s = Value::makeObject();
        s["schemaVersion"] = section.schemaVersion;
        s["data"] = section.data.isObject() ? section.data : Value::makeObject();
        plugins.object().set(id, std::move(s));
    }
    root["plugins"] = std::move(plugins);

    return root;
}

}  // namespace wpiemu::core::config
