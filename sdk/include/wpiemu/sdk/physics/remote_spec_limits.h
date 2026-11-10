#pragma once

// The fairness and safety limits a match server applies to the robot spec a remote client sends: ONE table of
// plausible-FRC ranges (kRemoteSpecLimits), a check that reports every violation and a clamp that returns a corrected
// copy. A client's spec is never trusted: its mass, wheels, tires and motors decide how its robot moves in the shared
// simulation, so these ranges are also the fairness limit. The check and the clamp walk the spec with the same
// traversal, so a clamped spec passes the check by construction.
//
// Reject (do not clamp) on `too_many` / `too_few` wheel counts and on an unknown drivetrain kind: the wheel setpoints
// the client sends follow its own wheel list, so the server should only accept the list as sent.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/sdk/core/geometry.h"
#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/dynamics/motor_model.h"
#include "wpiemu/sdk/physics/chassis_spec.h"

namespace wpiemu::sdk {

// A closed range; a NaN is in no range.
struct LimitRange {
    double min = 0.0;
    double max = 0.0;
};

struct RemoteSpecLimits {
    // Chassis. FRC: 125 lb (57 kg) robot plus battery and bumpers, at most 135 lb; the ceiling leaves wide margin.
    LimitRange massKg{5.0, 100.0};
    // Frame perimeter at most 120 in (about 3 m), so a side is at most 1.5 m; a frame under 20 cm is not a robot.
    LimitRange footprintM{0.2, 1.5};
    // Starting configuration height limit is 4 ft 6 in (1.37 m).
    LimitRange heightM{0.02, 1.5};
    // Frame bottom above the floor: bumper zone and ground clearance, never more than 30 cm.
    LimitRange groundClearanceM{0.0, 0.3};
    // The COM sits above the floor and below the height limit; the load transfer model needs it positive.
    LimitRange comHeightM{0.02, 1.5};
    // The COM stays inside the frame: at most this fraction of the frame side from its centre.
    double comOffsetFractionOfSize = 0.5;
    // The COM offset z is ignored by the simulator; only a finite value is required.
    LimitRange comOffsetZM{-10.0, 10.0};
    // Explicit inertia relative to a solid box of the same mass and size: 4x either way covers hollow frames and
    // heavy top masses without letting a robot spin or resist spinning unrealistically.
    LimitRange inertiaScale{0.25, 4.0};
    // Hull material: bumper fabric and polycarbonate range up to about 1 friction, and the field's restitution is small.
    LimitRange hullFriction{0.0, 1.0};
    LimitRange hullRestitution{0.0, 0.5};
    LimitRange hullRollingResistance{0.0, 0.1};
    // A drivetrain's contact loads follow chassis acceleration with a time constant of a few tens of milliseconds.
    LimitRange loadTransferTauS{0.0, 0.25};

    // Extra primitives: frame, bumpers and a handful of mechanism blocks; the editor rarely needs more than a dozen.
    std::size_t maxPrimitives = 32;
    // Every box side, cylinder diameter or height is within the 1.5 m frame limit.
    LimitRange primitiveExtentM{0.001, 1.5};
    LimitRange primitiveRadiusM{0.001, 0.75};
    // Primitives sit within the frame envelope plus bumpers.
    LimitRange primitivePositionM{-1.0, 1.0};
    // Informational mass of one primitive, below the robot limit.
    LimitRange primitiveMassKg{0.0, 100.0};
    // Names, kinds, tire model keys and colors are short identifiers ("#rrggbbaa" is the longest color).
    std::size_t maxNameLength = 32;
    std::size_t maxColorLength = 9;
    // Rotation quaternions must be unit length to within a percent before they are renormalised.
    double quaternionNormTolerance = 0.01;

    // Wheels: two for a differential drive up to eight (a six-wheel tank or an eight-wheel drop-centre), plus margin.
    std::size_t minWheels = 2;
    std::size_t maxWheels = 8;
    // 0.75 in to 6 in radius covers every FRC wheel (1.5 in to 8 in diameter) with margin: 2 cm to 15 cm.
    LimitRange wheelRadiusM{0.02, 0.15};
    // Wheel widths from a thin omni to a wide tread: 5 mm to 15 cm.
    LimitRange wheelWidthM{0.005, 0.15};
    // Hub positions within the frame envelope plus bumpers (1 m from the COM).
    LimitRange wheelPositionM{-1.0, 1.0};
    // Hub height above the floor in the rest pose; at or below 0 means one wheel radius. A hub never sits above 30 cm.
    LimitRange wheelHubHeightM{-1.0, 0.3};
    // Wheel plus rotating hardware inertia: 0.01 g m^2 to 50 g m^2.
    LimitRange wheelInertiaKgM2{1e-5, 0.05};
    // Drive reduction motor:wheel: direct drive to a heavily geared tank gearbox. Swerve modules are 4 to 8.
    LimitRange driveGearRatio{1.0, 30.0};
    // Azimuth reduction: swerve steer stages are 12 to 25; margin to 100.
    LimitRange steerGearRatio{1.0, 100.0};
    LimitRange steerInertiaKgM2{1e-4, 0.05};
    // Motors on one wheel gearbox: FRC robots use one to four per module or side.
    LimitRange driveMotorCount{1.0, 4.0};
    LimitRange driveEfficiency{0.5, 1.0};
    // Stator current per drive motor. Unlimited (0) is refused for a remote robot: a motor controller's stator limit
    // is at most about 120 A before it browns out the robot.
    LimitRange driveCurrentLimitA{5.0, 120.0};
    // The ground speed a wheel reaches with no load (motor free speed / gear ratio * radius). FRC robots top out near
    // 5 to 6 m/s; the margin allows the fastest legal gearings.
    double maxFreeSpeedMps = 8.0;

    // Motors, as a window around every motor in the FRC catalog (Kraken X60 FOC, NEO 550, 775pro, ...).
    LimitRange motorNominalVoltageV{10.0, 14.0};
    LimitRange motorStallTorqueNm{0.1, 12.0};
    LimitRange motorStallCurrentA{10.0, 600.0};
    LimitRange motorFreeCurrentA{0.0, 10.0};
    // 100 to 2100 rad/s (about 1000 to 20000 rpm): the 775pro at 18730 rpm is the fastest catalog motor.
    LimitRange motorFreeSpeedRadPerSec{100.0, 2100.0};

    // Tires: carpet and tread give 0.3 to 1.6 friction (the default is 1.1).
    LimitRange tireMu{0.3, 1.6};
    LimitRange tireSlideRatio{0.3, 1.0};
    LimitRange tirePeakSlipLong{0.02, 0.5};
    LimitRange tirePeakSlipLat{0.02, 0.6};
    LimitRange tireVRefMps{0.05, 2.0};
    // 1 = full friction circle; rollers carry 0.12 to 0.15.
    LimitRange tireLateralScale{0.05, 1.0};
    // Mecanum rollers are at +-45 degrees (0.785 rad); a little margin, never a perpendicular axis.
    LimitRange tireTractionAngleRad{-0.8, 0.8};

    // Zone poses a client may command (clampRemoteZonePose). A zone is a pickup volume or muzzle in the chassis frame, and
    // the client may move it to fake an extending intake or a turret, so the envelope is the frame box (centred on the
    // frame, not on the COM) grown by a reach margin in x and y. 0.75 m covers the extension FRC allows beyond the frame
    // perimeter (12 in = 0.3 m) plus half of a large pickup volume and a turret's lever arm, but keeps a zone from reaching
    // across the field. Never more than this far, whatever the volume's own size.
    double zoneReachMarginM = 0.75;
    // Highest point of a zone above the FLOOR; the lowest is the floor (a zone is never below the carpet). The robot
    // height limit (heightM.max, 1.5 m) is also the highest a mechanism or muzzle can reasonably sit.
    double zoneMaxHeightAboveFloorM = 1.5;
};

inline constexpr RemoteSpecLimits kRemoteSpecLimits{};

// Stable machine-readable violation codes.
namespace remote_spec_code {
inline constexpr const char* kNonFinite = "spec.non_finite";
inline constexpr const char* kOutOfRange = "spec.out_of_range";
inline constexpr const char* kTooMany = "spec.too_many";
inline constexpr const char* kTooFew = "spec.too_few";
inline constexpr const char* kUnlimitedCurrent = "spec.unlimited_current";
inline constexpr const char* kMotorInvalid = "spec.motor_invalid";
inline constexpr const char* kBadName = "spec.bad_name";
inline constexpr const char* kDuplicateName = "spec.duplicate_name";
inline constexpr const char* kTooLong = "spec.too_long";
inline constexpr const char* kBadEnum = "spec.bad_enum";
inline constexpr const char* kSpeedTooHigh = "spec.speed_too_high";
}  // namespace remote_spec_code

struct SpecViolation {
    std::string code;      // one of remote_spec_code
    std::string path;      // the offending field, e.g. "drivetrain.wheels[2].radiusM"
    std::string message;   // for logs and rejection text
};

struct RemoteRobotSpec {
    ChassisSpec chassis;
    DrivetrainSpec drivetrain;
};

namespace remote_spec_detail {

inline constexpr std::size_t kMaxViolations = 64;

inline std::string indexed(const std::string& base, std::size_t i) { return base + "[" + std::to_string(i) + "]"; }

template <class T>
bool inRange(T v, const LimitRange& r) {
    return v >= static_cast<T>(r.min) && v <= static_cast<T>(r.max);
}

inline double finiteOr(double v, double fallback) { return std::isfinite(v) ? v : fallback; }

// Check mode: records violations, leaves the spec alone.
class Checker {
public:
    std::vector<SpecViolation> violations;

    std::size_t mark() const { return violations.size(); }

    template <class T>
    void number(const std::string& path, T& v, const LimitRange& r, T /*fallback*/, bool allowZero = false) {
        if (!std::isfinite(static_cast<double>(v))) return add(remote_spec_code::kNonFinite, path, "value must be a finite number");
        if (allowZero && v == T{0}) return;
        if (!inRange(v, r)) add(remote_spec_code::kOutOfRange, path, "value is outside the allowed range");
    }

    void currentLimit(const std::string& path, float& v) {
        if (std::isfinite(v) && v == 0.f) return add(remote_spec_code::kUnlimitedCurrent, path, "an unlimited current limit is not allowed");
        number(path, v, kRemoteSpecLimits.driveCurrentLimitA, 60.f);
    }

    void text(const std::string& path, std::string& s, std::size_t maxLength, bool allowEmpty, const std::string& /*fallback*/) {
        if (s.size() > maxLength) add(remote_spec_code::kTooLong, path, "text is too long");
        else if (s.empty() && !allowEmpty) add(remote_spec_code::kBadName, path, "text must not be empty");
    }

    template <class E>
    void enumeration(const std::string& path, E& v, int last, E /*fallback*/) {
        const int raw = static_cast<int>(v);
        if (raw < 0 || raw > last) add(remote_spec_code::kBadEnum, path, "unknown enumeration value");
    }

    template <class Vec, class Pad>
    void listSize(const std::string& path, Vec& list, std::size_t minSize, std::size_t maxSize, Pad&&) {
        if (list.size() > maxSize) add(remote_spec_code::kTooMany, path, "too many entries");
        else if (list.size() < minSize) add(remote_spec_code::kTooFew, path, "too few entries");
    }

    void rotation(const std::string& path, Quat& q) {
        const double n = std::sqrt(double{q.x} * q.x + double{q.y} * q.y + double{q.z} * q.z + double{q.w} * q.w);
        if (!std::isfinite(n)) return add(remote_spec_code::kNonFinite, path, "rotation must be finite");
        if (std::fabs(n - 1.0) > kRemoteSpecLimits.quaternionNormTolerance)
            add(remote_spec_code::kOutOfRange, path, "rotation must be a unit quaternion");
    }

    // `before` is mark() taken before the fields of the motor were checked: field violations already explain it.
    void motorFaults(const std::string& path, MotorSpec& m, const MotorSpec& /*fallback*/, std::size_t before) {
        if (violations.size() != before) return;
        for (const MotorSpecFault& fault : motorSpecFaults(m)) add(remote_spec_code::kMotorInvalid, path + "." + fault.field, fault.why);
    }

    void uniqueNames(const std::string& path, std::vector<WheelSpec>& wheels, std::size_t count) {
        for (std::size_t i = 1; i < count; ++i)
            for (std::size_t j = 0; j < i; ++j)
                if (wheels[i].name == wheels[j].name) {
                    add(remote_spec_code::kDuplicateName, indexed(path, i) + ".name", "wheel names must be unique");
                    break;
                }
    }

    void freeSpeed(const std::string& path, WheelSpec& w) {
        const double gear = w.driveGearRatio, free = w.driveMotor.freeSpeedRadPerSec, r = w.radiusM;
        if (gear > 0.0 && free > 0.0 && r > 0.0 && std::isfinite(gear) && std::isfinite(free) && std::isfinite(r) &&
            free / gear * r > kRemoteSpecLimits.maxFreeSpeedMps)
            add(remote_spec_code::kSpeedTooHigh, path + ".driveGearRatio", "free-speed ground speed of the wheel is too high");
    }

private:
    void add(const char* code, const std::string& path, std::string message) {
        if (violations.size() < kMaxViolations) violations.push_back({code, path, std::move(message)});
    }
};

// Clamp mode: corrects the spec in place; the paths and messages are not needed.
class Clamper {
public:
    std::size_t mark() const { return 0; }

    template <class T>
    void number(const std::string&, T& v, const LimitRange& r, T fallback, bool allowZero = false) {
        if (!std::isfinite(static_cast<double>(v))) {
            if (allowZero) {
                v = T{0};
                return;
            }
            v = fallback;
        } else if (allowZero && v == T{0}) {
            return;
        }
        v = std::clamp(v, static_cast<T>(r.min), static_cast<T>(r.max));
    }

    void currentLimit(const std::string& path, float& v) { number(path, v, kRemoteSpecLimits.driveCurrentLimitA, 60.f); }

    void text(const std::string&, std::string& s, std::size_t maxLength, bool allowEmpty, const std::string& fallback) {
        if (s.size() > maxLength) s.resize(maxLength);
        if (s.empty() && !allowEmpty) s = fallback;
    }

    template <class E>
    void enumeration(const std::string&, E& v, int last, E fallback) {
        const int raw = static_cast<int>(v);
        if (raw < 0 || raw > last) v = fallback;
    }

    template <class Vec, class Pad>
    void listSize(const std::string&, Vec& list, std::size_t minSize, std::size_t maxSize, Pad&& pad) {
        if (list.size() > maxSize) list.resize(maxSize);
        while (list.size() < minSize) pad(list);
    }

    void rotation(const std::string&, Quat& q) {
        const double n = std::sqrt(double{q.x} * q.x + double{q.y} * q.y + double{q.z} * q.z + double{q.w} * q.w);
        if (!std::isfinite(n) || n == 0.0) q = Quat::identity();
        else if (std::fabs(n - 1.0) > kRemoteSpecLimits.quaternionNormTolerance) q = q.normalized();
    }

    void motorFaults(const std::string&, MotorSpec& m, const MotorSpec& fallback, std::size_t) {
        if (!motorSpecFaults(m).empty()) m = fallback;
    }

    void uniqueNames(const std::string&, std::vector<WheelSpec>& wheels, std::size_t count) {
        for (std::size_t i = 0; i < count; ++i) {
            std::string unique = wheels[i].name;
            const auto taken = [&](const std::string& n) {
                for (std::size_t j = 0; j < i; ++j)
                    if (wheels[j].name == n) return true;
                return false;
            };
            if (taken(unique)) unique = "w" + std::to_string(i);
            while (taken(unique)) unique += "x";
            wheels[i].name = std::move(unique);
        }
    }

    // Prefers a higher gear ratio and then a slower motor, both inside their own ranges, with 0.1 % margin.
    void freeSpeed(const std::string&, WheelSpec& w) {
        const double maxSpeed = kRemoteSpecLimits.maxFreeSpeedMps * 0.999;
        const double r = w.radiusM;
        if (w.driveMotor.freeSpeedRadPerSec / w.driveGearRatio * r <= kRemoteSpecLimits.maxFreeSpeedMps) return;
        w.driveGearRatio = static_cast<float>(
            std::min(kRemoteSpecLimits.driveGearRatio.max, w.driveMotor.freeSpeedRadPerSec * r / maxSpeed));
        if (w.driveMotor.freeSpeedRadPerSec / w.driveGearRatio * r > kRemoteSpecLimits.maxFreeSpeedMps)
            w.driveMotor.freeSpeedRadPerSec = maxSpeed * w.driveGearRatio / r;
    }
};

template <class V>
void visitMotor(V& v, const std::string& path, MotorSpec& m, const MotorSpec& fallback) {
    const RemoteSpecLimits& L = kRemoteSpecLimits;
    const std::size_t before = v.mark();
    v.text(path + ".name", m.name, L.maxNameLength, true, fallback.name);
    v.number(path + ".nominalVoltageV", m.nominalVoltageV, L.motorNominalVoltageV, fallback.nominalVoltageV);
    v.number(path + ".stallTorqueNm", m.stallTorqueNm, L.motorStallTorqueNm, fallback.stallTorqueNm);
    v.number(path + ".stallCurrentA", m.stallCurrentA, L.motorStallCurrentA, fallback.stallCurrentA);
    v.number(path + ".freeCurrentA", m.freeCurrentA, L.motorFreeCurrentA, fallback.freeCurrentA);
    v.number(path + ".freeSpeedRadPerSec", m.freeSpeedRadPerSec, L.motorFreeSpeedRadPerSec, fallback.freeSpeedRadPerSec);
    v.motorFaults(path, m, fallback, before);
}

template <class V>
void visitWheel(V& v, const std::string& p, WheelSpec& w, const WheelSpec& def) {
    const RemoteSpecLimits& L = kRemoteSpecLimits;
    v.text(p + ".name", w.name, L.maxNameLength, false, "wheel");
    v.enumeration(p + ".kind", w.kind, static_cast<int>(WheelKind::Swerve), def.kind);
    v.number(p + ".positionM.x", w.positionM.x, L.wheelPositionM, def.positionM.x);
    v.number(p + ".positionM.y", w.positionM.y, L.wheelPositionM, def.positionM.y);
    v.number(p + ".positionM.z", w.positionM.z, L.wheelHubHeightM, def.positionM.z);
    v.number(p + ".radiusM", w.radiusM, L.wheelRadiusM, def.radiusM);
    v.number(p + ".widthM", w.widthM, L.wheelWidthM, def.widthM);
    v.number(p + ".inertiaKgM2", w.inertiaKgM2, L.wheelInertiaKgM2, def.inertiaKgM2);
    v.number(p + ".tire.mu", w.tire.mu, L.tireMu, def.tire.mu);
    v.number(p + ".tire.slideRatio", w.tire.slideRatio, L.tireSlideRatio, def.tire.slideRatio);
    v.number(p + ".tire.peakSlipLong", w.tire.peakSlipLong, L.tirePeakSlipLong, def.tire.peakSlipLong);
    v.number(p + ".tire.peakSlipLat", w.tire.peakSlipLat, L.tirePeakSlipLat, def.tire.peakSlipLat);
    v.number(p + ".tire.vRefMps", w.tire.vRefMps, L.tireVRefMps, def.tire.vRefMps);
    v.number(p + ".tire.lateralScale", w.tire.lateralScale, L.tireLateralScale, def.tire.lateralScale);
    v.number(p + ".tire.tractionAngleRad", w.tire.tractionAngleRad, L.tireTractionAngleRad, def.tire.tractionAngleRad);
    visitMotor(v, p + ".driveMotor", w.driveMotor, def.driveMotor);
    v.number(p + ".driveMotorCount", w.driveMotorCount, L.driveMotorCount, def.driveMotorCount);
    v.number(p + ".driveGearRatio", w.driveGearRatio, L.driveGearRatio, def.driveGearRatio);
    v.number(p + ".driveEfficiency", w.driveEfficiency, L.driveEfficiency, def.driveEfficiency);
    v.currentLimit(p + ".driveCurrentLimitA", w.driveCurrentLimitA);
    visitMotor(v, p + ".steerMotor", w.steerMotor, def.steerMotor);
    v.number(p + ".steerGearRatio", w.steerGearRatio, L.steerGearRatio, def.steerGearRatio);
    v.number(p + ".steerInertiaKgM2", w.steerInertiaKgM2, L.steerInertiaKgM2, def.steerInertiaKgM2);
    v.freeSpeed(p, w);
}

template <class V>
void visitPrimitive(V& v, const std::string& p, PrimitiveSpec& prim, const PrimitiveSpec& def) {
    const RemoteSpecLimits& L = kRemoteSpecLimits;
    v.text(p + ".name", prim.name, L.maxNameLength, true, def.name);
    v.enumeration(p + ".kind", prim.kind, static_cast<int>(PrimitiveKind::Cylinder), def.kind);
    v.number(p + ".local.position.x", prim.local.position.x, L.primitivePositionM, 0.f);
    v.number(p + ".local.position.y", prim.local.position.y, L.primitivePositionM, 0.f);
    v.number(p + ".local.position.z", prim.local.position.z, L.primitivePositionM, 0.f);
    v.rotation(p + ".local.rotation", prim.local.rotation);
    v.number(p + ".sizeM.x", prim.sizeM.x, L.primitiveExtentM, def.sizeM.x);
    v.number(p + ".sizeM.y", prim.sizeM.y, L.primitiveExtentM, def.sizeM.y);
    v.number(p + ".sizeM.z", prim.sizeM.z, L.primitiveExtentM, def.sizeM.z);
    v.number(p + ".radiusM", prim.radiusM, L.primitiveRadiusM, def.radiusM);
    v.number(p + ".heightM", prim.heightM, L.primitiveExtentM, def.heightM);
    v.text(p + ".color", prim.color, L.maxColorLength, true, def.color);
    v.number(p + ".massKg", prim.massKg, L.primitiveMassKg, 0.f);
}

// The single traversal of the spec shared by the check and the clamp.
template <class V>
void visitRemoteSpec(V& v, ChassisSpec& c, DrivetrainSpec& d) {
    const RemoteSpecLimits& L = kRemoteSpecLimits;
    const ChassisSpec defC;
    const DrivetrainSpec defD;
    const WheelSpec defW;
    const PrimitiveSpec defP;

    v.number("chassis.massKg", c.massKg, L.massKg, defC.massKg);
    v.number("chassis.sizeM.x", c.sizeM.x, L.footprintM, defC.sizeM.x);
    v.number("chassis.sizeM.y", c.sizeM.y, L.footprintM, defC.sizeM.y);
    v.number("chassis.sizeM.z", c.sizeM.z, L.heightM, defC.sizeM.z);
    v.number("chassis.groundClearanceM", c.groundClearanceM, L.groundClearanceM, defC.groundClearanceM);
    v.number("chassis.comHeightM", c.comHeightM, L.comHeightM, defC.comHeightM);

    const double sx = finiteOr(c.sizeM.x, defC.sizeM.x), sy = finiteOr(c.sizeM.y, defC.sizeM.y);
    const double sz = finiteOr(c.sizeM.z, defC.sizeM.z), mass = finiteOr(c.massKg, defC.massKg);
    const double hx = L.comOffsetFractionOfSize * sx, hy = L.comOffsetFractionOfSize * sy;
    v.number("chassis.comOffsetM.x", c.comOffsetM.x, LimitRange{-hx, hx}, 0.f);
    v.number("chassis.comOffsetM.y", c.comOffsetM.y, LimitRange{-hy, hy}, 0.f);
    v.number("chassis.comOffsetM.z", c.comOffsetM.z, L.comOffsetZM, 0.f);

    // Zero means "derive from the box"; anything else must stay near the solid-box value.
    const double boxX = mass * (sy * sy + sz * sz) / 12.0, boxY = mass * (sx * sx + sz * sz) / 12.0,
                 boxZ = mass * (sx * sx + sy * sy) / 12.0;
    const auto scaled = [&L](double box) { return LimitRange{L.inertiaScale.min * box, L.inertiaScale.max * box}; };
    v.number("chassis.inertiaDiagKgM2.x", c.inertiaDiagKgM2.x, scaled(boxX), 0.f, true);
    v.number("chassis.inertiaDiagKgM2.y", c.inertiaDiagKgM2.y, scaled(boxY), 0.f, true);
    v.number("chassis.inertiaDiagKgM2.z", c.inertiaDiagKgM2.z, scaled(boxZ), 0.f, true);

    v.number("chassis.material.friction", c.material.friction, L.hullFriction, defC.material.friction);
    v.number("chassis.material.restitution", c.material.restitution, L.hullRestitution, defC.material.restitution);
    v.number("chassis.material.rollingResistance", c.material.rollingResistance, L.hullRollingResistance,
             defC.material.rollingResistance);

    v.listSize("chassis.primitives", c.primitives, 0, L.maxPrimitives, [](std::vector<PrimitiveSpec>&) {});
    for (std::size_t i = 0; i < std::min(c.primitives.size(), L.maxPrimitives); ++i)
        visitPrimitive(v, indexed("chassis.primitives", i), c.primitives[i], defP);

    v.text("drivetrain.kind", d.kind, L.maxNameLength, false, defD.kind);
    v.text("drivetrain.tireModel", d.tireModel, L.maxNameLength, false, defD.tireModel);
    v.number("drivetrain.loadTransferTauS", d.loadTransferTauS, L.loadTransferTauS, defD.loadTransferTauS);

    v.listSize("drivetrain.wheels", d.wheels, L.minWheels, L.maxWheels, [&defW](std::vector<WheelSpec>& list) {
        WheelSpec w = list.empty() ? defW : list.back();
        w.name = "w" + std::to_string(list.size());
        w.positionM.x = list.empty() ? 0.25f : w.positionM.x;
        w.positionM.y = list.empty() ? 0.25f : -w.positionM.y;
        list.push_back(std::move(w));
    });
    const std::size_t wheelCount = std::min(d.wheels.size(), L.maxWheels);
    for (std::size_t i = 0; i < wheelCount; ++i) visitWheel(v, indexed("drivetrain.wheels", i), d.wheels[i], defW);
    v.uniqueNames("drivetrain.wheels", d.wheels, wheelCount);
}

}  // namespace remote_spec_detail

// Every way the spec is outside kRemoteSpecLimits (at most 64 violations), empty when it is acceptable. Does not know the
// drivetrain registry: the caller still refuses a kind it cannot build.
inline std::vector<SpecViolation> checkRemoteSpec(const ChassisSpec& chassis, const DrivetrainSpec& drivetrain) {
    ChassisSpec c = chassis;
    DrivetrainSpec d = drivetrain;
    remote_spec_detail::Checker checker;
    remote_spec_detail::visitRemoteSpec(checker, c, d);
    return std::move(checker.violations);
}

// A copy of the spec inside kRemoteSpecLimits: non-finite numbers take their default, out-of-range ones the nearest
// bound, lists are cut to their cap (and a robot with fewer than two wheels gets mirrored copies of its last wheel),
// an unlimited current limit becomes the cap, a non-physical motor becomes the catalog default, names are made unique
// and a wheel that would exceed the free ground speed limit gets a higher gear ratio. checkRemoteSpec of the result is
// always empty.
inline RemoteRobotSpec clampRemoteSpec(const ChassisSpec& chassis, const DrivetrainSpec& drivetrain) {
    RemoteRobotSpec out{chassis, drivetrain};
    remote_spec_detail::Clamper clamper;
    remote_spec_detail::visitRemoteSpec(clamper, out.chassis, out.drivetrain);
    return out;
}

// A zone pose after the envelope of kRemoteSpecLimits (zoneReachMarginM, zoneMaxHeightAboveFloorM).
struct ClampedZonePose {
    Pose3 pose;
    bool changed = false;     // the position was moved into the envelope, or a part of the pose was unusable
    bool nonFinite = false;   // a coordinate or the rotation was NaN/inf: the caller should ignore the override
};

// The chassis-frame box a zone of this robot may be in: the frame box (centre at -comOffset in x and y, since the chassis
// frame's origin is the COM) grown by the reach margin in x and y; z from the floor (-comHeightM) up to
// zoneMaxHeightAboveFloorM above it. Orientation is not limited.
struct ZoneEnvelope {
    Vec3 min;
    Vec3 max;
};
inline ZoneEnvelope remoteZoneEnvelope(const ChassisSpec& chassis) {
    const RemoteSpecLimits& L = kRemoteSpecLimits;
    const float reachX = 0.5f * chassis.sizeM.x + static_cast<float>(L.zoneReachMarginM);
    const float reachY = 0.5f * chassis.sizeM.y + static_cast<float>(L.zoneReachMarginM);
    return {Vec3{-chassis.comOffsetM.x - reachX, -chassis.comOffsetM.y - reachY, -chassis.comHeightM},
            Vec3{-chassis.comOffsetM.x + reachX, -chassis.comOffsetM.y + reachY,
                 static_cast<float>(L.zoneMaxHeightAboveFloorM) - chassis.comHeightM}};
}

// Clamps `pose` (chassis frame) into remoteZoneEnvelope(chassis), per axis; the rotation is normalised, never limited. A
// non-finite position coordinate or rotation (or a zero quaternion) is replaced by the envelope centre / the identity and
// flagged (`nonFinite` for NaN and infinities, `changed` for every replacement); the pose is then not what the client sent
// and callers should treat the override as absent. `changed` is false for a pose already inside the envelope, even if its
// quaternion needed renormalising.
inline ClampedZonePose clampRemoteZonePose(const ChassisSpec& chassis, const Pose3& pose) {
    const ZoneEnvelope box = remoteZoneEnvelope(chassis);
    ClampedZonePose out;
    const auto axis = [&out](float v, float lo, float hi) {
        if (!std::isfinite(v)) {
            out.changed = out.nonFinite = true;
            return 0.5f * (lo + hi);
        }
        const float c = std::clamp(v, lo, hi);
        if (c != v) out.changed = true;
        return c;
    };
    out.pose.position = {axis(pose.position.x, box.min.x, box.max.x), axis(pose.position.y, box.min.y, box.max.y),
                         axis(pose.position.z, box.min.z, box.max.z)};
    const Quat& q = pose.rotation;
    const float norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (!finite(q)) {
        out.changed = out.nonFinite = true;
    } else if (!(norm > 0.f) || !std::isfinite(norm)) {
        out.changed = true;
    } else {
        out.pose.rotation = q.normalized();
    }
    return out;
}

}  // namespace wpiemu::sdk
