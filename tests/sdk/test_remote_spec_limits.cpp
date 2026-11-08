#include <doctest/doctest.h>

#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <string>
#include <type_traits>
#include <vector>

#include "wpiemu/sdk/physics/remote_spec_limits.h"
#include "wpiemu/sdk/physics/wheel_layout.h"

using namespace wpiemu::sdk;

namespace {

struct Spec {
    ChassisSpec c;
    DrivetrainSpec d;
};

Spec baseline() {
    Spec s;
    PrimitiveSpec frame;
    frame.name = "frame";
    frame.collides = true;
    s.c.primitives.push_back(frame);
    s.d.kind = drivetrain_kind::kSwerve;
    s.d.wheels = swerveWheels(WheelSpec{}, 0.25f, 0.25f);
    return s;
}

const SpecViolation* find(const std::vector<SpecViolation>& vs, const std::string& path, const char* code = nullptr) {
    for (const SpecViolation& v : vs)
        if (v.path == path && (code == nullptr || v.code == code)) return &v;
    return nullptr;
}

std::vector<SpecViolation> check(const Spec& s) { return checkRemoteSpec(s.c, s.d); }

struct Case {
    std::string path;
    std::function<void(Spec&, double)> set;
    double lo;
    double hi;
    bool isInt = false;
    bool isDouble = false;
};

#define FIELD(expr) [](Spec& s, double v) { (expr) = static_cast<std::remove_reference_t<decltype(expr)>>(v); }

std::vector<Case> cases() {
    const RemoteSpecLimits& L = kRemoteSpecLimits;
    const double halfSize = L.comOffsetFractionOfSize * static_cast<double>(0.7f);
    std::vector<Case> out;
    const auto add = [&](std::string path, std::function<void(Spec&, double)> set, const LimitRange& r, bool isInt = false,
                         bool isDouble = false) { out.push_back({std::move(path), std::move(set), r.min, r.max, isInt, isDouble}); };
    add("chassis.massKg", FIELD(s.c.massKg), L.massKg);
    add("chassis.sizeM.x", FIELD(s.c.sizeM.x), L.footprintM);
    add("chassis.sizeM.y", FIELD(s.c.sizeM.y), L.footprintM);
    add("chassis.sizeM.z", FIELD(s.c.sizeM.z), L.heightM);
    add("chassis.groundClearanceM", FIELD(s.c.groundClearanceM), L.groundClearanceM);
    add("chassis.comHeightM", FIELD(s.c.comHeightM), L.comHeightM);
    add("chassis.comOffsetM.x", FIELD(s.c.comOffsetM.x), {-halfSize, halfSize});
    add("chassis.comOffsetM.y", FIELD(s.c.comOffsetM.y), {-halfSize, halfSize});
    add("chassis.comOffsetM.z", FIELD(s.c.comOffsetM.z), L.comOffsetZM);
    add("chassis.material.friction", FIELD(s.c.material.friction), L.hullFriction);
    add("chassis.material.restitution", FIELD(s.c.material.restitution), L.hullRestitution);
    add("chassis.material.rollingResistance", FIELD(s.c.material.rollingResistance), L.hullRollingResistance);
    add("drivetrain.loadTransferTauS", FIELD(s.d.loadTransferTauS), L.loadTransferTauS);

    add("chassis.primitives[0].sizeM.x", FIELD(s.c.primitives[0].sizeM.x), L.primitiveExtentM);
    add("chassis.primitives[0].sizeM.y", FIELD(s.c.primitives[0].sizeM.y), L.primitiveExtentM);
    add("chassis.primitives[0].sizeM.z", FIELD(s.c.primitives[0].sizeM.z), L.primitiveExtentM);
    add("chassis.primitives[0].radiusM", FIELD(s.c.primitives[0].radiusM), L.primitiveRadiusM);
    add("chassis.primitives[0].heightM", FIELD(s.c.primitives[0].heightM), L.primitiveExtentM);
    add("chassis.primitives[0].local.position.x", FIELD(s.c.primitives[0].local.position.x), L.primitivePositionM);
    add("chassis.primitives[0].local.position.y", FIELD(s.c.primitives[0].local.position.y), L.primitivePositionM);
    add("chassis.primitives[0].local.position.z", FIELD(s.c.primitives[0].local.position.z), L.primitivePositionM);
    add("chassis.primitives[0].massKg", FIELD(s.c.primitives[0].massKg), L.primitiveMassKg);

    add("drivetrain.wheels[1].positionM.x", FIELD(s.d.wheels[1].positionM.x), L.wheelPositionM);
    add("drivetrain.wheels[1].positionM.y", FIELD(s.d.wheels[1].positionM.y), L.wheelPositionM);
    add("drivetrain.wheels[1].positionM.z", FIELD(s.d.wheels[1].positionM.z), L.wheelHubHeightM);
    add("drivetrain.wheels[1].radiusM", FIELD(s.d.wheels[1].radiusM), L.wheelRadiusM);
    add("drivetrain.wheels[1].widthM", FIELD(s.d.wheels[1].widthM), L.wheelWidthM);
    add("drivetrain.wheels[1].inertiaKgM2", FIELD(s.d.wheels[1].inertiaKgM2), L.wheelInertiaKgM2);
    // A slow motor keeps the ground speed limit out of the way of the lowest gear ratio.
    add("drivetrain.wheels[1].driveGearRatio",
        [](Spec& s, double v) {
            s.d.wheels[1].driveMotor.freeSpeedRadPerSec = kRemoteSpecLimits.motorFreeSpeedRadPerSec.min;
            s.d.wheels[1].driveGearRatio = static_cast<float>(v);
        },
        L.driveGearRatio);
    add("drivetrain.wheels[1].steerGearRatio", FIELD(s.d.wheels[1].steerGearRatio), L.steerGearRatio);
    add("drivetrain.wheels[1].steerInertiaKgM2", FIELD(s.d.wheels[1].steerInertiaKgM2), L.steerInertiaKgM2);
    add("drivetrain.wheels[1].driveMotorCount", FIELD(s.d.wheels[1].driveMotorCount), L.driveMotorCount, true);
    add("drivetrain.wheels[1].driveEfficiency", FIELD(s.d.wheels[1].driveEfficiency), L.driveEfficiency);
    add("drivetrain.wheels[1].driveCurrentLimitA", FIELD(s.d.wheels[1].driveCurrentLimitA), L.driveCurrentLimitA);
    add("drivetrain.wheels[1].tire.mu", FIELD(s.d.wheels[1].tire.mu), L.tireMu);
    add("drivetrain.wheels[1].tire.slideRatio", FIELD(s.d.wheels[1].tire.slideRatio), L.tireSlideRatio);
    add("drivetrain.wheels[1].tire.peakSlipLong", FIELD(s.d.wheels[1].tire.peakSlipLong), L.tirePeakSlipLong);
    add("drivetrain.wheels[1].tire.peakSlipLat", FIELD(s.d.wheels[1].tire.peakSlipLat), L.tirePeakSlipLat);
    add("drivetrain.wheels[1].tire.vRefMps", FIELD(s.d.wheels[1].tire.vRefMps), L.tireVRefMps);
    add("drivetrain.wheels[1].tire.lateralScale", FIELD(s.d.wheels[1].tire.lateralScale), L.tireLateralScale);
    add("drivetrain.wheels[1].tire.tractionAngleRad", FIELD(s.d.wheels[1].tire.tractionAngleRad), L.tireTractionAngleRad);

    add("drivetrain.wheels[1].driveMotor.nominalVoltageV", FIELD(s.d.wheels[1].driveMotor.nominalVoltageV),
        L.motorNominalVoltageV, false, true);
    add("drivetrain.wheels[1].driveMotor.stallTorqueNm", FIELD(s.d.wheels[1].driveMotor.stallTorqueNm), L.motorStallTorqueNm,
        false, true);
    add("drivetrain.wheels[1].driveMotor.stallCurrentA", FIELD(s.d.wheels[1].driveMotor.stallCurrentA), L.motorStallCurrentA,
        false, true);
    add("drivetrain.wheels[1].driveMotor.freeCurrentA", FIELD(s.d.wheels[1].driveMotor.freeCurrentA), L.motorFreeCurrentA,
        false, true);
    add("drivetrain.wheels[1].driveMotor.freeSpeedRadPerSec", FIELD(s.d.wheels[1].driveMotor.freeSpeedRadPerSec),
        L.motorFreeSpeedRadPerSec, false, true);
    add("drivetrain.wheels[1].steerMotor.stallTorqueNm", FIELD(s.d.wheels[1].steerMotor.stallTorqueNm), L.motorStallTorqueNm,
        false, true);
    add("drivetrain.wheels[1].steerMotor.freeSpeedRadPerSec", FIELD(s.d.wheels[1].steerMotor.freeSpeedRadPerSec),
        L.motorFreeSpeedRadPerSec, false, true);
    return out;
}

double outsideLow(const Case& c) {
    if (c.isInt) return c.lo - 1.0;
    if (c.isDouble) return std::nextafter(c.lo, -std::numeric_limits<double>::infinity());
    return static_cast<double>(std::nextafter(static_cast<float>(c.lo), -std::numeric_limits<float>::infinity()));
}

double outsideHigh(const Case& c) {
    if (c.isInt) return c.hi + 1.0;
    if (c.isDouble) return std::nextafter(c.hi, std::numeric_limits<double>::infinity());
    return static_cast<double>(std::nextafter(static_cast<float>(c.hi), std::numeric_limits<float>::infinity()));
}

// Values a float or double field can hold but an integer field cannot.
std::vector<double> nonFiniteValues(const Case& c) {
    if (c.isInt) return {};
    return {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity()};
}

}  // namespace

TEST_CASE("remote spec limits: the baseline spec is accepted") {
    const Spec s = baseline();
    CHECK(check(s).empty());
    CHECK(checkRemoteSpec(ChassisSpec{}, DrivetrainSpec{}).size() == 1);   // the default drivetrain has no wheels
}

TEST_CASE("remote spec limits: every numeric bound is accepted at the edge and rejected just outside") {
    for (const Case& c : cases()) {
        INFO("field " << c.path);
        for (const double edge : {c.lo, c.hi}) {
            Spec s = baseline();
            c.set(s, edge);
            CHECK(find(check(s), c.path) == nullptr);
        }
        for (const double outside : {outsideLow(c), outsideHigh(c)}) {
            Spec s = baseline();
            c.set(s, outside);
            CHECK(find(check(s), c.path, remote_spec_code::kOutOfRange) != nullptr);
        }
    }
}

TEST_CASE("remote spec limits: NaN and infinity are rejected in every numeric field") {
    for (const Case& c : cases()) {
        INFO("field " << c.path);
        for (const double bad : nonFiniteValues(c)) {
            Spec s = baseline();
            c.set(s, bad);
            CHECK(find(check(s), c.path, remote_spec_code::kNonFinite) != nullptr);
        }
    }
}

TEST_CASE("remote spec limits: clamping always yields a spec that passes the check") {
    for (const Case& c : cases()) {
        INFO("field " << c.path);
        std::vector<double> values{outsideLow(c), outsideHigh(c)};
        for (const double v : nonFiniteValues(c)) values.push_back(v);
        if (!c.isInt) {
            values.push_back(1e30);
            values.push_back(-1e30);
        }
        for (const double v : values) {
            Spec s = baseline();
            c.set(s, v);
            const RemoteRobotSpec fixed = clampRemoteSpec(s.c, s.d);
            const auto vs = checkRemoteSpec(fixed.chassis, fixed.drivetrain);
            CHECK(vs.empty());
            if (!vs.empty()) CHECK(vs.front().path == "");   // names the offender in the failure output
        }
    }
}

TEST_CASE("remote spec limits: clamping leaves an acceptable spec unchanged") {
    const Spec s = baseline();
    const RemoteRobotSpec fixed = clampRemoteSpec(s.c, s.d);
    CHECK(fixed.chassis.massKg == s.c.massKg);
    CHECK(fixed.chassis.sizeM == s.c.sizeM);
    REQUIRE(fixed.drivetrain.wheels.size() == s.d.wheels.size());
    for (std::size_t i = 0; i < s.d.wheels.size(); ++i) {
        CHECK(fixed.drivetrain.wheels[i].name == s.d.wheels[i].name);
        CHECK(fixed.drivetrain.wheels[i].radiusM == s.d.wheels[i].radiusM);
        CHECK(fixed.drivetrain.wheels[i].driveGearRatio == s.d.wheels[i].driveGearRatio);
        CHECK(fixed.drivetrain.wheels[i].driveMotor.name == s.d.wheels[i].driveMotor.name);
    }
    CHECK(fixed.chassis.primitives.size() == 1);
}

TEST_CASE("remote spec limits: clamping moves values to the nearest bound") {
    Spec s = baseline();
    s.c.massKg = 500.f;
    s.c.sizeM.x = 0.01f;
    s.d.wheels[0].radiusM = std::numeric_limits<float>::quiet_NaN();
    const RemoteRobotSpec fixed = clampRemoteSpec(s.c, s.d);
    CHECK(fixed.chassis.massKg == static_cast<float>(kRemoteSpecLimits.massKg.max));
    CHECK(fixed.chassis.sizeM.x == static_cast<float>(kRemoteSpecLimits.footprintM.min));
    CHECK(fixed.drivetrain.wheels[0].radiusM == WheelSpec{}.radiusM);
}

TEST_CASE("remote spec limits: an unlimited drive current is refused and capped") {
    Spec s = baseline();
    s.d.wheels[2].driveCurrentLimitA = 0.f;
    const auto vs = check(s);
    CHECK(find(vs, "drivetrain.wheels[2].driveCurrentLimitA", remote_spec_code::kUnlimitedCurrent) != nullptr);
    const RemoteRobotSpec fixed = clampRemoteSpec(s.c, s.d);
    CHECK(fixed.drivetrain.wheels[2].driveCurrentLimitA == static_cast<float>(kRemoteSpecLimits.driveCurrentLimitA.min));
    s.d.wheels[2].driveCurrentLimitA = -5.f;
    CHECK(find(check(s), "drivetrain.wheels[2].driveCurrentLimitA", remote_spec_code::kOutOfRange) != nullptr);
}

TEST_CASE("remote spec limits: the wheel count is limited to 2 through 8") {
    for (const std::size_t count : {std::size_t{2}, std::size_t{8}}) {
        Spec s = baseline();
        s.d.wheels.clear();
        for (std::size_t i = 0; i < count; ++i) {
            WheelSpec w;
            w.name = "w" + std::to_string(i);
            w.positionM = {0.1f * static_cast<float>(i), 0.2f, 0.f};
            s.d.wheels.push_back(w);
        }
        CHECK(check(s).empty());
    }
    Spec none = baseline();
    none.d.wheels.resize(1);
    CHECK(find(check(none), "drivetrain.wheels", remote_spec_code::kTooFew) != nullptr);

    Spec many = baseline();
    for (int i = 0; i < 6; ++i) {
        many.d.wheels.push_back(many.d.wheels[0]);
        many.d.wheels.back().name = "extra" + std::to_string(i);
    }
    REQUIRE(many.d.wheels.size() == 10);
    CHECK(find(check(many), "drivetrain.wheels", remote_spec_code::kTooMany) != nullptr);
    const RemoteRobotSpec fixed = clampRemoteSpec(many.c, many.d);
    CHECK(fixed.drivetrain.wheels.size() == kRemoteSpecLimits.maxWheels);

    Spec empty = baseline();
    empty.d.wheels.clear();
    const RemoteRobotSpec padded = clampRemoteSpec(empty.c, empty.d);
    CHECK(padded.drivetrain.wheels.size() == kRemoteSpecLimits.minWheels);
    CHECK(checkRemoteSpec(padded.chassis, padded.drivetrain).empty());

    Spec single = baseline();
    single.d.wheels.resize(1);
    const RemoteRobotSpec mirrored = clampRemoteSpec(single.c, single.d);
    REQUIRE(mirrored.drivetrain.wheels.size() == 2);
    CHECK(mirrored.drivetrain.wheels[1].positionM.y == -mirrored.drivetrain.wheels[0].positionM.y);
    CHECK(checkRemoteSpec(mirrored.chassis, mirrored.drivetrain).empty());
}

TEST_CASE("remote spec limits: the primitive count is capped") {
    Spec s = baseline();
    s.c.primitives.assign(kRemoteSpecLimits.maxPrimitives, PrimitiveSpec{});
    CHECK(check(s).empty());
    s.c.primitives.push_back(PrimitiveSpec{});
    CHECK(find(check(s), "chassis.primitives", remote_spec_code::kTooMany) != nullptr);
    CHECK(clampRemoteSpec(s.c, s.d).chassis.primitives.size() == kRemoteSpecLimits.maxPrimitives);
}

TEST_CASE("remote spec limits: motor sanity reuses the shared motor spec faults") {
    Spec s = baseline();
    auto& m = s.d.wheels[0].driveMotor;
    m.stallCurrentA = 10.0;
    m.freeCurrentA = 10.0;   // both inside their ranges, but the free current must stay below the stall current
    REQUIRE_FALSE(motorSpecFaults(m).empty());
    const auto vs = check(s);
    CHECK(find(vs, "drivetrain.wheels[0].driveMotor.freeCurrentA", remote_spec_code::kMotorInvalid) != nullptr);

    const RemoteRobotSpec fixed = clampRemoteSpec(s.c, s.d);
    CHECK(isValidMotorSpec(fixed.drivetrain.wheels[0].driveMotor));
    CHECK(fixed.drivetrain.wheels[0].driveMotor.name == WheelSpec{}.driveMotor.name);
    CHECK(checkRemoteSpec(fixed.chassis, fixed.drivetrain).empty());

    // A motor with an absurd torque is outside the fairness window even though it is a valid motor.
    Spec strong = baseline();
    strong.d.wheels[0].driveMotor.stallTorqueNm = 500.0;
    CHECK(isValidMotorSpec(strong.d.wheels[0].driveMotor));
    CHECK(find(check(strong), "drivetrain.wheels[0].driveMotor.stallTorqueNm", remote_spec_code::kOutOfRange) != nullptr);
}

TEST_CASE("remote spec limits: the catalog motors fit the motor window except the Romi motor") {
    for (const MotorPreset& preset : motors::presets()) {
        Spec s = baseline();
        s.d.wheels[0].driveMotor = preset.spec;
        s.d.wheels[0].driveGearRatio = 30.f;   // keep the ground speed limit out of the way
        const auto vs = check(s);
        INFO("motor " << preset.key);
        if (preset.key == "romi_built_in") {
            CHECK_FALSE(vs.empty());
        } else {
            CHECK(find(vs, "drivetrain.wheels[0].driveMotor.stallTorqueNm") == nullptr);
            CHECK(find(vs, "drivetrain.wheels[0].driveMotor.stallCurrentA") == nullptr);
            CHECK(find(vs, "drivetrain.wheels[0].driveMotor.freeCurrentA") == nullptr);
            CHECK(find(vs, "drivetrain.wheels[0].driveMotor.freeSpeedRadPerSec") == nullptr);
            CHECK(find(vs, "drivetrain.wheels[0].driveMotor.nominalVoltageV") == nullptr);
        }
    }
}

TEST_CASE("remote spec limits: wheel names must be present, short and unique") {
    Spec s = baseline();
    s.d.wheels[1].name = "";
    CHECK(find(check(s), "drivetrain.wheels[1].name", remote_spec_code::kBadName) != nullptr);
    s.d.wheels[1].name = std::string(kRemoteSpecLimits.maxNameLength + 1, 'a');
    CHECK(find(check(s), "drivetrain.wheels[1].name", remote_spec_code::kTooLong) != nullptr);
    s.d.wheels[1].name = std::string(kRemoteSpecLimits.maxNameLength, 'a');
    CHECK(check(s).empty());
    s.d.wheels[1].name = s.d.wheels[0].name;
    CHECK(find(check(s), "drivetrain.wheels[1].name", remote_spec_code::kDuplicateName) != nullptr);

    Spec bad = baseline();
    bad.d.wheels[0].name = "";
    bad.d.wheels[1].name = bad.d.wheels[2].name;
    bad.d.wheels[3].name = std::string(500, 'x');
    const RemoteRobotSpec fixed = clampRemoteSpec(bad.c, bad.d);
    CHECK(checkRemoteSpec(fixed.chassis, fixed.drivetrain).empty());
    CHECK(fixed.drivetrain.wheels[3].name.size() == kRemoteSpecLimits.maxNameLength);
}

TEST_CASE("remote spec limits: identifiers and enumerations are bounded") {
    Spec s = baseline();
    s.d.kind = std::string(100, 'k');
    s.d.tireModel = "";
    s.d.wheels[0].kind = static_cast<WheelKind>(9);
    s.c.primitives[0].kind = static_cast<PrimitiveKind>(-1);
    s.c.primitives[0].color = std::string(40, '#');
    const auto vs = check(s);
    CHECK(find(vs, "drivetrain.kind", remote_spec_code::kTooLong) != nullptr);
    CHECK(find(vs, "drivetrain.tireModel", remote_spec_code::kBadName) != nullptr);
    CHECK(find(vs, "drivetrain.wheels[0].kind", remote_spec_code::kBadEnum) != nullptr);
    CHECK(find(vs, "chassis.primitives[0].kind", remote_spec_code::kBadEnum) != nullptr);
    CHECK(find(vs, "chassis.primitives[0].color", remote_spec_code::kTooLong) != nullptr);
    const RemoteRobotSpec fixed = clampRemoteSpec(s.c, s.d);
    CHECK(checkRemoteSpec(fixed.chassis, fixed.drivetrain).empty());
    CHECK(fixed.drivetrain.tireModel == "brush");
}

TEST_CASE("remote spec limits: explicit inertia must stay near the solid box value") {
    Spec s = baseline();
    const double box = 50.0 * (0.7 * 0.7 + 0.7 * 0.7) / 12.0;
    s.c.inertiaDiagKgM2 = {0.f, 0.f, static_cast<float>(box)};   // zeros derive; only the set axis is checked
    CHECK(check(s).empty());
    s.c.inertiaDiagKgM2.z = static_cast<float>(box * 4.5);
    CHECK(find(check(s), "chassis.inertiaDiagKgM2.z", remote_spec_code::kOutOfRange) != nullptr);
    s.c.inertiaDiagKgM2.z = -1.f;
    CHECK(find(check(s), "chassis.inertiaDiagKgM2.z", remote_spec_code::kOutOfRange) != nullptr);
    const RemoteRobotSpec fixed = clampRemoteSpec(s.c, s.d);
    CHECK(checkRemoteSpec(fixed.chassis, fixed.drivetrain).empty());
}

TEST_CASE("remote spec limits: the free ground speed of a wheel is capped") {
    Spec s = baseline();
    auto& w = s.d.wheels[0];
    w.driveMotor.freeSpeedRadPerSec = 2000.0;
    w.driveGearRatio = 1.f;
    w.radiusM = 0.1f;
    CHECK(find(check(s), "drivetrain.wheels[0].driveGearRatio", remote_spec_code::kSpeedTooHigh) != nullptr);
    const RemoteRobotSpec fixed = clampRemoteSpec(s.c, s.d);
    CHECK(checkRemoteSpec(fixed.chassis, fixed.drivetrain).empty());
    const WheelSpec& f = fixed.drivetrain.wheels[0];
    CHECK(f.driveMotor.freeSpeedRadPerSec / f.driveGearRatio * f.radiusM <= kRemoteSpecLimits.maxFreeSpeedMps);
    CHECK(f.driveGearRatio > 1.f);

    // The gear ratio alone cannot fix a fast motor on a large wheel: the motor is slowed as well.
    w.radiusM = 0.15f;
    w.driveGearRatio = 2.f;
    const RemoteRobotSpec fixed2 = clampRemoteSpec(s.c, s.d);
    CHECK(checkRemoteSpec(fixed2.chassis, fixed2.drivetrain).empty());
}

TEST_CASE("remote spec limits: rotations must be unit quaternions") {
    Spec s = baseline();
    s.c.primitives[0].local.rotation = Quat{0.f, 0.f, 0.f, 1.05f};
    CHECK(find(check(s), "chassis.primitives[0].local.rotation", remote_spec_code::kOutOfRange) != nullptr);
    s.c.primitives[0].local.rotation = Quat{std::numeric_limits<float>::quiet_NaN(), 0.f, 0.f, 1.f};
    CHECK(find(check(s), "chassis.primitives[0].local.rotation", remote_spec_code::kNonFinite) != nullptr);
    const RemoteRobotSpec fixed = clampRemoteSpec(s.c, s.d);
    CHECK(checkRemoteSpec(fixed.chassis, fixed.drivetrain).empty());
    s.c.primitives[0].local.rotation = Quat{0.f, 0.f, 0.f, 0.f};
    const RemoteRobotSpec zero = clampRemoteSpec(s.c, s.d);
    CHECK(checkRemoteSpec(zero.chassis, zero.drivetrain).empty());
}

TEST_CASE("remote spec limits: a hostile spec is fully corrected and the violation list stays bounded") {
    Spec s;
    const float nan = std::numeric_limits<float>::quiet_NaN();
    s.c.massKg = nan;
    s.c.sizeM = {1e9f, -4.f, nan};
    s.c.comHeightM = -1.f;
    s.c.comOffsetM = {1e6f, nan, 3.f};
    s.c.inertiaDiagKgM2 = {1e9f, -2.f, nan};
    s.c.material = {nan, 99.f, -1.f};
    s.c.primitives.assign(500, PrimitiveSpec{});
    for (PrimitiveSpec& p : s.c.primitives) {
        p.sizeM = {nan, 1e9f, -1.f};
        p.local.position = {1e9f, nan, -1e9f};
        p.color = std::string(1000, 'c');
    }
    s.d.kind = "";
    s.d.loadTransferTauS = nan;
    for (int i = 0; i < 100; ++i) {
        WheelSpec w;
        w.name = "same";
        w.radiusM = i % 2 == 0 ? nan : 100.f;
        w.driveGearRatio = -3.f;
        w.driveCurrentLimitA = 0.f;
        w.driveMotor.stallTorqueNm = nan;
        w.steerMotor.freeCurrentA = 1e9;
        w.tire.mu = 50.f;
        s.d.wheels.push_back(w);
    }
    const auto vs = check(s);
    CHECK_FALSE(vs.empty());
    CHECK(vs.size() <= 64);
    const RemoteRobotSpec fixed = clampRemoteSpec(s.c, s.d);
    const auto after = checkRemoteSpec(fixed.chassis, fixed.drivetrain);
    CHECK(after.empty());
    if (!after.empty()) CHECK(after.front().path == "");
}

TEST_CASE("remote spec limits: violations carry stable codes and paths") {
    Spec s = baseline();
    s.c.massKg = 1.f;
    const auto vs = check(s);
    REQUIRE(vs.size() == 1);
    CHECK(vs[0].code == "spec.out_of_range");
    CHECK(vs[0].path == "chassis.massKg");
    CHECK_FALSE(vs[0].message.empty());
}
