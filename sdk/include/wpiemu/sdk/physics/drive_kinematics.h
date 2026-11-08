#pragma once

// Drive kinematics for the built-in drivetrain kinds ("swerve", "tank", "mecanum"): robot-relative chassis speeds to
// per-wheel setpoints and back, with wheel-speed desaturation and swerve module optimisation. Pure math, standard
// library only. A client turns gamepad input into setpoints with it; the match server turns setpoints into motor
// voltages (the server never needs it) and scripted bots and tests use it as the reference.
//
// The conventions are those of the simulator's traction drivetrains (core/physics drivetrains.cpp), so a robot that
// follows the setpoints moves as predicted:
//  * Chassis frame: +x forward, +y left, +omega counter-clockwise. Chassis speeds are those of the chassis body origin
//    (its centre of mass). WheelSpec::positionM x/y are relative to that origin already (chassis_builder.h
//    wheelHubLocal), so ChassisSpec::comOffsetM, which only shifts the hull, does not enter.
//  * A wheel's traction axis points at (steer angle + tire.tractionAngleRad) from chassis +x; positive wheel speed rolls
//    the wheel forward along it. The steer angle is 0 for fixed wheels and zero = chassis +x.
//  * The tire solver uses an effective radius r * cos(tractionAngle), so wheel speed = (ground speed along the traction
//    axis) / (r * cos(tractionAngle)). That reproduces WPILib's mecanum kinematics for +-45 degree rollers.
//  * Mecanum: a zero tire.tractionAngleRad takes the X layout from the sign of x*y of the wheel position (x*y > 0:
//    -45 deg, x*y < 0: +45 deg, as WPILib); a non-zero angle is used as given.
//  * Tank: fixed-heading wheels, grouped by the sign of positionM.y; every wheel of a side turns at the same rim speed
//    and lateral speed (vy) cannot be commanded.
//  * Swerve: the wheels for which isSteered() holds are modules; the others are fixed wheels along their traction axis.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/core/units.h"
#include "wpiemu/sdk/physics/chassis_spec.h"

namespace wpiemu::sdk {

struct ChassisSpeeds {
    float vx = 0.f;      // m/s forward
    float vy = 0.f;      // m/s left
    float omega = 0.f;   // rad/s counter-clockwise
};

struct WheelSetpoint {
    float wheelSpeedRadPerSec = 0.f;   // positive rolls forward along the wheel's traction axis
    float steerAngleRad = 0.f;         // module azimuth from chassis +x, 0 for fixed wheels
};

namespace drive_kinematics_detail {

enum class Layout { Swerve, Tank, Mecanum };

// Below this ground speed a module has no meaningful heading and keeps its previous steer angle.
inline constexpr double kMinModuleSpeedMps = 1e-4;
inline constexpr double kMinCosTraction = 1e-3;
// A turn of exactly 90 degrees (a strafe from rest) must not flip because of float rounding of the angles.
inline constexpr double kFlipToleranceRad = 1e-4;
inline constexpr double kPivotEpsilon = 1e-9;
inline constexpr double kQuarterPi = 0.78539816339744830962;

inline std::optional<Layout> layoutOf(std::string_view kind) {
    if (kind == drivetrain_kind::kSwerve) return Layout::Swerve;
    if (kind == drivetrain_kind::kTank) return Layout::Tank;
    if (kind == drivetrain_kind::kMecanum) return Layout::Mecanum;
    return std::nullopt;
}

inline bool finite(double v) { return std::isfinite(v); }

// The traction axis angle of a wheel at zero steer: the tire's own angle, or the X layout for a mecanum wheel without one.
inline double tractionAngle(Layout layout, const WheelSpec& wheel) {
    double phi = wheel.tire.tractionAngleRad;
    if (layout == Layout::Mecanum && phi == 0.0) {
        const double handedness = static_cast<double>(wheel.positionM.x) * static_cast<double>(wheel.positionM.y);
        if (handedness > 0.0) phi = -kQuarterPi;
        else if (handedness < 0.0) phi = kQuarterPi;
    }
    return phi;
}

inline bool isModule(Layout layout, const WheelSpec& wheel) { return layout == Layout::Swerve && isSteered(wheel); }

inline double lever(const DrivetrainSpec& spec) {
    double l = 0.0;
    for (const WheelSpec& w : spec.wheels) l = std::max(l, std::hypot(double{w.positionM.x}, double{w.positionM.y}));
    return l;
}

inline Status validateSpeeds(const ChassisSpeeds& s) {
    if (!finite(s.vx) || !finite(s.vy) || !finite(s.omega))
        return makeError(ErrorCode::InvalidArgument, "chassis speeds must be finite");
    return okStatus();
}

inline double previousSteerOf(std::span<const float> previous, std::size_t i) {
    return i < previous.size() && finite(previous[i]) ? static_cast<double>(previous[i]) : 0.0;
}

// Tank wheels grouped by side: 0 = left (y > 0), 1 = right (y < 0), 2 = on the centre line.
struct TankGroups {
    std::array<double, 3> meanY{};
    std::array<int, 3> count{};
    std::vector<int> groupOf;
};

inline TankGroups tankGroups(const DrivetrainSpec& spec) {
    TankGroups g;
    for (const WheelSpec& w : spec.wheels) {
        const int group = w.positionM.y > 0.f ? 0 : (w.positionM.y < 0.f ? 1 : 2);
        g.groupOf.push_back(group);
        g.meanY[static_cast<std::size_t>(group)] += w.positionM.y;
        ++g.count[static_cast<std::size_t>(group)];
    }
    for (std::size_t i = 0; i < 3; ++i)
        if (g.count[i] > 0) g.meanY[i] /= g.count[i];
    return g;
}

// Solves the 3x3 system a x = b (a symmetric, positive semi-definite) by Gaussian elimination with pivoting.
inline bool solve3(std::array<std::array<double, 3>, 3> a, std::array<double, 3> b, std::array<double, 3>& x) {
    double scale = 0.0;
    for (std::size_t i = 0; i < 3; ++i) scale = std::max(scale, std::fabs(a[i][i]));
    if (!(scale > 0.0)) return false;
    for (std::size_t col = 0; col < 3; ++col) {
        std::size_t pivot = col;
        for (std::size_t r = col + 1; r < 3; ++r)
            if (std::fabs(a[r][col]) > std::fabs(a[pivot][col])) pivot = r;
        if (std::fabs(a[pivot][col]) < kPivotEpsilon * scale) return false;
        std::swap(a[col], a[pivot]);
        std::swap(b[col], b[pivot]);
        for (std::size_t r = col + 1; r < 3; ++r) {
            const double f = a[r][col] / a[col][col];
            for (std::size_t c = col; c < 3; ++c) a[r][c] -= f * a[col][c];
            b[r] -= f * b[col];
        }
    }
    for (std::size_t i = 3; i-- > 0;) {
        double sum = b[i];
        for (std::size_t c = i + 1; c < 3; ++c) sum -= a[i][c] * x[c];
        x[i] = sum / a[i][i];
    }
    return true;
}

// Normal equations of the least-squares fit of the chassis speeds u = (vx, vy, omega * L) to the wheel measurements;
// L keeps the three columns of comparable size.
struct Normal3 {
    std::array<std::array<double, 3>, 3> a{};
    std::array<double, 3> b{};

    void addRow(double c0, double c1, double c2, double m) {
        const std::array<double, 3> row{c0, c1, c2};
        for (std::size_t i = 0; i < 3; ++i) {
            for (std::size_t j = 0; j < 3; ++j) a[i][j] += row[i] * row[j];
            b[i] += row[i] * m;
        }
    }

    // A module measures both components of its point velocity (heading and speed); a fixed wheel only the part along its
    // traction axis. `speedRadPerSec` and `steerRad` are the measurement (0 for a rank check).
    void addWheel(const WheelSpec& wheel, Layout layout, double L, double speedRadPerSec, double steerRad) {
        const double x = wheel.positionM.x, y = wheel.positionM.y;
        const double phi = tractionAngle(layout, wheel);
        const double m = speedRadPerSec * wheel.radiusM * std::cos(phi);
        if (isModule(layout, wheel)) {
            const double alpha = steerRad + phi;
            addRow(1.0, 0.0, -y / L, m * std::cos(alpha));
            addRow(0.0, 1.0, x / L, m * std::sin(alpha));
        } else {
            const double c = std::cos(phi), s = std::sin(phi);
            addRow(c, s, (x * s - y * c) / L, m);
        }
    }
};

// Tank: weighted fit of v_side = vx - omega * y_side. Returns false when the sides do not tell the rotation apart.
inline bool solveTank(const TankGroups& g, const std::array<double, 3>& meanSideSpeed, double& vx, double& omega) {
    double sw = 0, swy = 0, swyy = 0, sv = 0, svy = 0;
    for (std::size_t k = 0; k < 3; ++k) {
        if (g.count[k] == 0) continue;
        const double n = g.count[k], y = g.meanY[k], v = meanSideSpeed[k];
        sw += n; swy += n * y; swyy += n * y * y; sv += n * v; svy += n * v * y;
    }
    const double det = sw * swyy - swy * swy;
    if (!(std::fabs(det) > kPivotEpsilon * std::max(1.0, sw * swyy))) return false;
    vx = (swyy * sv - swy * svy) / det;
    omega = (swy * sv - sw * svy) / det;
    return true;
}

// The wheels must be usable (finite, positive radius, traction axes not perpendicular to the rim) and together able to
// produce the whole motion: a layout whose axes all point at one spot cannot turn the chassis, and one whose axes are
// all parallel cannot move it sideways. Such a drivetrain is refused rather than given setpoints that do nothing.
inline Status validateGeometry(const DrivetrainSpec& spec, Layout layout) {
    if (spec.wheels.empty()) return makeError(ErrorCode::InvalidArgument, "drivetrain has no wheels");
    for (const WheelSpec& w : spec.wheels) {
        if (!finite(w.positionM.x) || !finite(w.positionM.y))
            return makeError(ErrorCode::InvalidArgument, "wheel '" + w.name + "' has a non-finite position");
        if (!(finite(w.radiusM) && w.radiusM > 0.f))
            return makeError(ErrorCode::InvalidArgument, "wheel '" + w.name + "' needs a positive radius");
        if (!finite(w.tire.tractionAngleRad))
            return makeError(ErrorCode::InvalidArgument, "wheel '" + w.name + "' has a non-finite traction angle");
        if (layout != Layout::Tank && std::fabs(std::cos(tractionAngle(layout, w))) < kMinCosTraction)
            return makeError(ErrorCode::InvalidArgument, "wheel '" + w.name + "' has a traction axis perpendicular to its rim");
    }
    const double L = lever(spec);
    if (!(L > 1e-6)) return makeError(ErrorCode::InvalidArgument, "all wheels sit at the chassis origin");
    if (layout == Layout::Tank) {
        double vx = 0, omega = 0;
        if (!solveTank(tankGroups(spec), {}, vx, omega))
            return makeError(ErrorCode::InvalidArgument, "tank wheels on one side only cannot turn the chassis");
        return okStatus();
    }
    Normal3 n;
    for (const WheelSpec& w : spec.wheels) n.addWheel(w, layout, L, 0.0, 0.0);
    std::array<double, 3> u{};
    if (!solve3(n.a, n.b, u))
        return makeError(ErrorCode::InvalidArgument, "the wheel traction axes cannot produce both translation and rotation");
    return okStatus();
}

}  // namespace drive_kinematics_detail

// The wheel speed the drive motor reaches at `supplyVoltageV` (<= 0 or non-finite: the motor's nominal voltage) with
// no load, as wheel rad/s. 0 when the motor or the gearing is not physical. The motor's free speed is quoted at its
// nominal voltage and scales with the supply.
inline float maxWheelSpeedRadPerSec(const WheelSpec& wheel, double supplyVoltageV = 0.0) {
    const MotorSpec& m = wheel.driveMotor;
    const double volts = std::isfinite(supplyVoltageV) && supplyVoltageV > 0.0 ? supplyVoltageV : m.nominalVoltageV;
    const double gear = wheel.driveGearRatio;
    if (!(std::isfinite(m.freeSpeedRadPerSec) && m.freeSpeedRadPerSec > 0.0 && std::isfinite(m.nominalVoltageV) &&
          m.nominalVoltageV > 0.0 && std::isfinite(gear) && gear > 0.0 && std::isfinite(volts)))
        return 0.f;
    return static_cast<float>(m.freeSpeedRadPerSec * (volts / m.nominalVoltageV) / gear);
}

// The ground speed the slowest wheel's rim can reach: the straight-line limit a client scales its stick input by.
inline float maxLinearSpeedMps(const DrivetrainSpec& spec) {
    float limit = std::numeric_limits<float>::infinity();
    for (const WheelSpec& w : spec.wheels) limit = std::min(limit, maxWheelSpeedRadPerSec(w) * w.radiusM);
    return std::isfinite(limit) ? limit : 0.f;
}

// Wheel setpoints of `speeds` with no desaturation and no module optimisation. `previousSteerRad` (empty = all 0) is
// kept by a swerve module that has no ground speed. InvalidArgument for non-finite speeds, unusable geometry and
// Unsupported for a drivetrain kind other than swerve, tank or mecanum (a registry-extended kind is not guessed).
inline Result<std::vector<WheelSetpoint>> wheelSetpointsRaw(const DrivetrainSpec& spec, const ChassisSpeeds& speeds,
                                                            std::span<const float> previousSteerRad = {}) {
    using namespace drive_kinematics_detail;
    const std::optional<Layout> layout = layoutOf(spec.kind);
    if (!layout) return makeError(ErrorCode::Unsupported, "no kinematics for drivetrain kind '" + spec.kind + "'");
    if (Status s = validateSpeeds(speeds); !s.ok()) return s.error();
    if (Status s = validateGeometry(spec, *layout); !s.ok()) return s.error();
    if (!previousSteerRad.empty() && previousSteerRad.size() != spec.wheels.size())
        return makeError(ErrorCode::InvalidArgument, "previous steer angles do not match the wheel count");

    const double vx = speeds.vx, vy = speeds.vy, w = speeds.omega;
    std::vector<WheelSetpoint> out(spec.wheels.size());
    TankGroups tank;
    if (*layout == Layout::Tank) tank = tankGroups(spec);

    for (std::size_t i = 0; i < spec.wheels.size(); ++i) {
        const WheelSpec& wheel = spec.wheels[i];
        const double x = wheel.positionM.x, y = wheel.positionM.y, r = wheel.radiusM;
        const double phi = tractionAngle(*layout, wheel);
        WheelSetpoint& sp = out[i];
        if (*layout == Layout::Tank) {
            const double side = vx - w * tank.meanY[static_cast<std::size_t>(tank.groupOf[i])];
            sp.wheelSpeedRadPerSec = static_cast<float>(side / r);
        } else if (isModule(*layout, wheel)) {
            const double px = vx - w * y, py = vy + w * x;
            const double speed = std::hypot(px, py);
            if (speed < kMinModuleSpeedMps) {
                sp.wheelSpeedRadPerSec = 0.f;
                sp.steerAngleRad = static_cast<float>(previousSteerOf(previousSteerRad, i));
            } else {
                sp.steerAngleRad = static_cast<float>(wrapAngleRad(std::atan2(py, px) - phi));
                sp.wheelSpeedRadPerSec = static_cast<float>(speed / (r * std::cos(phi)));
            }
        } else {
            const double along = (vx - w * y) * std::cos(phi) + (vy + w * x) * std::sin(phi);
            sp.wheelSpeedRadPerSec = static_cast<float>(along / (r * std::cos(phi)));
        }
    }
    return out;
}

// Scales every wheel speed by one factor (<= 1) so that none exceeds its wheel's maxWheelSpeedRadPerSec; steer angles
// and the ratios between the speeds stay. Returns the factor (1 when nothing was scaled).
inline float desaturateWheelSetpoints(const DrivetrainSpec& spec, std::span<WheelSetpoint> setpoints) {
    double scale = 1.0;
    const std::size_t n = std::min(spec.wheels.size(), setpoints.size());
    for (std::size_t i = 0; i < n; ++i) {
        const double limit = maxWheelSpeedRadPerSec(spec.wheels[i]);
        const double speed = std::fabs(static_cast<double>(setpoints[i].wheelSpeedRadPerSec));
        if (speed > limit) scale = std::min(scale, limit / speed);
    }
    if (scale < 1.0)
        for (WheelSetpoint& sp : setpoints) sp.wheelSpeedRadPerSec = static_cast<float>(sp.wheelSpeedRadPerSec * scale);
    return static_cast<float>(scale);
}

// The same module motion by the shortest turn from `previousSteerRad`: a steer change beyond 90 degrees (plus a 1e-4 rad
// rounding margin) turns the module the other way round instead (180 degrees away, speed negated). The result is wrapped
// to (-pi, pi]. A setpoint with no speed keeps the previous angle unchanged.
inline WheelSetpoint optimizeSwerveSetpoint(WheelSetpoint target, float previousSteerRad) {
    using drive_kinematics_detail::finite;
    const double previous = finite(previousSteerRad) ? static_cast<double>(previousSteerRad) : 0.0;
    if (target.wheelSpeedRadPerSec == 0.f || !finite(target.steerAngleRad)) return {target.wheelSpeedRadPerSec, static_cast<float>(previous)};
    double delta = wrapAngleRad(static_cast<double>(target.steerAngleRad) - previous);
    double speed = target.wheelSpeedRadPerSec;
    if (std::fabs(delta) > 0.5 * kPi + drive_kinematics_detail::kFlipToleranceRad) {
        delta -= std::copysign(kPi, delta);
        speed = -speed;
    }
    return {static_cast<float>(speed), static_cast<float>(wrapAngleRad(previous + delta))};
}

// The full pipeline of a client: wheelSetpointsRaw, desaturateWheelSetpoints, then optimizeSwerveSetpoint for every
// swerve module.
inline Result<std::vector<WheelSetpoint>> wheelSetpoints(const DrivetrainSpec& spec, const ChassisSpeeds& speeds,
                                                         std::span<const float> previousSteerRad = {}) {
    using namespace drive_kinematics_detail;
    auto raw = wheelSetpointsRaw(spec, speeds, previousSteerRad);
    if (!raw.ok()) return raw;
    std::vector<WheelSetpoint> out = std::move(raw).value();
    for (const WheelSpec& wheel : spec.wheels)
        if (!(maxWheelSpeedRadPerSec(wheel) > 0.f))
            return makeError(ErrorCode::InvalidArgument, "wheel '" + wheel.name + "' has no usable drive motor or gearing");
    desaturateWheelSetpoints(spec, out);
    const Layout layout = *layoutOf(spec.kind);
    for (std::size_t i = 0; i < out.size(); ++i)
        if (isModule(layout, spec.wheels[i])) out[i] = optimizeSwerveSetpoint(out[i], static_cast<float>(previousSteerOf(previousSteerRad, i)));
    return out;
}

// Forward kinematics: the chassis speeds that best explain the wheel setpoints (least squares over the wheels, exact
// for setpoints produced by wheelSetpointsRaw). InvalidArgument when the wheels do not determine the motion (too few or
// degenerate wheels, such as a tank, which cannot report vy) or on non-finite input.
inline Result<ChassisSpeeds> chassisSpeedsFromWheels(const DrivetrainSpec& spec, std::span<const WheelSetpoint> setpoints) {
    using namespace drive_kinematics_detail;
    const std::optional<Layout> layout = layoutOf(spec.kind);
    if (!layout) return makeError(ErrorCode::Unsupported, "no kinematics for drivetrain kind '" + spec.kind + "'");
    if (Status s = validateGeometry(spec, *layout); !s.ok()) return s.error();
    if (setpoints.size() != spec.wheels.size())
        return makeError(ErrorCode::InvalidArgument, "wheel setpoint count does not match the wheel count");
    for (const WheelSetpoint& sp : setpoints)
        if (!finite(sp.wheelSpeedRadPerSec) || !finite(sp.steerAngleRad))
            return makeError(ErrorCode::InvalidArgument, "wheel setpoints must be finite");

    if (*layout == Layout::Tank) {
        // v_side = vx - omega * y_side over the side groups, weighted by wheel count.
        const TankGroups g = tankGroups(spec);
        std::array<double, 3> sideSpeed{};
        for (std::size_t i = 0; i < spec.wheels.size(); ++i)
            sideSpeed[static_cast<std::size_t>(g.groupOf[i])] +=
                static_cast<double>(setpoints[i].wheelSpeedRadPerSec) * spec.wheels[i].radiusM;
        for (std::size_t k = 0; k < 3; ++k)
            if (g.count[k] > 0) sideSpeed[k] /= g.count[k];
        double vx = 0, omega = 0;
        if (!solveTank(g, sideSpeed, vx, omega))
            return makeError(ErrorCode::InvalidArgument, "tank wheels do not determine the rotation (one side only)");
        return ChassisSpeeds{static_cast<float>(vx), 0.f, static_cast<float>(omega)};
    }

    const double L = lever(spec);
    Normal3 n;
    for (std::size_t i = 0; i < spec.wheels.size(); ++i)
        n.addWheel(spec.wheels[i], *layout, L, setpoints[i].wheelSpeedRadPerSec, setpoints[i].steerAngleRad);
    std::array<double, 3> u{};
    if (!solve3(n.a, n.b, u)) return makeError(ErrorCode::InvalidArgument, "wheels do not determine the chassis motion");
    return ChassisSpeeds{static_cast<float>(u[0]), static_cast<float>(u[1]), static_cast<float>(u[2] / L)};
}

// A drivetrain's kinematics bound to its spec, remembering the last steer angles so that module optimisation and the
// zero-speed hold work from one update to the next.
class DriveKinematics {
public:
    // Fails for a kind other than swerve, tank or mecanum and for geometry or motors the kinematics cannot use.
    static Result<DriveKinematics> create(DrivetrainSpec spec) {
        using namespace drive_kinematics_detail;
        const std::optional<Layout> layout = layoutOf(spec.kind);
        if (!layout) return makeError(ErrorCode::Unsupported, "no kinematics for drivetrain kind '" + spec.kind + "'");
        if (Status s = validateGeometry(spec, *layout); !s.ok()) return s.error();
        for (const WheelSpec& wheel : spec.wheels)
            if (!(maxWheelSpeedRadPerSec(wheel) > 0.f))
                return makeError(ErrorCode::InvalidArgument, "wheel '" + wheel.name + "' has no usable drive motor or gearing");
        return DriveKinematics(std::move(spec));
    }

    const DrivetrainSpec& spec() const { return spec_; }
    std::span<const float> previousSteerRad() const { return previousSteer_; }
    float maxLinearSpeedMps() const { return wpiemu::sdk::maxLinearSpeedMps(spec_); }

    // Forgets the steer history (`steerRad` empty: all modules straight, otherwise one angle per wheel).
    void resetSteer(std::span<const float> steerRad = {}) {
        previousSteer_.assign(spec_.wheels.size(), 0.f);
        if (steerRad.size() == previousSteer_.size()) previousSteer_.assign(steerRad.begin(), steerRad.end());
    }

    // wheelSetpoints() for `speeds`; the steer angles become the history. Unchanged history on failure.
    Result<std::vector<WheelSetpoint>> update(const ChassisSpeeds& speeds) {
        auto out = wheelSetpoints(spec_, speeds, previousSteer_);
        if (out.ok())
            for (std::size_t i = 0; i < previousSteer_.size(); ++i) previousSteer_[i] = out.value()[i].steerAngleRad;
        return out;
    }

    Result<ChassisSpeeds> chassisSpeeds(std::span<const WheelSetpoint> setpoints) const {
        return chassisSpeedsFromWheels(spec_, setpoints);
    }

private:
    explicit DriveKinematics(DrivetrainSpec spec) : spec_(std::move(spec)), previousSteer_(spec_.wheels.size(), 0.f) {}

    DrivetrainSpec spec_;
    std::vector<float> previousSteer_;
};

}  // namespace wpiemu::sdk
