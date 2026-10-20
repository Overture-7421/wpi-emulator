#pragma once

// Static descriptions of the chassis and drivetrain (also the persisted config shape).

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "wpiemu/sdk/core/geometry.h"
#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/dynamics/motor_model.h"
#include "wpiemu/sdk/physics/physics_types.h"
#include "wpiemu/sdk/physics/tire_model.h"

namespace wpiemu::sdk {

// docs/reference/physics.md: one rigid body, no suspension, body origin == COM. Dimensions are the bounding box
// of the frame perimeter (editor primitives refine visuals/collisions).
struct ChassisSpec {
    Vec3 sizeM{0.7f, 0.7f, 0.12f};           // frame box (x forward, y left, z up)
    float massKg = 50.f;
    float comHeightM = 0.20f;                // COM height above the floor (drives load transfer)
    Vec3 comOffsetM;                         // COM offset from the chassis box centre in XY (z ignored)
    float groundClearanceM = 0.03f;          // bottom of the frame above the floor
    Vec3 inertiaDiagKgM2;                    // all zero => derive from a solid box of sizeM
    Material material{0.3f, 0.1f, 0.f};
    std::vector<PrimitiveSpec> primitives;   // extra visual / collision primitives (frame, bumpers, ...)
};

enum class WheelKind { Standard, Omni, Mecanum, Swerve };

// The persisted key of a wheel kind ("standard", "omni", "mecanum", "swerve") and its inverse; nullopt for an unknown key.
inline const char* wheelKindName(WheelKind kind) {
    switch (kind) {
        case WheelKind::Standard: return "standard";
        case WheelKind::Omni: return "omni";
        case WheelKind::Mecanum: return "mecanum";
        case WheelKind::Swerve: return "swerve";
    }
    return "standard";
}

inline std::optional<WheelKind> parseWheelKind(std::string_view key) {
    for (const WheelKind kind : {WheelKind::Standard, WheelKind::Omni, WheelKind::Mecanum, WheelKind::Swerve})
        if (key == wheelKindName(kind)) return kind;
    return std::nullopt;
}

struct WheelSpec {
    std::string name;                // unique within the drivetrain: "fl", "fr", "left0", ...
    WheelKind kind = WheelKind::Standard;
    Vec3 positionM;                  // wheel hub in the chassis frame (z = hub height above the floor in the rest pose)
    float radiusM = 0.0508f;         // 4 in
    float widthM = 0.038f;
    float inertiaKgM2 = 1.5e-3f;     // wheel + rotating hardware
    TireParams tire;
    bool steered = false;            // swerve azimuth

    MotorSpec driveMotor = motors::krakenX60();
    int driveMotorCount = 1;
    float driveGearRatio = 6.75f;    // motor : wheel
    float driveEfficiency = 0.9f;
    float driveCurrentLimitA = 60.f; // 0 = unlimited (stator)

    MotorSpec steerMotor = motors::neo550();
    float steerGearRatio = 12.8f;
    float steerInertiaKgM2 = 0.004f;
};

namespace drivetrain_kind {
inline constexpr const char* kSwerve = "swerve";
inline constexpr const char* kTank = "tank";
inline constexpr const char* kMecanum = "mecanum";
}  // namespace drivetrain_kind

struct DrivetrainSpec {
    std::string kind = drivetrain_kind::kSwerve;   // key into DrivetrainRegistry (open set)
    std::string tireModel = "brush";               // key into TireModelRegistry
    std::vector<WheelSpec> wheels;
    float loadTransferTauS = 0.02f;                // low-pass on chassis acceleration (docs/reference/physics.md)
};

}  // namespace wpiemu::sdk
