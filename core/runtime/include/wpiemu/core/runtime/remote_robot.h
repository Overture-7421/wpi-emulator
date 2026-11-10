#pragma once

// Remote robots of a practice match inside SimulationSession (docs plan: "Remote robot: wheels, tires and motors").
// A remote robot has no WPILib program: a client sends commanded wheel setpoints (match::MatchInput), the session turns
// them into motor voltages and drives the robot through the SAME chassis, tire and drivetrain code as the local robot.
//
// This header holds the seam and the plain data the session API speaks, with no physics includes (the composition root
// injects the real chassis through RemoteChassisFactory, see physics_wiring.h; tests may inject a fake).
//
// Threading: everything here is used on the SIM THREAD only. The match host feeds inputs through the session between
// ticks (submitRemoteInput) and adds or removes robots between ticks, never inside one.

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>

#include "wpiemu/sdk/config/robot_config.h"
#include "wpiemu/sdk/config/start_position.h"
#include "wpiemu/sdk/core/interface.h"
#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/game/field.h"
#include "wpiemu/sdk/ipc/driver_station.h"
#include "wpiemu/sdk/match/input_messages.h"
#include "wpiemu/sdk/match/match_protocol.h"
#include "wpiemu/sdk/physics/drive_kinematics.h"
#include "wpiemu/sdk/physics/drivetrain.h"
#include "wpiemu/sdk/physics/physics_world.h"

namespace wpiemu::core::runtime {

// ---- limits -----------------------------------------------------------------------------------------------------

// The host's own robot is one of the match's kMaxRobotsPerMatch.
inline constexpr std::size_t kMaxRemoteRobots = sdk::match::kMaxRobotsPerMatch - 1;
// Inputs waiting for their target tick, per robot. A client at kMaxInputRateHz with the largest accepted lead needs
// kMaxRemoteInputLeadUs / (1e6 / kMaxInputRateHz) = 120 entries; the normal 60 Hz rate needs 60. On overflow the OLDEST
// queued input (lowest sequence) is evicted, so a flood of far-future inputs cannot grow memory.
inline constexpr std::size_t kMaxQueuedRemoteInputs = 128;
// An input whose target tick is further ahead than this is rejected (a client leads by tens of milliseconds).
inline constexpr std::int64_t kMaxRemoteInputLeadUs = 1'000'000;

// ---- the physical robot (injected) ------------------------------------------------------------------------------

// One remote robot's chassis, wheels and setpoint controller, as the session drives them. The session never steps the
// world through it: prepareStep for every robot, ONE IPhysicsWorld::step, then completeStep for every robot.
// Destroying it removes its body from the world, so it must be destroyed BEFORE the world.
class IRemoteChassis {
    WPIEMU_INTERFACE(IRemoteChassis)
public:
    virtual std::size_t wheelCount() const = 0;
    // One setpoint per wheel in DrivetrainSpec order (the session already sanitized them).
    virtual void setSetpoints(std::span<const sdk::WheelSetpoint> setpoints) = 0;
    // Zero wheel speed, steer targets unchanged: the robot brakes without the modules snapping straight.
    virtual void brake() = 0;
    // Before the world step: controller voltages (0 V when !enabled) and the drivetrain's tire forces.
    virtual void prepareStep(bool enabled, float dtSeconds) = 0;
    // After the world step.
    virtual void completeStep(float dtSeconds) = 0;
    // Teleport to `placement` at rest and forget setpoints and drivetrain state.
    virtual void reset(const sdk::RobotStartPose& placement) = 0;
    virtual sdk::BodyId body() const = 0;
    virtual std::span<const sdk::WheelState> wheelStates() const = 0;
};

// Builds a remote robot's chassis in `world` from the spec's chassis and drivetrain, stamping `userTag` on every shape
// (sdk::makeRobotTag). An empty factory in SessionParts makes addRemoteRobot return ErrorCode::Unsupported.
using RemoteChassisFactory = std::function<sdk::Result<std::unique_ptr<IRemoteChassis>>(
    sdk::IPhysicsWorld& world, const sdk::RobotConfig& spec, std::uint64_t userTag, const sdk::RobotStartPose& placement)>;

// ---- session API data -------------------------------------------------------------------------------------------

struct RemoteRobotRequest {
    sdk::match::ParticipantId participantId = sdk::match::kNoParticipant;   // not kHostParticipantId
    std::string name;
    sdk::proto::AllianceStation station = sdk::proto::AllianceStation::Red1;
    // The accepted Join spec, already decoded and defaulted by the caller (chassis, drivetrain, zones, plugin sections).
    // The session validates it STRICTLY (sdk::checkRemoteSpec) and never clamps: the match host decides whether to
    // clamp (sdk::clampRemoteSpec) before adding.
    sdk::RobotConfig spec;
};

// What happened to one submitted input.
enum class RemoteInputOutcome : std::uint8_t {
    Queued,       // waits for its target tick
    QueuedLate,   // its target tick had already passed on arrival; it applies on the next tick
    Duplicate,    // the same sequence is already queued or was the last one applied
    Stale,        // older than the input already applied: dropped
};

struct RemoteInputStats {
    std::uint64_t received = 0;     // every call to submitRemoteInput for this robot
    std::uint64_t queued = 0;       // accepted into the queue (Queued + QueuedLate)
    std::uint64_t applied = 0;      // inputs that became the robot's current setpoints
    std::uint64_t late = 0;         // QueuedLate
    std::uint64_t duplicate = 0;
    std::uint64_t stale = 0;
    std::uint64_t rejected = 0;     // wrong wheel count, sequence 0 or a target tick too far ahead
    std::uint64_t superseded = 0;   // queued but skipped because a newer sequence applied first
    std::uint64_t evicted = 0;      // pushed out of a full queue
    // The newest input applied (sdk::match::kNoInputSeq / proto::kNoAckTick before any) and the host tick it applied on
    // (what MatchSnapshot::ackInputSeq / ackTick report).
    std::uint32_t lastAppliedSeq = sdk::match::kNoInputSeq;
    std::uint64_t ackTick = sdk::proto::kNoAckTick;
    // targetTick of the newest-sequence input received minus the host tick it arrived on (negative: it arrived late);
    // what MatchSnapshot::inputLeadTicks reports after saturating to i16.
    std::int64_t inputLeadTicks = 0;
    std::size_t queueDepth = 0;     // inputs waiting now
};

struct RemoteRobotInfo {
    sdk::match::ParticipantId participantId = sdk::match::kNoParticipant;
    std::string name;
    sdk::proto::AllianceStation station = sdk::proto::AllianceStation::Red1;
    sdk::RobotStartPose placement;  // where reset() puts the robot; fixed when it was added
    RemoteInputStats stats;
    bool enabled = false;           // as of the last tick
    std::uint32_t zoneFlags = 0;    // active zones as the game sees them (0 while disabled or without input)
};

// ---- start placement --------------------------------------------------------------------------------------------

// A robot's footprint as a circle, for keeping start poses apart.
struct StartDisc {
    float xM = 0.f;
    float yM = 0.f;
    float radiusM = 0.f;
};

inline constexpr float kRemoteStartSetbackM = 2.0f;              // like the host's default start (robot_session.cpp)
inline constexpr float kRemoteStartSetbackFraction = 0.15f;
inline constexpr float kRemoteStartClearanceM = 0.15f;           // gap kept between two start footprints
inline constexpr float kRemoteStartStepM = 0.25f;                // how far a blocked start slides away from its wall

// The start pose of an alliance station on `frame` (only its length and width are read, so any season's field works):
// the blue alliance stands `min(2 m, 15% of the length)` from the wall at x = 0 facing +X, the red alliance the same
// distance from the far wall facing -X. Stations 1..3 spread over the width left to right AS SEEN BY THE ALLIANCE
// facing the field, like the Driver Station numbering (sdk::driverStationSpot): blue 1 is at the high-y side, red 1 at
// the low-y side, station 2 on the centre line.
sdk::RobotStartPose remoteStartPose(const sdk::FieldFrame& frame, sdk::proto::AllianceStation station);

// remoteStartPose for a robot whose footprint is the circle of `radiusM`: kept inside the field, and slid away from its
// alliance wall in kRemoteStartStepM steps until it clears every disc of `occupied` (the host robot's start, robots
// already placed) by kRemoteStartClearanceM. Pure and deterministic.
sdk::RobotStartPose remoteStartPoseClear(const sdk::FieldFrame& frame, sdk::proto::AllianceStation station, float radiusM,
                                         std::span<const StartDisc> occupied);

// The circle around a chassis footprint (half the diagonal of its frame box).
float chassisFootprintRadiusM(const sdk::ChassisSpec& chassis);

}  // namespace wpiemu::core::runtime
