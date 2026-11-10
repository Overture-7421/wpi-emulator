#pragma once

// Client-side prediction of the player's own robot with server reconciliation.
//
// Timeline. The predictor counts HOST ticks: state(k) is the own robot after k completed ticks of the host's run epoch, the
// same index MatchSnapshot::tick uses. The client runs it AHEAD of the host (by the input lead), so an input applied here at
// tick k is on screen at once and reaches the host before the host reaches tick k, where the host applies it too
// (MatchInput::targetTick = k). stepTick() advances one tick, applying a new input at its tick; every state is recorded.
//
// Reconciliation. A snapshot carries the authoritative planar state of the own robot after `snapshotTick` ticks and the newest
// input the host had applied by then (ackInputSeq). reconcile():
//   1. compares it with the recorded prediction for that tick (position, yaw, velocity, steer angles);
//   2. within the dead band (the quantization of the wire plus rounding) nothing happens, the prediction was right;
//   3. otherwise the recorded state is replaced by the authoritative one (height, tilt and vertical velocity, which the wire
//      does not carry, are kept from the prediction; wheel spin speed is kept too), and every tick after it is SIMULATED AGAIN
//      with the inputs the host has not acknowledged, each applied on its target tick or, if that has passed, at once (as the
//      host does with a late input);
//   4. the change this makes to the CURRENT pose is not shown at once: it is added to a visual offset that decays with time
//      constant correctionTauUs (pose, wheel steer and wheel spin alike), so the picture moves smoothly. A change beyond
//      snapPosM or snapYawRad is shown at once (the offset is dropped).
// The simulation is deterministic, so with no contact and the same inputs the prediction stays inside the dead band and no
// replay ever runs; a push or a late input costs one replay of (current tick - snapshot tick) ticks.

#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <vector>

#include "wpiemu/core/match/prediction_world.h"
#include "wpiemu/sdk/match/input_messages.h"
#include "wpiemu/sdk/match/snapshot_messages.h"
#include "wpiemu/sdk/physics/drive_kinematics.h"

namespace wpiemu::core::match {

struct OwnPredictorConfig {
    std::int64_t stepUs = 5000;
    double deadbandPosM = 0.01;
    double deadbandYawRad = 0.01;
    double deadbandVelMps = 0.05;
    double deadbandSteerRad = 0.05;
    double snapPosM = 1.0;
    double snapYawRad = 0.7853981633974483;   // 45 degrees
    std::int64_t correctionTauUs = 45'000;
    std::size_t historyTicks = 512;           // recorded states (2.5 s at 200 Hz)
    std::size_t maxInputs = 256;
};

// The authoritative planar state of the own robot from a snapshot (field frame; angles in radians).
struct OwnAuthoritative {
    double x = 0.0, y = 0.0, yaw = 0.0;
    double vx = 0.0, vy = 0.0, omega = 0.0;
    std::vector<double> steerRad;
    std::vector<double> spinRad;
};

OwnAuthoritative authoritativeFromWire(const sdk::match::RobotWireState& robot);

// One input the client generated: what the prediction applies at `targetTick` and what is sent.
struct PredictedInput {
    std::uint32_t seq = sdk::match::kNoInputSeq;
    std::uint64_t targetTick = 0;
    std::vector<sdk::WheelSetpoint> setpoints;
    std::uint32_t zoneFlags = 0;
    std::vector<sdk::match::ZonePoseInput> zonePoses;
};

enum class ReconcileKind : std::uint8_t {
    NotStarted,   // no prediction timeline yet
    Ahead,        // the snapshot is for a tick the prediction has not reached
    TooOld,       // older than the recorded history
    InBand,       // within the dead band: the prediction was right
    Corrected,    // rewound and replayed, correction smoothed
    Snapped,      // rewound and replayed, correction shown at once
};

struct ReconcileResult {
    ReconcileKind kind = ReconcileKind::NotStarted;
    double posErrM = 0.0;
    double yawErrRad = 0.0;
    double velErrMps = 0.0;
    double steerErrRad = 0.0;
    std::uint64_t replayedTicks = 0;
};

// What the player sees of the own robot: the interpolated prediction plus the decaying correction offset.
struct OwnShown {
    sdk::Pose3 pose;
    sdk::Vec3 linearVelocity;
    sdk::Vec3 angularVelocity;
    struct Wheel {
        double steerRad = 0.0;
        double spinRad = 0.0;
    };
    std::vector<Wheel> wheels;
};

class OwnPredictor {
public:
    // Called before every simulated tick (forward and replay) with the tick about to run, so the owner can place the other
    // robots' kinematic bodies for that tick.
    using TickHook = std::function<void(PredictionWorld&, std::uint64_t tick)>;

    OwnPredictor(std::unique_ptr<PredictionWorld> world, OwnPredictorConfig config);

    void setTickHook(TickHook hook) { hook_ = std::move(hook); }

    bool started() const { return started_; }
    // Number of completed ticks of the recorded present (state(tick()) is the newest).
    std::uint64_t tick() const { return tick_; }
    PredictionWorld& world() { return *world_; }
    const OwnPredictorConfig& config() const { return config_; }

    // (Re)starts the timeline from the authoritative state after `hostTick` ticks and simulates up to `toTick` (>= hostTick)
    // with the inputs held (an empty list: the robot at rest). Drops the correction offset.
    void start(const OwnAuthoritative& authoritative, std::uint64_t hostTick, std::uint64_t toTick, bool enabled);
    void stop() { started_ = false; }
    // Forgets the inputs kept for replay (a new session or run epoch: the host's queue is gone).
    void clearInputs() { inputs_.clear(); }

    // Simulates tick tick(): applies `input` (if any) first. The input's targetTick should equal tick().
    void stepTick(const PredictedInput* input, bool enabled);

    ReconcileResult reconcile(const OwnAuthoritative& authoritative, std::uint64_t snapshotTick, std::uint32_t ackInputSeq, bool enabled);

    // The own robot at `frac` (0..1) of the way from state(tick() - 1) to state(tick()), with the correction offset applied.
    OwnShown shown(double frac) const;
    // The newest predicted body state (no offset): where the robot is now, for field-relative driving.
    const OwnBodyState& newest() const { return history_.back(); }
    void decayCorrection(std::int64_t dtUs);
    double correctionMagnitudeM() const;
    std::size_t inputCount() const { return inputs_.size(); }

private:
    struct Correction {
        double x = 0.0, y = 0.0, yaw = 0.0;
        std::vector<double> steer, spin;
        void clear(std::size_t wheels);
    };

    const OwnBodyState* stateAt(std::uint64_t tick) const;
    void record(const OwnBodyState& s);
    void applyInput(const PredictedInput& in);
    void runTick(std::uint64_t tick, bool enabled);
    void prune(std::uint32_t ackInputSeq);

    std::unique_ptr<PredictionWorld> world_;
    OwnPredictorConfig config_;
    TickHook hook_;
    bool started_ = false;
    std::uint64_t tick_ = 0;
    std::uint64_t firstTick_ = 0;
    std::deque<OwnBodyState> history_;     // history_[i] = state(firstTick_ + i)
    std::deque<PredictedInput> inputs_;    // ascending seq: the last acknowledged one and everything newer
    Correction correction_;
};

}  // namespace wpiemu::core::match
