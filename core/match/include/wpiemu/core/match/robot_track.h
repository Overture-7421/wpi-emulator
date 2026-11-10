#pragma once

// The received states of ONE robot over time and the interpolation between them: what a game client draws for a remote robot
// (and for its own robot while it has no prediction). Samples arrive with the host's sim time; the client draws them a little
// in the past (its interpolation delay) so there is almost always a sample on both sides of the render time.
//
//   * Position and yaw: cubic Hermite between the two bracketing samples, using their velocities (the wire carries them), so a
//     30 Hz stream still moves smoothly through corners and accelerations.
//   * Steer angles: the short way round. Spin angles: unwrapped against the rate of the previous sample pair, so a fast wheel
//     does not appear to spin backwards between two samples, then linear.
//   * Zone flags: held from the earlier sample; zone poses: the two bracketing samples, blended by the caller.
//   * Beyond the newest sample (the buffer starved): dead reckoning with the sample's velocity for at most the extrapolation
//     limit (sdk::match::kMaxExtrapolationUs), then hold; `extrapolating` and `starved` say so.

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <vector>

#include "wpiemu/sdk/match/snapshot_messages.h"

namespace wpiemu::core::match {

struct RemoteWheelSample {
    double steerRad = 0.0;
    double spinRaw = 0.0;       // as sent, wrapped
    double spinCumRad = 0.0;    // unwrapped across samples
    double spinRateRadPerSec = 0.0;
};

struct RemoteSample {
    std::int64_t simTimeUs = 0;
    std::int64_t arrivalUs = 0;
    double x = 0.0, y = 0.0, yaw = 0.0;       // field frame, metres and radians
    double vx = 0.0, vy = 0.0, omega = 0.0;   // world-frame velocity and yaw rate
    std::vector<RemoteWheelSample> wheels;
    std::uint32_t zoneFlags = 0;
    std::vector<sdk::match::ZonePoseInput> zonePoses;   // overrides of the rest poses, chassis frame
};

// A RemoteSample from the wire state of a robot (wheel spin is left to RobotTrack::push to unwrap).
RemoteSample sampleFromWire(const sdk::match::RobotWireState& robot, std::int64_t simTimeUs, std::int64_t arrivalUs);

struct RobotAt {
    double x = 0.0, y = 0.0, yaw = 0.0;
    double vx = 0.0, vy = 0.0, omega = 0.0;     // of the drawn motion
    struct Wheel {
        double steerRad = 0.0;
        double spinRad = 0.0;
    };
    std::vector<Wheel> wheels;
    std::uint32_t zoneFlags = 0;
    // The samples the zone poses blend between (valid until the track changes): `to` is null when `from` stands alone.
    const RemoteSample* from = nullptr;
    const RemoteSample* to = nullptr;
    double blend = 0.0;                          // 0 = from, 1 = to
    bool extrapolating = false;                  // the render time is beyond the newest sample
    bool starved = false;                        // beyond the extrapolation limit: the robot is held
    std::int64_t extrapolatedUs = 0;
};

class RobotTrack {
public:
    static constexpr std::size_t kMaxSamples = 128;

    // False (nothing stored) when the sample is not newer than the newest one.
    bool push(RemoteSample sample);
    void clear() { samples_.clear(); }
    bool empty() const { return samples_.empty(); }
    std::size_t size() const { return samples_.size(); }
    const RemoteSample* latest() const { return samples_.empty() ? nullptr : &samples_.back(); }

    // The robot at sim time `renderSimUs`; nullopt when the track is empty.
    std::optional<RobotAt> at(std::int64_t renderSimUs, std::int64_t maxExtrapolationUs) const;

private:
    std::deque<RemoteSample> samples_;
};

}  // namespace wpiemu::core::match
