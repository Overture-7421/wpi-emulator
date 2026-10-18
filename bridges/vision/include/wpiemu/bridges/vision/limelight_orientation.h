#pragma once

// The robot's writes to robot_orientation_set, kept so that MegaTag2 can use the orientation the robot had when the
// frame was CAPTURED: a frame reaches the bridge one latency after its capture, and a real device interpolates the
// robot's yaw to the capture time of every frame [MT2: "MT2 uses interpolated yaw from robot's gyro sent via
// SetRobotOrientation()"].
//
// ntcore drops a write that equals the previous one, so the history holds only DISTINCT samples (time of the write,
// value) and the absence of a sample means "unchanged", not "unknown":
//   * the orientation at time T is the latest sample written at or before T; it holds until the robot writes
//     something else;
//   * when T lies between two samples written at most kMaxInterpolationGapUs apart (the robot is writing every loop)
//     the angles are interpolated along the SHORTEST arc: 170 -> -170 passes through 180, not through 0;
//   * samples farther apart were separated by loops that wrote nothing because nothing changed, so the earlier one
//     holds until the later one's write time: a step in the robot's yaw is not smeared over the quiet period before it.
//
// Before the first write the device only has its power-on default, all zeros, and that is what at() returns. MegaTag2
// is then solved with robot yaw = pitch = roll = 0, which is what a robot that never calls SetRobotOrientation gets on
// hardware: a pose rotated by its true heading.

#include <cstddef>
#include <cstdint>
#include <deque>

#include "wpiemu/bridges/vision/limelight_math.h"

namespace wpiemu::bridges::vision::limelight {

class OrientationHistory {
public:
    // A frame is at most 1 s old (config validation); keep twice that.
    static constexpr std::int64_t kSpanUs = 2'000'000;
    static constexpr std::size_t kMaxSamples = 512;
    static constexpr std::int64_t kMaxInterpolationGapUs = 50'000;

    // `timeUs` is the robot's timestamp of the write (NT time). A sample older than the newest one is ignored (ntcore
    // never delivers one); a second write with the same timestamp replaces the first.
    void add(std::int64_t timeUs, const RobotOrientation& orientation);

    // The orientation to use for a capture at `timeUs` (see above); all zeros before the first sample.
    RobotOrientation at(std::int64_t timeUs) const;

    bool empty() const { return samples_.empty(); }
    std::size_t size() const { return samples_.size(); }
    // The newest sample, nullptr while empty.
    const RobotOrientation* latest() const { return samples_.empty() ? nullptr : &samples_.back().orientation; }

private:
    struct Sample {
        std::int64_t timeUs = 0;
        RobotOrientation orientation;
    };
    std::deque<Sample> samples_;
};

}  // namespace wpiemu::bridges::vision::limelight
