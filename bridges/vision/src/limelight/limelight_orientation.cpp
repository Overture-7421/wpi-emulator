#include "wpiemu/bridges/vision/limelight_orientation.h"

#include <algorithm>

namespace wpiemu::bridges::vision::limelight {

void OrientationHistory::add(std::int64_t timeUs, const RobotOrientation& orientation) {
    if (!samples_.empty()) {
        if (timeUs < samples_.back().timeUs) return;
        if (timeUs == samples_.back().timeUs) {
            samples_.back().orientation = orientation;
            return;
        }
    }
    samples_.push_back({timeUs, orientation});

    // Drop what no capture can ask for any more, but keep the newest sample at or before the cutoff: it still holds.
    const std::int64_t cutoffUs = timeUs - kSpanUs;
    while (samples_.size() > 2 && samples_[1].timeUs <= cutoffUs) samples_.pop_front();
    while (samples_.size() > kMaxSamples) samples_.pop_front();
}

RobotOrientation OrientationHistory::at(std::int64_t timeUs) const {
    const auto after = std::upper_bound(samples_.begin(), samples_.end(), timeUs,
                                        [](std::int64_t t, const Sample& sample) { return t < sample.timeUs; });
    if (after == samples_.begin()) return RobotOrientation{};
    const Sample& before = *(after - 1);
    if (after == samples_.end() || after->timeUs - before.timeUs > kMaxInterpolationGapUs) return before.orientation;

    const double fraction =
        static_cast<double>(timeUs - before.timeUs) / static_cast<double>(after->timeUs - before.timeUs);
    RobotOrientation out = before.orientation;  // the rates are held, only the angles are interpolated
    out.yawDeg = lerpAngleDegrees(before.orientation.yawDeg, after->orientation.yawDeg, fraction);
    out.pitchDeg = lerpAngleDegrees(before.orientation.pitchDeg, after->orientation.pitchDeg, fraction);
    out.rollDeg = lerpAngleDegrees(before.orientation.rollDeg, after->orientation.rollDeg, fraction);
    return out;
}

}  // namespace wpiemu::bridges::vision::limelight
