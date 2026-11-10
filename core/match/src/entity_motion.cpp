#include "wpiemu/core/match/entity_motion.h"

#include <algorithm>
#include <cmath>

#include "wpiemu/sdk/match/entity_prediction.h"
#include "wpiemu/sdk/match/quantize.h"

namespace wpiemu::core::match {

namespace {

constexpr double kGravityMps2 = static_cast<double>(sdk::match::kEntityGravityCmps2) / 100.0;

// Velocity in cm/s is a metres-per-second value in the same x100 fixed point as a position in centimetres.
std::int16_t velocityCms(double mps) { return sdk::match::quantizeCentiI16(mps); }

}  // namespace

void EntityMotionTracker::observe(std::uint16_t groupId, std::int64_t simTimeUs, std::span<const sdk::Vec3> rawPositions,
                                  std::vector<sdk::match::EntityState>& entities) {
    std::vector<Track>& previous = groups_[groupId];
    std::vector<Track> next;
    next.reserve(entities.size());
    std::size_t j = 0;
    for (std::size_t i = 0; i < entities.size() && i < rawPositions.size(); ++i) {
        sdk::match::EntityState& e = entities[i];
        while (j < previous.size() && previous[j].id < e.id) ++j;
        const Track* old = j < previous.size() && previous[j].id == e.id ? &previous[j] : nullptr;

        Track t;
        t.id = e.id;
        t.position = rawPositions[i];
        t.timeUs = simTimeUs;
        if (old != nullptr && simTimeUs > old->timeUs && simTimeUs - old->timeUs <= kMaxGapUs) {
            const double dt = static_cast<double>(simTimeUs - old->timeUs) / 1e6;
            const double vx = (static_cast<double>(t.position.x) - old->position.x) / dt;
            const double vy = (static_cast<double>(t.position.y) - old->position.y) / dt;
            const double vz = (static_cast<double>(t.position.z) - old->position.z) / dt;
            t.hasVelocity = true;
            t.averageVelocity = sdk::Vec3(static_cast<float>(vx), static_cast<float>(vy), static_cast<float>(vz));
            t.midUs = old->timeUs + (simTimeUs - old->timeUs) / 2;

            // Faster than anything physical is a teleport or a recycled id, not a velocity to extrapolate.
            const bool plausible = vx * vx + vy * vy + vz * vz <= kMaxPlausibleSpeedMps * kMaxPlausibleSpeedMps;
            if (!plausible) {
                t.hasVelocity = false;
                next.push_back(t);
                continue;
            }

            bool ballistic = false;
            if (old->hasVelocity && t.midUs > old->midUs) {
                const double az = (vz - old->averageVelocity.z) / (static_cast<double>(t.midUs - old->midUs) / 1e6);
                ballistic = std::abs(az + kGravityMps2) < kBallisticToleranceG * kGravityMps2;
            } else {
                ballistic = std::abs(vz) > kUnknownBallisticVzMps;
            }
            e.ballistic = ballistic;
            e.vxCms = velocityCms(vx);
            e.vyCms = velocityCms(vy);
            e.vzCms = velocityCms(ballistic ? vz - kGravityMps2 * dt / 2.0 : vz);
        }
        next.push_back(t);
    }
    previous = std::move(next);
}

}  // namespace wpiemu::core::match
