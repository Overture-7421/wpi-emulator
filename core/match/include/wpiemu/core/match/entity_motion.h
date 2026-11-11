#pragma once

// Velocity and flight state of the entities of a practice match, derived by the host from the positions the plugin publishes
// (the telemetry snapshot has no velocities). The tracker sees every wire frame the host builds (one per snapshot instant,
// shared by all clients), remembers each entity's previous position and fills the velocity and `ballistic` flag of the
// frame's EntityState with the state the entity has AT the frame's sim time:
//
//   * velocity = (position - previous position) / elapsed sim time between the two frames, from the unquantized positions
//     (the 1 cm quantization of the wire would make a 16 ms difference useless), corrected to the end of the interval for an
//     entity in free flight;
//   * ballistic = the vertical acceleration over the last two intervals is about -g (free flight, apex included). With only
//     one interval known the entity counts as ballistic when it moves vertically faster than kUnknownBallisticVzMps;
//   * an entity seen for the first time, after a gap longer than kMaxGapUs, or moving faster than kMaxPlausibleSpeedMps (a
//     teleport) has velocity 0 and is not ballistic (the next frame corrects it: the prediction error then triggers an upsert).
//
// Inputs are sim times and positions only, so the result is a pure function of the frame sequence.

#include <cstdint>
#include <map>
#include <span>
#include <vector>

#include "wpiemu/sdk/core/math.h"
#include "wpiemu/sdk/match/snapshot_messages.h"

namespace wpiemu::core::match {

class EntityMotionTracker {
public:
    static constexpr std::int64_t kMaxGapUs = 250'000;
    static constexpr double kBallisticToleranceG = 0.35;   // |az + g| below this fraction of g makes an entity fly
    static constexpr double kStayBallisticToleranceG = 0.8;   // ... and below this fraction keeps a flying one flying
    static constexpr double kUnknownBallisticVzMps = 0.3;
    static constexpr double kMaxPlausibleSpeedMps = 50.0;   // faster is a teleport or a recycled id: velocity unknown
    static constexpr double kKinkMps = 1.0;                 // a velocity change gravity does not explain: a collision or a bounce

    // `entities` (ascending id, positions already quantized) and `rawPositions` (metres, same order) describe one group at
    // `simTimeUs`; the velocity and ballistic fields of `entities` are filled in. Forgets entities that are gone.
    // `unsettled` (same order as `entities`) is set to 1 for an entity whose velocity is not yet to be trusted for ONE frame:
    // it appeared in this frame (when the group was tracked before), or a collision or bounce happened inside the interval
    // that ended at this frame, so the averaged velocity mixes the velocity before and after it. The velocity of the next
    // frame is clean, so the host holds the entity's update back by that one frame instead of sending it twice.
    void observe(std::uint16_t groupId, std::int64_t simTimeUs, std::span<const sdk::Vec3> rawPositions,
                 std::vector<sdk::match::EntityState>& entities, std::vector<std::uint8_t>& unsettled);
    void reset() { groups_.clear(); }

private:
    struct Track {
        std::uint32_t id = 0;
        sdk::Vec3 position;
        std::int64_t timeUs = 0;
        bool hasVelocity = false;   // averageVelocity (over the interval ending at timeUs) is known
        sdk::Vec3 averageVelocity;
        std::int64_t midUs = 0;     // middle of that interval
        bool unsettled = false;     // reported unsettled in that frame
        bool kinked = false;        // a collision or bounce was detected in that frame
        bool ballistic = false;     // classified as in free flight in that frame
    };

    std::map<std::uint16_t, std::vector<Track>> groups_;
};

}  // namespace wpiemu::core::match
