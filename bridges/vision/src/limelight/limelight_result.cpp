#include "wpiemu/bridges/vision/limelight_result.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

namespace wpiemu::bridges::vision::limelight {

namespace {

using Observation = sdk::FiducialObservation;
using Tags = std::vector<const Observation*>;

static_assert(kRawFiducialsStride == kBotposeTagStride, "rawfiducials and botpose share one per-tag layout");

struct TagStats {
    std::size_t count = 0;
    double spanM = 0.0;
    double avgDistM = 0.0;
    double avgAreaPct = 0.0;
};

Vec3d positionOf(const sdk::Pose3& pose) { return {pose.position.x, pose.position.y, pose.position.z}; }

TagStats statsOf(const Tags& tags) {
    TagStats stats;
    stats.count = tags.size();
    if (tags.empty()) return stats;
    double distSum = 0.0;
    double areaSum = 0.0;
    for (std::size_t i = 0; i < tags.size(); ++i) {
        distSum += tags[i]->distanceM;
        areaSum += tags[i]->areaPct;
        for (std::size_t j = i + 1; j < tags.size(); ++j) {
            const double apart = distance(positionOf(tags[i]->fieldToTag), positionOf(tags[j]->fieldToTag));
            stats.spanM = std::max(stats.spanM, apart);
        }
    }
    stats.avgDistM = distSum / static_cast<double>(tags.size());
    stats.avgAreaPct = areaSum / static_cast<double>(tags.size());
    return stats;
}

// One tag's block of a botpose or rawfiducials array. The distance to the robot uses the mount the device believes in.
std::array<double, kRawFiducialsStride> tagBlock(const Observation& o, const Pose3d& assumedRobotToCamera) {
    const Pose3d robotToTag = assumedRobotToCamera * toPose3d(o.cameraToTagBest);
    std::array<double, kRawFiducialsStride> block{};
    block[kTagId] = static_cast<double>(o.id);
    block[kTagTxnc] = radToDeg(o.yawRad);
    block[kTagTync] = radToDeg(o.pitchRad);
    block[kTagTa] = o.areaPct;
    block[kTagDistToCameraM] = o.distanceM;
    block[kTagDistToRobotM] = length(robotToTag.position);
    block[kTagAmbiguity] = o.ambiguity;
    return block;
}

std::vector<double> tagBlocks(const Tags& tags, const Pose3d& assumedRobotToCamera) {
    std::vector<double> out;
    out.reserve(tags.size() * kRawFiducialsStride);
    for (const Observation* tag : tags) {
        const auto block = tagBlock(*tag, assumedRobotToCamera);
        out.insert(out.end(), block.begin(), block.end());
    }
    return out;
}

std::vector<double> assemble(const PoseDeg& pose, double latencyMs, const TagStats& stats,
                             const std::vector<double>& blocks) {
    std::array<double, kBotposeHeaderSize> header{};
    header[kBotposeX] = pose.x;
    header[kBotposeY] = pose.y;
    header[kBotposeZ] = pose.z;
    header[kBotposeRollDeg] = pose.rollDeg;
    header[kBotposePitchDeg] = pose.pitchDeg;
    header[kBotposeYawDeg] = pose.yawDeg;
    header[kBotposeLatencyMs] = latencyMs;
    header[kBotposeTagCount] = static_cast<double>(stats.count);
    header[kBotposeTagSpanM] = stats.spanM;
    header[kBotposeAvgTagDistM] = stats.avgDistM;
    header[kBotposeAvgTagAreaPct] = stats.avgAreaPct;
    std::vector<double> out;
    out.reserve(header.size() + blocks.size());
    out.insert(out.end(), header.begin(), header.end());
    out.insert(out.end(), blocks.begin(), blocks.end());
    return out;
}

// The three field flavours of one estimate (blue origin, red origin, field centre). Without tags there is no pose to
// move: all three are the same all-zero array.
struct Flavours {
    std::vector<double> blue;
    std::vector<double> red;
    std::vector<double> centered;
};

Flavours flavours(const PoseDeg& blue, double latencyMs, const Tags& used, const Pose3d& assumedRobotToCamera) {
    const TagStats stats = statsOf(used);
    const std::vector<double> blocks = tagBlocks(used, assumedRobotToCamera);
    Flavours out;
    if (used.empty()) {
        out.blue = assemble(PoseDeg{}, latencyMs, stats, blocks);
        out.red = out.blue;
        out.centered = out.blue;
        return out;
    }
    out.blue = assemble(blue, latencyMs, stats, blocks);
    out.red = assemble(toWpiRed(blue), latencyMs, stats, blocks);
    out.centered = assemble(toFieldCentered(blue), latencyMs, stats, blocks);
    return out;
}

bool contains(std::span<const int> ids, int id) { return std::find(ids.begin(), ids.end(), id) != ids.end(); }

bool isFinite(const PoseDeg& pose) {
    return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.z) && std::isfinite(pose.rollDeg) &&
           std::isfinite(pose.pitchDeg) && std::isfinite(pose.yawDeg);
}

// The tags of the frame's multi-tag solution (in frame order) when that solution can serve as MegaTag1: it is valid and
// none of its tags is filtered out. Empty otherwise.
Tags multiTagSolutionTags(const sdk::VisionFrame& frame, const Tags& allowed) {
    if (!frame.multiTag.valid || frame.multiTag.tagIds.empty()) return {};
    for (const int id : frame.multiTag.tagIds) {
        const bool allowedId =
            std::any_of(allowed.begin(), allowed.end(), [id](const Observation* o) { return o->id == id; });
        if (!allowedId) return {};
    }
    Tags used;
    for (const Observation* o : allowed)
        if (contains(frame.multiTag.tagIds, o->id)) used.push_back(o);
    return used;
}

}  // namespace

LimelightResult computeResult(const sdk::VisionFrame& frame, const ResultSettings& settings) {
    LimelightResult result;

    const std::int64_t totalUs = std::max<std::int64_t>(frame.latencyUs, 0);
    const std::int64_t captureUs = std::clamp<std::int64_t>(frame.captureLatencyUs, 0, totalUs);
    result.clMs = static_cast<double>(captureUs) / 1000.0;
    result.tlMs = static_cast<double>(totalUs - captureUs) / 1000.0;
    const double latencyMs = static_cast<double>(totalUs) / 1000.0;
    const Pose3d& assumed = settings.assumedRobotToCamera;

    Tags detected;
    if (!settings.driverMode)
        for (const Observation& o : frame.observations) detected.push_back(&o);

    result.rawFiducials = tagBlocks(detected, assumed);

    const Observation* primary = nullptr;
    if (settings.priorityId >= 0) {
        for (const Observation* o : detected)
            if (o->id == settings.priorityId) {
                primary = o;
                break;
            }
    } else if (!detected.empty()) {
        primary = detected.front();
    }
    if (primary != nullptr) {
        result.hasTarget = true;
        result.txDeg = radToDeg(primary->yawRad);
        result.tyDeg = radToDeg(primary->pitchRad);
        result.taPct = primary->areaPct;
        result.tid = static_cast<double>(primary->id);
    }

    Tags allowed;
    for (const Observation* o : detected)
        if (settings.idFilter.empty() || contains(settings.idFilter, o->id)) allowed.push_back(o);

    // MegaTag1
    Tags used1;
    PoseDeg pose1;
    if (!allowed.empty()) {
        used1 = multiTagSolutionTags(frame, allowed);
        Pose3d fieldToCamera;
        if (!used1.empty()) {
            fieldToCamera = toPose3d(frame.multiTag.fieldToCamera);
        } else {
            const Observation& tag = *allowed.front();
            used1 = {&tag};
            fieldToCamera = fieldToCameraFromTag(toPose3d(tag.fieldToTag), toPose3d(tag.cameraToTagBest));
        }
        pose1 = toPoseDeg(megaTag1RobotPose(fieldToCamera, assumed));
        if (!isFinite(pose1)) {  // a degenerate solve is "no estimate", never NaN on the robot's side
            pose1 = PoseDeg{};
            used1.clear();
        }
    }
    Flavours mt1 = flavours(pose1, latencyMs, used1, assumed);
    result.botposeWpiBlue = std::move(mt1.blue);
    result.botposeWpiRed = std::move(mt1.red);
    result.botpose = std::move(mt1.centered);

    // MegaTag2
    Tags used2;
    PoseDeg pose2;
    if (!allowed.empty()) {
        const Observation& tag = *allowed.front();
        pose2 = megaTag2RobotPose(toPose3d(tag.fieldToTag), toPose3d(tag.cameraToTagBest).position, assumed,
                                  settings.orientation);
        if (isFinite(pose2)) used2 = allowed;
    }
    Flavours mt2 = flavours(pose2, latencyMs, used2, assumed);
    result.botposeOrbWpiBlue = std::move(mt2.blue);
    result.botposeOrbWpiRed = std::move(mt2.red);
    result.botposeOrb = std::move(mt2.centered);

    return result;
}

}  // namespace wpiemu::bridges::vision::limelight
