#pragma once

// The physics user tag of a robot's shapes (ShapeDesc::userTag, returned in ContactEvent, SensorEvent and RayHit), so a
// contact or a sensor visit can be attributed to a robot of a practice match. Defined once here; the runtime stamps it
// on every remote robot and a game plugin that wants to know who pushed or entered reads it with the functions below.
//
//   bits 63..56  kRobotTagMarker (0xFE)     reserved for robots; a game plugin's own tags must not use this top byte
//   bits 55..16  0
//   bits 15..0   ParticipantId
//
// Collision-free with the existing schemes: tag 0 means "untagged" and has top byte 0, and the 2026 plugin's makeTag packs
// its entity kind (1..3) into the same top byte, so a plugin kind would have to be 0xFE to collide.

#include <cstdint>

#include "wpiemu/sdk/match/match_protocol.h"

namespace wpiemu::sdk {

inline constexpr std::uint64_t kRobotTagMarker = 0xFE;
inline constexpr int kRobotTagMarkerShift = 56;

constexpr std::uint64_t makeRobotTag(match::ParticipantId participant) {
    return (kRobotTagMarker << kRobotTagMarkerShift) | static_cast<std::uint64_t>(participant);
}

// True for exactly the values makeRobotTag produces.
constexpr bool isRobotTag(std::uint64_t tag) {
    return (tag >> 16) == (kRobotTagMarker << (kRobotTagMarkerShift - 16));
}

// The participant of a robot tag; match::kNoParticipant for any other tag.
constexpr match::ParticipantId robotTagParticipant(std::uint64_t tag) {
    return isRobotTag(tag) ? static_cast<match::ParticipantId>(tag & 0xFFFFu) : match::kNoParticipant;
}

}  // namespace wpiemu::sdk
