#pragma once

// Reserved attachment names and the attachment reference shared by mechanisms (MechanismConfig) and
// zones (ZoneConfig). Resolution and validation live in mechanisms/attachment.h.

#include <string>

namespace wpiemu::sdk {

// Reserved attachment names. "chassis" is the parent id meaning the robot chassis; "root" is
// the link every mechanism and the chassis expose (its own origin/mount frame). Neither may be used as
// a mechanism instance id / non-root link name.
inline constexpr const char* kChassisParent = "chassis";
inline constexpr const char* kRootLink = "root";

// Where a frame hangs: a named link of a mechanism (or the chassis).
struct MechanismAttachment {
    std::string parentMechanismId = kChassisParent;  // "chassis" or "" = the chassis
    std::string parentLink = kRootLink;              // must exist on the parent; the chassis only has "root"

    bool onChassis() const { return parentMechanismId.empty() || parentMechanismId == kChassisParent; }
    bool operator==(const MechanismAttachment&) const = default;
};

}  // namespace wpiemu::sdk
