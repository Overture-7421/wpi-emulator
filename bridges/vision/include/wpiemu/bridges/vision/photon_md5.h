#pragma once

// MD5 (RFC 1321), as much of it as the PhotonVision wire contract needs: a PhotonVision message UUID is the
// MD5 of its schema text (PhotonVision photon-serde README, v2026.3.4), so the bridge checks every profile
// against its own schema text before it writes a single byte (photon_profile.h).

#include <string>
#include <string_view>

namespace wpiemu::bridges::vision {

// The digest of `data` as 32 lower-case hexadecimal digits.
std::string md5Hex(std::string_view data);

}  // namespace wpiemu::bridges::vision
