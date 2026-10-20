#pragma once

namespace wpiemu::sdk {

// SDK API version. The SDK stays at 1.0 until the first release (docs/CONTRIBUTING.md): there is no compatibility
// history, saved files and wire peers from other revisions are simply refused.
//  * Major: plugins declare the major they target (GameMetadata::minSdkMajor).
//  * Minor: plugins may require a minimum minor (GameMetadata::minSdkMinor).
inline constexpr int kSdkVersionMajor = 1;
inline constexpr int kSdkVersionMinor = 0;

}  // namespace wpiemu::sdk