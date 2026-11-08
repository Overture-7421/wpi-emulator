#pragma once

// Cryptographically secure randomness from the operating system, for invite secrets and ephemeral keys.
//
// BCryptGenRandom on Windows, getentropy (getrandom as the fallback) on Linux, getentropy on macOS. There is no weak
// fallback: if the OS cannot supply entropy the call fails and the caller must not continue.

#include <cstddef>
#include <span>

#include "wpiemu/sdk/core/result.h"

namespace wpiemu::net {

// Fills the whole span with random bytes, or returns an IoError and leaves the content unspecified.
sdk::Status fillRandom(std::span<std::byte> out);

}  // namespace wpiemu::net
