#pragma once

#include <cstdint>

#include "wpiemu/sdk/ipc/messages.h"

namespace wpiemu::ipc {

// Wire protocol version negotiated during the extension <-> app handshake. The message layout and
// the version constant are owned by the SDK (wpiemu/sdk/ipc/messages.h); this module implements
// transports and codecs for them.
inline constexpr std::uint32_t kProtocolVersion = sdk::proto::kProtocolVersion;

std::uint32_t protocolVersion();

}  // namespace wpiemu::ipc
