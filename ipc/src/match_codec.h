#pragma once

// The match and relay messages (MessageType 13..28) for BinaryMessageCodec: their `fields` functions live in
// match_codec.cpp and use the same wire::Writer/Reader (and so the same hostile-input rules) as the HALSim messages.

#include <cstddef>
#include <span>
#include <vector>

#include "wpiemu/sdk/core/result.h"
#include "wpiemu/sdk/ipc/messages.h"

namespace wpiemu::ipc::detail {

// Appends the body (no frame header) of a match or relay message. False when a field exceeds the wire limits (a list
// over its cap, a string over its limit, an invalid room id); the caller discards the output.
bool encodeMatchBody(const sdk::proto::Message& message, std::vector<std::byte>& out);

// Decodes the body of `type` (a match or relay type). ProtocolError for malformed content, Unsupported for any other type.
sdk::Result<sdk::proto::Message> decodeMatchBody(sdk::proto::MessageType type, std::span<const std::byte> body);

}  // namespace wpiemu::ipc::detail
