#pragma once

// The codec every match peer speaks over its transport (the ipc binary codec), for the layers that may see net but not ipc:
// the game client app composes a MatchClient with it.

#include <memory>

#include "wpiemu/sdk/ipc/transport.h"

namespace wpiemu::net {

std::shared_ptr<const sdk::proto::IMessageCodec> makeMatchCodec();

}  // namespace wpiemu::net
