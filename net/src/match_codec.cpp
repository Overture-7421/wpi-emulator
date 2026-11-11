#include "wpiemu/net/match_codec.h"

#include "wpiemu/ipc/binary_codec.h"

namespace wpiemu::net {

std::shared_ptr<const sdk::proto::IMessageCodec> makeMatchCodec() { return std::make_shared<const ipc::BinaryMessageCodec>(); }

}  // namespace wpiemu::net
