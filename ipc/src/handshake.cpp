#include "wpiemu/ipc/handshake.h"

#include <algorithm>

#include "wpiemu/ipc/binary_codec.h"

namespace wpiemu::ipc {

using namespace sdk;
using namespace sdk::proto;

Result<ServerHandshakeResult> performServerHandshake(MessageChannel& channel, const ServerHandshakeConfig& config,
                                                     std::chrono::milliseconds timeout) {
    if (config.sessionId == kUnspecified)
        return makeError(ErrorCode::InvalidArgument, "ServerHandshakeConfig::sessionId must be non-zero");

    Message first;
    Result<bool> got = receiveSkippingUnknown(channel, first, timeout);
    if (!got) return got.error();
    if (!got.value()) return makeError(ErrorCode::Timeout, "no Hello received");

    HelloAck ack;
    ack.sessionId = config.sessionId;
    ack.runEpoch = config.runEpoch;
    ack.clockMode = config.clockMode;
    ack.stepUs = config.stepUs;
    ack.discoveryOnly = config.discoveryOnly;
    ack.enableL3Hooks = config.enableL3Hooks;
    ack.watchdogPolicy = config.watchdogPolicy;

    const Hello* hello = std::get_if<Hello>(&first);
    if (hello == nullptr || hello->protocolVersion < kMinCompatibleProtocolVersion) {
        ack.accepted = false;
        ack.reason = hello == nullptr ? "the first message must be Hello"
                                      : "protocol version " + std::to_string(hello->protocolVersion) + " is too old";
        (void)channel.send(ack);  // best effort: the connection is being refused anyway
        return makeError(hello == nullptr ? ErrorCode::ProtocolError : ErrorCode::VersionMismatch, ack.reason);
    }

    ack.accepted = true;
    ack.protocolVersion = std::min(hello->protocolVersion, kProtocolVersion);
    ack.watchdogTimeoutMs = negotiateWatchdogTimeoutMs(hello->proposedWatchdogMs, config.preferredWatchdogMs);
    if (Status s = channel.send(ack); !s) return s.error();
    return ServerHandshakeResult{*hello, std::move(ack)};
}

Result<HelloAck> performClientHandshake(MessageChannel& channel, const Hello& hello, std::chrono::milliseconds timeout) {
    if (Status s = channel.send(hello); !s) return s.error();
    Message reply;
    Result<bool> got = receiveSkippingUnknown(channel, reply, timeout);
    if (!got) return got.error();
    if (!got.value()) return makeError(ErrorCode::Timeout, "no HelloAck received");
    const HelloAck* ack = std::get_if<HelloAck>(&reply);
    if (ack == nullptr) return makeError(ErrorCode::ProtocolError, "the first reply must be HelloAck");
    if (!ack->accepted) return makeError(ErrorCode::Unsupported, "emulator refused the connection: " + ack->reason);
    return *ack;
}

}  // namespace wpiemu::ipc
