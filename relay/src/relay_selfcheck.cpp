#include "wpiemu/relay/relay_selfcheck.h"

#include <random>

#include "wpiemu/ipc/binary_codec.h"
#include "wpiemu/net/control_link.h"
#include "wpiemu/net/endpoint.h"
#include "wpiemu/net/enet_transport.h"

namespace wpiemu::relay {

namespace rl = sdk::match::relay;

namespace {

std::string randomRoomId() {
    std::random_device device;
    std::mt19937_64 rng((static_cast<std::uint64_t>(device()) << 32) ^ device());
    const std::string alphabet = rl::kRoomIdAlphabet;
    std::string id;
    for (std::size_t i = 0; i < rl::kRelayRoomIdChars; ++i) id.push_back(alphabet[rng() % alphabet.size()]);
    return id;
}

}  // namespace

SelfCheckResult runSelfCheck(const std::string& host, std::uint16_t port, std::chrono::milliseconds timeout) {
    SelfCheckResult result;
    const auto deadline = std::chrono::steady_clock::now() + timeout;

    net::NetEndpoint endpoint;
    endpoint.host = host;
    endpoint.port = port;
    net::EnetClientTransport client;
    sdk::Result<std::unique_ptr<sdk::proto::ITransport>> connected = client.connect(endpoint.format(), timeout);
    if (!connected) {
        const sdk::Error& error = connected.error();
        result.detail = "connect to " + endpoint.format() + " failed: " + error.message;
        if (error.code == sdk::ErrorCode::Timeout) result.failure = SelfCheckFailure::Unreachable;
        else if (error.code == sdk::ErrorCode::IoError && error.message.find("cannot resolve") != std::string::npos)
            result.failure = SelfCheckFailure::Resolve;
        else result.failure = SelfCheckFailure::Connect;
        return result;
    }
    std::unique_ptr<sdk::proto::ITransport> transport = std::move(connected).value();
    const std::string described = transport->describe();  // "enet://<ip>:<port>"
    if (described.rfind("enet://", 0) == 0) result.resolvedAddress = described.substr(7);
    net::IControlLink* control = net::controlLinkOf(*transport);
    if (control == nullptr) {
        result.detail = "the transport has no control channel";
        result.failure = SelfCheckFailure::Connect;
        return result;
    }

    const ipc::BinaryMessageCodec codec(net::kMaxControlFrameBytes);
    rl::JoinRoom join;
    join.roomId = randomRoomId();
    std::vector<std::byte> frame;
    if (const sdk::Status encoded = codec.encode(join, frame); !encoded) {
        result.detail = "cannot encode JoinRoom: " + encoded.error().message;
        result.failure = SelfCheckFailure::Protocol;
        return result;
    }
    if (const sdk::Status sent = control->sendControl(frame); !sent) {
        result.detail = "cannot send JoinRoom: " + sent.error().message;
        result.failure = SelfCheckFailure::HungUp;
        return result;
    }

    std::vector<std::byte> reply;
    while (std::chrono::steady_clock::now() < deadline) {
        const sdk::Result<bool> got = control->receiveControl(reply, sdk::proto::timeLeft(deadline));
        if (!got) {
            result.detail = "connection lost before the relay answered: " + got.error().message;
            result.failure = SelfCheckFailure::HungUp;
            return result;
        }
        if (!got.value()) continue;
        const sdk::Result<sdk::proto::Message> decoded = codec.decode(reply);
        if (!decoded) {
            result.detail = "undecodable reply: " + decoded.error().message;
            result.failure = SelfCheckFailure::Protocol;
            return result;
        }
        const auto* joinResult = std::get_if<rl::JoinResult>(&decoded.value());
        if (joinResult == nullptr) {
            result.detail = "unexpected reply message";
            result.failure = SelfCheckFailure::Protocol;
            return result;
        }
        result.answer = *joinResult;
        if (joinResult->ok) {
            result.failure = SelfCheckFailure::Protocol;
            result.detail = "the relay accepted a join to a room that cannot exist";
            return result;
        }
        result.ok = true;
        result.detail = "relay answered";
        return result;
    }
    result.detail = "no answer within " + std::to_string(timeout.count()) + " ms";
    result.failure = SelfCheckFailure::NoAnswer;
    return result;
}

}  // namespace wpiemu::relay
