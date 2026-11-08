#include <doctest/doctest.h>

#include <vector>

#include "wpiemu/ipc/binary_codec.h"

using namespace wpiemu;
using namespace wpiemu::ipc;
using namespace wpiemu::sdk;
using namespace wpiemu::sdk::proto;

// Until the match codec lands (todo ipc-match-codec, which deletes this file), match and relay messages are refused on
// encode and arrive as unknown, skippable types on decode, so the HALSim stream behaves as it did before they existed.

TEST_CASE("codec placeholder: match messages are not encoded by the HALSim codec") {
    const BinaryMessageCodec codec;
    std::vector<std::byte> out;
    const Status status = codec.encode(Message{match::Join{}}, out);
    CHECK_FALSE(status.ok());
    CHECK(status.error().code == ErrorCode::Unsupported);
    CHECK(out.empty());
}

TEST_CASE("codec placeholder: match type numbers decode as skippable unknown types") {
    const BinaryMessageCodec codec;
    for (std::uint8_t type = 13; type <= 28; ++type) {
        const std::vector<std::byte> frame{std::byte{1}, std::byte{type}, std::byte{0}};
        const Result<Message> decoded = codec.decode(frame);
        CAPTURE(static_cast<int>(type));
        REQUIRE_FALSE(decoded.ok());
        CHECK(isSkippableDecodeError(decoded.error()));
    }
}
