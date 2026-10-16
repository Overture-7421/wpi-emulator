#include <doctest/doctest.h>

#include "wpiemu/ipc/protocol.h"

TEST_CASE("ipc exposes a protocol version") {
    CHECK(wpiemu::ipc::protocolVersion() == wpiemu::ipc::kProtocolVersion);
}
