#include <doctest/doctest.h>

#include <cstdint>
#include <vector>

#include "wpiemu/bridges/vision/null_nt_api.h"
#include "wpiemu/bridges/vision/vision_bridges.h"
#include "wpiemu/sdk/devices/device_bridge.h"

namespace {

using namespace wpiemu::bridges::vision;

TEST_CASE("vision bridges: registration succeeds on an empty registry") {
    wpiemu::sdk::DeviceBridgeRegistry registry;
    CHECK(registerVisionBridges(registry).ok());
}

TEST_CASE("null nt api: every call is a harmless no-op and says why it is unavailable") {
    NullNtApi nt("test reason");
    CHECK_FALSE(nt.available());
    CHECK(nt.unavailableReason() == "test reason");
    CHECK(nt.nowUs() == 0);
    CHECK_FALSE(nt.publish("/a", NtType::Double, {}));
    CHECK_FALSE(nt.setDouble({}, 1, 1.0));
    CHECK_FALSE(nt.setTopicProperty("/a", "p", "1"));
    CHECK_FALSE(nt.subscribe("/a", kDefaultNtQueueDepth));
    CHECK_FALSE(nt.hasForeignPublisher("/a"));
    std::vector<NtValue> values;
    nt.read({}, values);
    CHECK(values.empty());
    std::vector<NtTopicInfo> topics;
    nt.topics("/", topics);
    CHECK(topics.empty());
}

}  // namespace
