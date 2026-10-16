#include <doctest/doctest.h>

#include "wpiemu/plugins/_template/plugin_info.h"

TEST_CASE("template plugin identifies itself") {
    CHECK(wpiemu::plugins::_template::pluginId() == "_template");
}
