#include <doctest/doctest.h>

#include "joystick_source.h"

using namespace wpiemu::app::runview;

TEST_CASE("the default gamepad backend, when compiled in, initialises and enumerates without a device") {
    const auto provider = createDefaultJoystickProvider();
    if (!provider) return;   // build without SDL2: keyboard only
    CHECK(provider->backendName() == QStringLiteral("SDL2"));
    provider->pump();
    for (const auto& source : provider->sources()) {
        const joy::RawInput raw = source->poll();   // any attached pad reports the WPILib Xbox layout
        CHECK(raw.axes.size() == joy::kAxisCount);
    }
}
