// Runs the doctest cases against the REAL simulation HAL (wpiHal) inside this process: the extension
// logic, the real HALSIM_* binding and a scripted fake emulator, with a hand-written "robot" thread
// that uses the C HAL API the way TimedRobot does. Not run when WPILib is unavailable at configure time.
#define DOCTEST_CONFIG_IMPLEMENT
#include <doctest/doctest.h>

#include <cstdio>

#include "wpiemu/hal_abi/hal_includes.h"

int main(int argc, char** argv) {
    // No HALSIM_EXTENSIONS here: the extension under test is driven in-process.
    if (!wpiemu::hal_abi::initializeSimHal()) {
        std::fprintf(stderr, "HAL_Initialize failed\n");
        return 2;
    }
    doctest::Context context(argc, argv);
    const int result = context.run();
    HAL_Shutdown();
    return result;
}
