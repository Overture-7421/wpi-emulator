// Built when the WPILib HAL was not found at configure time (see cmake/WpiEmuWpilib.cmake). Loading
// this in a robot process is a broken installation, so it says so loudly instead of silently
// leaving the robot unsynchronised.
#include <cstdio>

#if defined(_WIN32)
#define WPIEMU_EXPORT __declspec(dllexport)
#else
#define WPIEMU_EXPORT __attribute__((visibility("default")))
#endif

extern "C" WPIEMU_EXPORT int HALSIM_InitExtension(void) {
    std::fprintf(stderr,
                 "[wpiemu] ERROR extension: this halsim_wpiemu was built without the WPILib HAL; reconfigure with "
                 "-DWPIEMU_WPILIB_ROOT=<wpilib maven cache or extracted HAL> and rebuild\n");
    return -1;
}
