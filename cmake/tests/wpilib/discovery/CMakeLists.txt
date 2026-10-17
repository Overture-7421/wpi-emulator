# Mirror the real halsim_ext child-directory scope before configuring sibling bridges.
include("${WPIEMU_FIXTURE_REPO}/halsim_ext/cmake/WpiEmuWpilib.cmake")
if(NOT WPIEMU_WPILIB_FOUND)
    message(FATAL_ERROR "Fixture WPILib not discovered")
endif()
