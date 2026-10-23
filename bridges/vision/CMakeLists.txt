# Vision bridges: simulated Limelight / PhotonVision coprocessors presented to the robot through NetworkTables.
# `wpiemu_bridges_vision` depends only on wpiemu_sdk (the NetworkTables seam is INtBackend; tests use
# FakeNtBackend). The real ntcore binding is compiled in when ntcore was found
# (halsim_ext/cmake/WpiEmuWpilib.cmake, WPIEMU_NTCORE_FOUND).
add_library(wpiemu_bridges_vision STATIC
    src/registration.cpp
    src/limelight_bridge.cpp
    src/photon_bridge.cpp
    src/photon_encoding.cpp)
# The PhotonVision catalog: every season pack's verified PhotonLib versions and their wire schema
# (vendors.photonvision in seasons/<year>/season.json), plus the default of the season this build targets.
if(WPIEMU_WPILIB_SEASON)
    set(_wpiemu_photon_season "${WPIEMU_WPILIB_SEASON}")
else()
    wpiemu_default_season(_wpiemu_photon_season)
endif()
function(wpiemu_photon_schema_enum out schema)
    if(schema STREQUAL "photon-us")
        set(${out} "Microseconds" PARENT_SCOPE)
    elseif(schema STREQUAL "photon-ns")
        set(${out} "Nanoseconds" PARENT_SCOPE)
    else()
        message(FATAL_ERROR "wpiemu: unknown PhotonVision schema \"${schema}\" in a season pack")
    endif()
endfunction()
wpiemu_season_years(_wpiemu_photon_years)
set(_wpiemu_photon_rules "")
set(_wpiemu_photon_seen "")
foreach(_year IN LISTS _wpiemu_photon_years)
    wpiemu_season_value(_schema "${_year}" vendors photonvision schema)
    wpiemu_photon_schema_enum(_enum "${_schema}")
    wpiemu_season_array(_verified "${_year}" vendors photonvision verified)
    foreach(_version IN LISTS _verified)
        if(_version IN_LIST _wpiemu_photon_seen)
            message(FATAL_ERROR "wpiemu: PhotonLib ${_version} is verified by two season packs")
        endif()
        list(APPEND _wpiemu_photon_seen "${_version}")
        string(APPEND _wpiemu_photon_rules "        {\"${_version}\", PhotonSchema::${_enum}},\n")
    endforeach()
endforeach()
wpiemu_season_value(_wpiemu_photon_default_version "${_wpiemu_photon_season}" vendors photonvision defaultVersion)
wpiemu_season_value(_wpiemu_photon_default_schema "${_wpiemu_photon_season}" vendors photonvision schema)
wpiemu_photon_schema_enum(_wpiemu_photon_default_enum "${_wpiemu_photon_default_schema}")
file(CONFIGURE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/photon_catalog.cpp" @ONLY CONTENT
"// Generated from seasons/*/season.json by bridges/vision/CMakeLists.txt. Do not edit.
#include \"wpiemu/bridges/vision/photon_encoding.h\"

namespace wpiemu::bridges::vision {

const std::vector<PhotonVersionRule>& photonVersionRules() {
    static const std::vector<PhotonVersionRule> rules = {
@_wpiemu_photon_rules@    };
    return rules;
}

std::string_view photonDefaultVersion() { return \"@_wpiemu_photon_default_version@\"; }
PhotonSchema photonDefaultSchema() { return PhotonSchema::@_wpiemu_photon_default_enum@; }

}  // namespace wpiemu::bridges::vision
")
target_sources(wpiemu_bridges_vision PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/photon_catalog.cpp")
add_library(wpiemu::bridges_vision ALIAS wpiemu_bridges_vision)
target_include_directories(wpiemu_bridges_vision PUBLIC include)
target_link_libraries(wpiemu_bridges_vision PUBLIC wpiemu_sdk)
wpiemu_target_defaults(wpiemu_bridges_vision)

# Real ntcore binding, only when ntcore (and the HAL) were found by halsim_ext/cmake/WpiEmuWpilib.cmake. It has no
# link-time dependency on ntcore: it attaches to the module the robot process loaded (see nt_binding.h), so only the
# headers are used here. Without ntcore the bridges are registered inert (bridges/builtin_bridges.cpp).
if(TARGET wpiemu_wpilib_ntcore AND TARGET wpiemu_wpilib_hal)
    target_sources(wpiemu_bridges_vision PRIVATE src/ntcore_backend.cpp)
    get_target_property(_wpiemu_nt_include wpiemu_wpilib_ntcore INTERFACE_INCLUDE_DIRECTORIES)
    # SYSTEM: WPILib headers are third-party and not warning-clean under /W4.
    target_include_directories(wpiemu_bridges_vision SYSTEM PRIVATE ${_wpiemu_nt_include})
    target_link_libraries(wpiemu_bridges_vision PUBLIC wpiemu_hal_abi PRIVATE ${CMAKE_DL_LIBS})
    target_compile_definitions(wpiemu_bridges_vision PUBLIC WPIEMU_HAS_VISION_NT=1 WPIEMU_HAL_ABI_${WPIEMU_HAL_ABI_ID}=1)
    message(STATUS "bridges/vision: ntcore binding enabled")
endif()
