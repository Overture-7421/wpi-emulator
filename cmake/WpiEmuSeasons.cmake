include_guard(GLOBAL)

if(WPIEMU_EXTENSION_ONLY)
    return()
endif()
include(ExternalProject)

foreach(_season IN LISTS WPIEMU_SEASONS)
    wpiemu_season_value(_version "${_season}" wpilib version)   # fails configure for a season without a pack
    if(_season STREQUAL WPIEMU_WPILIB_SEASON)
        continue()
    endif()
    set(_build "${CMAKE_BINARY_DIR}/seasons/${_season}")
    set(_stage "${CMAKE_BINARY_DIR}/seasons/${_season}/stage")
    set(_args
        "-DWPIEMU_WPILIB_SEASON:STRING=${_season}"
        "-DWPIEMU_EXTENSION_ONLY:BOOL=ON"
        "-DWPIEMU_BUILD_TESTS:BOOL=${WPIEMU_BUILD_TESTS}"
        "-DWPIEMU_WPILIB_DOWNLOAD:BOOL=${WPIEMU_WPILIB_DOWNLOAD}"
        "-DWPIEMU_REQUIRE_REAL_HAL:BOOL=ON"
        "-DWPIEMU_WARNINGS_AS_ERRORS:BOOL=${WPIEMU_WARNINGS_AS_ERRORS}"
        "-DWPIEMU_PACKAGE_WPILIB_RUNTIME:BOOL=${WPIEMU_PACKAGE_WPILIB_RUNTIME}"
        "-DCMAKE_INSTALL_PREFIX:PATH=${_stage}"
        "-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}"
        "-DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}"
        "-DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}")
    # Extracted artifacts cannot be shared across seasons. An optional root is specific to the extra season.
    set(WPIEMU_WPILIB_ROOT_${_season} "" CACHE PATH "WPILib artifacts for the ${_season} extension sub-build")
    list(APPEND _args "-DWPIEMU_WPILIB_ROOT:PATH=${WPIEMU_WPILIB_ROOT_${_season}}")
    foreach(_setting IN ITEMS CMAKE_OSX_ARCHITECTURES CMAKE_OSX_DEPLOYMENT_TARGET CMAKE_TOOLCHAIN_FILE)
        if(DEFINED ${_setting} AND NOT "${${_setting}}" STREQUAL "")
            string(REPLACE ";" "|" _value "${${_setting}}")
            list(APPEND _args "-D${_setting}:STRING=${_value}")
        endif()
    endforeach()
    if(WPIEMU_BUILD_TESTS)
        set(_targets wpiemu_halsim_ext_hal_tests)
    else()
        set(_targets halsim_wpiemu)
    endif()
    ExternalProject_Add(wpiemu_season_${_season}
        SOURCE_DIR "${CMAKE_SOURCE_DIR}"
        BINARY_DIR "${_build}"
        LIST_SEPARATOR |
        CMAKE_ARGS ${_args}
        BUILD_COMMAND "${CMAKE_COMMAND}" --build <BINARY_DIR> --config $<CONFIG> --target ${_targets}
        INSTALL_COMMAND "${CMAKE_COMMAND}" --install <BINARY_DIR> --config $<CONFIG> --component Runtime
        BUILD_ALWAYS TRUE)
    if(TARGET wpiemu_season_e2e_tests)
        add_dependencies(wpiemu_season_e2e_tests wpiemu_season_${_season})
    endif()
    if(WPIEMU_BUILD_TESTS)
        add_test(NAME halsim_ext.season_${_season}
            COMMAND "${CMAKE_CTEST_COMMAND}" --test-dir "${_build}" -C $<CONFIG>
                -R "^halsim_ext.hal_integration$" --output-on-failure)
        set_tests_properties(halsim_ext.season_${_season} PROPERTIES TIMEOUT 150)
    endif()
    if(APPLE)
        set(_layout "wpiemu_app.app/Contents/PlugIns/wpiemu/${_season}")
    elseif(WIN32)
        set(_layout "extensions/${_season}")
    else()
        set(_layout "lib/wpiemu/${_season}")
    endif()
    install(DIRECTORY "${_stage}/${_layout}/" DESTINATION "${_layout}" COMPONENT Runtime)
    # A build-tree app uses the same year-qualified candidates as an installed app.
    if(CMAKE_CONFIGURATION_TYPES)
        set(_runtime_root "${CMAKE_BINARY_DIR}/bin/$<CONFIG>")
    else()
        set(_runtime_root "${CMAKE_BINARY_DIR}/bin")
    endif()
    add_custom_command(TARGET wpiemu_season_${_season} POST_BUILD
        COMMAND "${CMAKE_COMMAND}" "-DSOURCE=${_stage}/${_layout}"
            "-DDESTINATION=${_runtime_root}/${_season}"
            -P "${CMAKE_CURRENT_LIST_DIR}/StageSeasonExtension.cmake")
endforeach()
