# Qt-free orchestration of one robot run (config -> session -> IPC -> launcher). Linked by the Qt shell
# (RealRuntimeHost) and by the opt-in end-to-end test; contains no widgets and no season knowledge.
find_package(Threads REQUIRED)
add_library(wpiemu_app_host STATIC
    robot_session.cpp)
target_include_directories(wpiemu_app_host PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/..")
target_link_libraries(wpiemu_app_host PUBLIC wpiemu_core wpiemu_ipc wpiemu_sdk Threads::Threads)
wpiemu_target_defaults(wpiemu_app_host)

if(WPIEMU_BUILD_TESTS)
    # Opt-in end-to-end tests with samples/swerve_robot and samples/vision_robot (label e2e).
    add_executable(wpiemu_e2e_tests
        tests/e2e_test.cpp
        tests/e2e_vision_test.cpp)
    target_link_libraries(wpiemu_e2e_tests PRIVATE wpiemu_app_host wpiemu_plugins_builtin wpiemu_seasons_builtin)
    target_compile_definitions(wpiemu_e2e_tests PRIVATE
        "WPIEMU_E2E_DEFAULT_PROJECT=\"${CMAKE_SOURCE_DIR}/samples/swerve_robot/java\""
        "WPIEMU_E2E_VISION_DEFAULT_PROJECT=\"${CMAKE_SOURCE_DIR}/samples/vision_robot/java\"")
    if(TARGET halsim_wpiemu)
        add_dependencies(wpiemu_e2e_tests halsim_wpiemu)
        target_compile_definitions(wpiemu_e2e_tests PRIVATE "WPIEMU_E2E_EXTENSION=\"$<TARGET_FILE:halsim_wpiemu>\"")
    endif()
    wpiemu_doctest_target(wpiemu_e2e_tests)
    add_test(NAME e2e.swerve_sample COMMAND wpiemu_e2e_tests "--test-case=e2e: swerve sample*")
    set_tests_properties(e2e.swerve_sample PROPERTIES LABELS e2e SKIP_RETURN_CODE 77 TIMEOUT 1500)
    add_test(NAME e2e.vision_sample COMMAND wpiemu_e2e_tests "--test-case=e2e: vision sample*")
    set_tests_properties(e2e.vision_sample PROPERTIES LABELS e2e SKIP_RETURN_CODE 77 TIMEOUT 1500)
    # Season probes: every season pack that declares `probes` (a samples folder) gets e2e.season_<year>, e2e.vision_<year>
    # and e2e.vendor_<year>. The tests read their season from the environment (tests/season_probe.h).
    set(_wpiemu_probe_seasons "")
    wpiemu_season_years(_wpiemu_all_seasons)
    foreach(_year IN LISTS _wpiemu_all_seasons)
        file(READ "${CMAKE_SOURCE_DIR}/seasons/${_year}/season.json" _manifest)
        string(JSON _probes_type ERROR_VARIABLE _probes_error TYPE "${_manifest}" probes)
        if(NOT _probes_error AND _probes_type STREQUAL "STRING")
            list(APPEND _wpiemu_probe_seasons "${_year}")
        endif()
    endforeach()
    if(_wpiemu_probe_seasons)
        add_executable(wpiemu_season_e2e_tests tests/e2e_season_test.cpp tests/e2e_vision_season_test.cpp)
        add_dependencies(wpiemu_season_e2e_tests halsim_wpiemu)
        target_link_libraries(wpiemu_season_e2e_tests PRIVATE wpiemu_app_host wpiemu_plugins_builtin wpiemu_seasons_builtin)
        wpiemu_doctest_target(wpiemu_season_e2e_tests)
        add_executable(wpiemu_season_vendor_e2e_tests tests/e2e_vendor_season_test.cpp)
        add_dependencies(wpiemu_season_vendor_e2e_tests halsim_wpiemu)
        target_link_libraries(wpiemu_season_vendor_e2e_tests PRIVATE wpiemu_app_host wpiemu_plugins_builtin wpiemu_seasons_builtin)
        wpiemu_doctest_target(wpiemu_season_vendor_e2e_tests)
        foreach(_year IN LISTS _wpiemu_probe_seasons)
            wpiemu_season_value(_probes "${_year}" probes)
            wpiemu_season_value(_version "${_year}" wpilib version)
            # The build tree's own extension only serves the primary season; other seasons come from the composed install.
            if(_year STREQUAL WPIEMU_WPILIB_SEASON)
                set(_extension "$<TARGET_FILE:halsim_wpiemu>")
            else()
                set(_extension "")
            endif()
            set(_env "WPIEMU_SEASON_YEAR=${_year}" "WPIEMU_SEASON_VERSION=${_version}"
                "WPIEMU_SEASON_SAMPLES=${CMAKE_SOURCE_DIR}/${_probes}" "WPIEMU_SEASON_EXTENSION=${_extension}")
            add_test(NAME e2e.season_${_year} COMMAND wpiemu_season_e2e_tests "--test-case=e2e: season probe*")
            add_test(NAME e2e.vision_${_year} COMMAND wpiemu_season_e2e_tests "--test-case=e2e: season vision*")
            add_test(NAME e2e.vendor_${_year} COMMAND wpiemu_season_vendor_e2e_tests)
            foreach(_probe IN ITEMS season vision vendor)
                if(_probe STREQUAL "vendor")
                    set(_binary "$<TARGET_FILE_DIR:wpiemu_season_vendor_e2e_tests>")
                else()
                    set(_binary "$<TARGET_FILE_DIR:wpiemu_season_e2e_tests>")
                endif()
                set_tests_properties(e2e.${_probe}_${_year} PROPERTIES LABELS e2e SKIP_RETURN_CODE 77 TIMEOUT 1500
                    ENVIRONMENT "${_env};WPIEMU_SEASON_BINARY_DIR=${_binary}")
            endforeach()
        endforeach()
    endif()
endif()