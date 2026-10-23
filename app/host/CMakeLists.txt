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
    target_link_libraries(wpiemu_e2e_tests PRIVATE wpiemu_app_host wpiemu_plugins_builtin)
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
    add_executable(wpiemu_season_e2e_tests tests/e2e_season_test.cpp)
    add_dependencies(wpiemu_season_e2e_tests halsim_wpiemu)
    target_link_libraries(wpiemu_season_e2e_tests PRIVATE wpiemu_app_host wpiemu_plugins_builtin)
    if(WPIEMU_WPILIB_SEASON STREQUAL "2027")
        set(_season_extension "$<TARGET_FILE:halsim_wpiemu>")
    else()
        set(_season_extension "")
    endif()
    target_compile_definitions(wpiemu_season_e2e_tests PRIVATE
        "WPIEMU_SEASON_SAMPLES=\"${CMAKE_SOURCE_DIR}/samples/season_2027\""
        "WPIEMU_SEASON_BINARY_DIR=\"$<TARGET_FILE_DIR:wpiemu_season_e2e_tests>\""
        "WPIEMU_SEASON_EXTENSION=\"${_season_extension}\"")
    wpiemu_doctest_target(wpiemu_season_e2e_tests)
    add_test(NAME e2e.season_2027 COMMAND wpiemu_season_e2e_tests)
    set_tests_properties(e2e.season_2027 PROPERTIES LABELS e2e SKIP_RETURN_CODE 77 TIMEOUT 1500)
endif()
