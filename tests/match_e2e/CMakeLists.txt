# Opt-in practice-match end-to-end test (label e2e): the swerve sample hosting a match through a relay on loopback with scripted
# encrypted players. Its own executable and one robot launch; it sees the host (wpiemu_app_host), the relay and the net module,
# which only tests may combine (tools/check_boundaries.py). Environment knobs are listed at the top of the source.
add_executable(wpiemu_e2e_match_tests e2e_practice_match_test.cpp)
target_link_libraries(wpiemu_e2e_match_tests PRIVATE
    wpiemu_app_host wpiemu_relay_core wpiemu_net wpiemu_plugins_builtin wpiemu_seasons_builtin)
target_compile_definitions(wpiemu_e2e_match_tests PRIVATE
    "WPIEMU_E2E_DEFAULT_PROJECT=\"${CMAKE_SOURCE_DIR}/samples/swerve_robot/java\"")
if(TARGET halsim_wpiemu)
    add_dependencies(wpiemu_e2e_match_tests halsim_wpiemu)
    target_compile_definitions(wpiemu_e2e_match_tests PRIVATE "WPIEMU_E2E_EXTENSION=\"$<TARGET_FILE:halsim_wpiemu>\"")
endif()
wpiemu_doctest_target(wpiemu_e2e_match_tests)
add_test(NAME e2e.practice_match COMMAND wpiemu_e2e_match_tests)
set_tests_properties(e2e.practice_match PROPERTIES LABELS e2e SKIP_RETURN_CODE 77 TIMEOUT 1800)
