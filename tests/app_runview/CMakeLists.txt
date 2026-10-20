# Run view tests: doctest view-model tests plus GUI smoke tests under QT_QPA_PLATFORM=offscreen.
# Added from app/runview/CMakeLists.txt; deliberately excluded from the Qt-free wpiemu_tests glob.
file(GLOB WPIEMU_RUNVIEW_TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
find_package(Qt6 REQUIRED COMPONENTS Test)
add_executable(wpiemu_runview_tests ${WPIEMU_RUNVIEW_TEST_SOURCES})
target_link_libraries(wpiemu_runview_tests PRIVATE doctest::doctest wpiemu_app_runview Qt6::Test)
target_compile_definitions(wpiemu_runview_tests PRIVATE DOCTEST_CONFIG_USE_STD_HEADERS)
wpiemu_target_defaults(wpiemu_runview_tests)

enable_testing()
add_test(NAME runview.ui_tests COMMAND wpiemu_runview_tests)
set(_wpiemu_runview_test_env "QT_QPA_PLATFORM=set:offscreen")
if(WIN32)
    get_target_property(_qt_core_loc Qt6::Core IMPORTED_LOCATION_RELEASE)
    if(NOT _qt_core_loc)
        get_target_property(_qt_core_loc Qt6::Core LOCATION)
    endif()
    get_filename_component(_qt_bin_dir "${_qt_core_loc}" DIRECTORY)
    list(APPEND _wpiemu_runview_test_env "PATH=path_list_prepend:${_qt_bin_dir}")
endif()
set_tests_properties(runview.ui_tests PROPERTIES ENVIRONMENT_MODIFICATION "${_wpiemu_runview_test_env}")

# The demo is not a test, but it must leave cleanly: it once deleted a stack-allocated page at exit (heap corruption).
if(WPIEMU_BUILD_UI_DEMOS)
    add_test(NAME runview.demo_exit
             COMMAND wpiemu_runview_demo --scenario idle --screenshot "${CMAKE_CURRENT_BINARY_DIR}/demo_exit.png"
                     --settle 300 --size 640x400)
    set_tests_properties(runview.demo_exit PROPERTIES TIMEOUT 60 ENVIRONMENT_MODIFICATION "${_wpiemu_runview_test_env}")
endif()
