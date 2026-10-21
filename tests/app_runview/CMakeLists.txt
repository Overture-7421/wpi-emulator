# Run view tests: doctest view-model tests plus GUI smoke tests under QT_QPA_PLATFORM=offscreen.
# Added from app/runview/CMakeLists.txt; deliberately excluded from the Qt-free wpiemu_tests glob.
file(GLOB WPIEMU_RUNVIEW_TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
find_package(Qt6 REQUIRED COMPONENTS Test)
add_executable(wpiemu_runview_tests ${WPIEMU_RUNVIEW_TEST_SOURCES})
target_link_libraries(wpiemu_runview_tests PRIVATE wpiemu_app_runview Qt6::Test)
wpiemu_doctest_target(wpiemu_runview_tests)

add_test(NAME runview.ui_tests COMMAND wpiemu_runview_tests)
wpiemu_qt_test_environment(runview.ui_tests HEADLESS)

# The demo is not a test, but it must leave cleanly: it once deleted a stack-allocated page at exit (heap corruption).
if(WPIEMU_BUILD_UI_DEMOS)
    add_test(NAME runview.demo_exit
             COMMAND wpiemu_runview_demo --scenario idle --screenshot "${CMAKE_CURRENT_BINARY_DIR}/demo_exit.png"
                     --settle 300 --size 640x400)
    set_tests_properties(runview.demo_exit PROPERTIES TIMEOUT 60)
    wpiemu_qt_test_environment(runview.demo_exit HEADLESS)
endif()
