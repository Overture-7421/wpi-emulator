# Qt application shell (this stream: common/, wizard/, shell/, main.cpp). Links core, ipc, sdk; must NOT
# include or link year plugins directly (only the wpiemu_plugins_builtin aggregation seam, inside common/).
find_package(Qt6 REQUIRED COMPONENTS Svg)

# Page libraries owned by the editor / run view streams; optional so the shell builds without them.
foreach(_page editor runview)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${_page}/CMakeLists.txt")
        add_subdirectory(${_page})
    endif()
endforeach()

# This stream builds every widget in code (no .ui files); AUTOUIC would otherwise misread our
# ui_*.h headers as generated uic output. Set after the page libraries so their settings are untouched.
set(CMAKE_AUTOUIC OFF)
add_subdirectory(host)
add_subdirectory(common)
add_subdirectory(wizard)
add_subdirectory(shell)

qt_add_executable(wpiemu_app main.cpp)
target_link_libraries(wpiemu_app PRIVATE
    wpiemu_app_shell wpiemu_app_wizard wpiemu_app_common
    wpiemu_core wpiemu_ipc wpiemu_sdk wpiemu_plugins_builtin
    Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Quick Qt6::Quick3D Qt6::QuickWidgets)
target_compile_definitions(wpiemu_app PRIVATE WPIEMU_APP_VERSION="${PROJECT_VERSION}")
wpiemu_target_defaults(wpiemu_app)
set_target_properties(wpiemu_app PROPERTIES
    WIN32_EXECUTABLE OFF
    MACOSX_BUNDLE ON)

if(WPIEMU_BUILD_TESTS)
    enable_testing()   # the root enables it after this directory; tests registered here need it earlier
    add_subdirectory(tests)
endif()
