# Qt application shell (this stream: common/, wizard/, shell/, main.cpp). Links core, ipc, sdk; must NOT
# include or link year plugins directly (only the wpiemu_plugins_builtin aggregation seam, inside common/).
find_package(Qt6 REQUIRED COMPONENTS Svg)

option(WPIEMU_BUILD_UI_DEMOS "Build the standalone UI demo executables (wpiemu_editor_demo, wpiemu_runview_demo)" ON)

# The Qt scene support and host seam (SDK + Qt only) both page libraries share, declared before them.
add_subdirectory(ui)

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

qt_add_executable(wpiemu_app main.cpp)
target_link_libraries(wpiemu_app PRIVATE
    wpiemu_app_common
    wpiemu_core wpiemu_ipc wpiemu_sdk wpiemu_plugins_builtin
    Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Quick Qt6::Quick3D Qt6::QuickWidgets)
target_compile_definitions(wpiemu_app PRIVATE WPIEMU_APP_VERSION="${PROJECT_VERSION}")
wpiemu_target_defaults(wpiemu_app)
set_target_properties(wpiemu_app PROPERTIES
    WIN32_EXECUTABLE OFF
    MACOSX_BUNDLE ON)
wpiemu_qt_deploy_runtime(wpiemu_app)

# Dev convenience (Visual Studio): the Qt runtime is not deployed next to the exe (the shared bin/ directory also holds the
# test executables, which must keep resolving Qt from PATH), so generate the git-ignored .vs/launch.vs.json that puts the
# Qt bin directory on PATH when debugging wpiemu_app.
if(WIN32 AND PROJECT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    get_target_property(_qmake_exe Qt6::qmake IMPORTED_LOCATION)
    get_filename_component(_qt_bin_dir "${_qmake_exe}" DIRECTORY)
    file(TO_NATIVE_PATH "${_qt_bin_dir}" _qt_bin_native)
    string(REPLACE "\\" "\\\\" _qt_bin_json "${_qt_bin_native}")
    set(_launch_json
"{
  \"version\": \"0.2.1\",
  \"defaults\": {},
  \"configurations\": [
    {
      \"type\": \"default\",
      \"project\": \"CMakeLists.txt\",
      \"projectTarget\": \"wpiemu_app.exe\",
      \"name\": \"wpiemu_app.exe\",
      \"env\": { \"PATH\": \"${_qt_bin_json};\${env.PATH}\" }
    },
    {
      \"type\": \"default\",
      \"project\": \"CMakeLists.txt\",
      \"projectTarget\": \"wpiemu_app.exe\",
      \"name\": \"wpiemu_app.exe --demo\",
      \"args\": [ \"--demo\" ],
      \"env\": { \"PATH\": \"${_qt_bin_json};\${env.PATH}\" }
    }
  ]
}
")
    file(WRITE "${PROJECT_SOURCE_DIR}/.vs/launch.vs.json" "${_launch_json}")
endif()

if(WPIEMU_BUILD_TESTS)
    add_subdirectory(tests)
endif()
