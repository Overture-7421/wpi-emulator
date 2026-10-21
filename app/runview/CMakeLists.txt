# Run-time view (UI stream "ui-run"): RunPage + FakeRunBackend, built as a static library.
# Links only the SDK, the host contract and the shared scene support (both SDK + Qt) and Qt; the shell hands RunPage the shared IRuntimeHost.
# Layout: scene/camera/HUD/console/DS logic in plain classes + view models, widgets on top, qml/ for the
# Qt Quick 3D viewport (compiled into the library as resources).

option(WPIEMU_RUNVIEW_SDL2  "Gamepad support for the run view through SDL2 (system package, or fetched below)" ON)
option(WPIEMU_RUNVIEW_FETCH_SDL2 "Download and build SDL2 when no system SDL2 is found (needs network at configure time)" ON)

# No .ui files here; AUTOUIC would mistake the SDK header ui_descriptors.h for a generated "ui_*.h".
set(CMAKE_AUTOUIC OFF)

set(WPIEMU_RUNVIEW_SOURCES
    fake_run_backend.cpp
    run_control.cpp
    console_model.cpp
    hud_model.cpp
    scene_parts.cpp
    scene_model.cpp
    state_model.cpp
    state_window.cpp
    sim_stats.cpp
    joystick_types.cpp
    joystick_source.cpp
    fake_joystick.cpp
    sdl_gamepad_provider.cpp
    joystick_manager.cpp
    driver_station_model.cpp
    status_chip.cpp
    console_panel.cpp
    joystick_editor.cpp
    driver_station_panel.cpp
    run_viewport.cpp
    run_page.cpp)

qt_add_library(wpiemu_app_runview STATIC ${WPIEMU_RUNVIEW_SOURCES})
target_include_directories(wpiemu_app_runview PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/..")
target_link_libraries(wpiemu_app_runview
    PUBLIC wpiemu_sdk wpiemu_app_contract wpiemu_app_ui Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Quick Qt6::QuickWidgets Qt6::Quick3D)
wpiemu_target_defaults(wpiemu_app_runview)
set_target_properties(wpiemu_app_runview PROPERTIES AUTOMOC ON)

qt_add_resources(wpiemu_app_runview runview_qml
    PREFIX "/wpiemu/runview"
    FILES
        qml/RunScene.qml
        qml/PartModel.qml
        qml/TagMarker.qml
        qml/HudStack.qml
        qml/Scoreboard.qml
        qml/HudWidget.qml)

# Official AprilTag 36h11 images (ids 1-32, 10x10 px) textured onto the field tags.
file(GLOB WPIEMU_RUNVIEW_TAG_IMAGES CONFIGURE_DEPENDS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/assets/apriltags/36h11/*.png")
qt_add_resources(wpiemu_app_runview runview_apriltags
    PREFIX "/wpiemu/runview"
    FILES ${WPIEMU_RUNVIEW_TAG_IMAGES})

# ---- optional gamepad backend (SDL2) --------------------------------------------------------
if(WPIEMU_RUNVIEW_SDL2)
    find_package(SDL2 CONFIG QUIET)
    set(_runview_sdl_target "")
    if(TARGET SDL2::SDL2)
        set(_runview_sdl_target SDL2::SDL2)
    elseif(TARGET SDL2::SDL2-static)
        set(_runview_sdl_target SDL2::SDL2-static)
    elseif(WPIEMU_RUNVIEW_FETCH_SDL2)
        if(APPLE)
            enable_language(OBJC)
        endif()
        include(FetchContent)
        set(SDL_SHARED OFF CACHE BOOL "" FORCE)
        set(SDL_STATIC ON CACHE BOOL "" FORCE)
        set(SDL_TEST OFF CACHE BOOL "" FORCE)
        set(SDL2_DISABLE_INSTALL ON CACHE BOOL "" FORCE)
        set(SDL2_DISABLE_UNINSTALL ON CACHE BOOL "" FORCE)
        FetchContent_Declare(sdl2
            URL https://github.com/libsdl-org/SDL/releases/download/release-2.30.9/SDL2-2.30.9.zip
            DOWNLOAD_EXTRACT_TIMESTAMP TRUE
            EXCLUDE_FROM_ALL)
        FetchContent_MakeAvailable(sdl2)
        if(TARGET SDL2-static)
            set(_runview_sdl_target SDL2-static)
        endif()
    endif()
    if(_runview_sdl_target)
        target_link_libraries(wpiemu_app_runview PRIVATE ${_runview_sdl_target})
        target_compile_definitions(wpiemu_app_runview PRIVATE WPIEMU_HAS_SDL2=1)
        message(STATUS "Run view: gamepad support via SDL2 (${_runview_sdl_target})")
    else()
        message(STATUS "Run view: SDL2 not found; keyboard joystick only (set WPIEMU_RUNVIEW_FETCH_SDL2=ON to download it)")
    endif()
endif()

# ---- tests (sources live in tests/app_runview; excluded from the Qt-free wpiemu_tests glob) ----
if(WPIEMU_BUILD_TESTS AND EXISTS "${PROJECT_SOURCE_DIR}/tests/app_runview/CMakeLists.txt")
    add_subdirectory("${PROJECT_SOURCE_DIR}/tests/app_runview" "${CMAKE_CURRENT_BINARY_DIR}/tests")
endif()

# ---- standalone demo ------------------------------------------------------------------------
if(WPIEMU_BUILD_UI_DEMOS)
    qt_add_executable(wpiemu_runview_demo demo/main.cpp)
    target_link_libraries(wpiemu_runview_demo PRIVATE wpiemu_app_runview)
    wpiemu_target_defaults(wpiemu_runview_demo)
endif()
