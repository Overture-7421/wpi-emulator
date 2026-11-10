# Thin practice-match client (wpiemu_client): joins a match hosted by the desktop app over ENet or the relay, predicts its own
# robot and renders the shared scene. Links the SDK, core (physics/match), net, the plugin aggregation seam, the shared
# Qt scene support (app/ui) and the run view (app/runview); never ipc or a season plugin directly (tools/check_boundaries.py).
# Library sources are globbed, so adding a file here needs no CMake edit. Includes are rooted at app/ ("client/x.h").
set(CMAKE_AUTOUIC OFF)

file(GLOB WPIEMU_APP_CLIENT_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
list(FILTER WPIEMU_APP_CLIENT_SOURCES EXCLUDE REGEX "/main\\.cpp$")

qt_add_library(wpiemu_app_client STATIC ${WPIEMU_APP_CLIENT_SOURCES})
target_include_directories(wpiemu_app_client PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/..")
target_link_libraries(wpiemu_app_client
    PUBLIC wpiemu_sdk wpiemu_core wpiemu_match wpiemu_net wpiemu_app_ui wpiemu_app_runview
           Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Quick Qt6::Quick3D Qt6::QuickWidgets
    PRIVATE wpiemu_plugins_builtin)
wpiemu_target_defaults(wpiemu_app_client)
set_target_properties(wpiemu_app_client PROPERTIES AUTOMOC ON)
if(WIN32)
    target_link_libraries(wpiemu_app_client PRIVATE winmm)   # timeBeginPeriod: the client's worker wakes every ~2 ms
endif()

qt_add_executable(wpiemu_client main.cpp)
target_link_libraries(wpiemu_client PRIVATE
    wpiemu_app_client
    wpiemu_core wpiemu_net wpiemu_sdk wpiemu_plugins_builtin
    Qt6::Core Qt6::Gui Qt6::Widgets Qt6::Quick Qt6::Quick3D Qt6::QuickWidgets)
target_compile_definitions(wpiemu_client PRIVATE WPIEMU_APP_VERSION="${PROJECT_VERSION}")
wpiemu_target_defaults(wpiemu_client)
set_target_properties(wpiemu_client PROPERTIES WIN32_EXECUTABLE OFF)
