# Qt scene support the editor and the run view share (SDK + Qt only): conversions from SDK types to Qt Quick 3D, and the
# QML component that owns the single Z-up -> Y-up conversion. Both page libraries link it; it links neither core nor
# the host contract.
set(CMAKE_AUTOUIC OFF)

qt_add_library(wpiemu_app_ui STATIC scene_support.cpp)
# Includes are rooted at app/ ("ui/scene_support.h").
target_include_directories(wpiemu_app_ui PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/..")
target_link_libraries(wpiemu_app_ui PUBLIC wpiemu_sdk Qt6::Core Qt6::Gui)
wpiemu_target_defaults(wpiemu_app_ui)
set_target_properties(wpiemu_app_ui PROPERTIES AUTOMOC ON)

qt_add_resources(wpiemu_app_ui wpiemu_ui_qml
    PREFIX "/wpiemu/ui"
    FILES qml/ZUpWorld.qml)
