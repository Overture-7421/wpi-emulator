# Qt support every app layer shares (SDK + Qt only): conversions from SDK types to Qt Quick 3D and the QML component that
# owns the single Z-up -> Y-up conversion, plus the text and widget helpers the layers would otherwise copy. The editor,
# the run view, common, the wizard and the shell link it; it links neither core nor the host contract.
set(CMAKE_AUTOUIC OFF)

qt_add_library(wpiemu_app_ui STATIC scene_support.cpp qt_text.cpp widget_support.cpp)
# Includes are rooted at app/ ("ui/scene_support.h").
target_include_directories(wpiemu_app_ui PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/..")
target_link_libraries(wpiemu_app_ui PUBLIC wpiemu_sdk Qt6::Core Qt6::Gui Qt6::Widgets)
wpiemu_target_defaults(wpiemu_app_ui)
set_target_properties(wpiemu_app_ui PROPERTIES AUTOMOC ON)

qt_add_resources(wpiemu_app_ui wpiemu_ui_qml
    PREFIX "/wpiemu/ui"
    FILES qml/ZUpWorld.qml)
