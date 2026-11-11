# Qt support every app layer shares (SDK + Qt only): the host seam (IRuntimeHost, its Qt DTOs and metatypes),
# conversions from SDK types to Qt Quick 3D and the QML component that owns the single Z-up -> Y-up conversion,
# the text and widget helpers the layers would otherwise copy, and the settings port (IKeyValueStore) with the practice-match
# relay presets built on it (the game client uses both and may not link app/common). The editor, the run view, common, the
# wizard, the shell and the client link it; it links only the SDK and Qt.
set(CMAKE_AUTOUIC OFF)

qt_add_library(wpiemu_app_ui STATIC
    ../contract/runtime_host.cpp
    key_value_store.cpp
    relay_endpoints.cpp
    scene_support.cpp
    qt_text.cpp
    widget_support.cpp)
# Includes are rooted at app/ ("ui/scene_support.h", "contract/runtime_host.h").
target_include_directories(wpiemu_app_ui PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/..")
target_link_libraries(wpiemu_app_ui PUBLIC wpiemu_sdk Qt6::Core Qt6::Gui Qt6::Widgets)
wpiemu_target_defaults(wpiemu_app_ui)
set_target_properties(wpiemu_app_ui PROPERTIES AUTOMOC ON)

# The DNS label suffix of the deployed relay (infra/relay dnsLabelSuffix, for example "47c926"), baked into release builds so
# the Production relay preset works without any setting. Empty: the user sets it (relay_endpoints.h).
set(WPIEMU_RELAY_DNS_SUFFIX "" CACHE STRING "DNS label suffix of the deployed practice-match relay, for example 47c926")
if(WPIEMU_RELAY_DNS_SUFFIX)
    set_property(SOURCE relay_endpoints.cpp APPEND PROPERTY COMPILE_DEFINITIONS "WPIEMU_RELAY_DNS_SUFFIX=\"${WPIEMU_RELAY_DNS_SUFFIX}\"")
endif()

qt_add_resources(wpiemu_app_ui wpiemu_ui_qml
    PREFIX "/wpiemu/ui"
    FILES qml/ZUpWorld.qml)
