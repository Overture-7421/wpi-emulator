# The seam between the UI and whatever runs the simulation (IRuntimeHost, its Qt DTOs and metatypes).
# Links only the SDK and Qt, so the run view can consume it without core; the real host (wpiemu_app_common)
# and the scripted stand-ins implement it.
set(CMAKE_AUTOUIC OFF)

qt_add_library(wpiemu_app_contract STATIC runtime_host.cpp)
# Includes are rooted at app/ ("contract/runtime_host.h").
target_include_directories(wpiemu_app_contract PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/..")
target_link_libraries(wpiemu_app_contract PUBLIC wpiemu_sdk Qt6::Core)
wpiemu_target_defaults(wpiemu_app_contract)
set_target_properties(wpiemu_app_contract PROPERTIES AUTOMOC ON)
