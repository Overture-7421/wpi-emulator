# Season packs (docs/reference/seasons.md). Each seasons/<year>/game is an optional game plugin that links only
# wpiemu_sdk; the manifest names it, and builtin_plugins.cpp registers it. Platform-only seasons have no game/ directory.
wpiemu_season_years(_wpiemu_season_years)
set(_wpiemu_plugin_targets "")
set(_wpiemu_plugin_sounds "")
foreach(_year IN LISTS _wpiemu_season_years)
    wpiemu_season_game_plugin(_plugin "${_year}")
    if(NOT _plugin)
        continue()
    endif()
    add_subdirectory("${_year}/game")
    list(APPEND _wpiemu_plugin_targets "wpiemu_plugin_${_plugin}")
    # Sound files a plugin ships (<year>/game/assets/sfx/<asset>.wav), handed to the app through one global property of
    # "<file>|<plugin>/sfx/<asset>.wav" entries so it can embed them without naming a plugin.
    file(GLOB _sounds CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/${_year}/game/assets/sfx/*.wav")
    foreach(_file IN LISTS _sounds)
        get_filename_component(_asset "${_file}" NAME)
        list(APPEND _wpiemu_plugin_sounds "${_file}|${_plugin}/sfx/${_asset}")
    endforeach()
endforeach()
set_property(GLOBAL PROPERTY WPIEMU_BUILTIN_PLUGIN_SOUNDS "${_wpiemu_plugin_sounds}")
set_property(GLOBAL PROPERTY WPIEMU_BUILTIN_PLUGIN_TARGETS "${_wpiemu_plugin_targets}")

if(WPIEMU_BUILD_PLUGIN_TEMPLATE)
    add_subdirectory(_template/game)
endif()

# Season profiles: the manifests become the C++ catalog behind wpiemu::sdk::registerBuiltinSeasons().
function(wpiemu_cpp_string_list out)
    set(_text "")
    foreach(_item IN LISTS ARGN)
        string(APPEND _text "\"${_item}\", ")
    endforeach()
    set(${out} "${_text}" PARENT_SCOPE)
endfunction()

set(_wpiemu_season_calls "")
set(_wpiemu_season_functions "")
set(_wpiemu_season_index 0)
foreach(_year IN LISTS _wpiemu_season_years)
    wpiemu_season_value(_channel "${_year}" channel)
    wpiemu_season_value(_default "${_year}" default)
    wpiemu_season_value(_version "${_year}" wpilib version)
    wpiemu_season_array(_verified "${_year}" wpilib verified)
    wpiemu_cpp_string_list(_verified_cpp ${_verified})
    wpiemu_season_value(_install "${_year}" wpilib installFolder)
    wpiemu_season_value(_abi "${_year}" platform abi)
    wpiemu_season_value(_launch "${_year}" platform launch)
    wpiemu_season_value(_java "${_year}" toolchain javaMajor)
    wpiemu_season_game_plugin(_game "${_year}")
    if(_channel STREQUAL "preview")
        set(_channel_cpp "SeasonChannel::Preview")
    else()
        set(_channel_cpp "SeasonChannel::Stable")
    endif()
    if(_default)
        set(_default_cpp "true")
    else()
        set(_default_cpp "false")
    endif()
    set(_vendor_cpp "")
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/${_year}/season.json" _manifest)
    string(JSON _vendor_count LENGTH "${_manifest}" vendors)
    if(_vendor_count GREATER 0)
        math(EXPR _vendor_last "${_vendor_count} - 1")
        foreach(_v RANGE ${_vendor_last})
            string(JSON _vendor MEMBER "${_manifest}" vendors ${_v})
            wpiemu_season_value(_schema "${_year}" vendors ${_vendor} schema)
            wpiemu_season_value(_default_version "${_year}" vendors ${_vendor} defaultVersion)
            wpiemu_season_array(_vendor_verified "${_year}" vendors ${_vendor} verified)
            wpiemu_cpp_string_list(_vendor_verified_cpp ${_vendor_verified})
            string(APPEND _vendor_cpp "    p.vendors[\"${_vendor}\"] = VendorProfile{\"${_schema}\", \"${_default_version}\", {${_vendor_verified_cpp}}};\n")
        endforeach()
    endif()
    string(APPEND _wpiemu_season_functions "
SeasonProfile season${_wpiemu_season_index}() {
    SeasonProfile p;
    p.year = ${_year};
    p.channel = ${_channel_cpp};
    p.isDefault = ${_default_cpp};
    p.wpilibVersion = \"${_version}\";
    p.verifiedWpilibVersions = {${_verified_cpp}};
    p.installFolder = \"${_install}\";
    p.halAbi = \"${_abi}\";
    p.launchProfile = \"${_launch}\";
    p.javaMajor = ${_java};
    p.gamePluginId = \"${_game}\";
${_vendor_cpp}    return p;
}
")
    string(APPEND _wpiemu_season_calls "    keep(registry.add(season${_wpiemu_season_index}()));\n")
    math(EXPR _wpiemu_season_index "${_wpiemu_season_index} + 1")
endforeach()
file(CONFIGURE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/builtin_seasons.cpp" @ONLY CONTENT
"// Generated from seasons/*/season.json by seasons/CMakeLists.txt. Do not edit.
#include <utility>

#include \"wpiemu/sdk/season/builtin_seasons.h\"
#include \"wpiemu/sdk/season/season_registry.h\"

namespace wpiemu::sdk {
namespace {
@_wpiemu_season_functions@}  // namespace

Status registerBuiltinSeasons(SeasonRegistry& registry) {
    Status first = okStatus();
    auto keep = [&first](Status s) {
        if (first.ok() && !s.ok()) first = std::move(s);
    };
@_wpiemu_season_calls@    return first;
}

}  // namespace wpiemu::sdk
")
add_library(wpiemu_seasons_builtin STATIC "${CMAKE_CURRENT_BINARY_DIR}/builtin_seasons.cpp")
add_library(wpiemu::seasons_builtin ALIAS wpiemu_seasons_builtin)
target_link_libraries(wpiemu_seasons_builtin PUBLIC wpiemu_sdk)
wpiemu_target_defaults(wpiemu_seasons_builtin)

# Aggregation seam (docs/ARCHITECTURE.md): the single target that links every built-in plugin and
# defines wpiemu::sdk::registerBuiltinGamePlugins(). The app links only this target, never a
# wpiemu_plugin_* target. The template plugin is deliberately NOT included.
add_library(wpiemu_plugins_builtin STATIC builtin_plugins.cpp)
add_library(wpiemu::plugins_builtin ALIAS wpiemu_plugins_builtin)
target_link_libraries(wpiemu_plugins_builtin PUBLIC wpiemu_sdk PRIVATE ${_wpiemu_plugin_targets})
wpiemu_target_defaults(wpiemu_plugins_builtin)
