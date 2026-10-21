# The Qt 6 kit every first-party Qt module builds against, declared once for the full build and the standalone editor
# build. Point CMAKE_PREFIX_PATH at the kit, e.g. C:/Qt/<version>/msvc2022_64.
#   wpiemu_qt_test_environment(<test>... [HEADLESS])
find_package(Qt6 6.5 REQUIRED COMPONENTS Core Gui Widgets Quick Quick3D QuickWidgets)
qt_standard_project_setup()

# Qt DLLs are not deployed into the build tree, so Windows tests find them through PATH. HEADLESS also runs the tests
# with the offscreen platform plugin. Call it in the directory that added the tests.
function(wpiemu_qt_test_environment)
    cmake_parse_arguments(PARSE_ARGV 0 ARG "HEADLESS" "" "")
    set(environment "")
    if(ARG_HEADLESS)
        list(APPEND environment "QT_QPA_PLATFORM=set:offscreen")
    endif()
    if(WIN32)
        get_target_property(qt_core_location Qt6::Core IMPORTED_LOCATION_RELEASE)
        if(NOT qt_core_location)
            get_target_property(qt_core_location Qt6::Core LOCATION)
        endif()
        get_filename_component(qt_bin_dir "${qt_core_location}" DIRECTORY)
        list(APPEND environment "PATH=path_list_prepend:${qt_bin_dir}")
    endif()
    if(environment)
        set_property(TEST ${ARG_UNPARSED_ARGUMENTS} APPEND PROPERTY ENVIRONMENT_MODIFICATION "${environment}")
    endif()
endfunction()
