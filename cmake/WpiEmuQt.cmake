# The Qt 6 kit every first-party Qt module builds against, declared once for the full build and the standalone editor
# build. Point CMAKE_PREFIX_PATH at the kit, e.g. C:/Qt/<version>/msvc2022_64, or leave it unset to auto-discover /
# auto-download the pinned Qt release via aqtinstall.
#   wpiemu_qt_test_environment(<test>... [HEADLESS])
#   wpiemu_qt_deploy_runtime(<target>)
option(WPIEMU_QT_DOWNLOAD "Automatically download Qt 6 via aqtinstall when not found on the system" ON)

set(_wpiemu_qt_components Core Gui Widgets Quick Quick3D QuickWidgets Svg Multimedia)

# Clear stale Qt6*_DIR cache entries if a previous Qt installation was moved or deleted.
if(Qt6_DIR AND NOT EXISTS "${Qt6_DIR}/Qt6Config.cmake")
    unset(Qt6_DIR CACHE)
    foreach(_comp IN LISTS _wpiemu_qt_components ITEMS Test Tools ShaderTools Quick3DRuntimeRender Quick3DUtils)
        unset(Qt6${_comp}_DIR CACHE)
    endforeach()
endif()

# 1. Search standard local Qt 6 install locations first.
if(NOT Qt6_DIR)
    set(_wpiemu_qt_search_globs
        "${CMAKE_SOURCE_DIR}/.qt/6.*/*"
        "$ENV{HOME}/Qt/6.*/*")
    if(WIN32)
        list(PREPEND _wpiemu_qt_search_globs "C:/Qt/6.*/msvc*_64")
    endif()
    file(GLOB _wpiemu_qt_kits LIST_DIRECTORIES true ${_wpiemu_qt_search_globs})
    if(_wpiemu_qt_kits)
        list(SORT _wpiemu_qt_kits ORDER DESCENDING)
        foreach(_kit IN LISTS _wpiemu_qt_kits)
            if(EXISTS "${_kit}/lib/cmake/Qt6/Qt6Config.cmake")
                list(APPEND CMAKE_PREFIX_PATH "${_kit}")
            endif()
        endforeach()
    endif()
endif()

find_package(Qt6 6.5 QUIET COMPONENTS ${_wpiemu_qt_components})

# 2. If Qt 6 is still not on the machine and WPIEMU_QT_DOWNLOAD is ON, install the pinned release via aqtinstall.
if(NOT Qt6_FOUND AND WPIEMU_QT_DOWNLOAD)
    wpiemu_pinned_version(_wpiemu_qt_ver qt release)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)

    if(WIN32)
        set(_aqt_host "windows")
        set(_aqt_arch "win64_msvc2022_64")
        set(_aqt_kit_subdir "msvc2022_64")
        set(_aqt_out_dir "C:/Qt")
    elseif(APPLE)
        set(_aqt_host "mac")
        set(_aqt_arch "clang_64")
        set(_aqt_kit_subdir "macos")
        set(_aqt_out_dir "${CMAKE_SOURCE_DIR}/.qt")
    else()
        set(_aqt_host "linux")
        set(_aqt_arch "linux_gcc_64")
        set(_aqt_kit_subdir "gcc_64")
        set(_aqt_out_dir "${CMAKE_SOURCE_DIR}/.qt")
    endif()

    set(_aqt_kit_path "${_aqt_out_dir}/${_wpiemu_qt_ver}/${_aqt_kit_subdir}")
    # The kit may exist from an earlier checkout without a module added since (aqt adds modules to an installed kit).
    set(_aqt_modules qtquick3d qtshadertools qtmultimedia)
    if(NOT EXISTS "${_aqt_kit_path}/lib/cmake/Qt6/Qt6Config.cmake"
       OR NOT EXISTS "${_aqt_kit_path}/lib/cmake/Qt6Multimedia/Qt6MultimediaConfig.cmake")
        message(STATUS "wpiemu: Qt ${_wpiemu_qt_ver} (${_aqt_modules}) not found; installing via aqtinstall into ${_aqt_kit_path}...")
        set(_aqt_python "${Python3_EXECUTABLE}")
        execute_process(
            COMMAND "${_aqt_python}" -m aqt version
            RESULT_VARIABLE _aqt_check_rc
            OUTPUT_QUIET ERROR_QUIET)
        if(NOT _aqt_check_rc EQUAL 0)
            # Create an isolated venv in the build tree so PEP 668 / uv / system-managed Pythons never reject pip install.
            set(_aqt_venv "${CMAKE_BINARY_DIR}/_aqt_venv")
            if(WIN32)
                set(_aqt_venv_python "${_aqt_venv}/Scripts/python.exe")
            else()
                set(_aqt_venv_python "${_aqt_venv}/bin/python")
            endif()
            if(NOT EXISTS "${_aqt_venv_python}")
                execute_process(
                    COMMAND "${Python3_EXECUTABLE}" -m venv "${_aqt_venv}"
                    RESULT_VARIABLE _venv_rc)
                if(NOT _venv_rc EQUAL 0)
                    message(FATAL_ERROR "Failed to create Python venv for aqtinstall at ${_aqt_venv}")
                endif()
            endif()
            set(_aqt_python "${_aqt_venv_python}")
            execute_process(
                COMMAND "${_aqt_python}" -m aqt version
                RESULT_VARIABLE _aqt_venv_check_rc
                OUTPUT_QUIET ERROR_QUIET)
            if(NOT _aqt_venv_check_rc EQUAL 0)
                execute_process(
                    COMMAND "${_aqt_python}" -m pip install --quiet aqtinstall
                    RESULT_VARIABLE _pip_rc)
                if(NOT _pip_rc EQUAL 0)
                    message(FATAL_ERROR "Failed to install aqtinstall in ${_aqt_venv}")
                endif()
            endif()
        endif()
        execute_process(
            COMMAND "${_aqt_python}" -m aqt install-qt "${_aqt_host}" desktop "${_wpiemu_qt_ver}" "${_aqt_arch}"
                    -O "${_aqt_out_dir}" -m ${_aqt_modules}
            RESULT_VARIABLE _aqt_rc)
        if(NOT _aqt_rc EQUAL 0 AND WIN32)
            # Fall back to repo-local .qt if C:/Qt is not writable.
            set(_aqt_out_dir "${CMAKE_SOURCE_DIR}/.qt")
            set(_aqt_kit_path "${_aqt_out_dir}/${_wpiemu_qt_ver}/${_aqt_kit_subdir}")
            execute_process(
                COMMAND "${_aqt_python}" -m aqt install-qt "${_aqt_host}" desktop "${_wpiemu_qt_ver}" "${_aqt_arch}"
                        -O "${_aqt_out_dir}" -m ${_aqt_modules}
                RESULT_VARIABLE _aqt_rc)
        endif()
        if(NOT _aqt_rc EQUAL 0)
            message(FATAL_ERROR "aqtinstall failed to download Qt ${_wpiemu_qt_ver} (${_aqt_arch})")
        endif()
    endif()
    unset(Qt6_DIR CACHE)
    foreach(_comp IN LISTS _wpiemu_qt_components ITEMS Test Tools ShaderTools Quick3DRuntimeRender Quick3DUtils)
        unset(Qt6${_comp}_DIR CACHE)
    endforeach()
    list(PREPEND CMAKE_PREFIX_PATH "${_aqt_kit_path}")
    set(Qt6_DIR "${_aqt_kit_path}/lib/cmake/Qt6" CACHE PATH "Qt6 CMake configuration directory" FORCE)
endif()

if(NOT Qt6_FOUND)
    message(STATUS "wpiemu: the Qt kit needs Quick 3D, Shader Tools and Multimedia (match sounds); with aqtinstall: -m qtquick3d qtshadertools qtmultimedia")
endif()
find_package(Qt6 6.5 REQUIRED COMPONENTS ${_wpiemu_qt_components})
if(WPIEMU_BUILD_TESTS)
    find_package(Qt6 6.5 QUIET COMPONENTS Test)
endif()
qt_standard_project_setup()

# Helper: returns the directory containing the Qt6 shared libraries on Windows.
function(_wpiemu_get_qt_bin_dir out_var)
    get_target_property(_qmake_exe Qt6::qmake IMPORTED_LOCATION)
    if(_qmake_exe)
        get_filename_component(_qt_bin "${_qmake_exe}" DIRECTORY)
    else()
        get_target_property(_qt_core Qt6::Core IMPORTED_LOCATION_RELEASE)
        if(NOT _qt_core)
            get_target_property(_qt_core Qt6::Core LOCATION)
        endif()
        get_filename_component(_qt_bin "${_qt_core}" DIRECTORY)
    endif()
    set(${out_var} "${_qt_bin}" PARENT_SCOPE)
endfunction()

# Deploys the Qt runtime (DLLs, platform plugins, QML imports) into `<target>`'s build-tree output directory on Windows
# and configures VS_DEBUGGER_ENVIRONMENT so Debug and Release builds run self-contained from Visual Studio, CLI or Explorer.
function(wpiemu_qt_deploy_runtime target_name)
    if(NOT WIN32)
        return()
    endif()
    _wpiemu_get_qt_bin_dir(_qt_bin_dir)
    file(TO_NATIVE_PATH "${_qt_bin_dir}" _qt_bin_native)
    set_property(TARGET ${target_name} APPEND PROPERTY
        VS_DEBUGGER_ENVIRONMENT "PATH=${_qt_bin_native};$ENV{PATH}")

    if(TARGET Qt6::windeployqt)
        set(_windeployqt_cmd "$<TARGET_FILE:Qt6::windeployqt>")
    else()
        set(_windeployqt_cmd "${_qt_bin_dir}/windeployqt.exe")
    endif()

    if(EXISTS "${_qt_bin_dir}/windeployqt.exe" OR TARGET Qt6::windeployqt)
        add_custom_command(TARGET ${target_name} POST_BUILD
            COMMAND "${_windeployqt_cmd}"
                    --no-translations
                    --no-compiler-runtime
                    --no-opengl-sw
                    --no-system-d3d-compiler
                    --no-ffmpeg
                    --qmldir "${CMAKE_SOURCE_DIR}/app"
                    "$<TARGET_FILE:${target_name}>"
            COMMENT "Deploying Qt runtime for ${target_name} ($<CONFIG>)")
        get_filename_component(_qt_root_dir "${_qt_bin_dir}" DIRECTORY)
        set(_qoffscreen_dll "${_qt_root_dir}/plugins/platforms/qoffscreen$<$<CONFIG:Debug>:d>.dll")
        add_custom_command(TARGET ${target_name} POST_BUILD
            COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                    "${_qoffscreen_dll}"
                    "$<TARGET_FILE_DIR:${target_name}>/platforms/"
            COMMENT "Copying qoffscreen platform plugin for headless test suites ($<CONFIG>)")
        if(TARGET Qt6::Test)
            add_custom_command(TARGET ${target_name} POST_BUILD
                COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                        "$<TARGET_FILE:Qt6::Test>"
                        "$<TARGET_FILE_DIR:${target_name}>"
                COMMENT "Copying Qt6Test runtime DLL for test suites ($<CONFIG>)")
        endif()
    endif()
endfunction()

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
