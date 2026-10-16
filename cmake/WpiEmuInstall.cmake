# Install + CPack rules for distributable builds (docs/packaging.md). Included from the root CMakeLists.txt after
# every target exists. Nothing here changes how the build tree is built or tested, except for the application icon /
# version resources (Windows .rc, macOS bundle metadata) that are attached to wpiemu_app.
#
# Install layout (relative to the install prefix; the app finds the extension relative to its own location):
#   Windows   bin/wpiemu_app.exe (+ Qt DLLs)   extensions/halsim_wpiemu.dll   plugins/ qml/ (Qt, via windeployqt)
#   macOS     wpiemu_app.app/Contents/MacOS/wpiemu_app   .../Contents/PlugIns/wpiemu/libhalsim_wpiemu.dylib
#   Linux     bin/wpiemu_app   lib/wpiemu/libhalsim_wpiemu.so   share/{applications,icons,metainfo}
#             (Qt libraries are bundled by linuxdeploy for the AppImage, see packaging/linux/build-appimage.sh)
include_guard(GLOBAL)

option(WPIEMU_DEPLOY_QT "Bundle the Qt runtime (windeployqt/macdeployqt) when installing the app" ON)
option(WPIEMU_PACKAGE_WPILIB_RUNTIME
    "Also install the WPILib wpiHal/wpiutil libraries next to the extension (normally not needed: the robot process already loaded them)" OFF)
set(WPIEMU_MACOS_BUNDLE_ID "org.wpiemulator.app" CACHE STRING "CFBundleIdentifier of the macOS app")

set(_packaging_dir "${CMAKE_CURRENT_LIST_DIR}/../packaging")
cmake_path(NORMAL_PATH _packaging_dir)

# ---- layout ---------------------------------------------------------------------------------------------------------
if(APPLE)
    set(WPIEMU_INSTALL_APP_DIR ".")
    set(WPIEMU_INSTALL_EXTENSION_DIR "wpiemu_app.app/Contents/PlugIns/wpiemu")
    set(WPIEMU_INSTALL_DOC_DIR ".")
elseif(WIN32)
    set(WPIEMU_INSTALL_APP_DIR "bin")
    set(WPIEMU_INSTALL_EXTENSION_DIR "extensions")
    set(WPIEMU_INSTALL_DOC_DIR ".")
else()
    set(WPIEMU_INSTALL_APP_DIR "bin")
    set(WPIEMU_INSTALL_EXTENSION_DIR "lib/wpiemu")
    set(WPIEMU_INSTALL_DOC_DIR "share/doc/wpi-emulator")
endif()

# ---- the halsim extension (always installed; a stub that fails loudly when built without WPILib) ------------------------
if(TARGET halsim_wpiemu)
    if(APPLE)
        set_target_properties(halsim_wpiemu PROPERTIES INSTALL_RPATH "@loader_path")
    elseif(UNIX)
        set_target_properties(halsim_wpiemu PROPERTIES INSTALL_RPATH "$ORIGIN")
    endif()
    install(TARGETS halsim_wpiemu
        LIBRARY DESTINATION "${WPIEMU_INSTALL_EXTENSION_DIR}" COMPONENT Runtime
        RUNTIME DESTINATION "${WPIEMU_INSTALL_EXTENSION_DIR}" COMPONENT Runtime)

    if(WPIEMU_PACKAGE_WPILIB_RUNTIME AND TARGET wpiemu_wpilib_hal)
        install(FILES $<TARGET_FILE:wpiemu_wpilib_hal> $<TARGET_FILE:wpiemu_wpilib_wpiutil>
            DESTINATION "${WPIEMU_INSTALL_EXTENSION_DIR}" COMPONENT Runtime)
    endif()
endif()

# ---- the application -------------------------------------------------------------------------------------------------
if(TARGET wpiemu_app)
    install(TARGETS wpiemu_app
        RUNTIME DESTINATION "${WPIEMU_INSTALL_APP_DIR}" COMPONENT Runtime
        BUNDLE DESTINATION "${WPIEMU_INSTALL_APP_DIR}" COMPONENT Runtime)

    if(WIN32)
        enable_language(RC)
        set(WPIEMU_ICON_ICO "${_packaging_dir}/icons/wpiemu.ico")
        configure_file("${_packaging_dir}/windows/wpiemu.rc.in" "${CMAKE_BINARY_DIR}/packaging/wpiemu.rc" @ONLY)
        target_sources(wpiemu_app PRIVATE "${CMAKE_BINARY_DIR}/packaging/wpiemu.rc")
    elseif(APPLE)
        set(_icns "${_packaging_dir}/icons/wpiemu.icns")
        target_sources(wpiemu_app PRIVATE "${_icns}")
        set_source_files_properties("${_icns}" PROPERTIES MACOSX_PACKAGE_LOCATION Resources)
        set_target_properties(wpiemu_app PROPERTIES
            MACOSX_BUNDLE_INFO_PLIST "${_packaging_dir}/macos/Info.plist.in"
            MACOSX_BUNDLE_BUNDLE_NAME "WPI Emulator"
            MACOSX_BUNDLE_GUI_IDENTIFIER "${WPIEMU_MACOS_BUNDLE_ID}"
            MACOSX_BUNDLE_BUNDLE_VERSION "${PROJECT_VERSION}"
            MACOSX_BUNDLE_SHORT_VERSION_STRING "${PROJECT_VERSION}"
            MACOSX_BUNDLE_ICON_FILE "wpiemu.icns")
    endif()

    # Qt runtime deployment. Linux is left to linuxdeploy (AppImage); WPIEMU_DEPLOY_QT=ON also works there (generic tool).
    # Qt6::Svg is otherwise only visible inside app/ and the deploy-script generator (run here) cannot see it.
    if(WPIEMU_DEPLOY_QT AND (WIN32 OR APPLE))
        find_package(Qt6 REQUIRED COMPONENTS Svg)
        # The QML files are compiled in as plain resources (no qt_add_qml_module), so the importer scan has nothing to
        # look at: point the deploy tool at the sources instead, so QtQuick / QtQuick3D (+ plugins, shaders) are bundled.
        set(_qml_scan_root "${CMAKE_SOURCE_DIR}/app")
        if(WIN32)
            set(_deploy_tool_options --qmldir "${_qml_scan_root}")
        else()
            set(_deploy_tool_options "-qmldir=${_qml_scan_root}")
        endif()
        qt_generate_deploy_app_script(
            TARGET wpiemu_app
            OUTPUT_SCRIPT _wpiemu_qt_deploy_script
            DEPLOY_TOOL_OPTIONS ${_deploy_tool_options}
            NO_UNSUPPORTED_PLATFORM_ERROR)
        install(SCRIPT "${_wpiemu_qt_deploy_script}" COMPONENT Runtime)
    endif()
endif()

# ---- C++ runtime for the extension (loaded by the robot's JVM, which may not have loaded the VC++ runtime) -------------
if(WIN32)
    set(CMAKE_INSTALL_SYSTEM_RUNTIME_DESTINATION "${WPIEMU_INSTALL_EXTENSION_DIR}")
    set(CMAKE_INSTALL_SYSTEM_RUNTIME_COMPONENT Runtime)
    include(InstallRequiredSystemLibraries)
    # windeployqt only copies the compiler runtime when run from a VS developer prompt; install it app-local too.
    if(CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS)
        install(FILES ${CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS} DESTINATION "${WPIEMU_INSTALL_APP_DIR}" COMPONENT Runtime)
    endif()
endif()

# ---- notices and Linux desktop integration -----------------------------------------------------------------------------
install(FILES "${_packaging_dir}/THIRD-PARTY-NOTICES.txt"
    DESTINATION "${WPIEMU_INSTALL_DOC_DIR}" COMPONENT Runtime)

if(UNIX AND NOT APPLE)
    install(FILES "${_packaging_dir}/linux/wpiemu.desktop" DESTINATION share/applications COMPONENT Runtime)
    install(FILES "${_packaging_dir}/linux/wpiemu.metainfo.xml" DESTINATION share/metainfo COMPONENT Runtime)
    install(FILES "${_packaging_dir}/icons/wpiemu.svg" DESTINATION share/icons/hicolor/scalable/apps COMPONENT Runtime)
    foreach(_s 16 32 48 64 128 256 512)
        install(FILES "${_packaging_dir}/icons/wpiemu-${_s}.png"
            DESTINATION share/icons/hicolor/${_s}x${_s}/apps RENAME wpiemu.png COMPONENT Runtime)
    endforeach()
endif()

# ---- CPack -------------------------------------------------------------------------------------------------------------
set(CPACK_PACKAGE_NAME "WPI Emulator")
set(CPACK_PACKAGE_VENDOR "WPI Emulator contributors")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "${PROJECT_DESCRIPTION}")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "WPI Emulator")
set(CPACK_PACKAGE_CHECKSUM SHA256)

if(EXISTS "${CMAKE_SOURCE_DIR}/LICENSE")
    set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/LICENSE")
endif()

if(APPLE)
    set(_pkg_os "macos")
    set(_pkg_arch "${CMAKE_OSX_ARCHITECTURES}")
    if(NOT _pkg_arch)
        set(_pkg_arch "${CMAKE_HOST_SYSTEM_PROCESSOR}")
    endif()
elseif(WIN32)
    set(_pkg_os "windows")
    set(_pkg_arch "${CMAKE_SYSTEM_PROCESSOR}")
else()
    set(_pkg_os "linux")
    set(_pkg_arch "${CMAKE_SYSTEM_PROCESSOR}")
endif()
string(REPLACE ";" "-" _pkg_arch "${_pkg_arch}")
string(TOLOWER "${_pkg_arch}" _pkg_arch)
string(REPLACE "amd64" "x64" _pkg_arch "${_pkg_arch}")
string(REPLACE "x86_64" "x64" _pkg_arch "${_pkg_arch}")
set(CPACK_PACKAGE_FILE_NAME "wpi-emulator-${PROJECT_VERSION}-${_pkg_os}-${_pkg_arch}")

if(WIN32)
    find_program(WPIEMU_MAKENSIS makensis
        HINTS "C:/Program Files (x86)/NSIS" "C:/Program Files/NSIS" "$ENV{ProgramFiles}/NSIS")
    if(WPIEMU_MAKENSIS)
        set(CPACK_GENERATOR "NSIS;ZIP")
    else()
        set(CPACK_GENERATOR "ZIP")
        message(STATUS "wpiemu: NSIS (makensis) not found; cpack will only produce the portable ZIP")
    endif()
    # NSIS: per-machine 64-bit install, Start-menu + desktop shortcut, replaces an older install in place.
    set(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES64")
    set(CPACK_NSIS_PACKAGE_NAME "WPI Emulator")
    set(CPACK_NSIS_DISPLAY_NAME "WPI Emulator ${PROJECT_VERSION}")
    set(CPACK_NSIS_MUI_ICON "${_packaging_dir}/icons/wpiemu.ico")
    set(CPACK_NSIS_MUI_UNIICON "${_packaging_dir}/icons/wpiemu.ico")
    set(CPACK_NSIS_INSTALLED_ICON_NAME "bin\\\\wpiemu_app.exe")
    set(CPACK_NSIS_ENABLE_UNINSTALL_BEFORE_INSTALL ON)
    set(CPACK_NSIS_MODIFY_PATH OFF)
    set(CPACK_NSIS_MANIFEST_DPI_AWARE ON)
    set(CPACK_NSIS_IGNORE_LICENSE_PAGE ON)
    if(CPACK_RESOURCE_FILE_LICENSE)
        set(CPACK_NSIS_IGNORE_LICENSE_PAGE OFF)
    endif()
    set(CPACK_PACKAGE_EXECUTABLES "wpiemu_app" "WPI Emulator")
    set(CPACK_CREATE_DESKTOP_LINKS "wpiemu_app")
elseif(APPLE)
    set(CPACK_GENERATOR "DragNDrop")
    set(CPACK_DMG_FORMAT UDZO)
    set(CPACK_DMG_VOLUME_NAME "WPI Emulator ${PROJECT_VERSION}")
    set(CPACK_PACKAGE_ICON "${_packaging_dir}/icons/wpiemu.icns")
    set(CPACK_PRE_BUILD_SCRIPTS "${_packaging_dir}/macos/sign_bundle.cmake")
else()
    set(CPACK_GENERATOR "TGZ")
    set(CPACK_STRIP_FILES ON)
endif()

include(CPack)
