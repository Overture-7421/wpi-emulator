# Sourced by the AppImage's AppRun (linuxdeploy) before the application starts. Keep it POSIX-sh compatible.
export WPIEMU_PACKAGED=appimage
# Root directory with the season-specific libhalsim_wpiemu.so (install-relative lookup works too).
export WPIEMU_EXTENSION_DIR="${APPDIR}/usr/lib/wpiemu"
# The bundled Qt environment (QT_PLUGIN_PATH, LD_LIBRARY_PATH entries under $APPDIR, ...) must not leak into the robot
# processes the emulator launches (gradle, java, native robots); see docs/releasing.md.
