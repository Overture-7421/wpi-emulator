#!/usr/bin/env bash
# Builds WPI Emulator from source and packages it as an AppImage with linuxdeploy + linuxdeploy-plugin-qt.
#
#   QT_ROOT_DIR=$HOME/Qt/6.8.3/gcc_64 packaging/linux/build-appimage.sh
#
# Environment (all optional except a Qt location):
#   QT_ROOT_DIR / CMAKE_PREFIX_PATH  Qt 6.8 kit (gcc_64); QMAKE may point at qmake directly
#   BUILD_DIR      build + staging root                          (default: <repo>/build/appimage)
#   OUT_DIR        where the final .AppImage is written          (default: $BUILD_DIR/dist)
#   WPIEMU_WPILIB_ROOT      WPILib maven cache / extracted root for the HAL extension (optional)
#   WPIEMU_WPILIB_DOWNLOAD  ON (default): fetch WPILib headers/libs from frcmaven; the extension becomes a stub if
#                           that fails (halsim_ext/cmake/WpiEmuWpilib.cmake). Set OFF to stay offline.
#   WPIEMU_FETCH_SDL2       ON (default): build SDL2 (static) for gamepad support; OFF: keyboard joystick only
#   SKIP_BUILD=1   reuse an existing build in $BUILD_DIR
#
# Needs: cmake >= 3.25, ninja, a C++20 compiler, curl, sha256sum, and the Qt runtime libraries' system
# dependencies (libxcb*, libxkbcommon*, libgl1, libegl1, ...). Build on the OLDEST distro you want to support
# (CI uses ubuntu-22.04): an AppImage runs wherever the glibc of the build machine or newer is present.
# The tools are downloaded here (pinned release + sha256) and run with --appimage-extract-and-run, so no FUSE is needed.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${BUILD_DIR:-$ROOT/build/appimage}"
OUT_DIR="${OUT_DIR:-$BUILD_DIR/dist}"
TOOLS_DIR="${TOOLS_DIR:-$BUILD_DIR/tools}"
APPDIR="$BUILD_DIR/AppDir"
WPILIB_DOWNLOAD="${WPIEMU_WPILIB_DOWNLOAD:-ON}"
FETCH_SDL2="${WPIEMU_FETCH_SDL2:-ON}"

LINUXDEPLOY_VERSION="1-alpha-20250213-2"
LINUXDEPLOY_SHA256="4648f278ab3ef31f819e67c30d50f462640e5365a77637d7e6f2ad9fd0b4522a"
PLUGIN_QT_VERSION="1-alpha-20250213-1"
PLUGIN_QT_SHA256="15106be885c1c48a021198e7e1e9a48ce9d02a86dd0a1848f00bdbf3c1c92724"

die() { echo "error: $*" >&2; exit 1; }
step() { echo; echo "==> $*"; }

[[ "$(uname -s)" == "Linux" ]] || die "AppImages can only be built on Linux"
[[ "$(uname -m)" == "x86_64" ]] || die "only x86_64 is supported (linuxdeploy tool hashes below are x86_64 builds)"
for tool in cmake ninja curl sha256sum; do
    command -v "$tool" >/dev/null || die "missing required tool: $tool"
done

# ---- Qt ---------------------------------------------------------------------------------------------------------
QT_PREFIX="${QT_ROOT_DIR:-${CMAKE_PREFIX_PATH:-}}"
if [[ -z "${QMAKE:-}" ]]; then
    if [[ -n "$QT_PREFIX" && -x "$QT_PREFIX/bin/qmake" ]]; then
        QMAKE="$QT_PREFIX/bin/qmake"
    elif command -v qmake6 >/dev/null; then
        QMAKE="$(command -v qmake6)"
    else
        die "Qt not found: set QT_ROOT_DIR (e.g. ~/Qt/6.8.3/gcc_64) or QMAKE"
    fi
fi
[[ -n "$QT_PREFIX" ]] || QT_PREFIX="$("$QMAKE" -query QT_INSTALL_PREFIX)"
export QMAKE

# ---- download tools (pinned + verified) -----------------------------------------------------------------------------
fetch_tool() {  # name url sha256
    local name="$1" url="$2" sha="$3" dest="$TOOLS_DIR/$1"
    if [[ -x "$dest" ]] && echo "$sha  $dest" | sha256sum --check --status; then return 0; fi
    echo "downloading $name"
    curl --fail --location --silent --show-error --retry 3 --output "$dest" "$url"
    echo "$sha  $dest" | sha256sum --check --status || die "sha256 mismatch for $name (expected $sha)"
    chmod +x "$dest"
}

mkdir -p "$TOOLS_DIR" "$OUT_DIR"
step "linuxdeploy tools"
fetch_tool linuxdeploy-x86_64.AppImage \
    "https://github.com/linuxdeploy/linuxdeploy/releases/download/$LINUXDEPLOY_VERSION/linuxdeploy-x86_64.AppImage" \
    "$LINUXDEPLOY_SHA256"
fetch_tool linuxdeploy-plugin-qt-x86_64.AppImage \
    "https://github.com/linuxdeploy/linuxdeploy-plugin-qt/releases/download/$PLUGIN_QT_VERSION/linuxdeploy-plugin-qt-x86_64.AppImage" \
    "$PLUGIN_QT_SHA256"

# ---- configure / build / install into the AppDir ---------------------------------------------------------------------
if [[ "${SKIP_BUILD:-0}" != "1" ]]; then
    step "configure"
    cmake_args=(
        -S "$ROOT" -B "$BUILD_DIR" -G "Ninja Multi-Config"
        -DCMAKE_PREFIX_PATH="$QT_PREFIX"
        -DWPIEMU_BUILD_TESTS=OFF
        -DWPIEMU_DEPLOY_QT=OFF                      # linuxdeploy bundles Qt
        -DWPIEMU_WPILIB_DOWNLOAD="$WPILIB_DOWNLOAD"
        -DWPIEMU_RUNVIEW_FETCH_SDL2="$FETCH_SDL2")
    [[ -z "${WPIEMU_WPILIB_ROOT:-}" ]] || cmake_args+=(-DWPIEMU_WPILIB_ROOT="$WPIEMU_WPILIB_ROOT")
    cmake "${cmake_args[@]}"

    step "build (Release)"
    cmake --build "$BUILD_DIR" --config Release --target wpiemu_app halsim_wpiemu
fi

VERSION="$(sed -n 's/^CMAKE_PROJECT_VERSION:[A-Z]*=//p' "$BUILD_DIR/CMakeCache.txt")"
[[ -n "$VERSION" ]] || die "could not read the project version from $BUILD_DIR/CMakeCache.txt"
WPILIB_YEAR="$(sed -n 's/^WPIEMU_WPILIB_SEASON:STRING=//p' "$BUILD_DIR/CMakeCache.txt")"
[[ "$WPILIB_YEAR" =~ ^20[0-9]{2}$ ]] || die "could not read the WPILib season from $BUILD_DIR/CMakeCache.txt"

step "install into $APPDIR/usr"
rm -rf "$APPDIR"
cmake --install "$BUILD_DIR" --config Release --prefix "$APPDIR/usr"
[[ -x "$APPDIR/usr/bin/wpiemu_app" ]] || die "wpiemu_app was not installed"
EXT="$(find "$APPDIR/usr/lib/wpiemu" -name 'libhalsim_wpiemu.so' -print -quit 2>/dev/null || true)"
[[ -n "$EXT" ]] || die "libhalsim_wpiemu.so was not installed to usr/lib/wpiemu"
mkdir -p "$APPDIR/apprun-hooks"
install -m 0644 "$ROOT/packaging/linux/apprun-hooks/wpiemu.sh" "$APPDIR/apprun-hooks/wpiemu.sh"

# ---- bundle Qt + build the AppImage -------------------------------------------------------------------------------------
step "linuxdeploy"
export ARCH=x86_64
export APPIMAGE_EXTRACT_AND_RUN=1            # no FUSE in containers / CI
export LINUXDEPLOY_OUTPUT_VERSION="$VERSION"
export QML_SOURCES_PATHS="$ROOT/app"         # QML is compiled in as resources, so point the scanner at the sources
export EXTRA_QT_PLUGINS="tls"                # HTTPS backends of the in-app updater (QtNetwork)
export PATH="$TOOLS_DIR:$PATH"               # linuxdeploy finds linuxdeploy-plugin-qt next to itself / on PATH
# libwpiHal.so is resolved at run time by the robot process that loads the extension, so it is never bundled
# (--exclude-library below), but linuxdeploy fails unless it can locate the library while scanning the extension.
wpihal="$(find "$BUILD_DIR/_wpilib" "${WPIEMU_WPILIB_ROOT:-/nonexistent}" -name 'libwpiHal.so' -print -quit 2>/dev/null || true)"
[[ -z "$wpihal" ]] || export LD_LIBRARY_PATH="$(dirname "$wpihal")${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
rm -f "$OUT_DIR"/*.AppImage
(
    cd "$OUT_DIR"
    # (see the libwpiHal.so note above)
    "$TOOLS_DIR/linuxdeploy-x86_64.AppImage" \
        --appdir "$APPDIR" \
        --plugin qt \
        --exclude-library 'libwpiHal.so*' \
        --executable "$APPDIR/usr/bin/wpiemu_app" \
        --desktop-file "$APPDIR/usr/share/applications/wpiemu.desktop" \
        --icon-file "$APPDIR/usr/share/icons/hicolor/256x256/apps/wpiemu.png" \
        --output appimage
)

built="$(find "$OUT_DIR" -maxdepth 1 -name '*.AppImage' -print -quit)"
[[ -n "$built" ]] || die "linuxdeploy did not produce an AppImage"
FINAL="$OUT_DIR/wpi-emulator-$VERSION-linux-x86_64.AppImage"
mv -f "$built" "$FINAL"
chmod +x "$FINAL"
(cd "$OUT_DIR" && sha256sum "$(basename "$FINAL")" > "$(basename "$FINAL").sha256")

# ---- verify: runs with a scrubbed environment and carries the extension ----------------------------------------------------------
step "verify $FINAL"
env -i PATH=/usr/bin:/bin HOME="${HOME:-/tmp}" APPIMAGE_EXTRACT_AND_RUN=1 "$FINAL" --version
verify_dir="$BUILD_DIR/verify"
rm -rf "$verify_dir"; mkdir -p "$verify_dir"
(cd "$verify_dir" && APPIMAGE_EXTRACT_AND_RUN=1 "$FINAL" --appimage-extract "usr/lib/wpiemu/$WPILIB_YEAR/*" >/dev/null)
(cd "$verify_dir" && APPIMAGE_EXTRACT_AND_RUN=1 "$FINAL" --appimage-extract 'usr/plugins/tls/*' >/dev/null)
[[ -f "$verify_dir/squashfs-root/usr/lib/wpiemu/$WPILIB_YEAR/libhalsim_wpiemu.so" ]] || die "extension missing from the AppImage"
compgen -G "$verify_dir/squashfs-root/usr/plugins/tls/*backend*" >/dev/null || die "Qt TLS backend missing from the AppImage (the updater needs HTTPS)"
rm -rf "$verify_dir"

echo
echo "AppImage: $FINAL"
