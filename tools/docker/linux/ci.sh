#!/usr/bin/env bash
# Container entrypoint: mirrors the Linux steps of .github/workflows/{ci,release}.yml.
#   wpiemu-ci configure|build|test|install|appimage|all|shell
# The Windows checkout is mounted read-only at /host-src and synced to /work/src (shell scripts get LF endings,
# since a Windows working tree usually has CRLF). Build output lives in /build, a persistent volume.
set -euo pipefail

SRC=/work/src
BUILD="${BUILD_DIR:-/build/release}"
CTEST_ARGS="${CTEST_ARGS:-}"

sync_source() {
    mkdir -p "$SRC"
    rsync -rt --delete --exclude=/build --exclude=/.git --exclude=/.vs --exclude=/.vscode --exclude=/.idea \
        /host-src/ "$SRC/"
    find "$SRC" \( -name '*.sh' -o -path "$SRC/packaging/linux/*" -type f \) -not -path '*/build/*' -exec sed -i 's/\r$//' {} +
}
configure() {
    cmake -S "$SRC" -B "$BUILD" -G "Ninja Multi-Config" \
        -DCMAKE_PREFIX_PATH="$QT_ROOT_DIR" \
        -DWPIEMU_BUILD_TESTS=ON \
        -DWPIEMU_WPILIB_DOWNLOAD=ON \
        -DWPIEMU_RUNVIEW_FETCH_SDL2=ON "$@"
}
build() { cmake --build "$BUILD" --config Release --parallel; }
tests() { ctest --test-dir "$BUILD" -C Release --output-on-failure $CTEST_ARGS; }
install_smoke() {
    rm -rf /build/stage
    cmake --install "$BUILD" --config Release --prefix /build/stage
    /build/stage/bin/wpiemu_app --version
    test -f /build/stage/lib/wpiemu/libhalsim_wpiemu.so
}
appimage() {
    BUILD_DIR="$BUILD" OUT_DIR="$BUILD/pkg" SKIP_BUILD=1 bash "$SRC/packaging/linux/build-appimage.sh"
}

cmd="${1:-all}"
shift || true
sync_source
cd "$SRC"
case "$cmd" in
    configure) configure "$@" ;;
    build) build ;;
    test) tests ;;
    install) install_smoke ;;
    appimage) appimage ;;
    all) python3 tools/check_boundaries.py --root .; configure "$@"; build; tests; install_smoke; appimage ;;
    shell) exec bash ;;
    *) exec "$cmd" "$@" ;;
esac
