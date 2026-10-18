# Packaging and releases

Distributables are produced from the normal CMake build: `install()` rules + CPack (`cmake/WpiEmuInstall.cmake`,
included at the end of the root `CMakeLists.txt`), plus one script per platform that CMake cannot express.

| Platform | Artifact | Produced by | Status |
|---|---|---|---|
| Windows x64 | `wpi-emulator-<ver>-windows-x64.exe` (NSIS installer) and `.zip` (portable) | `cpack` (NSIS + ZIP generators) | **built and verified locally** (silent install, `--version` with only the system PATH, uninstall) |
| macOS universal (arm64 + x86_64) | `wpi-emulator-<ver>-macos-arm64-x64.dmg` | `cpack -G DragNDrop` + `macdeployqt` (via Qt deploy API) + `packaging/macos/*` | **built and verified in CI** (`macos-14`): both slices present, minimum OS 13.0, `--version` on arm64 and under Rosetta, no unbundled dependencies; unsigned unless the Apple secrets exist |
| Linux x86_64 | `wpi-emulator-<ver>-linux-x86_64.AppImage` | `packaging/linux/build-appimage.sh` (linuxdeploy + linuxdeploy-plugin-qt) | **built and verified in CI** (`ubuntu-22.04`) and locally in the Docker image (`tools/docker/linux`): `--version` with an empty environment, extension present in the image |

`.github/workflows/release.yml` builds all three on a `v*` tag (or manually) and attaches them to a **draft** GitHub release.
`.github/workflows/ci.yml` additionally runs the boundary check, `ctest` (headless, `QT_QPA_PLATFORM=offscreen`) and an
`cmake --install` smoke test (`wpiemu_app --version` from the install tree) on every push / PR.

## Decisions

* **Windows installer = CPack + NSIS.** CPack drives it natively (no extra project files), it supports silent install
  (`/S /D=<dir>`), per-machine install, shortcuts, in-place upgrade and uninstall, and NSIS is a one-line install on the
  CI runner. WiX/MSI was not chosen (needs WiX toolchain plus GUID/upgrade-code bookkeeping); it can be added later as another
  CPack generator without touching the install rules. A portable ZIP is always produced next to the installer. If `makensis`
  is not found at configure time only the ZIP is built (a status message says so).
* **Qt deployment uses the Qt 6.8 deploy API** (`qt_generate_deploy_app_script` -> `windeployqt` / `macdeployqt`) so the
  same rules serve `cmake --install` and CPack. The QML files are compiled in as plain resources (no `qt_add_qml_module`),
  so the automatic QML import scan sees nothing; the rules therefore pass `--qmldir <repo>/app` (`-qmldir=` on macOS). New
  `.qml` files anywhere under `app/` are picked up automatically; `QtQuick`, `QtQuick3D` and its plugin are bundled.
  `qsvg`, `qsvgicon`, the other `imageformats`, `platforms/qwindows` and the styles come from `windeployqt`'s module scan
  (the app links `Qt6::Svg`).
* **Linux uses AppImage**, not CPack: linuxdeploy-plugin-qt knows Qt's plugin / QML / xcb dependency tree, and an AppImage
  runs on any distro with a glibc at least as new as the build machine's. The CMake install rules are still used (with
  `WPIEMU_DEPLOY_QT=OFF`) to populate the AppDir; `cpack` on Linux produces a plain `.tar.gz` of the install tree
  (no bundled Qt) for distro packagers.
* **The WPILib runtime is not redistributed.** `halsim_wpiemu` imports `wpiHal` only; the robot process that loads the
  extension already has the WPILib libraries loaded and the loader binds to them. `-DWPIEMU_PACKAGE_WPILIB_RUNTIME=ON`
  copies `wpiHal`/`wpiutil` next to the extension if you ever need a standalone copy.

## Install layout and the extension lookup contract

```
Windows (C:\Program Files\WPI Emulator)        macOS (WPI Emulator.dmg)                         Linux (AppImage / prefix)
  bin\wpiemu_app.exe, Qt6*.dll, qt.conf          wpiemu_app.app/Contents/MacOS/wpiemu_app         usr/bin/wpiemu_app
  extensions\halsim_wpiemu.dll (+ VC++ runtime)  wpiemu_app.app/Contents/PlugIns/wpiemu/            usr/lib/wpiemu/libhalsim_wpiemu.so
  plugins\ qml\ translations\                      libhalsim_wpiemu.dylib                           usr/lib, usr/plugins (linuxdeploy)
  THIRD-PARTY-NOTICES.txt                        wpiemu_app.app/Contents/{Frameworks,PlugIns,Resources/qml}  usr/share/{applications,icons,metainfo}
```

The app finds the extension **relative to its own executable**, never through the registry. A build-tree
absolute fallback is tried last for development, not required by packaged installs.
Let `appDir = QCoreApplication::applicationDirPath()` (the directory of the running executable) and `lib` =
`halsim_wpiemu.dll` / `libhalsim_wpiemu.so` / `libhalsim_wpiemu.dylib`. The first existing file wins:

1. `$WPIEMU_EXTENSION` (full file path; explicit override for developers/tests)
2. `$WPIEMU_EXTENSION_DIR/<lib>` (set by the AppImage hook to `$APPDIR/usr/lib/wpiemu`)
3. `appDir/<lib>` (build tree: CMake puts the library next to the executable, `build/<preset>/bin/<Config>/`)
4. `appDir/../extensions/<lib>` (Windows install)
5. `appDir/../lib/wpiemu/<lib>` (Linux install prefix and AppImage)
6. `appDir/../PlugIns/wpiemu/<lib>` (macOS bundle: `Contents/MacOS/../PlugIns/wpiemu`)
7. `appDir/../../../<lib>` (macOS build tree: `bin/<Config>/wpiemu_app.app/Contents/MacOS` -> `bin/<Config>`)
8. Optional CMake-provided build-tree fallback (development only)

The returned path must be absolute and canonical (it is exported as `WPIEMU_EXT` / `HALSIM_EXTENSIONS`, which the robot's
JVM and gradle consume from another working directory). `cmake/WpiEmuInstall.cmake` exposes the install-relative
directory as `WPIEMU_INSTALL_EXTENSION_DIR` if a build-time consumer needs it.

This is implemented by the Qt-free `core::launcher::locateHalsimExtension` in
`core/runtime/include/wpiemu/core/launcher/extension_locator.h`. `RealRuntimeHost` supplies
`QCoreApplication::applicationDirPath()` and the environment, then passes the resolved path
through `RobotSession` to inspection/launch requests. Missing files produce the existing reinstall/rebuild
diagnostic; the resolver does not create or write files. Regression coverage is in
`tests/runtime/test_extension_locator.cpp` (`extension locator:` cases), including priorities,
packaged/build layouts and canonical paths.

### Environment of child processes (Linux AppImage; Windows/macOS need nothing)

The AppImage runs with `WPIEMU_PACKAGED=appimage`, `APPDIR`, `APPIMAGE` and linuxdeploy-plugin-qt's `QT_PLUGIN_PATH`,
`QML2_IMPORT_PATH`, `QT_QPA_PLATFORM_PLUGIN_PATH`, plus `$APPDIR/...` entries prepended to `LD_LIBRARY_PATH`, `PATH`,
`XDG_DATA_DIRS`. The robot launcher (gradle, the WPILib JDK, native robots) must not inherit them: when
`WPIEMU_PACKAGED` is set `makeLaunchEnvironment` drops those variables and removes every `PATH`-style entry that starts with `$APPDIR`
from the child environment. The Windows installer does **not** modify `PATH`, and the macOS bundle uses `@rpath`/`@executable_path`
only.
The `packaged env:` cases in `tests/runtime/test_launcher.cpp` cover this sanitization.

### Other requirements on the app

* Never write below `applicationDirPath()` (`Program Files` is read-only for users, an AppImage is a read-only mount).
  Settings, generated init scripts, logs and caches must go to `QStandardPaths` locations or the project's `.wpiemu/`.
* Keep the target names `wpiemu_app` (bundle `wpiemu_app.app`) and `halsim_wpiemu`; the install rules reference them.
* Any additional runtime file the app needs (data directories, plugin directories) must be added to the install rules in
  `cmake/WpiEmuInstall.cmake` and resolved relative to `applicationDirPath()` like the extension.

## Building the packages

### Windows (verified)

```powershell
cmake -S . -B build\packaging -G "Visual Studio 17 2022" -A x64 `
      -DCMAKE_PREFIX_PATH=C:/Qt/6.8.3/msvc2022_64 -DWPIEMU_WPILIB_DOWNLOAD=ON -DWPIEMU_RUNVIEW_FETCH_SDL2=ON
cmake --build build\packaging --config Release --parallel
cd build\packaging
cpack -C Release -B pkg            # pkg\wpi-emulator-<ver>-windows-x64.exe / .zip (+ .sha256)
```
Prerequisites: NSIS 3 (`winget install NSIS.NSIS`, or `choco install nsis`). `cpack -G ZIP` needs nothing extra.
Install tree only (no installer): `cmake --install build\packaging --config Release --prefix C:\stage`.

Check a build like CI does (system PATH only, so no Qt is picked up from the machine):
```powershell
$dest = "$PWD\inst"; Start-Process (Get-ChildItem pkg\*.exe).FullName -ArgumentList '/S',"/D=$dest" -Wait
$env:PATH = "$env:SystemRoot\System32;$env:SystemRoot"; & "$dest\bin\wpiemu_app.exe" --version
Start-Process "$dest\Uninstall.exe" -ArgumentList '/S',"_?=$dest" -Wait
```
The installer is per-machine (`RequestExecutionLevel admin`), 64-bit, creates Start-menu and desktop shortcuts and replaces an
older install in place. It ships the VC++ runtime app-locally (next to `wpiemu_app.exe` **and** next to the extension, whose
host process is the robot's JVM). The executable carries the application icon and version resource
(`packaging/windows/wpiemu.rc.in`).

*Authenticode signing is not wired up* (no certificate). Add a `CPACK_POST_BUILD_SCRIPTS` script that runs `signtool` /
Azure Trusted Signing on the `.exe` from CI secrets, and sign `wpiemu_app.exe`/`halsim_wpiemu.dll` in an install-time step
before CPack if SmartScreen reputation matters.

### macOS

```sh
cmake --preset macos -B build/release -DCMAKE_PREFIX_PATH=$HOME/Qt/6.8.3/macos \
      -DCMAKE_OSX_ARCHITECTURES="arm64;x86_64" \
      -DWPIEMU_WPILIB_DOWNLOAD=ON -DWPIEMU_RUNVIEW_FETCH_SDL2=ON
cmake --build build/release --config Release
(cd build/release && cpack -C Release -G DragNDrop -B pkg)       # pkg/wpi-emulator-<ver>-macos-arm64-x64.dmg
packaging/macos/notarize.sh build/release/pkg/wpi-emulator-*.dmg  # optional: sign dmg, notarize, staple
python tools/package_checksums.py build/release/pkg --verify
```
* `install()` puts `wpiemu_app.app` at the root of the image; the Qt deploy script runs `macdeployqt` on the installed
  bundle (frameworks, plugins, `Contents/Resources/qml`, `-appstore-compliant`). `packaging/macos/Info.plist.in` and the
  `.icns` icon are attached through target properties in `WpiEmuInstall.cmake` (`WPIEMU_MACOS_BUNDLE_ID`, default
  `org.wpiemulator.app`; change it to the identifier you own before shipping).
* **rpath of the extension:** `libhalsim_wpiemu.dylib` is installed to `Contents/PlugIns/wpiemu/` with
  `INSTALL_RPATH=@loader_path`; its build-tree rpath is stripped by `cmake --install`. It depends on `libwpiHal.dylib` through
  the install name recorded in the WPILib library; the robot process (which already loaded WPILib) satisfies it. This is
  *unverified* on macOS; if the dylib cannot be resolved, check `otool -L` and, as a fallback, set
  `-DWPIEMU_PACKAGE_WPILIB_RUNTIME=ON` and `install_name_tool -change` to `@loader_path/libwpiHal.dylib`.
* **Signing:** `packaging/macos/sign_bundle.cmake` is a `CPACK_PRE_BUILD_SCRIPTS` hook. With `WPIEMU_CODESIGN_IDENTITY`
  (and optionally `WPIEMU_CODESIGN_KEYCHAIN`) in the environment it signs every dylib/plugin (including the HALSim
  extension), the frameworks and the bundle with the hardened runtime and `packaging/macos/entitlements.plist`
  (`allow-jit` for the QML engine). Without it the bundle keeps the ad-hoc signature `macdeployqt` applies; users then have to
  right-click -> Open (or `xattr -dr com.apple.quarantine`).
* **Notarization:** `packaging/macos/notarize.sh` signs the dmg, runs `xcrun notarytool submit --wait`, staples and
  assesses it. Credentials come from the environment only: either `APPLE_ID` + `APPLE_TEAM_ID` + `APPLE_APP_PASSWORD`, or an
  App Store Connect API key (`APPLE_API_KEY_PATH`, `APPLE_API_KEY_ID`, `APPLE_API_ISSUER`). With no credentials it
  prints a notice and exits 0. Both signing-only and stapled paths refresh and verify the DMG's
  `.sha256` after their final mutation (CPack's earlier digest is no longer valid after signing).
* **Architectures:** the release is universal (`-DCMAKE_OSX_ARCHITECTURES="arm64;x86_64"`; the official Qt 6.8 macOS kit and
  WPILib's `osxuniversal` libraries are universal, Box3D and the fetched SDL2 build for both slices). The package name then
  contains `arm64-x64`. Without the option the build uses the host architecture. The verify step runs `lipo -archs` on the
  app and the extension and, when Rosetta is available, runs the x86_64 slice. The default deployment target is 13.0, set once before compiler initialization
  in the root `CMakeLists.txt` for preset and manual/release builds. The plist's
  `LSMinimumSystemVersion` is generated from `CMAKE_OSX_DEPLOYMENT_TARGET`, including an explicit
  command-line override. Release verification inspects each app/extension Mach-O slice with
  `packaging/macos/check_minimum_os.py` and rejects a recorded minimum above the plist value.
  This inspection is not proof of runtime support on the minimum OS; that smoke run remains unverified.

### Linux AppImage

```sh
QT_ROOT_DIR=$HOME/Qt/6.8.3/gcc_64 bash packaging/linux/build-appimage.sh      # -> build/appimage/dist/*.AppImage
```
From Windows or macOS use the Docker environment instead: `.\tools\docker\linux\run.ps1 all` (see [BUILDING.md](BUILDING.md)).
`libwpiHal.so` (needed by the extension, supplied by the robot process) is located through `LD_LIBRARY_PATH` while linuxdeploy
scans the extension and excluded from the bundle with `--exclude-library`.
The script configures + builds Release, installs into `build/appimage/AppDir/usr`, downloads **pinned and sha256-checked**
`linuxdeploy` and `linuxdeploy-plugin-qt` (`1-alpha-20250213`) into `build/appimage/tools`, runs them with
`--appimage-extract-and-run` (no FUSE needed), renames the result and verifies it (`--version` with a scrubbed environment,
extension present inside the image). `SKIP_BUILD=1 BUILD_DIR=<existing build>` reuses a build (what the release workflow does).
Build on the oldest distro you support (CI uses `ubuntu-22.04`): the AppImage and `libhalsim_wpiemu.so` both need at
least that glibc/libstdc++. The `packaging/linux/apprun-hooks/wpiemu.sh` hook is sourced by `AppRun` and exports
`WPIEMU_PACKAGED=appimage` and `WPIEMU_EXTENSION_DIR`. Mark scripts executable in git (`git update-index --chmod=+x`); the
workflow calls them through `bash`, and `packaging/.gitattributes` forces LF endings for `*.sh`.

System packages needed to build: see the `apt-get install` list in `.github/workflows/release.yml` (Qt's xcb/xkbcommon/GL
libraries, `libx11-dev`-style headers for the static SDL2, `libudev-dev`, `libasound2-dev`).

### Options

| Option | Default | Effect |
|---|---|---|
| `WPIEMU_DEPLOY_QT` | ON | run windeployqt / macdeployqt during `cmake --install` (Windows, macOS only; Linux uses linuxdeploy) |
| `WPIEMU_PACKAGE_WPILIB_RUNTIME` | OFF | also install `wpiHal` / `wpiutil` next to the extension |
| `WPIEMU_MACOS_BUNDLE_ID` | `org.wpiemulator.app` | `CFBundleIdentifier` |
| `WPIEMU_WPILIB_DOWNLOAD` | OFF | fetch the WPILib HAL zips from frcmaven (always ON in CI) |
| `WPIEMU_RUNVIEW_FETCH_SDL2` | ON | build SDL2 (static) for gamepad support (always ON in CI; no DLL to ship) |

## Release workflow (`.github/workflows/release.yml`)

* Triggers: tag `v*` (publishes a **draft** release with all artifacts and `.sha256` files; a tag containing `-` is marked
  pre-release) and `workflow_dispatch` (artifacts only, plus the `allow_stub_extension` switch).
* `preflight`: the tag (`v0.1.0`) must equal `project(... VERSION ...)` in the root `CMakeLists.txt`.
* `package` (matrix `windows-2022`, `ubuntu-22.04`, `macos-14`): install Qt **6.8.3** with `jurplel/install-qt-action`
  (`qtquick3d`, `qtshadertools`; the action caches the Qt install), cache the WPILib downloads, configure, build, run the full
  `ctest` suite (headless), package, verify the package, upload.
* **WPILib in CI:** `-DWPIEMU_WPILIB_DOWNLOAD=ON` makes `halsim_ext/cmake/WpiEmuWpilib.cmake` download
  `edu/wpi/first/{hal/hal-cpp,wpiutil/wpiutil-cpp}/2026.2.1/*-{headers,<platform>}.zip` from
  `https://frcmaven.wpi.edu/artifactory/release/` (the `WPILIB_VERSION` env must match `WPIEMU_WPILIB_VERSION`; all
  `windowsx86-64`, `linuxx86-64`, `osxuniversal` artifacts were checked to exist). If the download fails the configure
  still succeeds and the extension becomes the loud stub. The release job **fails in that case** ("Require the real HALSim
  extension") unless it was started manually with `allow_stub_extension`, so a stub is never shipped by accident.
* Windows verification installs the NSIS package silently and runs it with only the system PATH; macOS mounts the dmg and
  checks that no dependency points outside the bundle; Linux runs the AppImage in an empty environment.
* After all signing/stapling and package verification, `tools/package_checksums.py` refreshes and
  verifies each distributable's SHA256 sidecar immediately before upload on every platform.
  `--verify` checks without rewriting; use it again after any manual artifact operation.
* Secrets (all optional; unsigned builds are produced without them): `MACOS_CERTIFICATE` (base64 Developer ID `.p12`),
  `MACOS_CERTIFICATE_PASSWORD`, `MACOS_SIGN_IDENTITY`, `APPLE_ID`, `APPLE_TEAM_ID`, `APPLE_APP_PASSWORD`. The certificate
  is imported into a temporary keychain on the runner; nothing is stored in the repository.

## Icon

`packaging/icons/wpiemu.svg` is original artwork (a robot seen from above inside a circular field). `make_icons.py`
(pure Python, no dependencies) renders the same geometry to `wpiemu-<size>.png`, `wpiemu.ico` and `wpiemu.icns`; re-run it
after editing the shapes. The Windows `.rc`, the macOS bundle and the Linux hicolor icon theme all use these files.

## Licensing

`packaging/THIRD-PARTY-NOTICES.txt` is installed with the product. Qt is used under the LGPL v3 (dynamic linking, unmodified
binaries). Review and complete it (Box3D and SDL2 license texts, a product LICENSE file) before a public release; if a
`LICENSE` file exists at the repository root the NSIS installer shows it as the license page automatically. The AppStream
metadata (`wpiemu.metainfo.xml`) has no `project_license` yet for the same reason.

## Known gaps / not verified

* Linux and macOS packages are built and smoke-tested in CI, but never executed against a real robot project, and the macOS
  `@loader_path` resolution of `libwpiHal.dylib` from the robot process is still unverified.
* Signing, notarization and the macOS minimum-OS (13.0) runtime smoke run are exercised only with real Apple credentials /
  an older macOS; CI proves the Mach-O minimum-version metadata, not runtime support on 13.0.
* macOS CI uses `macos-14`; `macos-latest` (macOS 26 SDK) fails to link Qt 6.8.3 (missing AGL framework).
* The extension's behavior against the robot's own `libwpiHal` on Linux/macOS is still the open item of ADR 0001
  (`RTLD_LOCAL`), independent of packaging.
* Authenticode signing, an MSI, `.deb`/`.rpm` and Linux arm64 AppImages are not provided.
