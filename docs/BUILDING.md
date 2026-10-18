# Building

## Prerequisites
- CMake >= 3.25 (presets), Git, Python 3 (boundary check), a C++20 compiler
  (MSVC 2022, GCC 12+/Clang 15+, Apple Clang 15+), Ninja (non-VS presets).
- Qt 6.8 LTS (msvc2022_64 / gcc_64 / macos) with the `qtquick3d` and `qtshadertools` modules.
- Box3D v0.1.0 and doctest v2.4.11 are fetched automatically (FetchContent, pinned tags).
- WPILib 2026.2.1 `hal-cpp` / `wpiutil-cpp` / `ntcore-cpp` (headers + desktop `wpiHal` / `wpiutil` / `ntcore` libraries) for the HALSim
  extension and the HAL / vendor / vision bridges. CMake finds them in the WPILib installer's Maven cache
  (`C:\Users\Public\wpilib\2026\maven`, `~/wpilib/2026/maven`), or in `-DWPIEMU_WPILIB_ROOT=<dir>` /
  `$WPIEMU_WPILIB_ROOT`, and extracts them into `build/<tree>/_wpilib/<version>`. On a machine without WPILib add
  `-DWPIEMU_WPILIB_DOWNLOAD=ON` to fetch the zips from frcmaven (needs network; this is what CI does). Without WPILib
  configure still succeeds, but the extension is not built with HAL support and the real-HAL tests do not run.
  **ntcore** (`ntcore-cpp` + `wpinet-cpp`) is located by the same logic (`halsim_ext/cmake/WpiEmuWpilib.cmake` creates the imported
  target `wpiemu_wpilib_ntcore` and sets `WPIEMU_NTCORE_FOUND` when both are found; the staging step completes an older stage that
  predates ntcore support). It is needed only to build the vision bridges' ntcore binding (headers; at run time the binding uses the
  ntcore the robot already loaded, [vision.md](vision.md) section 7). Without it the extension still builds and loads, the vision
  bridges are registered but never publish (simulated cameras are inert), and the bridges' fake-backend tests still build and run.
  An already-extracted root (`include/hal/HAL.h`, `include/wpi/string.h`, desktop shared libraries)
  is also accepted through either the cache or environment. The resolved extracted root is passed
  to sibling HAL/vendor bridges for both discovery routes; the original user cache input is preserved.
- Optional: SDL2 for gamepads in the run view (found with `find_package(SDL2 CONFIG)`; `-DWPIEMU_RUNVIEW_FETCH_SDL2=ON`
  downloads and builds it). Without it the keyboard joystick still works.

### Installing Qt (any OS, outside the repo)
```powershell
pip install aqtinstall
python -m aqt install-qt windows desktop 6.8.3 win64_msvc2022_64 -O C:\Qt -m qtshadertools qtquick3d
# linux:  python -m aqt install-qt linux desktop 6.8.3 linux_gcc_64 -O ~/Qt -m qtshadertools qtquick3d
# macOS:  python -m aqt install-qt mac desktop 6.8.3 clang_64 -O ~/Qt -m qtshadertools qtquick3d
```
On this dev machine Qt lives at `C:\Qt\6.8.3\msvc2022_64`.

## Configure / build / test (Windows, Visual Studio 2022 generator)
No special shell is needed.
```powershell
cmake --preset windows-vs2022 -DCMAKE_PREFIX_PATH=C:/Qt/6.8.3/msvc2022_64
cmake --build --preset windows-vs2022-debug --parallel     # or windows-vs2022-release
ctest --preset windows-vs2022-debug
```

## Configure / build / test (Windows, Ninja + MSVC)
Must run in the same process as `vcvars64.bat` (the path below is the Professional edition; use your edition's folder,
e.g. `Community` or `BuildTools`):
```powershell
cmd /c 'call "C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvars64.bat" >nul && cmake --preset windows-ninja -DCMAKE_PREFIX_PATH=C:/Qt/6.8.3/msvc2022_64 && cmake --build --preset windows-ninja-release && ctest --preset windows-ninja-release'
```

## Linux / macOS
```sh
cmake --preset linux  -DCMAKE_PREFIX_PATH=$HOME/Qt/6.8.3/gcc_64     # or: --preset macos (clang_64)
cmake --build --preset linux-release
ctest --preset linux-release
```
The presets also read `$QT_ROOT_DIR` (set by `jurplel/install-qt-action` in CI) when
`CMAKE_PREFIX_PATH` is not passed on the command line.
macOS defaults to deployment minimum 13.0 before compiler initialization in the root
`CMakeLists.txt`; the bundle plist derives its minimum from the same configured value.
An explicit `-DCMAKE_OSX_DEPLOYMENT_TARGET=<version>` overrides it. Newer-runner smoke tests
do not prove runtime compatibility with the minimum supported OS.
Apple's libc++ marks floating-point `std::from_chars` / `std::to_chars` unavailable below macOS 13.3/26.0, so
`core/runtime/src/config/json_codec.cpp` uses its locale-independent stream fallback on Apple (do not use floating-point
`charconv` in new code). CI and releases run on `macos-14`: Qt 6.8.3 links the AGL framework, which the macOS 26 SDK
(`macos-latest`) no longer ships.

### Linux in Docker from Windows or macOS (matches the release runner)
`tools/docker/linux` builds an `ubuntu:22.04` image with the CI package set, CMake >= 3.25 and Qt 6.8.3, and runs the
same steps as `.github/workflows/{ci,release}.yml` (boundary check, configure with WPILib download, build, ctest, install
smoke test, AppImage). The checkout is mounted read-only and synced into the container (CRLF is stripped from shell and
packaging files); build output stays in the `wpiemu-linux-build` volume, so reruns are incremental.
```powershell
.\tools\docker\linux\run.ps1 all                       # check_boundaries + configure + build + ctest + install + AppImage
.\tools\docker\linux\run.ps1 test -CtestArgs '-R "^launcher"'
.\tools\docker\linux\run.ps1 appimage                  # needs a previous build; artifacts: /build/release/pkg in the volume
.\tools\docker\linux\run.ps1 shell                     # interactive shell (gdb is installed)
```
The container is headless (`QT_QPA_PLATFORM=offscreen`); it does not validate Quick 3D rendering. macOS cannot be
virtualized on non-Apple hardware: iterate on it with the CI workflow, which can run a single platform:
```sh
gh workflow run ci.yml --ref <branch> -f platforms=macos -f macos_universal=true
gh run watch
```

## Useful options
| Option | Default | Meaning |
|---|---|---|
| `WPIEMU_BUILD_APP` | ON | Build the Qt app (needs Qt). Turn OFF for Qt-free builds of everything else. |
| `WPIEMU_BUILD_TESTS` | ON | Unit tests + CTest checks. |
| `WPIEMU_BUILD_PLUGIN_TEMPLATE` | ON | Keep `plugins/_template` compiling. |
| `WPIEMU_WARNINGS_AS_ERRORS` | OFF | `/WX` / `-Werror` for first-party code (CI-quality check; see below). |
| `WPIEMU_BUILD_UI_DEMOS` | ON | Standalone `wpiemu_runview_demo` / `wpiemu_editor_demo` executables. |
| `WPIEMU_WPILIB_ROOT`, `WPIEMU_WPILIB_VERSION`, `WPIEMU_WPILIB_DOWNLOAD` | auto, 2026.2.1, OFF | Where the WPILib HAL comes from (see Prerequisites). |
| `WPIEMU_RUNVIEW_SDL2`, `WPIEMU_RUNVIEW_FETCH_SDL2` | ON, ON | Gamepad support in the run view. |

Build trees live in `build/<preset>/`; binaries and DLLs land in `build/<preset>/bin/<Config>/`. Use `-B <dir>` after the
preset to pick another directory, e.g. a clean warnings-as-errors check (this is how the release verification is done):
```powershell
cmd /c 'call "C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvars64.bat" >nul && cmake --preset windows-ninja -B build\final -DCMAKE_PREFIX_PATH=C:/Qt/6.8.3/msvc2022_64 -DWPIEMU_WARNINGS_AS_ERRORS=ON && cmake --build build\final --config Release && ctest --test-dir build\final -C Release'
```
Do not mix generators in one build dir (each preset has its own). `build/` is git-ignored.

## Tests
- `wpiemu_tests` (doctest): every `tests/**/*.cpp` is globbed; each `TEST_CASE` is a CTest test.
  Test case names must not contain `;` (CTest discovery splits on it).
- `tests/sdk/`: SDK contract tests plus a generated self-contained-include check (one TU per
  public SDK header, linked into `wpiemu_tests` to surface ODR violations). New SDK headers are picked up automatically.
- `boundaries.check` / `boundaries.self_test`: `tools/check_boundaries.py` (see ADR 0004).
- `infrastructure.release_tools`: dependency-free Python checksum and Mach-O metadata regressions
  (`python -B -m unittest discover -s tools\tests -v`), without executing macOS tools.
- `app.version`: `wpiemu_app --version` headless smoke test.
- Separate executables with their own CTest names: `halsim_ext.*` (extension + fake emulator), `host.*`
  (`wpiemu_host_tests`, `RobotSession` against synthetic projects), `app.ui_tests` (shell/wizard, offscreen),
  `app_editor.tests`, `runview.ui_tests` (set `QT_QPA_PLATFORM=offscreen`; Qt Quick 3D rendering is not covered offscreen).
- `ctest -N` lists everything; `ctest -R <regex>` runs a subset. The `e2e.shelby_2026` test is skipped unless opted in (below).
- Vision: `tests/bridges_vision` (encoders, fake publisher), the camera cases in `tests/runtime`, `tests/ipc`, `tests/app_editor/test_cameras.cc`
  and `tests/app_runview` run in the normal suites; see [vision.md](vision.md) section 9.

Run the architecture check directly: `python tools/check_boundaries.py`.

The offline configure-only WPILib regression fixture uses synthetic headers/libraries, without
FetchContent, downloads or a build. It checks extracted/Maven layouts, cache/environment inputs,
tests ON/OFF, both bridge bindings and cache preservation in a separate tree:
```powershell
python tools\check_wpilib_configure.py --generator "Visual Studio 17 2022" --architecture x64
```
On Linux/macOS use `--generator Ninja` (or an installed generator). The fixture needs CMake
and a working compiler, not Qt or WPILib. CI runs it on each platform. Its eight cases are
intentionally separate from normal CTest to avoid recursive configuration and shared build-tree changes.

## Notes
- Distributable packages (NSIS installer, dmg, AppImage): see [packaging.md](packaging.md).
- Box3D forces a static MSVC CRT in its own CMake; `cmake/WpiEmuDependencies.cmake` overrides
  it to the dynamic CRT so it links with first-party code.
- To run `wpiemu_app.exe` from the build tree, put `C:\Qt\6.8.3\msvc2022_64\bin` on `PATH`
  (or run `windeployqt`).

## End-to-end test (opt-in)
`e2e.shelby_2026` runs a real WPILib project through Gradle, the real `halsim_wpiemu` and the real session (about 30 s, needs WPILib 2026 with its JDK and offline
Maven, and the project). It is skipped (exit code 77, "SKIPPED e2e: ...") unless `WPIEMU_E2E` is set:
```powershell
$env:WPIEMU_E2E = "1"                              # required
$env:WPIEMU_E2E_PROJECT = "C:\path\to\robot"       # default: ..\FRC-Shelby-2026OS next to this repository (copied to %TEMP%, never modified)
# optional: WPIEMU_E2E_VERBOSE=1 (robot console), WPIEMU_E2E_KEEP=1 (keep the copy), WPIEMU_E2E_LAUNCH=direct (simulateExternal launch)
ctest --preset windows-ninja-release -L e2e --output-on-failure
```
The shell flavour (`RealRuntimeHost` + `ShellController`, extension found through the build-tree fallback) is the `real host e2e` case of `wpiemu_app_tests`,
enabled by the same variable.

### Vision sample end-to-end (opt-in)
`samples/vision_robot/{java,cpp}` are WPILib 2026.2.1 sample robots that read a Limelight (`LimelightHelpers`, vendored into the
sample) and a PhotonVision camera (PhotonLib `v2026.3.4` vendordep) and feed pose estimators; the robot code is unmodified by the
emulator. `e2e.vision_sample` (target `wpiemu_e2e_vision_tests`, `app/host/tests/e2e_vision_test.cpp`, label `e2e`) copies the
sample to `%TEMP%` (never modified), builds a `RobotConfig` with a Limelight and a PhotonVision camera matching the sample's mounts, runs it
through Gradle with the real extension and compares the robot's own console output with the emulator's ground truth (details in
[vision.md](vision.md) section 9). It needs the same prerequisites as above (WPILib toolchain, offline Maven; `JAVA_HOME` may have to
point at WPILib's JDK, e.g. `C:\Users\Public\wpilib\2026\jdk`) and a build with ntcore found (`WPIEMU_NTCORE_FOUND`):
```powershell
$env:WPIEMU_E2E = "1"                                                   # required, otherwise the test is skipped
$env:WPIEMU_E2E_VISION_PROJECT = "C:\path\to\wpi-emulator\samples\vision_robot\java"   # default: samples\vision_robot\java; use an absolute path
$env:WPIEMU_E2E_VISION_LANG = "java"                                    # java (default) | cpp, must match the project
ctest --preset windows-vs2022-release -R "^e2e[.]vision_sample$" --output-on-failure
```
For the C++ sample set `WPIEMU_E2E_VISION_PROJECT` to `samples\vision_robot\cpp` and `WPIEMU_E2E_VISION_LANG=cpp` (it needs the
Visual Studio C++ toolchain for desktop simulation). `WPIEMU_E2E_KEEP=1` and `WPIEMU_E2E_VERBOSE=1` (robot console) behave as for the other e2e test.
