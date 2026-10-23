# Build and test

Read this when you need a development build, a focused test run, or build diagnostics.
Run commands from the repository root.

## Quick start on Windows

For the Visual Studio 2022 generator, open PowerShell with CMake and Git on `PATH`.
Use a Qt 6.8.3 MSVC kit; the examples assume `C:\Qt\6.8.3\msvc2022_64`.
Set `QT_ROOT_DIR` to your kit if it is installed elsewhere:

```powershell
$env:QT_ROOT_DIR = "C:\Qt\6.8.3\msvc2022_64"
cmake --preset windows-vs2022 -DWPIEMU_WARNINGS_AS_ERRORS=ON
cmake --build --preset windows-vs2022-release --parallel
ctest --preset windows-vs2022-release
```

The configure step creates `build\windows-vs2022`. The first configure may download
WPILib, Box3D, doctest, and SDL2 sources. The build step compiles the app and tests.
CTest runs registered checks and reports opt-in robot tests as skipped unless enabled.

## Prerequisites

- Git, Python 3, CMake 3.25+, and a C++20 compiler. Windows uses MSVC 2022.
- Linux and macOS use GCC/Clang or Apple Clang with Ninja.
- Building the app needs a matching Qt kit with Quick 3D, Shader Tools, and Multimedia.
  See the [version pins](releasing.md#version-pins) before choosing another kit.
- CMake fetches pinned [Box3D](../cmake/WpiEmuDependencies.cmake) and
  [doctest](../cmake/WpiEmuTests.cmake) sources.
- The real HAL extension needs WPILib desktop HAL and wpiutil artifacts. CMake searches the
  selected season's Maven cache (for example, `C:\Users\Public\wpilib\<year>\maven` or
  `~/wpilib/<year>/maven`). Set `WPIEMU_WPILIB_ROOT` to another Maven cache or extracted
  artifact root, or allow CMake to download them.
  Without HAL, the extension is a failing stub and real-HAL tests are not built.
- Optional ntcore/wpinet artifacts enable vision publishing. Without them, camera
  publishing is inactive in 2026. The 2027 HAL requires ntcore, wpinet and datalog even
  without cameras; an incomplete extracted 2027 root fails configuration.

You can omit Qt by configuring with `-DWPIEMU_BUILD_APP=OFF`. The Qt UI and its tests
will then be unavailable.

## Windows generators

The Visual Studio generator does not need a developer shell. Substitute your Qt kit:

```powershell
cmake --preset windows-vs2022 -DCMAKE_PREFIX_PATH="C:\Qt\6.8.3\msvc2022_64" -DWPIEMU_WARNINGS_AS_ERRORS=ON
cmake --build --preset windows-vs2022-release --parallel
ctest --preset windows-vs2022-release
```

For Ninja with MSVC, the compiler environment must be set up in the same `cmd.exe`
process that runs CMake, the build, and CTest. Change `Professional` to your installed
Visual Studio edition:

```powershell
$env:QT_ROOT_DIR = "C:\Qt\6.8.3\msvc2022_64"
cmd.exe /c 'call "C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvars64.bat" >nul && cmake --preset windows-ninja -DWPIEMU_WARNINGS_AS_ERRORS=ON && cmake --build --preset windows-ninja-release --parallel && ctest --preset windows-ninja-release'
```

Do not split this command across separate shells. The `vcvars64.bat` environment does
not carry over to a later PowerShell session.

## Linux and macOS

Install Ninja and a Qt kit matching the project pins, then use the corresponding preset:

```sh
cmake --preset linux -DCMAKE_PREFIX_PATH="$HOME/Qt/6.8.3/gcc_64" -DWPIEMU_WARNINGS_AS_ERRORS=ON
cmake --build --preset linux-release --parallel
ctest --preset linux-release
```

On macOS, use the macOS kit and preset instead:

```sh
cmake --preset macos -DCMAKE_PREFIX_PATH="$HOME/Qt/6.8.3/macos" -DWPIEMU_WARNINGS_AS_ERRORS=ON
cmake --build --preset macos-release --parallel
ctest --preset macos-release
```

Presets read `QT_ROOT_DIR` unless you pass another prefix. Each generator needs its own
build tree; do not mix generators in one tree. Build and test presets also have `-debug`
variants. Build trees are under `build\<preset>` and binaries under `bin\<Config>`.
For packaging, see [Releasing](releasing.md).

## Configure options

Pass these options to `cmake --preset`, not `cmake --build`.

| Option | Default | Effect |
|---|---|---|
| `WPIEMU_BUILD_APP` | ON | Build the Qt application. OFF removes the Qt requirement. |
| `WPIEMU_BUILD_TESTS` | ON | Build tests and register CTest checks. |
| `WPIEMU_BUILD_PLUGIN_TEMPLATE` | ON | Compile the season-plugin skeleton. |
| `WPIEMU_WARNINGS_AS_ERRORS` | OFF | Promote first-party warnings to errors. |
| `WPIEMU_BUILD_UI_DEMOS` | ON | Build the editor and run-view demo applications. |
| `WPIEMU_WPILIB_SEASON` | The season pinned by `wpilib.default` | Select a WPILib season profile from `versions.json`. |
| `WPIEMU_SEASONS` | The selected build season | Semicolon-separated extensions to compose, for example `"2026;2027"`. Extra seasons use isolated, Qt-free sub-builds and must have real HAL artifacts. |
| `WPIEMU_EXTENSION_ONLY` | OFF | Build just the extension and its real-HAL tests, without app, physics or plugins. |
| `WPIEMU_REQUIRE_REAL_HAL` | OFF | Fail configuration instead of producing a stub when HAL is missing. |
| `WPIEMU_WPILIB_ROOT` | auto | Use a Maven cache, year directory, or extracted artifacts. Also read from the environment. |
| `WPIEMU_WPILIB_VERSION` | the selected season's pin | Override the profile's WPILib artifact version. |
| `WPIEMU_WPILIB_DOWNLOAD` | OFF; presets set ON | Download missing WPILib artifacts. On Windows, this can also copy a local JDK into the public WPILib year folder; bundled VC++ runtime DLLs are omitted. |
| `WPIEMU_RUNVIEW_SDL2` | ON | Enable SDL2 gamepads when available. Keyboard input works without SDL2. |
| `WPIEMU_RUNVIEW_FETCH_SDL2` | ON | Fetch SDL2 when no system package is found. Set OFF to avoid this download. |

## Multi-season builds

The normal developer build selects one season. Distribution builds compose both:

```powershell
cmake -S . -B build\dual-season -G "Visual Studio 18 2026" -A x64 -DCMAKE_PREFIX_PATH="C:\Qt\6.8.3\msvc2022_64" "-DWPIEMU_SEASONS=2026;2027" -DWPIEMU_WPILIB_DOWNLOAD=ON -DWPIEMU_REQUIRE_REAL_HAL=ON -DWPIEMU_WARNINGS_AS_ERRORS=ON
cmake --build build\dual-season --config Release --parallel
ctest --test-dir build\dual-season -C Release -R halsim_ext --output-on-failure
```

The primary build remains 2026/C++20; the 2027 extension sub-build uses C++23.
Building both requires Visual Studio 2026 or GCC 14+ (an appropriate recent Xcode on macOS).
Use a separate build directory rather than changing generators in an existing one.
`WPIEMU_WPILIB_ROOT_<year>` optionally supplies artifacts for an extra season;
the primary `WPIEMU_WPILIB_ROOT` is never reused across seasons.
Extra extensions stage under `seasons/<year>/stage` and are copied to the app's
build-tree `<year>` directory. Install and CPack include every requested season.

With JDK 25 and the native toolchain installed, set `WPIEMU_E2E=1` and run
`ctest --test-dir build\dual-season -C Release -R "^e2e.season_2027$" --output-on-failure`.
This runs both [2027 sample robots](../samples/season_2027/README.md).
`WPIEMU_E2E_JAVA_HOME` optionally chooses their JDK; `WPIEMU_E2E_LAUNCH=direct`
exercises external launch instead of Gradle init-script injection.

## Run tests and repository checks

Use CTest to list tests, select a focused case, or run the configured suite:

```powershell
ctest --preset windows-vs2022-release -N
ctest --preset windows-vs2022-release -R "^motor catalog carries the WPILib DCMotor table$" --output-on-failure
ctest --preset windows-vs2022-release -R "^(app[.]ui_tests|app_editor[.]tests|runview[.]ui_tests)$"
python tools\check_boundaries.py
python tools\check_boundaries.py --self-test
python tools\check_docs.py
python tools\check_versions.py
python -B -m unittest discover -s tools\tests -v
python tools\check_wpilib_configure.py --generator "Visual Studio 17 2022" --architecture x64
```

The WPILib configure check uses synthetic artifacts and does not build or load a robot.
On Linux/macOS, pass only `--generator Ninja`. The Qt-free doctest executable discovers
`tests/**/*.cpp`; the run-view suite is separate. Editor tests use `tests/app_editor/*.cc`
to avoid that source glob. CMake watches this source glob, and building the test
executable discovers each `TEST_CASE` as a CTest test. Test names must be unique and
contain no semicolons. SDK public headers also receive stand-alone include checks.

CTest provides offscreen Qt settings and the Windows Qt DLL path for UI suites through
`wpiemu_qt_test_environment(<test>)`. Direct Qt test binaries need
`QT_QPA_PLATFORM=offscreen` and the kit's `bin` directory on `PATH`. Doctest accepts
`--test-case="exact case name"`.

## What a passing test proves

A unit test proves the behavior covered by its assertions under that test's conditions.
The full CTest suite also exercises registered component and UI checks. A clean build
proves the configured targets compile; it does not prove every runtime path works.

Qt tests run offscreen. They can check widgets and view models, but they do not validate
interactive Quick 3D rendering. For that, launch `wpiemu_editor_demo` or
`wpiemu_runview_demo` and inspect the view yourself. A successful run does not establish
robot integration or behavior on other operating systems.

## Opt-in robot-project tests

These tests run copied sample projects; they do not edit the originals. They need the
WPILib JDK/toolchain, the project's Gradle wrapper and cached Maven dependencies, and a
build with real HAL support. Set `WPIEMU_E2E=1` to enable the swerve sample test:

```powershell
$env:WPIEMU_E2E = "1"
ctest --preset windows-vs2022-release -R "^e2e[.]swerve_sample$"
```

The test runs [`samples\swerve_robot\java`](../samples/swerve_robot/README.md).
Set `WPIEMU_E2E_PROJECT` to another robot project with the same CAN layout.
For vision, also set the project path and language:

```powershell
$env:WPIEMU_E2E_VISION_PROJECT = (Resolve-Path samples\vision_robot\java).Path
$env:WPIEMU_E2E_VISION_LANG = "java"
ctest --preset windows-vs2022-release -R "^e2e[.]vision_sample$"
```

Vision also needs ntcore. For C++, use `samples\vision_robot\cpp` and set the language to
`cpp`. Set `JAVA_HOME` to the WPILib JDK if discovery fails. Set
`WPIEMU_E2E_VERBOSE=1` to show robot output. A skipped test means its opt-in or
prerequisites were missing; a skip does not verify integration. Even a passing sample
test covers only that project and setup, not every robot or physical hardware.

## Linux in Docker

With Docker and PowerShell, the [Linux helper](../tools/docker/linux/run.ps1) builds,
tests, installs, and packages in a container. The checkout is read-only; incremental
output is kept in `wpiemu-linux-build`.

```powershell
.\tools\docker\linux\run.ps1 all
.\tools\docker\linux\run.ps1 test -CtestArgs '-R "^launcher"'
.\tools\docker\linux\run.ps1 shell
```

The container cannot validate rendering. Its GUI dependencies are listed in the
[Dockerfile](../tools/docker/linux/Dockerfile). Direct Windows app or demo launches need
`C:\Qt\6.8.3\msvc2022_64\bin` on `PATH`.

Next: [contribution conventions](CONTRIBUTING.md).
