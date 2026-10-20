# Building and testing

Read this when you need a development build, a focused test run or build diagnostics.
Prerequisites: a checkout and a terminal at the repository root.

## Prerequisites

- Git, Python 3, CMake 3.25+, and a C++20 compiler. Windows uses MSVC 2022;
  Linux/macOS use GCC/Clang or Apple Clang. Non-Visual-Studio presets also require Ninja.
- A matching Qt 6.8 kit with Quick 3D and Shader Tools; the Windows examples use
  `C:\Qt\6.8.3\msvc2022_64`. Omit Qt for a Qt-free build using the option below.
- CMake fetches pinned Box3D and doctest sources; see [dependency declarations](../cmake/WpiEmuDependencies.cmake).
- WPILib desktop HAL/wpiutil artifacts enable the real extension. CMake searches the installer's
  Maven cache (`C:\Users\Public\wpilib\2026\maven` or `~/wpilib/2026/maven`).
  Supply another Maven/extracted root with `WPIEMU_WPILIB_ROOT`, or enable downloads below.
  **Without HAL, the extension is a failing stub and real-HAL tests are absent**, not a usable robot integration.
  Optional ntcore/wpinet artifacts enable vision publishing; without them cameras are inert.

## Windows

The Visual Studio generator needs no developer shell. Substitute your Qt kit:
```powershell
cmake --preset windows-vs2022 -DCMAKE_PREFIX_PATH="C:\Qt\6.8.3\msvc2022_64" -DWPIEMU_WARNINGS_AS_ERRORS=ON
cmake --build --preset windows-vs2022-release --parallel
ctest --preset windows-vs2022-release
```

For Ninja/MSVC, keep environment setup and all commands in the **same `cmd.exe` process**.
Change `Professional` to your installed edition:
```powershell
$env:QT_ROOT_DIR = "C:\Qt\6.8.3\msvc2022_64"
cmd.exe /c 'call "C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvars64.bat" >nul && cmake --preset windows-ninja && cmake --build --preset windows-ninja-release --parallel && ctest --preset windows-ninja-release'
```

## Linux and macOS

Use the matching Qt kit and preset:
```sh
cmake --preset linux -DCMAKE_PREFIX_PATH="$HOME/Qt/6.8.3/gcc_64" -DWPIEMU_WARNINGS_AS_ERRORS=ON
cmake --build --preset linux-release --parallel
ctest --preset linux-release
# On macOS instead:
cmake --preset macos -DCMAKE_PREFIX_PATH="$HOME/Qt/6.8.3/macos" -DWPIEMU_WARNINGS_AS_ERRORS=ON
cmake --build --preset macos-release --parallel
ctest --preset macos-release
```

Presets read `QT_ROOT_DIR` unless overridden; build/test presets also have `-debug` variants.
Trees are `build\<preset>` with binaries under `bin\<Config>`; never mix generators in one tree.
Packaging and deployment belong in [releasing](releasing.md).

## Configure options
Pass these to configuration, not to `cmake --build`.

| Cache option | Default | Effect |
|---|---|---|
| `WPIEMU_BUILD_APP` | ON | OFF removes the Qt application requirement |
| `WPIEMU_BUILD_TESTS` | ON | Build tests and register CTest checks |
| `WPIEMU_BUILD_PLUGIN_TEMPLATE` | ON | Compile the season skeleton |
| `WPIEMU_WARNINGS_AS_ERRORS` | OFF | Promote first-party warnings to errors |
| `WPIEMU_BUILD_UI_DEMOS` | ON | Build editor/run-view visual demos |
| `WPIEMU_WPILIB_ROOT` | auto | Maven cache, year directory or extracted artifacts; also read from the environment |
| `WPIEMU_WPILIB_VERSION` | 2026.2.1 | WPILib artifact version |
| `WPIEMU_WPILIB_DOWNLOAD` | OFF | Fetch missing WPILib artifacts from frcmaven |
| `WPIEMU_RUNVIEW_SDL2` | ON | SDL2 gamepads when available; keyboard input works without SDL2 |
| `WPIEMU_RUNVIEW_FETCH_SDL2` | ON | Fetch SDL2 if no system package is found; OFF avoids that download |

## Tests and checks

List tests, select a case/suite, or run the checks directly:
```powershell
ctest --preset windows-vs2022-release -N
ctest --preset windows-vs2022-release -R "^runner: handshake assigns session, epoch, step and the negotiated watchdog$"
ctest --preset windows-vs2022-release -R "^(app[.]ui_tests|app_editor[.]tests|runview[.]ui_tests)$"
python tools\check_boundaries.py
python tools\check_boundaries.py --self-test
python tools\check_docs.py
python tools\check_wpilib_configure.py --generator "Visual Studio 17 2022" --architecture x64
```
The last command checks WPILib discovery offline with synthetic artifacts, without a build.
On Linux/macOS use `--generator Ninja` instead and omit `--architecture x64`.
The Qt-free doctest executable discovers `tests/**/*.cpp`, excluding the separate run-view suite.
Editor tests use `tests/app_editor/*.cc` to avoid that glob. Reconfigure after adding tests;
each discovered `TEST_CASE` becomes a CTest entry. Names must be unique and contain **no semicolons**.
SDK public headers automatically get stand-alone include checks.
CTest supplies offscreen Qt settings and the Windows Qt DLL path for UI suites. Direct Qt test
binaries need `QT_QPA_PLATFORM=offscreen` and the kit's `bin` on `PATH`; doctest accepts
`--test-case="exact case name"`. Offscreen does **not** validate Quick 3D rendering: use
`wpiemu_editor_demo` or `wpiemu_runview_demo` interactively. For source-writing conventions see
[Contributing](CONTRIBUTING.md#tests).

## Opt-in robot-project tests

These need the WPILib JDK/toolchain, the project's Gradle wrapper and cached Maven dependencies,
plus a real HAL-enabled build. Originals are untouched: tests run copied projects.
```powershell
$env:WPIEMU_E2E = "1"
ctest --preset windows-vs2022-release -R "^e2e[.]swerve_sample$"
$env:WPIEMU_E2E_VISION_PROJECT = (Resolve-Path samples\vision_robot\java).Path
$env:WPIEMU_E2E_VISION_LANG = "java"
ctest --preset windows-vs2022-release -R "^e2e[.]vision_sample$"
```
The swerve check drives [`samples\swerve_robot\java`](../samples/swerve_robot/README.md); `WPIEMU_E2E_PROJECT` selects
another robot with that CAN layout. Vision also needs ntcore. For C++, select `samples\vision_robot\cpp` and language `cpp`.
Set `JAVA_HOME` to the WPILib JDK if discovery fails; `WPIEMU_E2E_VERBOSE=1` shows robot output.
Without opt-in/prerequisites these tests report skipped; a skip does not verify integration.

## Linux in Docker

With Docker and PowerShell, [the Linux tool](../tools/docker/linux/run.ps1) builds/tests, installs and
packages in a container. The checkout is read-only; incremental output stays in `wpiemu-linux-build`.
```powershell
.\tools\docker\linux\run.ps1 all
.\tools\docker\linux\run.ps1 test -CtestArgs '-R "^launcher"'
.\tools\docker\linux\run.ps1 shell
```
The container cannot validate rendering; Linux GUI dependencies are in its [Dockerfile](../tools/docker/linux/Dockerfile).
Direct Windows app/demo runs need `C:\Qt\6.8.3\msvc2022_64\bin` on `PATH`.

Next: [contribution conventions](CONTRIBUTING.md).
