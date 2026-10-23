# Runtime and debug switches

**Read this when:** choosing an app command-line option, environment override, or build/packaging
switch. This is the source-checked inventory; it distinguishes user inputs from values the
launcher or tests supply.
**Prerequisites:** [Building](../BUILDING.md) for build and test commands;
[extension and IPC](extension-ipc.md) for the process boundary.

## Application command line

These options belong to `wpiemu_app`, not the editor demo or CTest executables. Qt's parser
also supplies `--help`; the app adds its Qt `--version` option. The exact `--version` argument
is checked before Qt starts and prints the emulator, SDK, IPC protocol, and game-plugin counts.

| Option | Scope and accepted value | Default and precedence | Source |
|---|---|---|---|
| `--help` | Print app usage and exit. | Qt parser behavior. | [`app/main.cpp`](../../app/main.cpp) |
| `--version` | Print version/build information and exit. | Checked before Qt argument parsing; if present, this early path wins over other arguments. | [`app/main.cpp`](../../app/main.cpp) |
| `--demo` | Run the simulated-device demo host instead of launching a robot project. No value. | Off; selecting it changes the host mode. | [`app/main.cpp`](../../app/main.cpp) |
| `--project <folder>` | Open the WPILib project at `<folder>` on startup. | No explicit project; when omitted, restore the last project. When supplied, it takes precedence over restoring the last project. | [`app/main.cpp`](../../app/main.cpp) |

## User overrides for a real run

Unset and empty values read through `sdk::readEnvironment` are treated alike. Boolean values
use the case-insensitive, whitespace-trimmed spellings `1`, `true`, `yes`, `on`, `0`, `false`,
`no`, and `off`; invalid values fall back to the documented default. `WPIEMU_LAUNCH_MODE` is
the exception: only the exact lowercase value `direct` selects direct launch.

| Name | Scope and accepted value | Default and precedence | Source |
|---|---|---|---|
| `WPIEMU_LAUNCH_MODE` | App real-run launcher: `direct` uses the `simulateExternal` launch; other values select the generated Gradle init-script launch. | Gradle init-script mode. | [`app/common/real_runtime_host.cpp`](../../app/common/real_runtime_host.cpp) |
| `WPIEMU_GRADLE_OFFLINE` | App real-run launcher: boolean; true adds Gradle's `--offline`. | False; invalid values also act false. | [`app/common/real_runtime_host.cpp`](../../app/common/real_runtime_host.cpp) |
| `WPIEMU_PRESTART_POLLING` | App real-run launcher: boolean; true asks the extension to poll for devices while the robot constructor is running. | False; this is off by default because pre-start polling can crash a JVM with some CTRE bridge versions. It is translated to the launcher-owned `WPIEMU_LIVE_DISCOVERY` value. | [`app/common/real_runtime_host.cpp`](../../app/common/real_runtime_host.cpp), [`app/host/robot_session.cpp`](../../app/host/robot_session.cpp) |
| `WPIEMU_EXTENSION` | App extension locator: a path to the extension library. | Unset/empty means auto-locate; a supplied file is tried before `WPIEMU_EXTENSION_DIR`, installed-layout candidates, and the build-tree fallback. A missing/invalid candidate does not prevent trying later candidates. | [`core/runtime/src/launcher/extension_locator.cpp`](../../core/runtime/src/launcher/extension_locator.cpp) |
| `WPIEMU_EXTENSION_DIR` | App extension locator: a directory containing the platform's `halsim_wpiemu` library. | Unset/empty means auto-locate; a supplied directory is tried after `WPIEMU_EXTENSION` and before installed-layout candidates and the build-tree fallback. | [`core/runtime/src/launcher/extension_locator.cpp`](../../core/runtime/src/launcher/extension_locator.cpp) |
| `JAVA_HOME` | JDK candidate for a robot run; must identify a runnable JDK. | Lower priority than the JDK selected in Settings and the project's season WPILib JDK; higher priority than `java` on `PATH` and the downloaded JDK. See [runtime JDK selection](runtime.md#launcher-boundary). The launcher passes its selected JDK to Gradle. | [`app/host/robot_session.cpp`](../../app/host/robot_session.cpp), [`core/runtime/src/launcher/robot_launcher.cpp`](../../core/runtime/src/launcher/robot_launcher.cpp) |

The extension lookup order is: explicit file, explicit directory plus the platform library
name, paths relative to the app's installed location, then the build-tree fallback. The
installed layouts are described in [Releasing](../releasing.md).

## Extension environment

The HALSim extension reads these values in the robot process. Values below are extension
configuration inputs unless marked **launcher-set**. A setting absent from the process
environment uses its stated default. When a primary name and its alias are both nonempty, the
primary name wins; an invalid primary value falls back to the default rather than retrying the
alias.

| Name | Classification; scope and accepted value | Default and precedence | Source |
|---|---|---|---|
| `WPIEMU_IPC_ENDPOINT` | **Launcher-set (do not set).** IPC address used to connect to this emulator run. | No endpoint means the extension stays inert. Takes precedence over `WPIEMU_ENDPOINT`; the emulator supplies both with the same generated address. | [`core/runtime/src/launcher/robot_launcher.cpp`](../../core/runtime/src/launcher/robot_launcher.cpp), [`halsim_ext/src/config.cpp`](../../halsim_ext/src/config.cpp) |
| `WPIEMU_ENDPOINT` | **Launcher-set alias (do not set).** Same IPC address format as `WPIEMU_IPC_ENDPOINT`. | Used only if the primary name is absent or empty. The launcher supplies both names. | [`sdk/include/wpiemu/sdk/core/environment.h`](../../sdk/include/wpiemu/sdk/core/environment.h), [`halsim_ext/src/config.cpp`](../../halsim_ext/src/config.cpp) |
| `WPIEMU_CLOCK` | **Launcher-set (do not set).** `master` / `lockstep` follows the emulator's clock; `realtime` / `free` forces real-time HAL clocking. | Defaults to following the emulator. The launcher supplies `master` or `realtime`; an invalid value keeps the default. | [`core/runtime/src/launcher/robot_launcher.cpp`](../../core/runtime/src/launcher/robot_launcher.cpp), [`halsim_ext/src/config.cpp`](../../halsim_ext/src/config.cpp) |
| `WPIEMU_PROGRAM_NAME` | **Launcher-set (do not set).** Nonempty display/log name, trimmed before use. | Defaults to the detected executable name. The launcher supplies the robot project directory's basename when available. | [`core/runtime/src/launcher/robot_launcher.cpp`](../../core/runtime/src/launcher/robot_launcher.cpp), [`halsim_ext/src/config.cpp`](../../halsim_ext/src/config.cpp) |
| `WPIEMU_LIVE_DISCOVERY` | **Launcher-set (do not set).** Boolean controlling device discovery during robot construction. | False; the app derives it from `WPIEMU_PRESTART_POLLING` and supplies `1` or `0`. | [`app/host/robot_session.cpp`](../../app/host/robot_session.cpp), [`halsim_ext/src/config.cpp`](../../halsim_ext/src/config.cpp) |
| `WPIEMU_IPC_CONNECT_TIMEOUT_MS` | Extension debugging: unsigned milliseconds in `0`–`600000`. | `10000` ms. Alias: `WPIEMU_CONNECT_TIMEOUT_MS`. Primary wins when nonempty. | [`sdk/include/wpiemu/sdk/core/environment.h`](../../sdk/include/wpiemu/sdk/core/environment.h), [`halsim_ext/include/wpiemu/halsim_ext/config.h`](../../halsim_ext/include/wpiemu/halsim_ext/config.h), [`halsim_ext/src/config.cpp`](../../halsim_ext/src/config.cpp) |
| `WPIEMU_CONNECT_TIMEOUT_MS` | Extension debugging alias for the IPC connect timeout; same range and units. | Used only if the primary name is absent or empty; otherwise ignored. | [`halsim_ext/include/wpiemu/halsim_ext/config.h`](../../halsim_ext/include/wpiemu/halsim_ext/config.h), [`halsim_ext/src/config.cpp`](../../halsim_ext/src/config.cpp) |
| `WPIEMU_IPC_WATCHDOG_MS` | Extension debugging: unsigned 32-bit milliseconds proposed to the emulator. | `0` means no proposal, so the emulator's negotiated/default watchdog value applies. Alias: `WPIEMU_WATCHDOG_MS`; primary wins when nonempty. | [`sdk/include/wpiemu/sdk/core/environment.h`](../../sdk/include/wpiemu/sdk/core/environment.h), [`halsim_ext/src/config.cpp`](../../halsim_ext/src/config.cpp) |
| `WPIEMU_WATCHDOG_MS` | Extension debugging alias for the proposed watchdog timeout; unsigned 32-bit milliseconds. | Used only if the primary name is absent or empty. | [`halsim_ext/include/wpiemu/halsim_ext/config.h`](../../halsim_ext/include/wpiemu/halsim_ext/config.h), [`halsim_ext/src/config.cpp`](../../halsim_ext/src/config.cpp) |
| `WPIEMU_HANDSHAKE_TIMEOUT_MS` | Extension debugging: unsigned milliseconds in `1`–`600000`. | `5000` ms. | [`halsim_ext/include/wpiemu/halsim_ext/config.h`](../../halsim_ext/include/wpiemu/halsim_ext/config.h), [`halsim_ext/src/config.cpp`](../../halsim_ext/src/config.cpp) |
| `WPIEMU_WATCHDOG` | Extension debugging: `on` / `off`, or a supported boolean spelling. | On. `off` disables the extension watchdog; malformed values keep the default. | [`halsim_ext/include/wpiemu/halsim_ext/config.h`](../../halsim_ext/include/wpiemu/halsim_ext/config.h), [`halsim_ext/src/config.cpp`](../../halsim_ext/src/config.cpp) |
| `WPIEMU_RECONNECT` | Extension debugging: boolean enabling reconnection after an IPC loss. | True. | [`halsim_ext/include/wpiemu/halsim_ext/config.h`](../../halsim_ext/include/wpiemu/halsim_ext/config.h), [`halsim_ext/src/config.cpp`](../../halsim_ext/src/config.cpp) |
| `WPIEMU_DISABLE` | Extension debugging: boolean disabling extension attachment. | False. When true, the extension loads inertly and the robot runs normally. | [`halsim_ext/include/wpiemu/halsim_ext/config.h`](../../halsim_ext/include/wpiemu/halsim_ext/config.h), [`halsim_ext/src/config.cpp`](../../halsim_ext/src/config.cpp) |
| `WPIEMU_LOG_LEVEL` | Extension debugging: `trace`, `debug`, `info`, `warn` / `warning`, or `error` (case-insensitive). | `info`. | [`sdk/include/wpiemu/sdk/core/environment.h`](../../sdk/include/wpiemu/sdk/core/environment.h), [`halsim_ext/src/config.cpp`](../../halsim_ext/src/config.cpp) |
| `WPIEMU_DISABLE_BRIDGES` | Extension debugging: comma-separated, case-sensitive bridge-name prefixes to omit (for example `ctre,hal.can`). | Empty means keep all bridges. Each bridge whose name starts with any listed prefix is omitted. | [`halsim_ext/src/extension.cpp`](../../halsim_ext/src/extension.cpp) |

The extension-only aliases exist for compatibility with older launcher spellings; prefer the
`WPIEMU_IPC_*` primary names. The emulator owns the IPC endpoint, clock choice, program name,
and live-discovery setting during a normal app launch. Do not override launcher-set values.
For IPC and lifecycle semantics, see [Extension and IPC](extension-ipc.md).

## Values set by launchers and packaging

These are implementation hand-offs, not user switches. The emulator sets `WPIEMU_EXT` in the
Gradle process; its generated init script transfers that extension path to GradleRIO's
`HALSIM_EXTENSIONS` for the robot process. The AppImage startup hook sets
`WPIEMU_PACKAGED=appimage` and `WPIEMU_EXTENSION_DIR`; AppImage itself supplies `APPDIR`.
When the package marker is nonempty, the launcher removes bundled Qt/library search paths from
the child robot environment so they do not leak into Gradle, Java, or native robot code.

| Name | Scope and value/default | Precedence | Source |
|---|---|---|---|
| `WPIEMU_EXT` | **Launcher-set (do not set):** absolute extension-library path required by the generated Gradle init script. | The generated script uses this path to set `HALSIM_EXTENSIONS`. | [`robot_launcher.h`](../../core/runtime/include/wpiemu/core/launcher/robot_launcher.h), [`init_script.cpp`](../../core/runtime/src/launcher/init_script.cpp) |
| `HALSIM_EXTENSIONS` | **Launcher-set (do not set):** extension library loaded by GradleRIO in the robot process. | Set by the generated init script for simulation tasks. | [`core/runtime/src/launcher/init_script.cpp`](../../core/runtime/src/launcher/init_script.cpp) |
| `WPIEMU_PACKAGED` | **Packaging launcher-set (do not set):** AppImage hook sets the nonempty marker `appimage`. | Unset/empty disables packaged-environment cleanup; any nonempty value enables it. | [`packaging/linux/apprun-hooks/wpiemu.sh`](../../packaging/linux/apprun-hooks/wpiemu.sh), [`core/runtime/src/launcher/robot_launcher.cpp`](../../core/runtime/src/launcher/robot_launcher.cpp) |
| `APPDIR` | **Packaging launcher-set (do not set):** AppImage mount/root directory. | Used only with a nonempty package marker to identify path entries to remove from the robot's environment. | [`core/runtime/src/launcher/robot_launcher.cpp`](../../core/runtime/src/launcher/robot_launcher.cpp) |

## Test-only switches

These names are read only by test harnesses. Their defaults and effects apply only when running
the named tests; CTest's opt-in robot-project instructions are in
[Building: opt-in robot-project tests](../BUILDING.md#opt-in-robot-project-tests).

| Name | Scope and accepted value | Default and precedence | Source |
|---|---|---|---|
| `WPIEMU_E2E` | Enable the opt-in robot-project end-to-end test when nonempty; use `1` as shown in Building. | Empty/unset skips the robot E2E. | [`app/host/tests/e2e_test.cpp`](../../app/host/tests/e2e_test.cpp), [`app/tests/test_real_host.cpp`](../../app/tests/test_real_host.cpp) |
| `WPIEMU_E2E_PROJECT` | Path to an alternate robot project; the swerve test expects its supported CAN layout. | Compiled-in sample project when empty. An explicit path replaces that default. | [`app/host/tests/e2e_test.cpp`](../../app/host/tests/e2e_test.cpp), [`app/tests/test_real_host.cpp`](../../app/tests/test_real_host.cpp) |
| `WPIEMU_E2E_KEEP` | Any nonempty value keeps the temporary copied project after the test. | Empty removes the copy. | [`app/host/tests/test_support.h`](../../app/host/tests/test_support.h) |
| `WPIEMU_E2E_VERBOSE` | Any nonempty value echoes robot console output. | Off when empty. | [`app/host/tests/e2e_test.cpp`](../../app/host/tests/e2e_test.cpp), [`app/host/tests/e2e_vision_test.cpp`](../../app/host/tests/e2e_vision_test.cpp) |
| `WPIEMU_E2E_LAUNCH` | Exact `direct` selects the direct `simulateExternal` path; any other value uses the Gradle init script. | Gradle init-script launch. | [`app/host/tests/e2e_test.cpp`](../../app/host/tests/e2e_test.cpp) |
| `WPIEMU_E2E_TANK` | Any nonempty value enables the saved tank-robot test case, which still must be selected by its doctest name. | Disabled when empty. | [`app/host/tests/e2e_test.cpp`](../../app/host/tests/e2e_test.cpp) |
| `WPIEMU_E2E_VISION_PROJECT` | Path to an alternate vision robot project. | Compiled-in vision sample when empty; explicit path takes precedence. | [`app/host/tests/e2e_vision_test.cpp`](../../app/host/tests/e2e_vision_test.cpp) |
| `WPIEMU_E2E_VISION_LANG` | Vision sample language: `java` or `cpp`. | `java` when empty. | [`app/host/tests/e2e_vision_test.cpp`](../../app/host/tests/e2e_vision_test.cpp) |
| `WPIEMU_E2E_VISION_GRADLE_INIT` | Path to an existing Gradle init script added to the vision test's Gradle arguments. | Unset means normal toolchain discovery. | [`app/host/tests/e2e_vision_test.cpp`](../../app/host/tests/e2e_vision_test.cpp) |
| `WPIEMU_E2E_NETWORK` | Any set value opts the update and toolchain-provisioner tests into GitHub access/downloads; use `1`. | Unset skips those network tests. | [`app/tests/test_update.cpp`](../../app/tests/test_update.cpp), [`app/tests/test_toolchain_provisioner.cpp`](../../app/tests/test_toolchain_provisioner.cpp) |
| `QT_QPA_PLATFORM` | Qt platform plugin used by app/UI test binaries. | Test entry points set `offscreen` only when empty; an explicit value is preserved. CTest also supplies the offscreen setting. | [`app/tests/main.cpp`](../../app/tests/main.cpp), [`tests/app_editor/main.cc`](../../tests/app_editor/main.cc), [`cmake/WpiEmuQt.cmake`](../../cmake/WpiEmuQt.cmake) |

The standalone `wpiemu_editor_demo` is a development visual demo, not the product app. Its
parser also supports `--help`; the options below are owned by
[`app/editor/demo/editor_demo.cpp`](../../app/editor/demo/editor_demo.cpp).

| Option | Scope and accepted value | Default and precedence |
|---|---|---|
| `--grab <file>` | Save the rendered page to the named file, then exit. | No grab unless supplied; with `--detect`, waits for scripted detection before saving. |
| `--select <item>` | Select `chassis`, `drivetrain`, `plugin`, `zone:<id>`, `camera:<id>`, `sensor:<id>`, `wheel:<name>`, `slot:<owner>.<slot>`, or a mechanism name. | No selection unless supplied. |
| `--detect` | Run scripted device detection. No value. | Off. |
| `--tank` | Use the demo's linked-wheel tank configuration. No value. | Off. |
| `--light` | Use the light palette. No value. | Dark palette. |
| `--size <WxH>` | Window size, for example `1500x900`. | `1500x900`. |

## Build and packaging variables

These are not runtime app arguments. Use [Building: configure options](../BUILDING.md#configure-options)
for the configure workflow and [Releasing](../releasing.md) for packaging commands. CMake
cache values are set with `-D` or a configure preset; an explicit cache value takes precedence
over the declaration's default. Boolean cache options accept CMake boolean values.

| Name | Classification; scope and default | Precedence / accepted values | Source |
|---|---|---|---|
| `WPIEMU_BUILD_APP` | CMake cache option; build Qt app, default `ON`. | Boolean; explicit cache/preset value controls. | [`CMakeLists.txt`](../../CMakeLists.txt) |
| `WPIEMU_BUILD_TESTS` | CMake cache option; build/register tests, default `ON`. | Boolean; explicit cache/preset value controls. | [`CMakeLists.txt`](../../CMakeLists.txt) |
| `WPIEMU_BUILD_PLUGIN_TEMPLATE` | CMake cache option; compile the template plugin, default `ON`. | Boolean; explicit cache/preset value controls. | [`CMakeLists.txt`](../../CMakeLists.txt) |
| `WPIEMU_BUILD_UI_DEMOS` | CMake cache option; build standalone visual demos, default `ON`. | Boolean; explicit cache/preset value controls. | [`app/CMakeLists.txt`](../../app/CMakeLists.txt) |
| `WPIEMU_APP_USE_EDITOR`, `WPIEMU_APP_USE_RUNVIEW` | CMake cache options; embed the editor and run-view pages when their libraries are built, each default `ON`. | Boolean; explicit cache/preset value controls. | [`app/common/CMakeLists.txt`](../../app/common/CMakeLists.txt) |
| `WPIEMU_RUNVIEW_SDL2`, `WPIEMU_RUNVIEW_FETCH_SDL2` | CMake cache options; gamepad support and SDL2 fetching, each default `ON`. | Boolean; explicit cache/preset value controls. | [`app/runview/CMakeLists.txt`](../../app/runview/CMakeLists.txt) |
| `WPIEMU_WPILIB_ROOT` | CMake cache path for extracted WPILib files or a Maven cache; empty means auto-detect. | A valid cache path is tried before the same-named environment path; then standard cache locations are searched. | [`halsim_ext/cmake/WpiEmuWpilib.cmake`](../../halsim_ext/cmake/WpiEmuWpilib.cmake) |
| `WPIEMU_WPILIB_VERSION` | CMake cache string selecting WPILib artifact version; default is the `wpilib` pin in `versions.json`. | Explicit cache value wins over the pinned default. | [`halsim_ext/cmake/WpiEmuWpilib.cmake`](../../halsim_ext/cmake/WpiEmuWpilib.cmake) |
| `WPIEMU_WPILIB_DOWNLOAD` | CMake cache option to download missing WPILib artifacts; declaration default `OFF`. | Boolean; the standard configure presets set it `ON`, and a command-line cache override can change that. | [`halsim_ext/cmake/WpiEmuWpilib.cmake`](../../halsim_ext/cmake/WpiEmuWpilib.cmake), [`CMakePresets.json`](../../CMakePresets.json) |
| `WPIEMU_WARNINGS_AS_ERRORS` | CMake cache option; root build default `OFF`, standalone editor build default `ON`. | Boolean; explicit cache value controls. | [`cmake/WpiEmuDefaults.cmake`](../../cmake/WpiEmuDefaults.cmake), [`CMakeLists.txt`](../../CMakeLists.txt), [`app/editor/standalone/CMakeLists.txt`](../../app/editor/standalone/CMakeLists.txt) |
| `WPIEMU_QT_DOWNLOAD` | CMake cache option to download Qt if not found, default `ON`. | Boolean; explicit cache/preset value controls. | [`cmake/WpiEmuQt.cmake`](../../cmake/WpiEmuQt.cmake) |
| `WPIEMU_DEPLOY_QT` | CMake cache option to deploy Qt with installs, default `ON`. | Boolean; explicit cache/preset value controls. | [`cmake/WpiEmuInstall.cmake`](../../cmake/WpiEmuInstall.cmake) |
| `WPIEMU_PACKAGE_WPILIB_RUNTIME` | CMake cache option to install WPILib runtime libraries next to the extension, default `OFF`. | Boolean; explicit cache/preset value controls. | [`cmake/WpiEmuInstall.cmake`](../../cmake/WpiEmuInstall.cmake) |
| `WPIEMU_MACOS_BUNDLE_ID` | CMake cache string for the macOS bundle identifier; default `org.wpiemulator.app`. | Explicit cache value replaces the default. | [`cmake/WpiEmuInstall.cmake`](../../cmake/WpiEmuInstall.cmake) |
| `QT_ROOT_DIR` | Packaging/build environment path to the Qt kit. | The AppImage build uses this before `CMAKE_PREFIX_PATH`; if neither identifies Qt it can use `QMAKE` or query `qmake`. | [`packaging/linux/build-appimage.sh`](../../packaging/linux/build-appimage.sh), [`CMakePresets.json`](../../CMakePresets.json) |
| `CMAKE_PREFIX_PATH` | CMake/packaging environment prefix path; used as the Qt kit fallback when `QT_ROOT_DIR` is unset. | The AppImage script prefers `QT_ROOT_DIR`; if no prefix is supplied, it uses `QMAKE` or discovers `qmake`. | [`packaging/linux/build-appimage.sh`](../../packaging/linux/build-appimage.sh), [`CMakePresets.json`](../../CMakePresets.json) |
| `QMAKE` | Packaging environment path to `qmake`. | Used as the Qt locator when the Qt prefix is not supplied. | [`packaging/linux/build-appimage.sh`](../../packaging/linux/build-appimage.sh) |
| `WPIEMU_FETCH_SDL2` | Packaging environment boolean for the AppImage build; default `ON`. | The script translates it to `WPIEMU_RUNVIEW_FETCH_SDL2`; use `ON`/`OFF`. | [`packaging/linux/build-appimage.sh`](../../packaging/linux/build-appimage.sh) |
| `WPIEMU_WPILIB_DOWNLOAD` | Packaging environment boolean for the AppImage build; default `ON`. | Passed as the CMake cache option of the same name; an explicit cache argument wins. | [`packaging/linux/build-appimage.sh`](../../packaging/linux/build-appimage.sh) |
| `WPIEMU_CODESIGN_IDENTITY` | macOS packaging/signing environment value naming a Developer ID identity. | Unset leaves the app ad-hoc signed; a set identity signs the app and enables DMG signing during notarization. | [`packaging/macos/sign_bundle.cmake`](../../packaging/macos/sign_bundle.cmake), [`packaging/macos/notarize.sh`](../../packaging/macos/notarize.sh) |
| `WPIEMU_CODESIGN_KEYCHAIN` | Optional macOS packaging environment path to the keychain containing the signing identity. | Unset uses the default keychain; set value is passed to signing tools. | [`packaging/macos/sign_bundle.cmake`](../../packaging/macos/sign_bundle.cmake), [`packaging/macos/notarize.sh`](../../packaging/macos/notarize.sh) |
| `APPLE_API_KEY_PATH`, `APPLE_API_KEY_ID`, `APPLE_API_ISSUER` | macOS notarization credentials: private-key file path, key ID, and issuer ID. | All three must be nonempty to select API-key authentication; this set takes precedence over Apple ID credentials. Unset/incomplete skips notarization unless the other credential set is complete. | [`packaging/macos/notarize.sh`](../../packaging/macos/notarize.sh) |
| `APPLE_ID`, `APPLE_TEAM_ID`, `APPLE_APP_PASSWORD` | macOS notarization credentials: Apple ID, team ID, and app-specific password. | Used only when all three are nonempty and the API-key set is incomplete. Keep secrets in the environment/secret store, never in the repository. | [`packaging/macos/notarize.sh`](../../packaging/macos/notarize.sh) |
| `BUILD_DIR` | AppImage packaging work/build directory; default `$ROOT/build/appimage`. | Explicit environment value replaces the default; `OUT_DIR` and `TOOLS_DIR` default beneath it. | [`packaging/linux/build-appimage.sh`](../../packaging/linux/build-appimage.sh) |
| `OUT_DIR` | AppImage output directory; default `$BUILD_DIR/dist`. | Explicit environment value replaces the default. | [`packaging/linux/build-appimage.sh`](../../packaging/linux/build-appimage.sh) |
| `TOOLS_DIR` | AppImage helper-tool directory; default `$BUILD_DIR/tools`. | Explicit environment value replaces the default. | [`packaging/linux/build-appimage.sh`](../../packaging/linux/build-appimage.sh) |
| `SKIP_BUILD` | AppImage packaging switch; exact `1` reuses the existing build directory instead of configuring/building. | Defaults to `0`; only `1` skips the build. | [`packaging/linux/build-appimage.sh`](../../packaging/linux/build-appimage.sh) |

## Reader coverage and exclusions

The production readers checked for this inventory are [`app/main.cpp`](../../app/main.cpp)
(app argument parser and early version path), [`app/common/real_runtime_host.cpp`](../../app/common/real_runtime_host.cpp)
(`envString` / `envFlag`), [`core/runtime/src/launcher/extension_locator.cpp`](../../core/runtime/src/launcher/extension_locator.cpp)
(extension lookup), [`core/runtime/src/launcher/robot_launcher.cpp`](../../core/runtime/src/launcher/robot_launcher.cpp)
(packaged-environment inputs and child environment construction), [`core/runtime/src/launcher/init_script.cpp`](../../core/runtime/src/launcher/init_script.cpp)
(launcher hand-off), [`core/runtime/src/launcher/project_inspector.cpp`](../../core/runtime/src/launcher/project_inspector.cpp)
(WPILib install-root discovery), [`app/host/robot_session.cpp`](../../app/host/robot_session.cpp)
(`JAVA_HOME` and launcher options), [`ipc/src/endpoint.cpp`](../../ipc/src/endpoint.cpp)
(IPC endpoint lookup), [`ipc/src/platform/unix_socket_posix.cpp`](../../ipc/src/platform/unix_socket_posix.cpp)
(POSIX socket base-directory selection), [`halsim_ext/src/config.cpp`](../../halsim_ext/src/config.cpp)
(extension environment parsing), and [`halsim_ext/src/extension.cpp`](../../halsim_ext/src/extension.cpp)
(bridge filtering). The environment helper is
[`sdk/include/wpiemu/sdk/core/environment.h`](../../sdk/include/wpiemu/sdk/core/environment.h),
which uses `std::getenv` (or the Windows environment API).

Qt's environment readers are [`app/common/system_probe.cpp`](../../app/common/system_probe.cpp)
(`QtSystemProbe::env`, including `SystemRoot`) and
[`app/common/toolchain_provisioner.cpp`](../../app/common/toolchain_provisioner.cpp)
(`qEnvironmentVariable("SystemRoot")`). POSIX IPC also reads `XDG_RUNTIME_DIR` and `TMPDIR`
in [`ipc/src/platform/unix_socket_posix.cpp`](../../ipc/src/platform/unix_socket_posix.cpp)
to choose a socket directory. WPILib install discovery reads `PUBLIC` and `USERPROFILE` on
Windows, or `HOME` elsewhere, in [`core/runtime/src/launcher/project_inspector.cpp`](../../core/runtime/src/launcher/project_inspector.cpp).
These OS/platform paths, `PATH`, and Qt plugin paths are not
emulator switches; they are used for discovery, temporary socket placement, or packaged-child
environment cleanup. Test harness environment reads include the Qt test entry points above,
[`app/tests/test_update.cpp`](../../app/tests/test_update.cpp),
[`app/tests/test_toolchain_provisioner.cpp`](../../app/tests/test_toolchain_provisioner.cpp),
[`app/tests/test_real_host.cpp`](../../app/tests/test_real_host.cpp), and scoped test helpers
such as [`tests/support/scoped_env.h`](../../tests/support/scoped_env.h); they are not production
inputs. A stale missing-extension diagnostic is not evidence of a supported input; the
extension locator names above are the actual environment overrides.

Next: [read the runtime and configuration contract](runtime.md).
