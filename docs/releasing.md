# Releasing

Read this when producing, checking or publishing distributable packages.
Prerequisites: the toolchain in [Building](BUILDING.md), a Release build for CPack, the platform's
packaging tools below, and a real WPILib-enabled HALSim extension. Camera publication also needs
ntcore support. This page owns packaging commands; general configure/build/test instructions
remain in Building. Run examples from the repository root unless stated otherwise.

Wiki publishing is a separate default-branch documentation workflow, not part of artifact release.
Its administrator setup and recovery procedure belong in
[documentation maintenance](guides/documentation.md).

## Artifacts and layout

[Install/CPack rules](../cmake/WpiEmuInstall.cmake) derive names from the root project version, which the
[release workflow](#release-automation-and-publication) sets to the version you release:

- Windows x64: `wpi-emulator-<version>-windows-x64.exe` (NSIS) and `.zip` (portable).
- macOS universal: `wpi-emulator-<version>-macos-arm64-x64.dmg`.
- Linux x86_64: `wpi-emulator-<version>-linux-x86_64.AppImage`.

Each distributable has a SHA256 sidecar. Linux CPack also offers a TGZ of the install tree;
that archive is not a self-contained Qt deployment.

Keep each package's relative layout intact:

- Windows: `bin\wpiemu_app.exe` and Qt/runtime DLLs, `extensions\halsim_wpiemu.dll`,
  Qt plugins/QML beside those directories, and `THIRD-PARTY-NOTICES.txt` at the install root.
- macOS: `wpiemu_app.app/Contents/MacOS/wpiemu_app`, the extension at
  `wpiemu_app.app/Contents/PlugIns/wpiemu/libhalsim_wpiemu.dylib`, plus bundled frameworks,
  Qt plugins and QML resources. The notice is installed beside the app bundle.
- Linux AppImage: `usr/bin/wpiemu_app`, `usr/lib/wpiemu/libhalsim_wpiemu.so`,
  bundled Qt dependencies and desktop integration; notices under `usr/share/doc/wpi-emulator`.

The app resolves the extension relative to its executable, not a registry entry.
`WPIEMU_EXTENSION` (file) and `WPIEMU_EXTENSION_DIR` (directory) are checked first;
existing packaged/build-tree candidates follow, with an optional development fallback last.
The [locator](../core/runtime/src/launcher/extension_locator.cpp) is the lookup contract.
Do not require an original build directory for a shipped package.

WPILib runtime libraries are normally supplied by the robot process, not redistributed.
Keep app data outside the install tree; see [project files](USER_GUIDE.md#settings-and-project-files).
The launcher must still leave robot source, build configuration and vendordeps unchanged.

## Windows packages

Install NSIS and make `makensis` discoverable **before configuring**; otherwise CPack selects
ZIP only. Reconfigure after adding NSIS. The release workflow installs it with Chocolatey.
Qt deployment defaults on and includes the app's QML imports; the VC++ runtime is installed
beside both the app and the extension, which is loaded by a different process.

After the normal Windows Release build:

```powershell
Push-Location build\windows-vs2022
cpack -C Release -B pkg
Pop-Location
python tools\package_checksums.py build\windows-vs2022\pkg
python tools\package_checksums.py build\windows-vs2022\pkg --verify
```

The installer is per-machine, defaults to `C:\Program Files\WPI Emulator`, creates shortcuts
and does not modify PATH. The portable ZIP uses the same layout; start `bin\wpiemu_app.exe`,
not an executable copied out of that tree.
For a staging-only check instead of an installer:

```powershell
cmake --install build\windows-vs2022 --config Release --prefix "$PWD\build\stage-windows"
& "$PWD\build\stage-windows\bin\wpiemu_app.exe" --version
```

Repeat the smoke check with a system-only PATH so a developer Qt installation cannot hide
missing runtime files. Check the installer with silent install/uninstall as in the
[release workflow](../.github/workflows/release.yml), then launch interactively and open a robot project.
Authenticode signing is not integrated; unsigned packages must not be represented as signed.
Any external signing process must finish before final checksum generation.

## macOS packages and signing

Use the macOS build from Building. For a universal package, configure it with
`-DCMAKE_OSX_ARCHITECTURES="arm64;x86_64"` and a universal Qt kit. The deployment minimum
defaults to macOS 13.0; `CMAKE_OSX_DEPLOYMENT_TARGET` also determines the bundle's declared minimum.
Set `WPIEMU_MACOS_BUNDLE_ID` to an identifier you own (default `org.wpiemulator.app`).

```sh
(cd build/macos && cpack -C Release -G DragNDrop -B pkg)
for dmg in build/macos/pkg/*.dmg; do bash packaging/macos/notarize.sh "$dmg"; done
python tools/package_checksums.py build/macos/pkg --verify
```

CPack uses Qt's deployment API to stage frameworks, plugins and QML inside the bundle.
Its [pre-build signing hook](../packaging/macos/sign_bundle.cmake) signs libraries,
frameworks and then the app with hardened-runtime entitlements when the environment provides
`WPIEMU_CODESIGN_IDENTITY`; `WPIEMU_CODESIGN_KEYCHAIN` optionally selects the keychain.
Without an identity it leaves the deployment tool's ad-hoc signature, not Developer ID signing.

[notarize.sh](../packaging/macos/notarize.sh) signs the DMG when an identity is set, then
submits, staples and assesses it with either credential set:

- `APPLE_ID`, `APPLE_TEAM_ID`, `APPLE_APP_PASSWORD`; or
- `APPLE_API_KEY_PATH`, `APPLE_API_KEY_ID`, `APPLE_API_ISSUER`.

Notarization requires the signing identity. With no notary credentials the script explicitly
skips notarization and exits successfully; that is **not** proof of notarization.
Signing-only and stapled paths refresh their digest after the final mutation.
Keep certificates, private keys and passwords out of the repository.

The workflow checks app/extension architecture slices, runs `--version` on the host and,
when Rosetta is available, the other slice, and checks dependency paths.
[check_minimum_os.py](../packaging/macos/check_minimum_os.py) compares Mach-O minima with the plist;
this checks metadata, not execution on the minimum OS or loading the extension in a robot process.

## Linux AppImage

Build on Linux x86_64 with the Qt kit and system dependencies listed in the release workflow.
The kit needs the `qtquick3d`, `qtshadertools` and `qtmultimedia` modules; the Windows and macOS workflow
steps check that the Multimedia backend plugin was deployed.
The release runner is Ubuntu 22.04; choose the oldest compatible build environment for your users.
Bundling does not remove glibc/libstdc++ requirements or provide graphics drivers.

```sh
QT_ROOT_DIR=$HOME/Qt/6.8.3/gcc_64 bash packaging/linux/build-appimage.sh
python tools/package_checksums.py build/appimage/dist --verify
```

The [script](../packaging/linux/build-appimage.sh) builds Release, stages
`build/appimage/AppDir/usr`, bundles dependencies with pinned, SHA256-checked linuxdeploy tools,
and writes `build/appimage/dist/*.AppImage`. Its checks run `--version` with a scrubbed
environment and confirm the extension exists inside the image. Tools run without requiring FUSE.
`SKIP_BUILD=1` reuses an existing `BUILD_DIR`; `OUT_DIR` changes the artifact destination.
These are script environment settings, not CMake options.
`WPIEMU_WPILIB_DOWNLOAD` defaults ON in this script; `WPIEMU_FETCH_SDL2` controls SDL2 fetching
(distinct from CMake's gamepad enable switch documented in Building).

The [AppRun hook](../packaging/linux/apprun-hooks/wpiemu.sh) sets `WPIEMU_PACKAGED=appimage`
and the extension directory. The robot launcher strips AppDir-specific Qt/library environment
entries before running Gradle or robot code, so bundled Qt does not contaminate their environment.
Do not remove that hook or require the robot to use the AppImage's libraries.
For the Docker-based packaging workflow from Windows/macOS, follow Building.

## Packaging settings

`WPIEMU_DEPLOY_QT` defaults ON and deploys Qt during installation on Windows/macOS;
Linux AppImage deployment belongs to linuxdeploy.
`WPIEMU_PACKAGE_WPILIB_RUNTIME` defaults OFF; enabling it installs `wpiHal`/`wpiutil`
beside the extension. It is not a substitute for testing loader behavior against the robot's runtime.
General dependency, SDL2 and build switches belong to [Building](BUILDING.md).

## Version pins

[versions.json](../versions.json) is the only home of the WPILib, Qt, CMake, Python, Ubuntu and macOS pins:
`qt.release` is the exact kit for releases and the Docker image, `qt.ci` the pattern CI installs.
CMake reads `wpilib` (the default of `WPIEMU_WPILIB_VERSION`) and `macosDeployment`. The workflows, Docker image,
AppImage script and docs cannot read it, so `tools\check_versions.py` (CTest `versions.check`) reports every copy that
disagrees, with file and line. To change a pin, edit the JSON and fix what the checker lists; an existing build tree
keeps its cached `WPIEMU_WPILIB_VERSION` until you pass the new value. The linuxdeploy versions and checksums live
only in the [AppImage script](../packaging/linux/build-appimage.sh).

## In-app updates

The app's updater (behaviour in the [app reference](reference/app.md#updates)) depends on conventions this process must keep:

- A published, non-draft, non-prerelease release is what `https://github.com/Overture-7421/wpi-emulator/releases/latest`
  redirects to, and its tag is `v<major>.<minor>.<patch>`. Other tags are ignored. Publish the draft only after every artifact
  is attached, because the app reads the release the moment it is public.
- Installer names are exactly those under Artifacts and layout (`windows-x64.exe`, `macos-arm64-x64.dmg`,
  `linux-x86_64.AppImage`), each with `<name>.sha256` in `sha256sum` format. Renaming an artifact or dropping a sidecar
  silently stops updates for that platform; the app treats a missing sidecar as "no update".
- The checksum is the only integrity check, and installers are not signed, so a compromised release is not detectable by the app.
- Each package must carry Qt's `tls` plugin folder (the HTTPS backends); the release workflow and AppImage
  script check it. A Linux host also needs OpenSSL 3. The NSIS finish page offers to start the new version.

## Release automation and publication

[release.yml](../.github/workflows/release.yml) runs only by manual dispatch. To release:

1. Merge what should ship to `main` and let CI pass.
2. Actions > **Release** > **Run workflow**, branch `main`, and type the version in `version`, for example `v0.0.8`
   (a suffix such as `-rc1` marks a prerelease).
3. Wait for the Windows/Linux/macOS matrix, which builds, runs CTest, packages, checks the artifacts and uploads them.
4. Inspect the artifacts and the **draft** release the run created, then publish it. Publishing creates the tag
   `vX.Y.Z` on the commit the run started from.

[release_version.py](../tools/release_version.py) checks the input first: `vMAJOR.MINOR.PATCH` with an optional
suffix, not `0.0.0`, not an existing tag and not lower than the latest tag. Nothing is pushed to `main` (its ruleset
blocks the workflow token): the committed `project(VERSION 0.0.0)` of `CMakeLists.txt` is a placeholder, the app's
development-build marker that never checks for updates, and each build job rewrites it in its own checkout. Installers
and the app therefore carry the typed version, and a job fails unless every installer is named for it.
A plain local `cpack` produces `wpi-emulator-0.0.0-...`. For a local build with a specific number, for example an older
one to try the updater, run `python tools\release_version.py apply 0.0.7`, build, then `git checkout -- CMakeLists.txt`.

Only a run on `main` drafts a release, and it stops if a release for that tag exists (delete a stale draft and run
again). A run from any other branch uploads the artifacts and drafts nothing, which is how to test workflow changes.

The workflow rejects a stub extension if HAL discovery/download failed, unless a manual run
explicitly sets `allow_stub_extension`. Never publish that diagnostic override as a usable robot simulator.
The optional Apple workflow secrets are `MACOS_CERTIFICATE` (base64 Developer ID certificate),
`MACOS_CERTIFICATE_PASSWORD`, `MACOS_SIGN_IDENTITY`, and the three Apple ID credentials above.
They are imported on the runner; without them no Developer ID/notarized release is established.

[ci.yml](../.github/workflows/ci.yml) separately configures platform builds, tests and install smoke checks
for main-branch pushes, pull requests and manual dispatch. Configured automation is not a claim
that a native Linux/macOS robot session or interactive 3D check was performed.
CTest opt-in e2e skips are not successful robot-project validation; follow Building to run them explicitly.

After signing/stapling and any other mutation, run [package_checksums.py](../tools/package_checksums.py)
on the artifact directory, then run it again with `--verify`. The workflow does this immediately
before upload. Before publishing the draft, inspect artifacts, run a target-platform robot session,
and check native rendering; `--version` and offscreen suites cannot prove either.

Review [THIRD-PARTY-NOTICES.txt](../packaging/THIRD-PARTY-NOTICES.txt) and required license texts
for every bundled dependency. A root `LICENSE` is currently absent; when present CPack uses it
for NSIS's license page. Linux AppStream metadata also has no product license declaration.
Package generation is not license clearance. Keep the [sample WPILib notice](../samples/WPILib-License.md)
and all upstream attributions intact.

Next: [User guide](USER_GUIDE.md) for installation and controls, or [Building](BUILDING.md) for validation setup.
