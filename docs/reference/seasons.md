# Season packs

Read this when adding a WPILib season, bumping a WPILib or vendor version, or changing how the
emulator picks an extension, launch task or schema for a project.
For the reasoning see the [architecture decisions](../decisions/README.md); for the upgrade checklist
see the [WPILib upgrade guide](../guides/wpilib-upgrade.md).

## What a pack is

A **season pack** is a folder `seasons/<year>/` with a manifest, `season.json`, and an optional
`game/` directory holding that season's [game plugin](game-plugin.md). Everything the build,
launcher, toolchain checks and vendor bridges need to know about a WPILib year is read from the
manifest. Code outside `seasons/`, tests, samples and docs does not contain season-year literals;
[`tools/check_seasons.py`](../../tools/check_seasons.py) enforces this.

`seasons/_template/` is the skeleton for a new pack. It is built to keep it compiling and is never
registered.

## Manifest

| Key | Meaning |
|---|---|
| `year` | The folder name as a number; the GradleRIO year the robot project declares. |
| `channel` | `stable` or `preview`. |
| `default` | Exactly one season is the default: the build's primary season and the fallback for a project whose year is unknown. |
| `wpilib.version` | The artifact version the extension is built against. It must be listed in `verified`. |
| `wpilib.verified` | Versions the emulator was verified with. A project on another version of a known year warns and continues. |
| `wpilib.installFolder` | The WPILib installer folder name (for example a preview suffix). Discovery also scans `<year>` and `<year>_*` folders. |
| `platform.abi` | The HAL ABI profile in `seasons/abi/<id>.json`. |
| `platform.launch` | The launch profile: `gradlerio-classic` or `gradlerio-modern`. |
| `toolchain.javaMajor` | The Java major the season builds with; selects the managed JDK download. |
| `vendors.photonvision` | The wire `schema` (`photon-us`, `photon-ns`), the `defaultVersion` and the `verified` versions. |
| `game` | `{"plugin": "game_<year>"}`, or `null` for a platform-only season that runs core-only. |
| `samples` | Sample robot projects whose GradleRIO version must equal `wpilib.version`. |

## HAL ABI profiles

`seasons/abi/<id>.json` describes what the extension needs from one generation of the HAL: the
Maven groups and header probes of the WPILib artifacts, the required C++ standard, whether ntcore
is mandatory and, when present, a `datalog` group that ntcore loads. A year that keeps the same
HAL names the same ABI; a new HAL needs a new profile and per-ABI adapters.

The robot-process modules include the per-ABI headers of [`hal_abi`](../../hal_abi) (for example
`wpiemu/hal_abi/hal_includes.h`) and never a WPILib include root. Behavior that differs between HAL
generations lives in `abi/<id>/` source folders of the module that owns it (`halsim_ext/src/hal`,
`bridges/hal/src`), selected by the season's ABI at configure time, so shared code carries no
`#if` on a year or ABI. Each ABI has its own real-HAL suites under `tests/hal/abi/<id>` and
`tests/abi/<id>`. The HAL is linked only into the robot-process extension, never into a game plugin.

## Launch profiles

Core names Gradle behavior by profile, not by year. `gradlerio-classic` runs the
`simulateJava`/`simulateNative` tasks and a fat robot jar. `gradlerio-modern` runs the project's
`run` task and `simulateExternalJava`, and launches from a classpath descriptor.
The season's manifest chooses the profile; see [Runtime](runtime.md#launcher-boundary).

## Version policy

| Situation | Behavior |
|---|---|
| Project's WPILib version is verified for its season | Silent. |
| Known year, unverified version | Warns (log and Toolchain row) and continues. |
| Unverified PhotonLib version | Warns and publishes with the season's default schema; PhotonLib's own check remains the final guard. |
| Year has no pack | Warns and tries the default season. |
| Extension year differs from the project year | Refused (`season_mismatch`); `WPIEMU_EXTENSION` overrides with a warning. |
| Protocol mismatch | Refused. |

## Build

`WPIEMU_WPILIB_SEASON` selects the primary season (default: the manifest marked `default`).
`WPIEMU_SEASONS` lists the extensions to compose; `all` expands to every pack. Each extra season is
built in an isolated extension-only sub-build; see [Building](../BUILDING.md#multi-season-builds).
The C++ standard of a build follows the primary season's ABI profile.

## Checks

`python tools\check_seasons.py` validates manifests (one default, pinned version verified, known ABI
and launch profile, existing samples whose GradleRIO version matches, the game plugin exists) and
the season-literal rule. `python tools\check_versions.py` validates the non-season pins and that each
season's Java major has a managed-JDK download.
