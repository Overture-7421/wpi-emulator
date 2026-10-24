# 0018: Season packs describe WPILib support as data

**Status:** Accepted

## Context

[0017](0017-multi-season-wpilib.md) builds one HALSim extension per WPILib season. Season
facts (pins, install folders, Java major, launch tasks, vendor schemas) were spread over
`versions.json`, `#if` year checks, launcher branches and CI lists, so a new year or a
mid-year update meant editing many files, and a new year silently inherited the newest branch.

## Decision

A **season pack** is `seasons/<year>/season.json` plus an optional `game/` plugin. The
manifest declares the WPILib pin and verified versions, install folder, Java major, HAL ABI,
launch profile, vendor schemas and samples. Game rules stay SDK-only; the pack only names
the plugin.

HAL differences follow the **HAL ABI** (`seasons/abi/<abi>.json`), not the year. Robot-process
code selects per-ABI adapters; a new year on an existing ABI is a manifest change. Launch
behavior is a named **launch profile** in core, chosen by the manifest.

Code, CMake, workflows and scripts outside `seasons/`, tests, samples and docs must not
contain season-year literals; `tools/check_seasons.py` enforces this.

An unverified WPILib or vendor version in a known season warns and continues. A year or
protocol mismatch is still refused: a wrong HAL ABI cannot run safely.

## Consequences

The WPILib dependency is declared by the pack but linked only into the extension; a
game plugin never links the HAL. A season may have a platform without a game plugin.
Adding a new HAL ABI remains a code change, guarded by the same checker.
