# ADR 0004: Modular boundaries

Status: accepted

## Context
The emulator must support new game years without touching the core or UI, and device/vendor
support without touching physics. Modularity has to be enforced mechanically, not by convention.

## Decision
Dependency rules (arrow = "may depend on"):

| Module | May depend on |
|---|---|
| `sdk` | nothing (header-only, no Qt/Box3D/WPILib) |
| `core` | `sdk`, Box3D (private) |
| `ipc` | `sdk` |
| `halsim_ext` | `sdk`, `ipc`, bridge aggregator (+ private HAL backend) |
| `bridges/hal`, `bridges/vendor` | `sdk` (not each other) |
| `bridges/` aggregator | `sdk`, bridge modules |
| `plugins/` aggregator | `sdk`, plugins |
| `app` | `sdk`, `core`, `ipc`, plugin aggregator, Qt |
| `plugins/<name>` | `sdk` only (not core, app, ipc, bridges, or other plugins) |

Nothing outside `plugins/` includes or links a plugin target (`wpiemu_plugin_*`). Plugins are
wired to the app by an aggregation seam (the `plugins/`-owned registration target introduced
with the `IGameModule` / `GamePluginRegistry` interfaces in the SDK freeze).

Conventions that the checker relies on:
- Public headers live at `<module>/include/wpiemu/<module>/...`
  (`wpiemu/bridges/<hal|vendor>/...`, `wpiemu/plugins/<name>/...`).
- Targets are named `wpiemu_<module>`, `wpiemu_bridges_<hal|vendor>`, `wpiemu_plugin_<name>`
  (`plugins/_template` -> `wpiemu_plugin_template`), plus `halsim_wpiemu`.

## Enforcement
`tools/check_boundaries.py` scans `#include` directives and `target_link_libraries` /
`target_include_directories` calls. It is registered as CTest `boundaries.check` (with
`boundaries.self_test` covering the checker itself) and runs in CI. The allowed-dependency table
is at the top of the script; changing a rule requires amending this ADR.

## Addendum (ADR 0005)
The "aggregation seam" is the target `wpiemu_plugins_builtin` (files directly in `plugins/`).
It may reference sdk and every plugin; `app` may link it but never include it; no other module
(and no plugin) may reference it. The checker additionally rejects identifiers that collide with
Qt keyword macros (`signals`, `slots`, `emit`, `foreach`) in non-app headers.

## Bridge aggregation and implementation targets
Files directly in `bridges/` and `wpiemu_bridges_builtin` form a distinct aggregation owner.
They may depend on SDK and the bridge modules; only `halsim_ext` may link the seam, and no other
owner may include its implementation files. The extension does not directly depend on individual
bridge modules. Bridge modules remain independent of each other. The HAL binding subtarget
`wpiemu_bridges_hal_halsim` belongs to `bridges/hal`; the aggregator selects it when available.

Implementation subtargets (for example `wpiemu_core_physics`, `wpiemu_app_common`,
`wpiemu_halsim_core`) retain their module's owner and rules, not a separate unrestricted edge.
The checker validates first-party declarations/aliases against its target map and rejects
unknown first-party dependencies. Imported `wpiemu_wpilib_hal` / `wpiemu_wpilib_wpiutil`
are explicitly classified third-party robot-process backends usable by the extension;
they do not expose a new SDK dependency. Real bridge bindings may use HAL/vendor headers
and backend libraries privately to implement their SDK seams.
