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
| `halsim_ext` | `sdk`, `ipc` |
| `bridges/hal`, `bridges/vendor` | `sdk` (not each other) |
| `app` | `sdk`, `core`, `ipc`, Qt |
| `plugins/<name>` | `sdk` only (not core, app, ipc, bridges, or other plugins) |

Nothing outside `plugins/` includes or links a plugin target (`wpiemu_plugin_*`). Plugins are
wired to the app by an aggregation seam (a `plugins/`-owned registration target, to be introduced
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
