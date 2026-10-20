# ADR 0019 - Core sensors fed by plugin game values

- Status: accepted
- Scope: `RobotConfig::sensors`, `IGameModule::contributeGameValues`, `IGameState::gameValue`,
  `SensorSystem`, `IDeviceHub::writeDistance`, editor

## Context

ADR 0018 added a season-specific `game_2026.fuel_sensor` mechanism that downcast to the plugin's
private game state. Sensors are not season-specific; only the quantity they observe is.

## Decision

- A first-class, core-owned `SensorConfig` (`id`, `label`, `kind`, `source`, `properties`, `mappings`)
  lives in `RobotConfig::sensors`. Kinds: `distance` (CANrange-style, metres) and `beam_break`
  (boolean). Sensors have no pose or link.
- A plugin publishes named **game values** (`GameValueInfo`, ids prefixed `<pluginId>.`) with
  `contributeGameValues()` and answers `IGameState::gameValue(id)` (`nullopt` when unknown). Core
  never inspects plugin state. "Game value" is distinct from `sdk::signal::*` device signals and
  from rate-limited HUD keys.
- A sensor's `source` must be a value owned by the selected plugin. Validation codes:
  `sensor_id_invalid`, `sensor_id_duplicate`, `sensor_source_missing` (W), `sensor_source_not_owned`,
  `sensor_source_unknown` (W), `sensor_property_invalid`.
- `SensorSystem` runs after the game's tick and writes through the hub (`writeDistance`,
  `writeDigital`), so sensors see the same tick's state. A source that is unavailable at runtime
  writes the idle reading and raises `sensor_source_unavailable`.
- `writeAnalog` is volts only; distance uses `writeDistance`.
- The editor edits sensors only; the run view lists them (reading, source value) in the Robot state window, fed by
  `SensorState` in the snapshot (`ISimSnapshot::sensors()`), and draws no viewport marker.
- No migration (ADR 0013): `game_2026.fuel_sensor` is removed and saved projects were rewritten.

## Consequences

New seasons add sensor inputs by publishing game values, with no core, UI or bridge changes.
Sensors cannot yet be driven by physical geometry or non-game sources.
