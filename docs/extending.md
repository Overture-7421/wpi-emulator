# Extending the emulator

Read this when choosing the smallest existing seam for a simulator feature.

Start with the seam below, then follow its guide for a procedure and its reference for behavior.
Keep game-year rules in plugins and robot-device translation in bridges, not in core or the UI.
The [architecture](ARCHITECTURE.md) explains module ownership and runtime constraints.
For a complete worked season example, see the [Season Builder's Book](book/README.md).

## Seam map

| Change you need | Use this seam | Guide | Contract |
|---|---|---|---|
| Support a new WPILib year, patch, preview revision or vendor-library release | A season pack: `seasons/<year>/season.json` (data only on an existing HAL generation) | [WPILib upgrade](guides/wpilib-upgrade.md) | [Season packs](reference/seasons.md) |
| Add a game's field, entities, rules, or metadata | An `IGameModule` in `plugins/<name>`; register it through the plugin aggregator | [Game plugins](guides/game-plugins.md) | [Game plugins](reference/game-plugin.md) |
| Add a game zone, game-fed value, or sound cue | Plugin metadata and `IGameState`; use generic sensor responses for game values | [Game plugins](guides/game-plugins.md) | [Game plugins](reference/game-plugin.md) |
| Adapt an indexed HAL device family | `ChannelFamilyBridge` and the HAL bridge module | [Devices and bridges](guides/devices.md) | [Bridges](reference/bridges.md) |
| Adapt a vendor SimDevice | Existing vendor bridge and its verified rule/decorator seam | [Devices and bridges](guides/devices.md) | [Bridges](reference/bridges.md) |
| Add a camera vendor | `IVisionBridge` in the vision module; keep encoding separate from its backend | [Vision](guides/vision.md) | [Bridges](reference/bridges.md) |
| Add a mechanism or different local dynamics | Generic mechanism template first; add an `IPlant` or `IMechanism` only when the behavior requires it | [Mechanisms](guides/mechanisms.md) | [Mechanisms](reference/mechanisms.md) |
| Add a configurable editor field | SDK `PropertySchema` builders and typed `ConfigBindings` | [Editor properties](guides/editor-properties.md) | [Application](reference/app.md) |
| Add motor constants or a motor behavior | Shared SDK `motors::presets()` table, or inject an `IMotorModel` at its composition point | [Mechanisms: motor data and behavior](guides/mechanisms.md#motor-data-and-behavior) | [Mechanisms](reference/mechanisms.md) |
| Add drivetrain, tire, physics, or transport behavior | The corresponding SDK strategy interface and registry or composition root | [Simulation strategies](guides/simulation-strategies.md) | [Physics](reference/physics.md) or [Extension and IPC](reference/extension-ipc.md) |

## Keep the seam intact

- Plugins depend only on SDK. The app uses the plugin aggregator; it does not link a season.
- Bridges depend only on SDK and their own backend. The extension uses the bridge aggregator.
- A motor's physical constants are not a controller bridge. A configured intake is not
  automatically a new mechanism class.
- Preserve the owning reference's units, identities, lifecycle, and failure behavior. Do not infer
  a physical CAN bus from a declared address or apply motor reversal twice.
- Update the owning reference when behavior changes. Update user documentation only when users
  can observe a behavior change.

Validate construction, normal operation, failure, reset, and teardown through the public seam.
Run focused tests and the boundary/documentation checks listed in [Building](BUILDING.md).
The [codebase tour](CODEBASE_TOUR.md) follows representative flows through source and tests.

Next: [follow a code path in the codebase tour](CODEBASE_TOUR.md).
