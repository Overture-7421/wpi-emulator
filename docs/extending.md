# Extending the emulator

Read this when adding a game, device adapter or configurable simulation feature.
Prerequisites: [Architecture](ARCHITECTURE.md), [a working build](BUILDING.md) and
[contribution conventions](CONTRIBUTING.md).

Choose the smallest existing SDK seam. The recipes below are the implementation checklists;
the linked reference pages own detailed behavior. Do not add season/vendor switches to core or UI.

## Add a season plugin

1. Copy [the template](../plugins/_template/README.md) into a new `plugins/game_<year>` folder.
   Rename the `_template` include directory/strings and namespace to
   `wpiemu/plugins/game_<year>` and `wpiemu::plugins::game_<year>`. Rename the CMake target/alias
   following `wpiemu_plugin_game_<year>` / `wpiemu::plugin_game_<year>`, and update `pluginId()` /
   `gameYear()` and metadata to the same identity and positive season year.
2. Implement [IGameModule](../sdk/include/wpiemu/sdk/game/game_module.h): `metadata`,
   `registerMechanisms`, `createField`, `createGameState` and `contributeConfigSchema`.
   Keep the module stateless between sessions; the returned `IField` owns its physics objects and
   `IGameState` owns session rules/entities. Implement reset and snapshot publication there.
3. Prefer generic mechanisms plus zones/game values over duplicating motor physics. Custom template
   keys, zone kinds and game-value IDs start with the plugin ID plus `.`; plugin config belongs only
   to its own section. Follow the [game-plugin contract](reference/game-plugin.md).
4. Return Qt-free editor/HUD descriptors when needed; publish every HUD key they bind to.
   Supply `aprilTagLayout()` (tags plus `fieldLengthM`/`fieldWidthM`, which Limelight red and
   center poses need) for simulated tag visibility and `fieldFrame()` for alliance-copy start
   positions. Empty/default implementations intentionally provide neither feature.
5. Place `WPIEMU_REGISTER_GAME_PLUGIN(YourModule)` **inside the plugin's namespace**. It defines
   `registerPlugin`; declare that function in the plugin's public header.
6. In [plugins/CMakeLists.txt](../plugins/CMakeLists.txt), add the subdirectory and link its target
   into `wpiemu_plugins_builtin`. Include the public header and call its namespaced `registerPlugin`
   in [the aggregator](../plugins/builtin_plugins.cpp), preserving registration errors.
   The target links only `wpiemu_sdk`; no core/app or another plugin changes for season logic.
   Conventionally named plugin targets/aliases are recognized by the boundary checker, which also fails a plugin
   directory that is not added, linked and registered here.
7. Add tests alongside the new plugin's source-mirroring test directory. If they include plugin
   headers, link the test executable to the new plugin in [test wiring](../tests/CMakeLists.txt).
   Exercise registration, defaults/ownership, field cleanup, rules, reset and HUD publication.
   Adapt [the multi-season integration pattern](../tests/plugins/test_new_year_plugin.cpp) to prove
   the new module works with the config pipeline and simulation session.
8. Document sourced dimensions/rules and deliberate approximations in the
   [game-plugin reference](reference/game-plugin.md). Follow the validation checklist below.

Plugins are built in with the host toolchain, not independently loaded shared libraries.

## Add a zone kind

1. Return a [ZoneKindInfo](../sdk/include/wpiemu/sdk/game/zone_kind.h) from
   `IGameModule::contributeZoneKinds()`: plugin-prefixed `kind`, label/description, `Volume` or
   `Muzzle` shape, property schema, default pose/size and gizmo color.
2. Consume matching `RobotConfig::zones` in the plugin. Resolve attachment pose using
   `IMechanismDirectory::linkPoseInChassis` and compose the zone's `local` pose; handle a missing
   parent/link safely. Use `evaluateActivation(zone.activation, ctx.mechanisms())` for gating.
3. Implement the pickup/launch/other game action in game state, not the zone metadata or UI.
   The editor generates its menu, inspector and gizmo from the kind description.
4. Test attachment motion, activation and the game's boundary cases with SDK physics/mechanism
   fakes; use real-physics coverage when geometry matters. See the
   [game-plugin contract](reference/game-plugin.md) for ownership and evaluation details.

## Add a game-fed sensor

1. Expose a plugin-prefixed [GameValueInfo](../sdk/include/wpiemu/sdk/game/game_value.h) through
   `IGameModule::contributeGameValues()`, with a useful label, description and unit.
2. Implement `IGameState::gameValue(id)` for that ID using live game state, returning no value when
   unavailable. Do not derive a sensor source from presentation-only HUD data.
3. Configure [SensorConfig](../sdk/include/wpiemu/sdk/config/sensor.h) with a Distance or BeamBreak
   kind, that `source`, response properties and its `sensor` slot mapping. It has no pose.
   Core's sensor system handles device feedback; the plugin need not write device signals.
4. Test the value as game state changes, the configured response, unavailable-source idle behavior
   and reset. Use the [runtime sensor contract](reference/runtime.md) for response/validation rules.
   A genuinely new sensor kind also needs SDK response/slot metadata, codec/validation and editor
   choices updated together; adding a source for an existing kind does not.

## Add a device bridge

1. For a new family, create a module under `bridges/` with public headers under its namespaced
   include directory. Keep third-party backend headers inside it and follow the
   [dependency rule](ARCHITECTURE.md#modules-and-dependencies).
   Declare its library and implementation sources in CMake, export its public include root,
   and link SDK plus permitted private backend dependencies.
2. Choose [SimDeviceBridge](../sdk/include/wpiemu/sdk/devices/sim_device_bridge.h) plus a
   `SimDeviceRule` table, [ChannelFamilyBridge](../sdk/include/wpiemu/sdk/devices/channel_family_bridge.h)
   for indexed HAL families (the HAL module's `HalFamilyBridge`/`IndexedHalBridge` bases supply the seam and
   channel plumbing), or implement [IDeviceBridge](../sdk/include/wpiemu/sdk/devices/device_bridge.h).
   Announce descriptors/capabilities, collect outputs, apply inputs, support unmapped loopback,
   retire freed devices and make shutdown idempotent.
3. Bind [canonical signals](../sdk/include/wpiemu/sdk/devices/signals.h) with explicit units,
   directions and conversions. Preserve [identity/alias and neutral-feedback rules](reference/bridges.md):
   a CAN-vendor rule with no observed bus uses `busIsStatic = false`, not an invented bus label.
4. Expose a module registration function that registers factories with
   `DeviceBridgeRegistry::add`. Add its subdirectory and link it into
   `wpiemu_bridges_builtin` in [bridge build wiring](../bridges/CMakeLists.txt); call the function
   from [the bridge aggregator](../bridges/builtin_bridges.cpp). The extension does not change, and the boundary
   checker fails a module that is not added, linked and called there.
5. For a new bridge module, update `ALLOWED`, `MODULE_OWNERS` and `TARGET_OWNERS` in
   [the boundary checker](../tools/check_boundaries.py), including the aggregator's outgoing edge
   and any aliases/binding targets. Do not allow a direct extension-to-module dependency.
6. Test discovery/update/retirement, capabilities, direction/scale, missing values, loopback and
   input-history reset through fake backends. Link new-module tests explicitly to that module
   in [test wiring](../tests/CMakeLists.txt), or register a dedicated test target (`wpiemu_doctest_target(<target>)`
   applies the doctest and warning settings): source discovery
   does not extend the fixed test-library list. Add the actual contract to the [bridge reference](reference/bridges.md).

## Add a vendor rule set

1. Enumerate the vendor wrapper's real SimDevices and values, or read its source. Record exact
   names, types, directions and units; do not guess them from a hardware API.
2. Add a rule table in the existing vendor module: full-match regex, ID/child groups, family/type,
   vendor, capabilities and canonical bindings with scale/offset. Reuse the device-bridge recipe
   if a separate module is actually needed.
3. Reuse `VendorSimDeviceBridge` and its backend decorators. Add `GuardedValueSpec` entries only
   for firmware-integrated inputs whose semantics require them; do not fork the guard.
   Do not enable CTRE-native read-back/hooks for another vendor; choose only verified policies.
4. Add construction to `SharedSetup` and factory registration to
   [vendor_bridges.cpp](../bridges/vendor/src/vendor_bridges.cpp), propagating checked-registration
   failures. The extension continues to use the existing aggregator.
5. Use [vendor fakes](../tests/bridges_vendor/fakes.h) and wrapper-faithful fixtures to test grouping,
   late/missing values, unit conversions, retirement, bus identity and any integrated-input behavior.
   Preserve the [bridge contract](reference/bridges.md), including quiescent config read-back,
   calibration/offset ownership and firmware timing limitations. Document the supported signal set.

## Add a vision vendor

1. Put the encoder and bridge in `bridges/vision`, or create a bridge module using the device-bridge
   wiring recipe. Keep `CameraFrame` encoding pure and isolate publication behind the existing
   [INtBackend seam](../bridges/vision/include/wpiemu/bridges/vision/nt_backend.h).
   Add new implementation files to `wpiemu_bridges_vision` in its
   [explicit source list](../bridges/vision/CMakeLists.txt); they are not auto-discovered.
2. Implement [IVisionBridge](../sdk/include/wpiemu/sdk/vision/vision_bridge.h) with a unique name.
   Start the backend, configure only cameras of your kind idempotently, poll robot-written settings,
   publish the producer's due-only frames, reset stale published state and stop idempotently.
   Keep the cameras in a [CameraSet](../sdk/include/wpiemu/sdk/vision/camera_bridge.h) of your `CameraState`.
3. Register factories through `VisionBridgeRegistry::add`. Wire them through
   `registerBuiltinVisionBridges` in the aggregator, not extension internals.
   The existing real-ntcore entry point is `registerNtVisionBridges`; retain its real/fake-backend
   selection when adding to the built-in vision module.
4. If needed, add a `CameraKind` and its key in
   [camera metadata](../sdk/include/wpiemu/sdk/config/camera.h), plus codec, validator and editor
   camera-menu handling. Keep the current schema policy; do not add migration code.
5. Test golden payloads, foreign-kind filtering and lifecycle reset with fake publishers.
   Test producer scheduling separately: no frame is supplied before `publishTimeUs`.
   Document topic layouts and timestamp requirements in the [bridge reference](reference/bridges.md).

## Add a mechanism template

1. First check whether the configurable generic mechanism already models the part; see the
   [mechanism contract](reference/mechanisms.md). A new name for an arm/roller is not a new template.
2. Implement [IMechanism](../sdk/include/wpiemu/sdk/mechanisms/mechanism.h), constructed with
   `MechanismBuildContext`. Read commands/write feedback only through mapped slots, advance local
   dynamics through `IPlant`, and use `ctx.coupling()` for chassis loads/constraints/payloads.
3. Describe it with `MechanismTemplateInfo`: type, display/category, origin, `slotSpecs`,
   `PropertySchema`, default geometry and optional links/motion axes.
   Plugin-owned template types and `origin` use the plugin's identity.
4. Register with `MechanismFactory::registerTemplate`: generic templates through
   [registerGenericMechanisms](../core/mechanisms/include/wpiemu/core/mechanisms/mechanism_templates.h),
   season-specific ones through the plugin's `registerMechanisms`. Metadata generates the menu,
   inspector and slot panel; do not add concrete-template switches to UI.
5. Publish moving link poses and game-facing values in `MechanismState`; report battery draw
   through `batteryCurrentA()`.
   Declare additional links without redeclaring implicit `root`. If supporting motion links,
   advertise/bind axes and honor prescribed motion rather than integrating a follower twice.
6. With `FakeSimEnvironment` and `FakeDeviceHub`, test commands/feedback, reset, unmapped neutral
   behavior and any moving link or coupling. Follow the
   [physics coupling contract](reference/physics.md) when testing load/constraint timing.

## Add an editor property

1. For template, zone or plugin parameters, extend the owner's
   [PropertySchema](../sdk/include/wpiemu/sdk/config/property_schema.h). Set a stable key, type,
   label/description, default, unit, ranges/options and grouping. Use `section`/`axis` for Drive
   placement and visibility metadata for dependent controls. Generic forms need no new widget.
   Build entries with the [`props::` builders](../sdk/include/wpiemu/sdk/config/property_builders.h)
   and read stored values with `getDouble/getBool/getString/getFiniteDouble` from the SDK
   [`Value`](../sdk/include/wpiemu/sdk/core/value.h); do not write a per-module builder or reader.
   A plugin's typed config section binds each spec to its member once with
   [`ConfigBindings`](../sdk/include/wpiemu/sdk/config/config_bindings.h), which supplies schema, decode and encode.
2. Implement consumption and validation in the owning module. Test defaults, an invalid value and
   persistence round-trip; use Qt-free editor/HUD descriptors for plugin presentation.
3. For a new structural robot field, update the SDK model, codec/defaults and validator together.
   Add its schema/value conversion in the editor model and route edits through
   `EditorDocument`/`EditorController` undoable config operations, never widget-side mutation.
4. Test edit/undo/redo, save/load and unit conversion where relevant. If placement or picking changes,
   add model/transform tests and check the demo visually. Follow [UI conventions](CONTRIBUTING.md#coding-conventions)
   and the [app contract](reference/app.md); keep runtime/persistence access behind the backend.

## Other SDK strategies

- **Drivetrain:** implement `IDrivetrain`, register with `DrivetrainRegistry::registerDrivetrain`
  and include it in [built-in registration](../core/physics/include/wpiemu/core/physics/drivetrains.h).
  Declare its slots once: `DrivetrainInfo::slotSpecs` is a pure function of the spec, shared by the validator,
  the editor and `IDrivetrain::slotSpecs()` (use `tractionSlotSpecs`, `isSteered` and the `steerSlotName`
  helpers). Never step the world from a drivetrain.
- **Tires:** implement stateless `ITireModel::solve`, register with `TireModelRegistry::registerModel`
  and the [tire registration](../core/physics/include/wpiemu/core/physics/tire_models.h).
  Check affected config selectors/editor choices when introducing a new strategy key.
- **Plants/motors:** register an `IPlant` with `PlantFactory::registerPlant` and supply the factory to
  the consuming template. `IMotorModel` is the motor strategy seam; sourced motor data belongs in
  the core motor catalog, not a season plugin's copy. See [mechanisms](reference/mechanisms.md).
- **Physics backend:** implement `IPhysicsWorld` behind its SDK contract and inject it at composition,
  keeping backend headers private. Test contacts/sensors, handle lifetimes and deterministic replay;
  see [physics](reference/physics.md).
- **Transport/codec:** implement `ITransport` / `IMessageCodec`, wire the composition roots and test
  both peers with round-trip/failure cases. Preserve the [lock-step contract](reference/extension-ipc.md).
- **Message field:** add the member to its struct in the headers [messages.h](../sdk/include/wpiemu/sdk/ipc/messages.h)
  lists and one line to that struct's `fields` function in the [binary codec](../ipc/src/binary_codec.cpp), which
  encodes and decodes. Put it after the `if (!c.more()) return;` guard so older peers still decode; extend the golden bytes.

## Validate the extension

- Exercise construction, normal use, reset and teardown through the public seam; include a focused
  failure case. Keep registration errors visible and metadata defaults valid.
- Use the [build/test/check commands](BUILDING.md); check boundary ownership and doc links after
  adding files. Use a real project or visual demo only when the changed behavior requires it.
- Update the owning reference contract and [user guide](USER_GUIDE.md) for observable changes.
  Record a new design decision only when the SDK/boundary rationale changes, through the
  [decision index](decisions/README.md).

Next: [understand configuration and runtime construction](reference/runtime.md).
