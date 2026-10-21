# Add game interactions and sensors

Read this when a season needs a pickup volume, launch point, activation gate or game-fed sensor.
Prerequisites: [a registered season](game-years.md), [mechanisms](mechanisms.md), and the
[game-plugin contract](../reference/game-plugin.md).

A **zone** describes a game action attached to the robot; it is not another motorized mechanism.
A **game value** is a numeric reading published by game state, independent of HUD formatting.
Core can translate that reading into a distance or beam-break device input.

## Describe a zone, then implement its action

1. Return `ZoneKindInfo` from `IGameModule::contributeZoneKinds()`. Give the kind a plugin-prefixed
   key, readable description, property schema, default local pose, color and appropriate shape.
   `Volume` has a size; `Muzzle` describes a launch origin/direction.
2. Read matching `RobotConfig::zones` in game state. Resolve the attachment through
   `IMechanismDirectory::linkPoseInChassis`, then compose the zone's local pose.
   An attachment link is a spatial frame; a motion link instead prescribes an axis.
3. Evaluate `evaluateActivation(zone.activation, ctx.mechanisms())` rather than creating a
   season-specific condition interpreter. Handle missing parents and unavailable readings safely.
4. Implement collection, launch, capacity and rate limits in game state. Metadata alone does not
   make a volume collect anything.

Follow the complete working path in
[`fuel_zones.cpp`](../../plugins/game_2026/src/fuel_zones.cpp) and
[`game_state.cpp`](../../plugins/game_2026/src/game_state.cpp): pickup metadata creates the
editor description, runtime zones receive resolved poses/activation, and game state owns fuel.
Attach a muzzle to a moving turret link instead of applying the same turret angle a second time.

## Test activation before testing full gameplay

This exact excerpt is inside
`zone activation: All/Any combination, empty list and missing mechanisms` in
[`test_zone.cpp`](../../tests/sdk/test_zone.cpp), using that file's scripted-mechanism fixture:

```cpp
Fixture f;
f.flywheel.values = {{"velocity", 10.0, "rad/s"}};

ZoneActivation act;
CHECK(evaluateActivation(act, &f.dir));  // empty: always active
CHECK(evaluateActivation(act, nullptr));
```

The rest of the same case adds `Above` conditions and checks All versus Any and missing mechanisms.
Add thresholds with real mechanism units and test signed versus absolute readings.
An unpublished value can fall back to the mechanism's `active` flag; do not assume “missing value”
is a numeric zero. The [contract](../reference/game-plugin.md) owns the detailed comparison rules.

## Publish a numeric game value

1. Return a plugin-prefixed `GameValueInfo` from `contributeGameValues()` with label, description
   and unit. The existing [game value metadata](../../plugins/game_2026/src/game_values.cpp)
   demonstrates the declaration.
2. Implement `IGameState::gameValue(id)` from live state. Return no value when unavailable.
   Do not decode a display string from the HUD or cache state in the module.
3. Configure a `SensorConfig` with the existing Distance or BeamBreak kind, that source ID,
   response properties, and a mapping for its `sensor` slot. Sensors have no geometric pose:
   this is a response to a game value, not a range ray cast.
4. Let core's sensor system write device feedback. A game plugin does not need a bridge dependency.

## Choose and verify the response

The exact threshold example below is from `sensors: readings follow the response mode` in
[`test_sensors.cpp`](../../tests/runtime/test_sensors.cpp), with that file's includes/SDK namespace:

```cpp
Value threshold = defaultsOf(sensorResponseSchema(SensorKind::Distance));
threshold["threshold"] = 6.0;
threshold["nearM"] = 0.1;
threshold["farM"] = 1.0;
CHECK(distanceReadingM(threshold, 5.0) == doctest::Approx(1.0));
CHECK(distanceReadingM(threshold, 6.0) == doctest::Approx(0.1));
```

Here `distanceReadingM` is imported from `wpiemu::core::runtime` in the surrounding test.
Expected result: crossing the game-value threshold changes the device distance from far to near.
The same case exercises linear interpolation, endpoint clamping and active-low beam breaks.
Response keys and idle semantics belong in the [runtime sensor contract](../reference/runtime.md).

## Wire and validate

Zone kinds and values are contributions from the already registered plugin. No new core registry,
UI switch or bridge target is needed. Add new plugin implementation sources to its CMake list;
the plugin still links only SDK. A genuinely new sensor kind requires SDK metadata, codec,
validation and editor handling together; a new source for an existing kind does not.

```powershell
ctest --preset windows-ninja-release -R "^(zone |zones:|sensors:|pipeline: zone|validator: zone)"
```

This executes both excerpts plus config round-trip, defaults, source ownership, mapping validation
and live session feedback. Test unavailable-source idle output, reset and attachment movement.
Use real-physics coverage for geometric collection/contact boundaries, not just fake overlap lists.

If a zone appears but does nothing, inspect matching kind, resolved attachment, activation and
capacity/rate limits in that order. If a sensor stays idle, compare source ID and advertised plugin
values before debugging the bridge.

Next: [add configurable properties without custom widgets](editor-properties.md).
