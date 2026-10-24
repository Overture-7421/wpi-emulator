# Add an editor property

An editor property is metadata for a value owned by the SDK or its runtime module. Generic forms
render the schema; typed bindings decode it; undoable editor operations persist user edits. Keep
these responsibilities together rather than writing a custom widget that bypasses config flow.
See the [app contract](../reference/app.md) for UI ownership.

## Choose the configuration owner

| Parameter belongs to | Add metadata at |
|---|---|
| A template or zone | Its `MechanismTemplateInfo` or `ZoneKindInfo` schema |
| A season's typed config | Its `ConfigBindings` table and contributed plugin schema |
| Robot/chassis/wheel/camera/start placement structure | SDK model and shared editor bindings |

Do not add a second copy of a plugin property in the app. Do not let a widget write persistence or
mutable runtime state directly.

## Build a useful property specification

Start with [`props::` builders](../../sdk/include/wpiemu/sdk/config/property_builders.h).
This exact excerpt from `property builders set the fields a schema entry needs` in
[`test_property_builders.cpp`](../../tests/sdk/test_property_builders.cpp) creates a bounded count:

```cpp
const PropertySpec n = props::integer("count", "Count", "Group", 3, 1, 8, "help");
CHECK(n.key == "count");
CHECK(n.label == "Count");
```

The surrounding file includes the builder headers and uses `wpiemu::sdk`.
The remaining assertions check type, default, count unit, bounds and description.
For a real parameter replace the generic label/help with wording that explains what changes.

1. Keep the stored key stable and unique within the schema; labels may be translated.
2. Use SI defaults and `PropertyUnit`; the UI's `UnitFormatter` handles display conversion.
3. Set `group` for readability. `PropertySection::Drive` plus `axis` places motor/actuation controls
   with the correct motion axis; placement controls belong in `Placement`.
4. Use `visibleWhenKey` / `visibleWhenEquals` for dependent controls. Hidden does not mean deleted:
   preserve stored values and validate the relevant runtime configuration.
5. Make defaults valid and give enum options stable values distinct from their display labels.

## Bind typed configuration once

[`game_2026/config.cpp`](../../seasons/2026/game/src/config.cpp) binds each schema spec to a
`GameConfig` member with `ConfigBindings`. That one table supplies schema, decode and encode;
do not separately maintain a parser, serializer and UI property list.

For structural SDK types use
[`robot_spec_bindings.h`](../../sdk/include/wpiemu/sdk/config/robot_spec_bindings.h).
This exact test excerpt demonstrates the shared chassis binding's single-key edit:

```cpp
ChassisSpec ch;
const auto chassis = chassisEditorBindings();
const Value encoded = chassis.encode(ch);
CHECK(validateProperties(chassis.schema(), encoded).empty());
CHECK(chassis.apply(ch, "massKg", Value(62.5)));
CHECK(ch.massKg == doctest::Approx(62.5f));
```

It is from `ConfigBindings supports strings, geometry, custom accessors and single-key apply` in
the same test file. Expected result: the default encodes to valid properties, and an edit changes
the typed member. This is binding-level code, **not** permission to bypass editor undo operations.

When reading untyped values, use `getDouble/getBool/getString/getFiniteDouble` from SDK `Value`;
choose the finite reader where NaN/infinity must fall back. Test wrong-type/missing input explicitly.
Schema validation and typed decoding are different steps; clamping behavior must be intentional.

## Complete the edit and persistence path

For a structural field, update these together:

1. The owning SDK model.
2. The shared codec field table in
   [`robot_config_codec_io.h`](../../core/runtime/src/config/robot_config_codec_io.h).
3. Shared validation in
   [`config_checks.h`](../../sdk/include/wpiemu/sdk/config/config_checks.h), preserving issue keys.
4. The SDK editor binding and the existing undoable `EditorDocument` / `EditorController`
   config-operation path under [`app/editor`](../../app/editor).

Metadata-only changes normally need no new target. New source files must enter their owner's CMake
list; editor tests use `.cc` files and their separate target, not the Qt-free `.cpp` test discovery.
Keep translated strings, `ThemeTokens` and `UnitFormatter` consistent with neighboring controls.

## Validate without confusing model checks with rendering

```powershell
ctest --preset windows-ninja-release -R "property builder|ConfigBindings|Value readers|^config:|^config schema|^app_editor[.]tests$"
```

This executes both excerpts, plugin config defaults/clamping/round-trip, and the editor suite.
Add coverage for edit/undo/redo, save/load and display-unit round-trip; preserve unrelated config
sections. For viewport/picking changes add transform tests and run the editor demo visually:
offscreen success does not verify Quick 3D rendering.

If a property saves but runtime ignores it, trace binding decode and consumption before adding a
widget workaround. If it moves to the wrong inspector card, inspect section/axis metadata first.
