# Season plugin skeleton

Read this when copying a starting point for a new season.
Prerequisites: [Architecture](../../docs/ARCHITECTURE.md) and the
[game-year walkthrough](../../docs/guides/game-years.md).

This compiling skeleton supplies an empty field, a ticking game state and a HUD counter.
It is built by default but is deliberately absent from the built-in plugin aggregator.

## Copy and rename

Copy `plugins\_template` to `plugins\game_<year>`. Rename the include subtree
`plugins\_template\include\wpiemu\plugins\_template` and namespace `wpiemu::plugins::_template` to match.
Rename the `wpiemu_plugin_template` target and `wpiemu::plugin_template` alias in
[CMakeLists.txt](CMakeLists.txt). Set `pluginId()` and `gameYear()` in
[plugin_info.cpp](src/plugin_info.cpp), then replace the `TODO(new season)` placeholders.
Keep the target's SDK-only dependency. Registration, implementation and validation steps
belong to the [game-year walkthrough](../../docs/guides/game-years.md), not to this skeleton checklist.

## Skeleton files

- [CMakeLists.txt](CMakeLists.txt): static plugin target.
- [plugin.h](include/wpiemu/plugins/_template/plugin.h): registration declaration.
- [plugin_info.h](include/wpiemu/plugins/_template/plugin_info.h) and
  [plugin_info.cpp](src/plugin_info.cpp): identity.
- [game_module.cpp](src/game_module.cpp): placeholder module, field and game state.

A season that plays its own sounds adds `assets/sfx/<name>.wav` and declares them with
`contributeSoundCues()`; the template ships none and is never embedded
([walkthrough](../../docs/guides/game-interactions.md#request-a-sound-cue)).

Next: [Game years](../../docs/guides/game-years.md) to implement, register, and exercise the season.
