# Season plugin skeleton

The template is a minimal compilable season plugin: its module describes the season, its field owns
static field geometry, and its game state resets and publishes a counter. It builds by default but
is deliberately absent from the built-in plugin aggregator. The
[game-plugin guide](../../../docs/guides/game-plugins.md) explains the full integration procedure.

## Copy and rename

Copy `seasons\_template` to `seasons\<year>` and fill in its [season.json](../season.json) (a game-less
platform season keeps `"game": null` and deletes `game\`). Rename the include subtree
`include\wpiemu\plugins\_template` and namespace `wpiemu::plugins::_template` to `game_<year>`.
Rename the `wpiemu_plugin_template` target and `wpiemu::plugin_template` alias in
[CMakeLists.txt](CMakeLists.txt). Set `pluginId()` and `gameYear()` in
[plugin_info.cpp](src/plugin_info.cpp), then replace the `TODO(new season)` placeholders.
Keep the target's SDK-only dependency. Registration, implementation and validation steps belong to the
[game-plugin guide](../../../docs/guides/game-plugins.md), not to this rename checklist.

## Skeleton files

- [CMakeLists.txt](CMakeLists.txt): static plugin target.
- [plugin.h](include/wpiemu/plugins/_template/plugin.h): registration declaration.
- [plugin_info.h](include/wpiemu/plugins/_template/plugin_info.h) and
  [plugin_info.cpp](src/plugin_info.cpp): identity.
- [game_module.cpp](src/game_module.cpp): placeholder module, field and game state.

A season that plays its own sounds adds `assets/sfx/<name>.wav` and declares them with
`contributeSoundCues()`; the template ships none and is never embedded. See
[sound cues](../../../docs/guides/game-plugins.md#request-a-sound-cue).
