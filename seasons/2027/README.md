# 2027 season pack (preview)

A platform-only season: it has an HALSim extension, launch profile, Java and vendor schemas but no game
plugin, so projects run core-only on an empty field. The manifest ([season.json](season.json)) pins
the WPILib alpha the extension is built against and the versions verified with it; everything else the
emulator knows about this year is read from it ([season packs](../../docs/reference/seasons.md)).

- Probe samples for the real-robot end-to-end tests: [2027 samples](../../samples/season_2027/README.md).
- Limits of the pinned preview (C++ PhotonLib, Phoenix coverage): see the
  [bridge compatibility reference](../../docs/reference/bridges.md#photonvision).
- Adding a game plugin later is a new `game/` directory and `"game": {"plugin": "game_<year>"}`
  in the manifest; see the [game plugin guide](../../docs/guides/game-plugins.md).
