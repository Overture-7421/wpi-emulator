# Sound cues travel as events, not snapshots

Read this when a game, the Driver Station or the UI needs to play a sound or send another one-shot notification.
Prerequisite: [Architecture](../ARCHITECTURE.md).

## Status
Accepted.

## Context
Practice should sound like a real match. Some cues are the same every season, such as the start
and end buzzers; others are a season's own, such as the alliance shift sound. A snapshot is
published every few ticks and the UI may skip any of them, so a one-shot carried in a snapshot can
be lost. The UI must also stay free of season knowledge, and plugins stay free of Qt.

## Decision
A cue is an id. The SDK owns the common ids (the FMS audio cues every season shares), the descriptor
that names a sound file, the sink a game calls and a bounded queue the host drains like logs. The
Driver Station requests the match lifecycle cues because it owns the Practice clock; a plugin
requests the cues whose timing is its season's. The owner of an id prefix owns its sound file: the
app for common ids, the plugin for its own, which the `plugins/` aggregation seam hands to the app
build so no season code or plugin name enters the UI. Playback sits behind an interface in the run
view, and the user's volume and mute live in application settings.

## Consequences
No cue is lost or repeated when the UI skips snapshots, and the wire protocol and `robot.json` are
unchanged. Qt Multimedia becomes a required dependency of the app build. Cues are not queued while
the page is hidden. A plugin's sounds are embedded at build time, so only built-in plugins can ship
audio, which matches static plugin registration. The contract is in the
[game-plugin reference](../reference/game-plugin.md#sound-cues).

Next: [User guide](../USER_GUIDE.md).
