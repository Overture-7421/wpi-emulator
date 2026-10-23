# Sound cues travel as events, not snapshots

Read this when a game, the Driver Station or the UI needs a one-shot notification.
Prerequisite: [Architecture](../ARCHITECTURE.md).

## Status
Accepted.

## Rationale
Snapshots describe current state and the UI may skip intermediate snapshots. A one-shot cue stored
only in a snapshot can therefore be lost; season-aware playback in the UI would also couple the
app to plugins.

## Decision
Represent sound requests as ids and hand them to the app through a bounded event queue. The app
resolves the cue to an asset and owns playback; plugins request cues without depending on Qt.

## Trade-offs
Events need an explicit bounded hand-off and asset ownership, and cues are not durable across
process restarts. This avoids adding audio state to snapshots or game-specific UI logic.

Current cue ownership and playback are in [Game plugins](../reference/game-plugin.md).
