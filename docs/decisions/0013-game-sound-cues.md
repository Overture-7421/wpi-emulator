# Game sound cues

Read this when a game needs to make a sound or the app seems to need a season's audio.
Prerequisite: [Architecture](../ARCHITECTURE.md).

## Status
Accepted.

## Context
The 2026 manual defines audio cues for the match timeline. The app had no audio. Snapshots are
published every few ticks and the UI may skip some, so an event that lasts one tick cannot be
read from them reliably. Deriving cues from HUD values in the UI would put season rules in the
UI, and the plugin cannot use Qt.

## Decision
The plugin owns its sounds: it declares them (`SoundCueInfo`), ships the audio files and raises
cues from its game state through a bounded `SoundCueLog` in the snapshot. Each cue has an
increasing sequence number and a simulation time; the UI plays each sequence once, stays silent
for what happened before it started watching, and drops stale cues. The app embeds every
plugin's audio files as Qt resources through one aggregated CMake property, so it never names a
season. Playback is a small `ISoundOutput` over Qt Multimedia's `QSoundEffect`; tests inject a
recording output. Volume belongs to the operating system; the app keeps only an on/off
preference.

## Consequences
A new season adds sounds without touching core or UI. Qt Multimedia becomes a required module of
the Qt kit, CI and the installers; Windows skips its FFmpeg DLLs because WAV playback does not
need them. Audio output cannot be verified headless, so a listening pass stays manual. Third-party
audio ships unmodified with its provenance and license in the notices.
Contracts belong in the [game-plugin reference](../reference/game-plugin.md#sound-cues).

Next: [User guide](../USER_GUIDE.md).
