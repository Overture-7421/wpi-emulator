# REBUILT match audio

Read this when replacing, adding or checking the sounds the 2026 plugin plays.
Prerequisites: the [game plugin reference](../../../../docs/reference/game-plugin.md#sound-cues).

These six files are the FMS audio cues of the 2026 Game Manual (section 5.12, Table 5-4). They are
**unmodified copies** of the same-named files in
[Team 254's Cheesy Arena](https://github.com/Team254/cheesy-arena/tree/248d9de497e701223e2cdbc986a19bbfea054c21/static/audio),
commit `248d9de497e701223e2cdbc986a19bbfea054c21`. Do not trim, normalise or re-encode them: the upstream license
does not allow redistributing modified copies without Team 254's permission. Its text and the attribution are in
[THIRD-PARTY-NOTICES.txt](../../../../packaging/THIRD-PARTY-NOTICES.txt). Balance loudness with `SoundCueInfo::gain`
in [`match_sounds.cpp`](../../src/match_sounds.cpp), not by editing the files.

| File | Manual cue | Format | SHA-256 |
|---|---|---|---|
| `start.wav` | Cavalry Charge (MATCH start) | PCM 16-bit mono 11.025 kHz | `6c1a61c91b68d594d22c2cfdc5d110eaec21b9f5b214e3a4c36e11a095bc8df0` |
| `end.wav` | Buzzer (AUTO ends, MATCH end) | PCM 16-bit mono 11.025 kHz | `11d81b82b77cee873244f7b4c4774d7a89379da111c98f8d03cecf045af7caf5` |
| `resume.wav` | 3 Bells (TELEOP begins) | PCM 16-bit mono 22.05 kHz | `a34bfdc52e8e441eeeaa30b95af28df46d2f6637901a2f75f30aba8ddc88d45d` |
| `shift_change.wav` | POWER UP - Linear Popping (shift starts) | PCM 16-bit mono 44.1 kHz | `2ed56e87bbc84f3e436a1660f0d2728cc78a435527bce8fc065fc842e9c0700f` |
| `warning.wav` | Steam Whistle (END GAME begins) | PCM 8-bit mono 11.025 kHz | `151984d4cfe331c52ed9fcbed2d1be50172a376691c7bfb5e96f83b1d97d1a86` |
| `abort.wav` | Foghorn (MATCH stopped) | PCM 16-bit stereo 44.1 kHz | `61ec6647bd9ab03051f7949512ec77fa870fffc0d90ee7ed0eae1109e0808d2b` |

Check a copy with `Get-FileHash -Algorithm SHA256 <file>` (PowerShell) or `sha256sum <file>`. The file-to-cue mapping
follows Cheesy Arena's `game/match_sounds.go`, which implements the same table.

Next: [game plugin reference](../../../../docs/reference/game-plugin.md#sound-cues).
