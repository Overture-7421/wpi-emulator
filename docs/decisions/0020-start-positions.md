# ADR 0020 - Start positions, run-time placement and the scoreboard widget

- Status: accepted
- Scope: `RobotConfig::startPositions`, `IGameModule::fieldFrame`, `ISimController::setStartPlacement`,
  `SimulationSession`, `IRuntimeHost` / `IRunBackend` start position, `HudWidgetKind::Scoreboard`,
  editor, run view, `game_2026`

## Context

ADR 0018 added one optional `startPose` that Reset restores and deliberately does not flip for the alliance. Teams
test on both alliances and from several autonomous start positions, and had to edit the robot setup each time.
The 2026 Practice HUD also showed red on the left and blue on the right (the opposite of the run view's home camera,
where the blue wall is on the left), had no scoreboard, and lost its scores as soon as the match ended.

## Decision

- `RobotConfig::startPositions` is a user-defined list of `StartPosition {id, label, alliance, pose}` (field frame,
  SI, `robot.json` array `startPositions`; absent = none). There are no built-in positions. `startPose` is unchanged
  and stays the **Default** start. Validator codes: `start_position_id_invalid`, `start_position_id_duplicate`,
  `start_position_invalid`.
- The season's field symmetry comes from the plugin: `IGameModule::fieldFrame()` returns length, width and
  `FieldSymmetry` (`None`, `Rotational`, `Mirrored`); `sdk::flipToOtherAlliance` maps a pose to the other
  alliance. The editor's **Copy to Red / Blue** tool uses it. `game_2026` is rotational (authored for blue, rotated
  180 degrees about the field centre for red). Without a plugin the tool is disabled.
- The placement is decided by the emulator, so the wire protocol does not change.
  `ISimController::setStartPlacement(optional<RobotStartPose>)` is queued in order with the other commands and sets the
  pose `SimulationSession::reset()` restores (x, y, yaw; the built spawn height is kept). It never teleports by itself;
  the UI sends it followed by a Reset. `host::Options::startPose` seeds a new session.
- The selected start position is state of the runtime host (`IRuntimeHost::startPosition`, `setStartPosition`), so it
  survives robot-code restarts, is cleared on a project switch or when the entry is deleted, and is re-resolved when the
  entry is edited. It is not persisted across app restarts.
- The run view has a Start position button. Choosing an entry also switches the Driver Station alliance, stops a running
  match and resets there; Restart keeps the choice.
- `HudWidgetKind::Scoreboard` (two `HudScoreSide`s, `sides[0]` on the left) is a generic widget; the 2026 plugin
  replaces its corner score/hub cards with one top-centre scoreboard (blue left, red right, leader highlight, lead
  margin, phase caption, hub status). It sits below the Driver Station clock pill.
- `game_2026` keeps the final score on the HUD (`match.phase` = `FINAL`) after a timed match stops, until a reset, the
  next timed match, or free play is enabled.

## Consequences

Robot odometry and gyro are not touched: robot code still has to seed its own pose (an auto's reset pose or vision).
Editing start positions is a config edit and so relaunches the robot code on the next Run start, like any edit.
Scores are fuel points only (tower and ranking points are not scored).
