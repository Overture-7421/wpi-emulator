# Robot editor (`app/editor`)

The 3D robot editor page: project tree (Robot → Chassis [centre of mass, shapes, Drivetrain → wheels], Mechanisms, Robot sensors, game node [volumes, zones]; followers nested under their source), and detected devices on the left (mechanisms are added from the tree's right-click menu; there is no catalog panel), the
Qt Quick 3D viewport with console / validation / telemetry tabs in the middle, and a property
inspector generated from the SDK's `PropertySchema` metadata on the right.

```
EditorPage (QWidget)                         editor_page.h        shell embeds this
 ├─ IEditorBackend (QObject, pure interface) editor_backend.h     shell implements this
 └─ FakeEditorBackend                        fake_editor_backend.h tests + demo
```

## Contract with the shell

`EditorPage(IEditorBackend&, QWidget* parent)` is the whole public surface, plus a few actions for
menus (`undoAction()`, `redoAction()`, `saveAction()`, `detectAction()`, ...) and the signals
`configEdited()` / `autosaveRequested()`. The page never includes core, ipc or plugins: it pulls
everything through the backend (`config()`, `catalog()`, `devices()`, `validate()`) and pushes edits
back (`submitConfig()` on every edit, `saveConfig()` for autosave / Ctrl+S, `startDetection()` /
`stopDetection()`). The catalog carries the mechanism templates, drivetrain kinds and the *selected
game plugin's* config schema and editor panels, so no season or template is known to the UI.

`saveConfig()` returns `sdk::Status` for authoritative persistence. On failure the controller keeps
the document dirty, displays persistent retry guidance and does not emit a successful-save
notification. Project export failure after app-data success is only a warning. The shell's Save
and project-transition flush use the same controller path; the adapter binds pending edits to the
project in which they were submitted.

## Layers

Motion links (ADR 0012) are edited from the **Motion** section of a wheel's or mechanism's inspector
(`views/drive_card.*`): an **Own motors | Linked to another part** choice, a source and a ratio per motion axis, converted between
the displayed rev/rev, m/m, m/rev, rev/m and SI storage. `model/motion_links.*` (`MotionIndex`,
`OwnerMotion`, `applyDrivenBy`) answers who follows whom for wheels and mechanisms alike. A part driven
by a link takes no motor: its motor slot is not listed (tree, inspector, drops), linking drops its mapping in the
same undo step, and the project tree nests it under its source as a read-only row (`FollowerRole`);
sensor slots stay usable. Because tree position then means "driven by", follower mechanisms are not draggable
and their attachment is shown as `@ parent` text. Each wheel is linked from its own Drive card (no drivetrain-level preset).
Rename preserves endpoint references; delete leaves actionable broken-link issues; duplicate copies
incoming links to the copied axis.

Gearboxes with several motors (ADR 0014): `SlotSpec::countKey/ordinal` make `ops::slotSpecsFor` list one motor row per
motor (`ops::motorGroups`, `setMotorCount`, `pruneInactiveMappings`; wheels use `<wheel>.driveMotorCount`). A controller row
(`views/device_row.*`) shows the mapped device, a **Reversed** toggle (`EditorController::setMotorReversed`, undoable) and
unmap; devices are never picked in the inspector, they are dragged onto the tree, and `mapDevice` on a part with no free
motor row appends a motor in the same undo step (`growableMotorGroup`). Followers of a motion link list no motor rows.

Type: a mechanism's type is its `motion` property (Rotary / Arm / Linear; there is no template switching, and the Add mechanism menus list core templates only, plugin templates are added from the plugin node's menu); a wheel's type is its `kind` (a swerve module is steered,
which moves the steer motor, ratio and controller row into Configure).

Inspector layout: a part (wheel / mechanism) page is an editable Name, then `views/section_card.*` cards with a colour per
role: **Drive** (`PropertySpec::section == Drive`, grouped per motion axis by `PropertySpec::axis`; `views/drive_card.*`),
**Configure** (type, other properties, plugin panels, read-only sensor rows), **Placement**, **Shapes**. `propertyGroupsFor`
emits `mech.drive:<axis>`, `mech.props`, `mech.placement` (wheels: `wheel.drive`, `wheel`, `wheel.placement`);
`applyProperty` accepts all of them.

## Zones (ADR 0016)

Zones (`RobotConfig::zones`, `sdk/config/zone.h`) are game-owned places on the robot, not mechanisms: the catalog carries the
selected plugin's `sdk::ZoneKindInfo` list (`EditorCatalog::zoneKinds`, `findZoneKind`), filled by the shell adapter from
`IGameModule::contributeZoneKinds()`. The editor knows no concrete kind.

* **Ops** (`model/config_ops.*`, `ops::` zone section, wrapped as undoable edits by `EditorDocument` and `EditorController::addZone`):
  `addZone` (id from the kind's last segment, unique against zone *and* mechanism ids; label, pose, size and property defaults from the
  kind), `removeZone`, `duplicateZone`, `reattachZone` (keeps the world pose like `reparentMechanism`), and the activation edits
  `addZoneCondition` / `setZoneCondition` / `removeZoneCondition` / `setZoneActivationMode`. `removeMechanism` moves the zones hanging on
  the removed mechanism to its parent in place; conditions keep a dangling mechanism id (the validator warns, the card shows *(missing)*).
  `setCom` keeps chassis-attached zones where they are.
* **Selection / tree**: `SelectionRef::zone(id)` (`SelKind::Zone`). The game node lists the zones after its volumes
  (`NodeKind::Zone`, detail `@ chassis` / `@ intake` / `@ turret.mover`); `ProjectTreeModel::setIssues` gives a zone node the warning state
  when an issue path (`zones[i]...`, resolved by `locateIssue`) targets it. Context menus (`editor_page.cpp`): the game node, the chassis
  and every mechanism offer **Add <kind>** per catalog kind; a zone offers Duplicate, Delete and Attach to. Dragging a zone onto a mechanism
  or the chassis emits `ProjectTreeModel::zoneDropped` (`kZoneMime`), which re-attaches it.
* **Inspector**: `propertyGroupsFor` emits `zone.general` (label; the id is a read-only line), `zone.placement` (attachment parent and link,
  pose, size for Volume kinds) and `zone.props` (the kind's `PropertySchema`); `applyProperty` accepts them. The **Activation** card
  (`views/activation_card.*`, `SectionRole::Activation`) is a custom widget: *Active when* All/Any, one row per condition (mechanism, value
  name, Above/Below/Within, threshold + tolerance, absolute, remove), *+ Add condition*, and *Always active* when empty. It edits through
  `EditorDocument::setZoneCondition` & co. and refreshes in place (rows are rebuilt only when the count changes); issue paths
  `...activation.conditions[j]...` map to the field key `activation:<j>` and show a badge on that row.
* **Viewport**: `buildScene` draws a Volume zone as a translucent `ItemRole::Overlay` box in the kind's colour at
  `zoneWorld` (attachment link pose × local) and a Muzzle zone as a marker plus an arrow along local +X; both are pickable
  (`SelKind::Zone`). `readTransform` / `writeTransform` edit pose and size (not size for Muzzle) so the gizmos, the numeric bar and undo
  merging work like for mechanisms. The editor shows no live "active" state; the run view does.

## Sensors

Sensors (`RobotConfig::sensors`, `sdk/config/sensor.h`) are core-owned Distance / Beam-break devices fed by a plugin *game value*
(`EditorCatalog::gameValues`, from `IGameModule::contributeGameValues()`). They have no pose, so there is no viewport marker.
`ops::addSensor` / `removeSensor` / `duplicateSensor` / `uniqueSensorId` are undoable through `EditorDocument` and
`EditorController::addSensor`; every other id generator avoids sensor ids. The inspector shows General, Source (a game-value
dropdown that keeps a saved-but-missing source as `(unavailable) <id>`), Response and the Device slot (`sensor`, owner = sensor id).
The project tree lists them under "Sensors" (`sensor:<id>` nodes with a nested slot row); `locateIssue` resolves `sensors[i]` paths.

## Cameras (ADR 0017)

Cameras (`RobotConfig::cameras`, `sdk/config/camera.h`) are simulated Limelight / PhotonVision coprocessors. They are attached to the
chassis or a mechanism link exactly like zones, but belong to no game plugin, so the editor offers them whatever the plugin is.

* **Ops** (`model/config_ops.*`, `ops::` camera section, undoable through `EditorDocument` and `EditorController::addCamera`):
  `uniqueCameraId` (distinct from camera, zone *and* mechanism ids; the mechanism and zone generators avoid camera ids too),
  `cameraDisplayName`, `cameraParentWorld` / `cameraWorld`, `makeCamera` / `addCamera` (a forward-facing mount just behind the front of the
  hull on the chassis, the link origin on a mechanism; `CameraConfig` optics defaults; id `limelight` / `photon`, label with a number from
  the second camera of a kind on, robot-code `name` unique per kind: Limelight `""`, `cam2`, `cam3`..., PhotonVision `photon`, `photon2`...;
  an unknown attachment falls back to the chassis), `removeCamera`, `duplicateCamera`, `reattachCamera` (keeps the world pose),
  `findCamera`. `removeMechanism` moves its cameras to the mechanism's parent in place and `setCom` keeps chassis cameras where they are.
  `cameraRobotPose` is the **robot-to-camera transform in the WPILib robot frame**: the display frame already has its origin on the floor
  below the hull centre with +X forward, +Y left, yaw CCW, so it equals `cameraWorld` (the rest pose for a moving mechanism).
* **Selection / tree**: `SelectionRef::camera(id)` (`SelKind::Camera`). A **Cameras** group under the robot node lists them
  (`NodeKind::Camera`, detail `@ chassis` / `@ turret.mover`); `ProjectTreeModel::setIssues` gives a camera the warning state when an
  issue path `cameras[i]...` (resolved by `locateIssue`, field keys = the inspector keys) targets it. Context menus (`editor_page.cpp`):
  the chassis, every mechanism and the robot/group nodes offer **Add camera > Limelight / PhotonVision**; a camera offers Duplicate, Delete
  and Attach to. Dragging a camera onto a mechanism or the chassis emits `ProjectTreeModel::cameraDropped` (`kCameraMime`).
* **Inspector**: `camera.general` (label, type Limelight / PhotonVision, camera name with the hint *Limelight: empty = 'limelight', otherwise
  'limelight-<name>'; PhotonVision: the PhotonCamera name*; the id is a read-only line), `camera.optics` (field of view, max range and
  incidence, fps, latency, resolution, position / yaw noise) and `camera.placement` (attachment parent and link, pose through the unit
  system) followed by a read-only **Robot-to-camera (WPILib robot frame)** line (`robotToCameraText`: x, y, z in the display units and
  roll / pitch / yaw; the tooltip has metres / radians to copy into robot code). Changing the type re-derives the default name so it stays
  unique. `applyProperty` accepts the three groups; validation issues surface through the field badges as for zones.
* **Viewport**: `buildScene` draws a small body box plus the field of view at `cameraWorld`: eight `ItemRole::Link` edge lines and a
  translucent `ItemRole::Overlay` far plane, in a colour per kind (`SceneTheme::limelight` / `photonVision`), all pickable as
  `SelKind::Camera`. The pyramid comes from `app/common/camera_frustum.h` (`cameraFrustum`, Qt-free, camera frame, +X forward), shared with
  the run view. `readTransform` / `writeTransform` allow move and rotate (not scale), so gizmos, the numeric bar and undo merging work.
* **Demo**: the sample robot has a Limelight on the chassis and a PhotonVision camera on the turret;
  `wpiemu_editor_demo --select camera:limelight`. `FakeEditorBackend::validate` mirrors the real camera checks.

| Folder | Contents | May include |
|---|---|---|
| `model/` | `EditorDocument` (config + `QUndoStack`), pure edit operations (`config_ops`), property groups / schemas, slot status + local validation, scene builder, transform access | SDK, Qt Core/Gui |
| `viewmodel/` | `EditorController` (actions, backend sync, validation, autosave), tree / device / issue / scene item models, `ViewportController` (camera, gizmo, picking) | model |
| `views/` | widgets only: inspector, property form + field editors, slot rows, tree view, device panel, bottom tabs, toasts, viewport host | viewmodel, model |
| `qml/` | `Viewport3D.qml`, a thin renderer fed by `ViewportController` (all colours, lists and camera values come from C++) | - |

Every edit is a snapshot command on the undo stack (`ConfigEditCommand`) built from the Qt-free
operations in `model/config_ops.h`; consecutive edits of one property, and a whole gizmo drag, merge
into one undo step.

Document change notifications distinguish authoritative undo/redo/load from live edit echoes.
Forms refresh all components on authoritative changes even when focused; ordinary echoes and
validation refreshes preserve focused typing and its cursor. Discovery bulk refreshes replace the
physical-ID directory, including non-live historical devices after a run restarts.

## Viewport conventions

World = the SDK's Z-up NWU frame with the floor at z = 0 below the hull centre; the chassis frame (origin
= COM) sits at `(comOffset.xy, comHeight)`; wheel `positionM.z` is a height above the floor. The Qt Quick
3D scene is Y-up, so everything lives under one node rotated -90 degrees about X (conversion only in QML).
Right-drag (or Alt+left-drag) orbits, left-drag on empty space or middle-drag pans, wheel zooms, left-click selects (a second click on a mechanism's shape
drills into the shape); W/E/R choose move / rotate / scale, F frames, 1-4 pick Front / Side / Top / 3D,
Ctrl toggles snapping while dragging, Del deletes, Ctrl+D duplicates, Esc clears the selection.

## Building and testing

* In the full tree: `app/CMakeLists.txt` adds this folder; target `wpiemu_app_editor` (static),
  `wpiemu_editor_demo` (option `WPIEMU_BUILD_UI_DEMOS`, default ON) and `wpiemu_app_editor_tests`
  (CTest `app_editor.tests`, runs with `QT_QPA_PLATFORM=offscreen`).
* Only this module, without the rest of the tree: `cmake -S app/editor/standalone -B build/ui-editor -G Ninja
  -DCMAKE_PREFIX_PATH=<Qt>` (see `standalone/CMakeLists.txt`).
* Test sources live in `tests/app_editor/*.cc` (not `.cpp`: `tests/CMakeLists.txt` globs `*.cpp` into the Qt-free
  `wpiemu_tests`).
* Demo: `wpiemu_editor_demo [--select intake|chassis|wheel:fl|slot:shooter.feeder|zone:pickup|plugin] [--detect] [--tank] [--light]
  [--grab out.png]`.
* Strings: model code uses `ETR("...")` (context `Editor`); run `lupdate -tr-function-alias translate+=ETR`.
