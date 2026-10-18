# Robot editor (`app/editor`)

The 3D robot editor page: project tree (Robot → Chassis [centre of mass, shapes, Drivetrain → wheels], Mechanisms, Cameras, Robot sensors, game node [volumes, zones]; followers nested under their source), and detected devices (and the cameras the robot uses) on the left (mechanisms are added from the tree's right-click menu; there is no catalog panel), the
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

## Start pose (`SimSettings::startPose`)

Where the robot starts on the field. The model holds an optional `sdk::StartPose {xM, yM, yawRad}` (field frame, the pose of the chassis
origin, i.e. the COM frame of the snapshot); `nullopt` means "the game field's default placement", which the shell adapter reports as
`EditorCatalog::defaultStartPose` (`host::startPlacementFor`), so the editor shows where an unset robot will start and knows no field.

* **Ops / document**: `ops::effectiveStartPose(config, catalog)` (configured, else the default) and `ops::setStartPose(config, optional)`
  (false when nothing changes); `EditorDocument::setStartPose(optional, dragSession)` / `clearStartPose()` make them undoable edits. Typing
  merges under one key, a viewport drag under its `drag:<session>` key (one drag = one undo step), clearing is a step of its own.
* **Inspector**: the Robot page has a second property group `robot.start` (X, Y in the display units, Yaw in degrees, stored in radians:
  `spec::startPoseSchema` / `startPoseValues` / `applyStartPose`; editing one field stores the whole effective pose) drawn as the **Start
  pose** card (`InspectorPanel::addStartPoseCard`): the fields, a hint saying whether the pose is set (and the default placement if
  not) and **Use default placement**. Selecting the marker (`SelKind::StartPose`) shows only that card. An invalid `sim.startPose` issue
  is located on its X field (`locateIssue`).
* **Viewport**: `buildScene` adds a translucent footprint of the hull at the effective pose (`kStartPoseFootprintId`, fainter while the
  pose is unset), a heading shaft and an arrow head (`kStartPoseNoseId`), all pickable with `SelectionRef::startPose()`. Pressing the
  footprint selects it (a click alone never creates a start pose) and dragging moves it over the floor plane through the grabbed
  point; dragging the arrow turns it to face the cursor. The ray/plane maths and the snapping rules are `app/common/robot_placement.h`
  (Qt-free, shared with the run view, tested in `app/tests/test_robot_placement.cpp`); `ViewportController::beginStartPoseDrag` /
  `updateStartPoseDrag` / `endStartPoseDrag` only connect them to the document, the editor's grid steps apply and Ctrl toggles them, and
  QML stays a pointer forwarder. `F` on the marker frames it; framing the robot ignores it.

## Cameras (ADR 0017)

Cameras (`RobotConfig::vision`, `sdk/vision/camera_config.h`) are configured in the robot project: the emulator computes the AprilTag
detections for them and a bridge in the robot process publishes the PhotonVision / Limelight NetworkTables entries. The editor only edits
`RobotConfig::vision` (the JSON codec and the validator are core's), and learns two more things through the backend, so it knows no season and
includes no core or plugin code: the selected plugin's AprilTag layouts (`EditorCatalog::fiducialLayouts`, `findFiducialLayout`,
`activeFiducialLayout(VisionConfig)`, filled by the shell adapter from `IGameModule::contributeFiducialLayouts()` like `zoneKinds`) and the
cameras the robot process reports using (`IEditorBackend::detectedCameras()` and `detectedCamerasChanged()`, from
`ISimSnapshot::detectedCameras()`; entries are merged by (vendor, NetworkTables name), kept for the run and cleared when a new run or
project starts; the controller republishes them as `detectedCameras()` / `detectedCamerasUpdated()`). The reports exist during a
discovery-only session and for a project with no camera at all. `FakeEditorBackend` provides two sample layouts (`sampleLayouts()`, a 6 x 4 m
field around the default start pose), scripted reports (`sampleDetectedCameras()`, `setDetectedCameras`, `announceCamera`, one camera announced
after every second device by `--detect`) and a sample robot with a PhotonVision `photon_front` and a Limelight `limelight_rear`
(`samples/vision_robot`'s mounts).

* **Ops** (`model/config_ops.*`, camera section; undoable through `EditorDocument` and `EditorController`): `addCamera(vendor, attachment)`
  (the id is unique against cameras, zones *and* mechanisms, which share one namespace; PhotonVision: NetworkTables name = id, 30 Hz; Limelight:
  `limelight`, `limelight-2`, ..., 90 Hz; 1280 x 800, 90 degrees, ideal; mounted 5 cm inside the front edge, 20 cm above the frame, tilted 20
  degrees up, a further chassis camera 12 cm to the side), `addCameraFromDetected(CameraRobotState)` (vendor and name from the report, the id a
  sanitised unique version of the name, a Limelight's reported pose becomes its mount, nothing when a camera of that vendor and name exists),
  `removeCamera`, `duplicateCamera` (own id and network name), `reattachCamera` (keeps the world pose, same rules as zones), `setCameraProperty`
  (every `CameraConfig` field by its persisted name; the id and the vendor are fixed), `setCameraExplicitIntrinsics` (freeze the camera matrix from the
  field of view or derive it again) and `setVisionProperty` (`layoutId`, `layoutFile`, `photonMessageUuid`). `removeMechanism` moves the cameras
  of the removed mechanism to its parent in place; `setCom` keeps chassis cameras where they are.
* **Selection / tree**: `SelectionRef::camera(id)` (`SelKind::Camera`) and `SelectionRef::vision()` (`SelKind::Vision`, the project-level
  settings). The **Cameras** group sits under the robot after Mechanisms and selects the vision settings; its rows (`NodeKind::Camera`, detail
  `photon-front · PhotonVision @ turret.mover`, dimmed and prefixed `disabled ·` when switched off) are draggable. `setIssues` gives a camera
  row, and the group, the warning state through `locateIssue`. Context menus (`editor_page.cpp`): the robot, chassis, every mechanism and the
  Cameras group offer **Add PhotonVision camera** / **Add Limelight camera**; a camera offers Duplicate, Delete and Attach to. Dragging a camera onto a
  mechanism or the chassis emits `ProjectTreeModel::cameraDropped` (`kCameraMime`), which re-attaches it.
* **Inspector** (`propertyGroupsFor`, schemas in `model/vision_specs.*`): `camera.general` (label, NetworkTables name with a vendor hint,
  enabled; the id, the vendor, the table path and what the robot process reported about the camera are read-only lines), `camera.placement`
  (attachment parent and link, pose in degrees), `camera.optics` (resolution, horizontal field of view or **Use a camera matrix** with fx / fy / cx /
  cy, frame rate, latency; Limelight: the capture-latency share), `camera.detection` (tag size, minimum tag pixels, maximum view angle, occlusion,
  multi-tag), `camera.realism` (corner noise, latency jitter, dropout %; collapsed while everything is zero, which reads *Ideal*) and, for a
  Limelight only, `camera.assumed` (**Limelight pose**: the camera pose the Limelight believes: the physical mount unless an override is set).
  The forms show milliseconds and percent; `applyCamera` translates them to the persisted microseconds and probability. The **Robot** page
  has a **Vision** card (`robot.vision`: AprilTag layout from the catalog, a custom layout file, an advanced PhotonLib message UUID override
  that is normally derived from the project's PhotonLib version) and the **Vision** selection (the Cameras group) shows the same card with the
  list of cameras and the add buttons. A relative layout file starts at the robot project folder (said in the field's hint).
* **Issues**: validator paths `vision.cameras[i].<field>` select the camera and the form field that shows them (the form's units: `latencyUs` →
  `latencyMs`, `dropoutProbability` → `dropoutPercent`; a collapsed card opens), `vision.layoutId` / `layoutFile` / `photonMessageUuid` select the
  Vision settings. `localIssues` adds `photon_uuid_malformed` (the override is not 32 hexadecimal digits). The vision subsystem's runtime
  warnings carry no path, so `locateIssue` reads their code and message: `vision_layout_missing` → the layout field, `vision_camera_status` → the
  camera named by id, `vision_camera_unconfigured` → the camera with that NetworkTables name, else the Vision settings.
* **Viewport**: every camera is a small body behind a lens (`camera:<id>`, `:lens`, pickable), four edges to the far corners, the rectangle of the far
  plane and a translucent far plane (`ItemRole::Frustum`, geometry from `app/common/camera_frustum.h`, shared with the run view; the plane is
  picked only when nothing solid is under the cursor), depth 10 % of the layout's field length (1 to 3 m), 1.5 m without a layout. A camera that
  is switched off is dimmed; the pyramid brightens while its camera is selected. `readTransform` / `writeTransform` (Move and Rotate, no size) give
  the camera the same gizmos, numeric bar and undo merging as a zone; framing a camera includes its pyramid, framing the robot does not.
  **Tags** in the viewport toolbar (`EditorSettings::showTags`), and any selected camera or the vision settings, draw the active layout's tags
  (`ItemRole::Tag`, `tag:<id>`, never picked) as thin plates as the robot would see the field from its start pose (`ops::fieldToDisplay`), within 7
  m of the robot; the ids are written next to the plates by QML from `ViewportController::tagLabels` (projected in C++). With a camera selected,
  the tags it would see are tinted (`cameraSeesTag`: the vision engine's rules without the line of sight, so the real detections can be fewer).
  A custom layout file draws nothing: the editor does not read it. Colours come from `SceneTheme` (`camera`, `cameraOff`, `tag`, `tagSeen`).
* **Detected cameras**: `DevicePanel` ends with a **Cameras (n)** section (`views/detected_cameras_section.*`, hidden when empty) for the
  cameras the robot code uses that the project does not list: a camera of the report belongs to the project when vendor and NetworkTables name
  match (`unconfiguredCameras` in `model/detected_cameras.*`; ids are never compared, they belong to the run that reported them). Each row shows
  the name, the vendor and a status line, **Add camera** (`EditorController::addCameraFromDetected`, selects the new camera) and, when
  configured cameras of the same vendor go unused by the robot code, a hint that the name in the project may differ from the one in the code:
  with exactly one such camera **Use this name** renames it (one undo step). The General card of a configured camera says whether the robot code
  uses it, and under which other names it uses cameras of that vendor.
* **Wizard**: `updateRobotConfig` copies the existing model and replaces only what the wizard owns, so cameras, layout, UUID override and start
  pose survive a re-run (`app/tests/test_robot_basics.cpp`, `test_wizard_view_model.cpp`, through the stored files too).
* **Tests**: `tests/app_editor/test_cameras.cc` (ops, document, controller, property groups, issues, tree, inspector, scene, viewport, detected
  cameras), `app/tests/test_gui_shell.cpp` (the adapter's layouts and reports), and the wizard tests above.

## Building and testing

* In the full tree: `app/CMakeLists.txt` adds this folder; target `wpiemu_app_editor` (static),
  `wpiemu_editor_demo` (option `WPIEMU_BUILD_UI_DEMOS`, default ON) and `wpiemu_app_editor_tests`
  (CTest `app_editor.tests`, runs with `QT_QPA_PLATFORM=offscreen`).
* Only this module, without the rest of the tree: `cmake -S app/editor/standalone -B build/ui-editor -G Ninja
  -DCMAKE_PREFIX_PATH=<Qt>` (see `standalone/CMakeLists.txt`).
* Test sources live in `tests/app_editor/*.cc` (not `.cpp`: `tests/CMakeLists.txt` globs `*.cpp` into the Qt-free
  `wpiemu_tests`).
* Demo: `wpiemu_editor_demo [--select intake|chassis|wheel:fl|slot:shooter.feeder|zone:pickup|camera:photon_front|vision|plugin] [--detect]
  [--view front|side|top|3d] [--frame] [--tags] [--disable-camera limelight_rear] [--tank] [--light] [--grab out.png]`.
* Strings: model code uses `ETR("...")` (context `Editor`); run `lupdate -tr-function-alias translate+=ETR`.
