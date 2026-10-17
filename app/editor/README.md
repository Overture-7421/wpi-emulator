# Robot editor (`app/editor`)

The 3D robot editor page: project tree, mechanism palette and detected devices on the left, the
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

| Folder | Contents | May include |
|---|---|---|
| `model/` | `EditorDocument` (config + `QUndoStack`), pure edit operations (`config_ops`), property groups / schemas, slot status + local validation, scene builder, transform access | SDK, Qt Core/Gui |
| `viewmodel/` | `EditorController` (actions, backend sync, validation, autosave), tree / device / issue / scene item models, `ViewportController` (camera, gizmo, picking) | model |
| `views/` | widgets only: inspector, property form + field editors, slot rows, tree view, device panel, palette, bottom tabs, toasts, viewport host | viewmodel, model |
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
Right-drag orbits, middle-drag pans, wheel zooms, left-click selects (a second click on a mechanism's shape
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
* Demo: `wpiemu_editor_demo [--select intake|chassis|wheel:fl|slot:shooter.feeder|plugin] [--detect] [--light]
  [--grab out.png]`.
* Strings: model code uses `ETR("...")` (context `Editor`); run `lupdate -tr-function-alias translate+=ETR`.
