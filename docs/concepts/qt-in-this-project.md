# Qt in this project

Read this when you want to understand how the desktop interface is built without treating every
screen as a QML application.

For example, the editor's toolbars and panels are [Qt](../GLOSSARY.md) widgets, while its 3D
viewport is a [QML](../GLOSSARY.md) scene inside a `QQuickWidget`. The run view uses the same
split: widgets provide controls and overlays, and QML draws the 3D scene.

## The application shell and event loop

The app creates a `QApplication`, builds its services and window, then enters `app.exec()`.
Qt's event loop dispatches input and other events to the interface. See the
[Qt application reference](https://doc.qt.io/qt-6/qapplication.html) and the entry point at
[`app/main.cpp`](../../app/main.cpp).

Qt signals and slots connect user actions to the relevant controller. For example, a viewport
button's `clicked` signal calls a viewport-controller method in
[`ViewportWidget`](../../app/editor/views/viewport_widget.cpp). This keeps a widget from owning
simulation state; the application's host and session layers own that work. See
[Architecture](../ARCHITECTURE.md) for process and thread ownership.

## Widgets, models, and QML

Most app controls are C++ widgets arranged in layouts. The editor and run view each embed a
[Qt Quick](../GLOSSARY.md) scene with `QQuickWidget`, Qt's widget for displaying a Qt Quick
interface; see its [class reference](https://doc.qt.io/qt-6/qquickwidget.html). The scene definitions live in
[`app/editor/qml`](../../app/editor/qml/Viewport3D.qml) and
[`app/runview/qml`](../../app/runview/qml/RunScene.qml). They are not a second simulation engine:
the scene displays data supplied by the C++ application.

The editor separates document/edit operations, view models, and widgets. Its document uses
`QUndoStack` for undoable configuration edits; the run view has scene and HUD models for
presentation. See [`editor_document.cpp`](../../app/editor/model/editor_document.cpp),
[`scene_model.cpp`](../../app/runview/scene_model.cpp), and
[`hud_model.cpp`](../../app/runview/hud_model.cpp).

Qt's object tree also helps with widget lifetime: child widgets are constructed with their parent,
as in the editor viewport. See Qt's guide to
[object trees and ownership](https://doc.qt.io/qt-6/objecttrees.html).

## Resources and build

The QML files are added to Qt resources by the editor and run-view CMake targets, then loaded from
`qrc:/...` URLs. This lets the app load those scenes from its own executable/library resources
rather than depending on a source-tree path. See the [editor CMake file](../../app/editor/CMakeLists.txt)
and [run-view CMake file](../../app/runview/CMakeLists.txt).

The app builds its widgets in C++ and does not use Qt Designer `.ui` files. Its CMake files turn
off `AUTOUIC` because an SDK header named `ui_descriptors.h` could otherwise look like generated
UI output. The shared Qt CMake setup handles meta-object processing; the
[run-view target](../../app/runview/CMakeLists.txt) also explicitly sets `AUTOMOC` on.

Some UI tests run with Qt's offscreen platform. That checks widget and model behavior without
opening a visible window; it does not prove that a hardware-accelerated Qt Quick 3D scene renders
correctly. The [build guide](../BUILDING.md) explains the project's test coverage.
