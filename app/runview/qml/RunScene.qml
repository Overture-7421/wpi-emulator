import QtQuick
import QtQuick3D

// Live run-view scene. Everything below the "world" node is expressed in SDK coordinates (Z-up, NWU);
// the node's -90 degree X rotation converts to Qt Quick 3D's Y-up space in this one place.
Item {
    id: root

    View3D {
        id: view
        anchors.fill: parent
        camera: cam

        environment: SceneEnvironment {
            clearColor: theme.background
            backgroundMode: SceneEnvironment.Color
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: SceneEnvironment.Medium
        }

        Node {
            id: world
            eulerRotation.x: -90

            PerspectiveCamera {
                id: cam
                position: scene.cameraPosition
                rotation: scene.cameraRotation
                fieldOfView: scene.cameraFov
                clipNear: 0.1
                clipFar: 300
            }

            DirectionalLight {
                eulerRotation: Qt.vector3d(28, 18, 0)
                brightness: theme.dark ? 1.1 : 1.3
                ambientColor: theme.dark ? "#4a5260" : "#8a929e"
            }
            DirectionalLight {
                eulerRotation: Qt.vector3d(-35, -140, 0)
                brightness: 0.35
            }

            // Ground far beyond the field.
            Model {
                source: "#Rectangle"
                position: Qt.vector3d(scene.fieldCenter.x, scene.fieldCenter.y, -0.1)
                scale: Qt.vector3d(1.2, 1.2, 1)
                materials: PrincipledMaterial { baseColor: theme.ground; roughness: 1.0; metalness: 0.0; cullMode: Material.NoCulling }
            }

            Repeater3D {
                model: scene.fieldParts
                delegate: PartModel {}
            }

            Node {
                visible: scene.robotVisible
                position: scene.chassisPosition
                rotation: scene.chassisRotation

                Repeater3D {
                    model: scene.robotParts
                    delegate: PartModel {}
                }
                Node {
                    visible: scene.zonesVisible
                    Repeater3D {
                        model: scene.zones
                        delegate: PartModel {}
                    }
                }
            }

            // Vision overlay (ADR 0017): cameras with their viewing pyramid, the field's AprilTags (the ones a camera sees highlighted) and
            // a ray from each camera to every tag it sees. World-frame poses like everything else under `world`.
            Node {
                visible: scene.camerasVisible
                Repeater3D {
                    model: scene.cameras
                    delegate: CameraVisual {}
                }
            }
            Node {
                visible: scene.tagsVisible
                Repeater3D {
                    model: scene.tags
                    delegate: PartModel {}
                }
            }
            Node {
                visible: scene.camerasVisible && scene.tagsVisible
                Repeater3D {
                    model: scene.tagRays
                    delegate: PartModel {}
                }
            }

            Repeater3D {
                model: scene.entityGroups
                delegate: Model {
                    required property bool isSphere
                    required property vector3d groupScale
                    required property color groupColor
                    required property var entityInstancing

                    source: isSphere ? "#Sphere" : "#Cube"
                    scale: groupScale
                    instancing: entityInstancing
                    materials: PrincipledMaterial { baseColor: groupColor; roughness: 0.45; metalness: 0.0 }
                }
            }
        }
    }

    MouseArea {
        id: pointer
        anchors.fill: parent
        acceptedButtons: Qt.LeftButton | Qt.RightButton | Qt.MiddleButton
        hoverEnabled: true
        property real lastX: 0
        property real lastY: 0
        property bool movingRobot: false
        property bool hoverRobot: false
        cursorShape: movingRobot ? Qt.ClosedHandCursor : (hoverRobot ? Qt.OpenHandCursor : Qt.ArrowCursor)
        onPressed: (mouse) => {
            lastX = mouse.x
            lastY = mouse.y
            // A plain left press on the robot picks it up while a simulation runs (the scene model decides); anywhere else it pans.
            movingRobot = mouse.button === Qt.LeftButton && !(mouse.modifiers & (Qt.AltModifier | Qt.ShiftModifier))
                && scene.beginRobotDrag(mouse.x, mouse.y)
        }
        onPositionChanged: (mouse) => {
            const dx = mouse.x - lastX
            const dy = mouse.y - lastY
            lastX = mouse.x
            lastY = mouse.y
            if (movingRobot) {
                scene.dragRobot(mouse.x, mouse.y, mouse.modifiers)
                return
            }
            if (!mouse.buttons) {
                hoverRobot = scene.robotAt(mouse.x, mouse.y)
                return
            }
            // Same mapping as the editor: right-drag or Alt+left-drag orbits, left-drag, middle-drag or Shift+drag pans.
            const orbiting = !(mouse.buttons & Qt.MiddleButton) && !(mouse.modifiers & Qt.ShiftModifier)
                && ((mouse.buttons & Qt.RightButton) || (mouse.modifiers & Qt.AltModifier))
            if (orbiting) scene.orbit(dx, dy)
            else scene.pan(dx, dy)
        }
        onReleased: {
            if (movingRobot) {
                scene.endRobotDrag()
                movingRobot = false
            }
        }
        onCanceled: {
            scene.endRobotDrag()
            movingRobot = false
        }
        onExited: hoverRobot = false
        onWheel: (wheel) => {
            // While the robot is held the wheel turns it; otherwise it zooms.
            if (!(movingRobot && scene.turnRobotDrag(wheel.angleDelta.y / 120.0, wheel.modifiers)))
                scene.zoom(wheel.angleDelta.y / 120.0)
        }
        onDoubleClicked: scene.resetCamera()
    }

    onWidthChanged: scene.setViewSize(width, height)
    onHeightChanged: scene.setViewSize(width, height)
    Component.onCompleted: scene.setViewSize(width, height)

    // Tag ids and camera names of the vision overlay. Qt Quick 3D has no 3D text, so the scene model projects each anchor with the orbit
    // camera that drives the 3D view and hides the label behind the camera, off screen, or for a tag nobody sees while far away.
    Item {
        anchors.fill: parent
        visible: scene.robotVisible
        Repeater {
            model: scene.labels
            delegate: Rectangle {
                id: tag

                required property string labelText
                required property real labelX
                required property real labelY
                required property bool labelShown
                required property bool labelEmphasis
                required property int labelKind

                visible: labelShown
                x: labelX - width / 2
                y: labelY - height - 2
                width: caption.implicitWidth + 10
                height: caption.implicitHeight + 3
                radius: 4
                color: Qt.rgba(theme.panel.r, theme.panel.g, theme.panel.b, labelEmphasis || labelKind === 1 ? 0.88 : 0.55)
                border.color: labelEmphasis ? theme.accent : theme.panelBorder
                border.width: 1
                Text {
                    id: caption
                    anchors.centerIn: parent
                    text: tag.labelText
                    color: tag.labelEmphasis || tag.labelKind === 1 ? theme.text : theme.mutedText
                    font.pixelSize: tag.labelKind === 1 ? 12 : 11
                    font.bold: tag.labelEmphasis
                }
            }
        }
    }

    // Game-plugin HUD: generic renderer of HudWidgetDescriptors.
    Item {
        anchors.fill: parent
        anchors.margins: 12
        visible: options.showHud && scene.robotVisible

        HudStack { anchorIndex: 0; hAlign: 0; anchors.left: parent.left; anchors.top: parent.top }
        HudStack { anchorIndex: 1; hAlign: 1; anchors.horizontalCenter: parent.horizontalCenter; anchors.top: parent.top }
        HudStack { id: hudTopRight; anchorIndex: 2; hAlign: 2; anchors.right: parent.right; anchors.top: parent.top }
        HudStack { anchorIndex: 3; hAlign: 0; anchors.left: parent.left; anchors.bottom: parent.bottom }
        HudStack { anchorIndex: 4; hAlign: 1; anchors.horizontalCenter: parent.horizontalCenter; anchors.bottom: parent.bottom }
        HudStack { anchorIndex: 5; hAlign: 2; anchors.right: parent.right; anchors.bottom: parent.bottom; anchors.bottomMargin: 54 }
        HudStack { anchorIndex: 6; hAlign: 1; anchors.centerIn: parent }
    }

    MechanismPanel {
        anchors.left: parent.left
        anchors.leftMargin: 12
        anchors.verticalCenter: parent.verticalCenter
        visible: options.showMechanisms && scene.robotVisible && count > 0
    }

    // Vision cards: right edge, centred (the mechanism cards use the left edge; the game HUD the corners and the top and bottom centre),
    // but never over the game HUD of the top-right corner.
    CameraPanel {
        anchors.right: parent.right
        anchors.rightMargin: 12
        y: Math.max((parent.height - height) / 2, hudTopRight.visible ? hudTopRight.mapToItem(root, 0, hudTopRight.height).y + 8 : 12)
        visible: scene.camerasVisible && scene.robotVisible && count > 0
    }
}
