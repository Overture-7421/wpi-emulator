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

            Node {
                visible: scene.tagsVisible
                Repeater3D {
                    model: scene.tags
                    delegate: PartModel {}
                }
                Repeater3D {
                    model: scene.tagMarkers
                    delegate: TagMarker {}
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
        property real lastX: 0
        property real lastY: 0
        onPressed: (mouse) => { lastX = mouse.x; lastY = mouse.y }
        onPositionChanged: (mouse) => {
            const dx = mouse.x - lastX
            const dy = mouse.y - lastY
            lastX = mouse.x
            lastY = mouse.y
            // Same mapping as the editor: right-drag or Alt+left-drag orbits, left-drag, middle-drag or Shift+drag pans.
            const orbiting = !(mouse.buttons & Qt.MiddleButton) && !(mouse.modifiers & Qt.ShiftModifier)
                && ((mouse.buttons & Qt.RightButton) || (mouse.modifiers & Qt.AltModifier))
            if (orbiting) scene.orbit(dx, dy)
            else scene.pan(dx, dy)
        }
        onWheel: (wheel) => scene.zoom(wheel.angleDelta.y / 120.0)
        onDoubleClicked: scene.resetCamera()
    }

    onHeightChanged: scene.setViewHeight(height)
    Component.onCompleted: scene.setViewHeight(height)

    // Game-plugin HUD: generic renderer of HudWidgetDescriptors.
    Item {
        anchors.fill: parent
        anchors.margins: 12
        visible: options.showHud && scene.robotVisible

        HudStack { anchorIndex: 0; hAlign: 0; anchors.left: parent.left; anchors.top: parent.top }
        HudStack { anchorIndex: 1; hAlign: 1; anchors.horizontalCenter: parent.horizontalCenter; anchors.top: parent.top }
        HudStack { anchorIndex: 2; hAlign: 2; anchors.right: parent.right; anchors.top: parent.top }
        HudStack { anchorIndex: 3; hAlign: 0; anchors.left: parent.left; anchors.bottom: parent.bottom }
        HudStack { anchorIndex: 4; hAlign: 1; anchors.horizontalCenter: parent.horizontalCenter; anchors.bottom: parent.bottom }
        HudStack { anchorIndex: 5; hAlign: 2; anchors.right: parent.right; anchors.bottom: parent.bottom; anchors.bottomMargin: 54 }
        HudStack { anchorIndex: 6; hAlign: 1; anchors.centerIn: parent }
    }
}
