import QtQuick
import QtQuick3D

// Thin renderer for the editor viewport: every list, colour and camera value comes from the C++
// ViewportController (`ctl`), and pointer input is forwarded back to it. World content lives under a
// node rotated -90 degrees about X so it can be expressed directly in the SDK's Z-up frame.
Item {
    id: root
    property var ctl: editorCtl
    clip: true

    function hitsAt(x, y) {
        var results = view.pickAll(x, y)
        var ids = []
        for (var i = 0; i < results.length; ++i) {
            var hit = results[i].objectHit
            if (hit && hit.pickId)
                ids.push(hit.pickId)
        }
        return ids
    }

    component DecorModel: Model {
        required property var modelData
        source: modelData.source3d
        position: modelData.pos
        rotation: modelData.rot
        scale: modelData.scl
        pickable: modelData.pickId !== ""
        property string pickId: modelData.pickId
        materials: DefaultMaterial {
            diffuseColor: modelData.tint
            lighting: DefaultMaterial.NoLighting
            opacity: modelData.alpha
        }
    }

    component SceneDelegate: Model {
        required property string itemId
        required property string source3d
        required property vector3d pos
        required property quaternion rot
        required property vector3d scl
        required property color tint
        required property real alpha
        required property bool isPickable
        source: source3d
        position: pos
        rotation: rot
        scale: scl
        pickable: isPickable
        property string pickId: "i:" + itemId
        materials: DefaultMaterial {
            diffuseColor: tint
            specularAmount: 0.15
            opacity: alpha
        }
    }

    View3D {
        id: view
        anchors.fill: parent
        camera: cam
        environment: SceneEnvironment {
            clearColor: root.ctl.backgroundColor
            backgroundMode: SceneEnvironment.Color
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: SceneEnvironment.High
        }

        DirectionalLight {
            eulerRotation: Qt.vector3d(-48, -32, 0)
            brightness: 0.9
            ambientColor: "#505050"
        }
        DirectionalLight {
            eulerRotation: Qt.vector3d(32, 145, 0)
            brightness: 0.4
        }

        Node {
            id: world
            eulerRotation.x: -90

            PerspectiveCamera {
                id: cam
                position: root.ctl.cameraPosition
                rotation: root.ctl.cameraRotation
                fieldOfView: root.ctl.fieldOfView
                clipNear: 0.02
                clipFar: 400
            }
            Repeater3D {
                model: root.ctl.decor
                delegate: DecorModel {}
            }
            Repeater3D {
                model: root.ctl.sceneModel
                delegate: SceneDelegate {}
            }
            Repeater3D {
                model: root.ctl.selectionEdges
                delegate: DecorModel {}
            }
            Repeater3D {
                model: root.ctl.gizmoHandles
                delegate: DecorModel {
                    depthBias: -400
                }
            }
        }
    }

    MouseArea {
        anchors.fill: parent
        acceptedButtons: Qt.LeftButton | Qt.RightButton | Qt.MiddleButton
        onPressed: mouse => root.ctl.press(mouse.x, mouse.y, mouse.button, mouse.modifiers, root.hitsAt(mouse.x, mouse.y))
        onPositionChanged: mouse => root.ctl.move(mouse.x, mouse.y, mouse.buttons, mouse.modifiers)
        onReleased: mouse => root.ctl.release(mouse.x, mouse.y, mouse.button)
        onWheel: wheel => root.ctl.wheel(wheel.angleDelta.y)
    }

    onWidthChanged: ctl.setViewSize(width, height)
    onHeightChanged: ctl.setViewSize(width, height)
    Component.onCompleted: ctl.setViewSize(width, height)

    // AprilTag ids: the controller projects every tag plate to the screen, the renderer only writes the text.
    Repeater {
        model: root.ctl.tagLabels
        delegate: Rectangle {
            id: tagLabel
            required property var modelData
            x: modelData.x - width / 2
            y: modelData.y - height / 2
            width: tagText.implicitWidth + 8
            height: tagText.implicitHeight + 2
            radius: 3
            color: Qt.rgba(root.ctl.backgroundColor.r, root.ctl.backgroundColor.g, root.ctl.backgroundColor.b, 0.72)
            border.color: modelData.color
            border.width: 1
            Text {
                id: tagText
                anchors.centerIn: parent
                text: tagLabel.modelData.text
                color: tagLabel.modelData.color
                font.pixelSize: 11
                font.bold: true
            }
        }
    }

    // Orientation widget: how the field axes (X forward, Y left, Z up) project onto the screen.
    Item {
        id: axes
        x: 14
        y: root.height - 74
        width: 60
        height: 60
        Repeater {
            model: root.ctl.axisIndicator
            delegate: Item {
                id: axis
                required property var modelData
                readonly property real reach: 22
                Rectangle {
                    x: 30
                    y: 29
                    width: Math.max(2, Math.hypot(axis.modelData.x, axis.modelData.y) * axis.reach)
                    height: 2
                    radius: 1
                    color: axis.modelData.color
                    opacity: axis.modelData.depth > 0 ? 0.45 : 1
                    transformOrigin: Item.Left
                    rotation: -Math.atan2(axis.modelData.y, axis.modelData.x) * 180 / Math.PI
                }
                Text {
                    x: 30 + axis.modelData.x * (axis.reach + 8) - width / 2
                    y: 30 - axis.modelData.y * (axis.reach + 8) - height / 2
                    text: axis.modelData.label
                    color: axis.modelData.color
                    font.bold: true
                    font.pixelSize: 11
                }
            }
        }
    }

    Text {
        anchors.left: parent.left
        anchors.leftMargin: 80
        anchors.bottom: parent.bottom
        anchors.bottomMargin: 14
        text: qsTr("X forward \u00b7 Y left \u00b7 Z up   grid 0.5 m")
        color: root.ctl.textColor
        opacity: 0.55
        font.pixelSize: 11
    }

    Rectangle {
        visible: root.ctl.hintText.length > 0
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: parent.top
        anchors.topMargin: 18
        width: Math.min(parent.width - 40, hintLabel.implicitWidth + 28)
        height: hintLabel.implicitHeight + 18
        radius: 8
        color: Qt.rgba(root.ctl.accentColor.r, root.ctl.accentColor.g, root.ctl.accentColor.b, 0.18)
        border.color: root.ctl.accentColor
        Text {
            id: hintLabel
            anchors.fill: parent
            anchors.margins: 9
            text: root.ctl.hintText
            color: root.ctl.textColor
            wrapMode: Text.WordWrap
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            font.pixelSize: 13
        }
    }
}
