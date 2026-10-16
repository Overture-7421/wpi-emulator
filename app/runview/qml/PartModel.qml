import QtQuick
import QtQuick3D

// One primitive of the robot / field / overlay lists (roles of PartListModel).
Model {
    required property int shape
    required property vector3d partPosition
    required property quaternion partRotation
    required property vector3d partScale
    required property color partColor
    required property bool shown

    visible: shown
    source: shape === 1 ? "#Cylinder" : "#Cube"
    position: partPosition
    rotation: partRotation
    scale: partScale
    materials: PrincipledMaterial {
        baseColor: partColor
        roughness: 0.55
        metalness: 0.05
        alphaMode: partColor.a < 1.0 ? PrincipledMaterial.Blend : PrincipledMaterial.Opaque
    }
}
