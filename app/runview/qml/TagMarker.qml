import QtQuick
import QtQuick3D

// One AprilTag image: a quad on the tag's face plane (roles of TagMarkerModel). #Rectangle is 100 x 100 units.
Model {
    required property vector3d markerPosition
    required property quaternion markerRotation
    required property real markerSize
    required property url markerImage

    source: "#Rectangle"
    position: markerPosition
    rotation: markerRotation
    scale: Qt.vector3d(markerSize / 100, markerSize / 100, 1)
    materials: PrincipledMaterial {
        alphaMode: PrincipledMaterial.Opaque
        roughness: 0.9
        metalness: 0.0
        baseColorMap: Texture {
            source: markerImage
            magFilter: Texture.Nearest
            minFilter: Texture.Linear
            mipFilter: Texture.Linear
            generateMipmaps: true
        }
    }
}
