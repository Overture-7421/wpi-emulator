import QtQuick
import QtQuick3D

// One camera of the vision overlay (roles of CameraListModel): a small body marker and the translucent viewing pyramid, both in the
// camera frame (+X = the optical axis). The poses are SDK-frame; the `world` node above converts them to Qt Quick 3D's Y-up space.
Node {
    id: camera

    required property vector3d camPosition
    required property quaternion camRotation
    required property vector3d bodyScale
    required property color bodyColor
    required property color frustumColor
    required property var frustumGeometry

    position: camPosition
    rotation: camRotation

    Model {
        source: "#Cube"
        scale: camera.bodyScale
        materials: PrincipledMaterial {
            baseColor: camera.bodyColor
            roughness: 0.5
            metalness: 0.1
            alphaMode: camera.bodyColor.a < 1.0 ? PrincipledMaterial.Blend : PrincipledMaterial.Opaque
        }
    }

    Model {
        geometry: camera.frustumGeometry
        materials: PrincipledMaterial {
            lighting: PrincipledMaterial.NoLighting
            baseColor: camera.frustumColor
            alphaMode: PrincipledMaterial.Blend
            cullMode: Material.NoCulling
        }
    }
}
