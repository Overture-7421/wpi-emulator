import QtQuick3D

// The root of everything a viewport draws from SDK data. Qt Quick 3D is Y-up; config, physics and the C++ scene
// models are Z-up NWU, so the children of this node are expressed in SDK coordinates and its -90 degree rotation
// about X is the one place that converts them. Lights and other Y-up content belong outside it.
Node {
    eulerRotation.x: -90
}
