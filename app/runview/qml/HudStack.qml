import QtQuick

// All HUD widgets of one anchor position. hAlign: 0 left, 1 centre, 2 right.
Column {
    id: stack

    property int anchorIndex: 0
    property int hAlign: 0

    spacing: 8

    Repeater {
        model: hud.anchorModel(stack.anchorIndex)
        delegate: HudWidget {
            x: stack.hAlign === 1 ? (stack.width - width) / 2 : (stack.hAlign === 2 ? stack.width - width : 0)
        }
    }
}
