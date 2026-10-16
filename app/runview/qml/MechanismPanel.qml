import QtQuick

// Live mechanism state cards (positions / velocities published by the mechanisms).
Column {
    id: panel
    readonly property alias count: repeater.count
    spacing: 8

    Repeater {
        id: repeater
        model: mechanisms
        delegate: Rectangle {
            required property string label
            required property bool active
            required property string values

            width: Math.max(170, body.implicitWidth + 24)
            height: body.implicitHeight + 18
            radius: 10
            color: theme.panel
            border.color: theme.panelBorder
            border.width: 1

            Column {
                id: body
                anchors.centerIn: parent
                spacing: 4
                Row {
                    spacing: 7
                    Rectangle {
                        width: 9; height: 9; radius: 5
                        anchors.verticalCenter: parent.verticalCenter
                        color: active ? theme.success : theme.mutedText
                    }
                    Text {
                        text: label
                        color: theme.text
                        font.pixelSize: 13
                        font.bold: true
                    }
                }
                Text {
                    visible: values.length > 0
                    text: values
                    color: theme.mutedText
                    font.family: "Consolas, Menlo, monospace"
                    font.pixelSize: 11
                    lineHeight: 1.1
                }
            }
        }
    }
}
