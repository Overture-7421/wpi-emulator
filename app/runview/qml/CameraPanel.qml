import QtQuick

// The vision cards (CameraHudModel): the problems the runtime raised first, then one card per configured camera with its state, capture
// rate, latency, the tags in view and what the robot code asked the camera for.
Column {
    id: panel

    readonly property int count: notices.count + cards.count
    spacing: 8

    Repeater {
        id: notices
        model: cameraHud.notices
        delegate: Rectangle {
            id: notice

            required property string title
            required property string detail
            required property color tone

            width: 230
            height: noticeBody.implicitHeight + 18
            radius: 10
            color: theme.panel
            border.color: notice.tone
            border.width: 1

            Column {
                id: noticeBody
                x: 12
                y: 9
                width: parent.width - 24
                spacing: 3
                Text {
                    width: parent.width
                    text: notice.title
                    color: notice.tone
                    font.pixelSize: 13
                    font.bold: true
                    wrapMode: Text.WordWrap
                }
                Text {
                    width: parent.width
                    visible: text.length > 0
                    text: notice.detail
                    color: theme.mutedText
                    font.pixelSize: 11
                    wrapMode: Text.WordWrap
                }
            }
        }
    }

    Repeater {
        id: cards
        model: cameraHud
        delegate: Rectangle {
            id: card

            required property string label
            required property string vendor
            required property string stateText
            required property color tone
            required property string metrics
            required property string tags
            required property string request
            required property bool active

            width: 230
            height: body.implicitHeight + 18
            radius: 10
            color: theme.panel
            border.color: theme.panelBorder
            border.width: 1

            Column {
                id: body
                x: 12
                y: 9
                width: parent.width - 24
                spacing: 5

                Item {
                    width: parent.width
                    height: name.implicitHeight
                    Rectangle {
                        id: dot
                        width: 9
                        height: 9
                        radius: 5
                        anchors.verticalCenter: parent.verticalCenter
                        color: card.tone
                    }
                    Text {
                        id: name
                        anchors.left: dot.right
                        anchors.leftMargin: 7
                        anchors.right: kind.left
                        anchors.rightMargin: 6
                        text: card.label
                        color: theme.text
                        font.pixelSize: 13
                        font.bold: true
                        elide: Text.ElideRight
                    }
                    Text {
                        id: kind
                        anchors.right: parent.right
                        anchors.verticalCenter: parent.verticalCenter
                        text: card.vendor
                        color: theme.mutedText
                        font.pixelSize: 10
                    }
                }

                // The chip and the measured rate share a row while there is room.
                Flow {
                    width: parent.width
                    spacing: 8
                    Rectangle {
                        width: Math.min(parent.width, chip.implicitWidth + 16)
                        height: chip.implicitHeight + 4
                        radius: height / 2
                        color: Qt.rgba(card.tone.r, card.tone.g, card.tone.b, 0.18)
                        border.color: Qt.rgba(card.tone.r, card.tone.g, card.tone.b, 0.6)
                        border.width: 1
                        Text {
                            id: chip
                            anchors.centerIn: parent
                            width: parent.width - 16
                            text: card.stateText
                            color: card.tone
                            font.pixelSize: 11
                            font.bold: true
                            elide: Text.ElideRight
                        }
                    }
                    Text {
                        height: chip.implicitHeight + 4
                        verticalAlignment: Text.AlignVCenter
                        text: card.metrics
                        color: theme.text
                        font.pixelSize: 12
                    }
                }

                Text {
                    width: parent.width
                    text: card.tags
                    color: card.active ? theme.text : theme.mutedText
                    font.pixelSize: 12
                    font.bold: card.active
                    wrapMode: Text.WordWrap
                }

                Text {
                    width: parent.width
                    visible: card.request.length > 0
                    text: card.request
                    color: theme.mutedText
                    font.pixelSize: 11
                    wrapMode: Text.WordWrap
                }
            }
        }
    }
}
