import QtQuick

// Two competitors side by side (HudWidgetKind::Scoreboard, HudModel roles): sides[0] left, sides[1] right, the
// leading side highlighted, a caption and the lead margin in the centre.
Row {
    id: board

    required property var sides
    required property string caption
    required property string lead
    required property int leader

    spacing: 10

    component SideCard: Rectangle {
        id: card
        property var info: null
        readonly property color tint: info && info.color !== "" ? info.color : theme.accent
        readonly property bool leading: info ? info.leading : false

        width: 150
        height: sideContent.implicitHeight + 16
        radius: 10
        color: Qt.rgba(tint.r, tint.g, tint.b, leading ? 0.34 : 0.16)
        border.color: leading ? tint : theme.panelBorder
        border.width: leading ? 3 : 1

        Column {
            id: sideContent
            anchors.centerIn: parent
            spacing: 2

            Text {
                anchors.horizontalCenter: parent.horizontalCenter
                text: card.info ? card.info.label : ""
                color: card.tint
                font.pixelSize: 13
                font.bold: true
                font.letterSpacing: 1.5
            }
            Text {
                anchors.horizontalCenter: parent.horizontalCenter
                text: card.info ? card.info.score : ""
                color: theme.text
                font.pixelSize: 34
                font.bold: true
            }
            Row {
                anchors.horizontalCenter: parent.horizontalCenter
                spacing: 6
                visible: card.info !== null && card.info.status !== ""
                Rectangle {
                    width: 10; height: 10; radius: 5
                    anchors.verticalCenter: parent.verticalCenter
                    color: card.info && card.info.statusActive ? theme.success : theme.mutedText
                }
                Text {
                    text: card.info ? card.info.status : ""
                    color: card.info && card.info.statusActive ? theme.text : theme.mutedText
                    font.pixelSize: 13
                    font.bold: true
                }
            }
        }
    }

    SideCard { info: board.sides.length > 0 ? board.sides[0] : null }

    Rectangle {
        anchors.verticalCenter: parent.verticalCenter
        width: Math.max(110, centreContent.implicitWidth + 24)
        height: centreContent.implicitHeight + 16
        radius: 10
        color: theme.panel
        border.color: theme.panelBorder
        border.width: 1

        Column {
            id: centreContent
            anchors.centerIn: parent
            spacing: 4
            Text {
                anchors.horizontalCenter: parent.horizontalCenter
                visible: board.caption !== ""
                text: board.caption
                color: theme.mutedText
                font.pixelSize: 12
                font.bold: true
                font.letterSpacing: 1.0
            }
            Text {
                anchors.horizontalCenter: parent.horizontalCenter
                text: board.lead
                color: board.leader >= 0 && board.sides.length > board.leader && board.sides[board.leader].color !== ""
                       ? board.sides[board.leader].color : theme.text
                font.pixelSize: 18
                font.bold: true
            }
        }
    }

    SideCard { info: board.sides.length > 1 ? board.sides[1] : null }
}
