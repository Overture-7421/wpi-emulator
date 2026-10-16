import QtQuick

// One HUD widget (HudWidgetDescriptor evaluated by HudModel). kind: 0 Counter, 1 Meter, 2 Gauge,
// 3 Text, 4 Badge, 5 Timer.
Rectangle {
    id: card

    required property string title
    required property int kind
    required property bool valid
    required property bool active
    required property real fraction
    required property string text
    required property string accent

    readonly property color tint: accent !== "" ? accent : theme.accent

    implicitWidth: Math.max(120, content.implicitWidth + 24)
    implicitHeight: content.implicitHeight + 18
    radius: 10
    color: theme.panel
    border.color: theme.panelBorder
    border.width: 1
    opacity: valid ? 1.0 : 0.65
    // Values the game does not publish right now (e.g. match timing outside Practice) hide the widget.
    visible: valid

    Column {
        id: content
        anchors.centerIn: parent
        spacing: 4

        Text {
            text: card.title
            color: theme.mutedText
            font.pixelSize: 11
            font.capitalization: Font.AllUppercase
            font.letterSpacing: 0.8
        }

        Text {
            visible: card.kind === 0 || card.kind === 3 || card.kind === 5
            text: card.text
            color: theme.text
            font.pixelSize: card.kind === 5 ? 28 : 20
            font.bold: true
        }

        Item {
            visible: card.kind === 1
            width: Math.max(150, valueText.implicitWidth)
            height: valueText.implicitHeight + 6 + 8
            Text {
                id: valueText
                text: card.text
                color: theme.text
                font.pixelSize: 16
                font.bold: true
            }
            Rectangle {
                y: valueText.implicitHeight + 6
                width: parent.width
                height: 8
                radius: 4
                color: theme.panelBorder
                Rectangle {
                    width: parent.width * card.fraction
                    height: parent.height
                    radius: 4
                    color: card.tint
                    Behavior on width { NumberAnimation { duration: 120 } }
                }
            }
        }

        Item {
            visible: card.kind === 2
            width: 84
            height: 62
            Canvas {
                id: arc
                anchors.fill: parent
                property real value: card.fraction
                onValueChanged: requestPaint()
                onPaint: {
                    const ctx = getContext("2d")
                    ctx.reset()
                    const cx = width / 2, cy = height - 8, r = 34
                    const start = Math.PI * 0.85, end = Math.PI * 2.15
                    ctx.lineWidth = 8
                    ctx.lineCap = "round"
                    ctx.strokeStyle = theme.panelBorder
                    ctx.beginPath(); ctx.arc(cx, cy, r, start, end); ctx.stroke()
                    ctx.strokeStyle = card.tint
                    ctx.beginPath(); ctx.arc(cx, cy, r, start, start + (end - start) * value); ctx.stroke()
                }
            }
            Text {
                anchors.horizontalCenter: parent.horizontalCenter
                anchors.bottom: parent.bottom
                anchors.bottomMargin: 6
                text: card.text
                color: theme.text
                font.pixelSize: 14
                font.bold: true
            }
        }

        Row {
            visible: card.kind === 4
            spacing: 8
            Rectangle {
                width: 12; height: 12; radius: 6
                anchors.verticalCenter: parent.verticalCenter
                color: card.active ? theme.success : theme.mutedText
            }
            Text {
                text: card.text
                color: card.active ? theme.text : theme.mutedText
                font.pixelSize: 16
                font.bold: true
            }
        }
    }
}
