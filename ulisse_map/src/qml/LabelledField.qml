import QtQuick 2.9
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import QtGraphicalEffects 1.0
import QtQuick.Dialogs 1.2
import "."

GroupBox {
    id: root
    //y: -10
    height: root.contentHeight
    clip: true

    label: Label {
        x: root.leftPadding
        y: 15
        width: root.availableWidth
        text: root.title
        horizontalAlignment: Text.AlignHCenter
        elide: Text.ElideRight
        font.weight: Font.DemiBold

        background: Rectangle {
            y: 0
            width: 0
            height: 0
            color: "transparent"
            border.color: "transparent"
        }
    }
    background: Rectangle {
        y: 0
        width: 0
        height: 0
        color: "transparent"
        border.color: "transparent"
    }
}
