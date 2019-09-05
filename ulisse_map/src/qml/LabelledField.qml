import QtQuick 2.11
import QtQuick.Controls 2.4
import QtQuick.Controls.Material 2.4
import QtQuick.Controls.Universal 2.4
import QtQuick.Dialogs 1.2
import QtQuick.Layouts 1.11
import QtQuick.Window 2.4
import QtLocation 5.11
import QtPositioning 5.11
import Qt.labs.settings 1.0
import QtGraphicalEffects 1.0
import "."

GroupBox {
    id: root
    y: -10
    width: 200
    height: root.contentHeight
    //font.capitalization: Font.AllUppercase
    clip: true

    label: Label {
        x: root.leftPadding
        y: 25
        width: root.availableWidth
        text: root.title
        horizontalAlignment: Text.AlignHCenter
        elide: Text.ElideRight

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
