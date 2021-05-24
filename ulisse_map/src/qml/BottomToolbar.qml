import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import "."

Rectangle {

    property alias recenterButton: recenterButton
    property alias clearPathButton: clearPathButton
    property alias followMeCheckbox: followMeCheckbox
    property alias overlayStatusCbox: overlayStatusCbox
    property alias gpsIconCBox: gpsIconCBox
    property alias engineButton: engineButton

    RowLayout {
        anchors.fill: parent
        width: parent.width
        height: parent.height - recenterButton.height
        spacing: 8

        Button {
            id: recenterButton
            text: "Recenter"
            highlighted: true
            Material.background: mainColor
            Layout.leftMargin: 10
        }

        Button {
            id: clearPathButton
            text: "Clear trace"
            highlighted: true
            Material.accent: orange
            Layout.alignment: Qt.AlignLeft
        }

        CheckBox {
            id: followMeCheckbox
            text: "Follow vehicle"
            //Layout.alignment: Qt.AlignLeft
            //Material.accent: mainColor
            checked: false
        }

        CheckBox {
            id: overlayStatusCbox
            text: "Show Overlay"
            //Material.accent: orange
            checked: true
        }

        CheckBox {
            id: gpsIconCBox
            text: "Show GPS"
            checked: false
        }

        Rectangle {
            id: rectangle
            width: 200
            height: 200
            color: "#ffffff"
            Layout.fillWidth: true
            Layout.fillHeight: true
        }

        Button {
            id: engineButton
            anchors.rightMargin: parent.anchors.rightMargin
            //Layout.rightMargin: 5
            text: "engine"
            highlighted: true
            Material.accent: red
            Layout.alignment: Qt.AlignRight
            Layout.rightMargin: 20
        }
    }
}

