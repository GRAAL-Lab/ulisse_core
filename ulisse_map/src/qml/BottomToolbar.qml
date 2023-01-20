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
    //property alias overlayStatusCbox: overlayStatusCbox
    property alias gpsIconCBox: gpsIconCBox
    property alias enableRefButton: enableRefButton
    property alias enginePowerButton: enginePowerButton

    RowLayout {
        anchors.fill: parent
        width: parent.width
        height: parent.height - recenterButton.height
        spacing: 8

        Button {
            id: recenterButton
            text: "Recenter"
            font.pointSize: 9
            highlighted: true
            Material.background: grey
            Layout.leftMargin: 10
        }

        Button {
            id: clearPathButton
            text: "Clear trace"
            font.pointSize: 9
            highlighted: true
            Material.accent: grey
            Layout.alignment: Qt.AlignLeft
        }

        CheckBox {
            id: followMeCheckbox
            text: "Follow vehicle"
            //Layout.alignment: Qt.AlignLeft
            Material.accent: grey
            checked: false
        }

        CheckBox {
            id: gpsIconCBox
            text: "Show GPS"
            Material.accent: grey
            checked: false
        }

        Rectangle {
            id: rectangle
            width: 200
            height: 200
            color: dimmedwhite
            Layout.fillWidth: true
            Layout.fillHeight: true
        }

        Button {
            id: enableRefButton
            anchors.rightMargin: parent.anchors.rightMargin
            //Layout.rightMargin: 5
            text: "Enable Ref"
            highlighted: true
            Material.accent: mainColor
            Layout.alignment: Qt.AlignRight
            Layout.rightMargin: 20
        }

        Button {
            id: enginePowerButton
            anchors.rightMargin: parent.anchors.rightMargin
            //Layout.rightMargin: 5
            text: "Engines"
            highlighted: true
            Material.accent: red
            Layout.alignment: Qt.AlignRight
            Layout.rightMargin: 20
        }
    }
}

