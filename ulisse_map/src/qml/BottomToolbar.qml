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

        Column {
            id: controllerAndRobotIndicators
            Layout.rightMargin: 5
            spacing: 6
            Rectangle {
                width: controllerEnabled.contentWidth + 6
                height: controllerEnabled.contentHeight + 3
                border.color: fbkUpdater ? fbkUpdater.control_alive ? "green" : "red" : ""
                border.width: 1
                radius: 5

                Text {
                    id: controllerEnabled
                    verticalAlignment: Text.AlignVCenter
                    horizontalAlignment: Text.AlignHCenter
                    anchors.fill:parent
                    font.pointSize: 8
                    color: fbkUpdater ? fbkUpdater.control_alive ? "green" : "red" : ""
                    text:  fbkUpdater ? fbkUpdater.control_alive ? qsTr("Controller On") : qsTr("Controller Off") : ""
                }
            }

            Rectangle {
                width: vehicleOnline.contentWidth + 6
                height: vehicleOnline.contentHeight + 3
                border.color: fbkUpdater ? fbkUpdater.vehicle_alive ? "green" : "red" : ""
                border.width: 1
                radius: 5

                Text {
                    id: vehicleOnline
                    verticalAlignment: Text.AlignVCenter
                    horizontalAlignment: Text.AlignHCenter
                    anchors.fill:parent
                    font.pointSize: 8
                    color: fbkUpdater ? fbkUpdater.vehicle_alive ? "green" : "red" : ""
                    text:  fbkUpdater ? fbkUpdater.vehicle_alive ? qsTr("Vehicle Online") : qsTr("Vehicle Offline") : ""
                }
            }
        }

        Column {
            id: rcAndThrustersndicators
            Layout.rightMargin: 5
            spacing: 6
            Rectangle {
                width: rc_enabled.contentWidth + 6
                height: rc_enabled.contentHeight + 3
                border.color: fbkUpdater ? fbkUpdater.radio_controller_enabled ? "red" : "green" : ""
                border.width: 1
                radius: 5

                Text {
                    id: rc_enabled
                    verticalAlignment: Text.AlignVCenter
                    horizontalAlignment: Text.AlignHCenter
                    anchors.fill:parent
                    font.pointSize: 8
                    color: fbkUpdater ? fbkUpdater.radio_controller_enabled ? "red" : "green" : ""
                    text:  fbkUpdater ? fbkUpdater.radio_controller_enabled ? qsTr("Radio Controller On") : qsTr("Radio Controller Off") : ""
                }
            }

            Rectangle {
                width: thrusters_enabled.contentWidth + 6
                height: thrusters_enabled.contentHeight + 3
                border.color: fbkUpdater ? fbkUpdater.thruster_ref_enabled ? "green" : "red" : ""
                border.width: 1
                radius: 5

                Text {
                    id: thrusters_enabled
                    verticalAlignment: Text.AlignVCenter
                    horizontalAlignment: Text.AlignHCenter
                    anchors.fill:parent
                    font.pointSize: 8
                    color: fbkUpdater ? fbkUpdater.thruster_ref_enabled ? "green" : "red" : ""
                    text:  fbkUpdater ? fbkUpdater.thruster_ref_enabled ? qsTr("Motor Ref. Enabled") : qsTr("Motor Ref. Disabled") : ""
                }
            }
        }

        Button {
            id: recenterButton
            text: "Recenter"
            font.pointSize: 9
            highlighted: true
            //Material.background: grey
            Layout.leftMargin: 10
        }

        Button {
            id: clearPathButton
            text: "Clear trace"
            font.pointSize: 9
            highlighted: true
            //Material.accent: grey
            Layout.alignment: Qt.AlignLeft
        }

        CheckBox {
            id: followMeCheckbox
            text: "Follow vehicle"
            //Material.accent: grey
            checked: false
        }

        CheckBox {
            id: gpsIconCBox
            text: "Show GPS"
            //Material.accent: grey
            checked: false
        }

        // Spacer item
        Item {
            Layout.fillWidth: true
            Layout.fillHeight: true
            //Rectangle { anchors.fill: parent; color: "#ffaaaa" } // to visualize the spacer
        }

        Button {
            id: enableRefButton
            anchors.rightMargin: parent.anchors.rightMargin
            //Layout.rightMargin: 5
            text: "Enable Ref"
            highlighted: true
            Material.accent: mainColor
            Layout.alignment: Qt.AlignRight
            Layout.rightMargin: 10
            enabled: fbkUpdater ? !fbkUpdater.thruster_ref_enabled : false
        }

        Button {
            id: enginePowerButton
            //anchors.rightMargin: parent.anchors.rightMargin
            //Layout.rightMargin: 5
            text: "Engines"
            highlighted: true
            Material.accent: red
            Layout.alignment: Qt.AlignRight
            Layout.rightMargin: 15
        }
    }
}

