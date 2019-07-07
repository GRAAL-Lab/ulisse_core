import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.2
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import QtQuick.Dialogs 1.2

RowLayout {
    property alias waypointsButton: waypointsButton
    property alias wpRestartButton: wpRestartButton
    property alias wpDeleteButton: wpDeleteButton
    property alias squaredrawButton: squaredrawButton
    id: my_waypoint_ctrl
    //width: parent.width
    property alias wpRadius: waypointRadius.text
    property alias wpButtonText: waypointsButton.text
    property alias wpButtonHighlighted: waypointsButton.highlighted
    property bool loopPath: false
    state: {
        0: "empty",
                1: "creating",
                2: "active",
                3: "stopped"
    }[mapView.pathCurrentState]

    Button {
        id: waypointsButton
        Layout.preferredWidth: 90
        Layout.alignment: Qt.AlignLeft
        Layout.fillWidth: true
        highlighted: true
        Material.accent: secondaryAccentColor
        enabled: true
    }

    Button {
        id: wpRestartButton
        Material.accent: secondaryAccentColor
        Layout.preferredWidth: 30
        Layout.minimumWidth: 30
        Layout.maximumWidth: 30

        Layout.alignment: Qt.AlignLeft

        ToolTip.text: qsTr("Restart path from beginning")
        ToolTip.delay: 500
        ToolTip.timeout: 5000
        ToolTip.visible: hovered
        enabled: true

        Image {
            id: restartIco
            anchors.fill: parent
            source: wpRestartButton.enabled ? "qrc:/images/restart-256.png" : "qrc:/images/restart-256_grey.png"
            mipmap: true
            fillMode: Image.PreserveAspectFit
            horizontalAlignment: Image.AlignHCenter
            verticalAlignment: Image.AlignVCenter
        }
    }

    Button {
        id: wpDeleteButton
        Material.accent: Material.Red
        highlighted: true
        Layout.preferredWidth: 30
        Layout.minimumWidth: 30
        Layout.maximumWidth: 30
        text: "X"
        font.weight: Font.Bold
        font.pointSize: 12

        ToolTip.text: qsTr("Delete the current path and stop")
        ToolTip.delay: 500
        ToolTip.timeout: 5000
        ToolTip.visible: hovered
        enabled: true
    }

    TextField {
        id: waypointRadius
        objectName: "waypointRadius"
        Layout.preferredWidth: 45
        Layout.minimumWidth: 45
        Layout.maximumWidth: 45
        font.pointSize: 10
        placeholderText: "Radius"
        selectByMouse: true
        text: "5.0"
        enabled: true

        ToolTip.text: qsTr("Acceptance Radius (meters)")
        ToolTip.delay: 500
        ToolTip.timeout: 5000
        ToolTip.visible: hovered

        validator: DoubleValidator {
            bottom: 0.0
            top: 101.0
            decimals: 3
            notation: DoubleValidator.StandardNotation
        }
    }
    Button {
        id: squaredrawButton
        Layout.preferredWidth: 90
        Layout.alignment: Qt.AlignRight
        Layout.fillWidth: true
        highlighted: false
        Material.accent: secondaryAccentColor
        enabled: true
    }

    states: [
        State {
            name: "empty"
            PropertyChanges {
                target: waypointsButton
                text: "Create Path"
                highlighted: false
            }
            PropertyChanges {
                target: squaredrawButton
                text: "Draw Square"
            }
            PropertyChanges {
                target: wpRestartButton
                enabled: false
            }
            PropertyChanges {
                target: wpDeleteButton
                enabled: false
            }
        },
        State {
            name: "creating"
            PropertyChanges {
                target: waypointsButton
                text: "Send Path"
            }
            PropertyChanges {
                target: wpRestartButton
                enabled: false
            }
        },
        State {
            name: "active"
            PropertyChanges {
                target: waypointsButton
                Material.accent: mainAccentColor
                text: "Pause path"
            }
            PropertyChanges {
                target: waypointRadius
                enabled: false
            }
        },
        State {
            name: "stopped"
            PropertyChanges {
                target: waypointsButton
                Material.accent: mainAccentColor
                text: "Resume path"
            }
        },
        State {
            name: "creatingsquare"
            PropertyChanges {
                target: waypointsButton
                enabled: false
            }
        }
    ]
}
