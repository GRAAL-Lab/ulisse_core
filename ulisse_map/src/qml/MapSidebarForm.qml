import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.1
import "."

Rectangle {
    property alias markerText: markerTextLabel.text
    property alias markerTextColor: markerTextLabel.textColor
    property color labelColor1: "#000000"
    property color labelColor2: "#000000"
    property color labelColor3: "#000000"
    property alias commandRect: commandRect
    property alias speedHeadTimeout: commandRect.speedHeadTimeout
    border.width: 5
    border.color: lightgrey
    visible: true

    ColumnLayout {
        id: leftbarlayout
        anchors.fill: parent
        Layout.leftMargin: 15
        width: parent.width
        spacing: 0

        Pane {
            id: statusdatarect
            height: 120
            clip: false
            Layout.alignment: Qt.AlignHCenter | Qt.AlignTop
            Layout.fillWidth: true
            Layout.fillHeight: false

            ColumnLayout {
                id: mycol
                anchors.top: parent.top
                anchors.topMargin: 0
                anchors.right: parent.right
                anchors.rightMargin: 0
                anchors.left: parent.left
                anchors.leftMargin: 0
                Layout.alignment: Qt.AlignLeft

                Label {
                    id: my_label
                    font.pointSize: 12
                    font.weight: Font.DemiBold
                    color: labelColor3
                    text: "Status"
                    verticalAlignment: Text.AlignVCenter
                    horizontalAlignment: Text.AlignHCenter
                    Layout.fillWidth: true
                    Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                }

                LabelledText {
                    id: ulisseStateLabel
                    h:10
                    w:100
                    labelColor: labelColor1
                    label: "State"
                    text: fbkUpdater.vehicle_state
                    Layout.fillWidth: true
                    textBoldness: Font.DemiBold
                }

                LabelledText {
                    id: ulissePosLabel
                    h:10
                    w:100
                    labelColor: labelColor1
                    label: "Position"
                    text: fbkUpdater.ulisse_pos.latitude.toFixed(
                              8) + ", " + fbkUpdater.ulisse_pos.longitude.toFixed(
                              8)
                    Layout.fillWidth: true
                }

                LabelledText {
                    id: markerTextLabel
                    h:10
                    w:100
                    labelColor: labelColor2
                    label: "Target"
                    text: "Left click on map"
                    Layout.fillWidth: true
                }

                LabelledText {
                    id: goalDistLabel
                    h:10
                    w:100
                    text: fbkUpdater.goal_distance.toFixed(2) + " (m)"
                    objectName: "goalDistance"
                    labelColor: labelColor1
                    label: "Distance"
                    Layout.fillWidth: true
                }

                Text {
                    id: markerText
                    font.pointSize: 8
                    color: 'grey'
                    text: "(Left click to set marker)"
                    verticalAlignment: Text.AlignVCenter
                    Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                    Layout.fillWidth: true
                    horizontalAlignment: Text.AlignHCenter
                }
            }
        }

        CommandPane {
            id: commandRect
            antialiasing: true
            Layout.alignment: Qt.AlignLeft | Qt.AlignTop
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.bottomMargin: 2
            //Material.background: Material.color(Material.BlueGrey, Material.Shade50)
        }
    }
}
