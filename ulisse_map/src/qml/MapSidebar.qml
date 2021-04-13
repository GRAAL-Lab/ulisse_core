import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.1
import "."

Rectangle {
    property alias markerText: markerTextLabel.text
    property alias markerTextColor: markerTextLabel.textColor
    property alias sweepPathCmdPane: commandRect.sweepPathCmdPane
    property color labelColor1: blue
    property color labelColor2: red
    property color labelColor3: green
    property alias commandRect: commandRect
    property alias speedHeadTimeout: commandRect.speedHeadTimeout
    //border.width: 5
    //border.color: lightgrey
    visible: true

    ColumnLayout {
        id: leftbarlayout
        width: parent.width
        spacing: 2

        Pane {
            id: statusdatarect
            Layout.alignment: Qt.AlignHCenter | Qt.AlignTop
            Layout.fillWidth: true
            Layout.fillHeight: false
            Material.elevation: 0

            ColumnLayout {
                Layout.alignment: Qt.AlignLeft

                Label {
                    id: my_label
                    font.pointSize: 12
                    font.weight: Font.DemiBold
                    color: labelColor1
                    text: "Status"
                    verticalAlignment: Text.AlignVCenter
                    horizontalAlignment: Text.AlignHCenter
                    Layout.fillWidth: true
                    Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                }

                LabelledText {
                    id: ulisseStateLabel
                    h:12
                    w:100
                    labelColor: labelColor1
                    label: "State"
                    text: fbkUpdater.vehicle_state
                    Layout.fillWidth: true
                    textBoldness: Font.DemiBold
                }

                LabelledText {
                    id: ulissePosLabel
                    h:12
                    w:100
                    labelColor: labelColor1
                    label: "Position"
                    text: fbkUpdater.ulisse_pos.latitude.toFixed(8) + ", " +
                          fbkUpdater.ulisse_pos.longitude.toFixed(8)
                    Layout.fillWidth: true
                }

                LabelledText {
                    id: markerTextLabel
                    h:12
                    w:100
                    labelColor: labelColor2
                    label: "Marker"
                    text: "Left click on map"
                    Layout.fillWidth: true
                }

                LabelledText {
                    id: goalDistLabel
                    h:12
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
            //antialiasing: true
            Layout.alignment: Qt.AlignLeft | Qt.AlignTop
            Layout.fillWidth: true
            Layout.fillHeight: false
            Material.background: Material.color(Material.BlueGrey, Material.Shade50)
        }
    }
}
