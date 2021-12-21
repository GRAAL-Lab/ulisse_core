import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.1
import "."

Rectangle {
    property alias markerText: markerTextLabel.text
    property alias markerTextColor: markerTextLabel.textColor
    property alias pathCmdPane: commandRect.pathCmdPane
    property alias commandRect: commandRect
    property alias speedHeadTimeout: commandRect.speedHeadTimeout

    visible: true

    ColumnLayout {
        id: leftbarlayout
        width: parent.width
        spacing: 0

        Pane {
            id: statusdatarect
            Layout.alignment: Qt.AlignHCenter | Qt.AlignTop
            Layout.fillWidth: true
            Layout.fillHeight: false
            Material.elevation: 0
            //Material.background: Material.color(Material.BlueGrey, Material.Shade50)

            ColumnLayout {
                Layout.alignment: Qt.AlignLeft
                width: parent.width

                Label {
                    font.pointSize: 12
                    font.weight: Font.DemiBold
                    color: mainColor
                    text: "Status"
                    verticalAlignment: Text.AlignVCenter
                    horizontalAlignment: Text.AlignHCenter
                    Layout.fillWidth: true
                    Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                }

                LabelledText {
                    id: ulisseStateLabel
                    //h:12
                    //w:100
                    labelColor: blue
                    label: "State"
                    text: fbkUpdater.vehicle_state
                    Layout.fillWidth: true
                    textBoldness: Font.DemiBold
                }

                LabelledText {
                    id: ulissePosLabel
                    //h:12
                    //w:100
                    labelColor: blue
                    label: "Position"
                    text: fbkUpdater.ulisse_pos.latitude.toFixed(8) + ", " +
                          fbkUpdater.ulisse_pos.longitude.toFixed(8)
                    Layout.fillWidth: true
                }

                LabelledText {
                    id: markerTextLabel
                    //h:12
                    //w:100
                    labelColor: red
                    label: "Marker"
                    text: "Left click on map"
                    Layout.fillWidth: true
                }

                LabelledText {
                    id: goalDistLabel
                    //h:12
                    //w:100
                    text: fbkUpdater.goal_distance.toFixed(2) + " (m)"
                    objectName: "goalDistance"
                    labelColor: blue
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
            Layout.alignment: Qt.AlignLeft | Qt.AlignTop
            Layout.fillWidth: true
            Layout.fillHeight: false
            //Material.background: "white"//Material.color(Material.BlueGrey, Material.Shade50)
        }
    }
}
