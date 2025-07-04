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
    property alias savePathDialog: commandRect.savePathDialog
    property alias sliderHeading: commandRect.sliderHeading

    visible: true

    ColumnLayout {
        id: leftbarlayout
        anchors.fill: parent
        spacing: 0

        Pane {
            id: statusdatarect

            Layout.topMargin: 10
            Layout.alignment: Qt.AlignHCenter
            Layout.preferredWidth: parent.width - 20
            //Layout.fillWidth: true
            //Layout.fillHeight: false
            Material.elevation: 3
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
                    label: "Vehicle State"
                    text: fbkUpdater !== null ? fbkUpdater.vehicle_state : "Undefined"
                    Layout.fillWidth: true
                    textBoldness: Font.DemiBold
                }

                LabelledText {
                    id: ulissePosLabel
                    //h:12
                    //w:100
                    labelColor: blue
                    label: "Vehicle Position"
                    text: fbkUpdater !== null ? fbkUpdater.ulisse_pos.latitude.toFixed(7) + ", " +
                          fbkUpdater.ulisse_pos.longitude.toFixed(7) : " "
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
                    text: fbkUpdater !== null ? fbkUpdater.goal_distance.toFixed(2) + " (m)" : " "
                    objectName: "goalDistance"
                    labelColor: blue
                    label: "Target Distance"
                    Layout.fillWidth: true
                }

                //Text {
                //    id: markerText
                //    font.pointSize: 8
                //    color: 'grey'
                //    text: "(Left click to set marker)"
                //    verticalAlignment: Text.AlignVCenter
                //    Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                //    Layout.fillWidth: true
                //    horizontalAlignment: Text.AlignHCenter
                //}
            }
        }

        CommandPane {
            id: commandRect
            Layout.alignment: Qt.AlignLeft | Qt.AlignTop
            Layout.fillWidth: true
            Layout.fillHeight: true

            Layout.margins: 10
            //Material.background: "white"//Material.color(Material.BlueGrey, Material.Shade50)
        }
    }
}
