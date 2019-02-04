import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.1
import "."

Rectangle {

    property alias markerText: markerTextLabel.text
    property alias markerTextColor: markerTextLabel.textColor

    ColumnLayout {
        id: leftbarlayout
        anchors.fill: parent
        spacing: 6
        Layout.leftMargin: 15

        Pane {
            id: statusdatarect
            Layout.alignment: Qt.AlignCenter
            Layout.preferredWidth: parent.width - panesMargin
            Material.elevation: myElevation  * 0

            ColumnLayout {
                id: statusdatalayout
                width: parent.width
                Layout.preferredHeight: ulisseStateLabel.height + ulissePosLabel.height
                spacing: 0

                Label {
                    Layout.alignment: Qt.AlignHCenter
                    font.pointSize: 11
                    font.weight: Font.DemiBold
                    color: 'gray'
                    text: "Status"
                }

                LabelledText {
                    id: ulisseStateLabel
                    labelColor: Material.color(mainColor, Material.Shade700)
                    label: "Ulisse State"
                    textColor: 'lightgray'
                    text: "%1".arg(fbkUpdater.vehicle_state)
                    textBoldness: Font.DemiBold
                    onTextChanged: {
                        if(ulisse_state_changed){
                            ulisseStateLabel.textColor = 'darkslategray';
                        }
                        ulisse_state_changed = true;
                    }
                }

                LabelledText {
                    id: ulissePosLabel
                    labelColor: Material.color(mainColor, Material.Shade700)
                    label: "Ulisse Coordinates"
                    textColor: 'darkslategray'
                    text: "%1, %2".arg(fbkUpdater.ulisse_pos.latitude).arg(fbkUpdater.ulisse_pos.longitude)
                }

            }
        }

        Pane {
            id: goaldatarect
            Layout.alignment: Qt.AlignCenter
            Layout.preferredWidth: parent.width - panesMargin
            Material.elevation: myElevation  * 0

            ColumnLayout {
                id: goaldatalayout
                width: parent.width
                Layout.preferredHeight: goalTextLabel.height
                spacing: 0

                Label {
                    Layout.alignment: Qt.AlignHCenter
                    font.pointSize: 11
                    font.weight: Font.DemiBold
                    color: 'gray'
                    text: "Goal"
                }

                LabelledText {
                    id: goalTextLabel
                    labelColor: Material.color(mainColor, Material.Shade700)
                    label: "Goal Coordinates"
                    textColor: 'darkslategray'
                    text: "%1, %2".arg(fbkUpdater.goal_pos.latitude).arg(fbkUpdater.goal_pos.longitude)
                }

                LabelledText {
                    id: goalDistLabel
                    labelColor: Material.color(mainColor, Material.Shade700)
                    label: "Distance to Target"
                    textColor: 'darkslategray'
                    text: "%1 (m)".arg(fbkUpdater.goal_distance)

                }
            }
        }

        Pane {
            id: infodatarect
            Layout.alignment: Qt.AlignCenter
            Layout.preferredWidth: parent.width - panesMargin
            //Layout.preferredHeight: infodatalayout.height
            Material.elevation: myElevation * 0

            ColumnLayout {
                id: infodatalayout
                width: parent.width
                Layout.preferredHeight: markerTextLabel.height + goalTextLabel.height

                LabelledText {
                    id: markerTextLabel
                    labelColor: Material.color(Material.Red, Material.Shade800)
                    label: "Marker Coordinates"
                    textColor: 'lightgray'
                    text: "Right click on map"
                }
            }
        }

        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.minimumHeight: 14
            color: 'transparent'
            Text {
                //anchors.centerIn: parent
                width: parent.width
                font.pointSize: 8
                color: 'darkslategray'
                text: "(Right click to set marker)"
                horizontalAlignment: Text.AlignHCenter
            }
        }


        CommandPane {
            id: commandRect
            Layout.alignment: Qt.AlignCenter
            Layout.preferredWidth: parent.width - panesMargin
            Layout.bottomMargin: 10
            Material.elevation: myElevation
            //Material.background: Material.color(Material.BlueGrey, Material.Shade50)
        }


    }
}
