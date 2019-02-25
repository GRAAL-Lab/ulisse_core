import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtQuick.Controls.Material 2.1
import "."

Rectangle {

    property alias markerText: markerTextLabel.text
    property alias markerTextColor: markerTextLabel.textColor
    property alias waypointRadius: commandRect.wpRad
    color: Material.background

    ColumnLayout {
        id: leftbarlayout
        anchors.fill: parent
        spacing: 6
        Layout.leftMargin: 15

        Pane {
            id: statusdatarect
            Layout.alignment: Qt.AlignCenter
            Layout.preferredWidth: parent.width - panesMargin
            Material.elevation: myElevation

            Column{

                width: parent.width
                height: statusdatalayout.height + goaldatalayout.height + markerlayout.height
                spacing: 20

                ColumnLayout {
                    id: statusdatalayout
                    width: parent.width
                    Layout.preferredHeight: ulisseStateLabel.height + ulissePosLabel.height
                    spacing: 2

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
                        text: "%1".arg(fbkUpdater.vehicle_state)
                        textBoldness: Font.DemiBold
                    }

                    LabelledText {
                        id: ulissePosLabel
                        labelColor: Material.color(mainColor, Material.Shade700)
                        label: "Ulisse Coordinates"
                        text: "%1, %2".arg(fbkUpdater.ulisse_pos.latitude).arg(fbkUpdater.ulisse_pos.longitude)
                    }
                }

                ColumnLayout {
                    id: goaldatalayout
                    width: parent.width
                    Layout.preferredHeight: goalTextLabel.height + goalDistLabel.height
                    spacing: 2

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
                        text: "%1, %2".arg(fbkUpdater.goal_pos.latitude).arg(fbkUpdater.goal_pos.longitude)
                    }

                    LabelledText {
                        id: goalDistLabel
                        objectName: "goalDistance"
                        labelColor: Material.color(mainColor, Material.Shade700)
                        label: "Distance to Target"
                        text: "%1 (m)".arg(fbkUpdater.goal_distance)

                    }
                }

                ColumnLayout {
                    id: markerlayout
                    width: parent.width
                    Layout.preferredHeight: markerTextLabel.height

                    LabelledText {
                        id: markerTextLabel
                        labelColor: Material.color(Material.Red, Material.Shade800)
                        label: "Marker Coordinates"
                        text: "Left click on map"
                    }
                }
            }
        }

        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.minimumHeight: 14
            color: 'transparent'
            Text {
                width: parent.width
                font.pointSize: 8
                color: 'gray'
                text: "(Left click to set marker)"
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
