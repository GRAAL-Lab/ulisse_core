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
    border.width: 1
    visible: true

    ColumnLayout {
        id: leftbarlayout
        anchors.fill: parent
        Layout.leftMargin: 15
        width: parent.width
        spacing: 0

        Pane {
            id: statusdatarect
            Layout.fillHeight: true
            Layout.minimumWidth: leftbarlayout.width
            Layout.preferredWidth: parent.width
            Layout.minimumHeight: mycol.height
            Layout.alignment: Qt.AlignLeft

            ColumnLayout {
                id: mycol
                width: statusdatarect.width - 30
                Layout.alignment: Qt.AlignLeft

                ColumnLayout {
                    id: statusdatalayout
                    width: mycol.width
                    Layout.minimumWidth: width
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Layout.alignment: Qt.AlignLeft
                    Layout.bottomMargin: 5
                    height: my_label.Top - goalDistLabel.Bottom
                    clip: false
                    visible: true

                    Label {
                        id: my_label
                        Layout.alignment: Qt.AlignHCenter
                        font.pointSize: 12
                        font.weight: Font.DemiBold
                        color: labelColor3
                        text: "Status"
                    }

                    LabelledText {
                        id: ulisseStateLabel
                        Layout.minimumWidth: statusdatalayout.width
                        labelColor: labelColor1
                        label: "Ulisse State"
                        text: fbkUpdater.vehicle_state
                        textBoldness: Font.DemiBold
                    }

                    LabelledText {
                        id: ulissePosLabel
                        Layout.minimumWidth: statusdatalayout.width
                        labelColor: labelColor1
                        label: "Ulisse Coordinates"
                        text: fbkUpdater.ulisse_pos.latitude + ", "
                              + fbkUpdater.ulisse_pos.longitude
                    }

                    LabelledText {
                        id: goalDistLabel
                        Layout.minimumWidth: statusdatalayout.width
                        objectName: "goalDistance"
                        labelColor: labelColor1
                        label: "Distance to Target"
                        text: fbkUpdater.goal_distance + " (m)"
                    }
                }

                /*ColumnLayout {
                    id: goaldatalayout
                    width: parent.width
                    Layout.preferredHeight: goalTextLabel.height + goalDistLabel.height
                    spacing: 2

                    Label {
                        Layout.alignment: Qt.AlignHCenter
                        font.pointSize: 11
                        font.weight: Font.DemiBold
                        color: 'grey'
                        text: "Goal"
                    }

                    LabelledText {
                        id: goalTextLabel
                        labelColor: Material.color(mainColor, Material.Shade700)
                        label: "Goal Coordinates"
                        text: "%1, %2".arg(fbkUpdater.goal_pos.latitude).arg(fbkUpdater.goal_pos.longitude)
                    }


                }*/
                ColumnLayout {
                    id: markerlayout
                    Layout.minimumWidth: mycol.width
                    Layout.minimumHeight: markerTextLabel.Top - markerText.Bottom

                    LabelledText {
                        id: markerTextLabel
                        Layout.minimumWidth: markerlayout.width
                        labelColor: labelColor2
                        label: "Marker Coordinates"
                        text: "Left click on map"
                    }
                    Text {
                        id: markerText
                        Layout.alignment: Qt.AlignHCenter
                        width: markerlayout.width
                        font.pointSize: 8
                        color: 'grey'
                        text: "(Left click to set marker)"
                        horizontalAlignment: Text.AlignHCenter
                    }
                }
            }
        }

        CommandPane {
            id: commandRect
            antialiasing: true
            Layout.alignment: Qt.AlignLeft | Qt.AlignVCenter
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.bottomMargin: 2
            //Material.background: Material.color(Material.BlueGrey, Material.Shade50)
        }
    }
}
