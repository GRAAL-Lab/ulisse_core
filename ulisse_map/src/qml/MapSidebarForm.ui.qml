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
            Layout.fillHeight: true
            Layout.minimumWidth: leftbarlayout.width
            Layout.preferredWidth: parent.width
            Layout.preferredHeight: parent.height/6
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
                    Layout.alignment: Qt.AlignLeft

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
                        labelColor: labelColor1
                        label: "Ulisse State"
                        text: fbkUpdater.vehicle_state
                        Layout.fillWidth: true
                        textBoldness: Font.DemiBold
                    }

                    LabelledText {
                        id: ulissePosLabel
                        labelColor: labelColor1
                        label: "Ulisse Coordinates"
                        text: fbkUpdater.ulisse_pos.latitude + ", "
                              + fbkUpdater.ulisse_pos.longitude
                        Layout.fillWidth: true
                    }

                    LabelledText {
                        id: goalDistLabel
                        objectName: "goalDistance"
                        labelColor: labelColor1
                        label: "Distance to Target"
                        text: fbkUpdater.goal_distance + " (m)"
                        Layout.fillWidth: true
                    }

                    LabelledText {
                        id: markerTextLabel
                        labelColor: labelColor2
                        label: "Marker Coordinates"
                        text: "Left click on map"
                        Layout.fillWidth: true
                    }
                    Text {
                        id: markerText
                        Layout.alignment: Qt.AlignHCenter
                        font.pointSize: 8
                        color: 'grey'
                        text: "(Left click to set marker)"
                        horizontalAlignment: Text.AlignHCenter
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
