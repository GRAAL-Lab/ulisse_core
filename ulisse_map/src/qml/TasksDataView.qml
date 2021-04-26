import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import "."

Rectangle {

    property var titlesize: 15
    property var labelsize: 13
    color: Material.background

    ColumnLayout {
        id: taskColumnView

        anchors.fill: parent

        Pane {
            id: task1Pane
            Layout.fillWidth: true

            ColumnLayout {
                id: task1Data
                width: parent.width
                Layout.fillHeight: true
                Label {
                    Layout.fillHeight: true
                    Layout.alignment: Qt.AlignHCenter
                    font.pointSize: 18
                    font.weight: Font.DemiBold
                    color: 'dodgerblue'
                    text: "Task 1"
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: 'dodgerblue'
                    label: "gain"
                    textColor: 'grey'
                    text: " "//.arg(fbkUpdater.ulisse_yaw_deg)
                    lsize: labelsize
                    tsize: titlesize

                }

                LabelledText {
                    Layout.fillHeight: true
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: 'dodgerblue'
                    textColor: 'grey'
                    text: " "//fbkUpdater.vehicle_state
                    label: qsTr("x")

                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: 'dodgerblue'
                    label: "x_dot"
                    textColor: 'grey'
                    text: " "//fbkUpdater.gps_time

                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: 'dodgerblue'
                    label: "x_dot_bar"
                    textColor: 'grey'
                    text: " "//"%1, %2".arg(fbkUpdater.gps_pos.latitude).arg(fbkUpdater.gps_pos.longitude)

                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: 'dodgerblue'
                    label: "A"
                    textColor: 'grey'
                    text: " "//

                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: 'dodgerblue'
                    label: "A_ext"
                    textColor: 'grey'
                    text: " "//"%1 m/s".arg(fbkUpdater.ulisse_surge)

                }
            }
        }
    }
}
