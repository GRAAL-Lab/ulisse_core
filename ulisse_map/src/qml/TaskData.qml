import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import "."

Item {
    Layout.preferredHeight: taskDataColumn.implicitHeight
    Layout.fillWidth: true
    property var labelsize: 12
    property var titlesize: 10

    property var taskName: "Undefined"

    ColumnLayout {
        id: taskDataColumn
        width: parent.width
        Layout.fillHeight: true

        Label {
            Layout.fillHeight: true
            //Layout.alignment: Qt.AlignHCenter
            font.pointSize: 14
            font.weight: Font.DemiBold
            color: Material.color(Material.Blue, Material.Shade700)
            text: taskName
        }

        LabelledText {
            Layout.alignment: Qt.AlignHCenter
            labelColor: 'dodgerblue'
            textColor: 'grey'
            label: qsTr("Reference")
            text: " "//.arg(fbkUpdater.ulisse_yaw_deg)

        }

        LabelledText {
            //Layout.fillHeight: true
            Layout.alignment: Qt.AlignHCenter
            labelColor: 'dodgerblue'
            textColor: 'grey'
            label: qsTr("Internal Act.")
            text: " "//fbkUpdater.vehicle_state
        }

        LabelledText {
            Layout.alignment: Qt.AlignHCenter
            labelColor: 'dodgerblue'

            textColor: 'grey'
            label: qsTr("External Act.")
            text: " "//fbkUpdater.gps_time

        }
    }
}
