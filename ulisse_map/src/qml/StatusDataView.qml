import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import "."

Rectangle {
    property var labelsize: 11
    property var textsize: 11
    property real panesMargin: 14
    property real panesWidth: 252
    property bool responsive: false

    color: Material.background

    GridLayout {
        id: gridView

        anchors.fill: parent
        columnSpacing: 0
        rowSpacing: 1
        rows: 2
        columns: 1

        Pane {
            id: statusPane

            Layout.rowSpan: 1
            Layout.columnSpan: 1
            Layout.fillWidth: true
            Layout.preferredWidth: gridView.width / 2

            ColumnLayout {
                id: statusata
                width: parent.width
                Layout.fillHeight: true
                Label {
                    Layout.fillHeight: true
                    Layout.alignment: Qt.AlignHCenter
                    font.pointSize: 18
                    font.weight: Font.DemiBold
                    color: 'dodgerblue'
                    text: "Status"
                }

                LabelledText {
                    Layout.fillHeight: true
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: 'dodgerblue'
                    textColor: 'grey'
                    text: fbkUpdater.vehicle_state
                    label: qsTr("Vehicle State")
                    lsize: labelsize
                    tsize: textsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: 'dodgerblue'
                    label: "GPS time"
                    textColor: 'grey'
                    text: fbkUpdater.gps_time
                    lsize: labelsize
                    tsize: textsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: 'dodgerblue'
                    label: "GPS Pos"
                    textColor: 'grey'
                    text: "%1, %2".arg(fbkUpdater.gps_pos.latitude).arg(
                              fbkUpdater.gps_pos.longitude)
                    lsize: labelsize
                    tsize: textsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: 'dodgerblue'
                    label: "Filtered Pos"
                    textColor: 'grey'
                    text: "%1, %2".arg(fbkUpdater.ulisse_pos.latitude).arg(
                              fbkUpdater.ulisse_pos.longitude)
                    lsize: labelsize
                    tsize: textsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: 'dodgerblue'
                    label: "Surge"
                    textColor: 'grey'
                    text: "%1 m/s".arg(fbkUpdater.ulisse_surge)
                    lsize: labelsize
                    tsize: textsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: 'dodgerblue'
                    label: "Heading"
                    textColor: 'grey'
                    text: "%1°".arg(fbkUpdater.ulisse_yaw_deg)
                    lsize: labelsize
                    tsize: textsize
                }
            }
        }

        Pane {
            id: lowLevelPane

            Layout.rowSpan: 1
            Layout.columnSpan: 1
            Layout.fillWidth: true
            Layout.preferredWidth: gridView.width / 2

            ColumnLayout {
                id: lowLevelData
                width: parent.width
                Layout.alignment: Qt.AlignHCenter
                Label {
                    Layout.alignment: Qt.AlignHCenter
                    font.pointSize: 18
                    font.weight: Font.DemiBold
                    color: 'tomato'
                    text: "Low Level"
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: 'tomato'
                    label: "Micro Loop Count"
                    textColor: 'grey'
                    text: "TBD"//fbkUpdater.micro_loop_count
                    lsize: labelsize
                    tsize: textsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: 'tomato'
                    label: "Battery"
                    textColor: 'grey'
                    text: "Left: %1 \% - Right: %2 \%"
                              .arg(fbkUpdater.battery_perc_L).arg(fbkUpdater.battery_perc_R)
                    lsize: labelsize
                    tsize: textsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: 'tomato'
                    label: "SW 485 Status"
                    textColor: 'grey'
                    text: "timestamp: "//.arg(fbkUpdater.right_satellite_received485)
                    lsize: labelsize
                    tsize: textsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: 'tomato'
                    label: "Ambient"
                    textColor: 'grey'
                    text: "Temperature: °C \nHumidity: \%"//.arg(fbkUpdater.right_satellite_received485)
                    lsize: labelsize
                    tsize: textsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: 'tomato'
                    label: "Compass"
                    textColor: 'grey'
                    text: "RPY: "//.arg(fbkUpdater.right_satellite_received485)
                    lsize: labelsize
                    tsize: textsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: 'tomato'
                    label: "IMU"
                    textColor: 'grey'
                    text: "Accelerometer: \nGyroscope:"//.arg(fbkUpdater.right_satellite_received485)
                    lsize: labelsize
                    tsize: textsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: 'tomato'
                    label: "Magnetometer"
                    textColor: 'grey'
                    text: "strenght"//.arg(fbkUpdater.right_satellite_received485)
                    lsize: labelsize
                    tsize: textsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: 'tomato'
                    label: "Motors Speed"
                    textColor: 'grey'
                    text: "Left: , Right:" //to be divided by 6
                    lsize: labelsize
                    tsize: textsize
                }
            }
        }
    }
}
