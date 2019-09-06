import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import "."

Rectangle {
    property var _tsize: 15
    property var _lsize: 13
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
        columns: 2

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
                    lsize: _lsize
                    tsize: _tsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: 'dodgerblue'
                    label: "GPS time"
                    textColor: 'grey'
                    text: fbkUpdater.gps_time
                    lsize: _lsize
                    tsize: _tsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: 'dodgerblue'
                    label: "GPS Pos"
                    textColor: 'grey'
                    text: "%1, %2".arg(fbkUpdater.gps_pos.latitude).arg(
                              fbkUpdater.gps_pos.longitude)
                    lsize: _lsize
                    tsize: _tsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: 'dodgerblue'
                    label: "Filtered Pos"
                    textColor: 'grey'
                    text: "%1, %2".arg(fbkUpdater.ulisse_pos.latitude).arg(
                              fbkUpdater.ulisse_pos.longitude)
                    lsize: _lsize
                    tsize: _tsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: 'dodgerblue'
                    label: "Surge"
                    textColor: 'grey'
                    text: "%1 m/s".arg(fbkUpdater.ulisse_surge)
                    lsize: _lsize
                    tsize: _tsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    labelColor: 'dodgerblue'
                    label: "Heading"
                    textColor: 'grey'
                    text: "%1°".arg(fbkUpdater.ulisse_yaw_deg)
                    lsize: _lsize
                    tsize: _tsize
                }
            }
        }

        Pane {
            id: goalPane

            Layout.rowSpan: 1
            Layout.columnSpan: 1
            Layout.fillWidth: true
            Layout.preferredWidth: gridView.width / 2

            ColumnLayout {
                id: goalData
                width: parent.width

                Label {
                    Layout.alignment: Qt.AlignHCenter
                    font.pointSize: 18
                    font.weight: Font.DemiBold
                    color: 'seagreen'
                    text: "Goal"
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    //id: batteryPercLeft
                    labelColor: 'seagreen'
                    label: "Goal Position"
                    textColor: 'grey'
                    text: "%1, %2".arg(fbkUpdater.goal_pos.latitude).arg(
                              fbkUpdater.goal_pos.longitude)
                    lsize: _lsize
                    tsize: _tsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    //id: batteryPercRight
                    labelColor: 'seagreen'
                    label: "Goal Heading"
                    textColor: 'grey'
                    text: "%1°".arg(fbkUpdater.goal_heading)
                    lsize: _lsize
                    tsize: _tsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    //id: batteryPercRight
                    labelColor: 'seagreen'
                    label: "Distance to Goal"
                    textColor: 'grey'
                    text: "%1 (m)".arg(fbkUpdater.goal_distance)
                    lsize: _lsize
                    tsize: _tsize
                }
            }
        }

        Pane {
            id: ctrlPane

            Layout.rowSpan: 1
            Layout.columnSpan: 1
            Layout.fillWidth: true
            Layout.preferredWidth: gridView.width / 2
            Layout.alignment: Qt.AlignHCenter
            ColumnLayout {
                id: ctrlData
                width: parent.width

                Label {
                    Layout.alignment: Qt.AlignHCenter
                    font.pointSize: 18
                    font.weight: Font.DemiBold
                    color: 'orange'
                    text: "Control"
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    //id: batteryPercLeft
                    labelColor: 'orange'
                    label: "Desired Surge"
                    textColor: 'grey'
                    text: "%1 m/s".arg(fbkUpdater.desired_surge)
                    lsize: _lsize
                    tsize: _tsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    //id: batteryPercRight
                    labelColor: 'orange'
                    label: "Desired Jog"
                    textColor: 'grey'
                    text: "%1 rad/s".arg(fbkUpdater.desired_jog)
                    lsize: _lsize
                    tsize: _tsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    //id: batteryPercRight
                    labelColor: 'orange'
                    label: "Motor Control Ref"
                    textColor: 'grey'
                    text: "L: %1 \%\nR: %2 \%".arg(
                              fbkUpdater.thrust_ref_left).arg(
                              fbkUpdater.thrust_ref_right)
                    lsize: _lsize
                    tsize: _tsize
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
                    id: batteryPercLeft
                    labelColor: 'tomato'
                    label: "Battery"
                    textColor: 'grey'
                    text: "L: %1 \%\nR: %2 \%".arg(
                              fbkUpdater.battery_perc_L).arg(
                              fbkUpdater.battery_perc_R)
                    lsize: _lsize
                    tsize: _tsize
                }

                LabelledText {
                    Layout.alignment: Qt.AlignHCenter
                    id: sw485status
                    labelColor: 'tomato'
                    label: "SW 485 Status"
                    textColor: 'grey'
                    text: "right_satellite_received: %1".arg(
                              fbkUpdater.right_satellite_received485)
                    lsize: _lsize
                    tsize: _tsize
                    /* "missed_deadlines: %1
left_motor_received: %2
left_motor_sent: %3
right_motor_received: %4
right_motor_sent: %5
left_satellite_received: %6
left_satellite_sent: %7
right_satellite_received: %8
right_satellite_sent: %9".arg(fbkUpdater.missed_deadlines485).arg(fbkUpdater.left_motor_received485)
.arg(fbkUpdater.left_motor_sent485).arg(fbkUpdater.right_motor_received485).arg(fbkUpdater.right_motor_sent485)
.arg(fbkUpdater.left_satellite_received485).arg(fbkUpdater.left_satellite_sent485)
.arg(fbkUpdater.right_satellite_received485).arg(fbkUpdater.right_satellite_sent485); */
                }
            }
        }
        /*
        Goal Pos:	0, 0
        Goal Distance:	0
        Goal Heading:	0
        Desired Speed:	0
        Desired Jog:	0
        Motor Map Out:	0, 0
        Motor Ctrl Ref:	0, 0
        */
    }
}
