import QtQuick 2.6
import QtQuick.Layouts 1.3
import QtQuick.Controls 2.1
import QtLocation 5.6
import QtPositioning 5.6
import QtQuick.Controls.Material 2.1
import "."

Rectangle {
    property real myElevation: 6
    property real panesMargin: 14
    property real panesWidth: 252
    property bool responsive: false
    color: Material.background

    GridLayout {
        id: gridView
        width: parent.width
        columnSpacing: 0
        rowSpacing: 1
        rows:2
        columns:2

        Pane {
            id: statusPane
            Material.elevation: myElevation
            Layout.rowSpan: 1
            Layout.columnSpan: 1
            Layout.fillWidth: true
            Layout.preferredWidth: gridView.width/2

            ColumnLayout {
                id: statusata
                width: parent.width

                Label {
                    Layout.alignment: Qt.AlignHCenter
                    font.pointSize: 12
                    font.weight: Font.DemiBold
                    color: 'gray'
                    text: "Status"
                }

                LabelledText {
                    labelColor: 'dodgerblue'
                    label: "Vehicle State"
                    textColor: 'gray'
                    text: fbkUpdater.vehicle_state
                }

                LabelledText {
                    labelColor: 'dodgerblue'
                    label: "GPS time"
                    textColor: 'gray'
                    text: fbkUpdater.gps_time
                }

                LabelledText {
                    labelColor: 'dodgerblue'
                    label: "GPS Pos"
                    textColor: 'gray'
                    text: "%1, %2".arg(fbkUpdater.gps_pos.latitude).arg(fbkUpdater.gps_pos.longitude)
                }

                LabelledText {
                    labelColor: 'dodgerblue'
                    label: "Filtered Pos"
                    textColor: 'gray'
                    text: "%1, %2".arg(fbkUpdater.ulisse_pos.latitude).arg(fbkUpdater.ulisse_pos.longitude)
                }

                LabelledText {
                    labelColor: 'dodgerblue'
                    label: "Surge"
                    textColor: 'gray'
                    text: "%1 m/s".arg(fbkUpdater.ulisse_surge)
                }

                LabelledText {
                    labelColor: 'dodgerblue'
                    label: "Heading"
                    textColor: 'gray'
                    text: "%1°".arg(fbkUpdater.ulisse_yaw_deg)
                }
            }
        }

        Pane {
            id: goalPane
            Material.elevation: myElevation
            Layout.rowSpan: 1
            Layout.columnSpan: 1
            Layout.fillWidth: true
            Layout.preferredWidth: gridView.width/2

            ColumnLayout {
                id: goalData
                width: parent.width

                Label {
                    Layout.alignment: Qt.AlignHCenter
                    font.pointSize: 12
                    font.weight: Font.DemiBold
                    color: 'gray'
                    text: "Goal"
                }

                LabelledText {
                    //id: batteryPercLeft
                    labelColor: 'seagreen'
                    label: "Goal Position"
                    textColor: 'gray'
                    text: "%1, %2".arg(fbkUpdater.goal_pos.latitude).arg(fbkUpdater.goal_pos.longitude)
                }

                LabelledText {
                    //id: batteryPercRight
                    labelColor: 'seagreen'
                    label: "Goal Heading"
                    textColor: 'gray'
                    text: "%1°".arg(fbkUpdater.goal_heading)
                }

                LabelledText {
                    //id: batteryPercRight
                    labelColor: 'seagreen'
                    label: "Distance to Goal"
                    textColor: 'gray'
                    text: "%1 (m)".arg(fbkUpdater.goal_distance)
                }
            }
        }

        Pane {
            id: ctrlPane
            Material.elevation: myElevation
            Layout.rowSpan: 1
            Layout.columnSpan: 1
            Layout.fillWidth: true
            Layout.preferredWidth: gridView.width/2

            ColumnLayout {
                id: ctrlData
                width: parent.width

                Label {
                    Layout.alignment: Qt.AlignHCenter
                    font.pointSize: 12
                    font.weight: Font.DemiBold
                    color: 'gray'
                    text: "Control"
                }

                LabelledText {
                    //id: batteryPercLeft
                    labelColor: 'orange'
                    label: "Desired Surge"
                    textColor: 'gray'
                    text: "%1 m/s".arg(fbkUpdater.desired_surge)
                }

                LabelledText {
                    //id: batteryPercRight
                    labelColor: 'orange'
                    label: "Desired Jog"
                    textColor: 'gray'
                    text:  "%1 rad/s".arg(fbkUpdater.desired_jog)
                }

                LabelledText {
                    //id: batteryPercRight
                    labelColor: 'orange'
                    label: "Motor Control Ref"
                    textColor: 'gray'
                    text: "L: %1 \%\nR: %2 \%".arg(fbkUpdater.thrust_ref_left).arg(fbkUpdater.thrust_ref_right)
                }
            }
        }

        Pane {
            id: lowLevelPane
            Material.elevation: myElevation
            Layout.rowSpan: 1
            Layout.columnSpan: 1
            Layout.fillWidth: true
            Layout.preferredWidth: gridView.width/2

            ColumnLayout {
                id: lowLevelData
                width: parent.width

                Label {
                    Layout.alignment: Qt.AlignHCenter
                    font.pointSize: 12
                    font.weight: Font.DemiBold
                    color: 'gray'
                    text: "Low Level"
                }

                LabelledText {
                    id: batteryPercLeft
                    labelColor: 'tomato'
                    label: "Battery"
                    textColor: 'gray'
                    text: "L: %1 \%\nR: %2 \%".arg(fbkUpdater.battery_perc_L).arg(fbkUpdater.battery_perc_R)
                }

                LabelledText {
                    id: sw485status
                    labelColor: 'tomato'
                    label: "SW 485 Status"
                    textColor: 'gray'
                    text: "right_satellite_received: %1".arg(fbkUpdater.right_satellite_received485);
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
